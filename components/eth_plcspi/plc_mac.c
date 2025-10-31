#include <string.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_eth_mac.h"
#include "esp_eth.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "plcspi.h"
#include "esp_heap_caps.h"
#include <errno.h>

static const char *TAG = "plc-mac";

/* ===== 可开关的定位宏（默认 0，不改变现有行为） ===== */
#ifndef PLC_RX_TASK_CREATE_IN_INIT
#define PLC_RX_TASK_CREATE_IN_INIT  0   /* 1=在 init() 里就创建 RX 任务 */
#endif

#ifndef PLC_FORCE_STARTED_IN_INIT
#define PLC_FORCE_STARTED_IN_INIT   0   /* 1=在 init() 里强制 started=true（仅用于定位） */
#endif

/* 让步时间（毫秒），用于避免“INT 高但无帧”的紧密循环鞭打 CPU */
#ifndef PLC_RX_YIELD_ON_EMPTY_CTR_MS
#define PLC_RX_YIELD_ON_EMPTY_CTR_MS 10
#endif

/* 每轮外层循环结束的小让步，阻断极端情况下的零延迟重入 */
#ifndef PLC_RX_YIELD_AFTER_LOOP_MS
#define PLC_RX_YIELD_AFTER_LOOP_MS 10
#endif

/* ===== SLAC 帧接收队列配置 ===== */
#ifndef PLC_RX_MAX
#define PLC_RX_MAX    1600
#endif
#ifndef PLC_RX_POOL_N
#define PLC_RX_POOL_N 8          /* 可按带宽调大 */
#endif

typedef struct {
    uint8_t *buf;
    uint16_t len;
} plc_rx_frame_t;

/* 简易 buffer 池（固定块、零拷贝交接） */
static uint8_t s_pool[PLC_RX_POOL_N][PLC_RX_MAX];
static uint8_t s_pool_used[PLC_RX_POOL_N];
static portMUX_TYPE s_pool_mux = portMUX_INITIALIZER_UNLOCKED;

static uint8_t* pool_get(void){
    taskENTER_CRITICAL(&s_pool_mux);
    for (int i=0;i<PLC_RX_POOL_N;i++){
        if (!s_pool_used[i]) { s_pool_used[i]=1; taskEXIT_CRITICAL(&s_pool_mux); return s_pool[i]; }
    }
    taskEXIT_CRITICAL(&s_pool_mux);
    return NULL;
}
static void pool_put(uint8_t *p){
    taskENTER_CRITICAL(&s_pool_mux);
    for (int i=0;i<PLC_RX_POOL_N;i++){
        if (s_pool[i]==p){ s_pool_used[i]=0; break; }
    }
    taskEXIT_CRITICAL(&s_pool_mux);
}

/* ============ hexdump（调试用） ============ */
static inline void hexdump_simple(const uint8_t *p, size_t len){
    for(size_t i=0;i<len;i++){
        printf("%02X%s", p[i], ((i+1)%16)?" ":"\n");
    }
    if (len%16) puts("");
}


typedef struct {
    esp_eth_mac_t      parent;      /* vtbl 放首位 */
    esp_eth_mediator_t *mediator;
    void               *ll;         /* 低层句柄 */
    bool                started;
    TaskHandle_t        rx_task;
    uint8_t             mac_addr[6];

    /* 新增：SLAC 收包路径 */
    QueueHandle_t       rxq;        /* 仅缓存 0x88E1 帧给上层 readpacket */
    bool                slac_mode;  /* true=SLAC阶段：0x88E1 入队，其它不上送 lwIP */
} plc_mac_t;

#ifndef CONTAINER_OF
#define CONTAINER_OF(ptr, type, member) ((type *)((char *)(ptr) - offsetof(type, member)))
#endif
#define MAC_FROM_PARENT(p) CONTAINER_OF((p), plc_mac_t, parent)

/* ======== 对外 API：切换 SLAC 模式 / 读一帧（平替 Linux readpacket） ======== */
/* 说明：
 *  1) SLAC 模式开启后，RX 任务会把 EtherType==0x88E1 的帧放入 rxq；其它帧丢弃（不进入 lwIP）。
 *  2) SLAC 成功后，调用 plc_set_slac_mode(false) 关闭，恢复全部帧走 lwIP。
 */
esp_err_t plc_set_slac_mode(esp_eth_mac_t *mac, bool enable){
    if (!mac) return ESP_ERR_INVALID_ARG;
    plc_mac_t *m = MAC_FROM_PARENT(mac);
    m->slac_mode = enable;
    ESP_LOGI(TAG, "SLAC mode %s", enable?"ON":"OFF");
    return ESP_OK;
}

/* 返回：>0=实际帧长；0=超时无数据；<0=错误（置 errno）
 * 注意：会把数据拷贝到 caller 的 memory，长度不超过 extent；若被截断，返回值仍为原始帧长。 */
ssize_t plc_readpacket(esp_eth_mac_t *mac, void *memory, ssize_t extent, int timeout_ms, bool verbose){
    if (!mac || !memory || extent<=0){ errno=EINVAL; return -1; }
    plc_mac_t *m = MAC_FROM_PARENT(mac);
    if (!m->rxq){ errno=ENODEV; return -1; }

    plc_rx_frame_t frm;
    memset(memory, 0, (size_t)extent);
    if (xQueueReceive(m->rxq, &frm, pdMS_TO_TICKS(timeout_ms)) != pdTRUE){
        return 0; /* 超时 */
    }

    ssize_t ret_len = frm.len;          /* 原始长度 */
    ssize_t copy_n  = (ret_len>extent)?extent:ret_len;
    memcpy(memory, frm.buf, (size_t)copy_n);
    if (verbose) hexdump_simple((const uint8_t*)memory, (size_t)copy_n);
    pool_put(frm.buf);
    return ret_len;
}

/* ================= RX 任务 ================= */
static void plc_rx_task(void *arg)
{
    plc_mac_t *m = (plc_mac_t *)arg;
    ESP_LOGI(TAG, "RX task started (core=%d, prio=%d, stack~%u)",
             xPortGetCoreID(), uxTaskPriorityGet(NULL), (unsigned)uxTaskGetStackHighWaterMark(NULL));

    static uint8_t frame[1600 + DET_SOF_LEN + DET_DFT_LEN];

    for (;;) {
        if (!m->started || !m->rxq) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }

        /* 等待 INT 高电平（底层纯轮询） */
        if (plc_ll_spi_wait_irq(m->ll, 1000) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        for (;;) {
            /* 1) DET | CTR */
            uint8_t tx[DET_CMD_LEN] = {
                (uint8_t)((DET_CMD >> 8) & 0xFF),
                (uint8_t)(DET_CMD & 0xFF),
                (uint8_t)((CMD_CTR >> 8) & 0xFF),
                (uint8_t)(CMD_CTR & 0xFF),
            };
            if (plc_ll_spi_tx(m->ll, tx, sizeof(tx)) != ESP_OK) { vTaskDelay(pdMS_TO_TICKS(5)); break; }

            uint8_t rsp[DET_CMD_LEN] = {0};
            if (plc_ll_spi_rx(m->ll, rsp, sizeof(rsp)) != ESP_OK) { vTaskDelay(pdMS_TO_TICKS(5)); break; }

            /* rsp[1] == 0x01 => 有帧；长度 12bit 在 rsp[2:3] */
            uint16_t len = (uint16_t)(((rsp[2] << 8) | rsp[3]) & 0x0FFF);

            if (!(rsp[1] == 0x01 && len >= 60 && len <= 1518)) {
                /* 无帧/非法长度：小憩后退出内层，回到 wait_irq */
                vTaskDelay(pdMS_TO_TICKS(PLC_RX_YIELD_ON_EMPTY_CTR_MS));
                break;
            }

            size_t total = (size_t)len + DET_SOF_LEN + DET_DFT_LEN;
            if (total > sizeof(frame)) {
                ESP_LOGW(TAG, "too long: %u", (unsigned)len);
                vTaskDelay(pdMS_TO_TICKS(5));
                break;
            }

            /* 2) 一次 DMA 收：SOF(2) + payload(len) + DFT(2) */
            if (plc_ll_spi_rx(m->ll, frame, total) != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(5));
                break;
            }

            /* 3) 校验标记 */
            uint16_t sof = (uint16_t)((frame[0] << 8) | frame[1]);
            uint16_t dft = (uint16_t)((frame[total - 2] << 8) | frame[total - 1]);
            if (sof != DET_SOF || dft != DET_DFT) {
                ESP_LOGW(TAG, "marker bad sof=0x%04x dft=0x%04x len=%u", sof, dft, (unsigned)len);
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }

            /* payload 指向以太帧 */
            uint8_t *eth = &frame[DET_SOF_LEN];

            // 调试打印头部
            if (len >= 14) {
                uint8_t *d = eth, *s = eth + 6;
                uint16_t et = ((uint16_t)eth[12] << 8) | eth[13];
                ESP_LOGI(TAG, "RX <- plc  dst=%02x:%02x:%02x:%02x:%02x:%02x src=%02x:%02x:%02x:%02x:%02x:%02x type=0x%04x len=%u",
                        d[0],d[1],d[2],d[3],d[4],d[5],
                        s[0],s[1],s[2],s[3],s[4],s[5],
                        et, (unsigned)len);
            }

            /* 4) 分流：
             *  - SLAC 模式：0x88E1 -> 入队；其他帧丢弃。
             *  - 正常模式：全部交 lwIP（mediator->stack_input 接管后续内存）。
             */
            uint16_t ether_type = ((uint16_t)eth[12] << 8) | eth[13];
            if (m->slac_mode) {
                if (ether_type == 0x88E1) {
                    uint8_t *buf = pool_get();
                    if (!buf) {
                        ESP_LOGW(TAG, "rx pool full, drop 0x88E1 len=%u", (unsigned)len);
                        continue;
                    }
                    memcpy(buf, eth, len);
                    plc_rx_frame_t frm = { .buf = buf, .len = (uint16_t)len };
                    if (xQueueSend(m->rxq, &frm, 0) != pdTRUE) {
                        pool_put(buf);
                        ESP_LOGW(TAG, "rx queue full, drop 0x88E1 len=%u", (unsigned)len);
                    }
                } else {
                    // SLAC 阶段：忽略非 0x88E1 帧
                }
            } else {
                // 非 SLAC：直接上送 lwIP（成功则所有权转移；失败我们 free）
                uint8_t *payload = (uint8_t *)heap_caps_malloc(len, MALLOC_CAP_8BIT);
                if (!payload) { ESP_LOGE(TAG, "rx oom len=%u", (unsigned)len); continue; }
                memcpy(payload, eth, len);
                esp_err_t se = ESP_FAIL;
                if (m->mediator) se = m->mediator->stack_input(m->mediator, payload, len);
                if (se != ESP_OK) {
                    free(payload);
                    ESP_LOGW(TAG, "stack_input=%d len=%u (freed)", se, (unsigned)len);
                }
            }
        }

        plc_ll_spi_reenable_irq(m->ll); /* 轮询模式 no-op，保留调用不改上层逻辑 */

        /* 变动点 2：每轮外层循环结束后补 1ms 让步，阻断高电平快重入 */
        vTaskDelay(pdMS_TO_TICKS(PLC_RX_YIELD_AFTER_LOOP_MS));
    }
}

/* ================ vtbl & 基础实现 ================= */
static esp_err_t plc_set_mediator(esp_eth_mac_t *mac, esp_eth_mediator_t *m)
{ MAC_FROM_PARENT(mac)->mediator = m; return ESP_OK; }

static esp_err_t plc_init(esp_eth_mac_t *mac)
{
    plc_mac_t *m = MAC_FROM_PARENT(mac);
    ESP_LOGI(TAG, "mac init");

#if PLC_RX_TASK_CREATE_IN_INIT
    if (!m->rx_task) {
        /* 钉到 CPU1 */
        BaseType_t ok = xTaskCreatePinnedToCore(plc_rx_task, "plc_rx", 4096, m, 1, &m->rx_task, 1);
        if (ok != pdPASS || m->rx_task == NULL) {
            ESP_LOGE(TAG, "xTaskCreate in init FAILED");
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGI(TAG, "rx task created in init: %p", (void*)m->rx_task);
    }
#endif

    /* 确保 rx 队列在任务运行前已创建（避免 xQueueSend 断言） */
    if (!m->rxq) {
        m->rxq = xQueueCreate(PLC_RX_POOL_N, sizeof(plc_rx_frame_t));
        if (!m->rxq) {
            ESP_LOGE(TAG, "rx queue create failed in init");
            return ESP_ERR_NO_MEM;
        }
    }

#if PLC_FORCE_STARTED_IN_INIT
    vTaskDelay(pdMS_TO_TICKS(1000));
    m->started = true;
    ESP_LOGW(TAG, "FORCE started=true in init (for debugging)");
#endif

    return ESP_OK;
}

static esp_err_t plc_deinit(esp_eth_mac_t *mac)
{
    (void)mac;
    ESP_LOGI(TAG, "mac deinit");
    return ESP_OK;
}

static esp_err_t plc_start(esp_eth_mac_t *mac)
{
    plc_mac_t *m = MAC_FROM_PARENT(mac);

    ESP_LOGI(TAG, "mac start() ENTER");
    size_t free8  = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t free32 = heap_caps_get_free_size(MALLOC_CAP_32BIT);
    ESP_LOGI(TAG, "heap free 8bit=%u 32bit=%u", (unsigned)free8, (unsigned)free32);

    if (!m->rx_task) {
        /* 钉到 CPU1 */
        BaseType_t ok = xTaskCreatePinnedToCore(plc_rx_task, "plc_rx", 4096, m, 1, &m->rx_task, 1);
        if (ok != pdPASS || m->rx_task == NULL) {
            ESP_LOGE(TAG, "xTaskCreate plc_rx FAILED (ret=%ld)", (long)ok);
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGI(TAG, "xTaskCreate OK, handle=%p", (void*)m->rx_task);
    } else {
        ESP_LOGW(TAG, "plc_rx already exists: handle=%p", (void*)m->rx_task);
    }

    if (!m->rxq) {
        m->rxq = xQueueCreate(PLC_RX_POOL_N, sizeof(plc_rx_frame_t));
        if (!m->rxq) {
            ESP_LOGE(TAG, "rx queue create failed");
            return ESP_ERR_NO_MEM;
        }
    }
    m->slac_mode = false; /* 默认关闭 SLAC 模式 */

    m->started = true;
    ESP_LOGI(TAG, "mac start() EXIT");
    return ESP_OK;
}

static esp_err_t plc_stop(esp_eth_mac_t *mac)
{
    plc_mac_t *m = MAC_FROM_PARENT(mac);
    m->started = false;
    ESP_LOGI(TAG, "mac stop");
    return ESP_OK;
}


static esp_err_t plc_transmit(esp_eth_mac_t *mac, uint8_t *buf, uint32_t length)
{
    plc_mac_t *m = MAC_FROM_PARENT(mac);
    if (!m->started) return ESP_ERR_INVALID_STATE;

    // 以太帧按不含 FCS 的常规长度限制
    if (length < 14 || length > 1518) {
        ESP_LOGW(TAG, "tx drop: bad len=%u", (unsigned)length);
        return ESP_ERR_INVALID_ARG;
    }

    // total_len：帧体需要 pad 到 60B（以太网最小帧，不含 FCS）
    uint16_t total_len = (length < 60) ? 60 : (uint16_t)length;

    // 组“一口气发送”的缓冲区：SOF(2) + payload(total_len) + DFT(2)
    const size_t one_shot_len = DET_SOF_LEN + total_len + DET_DFT_LEN;
    uint8_t *tx = (uint8_t *)heap_caps_malloc(one_shot_len, MALLOC_CAP_8BIT);
    if (!tx) return ESP_ERR_NO_MEM;

    // SOF
    tx[0] = (DET_SOF >> 8) & 0xFF;
    tx[1] = (DET_SOF) & 0xFF;

    // payload（14..length）原样拷贝；若小于 60B，用 0 填充尾部
    memcpy(&tx[DET_SOF_LEN], buf, length);
    if (length < 60) {
        memset(&tx[DET_SOF_LEN + length], 0, 60 - length);
    }

    // DFT
    tx[DET_SOF_LEN + total_len + 0] = (DET_DFT >> 8) & 0xFF;
    tx[DET_SOF_LEN + total_len + 1] = (DET_DFT) & 0xFF;

    // 先 RTS 握手（带上 total_len）
    esp_err_t err = plc_ll_spi_rts_wait_ctr(m->ll, total_len, /*timeout_ms=*/20);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "tx: RTS/CTR handshake failed (%s)", esp_err_to_name(err));
        free(tx);
        return err;
    }

    // 一次事务把 SOF+payload+DFT 整包发掉（DMA）
    err = plc_ll_spi_tx(m->ll, tx, one_shot_len);
    free(tx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "tx: one-shot payload failed (%s)", esp_err_to_name(err));
        return err;
    }

    // 可选：打印头部，联调更直观
    if (length >= 14) {
        uint8_t *d = buf, *s = buf + 6;
        uint16_t et = ((uint16_t)buf[12] << 8) | buf[13];
        ESP_LOGI(TAG, "TX <- netif  dst=%02x:%02x:%02x:%02x:%02x:%02x src=%02x:%02x:%02x:%02x:%02x:%02x type=0x%04x len=%u(padded=%u)",
                 d[0],d[1],d[2],d[3],d[4],d[5],
                 s[0],s[1],s[2],s[3],s[4],s[5],
                 et, (unsigned)length, (unsigned)total_len);
    }
    return ESP_OK;
}


static esp_err_t plc_transmit_vargs(esp_eth_mac_t *mac, uint32_t argc, va_list args)
{ (void)mac; (void)argc; (void)args; return ESP_ERR_NOT_SUPPORTED; }

static esp_err_t plc_receive(esp_eth_mac_t *mac, uint8_t *buf, uint32_t *length)
{ (void)mac; (void)buf; (void)length; return ESP_ERR_NOT_SUPPORTED; }

static esp_err_t plc_read_phy_reg(esp_eth_mac_t *mac, uint32_t a, uint32_t r, uint32_t *v)
{ (void)mac; (void)a; (void)r; (void)v; return ESP_ERR_NOT_SUPPORTED; }
static esp_err_t plc_write_phy_reg(esp_eth_mac_t *mac, uint32_t a, uint32_t r, uint32_t v)
{ (void)mac; (void)a; (void)r; (void)v; return ESP_ERR_NOT_SUPPORTED; }

static esp_err_t plc_set_addr(esp_eth_mac_t *mac, uint8_t *addr)
{ memcpy(MAC_FROM_PARENT(mac)->mac_addr, addr, 6); return ESP_OK; }
static esp_err_t plc_get_addr(esp_eth_mac_t *mac, uint8_t *addr)
{
    plc_mac_t *m = MAC_FROM_PARENT(mac);
    if (!(m->mac_addr[0]|m->mac_addr[1]|m->mac_addr[2]|m->mac_addr[3]|m->mac_addr[4]|m->mac_addr[5])) {
        uint8_t def[6] = {0x02,0x00,0xDE,0xAD,0xBE,0xEF}; memcpy(addr, def, 6);
    } else memcpy(addr, m->mac_addr, 6);
    return ESP_OK;
}
static esp_err_t plc_set_speed(esp_eth_mac_t *mac, eth_speed_t s) { (void)mac; (void)s; return ESP_OK; }
static esp_err_t plc_set_duplex(esp_eth_mac_t *mac, eth_duplex_t d) { (void)mac; (void)d; return ESP_OK; }
static esp_err_t plc_set_link(esp_eth_mac_t *mac, eth_link_t l) { (void)mac; (void)l; return ESP_OK; }
static esp_err_t plc_set_promiscuous(esp_eth_mac_t *mac, bool e) { (void)mac; (void)e; return ESP_OK; }
static esp_err_t plc_enable_flow_ctrl(esp_eth_mac_t *mac, bool e) { (void)mac; (void)e; return ESP_OK; }
static esp_err_t plc_set_peer_pause_ability(esp_eth_mac_t *mac, uint32_t a) { (void)mac; (void)a; return ESP_OK; }
static esp_err_t plc_custom_ioctl(esp_eth_mac_t *mac, int cmd, void *data)
{ (void)mac; (void)cmd; (void)data; return ESP_ERR_NOT_SUPPORTED; }

static esp_err_t plc_del(esp_eth_mac_t *mac)
{
    plc_mac_t *m = MAC_FROM_PARENT(mac);
    if (m->rx_task) { vTaskDelete(m->rx_task); m->rx_task = NULL; }
    if (m->rxq) { vQueueDelete(m->rxq); m->rxq=NULL; }
    free(m);
    return ESP_OK;
}

/* ===== 工厂函数 ===== */
esp_eth_mac_t *esp_eth_mac_new_plcspi(const plcspi_config_t *cfg, const eth_mac_config_t *mac_config)
{
    (void)mac_config;
    plc_mac_t *m = (plc_mac_t *)calloc(1, sizeof(plc_mac_t));
    if (!m) return NULL;

    if (plc_ll_spi_create(cfg, &m->ll) != ESP_OK) {
        free(m);
        return NULL;
    }

    m->parent.set_mediator            = plc_set_mediator;
    m->parent.init                    = plc_init;
    m->parent.deinit                  = plc_deinit;
    m->parent.start                   = plc_start;
    m->parent.stop                    = plc_stop;
    m->parent.transmit                = plc_transmit;
    m->parent.transmit_vargs          = plc_transmit_vargs;
    m->parent.receive                 = plc_receive;
    m->parent.read_phy_reg            = plc_read_phy_reg;
    m->parent.write_phy_reg           = plc_write_phy_reg;
    m->parent.set_addr                = plc_set_addr;
    m->parent.get_addr                = plc_get_addr;
    m->parent.set_speed               = plc_set_speed;
    m->parent.set_duplex              = plc_set_duplex;
    m->parent.set_link                = plc_set_link;
    m->parent.set_promiscuous         = plc_set_promiscuous;
    m->parent.enable_flow_ctrl        = plc_enable_flow_ctrl;
    m->parent.set_peer_pause_ability  = plc_set_peer_pause_ability;
    m->parent.custom_ioctl            = plc_custom_ioctl;
    m->parent.del                     = plc_del;

    ESP_LOGI(TAG, "mac vtbl ok");
    ESP_LOGI(TAG, "vtbl set: start=%p init=%p stop=%p tx=%p",
             (void*)m->parent.start, (void*)m->parent.init, (void*)m->parent.stop, (void*)m->parent.transmit);
    return &m->parent;
}
