#include <string.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_eth_mac.h"
#include "esp_eth.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "plcspi.h"
#include "esp_heap_caps.h"

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
#define PLC_RX_YIELD_ON_EMPTY_CTR_MS 40
#endif

/* 每轮外层循环结束的小让步，阻断极端情况下的零延迟重入 */
#ifndef PLC_RX_YIELD_AFTER_LOOP_MS
#define PLC_RX_YIELD_AFTER_LOOP_MS 10
#endif

typedef struct {
    esp_eth_mac_t      parent;      /* vtbl 放首位 */
    esp_eth_mediator_t *mediator;
    void               *ll;         /* 低层句柄 */
    bool                started;
    TaskHandle_t        rx_task;
    uint8_t             mac_addr[6];
} plc_mac_t;

#ifndef CONTAINER_OF
#define CONTAINER_OF(ptr, type, member) ((type *)((char *)(ptr) - offsetof(type, member)))
#endif
#define MAC_FROM_PARENT(p) CONTAINER_OF((p), plc_mac_t, parent)

/* ================= RX 任务 ================= */
static void plc_rx_task(void *arg)
{
    plc_mac_t *m = (plc_mac_t *)arg;
    ESP_LOGI(TAG, "RX task started (core=%d, prio=%d, stack~%u)",
             xPortGetCoreID(), uxTaskPriorityGet(NULL), (unsigned)uxTaskGetStackHighWaterMark(NULL));

    static uint8_t frame[1600 + DET_SOF_LEN + DET_DFT_LEN];

    for (;;) {
        if (!m->started) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }

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
                /* 变动点 1：无帧/非法长度时先小憩 5ms 再退出内层，避免鞭打 CPU */
                vTaskDelay(pdMS_TO_TICKS(PLC_RX_YIELD_ON_EMPTY_CTR_MS));
                break;  /* 退出内层，回到 wait_irq */
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

            /* ✅ 把 payload 拷贝到堆内存，再交给协议栈（栈会 free） */
            uint8_t *payload = (uint8_t *)heap_caps_malloc(len, MALLOC_CAP_8BIT);
            if (!payload) {
                ESP_LOGE(TAG, "rx oom len=%u", (unsigned)len);
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }
            memcpy(payload, &frame[DET_SOF_LEN], len);

            // === 调试打印：以太网头 ===
            if (len >= 14) {
                uint8_t *d = payload, *s = payload + 6;
                uint16_t et = ((uint16_t)payload[12] << 8) | payload[13];
                ESP_LOGI(TAG, "RX -> netif  dst=%02x:%02x:%02x:%02x:%02x:%02x src=%02x:%02x:%02x:%02x:%02x:%02x type=0x%04x len=%u",
                        d[0],d[1],d[2],d[3],d[4],d[5],
                        s[0],s[1],s[2],s[3],s[4],s[5],
                        et, (unsigned)len);
            }

            /* 4) 喂协议栈：纯 payload（成功则所有权转移；失败我们 free） */
            esp_err_t se = ESP_FAIL;
            if (m->mediator) se = m->mediator->stack_input(m->mediator, payload, len);
            if (se != ESP_OK) {
                free(payload);
                ESP_LOGW(TAG, "stack_input=%d len=%u (freed)", se, (unsigned)len);
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

#if PLC_FORCE_STARTED_IN_INIT
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
    uint8_t det[DET_CMD_LEN] = {
        (uint8_t)((DET_CMD >> 8) & 0xFF),
        (uint8_t)(DET_CMD & 0xFF),
        (uint8_t)((CMD_RTS >> 8) & 0xFF),
        (uint8_t)(CMD_RTS & 0xFF),
    };
    if (plc_ll_spi_tx(m->ll, det, sizeof(det)) != ESP_OK) return ESP_FAIL;

    uint8_t sof[DET_SOF_LEN] = { (uint8_t)(DET_SOF >> 8), (uint8_t)(DET_SOF & 0xFF) };
    if (plc_ll_spi_tx(m->ll, sof, sizeof(sof)) != ESP_OK) return ESP_FAIL;

    if (plc_ll_spi_tx(m->ll, buf, length) != ESP_OK) return ESP_FAIL;

    uint8_t dft[DET_DFT_LEN] = { (uint8_t)(DET_DFT >> 8), (uint8_t)(DET_DFT & 0xFF) };
    if (plc_ll_spi_tx(m->ll, dft, sizeof(dft)) != ESP_OK) return ESP_FAIL;

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
