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

static const char *TAG = "plc-mac";

/* 私有对象 */
typedef struct {
    esp_eth_mac_t      parent;      /* vtbl 放首位：与 IDF 头一致 */
    esp_eth_mediator_t *mediator;
    void               *ll;         /* 低层句柄（plc_ll_t*），对外不暴露类型 */
    bool                started;
    TaskHandle_t        rx_task;
    uint8_t             mac_addr[6];
} plc_mac_t;

#ifndef CONTAINER_OF
#define CONTAINER_OF(ptr, type, member) ((type *)((char *)(ptr) - offsetof(type, member)))
#endif
#define MAC_FROM_PARENT(p) CONTAINER_OF((p), plc_mac_t, parent)

/* ================= RX 任务：上升沿→CTR 长度→DMA 收帧→校验 SOF/DFT→喂栈 ================= */
static void plc_rx_task(void *arg)
{
    plc_mac_t *m = (plc_mac_t *)arg;
    ESP_LOGI(TAG, "RX task started");

    /* 留 1600 + 2 + 2 的空间（以太网最大帧 + SOF + DFT） */
    static uint8_t frame[1600 + DET_SOF_LEN + DET_DFT_LEN];

    for (;;) {
        if (!m->started) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

        /* 等待上升沿（ISR 轻量唤醒） */
        if (plc_ll_spi_wait_irq(m->ll, 1000) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        /* 读到 INT 变低为止 */
        while (plc_ll_spi_int_level(m->ll)) {
            /* 1) 发送 DET|CTR 获取长度（4 字节） */
            uint8_t cmd[DET_CMD_LEN] = {
                (uint8_t)((DET_CMD >> 8) & 0xFF), (uint8_t)(DET_CMD & 0xFF),
                (uint8_t)((CMD_CTR >> 8) & 0xFF), (uint8_t)(CMD_CTR & 0xFF)
            };
            if (plc_ll_spi_tx(m->ll, cmd, sizeof(cmd)) != ESP_OK) { vTaskDelay(1); break; }

            uint8_t rsp[DET_CMD_LEN] = {0};
            if (plc_ll_spi_rx(m->ll, rsp, sizeof(rsp)) != ESP_OK) { vTaskDelay(1); break; }

            /* rsp[1] == 0x01 => 有帧；长度 12bit 在 rsp[2:3] */
            uint16_t len = (uint16_t)(((rsp[2] << 8) | rsp[3]) & 0x0FFF);
            if (!(rsp[1] == 0x01 && len >= 60 && len <= 1518)) {
                /* 无帧或非法长度，跳出内层让出 CPU */
                break;
            }

            size_t total = (size_t)len + DET_SOF_LEN + DET_DFT_LEN;
            if (total > sizeof(frame)) { ESP_LOGW(TAG, "too long: %u", (unsigned)len); vTaskDelay(1); break; }

            /* 2) 一次 DMA 收：SOF(2) + payload(len) + DFT(2) */
            if (plc_ll_spi_rx(m->ll, frame, total) != ESP_OK) { vTaskDelay(1); break; }

            /* 3) 校验标记 */
            uint16_t sof = (uint16_t)((frame[0] << 8) | frame[1]);
            uint16_t dft = (uint16_t)((frame[total - 2] << 8) | frame[total - 1]);
            if (sof != DET_SOF || dft != DET_DFT) {
                ESP_LOGW(TAG, "marker bad sof=0x%04x dft=0x%04x len=%u", sof, dft, (unsigned)len);
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }

            /* 4) 喂上层协议栈：纯 payload */
            if (m->mediator) {
                esp_err_t se = m->mediator->stack_input(m->mediator, &frame[DET_SOF_LEN], len);
                if (se != ESP_OK) ESP_LOGW(TAG, "stack_input=%d len=%u", se, (unsigned)len);
            }

            /* 小憩避免死转 */
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        /* INT 已经拉低，重新打开 GPIO 中断 */
        plc_ll_spi_reenable_irq(m->ll);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/* ================= 必要回调（vtbl：严格按 esp_eth_mac_s 顺序） ================= */
static esp_err_t plc_set_mediator(esp_eth_mac_t *mac, esp_eth_mediator_t *mediator)
{ MAC_FROM_PARENT(mac)->mediator = mediator; return ESP_OK; }

static esp_err_t plc_init(esp_eth_mac_t *mac)
{
    plc_mac_t *m = MAC_FROM_PARENT(mac);
    if (m->rx_task == NULL) {
        if (xTaskCreatePinnedToCore(plc_rx_task, "plc_rx", 4096, m, 3, &m->rx_task, 1) != pdPASS) {
            return ESP_FAIL;
        }
    }
    ESP_LOGI(TAG, "init");
    return ESP_OK;
}

static esp_err_t plc_deinit(esp_eth_mac_t *mac) { ESP_LOGI(TAG, "deinit"); return ESP_OK; }
static esp_err_t plc_start(esp_eth_mac_t *mac)  { MAC_FROM_PARENT(mac)->started = true;  ESP_LOGI(TAG, "start"); return ESP_OK; }
static esp_err_t plc_stop(esp_eth_mac_t *mac)   { MAC_FROM_PARENT(mac)->started = false; ESP_LOGI(TAG, "stop");  return ESP_OK; }

/* 发送：如果你当前只做 RX，可返回不支持；若要 TX，可在此封装 DET|RTS + 帧 */
static esp_err_t plc_transmit(esp_eth_mac_t *mac, uint8_t *buf, uint32_t len)
{
    (void)mac; (void)buf; (void)len;
    return ESP_ERR_NOT_SUPPORTED;
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
    plc_ll_spi_destroy(m->ll);
    free(m);
    return ESP_OK;
}

/* ========== 工厂函数（名字/签名不变，vtbl 严格按头文件顺序填充） ========== */
esp_eth_mac_t *esp_eth_mac_new_plcspi(const plcspi_config_t *cfg, const eth_mac_config_t *mac_cfg)
{
    (void)mac_cfg;
    plc_mac_t *m = (plc_mac_t *)calloc(1, sizeof(plc_mac_t));
    if (!m) return NULL;
    if (plc_ll_spi_create(cfg, &m->ll) != ESP_OK) { free(m); return NULL; }

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
    return &m->parent;
}
