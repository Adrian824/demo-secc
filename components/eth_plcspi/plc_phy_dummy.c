#include "esp_eth_phy.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "plc-phy";

typedef struct {
    esp_eth_phy_t       parent;     /* vtbl 放首位 */
    esp_eth_mediator_t *mediator;
    uint32_t            addr;
    bool                powered;
} plc_phy_t;

#ifndef CONTAINER_OF
#define CONTAINER_OF(ptr, type, member) ((type *)((char *)(ptr) - offsetof(type, member)))
#endif
#define PHY_FROM_PARENT(p) CONTAINER_OF((p), plc_phy_t, parent)

static esp_err_t phy_set_mediator(esp_eth_phy_t *p, esp_eth_mediator_t *m)
{ PHY_FROM_PARENT(p)->mediator = m; return ESP_OK; }

static esp_err_t phy_reset(esp_eth_phy_t *p)    { (void)p; return ESP_OK; }
static esp_err_t phy_reset_hw(esp_eth_phy_t *p) { (void)p; return ESP_OK; }

static esp_err_t phy_init(esp_eth_phy_t *p)
{
    plc_phy_t *phy = PHY_FROM_PARENT(p);
    if (phy->mediator && phy->mediator->on_state_changed) {
        // 告诉上层：链路已 UP
        phy->mediator->on_state_changed(phy->mediator, ETH_STATE_LINK, (void *)ETH_LINK_UP);
        //（可选）同时把速率/双工也告诉上层，避免 netif 状态不完整
        phy->mediator->on_state_changed(phy->mediator, ETH_STATE_SPEED, (void *)ETH_SPEED_10M);
        phy->mediator->on_state_changed(phy->mediator, ETH_STATE_DUPLEX, (void *)ETH_DUPLEX_FULL);
    }
    ESP_LOGI(TAG, "phy dummy init -> LINK UP");
    return ESP_OK;
}
static esp_err_t phy_deinit(esp_eth_phy_t *p)   { (void)p; return ESP_OK; }

static esp_err_t phy_autonego_ctrl(esp_eth_phy_t *p, eth_phy_autoneg_cmd_t cmd, bool *stat)
{ (void)p; (void)cmd; if (stat) *stat = false; return ESP_OK; }

static esp_err_t phy_get_link(esp_eth_phy_t *p)
{
    plc_phy_t *phy = PHY_FROM_PARENT(p);
    if (phy->mediator && phy->mediator->on_state_changed) {
        // 轮询时同样回报 UP（dummy 就一直 UP）
        phy->mediator->on_state_changed(phy->mediator, ETH_STATE_LINK, (void *)ETH_LINK_UP);
    }
    return ESP_OK;
}
static esp_err_t phy_set_link(esp_eth_phy_t *p, eth_link_t l) { (void)p; (void)l; return ESP_OK; }

static esp_err_t phy_pwrctl(esp_eth_phy_t *p, bool enable) { PHY_FROM_PARENT(p)->powered = enable; return ESP_OK; }
static esp_err_t phy_set_addr(esp_eth_phy_t *p, uint32_t a) { PHY_FROM_PARENT(p)->addr = a; return ESP_OK; }
static esp_err_t phy_get_addr(esp_eth_phy_t *p, uint32_t *a) { if (a) *a = PHY_FROM_PARENT(p)->addr; return ESP_OK; }
static esp_err_t phy_advertise_pause_ability(esp_eth_phy_t *p, uint32_t a) { (void)p; (void)a; return ESP_OK; }
static esp_err_t phy_loopback(esp_eth_phy_t *p, bool e) { (void)p; (void)e; return ESP_OK; }
static esp_err_t phy_set_speed(esp_eth_phy_t *p, eth_speed_t s) { (void)p; (void)s; return ESP_OK; }
static esp_err_t phy_set_duplex(esp_eth_phy_t *p, eth_duplex_t d) { (void)p; (void)d; return ESP_OK; }
static esp_err_t phy_custom_ioctl(esp_eth_phy_t *p, int cmd, void *arg) { (void)p; (void)cmd; (void)arg; return ESP_OK; }

static esp_err_t phy_del(esp_eth_phy_t *p) { free(PHY_FROM_PARENT(p)); return ESP_OK; }

esp_eth_phy_t *esp_eth_phy_new_plc_dummy(const eth_phy_config_t *phy_config)
{
    (void)phy_config;
    plc_phy_t *phy = (plc_phy_t *)calloc(1, sizeof(plc_phy_t));
    if (!phy) return NULL;

    phy->parent.set_mediator               = phy_set_mediator;
    phy->parent.reset                      = phy_reset;
    phy->parent.reset_hw                   = phy_reset_hw;
    phy->parent.init                       = phy_init;
    phy->parent.deinit                     = phy_deinit;
    phy->parent.autonego_ctrl              = phy_autonego_ctrl;
    phy->parent.get_link                   = phy_get_link;
    phy->parent.set_link                   = phy_set_link;
    phy->parent.pwrctl                     = phy_pwrctl;
    phy->parent.set_addr                   = phy_set_addr;
    phy->parent.get_addr                   = phy_get_addr;
    phy->parent.advertise_pause_ability    = phy_advertise_pause_ability;
    phy->parent.loopback                   = phy_loopback;
    phy->parent.set_speed                  = phy_set_speed;
    phy->parent.set_duplex                 = phy_set_duplex;
    phy->parent.custom_ioctl               = phy_custom_ioctl;
    phy->parent.del                        = phy_del;

    return &phy->parent;
}
