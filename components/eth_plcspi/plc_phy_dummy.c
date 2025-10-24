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
    eth_link_t link = ETH_LINK_UP;
    if (phy->mediator && phy->mediator->on_state_changed) {
        phy->mediator->on_state_changed(phy->mediator, ETH_STATE_LINK, &link);
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
    eth_link_t link = ETH_LINK_UP;
    if (phy->mediator && phy->mediator->on_state_changed) {
        phy->mediator->on_state_changed(phy->mediator, ETH_STATE_LINK, &link);
    }
    return ESP_OK;
}
static esp_err_t phy_set_link(esp_eth_phy_t *p, eth_link_t l) { (void)p; (void)l; return ESP_OK; }

static esp_err_t phy_pwrctl(esp_eth_phy_t *p, bool enable) { PHY_FROM_PARENT(p)->powered = enable; return ESP_OK; }
static esp_err_t phy_set_addr(esp_eth_phy_t *p, uint32_t addr) { PHY_FROM_PARENT(p)->addr = addr; return ESP_OK; }
static esp_err_t phy_get_addr(esp_eth_phy_t *p, uint32_t *addr) { if (addr) *addr = PHY_FROM_PARENT(p)->addr; return ESP_OK; }

static esp_err_t phy_advertise_pause(esp_eth_phy_t *p, uint32_t ability) { (void)p; (void)ability; return ESP_OK; }
static esp_err_t phy_loopback(esp_eth_phy_t *p, bool en)  { (void)p; (void)en; return ESP_OK; }
static esp_err_t phy_set_speed(esp_eth_phy_t *p, eth_speed_t s) { (void)p; (void)s; return ESP_OK; }
static esp_err_t phy_set_duplex(esp_eth_phy_t *p, eth_duplex_t d){ (void)p; (void)d; return ESP_OK; }

static esp_err_t phy_custom_ioctl(esp_eth_phy_t *p, int cmd, void *data)
{ (void)p; (void)cmd; (void)data; return ESP_ERR_NOT_SUPPORTED; }

static esp_err_t phy_del(esp_eth_phy_t *p)
{
    free(PHY_FROM_PARENT(p));
    return ESP_OK;
}

/* IDF 802.3 MDIO API：这里不做实现，保持符号存在即可（若工程引用） */
esp_err_t esp_eth_phy_802_3_reg_read(esp_eth_phy_t *phy, uint32_t phy_addr, uint32_t reg_addr, uint32_t *reg_value)
{ (void)phy; (void)phy_addr; (void)reg_addr; (void)reg_value; return ESP_ERR_NOT_SUPPORTED; }
esp_err_t esp_eth_phy_802_3_reg_write(esp_eth_phy_t *phy, uint32_t phy_addr, uint32_t reg_addr, uint32_t reg_value)
{ (void)phy; (void)phy_addr; (void)reg_addr; (void)reg_value; return ESP_ERR_NOT_SUPPORTED; }

/* 工厂函数（签名不变；vtbl 严格按 esp_eth_phy_s 顺序填充） */
esp_err_t esp_eth_phy_new_plc_dummy_fill(esp_eth_phy_t *out); /* 可选：不导出，仅内部使用 */

esp_eth_phy_t *esp_eth_phy_new_plc_dummy(const eth_phy_config_t *phy_cfg)
{
    (void)phy_cfg;
    plc_phy_t *phy = (plc_phy_t *)calloc(1, sizeof(plc_phy_t));
    if (!phy) return NULL;

    esp_eth_phy_t *p = &phy->parent;
    p->set_mediator              = phy_set_mediator;
    p->reset                     = phy_reset;
    p->reset_hw                  = phy_reset_hw;
    p->init                      = phy_init;
    p->deinit                    = phy_deinit;
    p->autonego_ctrl             = phy_autonego_ctrl;
    p->get_link                  = phy_get_link;
    p->set_link                  = phy_set_link;
    p->pwrctl                    = phy_pwrctl;
    p->set_addr                  = phy_set_addr;
    p->get_addr                  = phy_get_addr;
    p->advertise_pause_ability   = phy_advertise_pause;
    p->loopback                  = phy_loopback;
    p->set_speed                 = phy_set_speed;
    p->set_duplex                = phy_set_duplex;
    p->custom_ioctl              = phy_custom_ioctl;
    p->del                       = phy_del;

    return &phy->parent;
}
