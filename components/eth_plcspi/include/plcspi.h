#pragma once
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_eth.h"
#include "esp_eth_mac.h"
#include "esp_eth_phy.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define DET_CMD        0x0001u
#define DET_SOF        0x0002u
#define DET_DFT        0x55AAu
#define CMD_SHIFT      12
#define CMD_RTS        (0x1u << CMD_SHIFT)
#define CMD_CTR        (0x2u << CMD_SHIFT)

#define DET_CMD_LEN    4u
#define DET_SOF_LEN    2u
#define DET_DFT_LEN    2u

typedef struct {
    spi_host_device_t spi_host;
    int mosi_io_num;
    int miso_io_num;
    int sclk_io_num;
    int cs_io_num;
    int irq_io_num;     /* PLC INT，高电平=有数据 */
    int rst_io_num;     /* -1 不用 */
    int dma_chan;
    uint32_t clock_hz;
    uint8_t  mode;
    uint16_t cs_setup_us;
    uint16_t cs_hold_us;
    int queue_size;
} plcspi_config_t;

esp_err_t plc_ll_spi_create(const plcspi_config_t *cfg, void **out_ll);
void      plc_ll_spi_destroy(void *ll);
esp_err_t plc_ll_spi_tx(void *ll, const void *buf, size_t len);
esp_err_t plc_ll_spi_rx(void *ll, void *buf, size_t len);
esp_err_t plc_ll_spi_txrx(void *ll, const void *tx, void *rx, size_t len);


bool      plc_ll_spi_has_data(void *ll);
esp_err_t plc_ll_spi_wait_irq(void *ll, uint32_t timeout_ms);
bool      plc_ll_spi_int_level(void *ll);
void      plc_ll_spi_reenable_irq(void *ll);

esp_eth_mac_t *esp_eth_mac_new_plcspi(const plcspi_config_t *cfg, const eth_mac_config_t *mac_config);
esp_eth_phy_t *esp_eth_phy_new_plc_dummy(const eth_phy_config_t *phy_config);
