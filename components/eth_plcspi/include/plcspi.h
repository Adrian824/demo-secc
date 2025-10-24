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

/* ===== 协议常量：DET/CTR/SOF/DFT ===== */
#define DET_CMD        0x0001u
#define DET_SOF        0x0002u
#define DET_DFT        0x55AAu
#define CMD_SHIFT      12
#define CMD_RTS        (0x1u << CMD_SHIFT)
#define CMD_CTR        (0x2u << CMD_SHIFT)

#define DET_CMD_LEN    4u
#define DET_SOF_LEN    2u
#define DET_DFT_LEN    2u

/* ===== 低层 SPI 配置（字段名尽量与现工程一致） ===== */
typedef struct {
    spi_host_device_t spi_host;   /* SPI2_HOST/SPI3_HOST */
    int mosi_io_num;
    int miso_io_num;
    int sclk_io_num;
    int cs_io_num;
    int irq_io_num;               /* PLC INT，高电平=有数据 */
    int rst_io_num;               /* 可选 RST，-1 不用 */
    int dma_chan;                 /* SPI DMA 通道 */
    uint32_t clock_hz;            /* SPI 时钟 */
    uint8_t  mode;                /* SPI 模式 0..3 */
    uint16_t cs_setup_us;         /* CS 置前保持(us) */
    uint16_t cs_hold_us;          /* CS 置后保持(us) */
    int queue_size;               /* 事务队列长度 */
} plcspi_config_t;

/* ===== 低层 SPI 句柄 & API（不改名不改参） ===== */
esp_err_t plc_ll_spi_create(const plcspi_config_t *cfg, void **out_ll);
void      plc_ll_spi_destroy(void *ll);

esp_err_t plc_ll_spi_tx(void *ll, const void *buf, size_t len);
esp_err_t plc_ll_spi_rx(void *ll, void *buf, size_t len);
esp_err_t plc_ll_spi_txrx(void *ll, const void *tx, void *rx, size_t len);

bool      plc_ll_spi_has_data(void *ll);                 /* 读取 INT 电平 */
esp_err_t plc_ll_spi_wait_irq(void *ll, uint32_t timeout_ms); /* 等待上升沿 */
bool      plc_ll_spi_int_level(void *ll);                /* 当前 INT */
void      plc_ll_spi_reenable_irq(void *ll);             /* 读空后再开中断 */

/* ===== 工厂函数（不改对外名字/签名） ===== */
esp_eth_mac_t *esp_eth_mac_new_plcspi(const plcspi_config_t *cfg, const eth_mac_config_t *mac_config);
esp_eth_phy_t *esp_eth_phy_new_plc_dummy(const eth_phy_config_t *phy_config);
