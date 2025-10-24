#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_eth_mac.h"
#include "esp_eth_phy.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "plcspi.h"

static const char *TAG = "plc-main";

static void eth_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    esp_eth_handle_t eth = *(esp_eth_handle_t *)arg;
    uint8_t mac[6] = {0};
    switch (id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth, ETH_CMD_G_MAC_ADDR, mac);
        ESP_LOGI(TAG, "Link Up, MAC %02x:%02x:%02x:%02x:%02x:%02x",
                 mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Stopped");
        break;
    default: break;
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_config_t ifcfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *netif = esp_netif_new(&ifcfg);

    ESP_LOGI(TAG, "PLC SPI (RX-only) bring-up start");

    plcspi_config_t plc_cfg = {
        .spi_host    = SPI2_HOST,
        .mosi_io_num = 37,
        .miso_io_num = 36,
        .sclk_io_num = 38,
        .cs_io_num   = 35,
        .irq_io_num  = 7,
        .rst_io_num  = 6,
        .dma_chan    = SPI_DMA_CH_AUTO,
        .clock_hz    = 6 * 1000 * 1000,
        .mode        = 3,
        .cs_setup_us = 1,
        .cs_hold_us  = 0,
        .queue_size  = 4,
    };

    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();

    esp_eth_mac_t *mac = esp_eth_mac_new_plcspi(&plc_cfg, &mac_cfg);
    esp_eth_phy_t *phy = esp_eth_phy_new_plc_dummy(&phy_cfg);
    assert(mac && phy);

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));

    esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(eth_handle);
    ESP_ERROR_CHECK(esp_netif_attach(netif, glue));

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, eth_event_handler, &eth_handle));

    uint8_t mymac[6] = {0x02,0x00,0xDE,0xAD,0xBE,0xEF};
    ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, mymac));
    ESP_LOGI(TAG, "%02x:%02x:%02x:%02x:%02x:%02x", mymac[0],mymac[1],mymac[2],mymac[3],mymac[4],mymac[5]);
    ESP_LOGI(TAG, "ethernet attached");

    ESP_ERROR_CHECK(esp_eth_start(eth_handle));

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_LOGI(TAG, "heartbeat");
    }
}
