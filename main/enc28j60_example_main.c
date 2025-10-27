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
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


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



static const char *TX_TAG = "tx_smoke";
// 简单的周期性广播帧发送任务：每 1s 发一帧 60 字节（不含FCS）
static void tx_smoke_task(void *arg)
{
    esp_eth_handle_t eth = (esp_eth_handle_t)arg;

    // 准备一个 60B 的以太帧缓冲（不含FCS）
    //uint8_t frame[60];
    //memset(frame, 0, sizeof(frame));

    // 目的 MAC：广播 FF:FF:FF:FF:FF:FF
    // memset(&frame[0], 0xFF, 6);

    // // 源 MAC：读取 esp_eth 的本机 MAC
    // uint8_t src_mac[6] = {0};
    // if (esp_eth_ioctl(eth, ETH_CMD_G_MAC_ADDR, src_mac) == ESP_OK) {
    //     memcpy(&frame[6], src_mac, 6);
    // } else {
    //     // 读不到也无妨，默认 00 填充
    //     ESP_LOGW(TX_TAG, "get MAC failed, using 00:00:00:00:00:00");
    // }

    // // Ethertype：0x88E1（HPGP/SLAC 管理面，随便选一个非 IP，用来“能看到有TX就行”）
    // frame[12] = 0x88;
    // frame[13] = 0xE1;

    // // payload 14..59：随便填充一些递增字节，便于示波器/LA 观察
    // for (int i = 14; i < 60; ++i) frame[i] = (uint8_t)i;

    uint8_t frame[60] = {0x00, 0x13, 0xd7, 0x11, 0x00, 0x01, 0x00, 0x3a, 0x31, 0x51, 0x38, 0x35, 0x88, 0xe1, 0x01, 0x30, 0xa0, 0x00, 0x00, 0x00, 0x13, 0xd7};
    
    // 周期性发送
    while (1) {
        esp_err_t err = esp_eth_transmit(eth, frame, sizeof(frame));
        if (err != ESP_OK) {
            ESP_LOGE(TX_TAG, "esp_eth_transmit failed: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TX_TAG, "broadcast 60B sent");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static const char *SDP_TAG = "sdp_sendto";
static TaskHandle_t s_sdp_task = NULL;
esp_netif_t *g_eth_netif;
// 任务：每 2s 发一次 UDP（IPv4 广播 + IPv6 组播各发一条）
static void sdp_sendto_task(void *arg)
{
    // ===== IPv4 广播：255.255.255.255:15118（示例端口，按需修改） =====
    int s4 = socket(AF_INET, SOCK_DGRAM, 0);
    int yes = 1; setsockopt(s4, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));

    struct sockaddr_in dst4 = {0};
    dst4.sin_family = AF_INET;
    dst4.sin_port   = htons(15118);                // 示例：15118
    dst4.sin_addr.s_addr = inet_addr("255.255.255.255");

    // ===== IPv6 组播：ff02::1:2:15118（示例：链路本地组播，端口同上）=====
    int s6 = socket(AF_INET6, SOCK_DGRAM, 0);
    struct sockaddr_in6 dst6 = {0};
    dst6.sin6_family = AF_INET6;
    dst6.sin6_port   = htons(15118);
    inet6_aton("ff02::1", &dst6.sin6_addr);        // 示例用 all-nodes，可换成你的 SDP 组播地址
    // VERY IMPORTANT: 指定接口 scope_id（以太网 netif 的索引），否则发不出去
    if (g_eth_netif) {
        dst6.sin6_scope_id = esp_netif_get_netif_impl_index(g_eth_netif);
    }

    // 示例 payload：你后面可以替换为 V2GTP 头 + SDP 内容
    const uint8_t sdp_payload[] = { 'S','D','P','-','H','E','L','L','O' };

    while (1) {
        // IPv4 广播
        int r4 = sendto(s4, sdp_payload, sizeof(sdp_payload), 0,
                        (struct sockaddr*)&dst4, sizeof(dst4));
        if (r4 >= 0) ESP_LOGI(SDP_TAG, "IPv4 broadcast sent (%dB)", r4);
        else         ESP_LOGE(SDP_TAG, "IPv4 sendto fail errno=%d", errno);

        // IPv6 组播（链路本地）
        int r6 = sendto(s6, sdp_payload, sizeof(sdp_payload), 0,
                        (struct sockaddr*)&dst6, sizeof(dst6));
        if (r6 >= 0) ESP_LOGI(SDP_TAG, "IPv6 mcast sent (%dB)", r6);
        else         ESP_LOGE(SDP_TAG, "IPv6 sendto fail errno=%d", errno);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}






void app_main(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_config_t ifcfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *netif = esp_netif_new(&ifcfg);


    g_eth_netif = netif;
    esp_netif_dhcpc_stop(netif);

    // 配一个随便的私网地址（示例：192.168.7.2/24），网关可 0.0.0.0
    esp_netif_ip_info_t ip;
    ip.ip.addr      = ipaddr_addr("192.168.7.2");
    ip.netmask.addr = ipaddr_addr("255.255.255.0");
    ip.gw.addr      = ipaddr_addr("0.0.0.0");
    ESP_ERROR_CHECK(esp_netif_set_ip_info(netif, &ip));

    // 可选：把这个 netif 设为默认
    esp_netif_set_default_netif(netif);
    
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
    vTaskDelay(100);
    // 在 start 成功后，开发送任务
    //xTaskCreate(tx_smoke_task, "tx_smoke", 3072, (void*)eth_handle, 4, NULL);
    xTaskCreate(sdp_sendto_task, "sdp_sendto", 4096, NULL, 4, &s_sdp_task);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_LOGI(TAG, "heartbeat");
    }
}
