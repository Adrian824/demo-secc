#include "plcspi.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include "esp_check.h"

static const char *TAG = "plc-ll";

typedef struct {
    spi_device_handle_t spi;
    int cs_io, irq_io, rst_io;
    uint16_t cs_setup_us, cs_hold_us;
    SemaphoreHandle_t rx_sem;
    SemaphoreHandle_t spi_lock;
} plc_ll_t;

static void IRAM_ATTR plc_gpio_isr(void *arg)
{
    plc_ll_t *ll = (plc_ll_t *)arg;
    // 先关本引脚中断，留给任务在“读空后”再开
    gpio_intr_disable(ll->irq_io);
    BaseType_t hpw = pdFALSE;
    if (ll->rx_sem) {
        xSemaphoreGiveFromISR(ll->rx_sem, &hpw);
    }
    if (hpw) portYIELD_FROM_ISR();
}

static inline esp_err_t do_trans(plc_ll_t *ll, const void *tx, void *rx, size_t len)
{
    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    if (ll->spi_lock) xSemaphoreTake(ll->spi_lock, portMAX_DELAY);
    esp_err_t ret = spi_device_transmit(ll->spi, &t);
    if (ll->spi_lock) xSemaphoreGive(ll->spi_lock);
    return ret;
}

esp_err_t plc_ll_spi_create(const plcspi_config_t *cfg, void **out_ll)
{
    if (!cfg || !out_ll) return ESP_ERR_INVALID_ARG;

    esp_err_t e = gpio_install_isr_service(0);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;

    plc_ll_t *ll = (plc_ll_t *)calloc(1, sizeof(plc_ll_t));
    if (!ll) return ESP_ERR_NO_MEM;

    ll->cs_io       = cfg->cs_io_num;
    ll->irq_io      = cfg->irq_io_num;
    ll->rst_io      = cfg->rst_io_num;
    ll->cs_setup_us = cfg->cs_setup_us;
    ll->cs_hold_us  = cfg->cs_hold_us;

    spi_bus_config_t bus = {
        .mosi_io_num     = cfg->mosi_io_num,
        .miso_io_num     = cfg->miso_io_num,
        .sclk_io_num     = cfg->sclk_io_num,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 2048,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(cfg->spi_host, &bus, cfg->dma_chan));

    uint32_t pre32  = cfg->cs_setup_us * (cfg->clock_hz / 1000000UL);
    uint32_t post32 = cfg->cs_hold_us  * (cfg->clock_hz / 1000000UL);
    uint8_t  pre  = (pre32  > 255) ? 255 : (uint8_t)pre32;
    uint8_t  post = (post32 > 255) ? 255 : (uint8_t)post32;

    spi_device_interface_config_t dev = {
        .clock_speed_hz   = (int)cfg->clock_hz,
        .mode             = cfg->mode,
        .spics_io_num     = cfg->cs_io_num,
        .queue_size       = cfg->queue_size > 0 ? cfg->queue_size : 4,
        .cs_ena_pretrans  = pre,
        .cs_ena_posttrans = post,
        .flags            = SPI_DEVICE_NO_DUMMY,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(cfg->spi_host, &dev, &ll->spi));

    // 1) 先建锁/信号量（避免早期丢唤醒）
    ll->spi_lock = xSemaphoreCreateMutex();
    ll->rx_sem   = xSemaphoreCreateBinary();

    // 2) RST 单独配置为输出并完成上电复位
    if (ll->rst_io >= 0) {
        gpio_config_t rst = {
            .pin_bit_mask = 1ULL << ll->rst_io,
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = 0,
            .pull_down_en = 0,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&rst));
        gpio_set_level(ll->rst_io, 0);
        vTaskDelay(pdMS_TO_TICKS(50));   // 足够即可，后续可调小到 10~50ms
        gpio_set_level(ll->rst_io, 1);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // 3) IRQ 仅自己配置为输入 + 高电平触发（不要带上 RST）
    gpio_config_t irq = {
        .pin_bit_mask = 1ULL << ll->irq_io,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = 0,
        .pull_down_en = 0,
        .intr_type    = GPIO_INTR_HIGH_LEVEL,
    };
    ESP_ERROR_CHECK(gpio_config(&irq));

    // 4) 注册回调，最后一步才开启中断
    gpio_isr_handler_add(ll->irq_io, plc_gpio_isr, ll);
    gpio_intr_enable(ll->irq_io);


    ESP_LOGI(TAG, "SPI init ok host=%d CS=%d CLK=%d MOSI=%d MISO=%d IRQ=%d RST=%d freq=%uHz",
             (int)cfg->spi_host, cfg->cs_io_num, cfg->sclk_io_num,
             cfg->mosi_io_num, cfg->miso_io_num, cfg->irq_io_num, cfg->rst_io_num,
             (unsigned)cfg->clock_hz);

    *out_ll = ll;
    return ESP_OK;
}

void plc_ll_spi_destroy(void *h)
{
    plc_ll_t *ll = (plc_ll_t *)h;
    if (!ll) return;
    if (ll->irq_io >= 0) 
    {
        gpio_intr_disable(ll->irq_io);
        gpio_isr_handler_remove(ll->irq_io);
    }
    if (ll->spi) {
        spi_device_handle_t d = ll->spi;
        ll->spi = NULL;
        spi_bus_remove_device(d);
    }

    if (ll->rx_sem)  vSemaphoreDelete(ll->rx_sem);
    if (ll->spi_lock) vSemaphoreDelete(ll->spi_lock);

    free(ll);
}

esp_err_t plc_ll_spi_tx(void *h, const void *buf, size_t len)
{ plc_ll_t *ll = (plc_ll_t *)h; return do_trans(ll, buf, NULL, len); }

esp_err_t plc_ll_spi_rx(void *h, void *buf, size_t len)
{ plc_ll_t *ll = (plc_ll_t *)h; return do_trans(ll, NULL, buf, len); }

esp_err_t plc_ll_spi_txrx(void *h, const void *tx, void *rx, size_t len)
{ plc_ll_t *ll = (plc_ll_t *)h; return do_trans(ll, tx, rx, len); }

/* 仅轮询高电平 */
bool plc_ll_spi_has_data(void *h)
{ plc_ll_t *ll = (plc_ll_t *)h; return gpio_get_level(ll->irq_io); }

bool plc_ll_spi_int_level(void *h)
{ plc_ll_t *ll = (plc_ll_t *)h; return gpio_get_level(ll->irq_io); }

esp_err_t plc_ll_spi_wait_irq(void *h, uint32_t timeout_ms)
{
    plc_ll_t *ll = (plc_ll_t *)h;
    // 先快速看一眼电平（避免错过已为高的情况）
    if (gpio_get_level(ll->irq_io)) return ESP_OK;

    if (ll->rx_sem) {
        if (xSemaphoreTake(ll->rx_sem,
               timeout_ms ? pdMS_TO_TICKS(timeout_ms) : portMAX_DELAY) == pdTRUE) {
            return ESP_OK;
        }
        return ESP_ERR_TIMEOUT;
    }
    // 没有 rx_sem（极端或未初始化）=> 轮询退化
    TickType_t start = xTaskGetTickCount();
    for (;;) {
        if (gpio_get_level(ll->irq_io)) return ESP_OK;
        if (timeout_ms &&
            (xTaskGetTickCount() - start) >= pdMS_TO_TICKS(timeout_ms)) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}



// 发送 DET|RTS(len)，并等待回读到 DET|CTR(accept)；超时返回 ESP_ERR_TIMEOUT
esp_err_t plc_ll_spi_rts_wait_ctr(void *h, uint16_t total_len, uint32_t timeout_ms)
{
    plc_ll_t *ll = (plc_ll_t *)h;
    uint8_t cmd[4];
    uint8_t rx[4];
    uint8_t dummy[4] = {0xAA,0xAA,0xAA,0xAA};
    TickType_t start = xTaskGetTickCount();

    // 组 DET|RTS(len) 4 字节（高字节在前）
    cmd[0] = (DET_CMD >> 8) & 0xFF;
    cmd[1] = (DET_CMD) & 0xFF;
    cmd[2] = ((CMD_RTS | total_len) >> 8) & 0xFF;
    cmd[3] = ((CMD_RTS | total_len) & 0xFF);

    // 发 RTS
    ESP_RETURN_ON_ERROR(plc_ll_spi_tx(ll, cmd, sizeof(cmd)), TAG, "RTS tx failed");

    // 轮询回读 CTR
    for (;;) {
        // 发 0xAA 0xAA 0xAA 0xAA 占位，读 4 字节状态
        ESP_RETURN_ON_ERROR(plc_ll_spi_txrx(ll, dummy, rx, sizeof(rx)), TAG, "CTR rx failed");

        // 期望: 00 01 20 00  （= DET_CMD, CMD_CTR|0x0000）
        if (rx[0] == 0x00 && rx[1] == 0x01 && rx[2] == 0x20 && rx[3] == 0x00) {
            return ESP_OK;
        }
        if (timeout_ms && (xTaskGetTickCount() - start) >= pdMS_TO_TICKS(timeout_ms)) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}



void plc_ll_spi_reenable_irq(void *h)
{
    plc_ll_t *ll = (plc_ll_t *)h;
    if (gpio_get_level(ll->irq_io)) {
        // 仍为高：不重新开中断，让上层循环继续读
        return;
    }
    gpio_intr_enable(ll->irq_io);
}
