// plc_ll_spi.c
#include "plcspi.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "plc-ll";

/* 内部低层句柄；不改变对外接口类型/名称 */
typedef struct {
    spi_device_handle_t spi;
    int cs_io, irq_io, rst_io;
    SemaphoreHandle_t irq_sem;    /* ISR 唤醒 RX 任务 */
    uint16_t cs_setup_us, cs_hold_us;
} plc_ll_t;

/* ====== 私有 SPI 事务封装 ====== */
static inline esp_err_t do_trans(plc_ll_t *ll, const void *tx, void *rx, size_t len)
{
    spi_transaction_t t = {
        .length   = len * 8,
        .tx_buffer= tx,
        .rx_buffer= rx,
    };
    return spi_device_transmit(ll->spi, &t);
}

/* ====== ISR：高电平触发。仅做轻量唤醒，不访问 SPI ====== */
static void IRAM_ATTR plc_ll_gpio_isr(void *arg)
{
    plc_ll_t *ll = (plc_ll_t *)arg;

    // 电平触发可能反复进中断；只有确认仍为高才屏蔽并唤醒任务
    if (gpio_get_level(ll->irq_io)) {
        gpio_intr_disable(ll->irq_io); // 交给任务去“读空->电平回低->再开中断”
        BaseType_t hpw = pdFALSE;
        if (ll->irq_sem) {
            xSemaphoreGiveFromISR(ll->irq_sem, &hpw);
        }
        if (hpw == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

/* ====== 对外 API：创建/销毁 ====== */
esp_err_t plc_ll_spi_create(const plcspi_config_t *cfg, void **out_ll)
{
    if (!cfg || !out_ll) return ESP_ERR_INVALID_ARG;

    plc_ll_t *ll = (plc_ll_t *)calloc(1, sizeof(plc_ll_t));
    if (!ll) return ESP_ERR_NO_MEM;

    ll->cs_io       = cfg->cs_io_num;
    ll->irq_io      = cfg->irq_io_num;
    ll->rst_io      = cfg->rst_io_num;
    ll->cs_setup_us = cfg->cs_setup_us;
    ll->cs_hold_us  = cfg->cs_hold_us;

    /* SPI 总线初始化（保持原参数语义不变） */
    spi_bus_config_t bus = {
        .mosi_io_num     = cfg->mosi_io_num,
        .miso_io_num     = cfg->miso_io_num,
        .sclk_io_num     = cfg->sclk_io_num,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 2048,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(cfg->spi_host, &bus, cfg->dma_chan));

    // cs_ena_* 以 SPI 时钟周期为单位，做近似换算
    uint8_t pre  = (uint8_t)((cfg->cs_setup_us * (cfg->clock_hz / 1000000UL)));
    uint8_t post = (uint8_t)((cfg->cs_hold_us  * (cfg->clock_hz / 1000000UL)));

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

    /* === 先做外设上电复位：RST 低 1s → 高，给 10ms 恢复时间 === */
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
        vTaskDelay(pdMS_TO_TICKS(1000));  // 低 1s
        gpio_set_level(ll->rst_io, 1);
        vTaskDelay(pdMS_TO_TICKS(10));    // 恢复 10ms
    }

    /* IRQ 引脚：输入+无上下拉（由外部电路决定）+ 高电平触发 */
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << ll->irq_io,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = 0,
        .pull_down_en = 0,
        .intr_type    = GPIO_INTR_HIGH_LEVEL,  // ★ 高电平触发
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    // 仅第一次安装 ISR service；多次调用会返回 ESP_ERR_INVALID_STATE
    esp_err_t isr_ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (isr_ret != ESP_OK && isr_ret != ESP_ERR_INVALID_STATE) {
        return isr_ret;
    }

    ll->irq_sem = xSemaphoreCreateBinary();
    if (!ll->irq_sem) return ESP_ERR_NO_MEM;

    ESP_ERROR_CHECK(gpio_isr_handler_add(ll->irq_io, plc_ll_gpio_isr, ll));

    // 复位完成后再打开中断，避免复位窗口内的伪触发
    ESP_ERROR_CHECK(gpio_intr_enable(ll->irq_io));

    // 兜底：若此刻已为高（极端时序/外设仍报告中断），模拟 ISR 行为一次唤醒任务
    if (gpio_get_level(ll->irq_io)) {
        gpio_intr_disable(ll->irq_io);
        if (ll->irq_sem) xSemaphoreGive(ll->irq_sem);
    }

    ESP_LOGI(TAG,
             "init ok host=%d CS=%d CLK=%d MOSI=%d MISO=%d IRQ=%d RST=%d freq=%uHz (HIGH-LEVEL IRQ)",
             (int)cfg->spi_host, cfg->cs_io_num, cfg->sclk_io_num,
             cfg->mosi_io_num, cfg->miso_io_num, cfg->irq_io_num, cfg->rst_io_num,
             (unsigned)cfg->clock_hz);

    *out_ll = ll;
    return ESP_OK;
}

void plc_ll_spi_destroy(void *h)
{
    if (!h) return;
    plc_ll_t *ll = (plc_ll_t *)h;

    if (ll->irq_io >= 0) {
        gpio_isr_handler_remove(ll->irq_io);
        gpio_intr_disable(ll->irq_io);
    }
    if (ll->spi) {
        spi_bus_remove_device(ll->spi);
        // 是否 spi_bus_free(host) 由上层统一决定；保持原行为不改动
    }
    if (ll->irq_sem) vSemaphoreDelete(ll->irq_sem);
    free(ll);
}

/* ====== 对外 API：同步 SPI 读写（保持原签名/语义） ====== */
esp_err_t plc_ll_spi_tx(void *h, const void *buf, size_t len)
{
    if (!h || !buf || !len) return ESP_ERR_INVALID_ARG;
    return do_trans((plc_ll_t *)h, buf, NULL, len);
}

esp_err_t plc_ll_spi_rx(void *h, void *buf, size_t len)
{
    if (!h || !buf || !len) return ESP_ERR_INVALID_ARG;
    memset(buf, 0, len);
    return do_trans((plc_ll_t *)h, NULL, buf, len);
}

esp_err_t plc_ll_spi_txrx(void *h, const void *tx, void *rx, size_t len)
{
    if (!h || (!tx && !rx) || !len) return ESP_ERR_INVALID_ARG;
    return do_trans((plc_ll_t *)h, tx, rx, len);
}

/* ====== 对外 API：IRQ 工具（保持原签名/语义） ====== */
bool plc_ll_spi_has_data(void *h)
{
    plc_ll_t *ll = (plc_ll_t *)h;
    return gpio_get_level(ll->irq_io);
}

esp_err_t plc_ll_spi_wait_irq(void *h, uint32_t timeout_ms)
{
    plc_ll_t *ll = (plc_ll_t *)h;

    // 若当前已为高，模拟 ISR 的第一步：先关中断再返回 OK
    if (gpio_get_level(ll->irq_io)) {
        gpio_intr_disable(ll->irq_io);
        return ESP_OK;
    }

    // 否则等待 ISR 丢过来的信号量
    return (xSemaphoreTake(ll->irq_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE)
           ? ESP_OK : ESP_ERR_TIMEOUT;
}

bool plc_ll_spi_int_level(void *h)
{
    plc_ll_t *ll = (plc_ll_t *)h;
    return gpio_get_level(ll->irq_io);
}

void plc_ll_spi_reenable_irq(void *h)
{
    plc_ll_t *ll = (plc_ll_t *)h;
    // 幂等重开；按你的任务流程，一般在确认电平已低后调用
    gpio_intr_enable(ll->irq_io);
}
