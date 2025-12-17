#include "spi_slave_driver.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "SPI_SLAVE";

static bool driver_initialized = false;
static SemaphoreHandle_t driver_mutex = NULL;
static bool slave_configured[SPI_SLAVE_HOST_COUNT] = {false};
static spi_slave_driver_config_t slave_configs[SPI_SLAVE_HOST_COUNT] = {0};

esp_err_t spi_slave_driver_init(void)
{
    if (driver_initialized) {
        ESP_LOGW(TAG, "SPI slave driver already initialized");
        return ESP_OK;
    }

    driver_mutex = xSemaphoreCreateMutex();
    if (driver_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    memset(slave_configured, 0, sizeof(slave_configured));
    memset(slave_configs, 0, sizeof(slave_configs));

    driver_initialized = true;
    ESP_LOGI(TAG, "SPI slave driver initialized");
    return ESP_OK;
}

esp_err_t spi_slave_driver_config(const spi_slave_driver_config_t *config)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "SPI slave driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid configuration pointer");
        return ESP_ERR_INVALID_ARG;
    }

    // SPI1 is reserved for flash
    if (config->host_id < 1 || config->host_id >= SPI_SLAVE_HOST_COUNT) {
        ESP_LOGE(TAG, "Invalid SPI host ID for slave: %d (use SPI2_HOST or SPI3_HOST)", config->host_id);
        return ESP_ERR_INVALID_ARG;
    }

    if (config->mode > 3) {
        ESP_LOGE(TAG, "Invalid SPI mode: %d (must be 0-3)", config->mode);
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    // Free existing slave if configured
    if (slave_configured[config->host_id]) {
        spi_slave_free((spi_host_device_t)config->host_id);
        slave_configured[config->host_id] = false;
    }

    spi_bus_config_t bus_config = {
        .mosi_io_num = config->mosi_io_num,
        .miso_io_num = config->miso_io_num,
        .sclk_io_num = config->sclk_io_num,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };

    spi_slave_interface_config_t slave_config = {
        .mode = config->mode,
        .spics_io_num = config->cs_io_num,
        .queue_size = config->queue_size > 0 ? config->queue_size : 3,
        .flags = config->flags,
        .post_setup_cb = config->post_setup_cb,
        .post_trans_cb = config->post_trans_cb,
    };

    esp_err_t ret = spi_slave_initialize((spi_host_device_t)config->host_id, &bus_config, &slave_config, config->dma_chan);
    if (ret != ESP_OK) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGE(TAG, "Failed to initialize SPI slave on host %d: %s", config->host_id, esp_err_to_name(ret));
        return ret;
    }

    memcpy(&slave_configs[config->host_id], config, sizeof(spi_slave_driver_config_t));
    slave_configured[config->host_id] = true;

    xSemaphoreGive(driver_mutex);

    ESP_LOGI(TAG, "SPI slave configured: host=%d, MOSI=%d, MISO=%d, SCLK=%d, CS=%d, mode=%d",
             config->host_id, config->mosi_io_num, config->miso_io_num, 
             config->sclk_io_num, config->cs_io_num, config->mode);
    return ESP_OK;
}

esp_err_t spi_slave_driver_free(int host_id)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "SPI slave driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (host_id < 1 || host_id >= SPI_SLAVE_HOST_COUNT) {
        ESP_LOGE(TAG, "Invalid SPI host ID: %d", host_id);
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    if (!slave_configured[host_id]) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGW(TAG, "SPI slave not configured on host %d", host_id);
        return ESP_OK;
    }

    esp_err_t ret = spi_slave_free((spi_host_device_t)host_id);
    slave_configured[host_id] = false;
    memset(&slave_configs[host_id], 0, sizeof(spi_slave_driver_config_t));

    xSemaphoreGive(driver_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to free SPI slave: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SPI slave freed: host=%d", host_id);
    return ESP_OK;
}

esp_err_t spi_slave_driver_queue_trans(int host_id, const uint8_t *tx_data, uint8_t *rx_data, size_t len, int timeout_ms)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "SPI slave driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (host_id < 1 || host_id >= SPI_SLAVE_HOST_COUNT) {
        ESP_LOGE(TAG, "Invalid SPI host ID: %d", host_id);
        return ESP_ERR_INVALID_ARG;
    }

    if (len == 0) {
        ESP_LOGE(TAG, "Invalid length");
        return ESP_ERR_INVALID_ARG;
    }

    if (tx_data == NULL && rx_data == NULL) {
        ESP_LOGE(TAG, "Both tx_data and rx_data cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (!slave_configured[host_id]) {
        ESP_LOGE(TAG, "SPI slave not configured on host %d", host_id);
        return ESP_ERR_INVALID_STATE;
    }

    // Validate DMA buffer capability when DMA is enabled
    if (slave_configs[host_id].dma_chan != 0) {
        if (tx_data && !esp_ptr_dma_capable(tx_data)) {
            ESP_LOGE(TAG, "TX buffer not DMA-capable (must be in DRAM)");
            return ESP_ERR_INVALID_ARG;
        }
        if (rx_data && !esp_ptr_dma_capable(rx_data)) {
            ESP_LOGE(TAG, "RX buffer not DMA-capable (must be in DRAM)");
            return ESP_ERR_INVALID_ARG;
        }
    }

    spi_slave_transaction_t trans = {
        .length = len * 8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    esp_err_t ret = spi_slave_queue_trans((spi_host_device_t)host_id, &trans, pdMS_TO_TICKS(timeout_ms));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI slave queue trans failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t spi_slave_driver_get_result(int host_id, int timeout_ms, size_t *out_trans_len)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "SPI slave driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (host_id < 1 || host_id >= SPI_SLAVE_HOST_COUNT) {
        ESP_LOGE(TAG, "Invalid SPI host ID: %d", host_id);
        return ESP_ERR_INVALID_ARG;
    }

    if (!slave_configured[host_id]) {
        ESP_LOGE(TAG, "SPI slave not configured on host %d", host_id);
        return ESP_ERR_INVALID_STATE;
    }

    spi_slave_transaction_t *ret_trans;
    esp_err_t ret = spi_slave_get_trans_result((spi_host_device_t)host_id, &ret_trans, pdMS_TO_TICKS(timeout_ms));
    if (ret != ESP_OK) {
        if (ret != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "SPI slave get trans result failed: %s", esp_err_to_name(ret));
        }
        return ret;
    }

    if (out_trans_len != NULL) {
        *out_trans_len = ret_trans->trans_len / 8;
    }

    return ESP_OK;
}

esp_err_t spi_slave_driver_transmit_receive(int host_id, const uint8_t *tx_data, uint8_t *rx_data, size_t len, int timeout_ms, size_t *out_trans_len)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "SPI slave driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (host_id < 1 || host_id >= SPI_SLAVE_HOST_COUNT) {
        ESP_LOGE(TAG, "Invalid SPI host ID: %d", host_id);
        return ESP_ERR_INVALID_ARG;
    }

    if (len == 0) {
        ESP_LOGE(TAG, "Invalid length");
        return ESP_ERR_INVALID_ARG;
    }

    if (tx_data == NULL && rx_data == NULL) {
        ESP_LOGE(TAG, "Both tx_data and rx_data cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (!slave_configured[host_id]) {
        ESP_LOGE(TAG, "SPI slave not configured on host %d", host_id);
        return ESP_ERR_INVALID_STATE;
    }

    // Validate DMA buffer capability when DMA is enabled
    if (slave_configs[host_id].dma_chan != 0) {
        if (tx_data && !esp_ptr_dma_capable(tx_data)) {
            ESP_LOGE(TAG, "TX buffer not DMA-capable (must be in DRAM)");
            return ESP_ERR_INVALID_ARG;
        }
        if (rx_data && !esp_ptr_dma_capable(rx_data)) {
            ESP_LOGE(TAG, "RX buffer not DMA-capable (must be in DRAM)");
            return ESP_ERR_INVALID_ARG;
        }
    }

    spi_slave_transaction_t trans = {
        .length = len * 8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    esp_err_t ret = spi_slave_transmit((spi_host_device_t)host_id, &trans, pdMS_TO_TICKS(timeout_ms));
    if (ret != ESP_OK) {
        if (ret != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "SPI slave transmit failed: %s", esp_err_to_name(ret));
        }
        return ret;
    }

    if (out_trans_len != NULL) {
        *out_trans_len = trans.trans_len / 8;
    }

    return ESP_OK;
}

bool spi_slave_driver_is_configured(int host_id)
{
    if (host_id < 1 || host_id >= SPI_SLAVE_HOST_COUNT) {
        return false;
    }
    return slave_configured[host_id];
}

esp_err_t spi_slave_driver_deinit(void)
{
    if (!driver_initialized) {
        ESP_LOGW(TAG, "SPI slave driver not initialized");
        return ESP_OK;
    }

    if (driver_mutex != NULL) {
        if (xSemaphoreTake(driver_mutex, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to take mutex during deinit");
            return ESP_ERR_TIMEOUT;
        }
    }

    for (int host = 1; host < SPI_SLAVE_HOST_COUNT; host++) {
        if (slave_configured[host]) {
            spi_slave_free((spi_host_device_t)host);
            slave_configured[host] = false;
        }
    }

    memset(slave_configs, 0, sizeof(slave_configs));

    driver_initialized = false;

    if (driver_mutex != NULL) {
        vSemaphoreDelete(driver_mutex);
        driver_mutex = NULL;
    }

    ESP_LOGI(TAG, "SPI slave driver deinitialized");
    return ESP_OK;
}

