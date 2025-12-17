#include "spi_master_driver.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "SPI_MASTER";

static bool driver_initialized = false;
static SemaphoreHandle_t driver_mutex = NULL;
static bool bus_configured[SPI_HOST_COUNT] = {false};
static spi_master_bus_config_t bus_configs[SPI_HOST_COUNT] = {0};
static spi_master_dev_entry_t devices[SPI_HOST_COUNT][MAX_DEVICES_PER_HOST];

/**
 * @brief Find device entry by CS pin
 *        Must be called with mutex held
 */
static spi_master_dev_entry_t* find_device_entry(int host_id, int cs_io_num)
{
    for (int i = 0; i < MAX_DEVICES_PER_HOST; i++) {
        if (devices[host_id][i].in_use && devices[host_id][i].cs_pin == cs_io_num) {
            return &devices[host_id][i];
        }
    }
    return NULL;
}

/**
 * @brief Find free device slot
 *        Must be called with mutex held
 */
static spi_master_dev_entry_t* find_free_slot(int host_id)
{
    for (int i = 0; i < MAX_DEVICES_PER_HOST; i++) {
        if (!devices[host_id][i].in_use) {
            return &devices[host_id][i];
        }
    }
    return NULL;
}

/**
 * @brief Get device handle for a given host and CS pin
 *        Must be called with mutex held
 */
static esp_err_t get_device_handle(int host_id, int cs_io_num, spi_device_handle_t *out_handle)
{
    if (host_id < 0 || host_id >= SPI_HOST_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!bus_configured[host_id]) {
        ESP_LOGE(TAG, "SPI bus %d not configured", host_id);
        return ESP_ERR_INVALID_STATE;
    }

    spi_master_dev_entry_t *entry = find_device_entry(host_id, cs_io_num);
    if (entry == NULL) {
        ESP_LOGE(TAG, "SPI device not found for host %d, CS pin %d", host_id, cs_io_num);
        return ESP_ERR_NOT_FOUND;
    }

    *out_handle = entry->handle;
    return ESP_OK;
}

esp_err_t spi_master_driver_init(void)
{
    if (driver_initialized) {
        ESP_LOGW(TAG, "SPI master driver already initialized");
        return ESP_OK;
    }

    driver_mutex = xSemaphoreCreateMutex();
    if (driver_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    memset(bus_configured, 0, sizeof(bus_configured));
    memset(bus_configs, 0, sizeof(bus_configs));
    
    for (int host = 0; host < SPI_HOST_COUNT; host++) {
        for (int i = 0; i < MAX_DEVICES_PER_HOST; i++) {
            devices[host][i].cs_pin = -1;
            devices[host][i].handle = NULL;
            devices[host][i].in_use = false;
        }
    }

    driver_initialized = true;
    ESP_LOGI(TAG, "SPI master driver initialized");
    return ESP_OK;
}

esp_err_t spi_master_driver_config_bus(const spi_master_bus_config_t *config)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "SPI master driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid configuration pointer");
        return ESP_ERR_INVALID_ARG;
    }

    if (config->host_id < 0 || config->host_id >= SPI_HOST_COUNT) {
        ESP_LOGE(TAG, "Invalid SPI host ID: %d", config->host_id);
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    // If bus already configured, remove all devices first
    if (bus_configured[config->host_id]) {
        for (int i = 0; i < MAX_DEVICES_PER_HOST; i++) {
            if (devices[config->host_id][i].in_use && devices[config->host_id][i].handle != NULL) {
                spi_bus_remove_device(devices[config->host_id][i].handle);
                devices[config->host_id][i].handle = NULL;
                devices[config->host_id][i].cs_pin = -1;
                devices[config->host_id][i].in_use = false;
            }
        }
        spi_bus_free((spi_host_device_t)config->host_id);
    }

    if (config->max_transfer_sz <= 0) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGE(TAG, "max_transfer_sz must be greater than 0");
        return ESP_ERR_INVALID_ARG;
    }

    spi_bus_config_t bus_config = {
        .mosi_io_num = config->mosi_io_num,
        .miso_io_num = config->miso_io_num,
        .sclk_io_num = config->sclk_io_num,
        .quadwp_io_num = config->quadwp_io_num,
        .quadhd_io_num = config->quadhd_io_num,
        .max_transfer_sz = config->max_transfer_sz,
    };

    esp_err_t ret = spi_bus_initialize((spi_host_device_t)config->host_id, &bus_config, config->dma_chan);
    if (ret != ESP_OK) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGE(TAG, "Failed to initialize SPI bus %d: %s", config->host_id, esp_err_to_name(ret));
        return ret;
    }

    memcpy(&bus_configs[config->host_id], config, sizeof(spi_master_bus_config_t));
    bus_configured[config->host_id] = true;

    xSemaphoreGive(driver_mutex);

    ESP_LOGI(TAG, "SPI bus %d configured: MOSI=%d, MISO=%d, SCLK=%d", 
             config->host_id, config->mosi_io_num, config->miso_io_num, config->sclk_io_num);
    return ESP_OK;
}

esp_err_t spi_master_driver_add_device(const spi_master_device_config_t *config)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "SPI master driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid configuration pointer");
        return ESP_ERR_INVALID_ARG;
    }

    if (config->host_id < 0 || config->host_id >= SPI_HOST_COUNT) {
        ESP_LOGE(TAG, "Invalid SPI host ID: %d", config->host_id);
        return ESP_ERR_INVALID_ARG;
    }

    if (config->mode > 3) {
        ESP_LOGE(TAG, "Invalid SPI mode: %d (must be 0-3)", config->mode);
        return ESP_ERR_INVALID_ARG;
    }

    if (config->clock_speed_hz == 0) {
        ESP_LOGE(TAG, "Device clock speed must be specified");
        return ESP_ERR_INVALID_ARG;
    }
    if (config->clock_speed_hz > SPI_MAX_CLOCK_SPEED_HZ) {
        ESP_LOGE(TAG, "Clock speed %lu Hz exceeds maximum %d Hz", 
                 (unsigned long)config->clock_speed_hz, SPI_MAX_CLOCK_SPEED_HZ);
        return ESP_ERR_INVALID_ARG;
    }
    if (config->clock_speed_hz < SPI_MIN_CLOCK_SPEED_HZ) {
        ESP_LOGE(TAG, "Clock speed %lu Hz below minimum %d Hz", 
                 (unsigned long)config->clock_speed_hz, SPI_MIN_CLOCK_SPEED_HZ);
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    if (!bus_configured[config->host_id]) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGE(TAG, "SPI bus %d not configured", config->host_id);
        return ESP_ERR_INVALID_STATE;
    }

    spi_master_dev_entry_t *existing = find_device_entry(config->host_id, config->cs_io_num);
    if (existing != NULL) {
        spi_bus_remove_device(existing->handle);
        existing->handle = NULL;
        existing->cs_pin = -1;
        existing->in_use = false;
    }

    spi_master_dev_entry_t *slot = find_free_slot(config->host_id);
    if (slot == NULL) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGE(TAG, "No free device slots for host %d", config->host_id);
        return ESP_ERR_NO_MEM;
    }

    spi_device_interface_config_t dev_config = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = config->mode,
        .duty_cycle_pos = 128,
        .cs_ena_pretrans = config->cs_ena_pretrans,
        .cs_ena_posttrans = config->cs_ena_posttrans,
        .clock_speed_hz = config->clock_speed_hz,
        .input_delay_ns = config->input_delay_ns,
        .spics_io_num = config->cs_io_num,
        .flags = config->flags,
        .queue_size = config->queue_size > 0 ? config->queue_size : 1,
    };

    esp_err_t ret = spi_bus_add_device((spi_host_device_t)config->host_id, &dev_config, &slot->handle);
    if (ret != ESP_OK) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    slot->cs_pin = config->cs_io_num;
    slot->in_use = true;

    xSemaphoreGive(driver_mutex);

    ESP_LOGI(TAG, "SPI device added: host=%d, CS=%d, speed=%lu Hz, mode=%d", 
             config->host_id, config->cs_io_num, (unsigned long)config->clock_speed_hz, config->mode);
    return ESP_OK;
}

esp_err_t spi_master_driver_remove_device(int host_id, int cs_io_num)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "SPI master driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (host_id < 0 || host_id >= SPI_HOST_COUNT) {
        ESP_LOGE(TAG, "Invalid SPI host ID: %d", host_id);
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    spi_master_dev_entry_t *entry = find_device_entry(host_id, cs_io_num);
    if (entry == NULL) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGW(TAG, "SPI device not found: host=%d, CS=%d", host_id, cs_io_num);
        return ESP_ERR_NOT_FOUND;
    }

    esp_err_t ret = spi_bus_remove_device(entry->handle);
    entry->handle = NULL;
    entry->cs_pin = -1;
    entry->in_use = false;

    xSemaphoreGive(driver_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SPI device removed: host=%d, CS=%d", host_id, cs_io_num);
    return ESP_OK;
}

esp_err_t spi_master_driver_transmit(int host_id, int cs_io_num, const uint8_t *tx_data, size_t tx_len, int timeout_ms)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "SPI master driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (tx_data == NULL || tx_len == 0) {
        ESP_LOGE(TAG, "Invalid data pointer or length");
        return ESP_ERR_INVALID_ARG;
    }

    if (host_id < 0 || host_id >= SPI_HOST_COUNT) {
        ESP_LOGE(TAG, "Invalid SPI host ID: %d", host_id);
        return ESP_ERR_INVALID_ARG;
    }

    // Validate DMA buffer capability when DMA is enabled
    if (bus_configs[host_id].dma_chan != 0) {
        if (!esp_ptr_dma_capable(tx_data)) {
            ESP_LOGE(TAG, "TX buffer not DMA-capable (must be in DRAM)");
            return ESP_ERR_INVALID_ARG;
        }
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    spi_device_handle_t dev_handle;
    esp_err_t ret = get_device_handle(host_id, cs_io_num, &dev_handle);
    
    xSemaphoreGive(driver_mutex);

    if (ret != ESP_OK) {
        return ret;
    }

    spi_transaction_t trans = {
        .length = tx_len * 8,
        .tx_buffer = tx_data,
        .rx_buffer = NULL,
    };

    ret = spi_device_queue_trans(dev_handle, &trans, pdMS_TO_TICKS(timeout_ms));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI queue transmit failed: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_transaction_t *ret_trans;
    ret = spi_device_get_trans_result(dev_handle, &ret_trans, pdMS_TO_TICKS(timeout_ms));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI get transmit result failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t spi_master_driver_receive(int host_id, int cs_io_num, uint8_t *rx_data, size_t rx_len, int timeout_ms)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "SPI master driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (rx_data == NULL || rx_len == 0) {
        ESP_LOGE(TAG, "Invalid data pointer or length");
        return ESP_ERR_INVALID_ARG;
    }

    if (host_id < 0 || host_id >= SPI_HOST_COUNT) {
        ESP_LOGE(TAG, "Invalid SPI host ID: %d", host_id);
        return ESP_ERR_INVALID_ARG;
    }

    // Validate DMA buffer capability when DMA is enabled
    if (bus_configs[host_id].dma_chan != 0) {
        if (!esp_ptr_dma_capable(rx_data)) {
            ESP_LOGE(TAG, "RX buffer not DMA-capable (must be in DRAM)");
            return ESP_ERR_INVALID_ARG;
        }
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    spi_device_handle_t dev_handle;
    esp_err_t ret = get_device_handle(host_id, cs_io_num, &dev_handle);
    
    xSemaphoreGive(driver_mutex);

    if (ret != ESP_OK) {
        return ret;
    }

    spi_transaction_t trans = {
        .length = rx_len * 8,
        .tx_buffer = NULL,
        .rx_buffer = rx_data,
    };

    ret = spi_device_queue_trans(dev_handle, &trans, pdMS_TO_TICKS(timeout_ms));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI queue receive failed: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_transaction_t *ret_trans;
    ret = spi_device_get_trans_result(dev_handle, &ret_trans, pdMS_TO_TICKS(timeout_ms));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI get receive result failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t spi_master_driver_transmit_receive(int host_id, int cs_io_num, const uint8_t *tx_data, uint8_t *rx_data, size_t len, int timeout_ms)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "SPI master driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (len == 0) {
        ESP_LOGE(TAG, "Invalid length");
        return ESP_ERR_INVALID_ARG;
    }

    if (tx_data == NULL && rx_data == NULL) {
        ESP_LOGE(TAG, "Both tx_data and rx_data cannot be NULL");
        return ESP_ERR_INVALID_ARG;
    }

    if (host_id < 0 || host_id >= SPI_HOST_COUNT) {
        ESP_LOGE(TAG, "Invalid SPI host ID: %d", host_id);
        return ESP_ERR_INVALID_ARG;
    }

    // Validate DMA buffer capability when DMA is enabled
    if (bus_configs[host_id].dma_chan != 0) {
        if (tx_data && !esp_ptr_dma_capable(tx_data)) {
            ESP_LOGE(TAG, "TX buffer not DMA-capable (must be in DRAM)");
            return ESP_ERR_INVALID_ARG;
        }
        if (rx_data && !esp_ptr_dma_capable(rx_data)) {
            ESP_LOGE(TAG, "RX buffer not DMA-capable (must be in DRAM)");
            return ESP_ERR_INVALID_ARG;
        }
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    spi_device_handle_t dev_handle;
    esp_err_t ret = get_device_handle(host_id, cs_io_num, &dev_handle);
    
    xSemaphoreGive(driver_mutex);

    if (ret != ESP_OK) {
        return ret;
    }

    spi_transaction_t trans = {
        .length = len * 8,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    ret = spi_device_queue_trans(dev_handle, &trans, pdMS_TO_TICKS(timeout_ms));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI queue transmit-receive failed: %s", esp_err_to_name(ret));
        return ret;
    }

    spi_transaction_t *ret_trans;
    ret = spi_device_get_trans_result(dev_handle, &ret_trans, pdMS_TO_TICKS(timeout_ms));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI get transmit-receive result failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

bool spi_master_driver_is_bus_configured(int host_id)
{
    if (host_id < 0 || host_id >= SPI_HOST_COUNT) {
        return false;
    }
    return bus_configured[host_id];
}

esp_err_t spi_master_driver_deinit(void)
{
    if (!driver_initialized) {
        ESP_LOGW(TAG, "SPI master driver not initialized");
        return ESP_OK;
    }

    if (driver_mutex != NULL) {
        if (xSemaphoreTake(driver_mutex, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to take mutex during deinit");
            return ESP_ERR_TIMEOUT;
        }
    }

    for (int host = 0; host < SPI_HOST_COUNT; host++) {
        for (int i = 0; i < MAX_DEVICES_PER_HOST; i++) {
            if (devices[host][i].in_use && devices[host][i].handle != NULL) {
                spi_bus_remove_device(devices[host][i].handle);
                devices[host][i].handle = NULL;
                devices[host][i].cs_pin = -1;
                devices[host][i].in_use = false;
            }
        }
    }

    for (int i = 0; i < SPI_HOST_COUNT; i++) {
        if (bus_configured[i]) {
            spi_bus_free((spi_host_device_t)i);
        }
    }

    memset(bus_configured, 0, sizeof(bus_configured));
    memset(bus_configs, 0, sizeof(bus_configs));

    driver_initialized = false;

    if (driver_mutex != NULL) {
        vSemaphoreDelete(driver_mutex);
        driver_mutex = NULL;
    }

    ESP_LOGI(TAG, "SPI master driver deinitialized");
    return ESP_OK;
}

