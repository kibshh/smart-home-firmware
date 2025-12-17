#include "uart_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "UART_DRIVER";

static bool driver_initialized = false;
static SemaphoreHandle_t driver_mutex = NULL;
static bool port_configured[UART_PORT_COUNT] = {false};
static QueueHandle_t event_queues[UART_PORT_COUNT] = {NULL};

/**
 * @brief Internal unlocked version of uart_driver_deconfig()
 *        Assumes mutex is already held by caller
 */
static esp_err_t uart_driver_deconfig_unlocked(uart_port_t port_num);

esp_err_t uart_driver_init(void)
{
    if (driver_initialized) {
        ESP_LOGW(TAG, "UART driver already initialized");
        return ESP_OK;
    }

    driver_mutex = xSemaphoreCreateMutex();
    if (driver_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    memset(port_configured, 0, sizeof(port_configured));

    driver_initialized = true;
    ESP_LOGI(TAG, "UART driver initialized");
    return ESP_OK;
}

esp_err_t uart_driver_config(const uart_driver_config_t *config)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "UART driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid configuration pointer");
        return ESP_ERR_INVALID_ARG;
    }

    if (config->port_num < 0 || config->port_num >= UART_PORT_COUNT) {
        ESP_LOGE(TAG, "Invalid UART port number: %d", config->port_num);
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    // If port already configured, deconfigure it first (use unlocked version since mutex is already held)
    if (port_configured[config->port_num]) {
        uart_driver_deconfig_unlocked(config->port_num);
    }

    // Create event queue if queue_size > 0 (required for pattern detection)
    QueueHandle_t event_queue = NULL;
    if (config->queue_size > 0) {
        event_queue = xQueueCreate(config->queue_size, sizeof(uart_event_t));
        if (event_queue == NULL) {
            xSemaphoreGive(driver_mutex);
            ESP_LOGE(TAG, "Failed to create UART event queue");
            return ESP_ERR_NO_MEM;
        }
        event_queues[config->port_num] = event_queue;
    } else {
        event_queues[config->port_num] = NULL;
    }

    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = config->baud_rate,
        .data_bits = config->data_bits,
        .parity = config->parity,
        .stop_bits = config->stop_bits,
        .flow_ctrl = config->flow_ctrl,
        .source_clk = config->source_clk,
    };

    esp_err_t ret = uart_driver_install(
        config->port_num,
        config->rx_buffer_size,
        config->tx_buffer_size,
        config->queue_size,
        &event_queue,  // Event queue for pattern detection (pass pointer)
        config->intr_alloc_flags
    );

    if (ret != ESP_OK) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(config->port_num, &uart_config);
    if (ret != ESP_OK) {
        if (event_queue != NULL) {
            vQueueDelete(event_queue);
            event_queues[config->port_num] = NULL;
        }
        uart_driver_delete(config->port_num);
        xSemaphoreGive(driver_mutex);
        ESP_LOGE(TAG, "Failed to configure UART parameters: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set UART pins
    ret = uart_set_pin(
        config->port_num,
        config->tx_io_num,
        config->rx_io_num,
        config->rts_io_num,
        config->cts_io_num
    );

    if (ret != ESP_OK) {
        if (event_queue != NULL) {
            vQueueDelete(event_queue);
            event_queues[config->port_num] = NULL;
        }
        uart_driver_delete(config->port_num);
        xSemaphoreGive(driver_mutex);
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }

    port_configured[config->port_num] = true;

    xSemaphoreGive(driver_mutex);

    ESP_LOGI(TAG, "UART port %d configured: baud=%d, tx=%d, rx=%d",
             config->port_num, config->baud_rate, config->tx_io_num, config->rx_io_num);
    return ESP_OK;
}

/**
 * @brief Internal unlocked version of uart_driver_deconfig()
 *        Assumes mutex is already held by caller
 */
static esp_err_t uart_driver_deconfig_unlocked(uart_port_t port_num)
{
    if (!port_configured[port_num]) {
        ESP_LOGW(TAG, "UART port %d not configured", port_num);
        return ESP_OK;
    }

    esp_err_t ret = uart_driver_delete(port_num);
    
    // Clean up event queue if it exists
    if (event_queues[port_num] != NULL) {
        vQueueDelete(event_queues[port_num]);
        event_queues[port_num] = NULL;
    }
    
    port_configured[port_num] = false;

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete UART driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "UART port %d deconfigured", port_num);
    return ESP_OK;
}

esp_err_t uart_driver_deconfig(uart_port_t port_num)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "UART driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (port_num < 0 || port_num >= UART_PORT_COUNT) {
        ESP_LOGE(TAG, "Invalid UART port number: %d", port_num);
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = uart_driver_deconfig_unlocked(port_num);

    xSemaphoreGive(driver_mutex);

    return ret;
}

int uart_driver_write(uart_port_t port_num, const uint8_t *data, size_t len, int timeout_ms)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "UART driver not initialized");
        return -1;
    }

    if (port_num < 0 || port_num >= UART_PORT_COUNT) {
        ESP_LOGE(TAG, "Invalid UART port number: %d", port_num);
        return -1;
    }

    if (data == NULL || len == 0) {
        ESP_LOGE(TAG, "Invalid data pointer or length");
        return -1;
    }

    if (!port_configured[port_num]) {
        ESP_LOGE(TAG, "UART port %d not configured", port_num);
        return -1;
    }

    TickType_t timeout = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);
    if (timeout_ms < 0) {
        timeout = portMAX_DELAY;
    }

    int bytes_written = uart_write_bytes(port_num, data, len);
    if (bytes_written < 0) {
        ESP_LOGE(TAG, "UART write failed");
        return -1;
    }

    // Wait for transmission to complete if timeout specified
    if (timeout > 0) {
        esp_err_t ret = uart_wait_tx_done(port_num, timeout);
        if (ret != ESP_OK) {
            if (ret != ESP_ERR_TIMEOUT) {
                ESP_LOGE(TAG, "UART wait TX done failed: %s", esp_err_to_name(ret));
            }
            return bytes_written;  // Return bytes written even if timeout
        }
    }

    return bytes_written;
}

int uart_driver_read(uart_port_t port_num, uint8_t *data, size_t len, int timeout_ms)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "UART driver not initialized");
        return -1;
    }

    if (port_num < 0 || port_num >= UART_PORT_COUNT) {
        ESP_LOGE(TAG, "Invalid UART port number: %d", port_num);
        return -1;
    }

    if (data == NULL || len == 0) {
        ESP_LOGE(TAG, "Invalid data pointer or length");
        return -1;
    }

    if (!port_configured[port_num]) {
        ESP_LOGE(TAG, "UART port %d not configured", port_num);
        return -1;
    }

    TickType_t timeout = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);
    if (timeout_ms < 0) {
        timeout = portMAX_DELAY;
    }

    int bytes_read = uart_read_bytes(port_num, data, len, timeout);
    if (bytes_read < 0) {
        ESP_LOGE(TAG, "UART read failed");
        return -1;
    }

    return bytes_read;
}

int uart_driver_get_buffered_data_len(uart_port_t port_num)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "UART driver not initialized");
        return -1;
    }

    if (port_num < 0 || port_num >= UART_PORT_COUNT) {
        ESP_LOGE(TAG, "Invalid UART port number: %d", port_num);
        return -1;
    }

    if (!port_configured[port_num]) {
        ESP_LOGE(TAG, "UART port %d not configured", port_num);
        return -1;
    }

    size_t buffered_size = 0;
    esp_err_t ret = uart_get_buffered_data_len(port_num, &buffered_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get buffered data length: %s", esp_err_to_name(ret));
        return -1;
    }

    return (int)buffered_size;
}

esp_err_t uart_driver_flush_rx(uart_port_t port_num)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "UART driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (port_num < 0 || port_num >= UART_PORT_COUNT) {
        ESP_LOGE(TAG, "Invalid UART port number: %d", port_num);
        return ESP_ERR_INVALID_ARG;
    }

    if (!port_configured[port_num]) {
        ESP_LOGE(TAG, "UART port %d not configured", port_num);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = uart_flush(port_num);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to flush RX buffer: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t uart_driver_flush_tx(uart_port_t port_num)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "UART driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (port_num < 0 || port_num >= UART_PORT_COUNT) {
        ESP_LOGE(TAG, "Invalid UART port number: %d", port_num);
        return ESP_ERR_INVALID_ARG;
    }

    if (!port_configured[port_num]) {
        ESP_LOGE(TAG, "UART port %d not configured", port_num);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = uart_wait_tx_done(port_num, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to flush TX buffer: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t uart_driver_set_baudrate(uart_port_t port_num, uint32_t baud_rate)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "UART driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (port_num < 0 || port_num >= UART_PORT_COUNT) {
        ESP_LOGE(TAG, "Invalid UART port number: %d", port_num);
        return ESP_ERR_INVALID_ARG;
    }

    if (!port_configured[port_num]) {
        ESP_LOGE(TAG, "UART port %d not configured", port_num);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = uart_set_baudrate(port_num, baud_rate);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set baud rate: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "UART port %d baud rate set to %lu", port_num, (unsigned long)baud_rate);
    return ESP_OK;
}

esp_err_t uart_driver_get_baudrate(uart_port_t port_num, uint32_t *baud_rate)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "UART driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (port_num < 0 || port_num >= UART_PORT_COUNT) {
        ESP_LOGE(TAG, "Invalid UART port number: %d", port_num);
        return ESP_ERR_INVALID_ARG;
    }

    if (baud_rate == NULL) {
        ESP_LOGE(TAG, "Invalid baud rate pointer");
        return ESP_ERR_INVALID_ARG;
    }

    if (!port_configured[port_num]) {
        ESP_LOGE(TAG, "UART port %d not configured", port_num);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = uart_get_baudrate(port_num, baud_rate);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get baud rate: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t uart_driver_set_pattern(uart_port_t port_num, char pattern_chr, uint8_t chr_num, int chr_tout, int post_idle, int pre_idle)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "UART driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (port_num < 0 || port_num >= UART_PORT_COUNT) {
        ESP_LOGE(TAG, "Invalid UART port number: %d", port_num);
        return ESP_ERR_INVALID_ARG;
    }

    if (!port_configured[port_num]) {
        ESP_LOGE(TAG, "UART port %d not configured", port_num);
        return ESP_ERR_INVALID_STATE;
    }

    // Pattern detection requires an event queue
    if (event_queues[port_num] == NULL) {
        ESP_LOGE(TAG, "Pattern detection requires event queue (set queue_size > 0 in config)");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = uart_enable_pattern_det_baud_intr(port_num, pattern_chr, chr_num, chr_tout, post_idle, pre_idle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set pattern detection: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "UART port %d pattern detection enabled: char=0x%02X, num=%d", port_num, pattern_chr, chr_num);
    return ESP_OK;
}

esp_err_t uart_driver_get_pattern(uart_port_t port_num, int timeout_ms)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "UART driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (port_num < 0 || port_num >= UART_PORT_COUNT) {
        ESP_LOGE(TAG, "Invalid UART port number: %d", port_num);
        return ESP_ERR_INVALID_ARG;
    }

    if (!port_configured[port_num]) {
        ESP_LOGE(TAG, "UART port %d not configured", port_num);
        return ESP_ERR_INVALID_STATE;
    }

    // Pattern detection requires an event queue
    if (event_queues[port_num] == NULL) {
        ESP_LOGE(TAG, "Pattern detection requires event queue (set queue_size > 0 in config)");
        return ESP_ERR_INVALID_STATE;
    }

    TickType_t timeout = (timeout_ms == 0) ? 0 : pdMS_TO_TICKS(timeout_ms);
    if (timeout_ms < 0) {
        timeout = portMAX_DELAY;
    }

    // Keep reading events until we get a pattern detection event or timeout
    uart_event_t event;
    TickType_t start_time = xTaskGetTickCount();
    TickType_t elapsed = 0;
    
    while (elapsed < timeout || timeout == portMAX_DELAY) {
        TickType_t remaining = (timeout == portMAX_DELAY) ? portMAX_DELAY : (timeout - elapsed);
        if (xQueueReceive(event_queues[port_num], &event, remaining) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }

        // Check if this is a pattern detection event
        if (event.type == UART_PATTERN_DET) {
            ESP_LOGD(TAG, "Pattern detected on UART port %d", port_num);
            return ESP_OK;
        }

        // Not a pattern event - log and continue waiting
        ESP_LOGD(TAG, "Received non-pattern event: type=%d, continuing to wait for pattern", event.type);
        
        if (timeout != portMAX_DELAY) {
            elapsed = xTaskGetTickCount() - start_time;
        }
    }

    return ESP_ERR_TIMEOUT;
}

bool uart_driver_is_configured(uart_port_t port_num)
{
    if (port_num < 0 || port_num >= UART_PORT_COUNT) {
        return false;
    }
    return port_configured[port_num];
}

esp_err_t uart_driver_deinit(void)
{
    if (!driver_initialized) {
        ESP_LOGW(TAG, "UART driver not initialized");
        return ESP_OK;
    }

    if (driver_mutex != NULL) {
        if (xSemaphoreTake(driver_mutex, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to take mutex during deinit");
            return ESP_ERR_TIMEOUT;
        }
    }

    // Deconfigure all ports
    for (int i = 0; i < UART_PORT_COUNT; i++) {
        if (port_configured[i]) {
            uart_driver_delete((uart_port_t)i);
            // Clean up event queue if it exists
            if (event_queues[i] != NULL) {
                vQueueDelete(event_queues[i]);
                event_queues[i] = NULL;
            }
            port_configured[i] = false;
        }
    }

    driver_initialized = false;

    if (driver_mutex != NULL) {
        vSemaphoreDelete(driver_mutex);
        driver_mutex = NULL;
    }

    ESP_LOGI(TAG, "UART driver deinitialized");
    return ESP_OK;
}

