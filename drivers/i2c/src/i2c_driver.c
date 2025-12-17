#include "i2c_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "I2C_DRIVER";
static bool driver_initialized = false;
static SemaphoreHandle_t driver_mutex = NULL;
static i2c_master_bus_handle_t i2c_bus_handles[I2C_PORT_COUNT] = {NULL};
static bool bus_configured[I2C_PORT_COUNT] = {false};
// Store bus config for recovery
static i2c_driver_bus_config_t bus_configs[I2C_PORT_COUNT] = {0};
// Cache device handles: [port][address] - supports both 7-bit (0-127) and 10-bit (0-1023)
static i2c_master_dev_handle_t i2c_device_handles[I2C_PORT_COUNT][MAX_I2C_ADDRESS + 1] = {{NULL}};

/**
 * @brief Get or create device handle for a given port and address
 *        Must be called with mutex held
 * 
 * @param port I2C port number
 * @param device_addr I2C device address (7-bit: 0-127, 10-bit: 0-1023)
 * @param addr_length Address length (I2C_ADDR_BIT_LEN_7 or I2C_ADDR_BIT_LEN_10)
 * @param out_handle Pointer to store the device handle
 * @return esp_err_t ESP_OK on success
 */
static esp_err_t get_or_create_device_handle(int port, uint16_t device_addr, i2c_addr_bit_len_t addr_length, i2c_master_dev_handle_t *out_handle)
{
    if (port >= I2C_PORT_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    // Validate address range based on address length
    if (addr_length == I2C_ADDR_BIT_LEN_7) {
        if (device_addr > MAX_I2C_ADDRESS_7BIT) {
            return ESP_ERR_INVALID_ARG;
        }
    } else if (addr_length == I2C_ADDR_BIT_LEN_10) {
        if (device_addr > MAX_I2C_ADDRESS_10BIT) {
            return ESP_ERR_INVALID_ARG;
        }
    } else {
        return ESP_ERR_INVALID_ARG;
    }

    if (!bus_configured[port] || i2c_bus_handles[port] == NULL) {
        ESP_LOGE(TAG, "I2C bus %d not configured", port);
        return ESP_ERR_INVALID_STATE;
    }

    // Check if handle already exists
    if (i2c_device_handles[port][device_addr] != NULL) {
        *out_handle = i2c_device_handles[port][device_addr];
        return ESP_OK;
    }

    // Create new device handle with clock speed from bus config
    i2c_device_config_t dev_config = {
        .dev_addr_length = addr_length,
        .device_address = device_addr,
        .scl_speed_hz = bus_configs[port].clk_speed,
    };

    esp_err_t ret = i2c_master_bus_add_device(i2c_bus_handles[port], &dev_config, &i2c_device_handles[port][device_addr]);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device at address 0x%03X (%d-bit): %s", device_addr, 
                 (addr_length == I2C_ADDR_BIT_LEN_7) ? 7 : 10, esp_err_to_name(ret));
        return ret;
    }

    *out_handle = i2c_device_handles[port][device_addr];
    return ESP_OK;
}

/**
 * @brief Toggle SCL to release stuck slaves
 *        This generates clock pulses to free slaves holding SDA low
 * 
 * @param scl_io SCL GPIO pin
 * @param sda_io SDA GPIO pin
 */
static void i2c_bus_clear(int scl_io, int sda_io)
{
    // Configure SCL as output, SDA as input
    gpio_set_direction(scl_io, GPIO_MODE_OUTPUT_OD);
    gpio_set_direction(sda_io, GPIO_MODE_INPUT);
    gpio_set_level(scl_io, 1);

    // Toggle SCL up to 9 times to release any stuck slave
    for (int i = 0; i < 9; i++) {
        // Check if SDA is released (high)
        if (gpio_get_level(sda_io) == 1) {
            break;
        }
        // Toggle SCL
        gpio_set_level(scl_io, 0);
        ets_delay_us(5);
        gpio_set_level(scl_io, 1);
        ets_delay_us(5);
    }

    // Generate STOP condition
    gpio_set_direction(sda_io, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(sda_io, 0);
    ets_delay_us(5);
    gpio_set_level(scl_io, 1);
    ets_delay_us(5);
    gpio_set_level(sda_io, 1);
    ets_delay_us(5);
}

esp_err_t i2c_driver_init(void)
{
    if (driver_initialized) {
        ESP_LOGW(TAG, "I2C driver already initialized");
        return ESP_OK;
    }

    // Create mutex for thread-safe state access
    driver_mutex = xSemaphoreCreateMutex();
    if (driver_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    // Initialize state tracking
    memset(i2c_bus_handles, 0, sizeof(i2c_bus_handles));
    memset(bus_configured, 0, sizeof(bus_configured));
    memset(bus_configs, 0, sizeof(bus_configs));
    memset(i2c_device_handles, 0, sizeof(i2c_device_handles));

    driver_initialized = true;
    ESP_LOGI(TAG, "I2C driver initialized");
    return ESP_OK;
}

esp_err_t i2c_driver_config_bus(const i2c_driver_bus_config_t *config)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "I2C driver not initialized. Call i2c_driver_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid configuration pointer");
        return ESP_ERR_INVALID_ARG;
    }

    if (config->port >= I2C_PORT_COUNT) {
        ESP_LOGE(TAG, "Invalid I2C port: %d", config->port);
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    // If bus already configured, delete all device handles first, then delete bus
    if (i2c_bus_handles[config->port] != NULL) {
        // Delete all device handles for this port
        for (int addr = 0; addr <= MAX_I2C_ADDRESS; addr++) {
            if (i2c_device_handles[config->port][addr] != NULL) {
                i2c_master_bus_rm_device(i2c_device_handles[config->port][addr]);
                i2c_device_handles[config->port][addr] = NULL;
            }
        }
        i2c_del_master_bus(i2c_bus_handles[config->port]);
        i2c_bus_handles[config->port] = NULL;
    }

    // Clear the bus before configuring (release any stuck slaves)
    i2c_bus_clear(config->scl_io_num, config->sda_io_num);

    // Configure I2C bus
    // Note: ESP-IDF's enable_internal_pullup flag controls both SDA and SCL pullups together.
    // We only enable internal pullups if both are explicitly requested.
    // Note: Clock speed is set per-device, not per-bus in the new ESP-IDF API
    i2c_master_bus_config_t bus_config = {
        .i2c_port = config->port,
        .sda_io_num = config->sda_io_num,
        .scl_io_num = config->scl_io_num,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = config->sda_pullup_en && config->scl_pullup_en,
        },
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &i2c_bus_handles[config->port]);
    if (ret != ESP_OK) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGE(TAG, "Failed to create I2C bus %d: %s", config->port, esp_err_to_name(ret));
        return ret;
    }

    // Store config for potential recovery
    memcpy(&bus_configs[config->port], config, sizeof(i2c_driver_bus_config_t));
    bus_configured[config->port] = true;

    xSemaphoreGive(driver_mutex);

    ESP_LOGI(TAG, "I2C bus %d configured: SDA=%d, SCL=%d, Speed=%lu Hz", 
             config->port, config->sda_io_num, config->scl_io_num, config->clk_speed);
    return ESP_OK;
}

esp_err_t i2c_driver_reset_bus(int port)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "I2C driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (port >= I2C_PORT_COUNT) {
        ESP_LOGE(TAG, "Invalid I2C port: %d", port);
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    if (!bus_configured[port]) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGE(TAG, "I2C bus %d not configured", port);
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGW(TAG, "Resetting I2C bus %d", port);

    // Delete all device handles for this port
    for (int addr = 0; addr <= MAX_I2C_ADDRESS; addr++) {
        if (i2c_device_handles[port][addr] != NULL) {
            i2c_master_bus_rm_device(i2c_device_handles[port][addr]);
            i2c_device_handles[port][addr] = NULL;
        }
    }

    // Delete bus
    if (i2c_bus_handles[port] != NULL) {
        i2c_del_master_bus(i2c_bus_handles[port]);
        i2c_bus_handles[port] = NULL;
    }

    // Clear the bus (toggle SCL to release stuck slaves)
    i2c_bus_clear(bus_configs[port].scl_io_num, bus_configs[port].sda_io_num);

    // Recreate bus with stored config
    // Note: Clock speed is set per-device, not per-bus in the new ESP-IDF API
    i2c_master_bus_config_t bus_config = {
        .i2c_port = port,
        .sda_io_num = bus_configs[port].sda_io_num,
        .scl_io_num = bus_configs[port].scl_io_num,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = bus_configs[port].sda_pullup_en && bus_configs[port].scl_pullup_en,
        },
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &i2c_bus_handles[port]);
    
    xSemaphoreGive(driver_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to recreate I2C bus %d: %s", port, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C bus %d reset complete", port);
    return ESP_OK;
}

esp_err_t i2c_driver_write(int port, uint16_t device_addr, i2c_addr_bit_len_t addr_length, const uint8_t *data, size_t data_len, int timeout_ms)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "I2C driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (port >= I2C_PORT_COUNT) {
        ESP_LOGE(TAG, "Invalid I2C port: %d", port);
        return ESP_ERR_INVALID_ARG;
    }

    if (data == NULL || data_len == 0) {
        ESP_LOGE(TAG, "Invalid data pointer or length");
        return ESP_ERR_INVALID_ARG;
    }

    // Lock only to get/create device handle
    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    i2c_master_dev_handle_t dev_handle;
    esp_err_t ret = get_or_create_device_handle(port, device_addr, addr_length, &dev_handle);
    
    xSemaphoreGive(driver_mutex);

    if (ret != ESP_OK) {
        return ret;
    }

    // I2C transfer is thread-safe, no mutex needed
    ret = i2c_master_transmit(dev_handle, data, data_len, pdMS_TO_TICKS(timeout_ms));

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t i2c_driver_read(int port, uint16_t device_addr, i2c_addr_bit_len_t addr_length, uint8_t *data, size_t data_len, int timeout_ms)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "I2C driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (port >= I2C_PORT_COUNT) {
        ESP_LOGE(TAG, "Invalid I2C port: %d", port);
        return ESP_ERR_INVALID_ARG;
    }

    if (data == NULL || data_len == 0) {
        ESP_LOGE(TAG, "Invalid data pointer or length");
        return ESP_ERR_INVALID_ARG;
    }

    // Lock only to get/create device handle
    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    i2c_master_dev_handle_t dev_handle;
    esp_err_t ret = get_or_create_device_handle(port, device_addr, addr_length, &dev_handle);
    
    xSemaphoreGive(driver_mutex);

    if (ret != ESP_OK) {
        return ret;
    }

    // I2C transfer is thread-safe, no mutex needed
    // Note: This function reads raw data without writing a register address first.
    // For most sensors, use i2c_driver_read_reg() instead.
    ret = i2c_master_receive(dev_handle, data, data_len, pdMS_TO_TICKS(timeout_ms));

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t i2c_driver_write_reg(int port, uint16_t device_addr, i2c_addr_bit_len_t addr_length, uint8_t reg_addr, const uint8_t *data, size_t data_len, int timeout_ms)
{
    if (data == NULL || data_len == 0) {
        ESP_LOGE(TAG, "Invalid data pointer or length");
        return ESP_ERR_INVALID_ARG;
    }

    // Allocate buffer for register address + data
    uint8_t *write_buf = malloc(data_len + 1);
    if (write_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory");
        return ESP_ERR_NO_MEM;
    }

    write_buf[0] = reg_addr;
    memcpy(&write_buf[1], data, data_len);

    esp_err_t ret = i2c_driver_write(port, device_addr, addr_length, write_buf, data_len + 1, timeout_ms);

    free(write_buf);
    return ret;
}

esp_err_t i2c_driver_read_reg(int port, uint16_t device_addr, i2c_addr_bit_len_t addr_length, uint8_t reg_addr, uint8_t *data, size_t data_len, int timeout_ms)
{
    return i2c_driver_write_read(port, device_addr, addr_length, &reg_addr, 1, data, data_len, timeout_ms);
}

esp_err_t i2c_driver_write_read(int port, uint16_t device_addr, i2c_addr_bit_len_t addr_length, const uint8_t *write_data, size_t write_len, uint8_t *read_data, size_t read_len, int timeout_ms)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "I2C driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (port >= I2C_PORT_COUNT) {
        ESP_LOGE(TAG, "Invalid I2C port: %d", port);
        return ESP_ERR_INVALID_ARG;
    }

    if ((write_data == NULL && write_len > 0) || (read_data == NULL && read_len > 0) || (write_len == 0 && read_len == 0)) {
        ESP_LOGE(TAG, "Invalid data pointer or length");
        return ESP_ERR_INVALID_ARG;
    }

    // Lock only to get/create device handle
    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    i2c_master_dev_handle_t dev_handle;
    esp_err_t ret = get_or_create_device_handle(port, device_addr, addr_length, &dev_handle);
    
    xSemaphoreGive(driver_mutex);

    if (ret != ESP_OK) {
        return ret;
    }

    // I2C transfer is thread-safe, no mutex needed
    ret = i2c_master_transmit_receive(dev_handle, write_data, write_len, read_data, read_len, pdMS_TO_TICKS(timeout_ms));

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write-read failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t i2c_driver_probe(int port, uint16_t device_addr, i2c_addr_bit_len_t addr_length, int timeout_ms)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "I2C driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (port >= I2C_PORT_COUNT) {
        ESP_LOGE(TAG, "Invalid I2C port: %d", port);
        return ESP_ERR_INVALID_ARG;
    }

    // Lock only to get/create device handle
    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    i2c_master_dev_handle_t dev_handle;
    esp_err_t ret = get_or_create_device_handle(port, device_addr, addr_length, &dev_handle);
    
    xSemaphoreGive(driver_mutex);

    if (ret != ESP_OK) {
        return ret;
    }

    // Probe by attempting a write with 0 bytes (device should ACK address if present)
    // This works for both 7-bit and 10-bit addresses
    ret = i2c_master_transmit(dev_handle, NULL, 0, pdMS_TO_TICKS(timeout_ms));
    
    // ESP_ERR_NOT_FOUND means device didn't ACK (not present)
    // ESP_OK means device ACKed (present)
    return ret;
}

esp_err_t i2c_driver_deinit(void)
{
    if (!driver_initialized) {
        ESP_LOGW(TAG, "I2C driver not initialized");
        return ESP_OK;
    }

    // Take mutex to prevent race conditions
    if (driver_mutex != NULL) {
        if (xSemaphoreTake(driver_mutex, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to take mutex during deinit");
            return ESP_ERR_TIMEOUT;
        }
    }

    // Delete all device handles first
    for (int port = 0; port < I2C_PORT_COUNT; port++) {
        for (int addr = 0; addr <= MAX_I2C_ADDRESS; addr++) {
            if (i2c_device_handles[port][addr] != NULL) {
                i2c_master_bus_rm_device(i2c_device_handles[port][addr]);
                i2c_device_handles[port][addr] = NULL;
            }
        }
    }

    // Delete all I2C bus handles
    for (int i = 0; i < I2C_PORT_COUNT; i++) {
        if (i2c_bus_handles[i] != NULL) {
            i2c_del_master_bus(i2c_bus_handles[i]);
            i2c_bus_handles[i] = NULL;
        }
    }

    memset(bus_configured, 0, sizeof(bus_configured));
    memset(bus_configs, 0, sizeof(bus_configs));

    driver_initialized = false;

    // Delete mutex last
    if (driver_mutex != NULL) {
        vSemaphoreDelete(driver_mutex);
        driver_mutex = NULL;
    }

    ESP_LOGI(TAG, "I2C driver deinitialized");
    return ESP_OK;
}
