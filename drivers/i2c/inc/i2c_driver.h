#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_PORT_COUNT 2
#define MAX_I2C_ADDRESS_7BIT 127   // 7-bit address range (0x00-0x7F)
#define MAX_I2C_ADDRESS_10BIT 1023 // 10-bit address range (0x000-0x3FF)
#define MAX_I2C_ADDRESS MAX_I2C_ADDRESS_10BIT  // Support both 7-bit and 10-bit

/**
 * @brief I2C bus configuration structure
 */
typedef struct {
    int port;                      ///< I2C port number (0 or 1)
    int sda_io_num;                ///< SDA GPIO pin number
    int scl_io_num;                ///< SCL GPIO pin number
    uint32_t clk_speed;            ///< I2C clock speed in Hz (typically 100000 or 400000)
    bool sda_pullup_en;            ///< Enable SDA pull-up (both SDA and SCL pullups are enabled together if both flags are true)
    bool scl_pullup_en;            ///< Enable SCL pull-up (both SDA and SCL pullups are enabled together if both flags are true)
} i2c_driver_bus_config_t;

/**
 * @brief Initialize I2C driver
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_driver_init(void);

/**
 * @brief Configure I2C bus
 * 
 * @param config I2C bus configuration structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_driver_config_bus(const i2c_driver_bus_config_t *config);

/**
 * @brief Write data to I2C device
 * 
 * @param port I2C port number
 * @param device_addr I2C device address (7-bit: 0-127, 10-bit: 0-1023)
 * @param addr_length Address length (I2C_ADDR_BIT_LEN_7 or I2C_ADDR_BIT_LEN_10)
 * @param data Pointer to data buffer
 * @param data_len Length of data to write
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_driver_write(int port, uint16_t device_addr, i2c_addr_bit_len_t addr_length, const uint8_t *data, size_t data_len, int timeout_ms);

/**
 * @brief Read data from I2C device (raw read without register address)
 * 
 * @note This function performs a raw read without writing a register address first.
 *       It only works for devices that auto-increment internal register pointers
 *       or streaming devices. For most sensors, use i2c_driver_read_reg() instead.
 * 
 * @param port I2C port number
 * @param device_addr I2C device address (7-bit: 0-127, 10-bit: 0-1023)
 * @param addr_length Address length (I2C_ADDR_BIT_LEN_7 or I2C_ADDR_BIT_LEN_10)
 * @param data Pointer to data buffer
 * @param data_len Length of data to read
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_driver_read(int port, uint16_t device_addr, i2c_addr_bit_len_t addr_length, uint8_t *data, size_t data_len, int timeout_ms);

/**
 * @brief Write data to I2C device register
 * 
 * @param port I2C port number
 * @param device_addr I2C device address (7-bit: 0-127, 10-bit: 0-1023)
 * @param addr_length Address length (I2C_ADDR_BIT_LEN_7 or I2C_ADDR_BIT_LEN_10)
 * @param reg_addr Register address
 * @param data Pointer to data buffer
 * @param data_len Length of data to write
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_driver_write_reg(int port, uint16_t device_addr, i2c_addr_bit_len_t addr_length, uint8_t reg_addr, const uint8_t *data, size_t data_len, int timeout_ms);

/**
 * @brief Read data from I2C device register
 * 
 * @param port I2C port number
 * @param device_addr I2C device address (7-bit: 0-127, 10-bit: 0-1023)
 * @param addr_length Address length (I2C_ADDR_BIT_LEN_7 or I2C_ADDR_BIT_LEN_10)
 * @param reg_addr Register address
 * @param data Pointer to data buffer
 * @param data_len Length of data to read
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_driver_read_reg(int port, uint16_t device_addr, i2c_addr_bit_len_t addr_length, uint8_t reg_addr, uint8_t *data, size_t data_len, int timeout_ms);

/**
 * @brief Write then read data from I2C device (combined transaction)
 * 
 * @param port I2C port number
 * @param device_addr I2C device address (7-bit: 0-127, 10-bit: 0-1023)
 * @param addr_length Address length (I2C_ADDR_BIT_LEN_7 or I2C_ADDR_BIT_LEN_10)
 * @param write_data Pointer to write data buffer
 * @param write_len Length of data to write
 * @param read_data Pointer to read data buffer
 * @param read_len Length of data to read
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_driver_write_read(int port, uint16_t device_addr, i2c_addr_bit_len_t addr_length, const uint8_t *write_data, size_t write_len, uint8_t *read_data, size_t read_len, int timeout_ms);

/**
 * @brief Probe I2C device (check if device responds)
 * 
 * @param port I2C port number
 * @param device_addr I2C device address (7-bit: 0-127, 10-bit: 0-1023)
 * @param addr_length Address length (I2C_ADDR_BIT_LEN_7 or I2C_ADDR_BIT_LEN_10)
 * @param timeout_ms Timeout in milliseconds
 * @return esp_err_t ESP_OK if device responds
 */
esp_err_t i2c_driver_probe(int port, uint16_t device_addr, i2c_addr_bit_len_t addr_length, int timeout_ms);

/**
 * @brief Reset I2C bus
 * 
 * Use this function to recover from a stuck bus condition.
 * It will toggle SCL to release stuck slaves, delete and recreate the bus.
 * All cached device handles will be invalidated.
 * 
 * @param port I2C port number
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_driver_reset_bus(int port);

/**
 * @brief Deinitialize I2C driver
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t i2c_driver_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // I2C_DRIVER_H

