# I2C Driver

ESP32 I2C (Inter-Integrated Circuit) master driver abstraction layer for smart home firmware.

## Features

- Thread-safe I2C operations
- I2C bus configuration
- Read/write operations
- Register read/write helpers
- Combined write-read transactions
- Device probing
- Bus recovery (for stuck bus conditions)
- Device handle caching for efficiency

## Usage Example

```c
#include "i2c_driver.h"

void app_main(void)
{
    // Initialize I2C driver
    i2c_driver_init();

    // Configure I2C bus
    i2c_driver_bus_config_t bus_config = {
        .port = I2C_PORT_0,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .clk_speed = 100000,        // 100kHz
        .sda_pullup_en = true,
        .scl_pullup_en = true
    };
    i2c_driver_config_bus(&bus_config);

    // Probe device (check if device responds) - 7-bit address example
    uint16_t device_addr_7bit = 0x48;  // 7-bit address: 0x48
    esp_err_t ret = i2c_driver_probe(I2C_PORT_0, device_addr_7bit, I2C_ADDR_BIT_LEN_7, 100);
    if (ret == ESP_OK) {
        ESP_LOGI("APP", "Device found at 7-bit address 0x%02X", device_addr_7bit);
    }

    // Write data to device (7-bit address)
    uint8_t write_data[] = {0x01, 0x02, 0x03};
    i2c_driver_write(I2C_PORT_0, device_addr_7bit, I2C_ADDR_BIT_LEN_7, write_data, sizeof(write_data), 100);

    // Read data from device (raw read, for streaming devices)
    uint8_t read_data[4];
    i2c_driver_read(I2C_PORT_0, device_addr_7bit, I2C_ADDR_BIT_LEN_7, read_data, sizeof(read_data), 100);

    // Write to register (most common for sensors) - 7-bit address
    uint8_t reg_addr = 0x00;
    uint8_t reg_data[] = {0xFF};
    i2c_driver_write_reg(I2C_PORT_0, device_addr_7bit, I2C_ADDR_BIT_LEN_7, reg_addr, reg_data, sizeof(reg_data), 100);

    // Read from register (most common for sensors) - 7-bit address
    uint8_t reg_value;
    i2c_driver_read_reg(I2C_PORT_0, device_addr_7bit, I2C_ADDR_BIT_LEN_7, reg_addr, &reg_value, 1, 100);

    // Combined write-read transaction - 7-bit address
    uint8_t cmd = 0x10;
    uint8_t response[2];
    i2c_driver_write_read(I2C_PORT_0, device_addr_7bit, I2C_ADDR_BIT_LEN_7, &cmd, 1, response, sizeof(response), 100);

    // 10-bit address example
    uint16_t device_addr_10bit = 0x123;  // 10-bit address: 0x123
    ret = i2c_driver_probe(I2C_PORT_0, device_addr_10bit, I2C_ADDR_BIT_LEN_10, 100);
    if (ret == ESP_OK) {
        ESP_LOGI("APP", "Device found at 10-bit address 0x%03X", device_addr_10bit);
    }
    i2c_driver_read_reg(I2C_PORT_0, device_addr_10bit, I2C_ADDR_BIT_LEN_10, reg_addr, &reg_value, 1, 100);

    // Bus recovery (if bus gets stuck)
    if (ret == ESP_ERR_TIMEOUT) {
        i2c_driver_reset_bus(I2C_PORT_0);
    }
}
```

## I2C Ports

ESP32 has 2 I2C ports:
- `I2C_PORT_0` - I2C port 0
- `I2C_PORT_1` - I2C port 1

## Common I2C GPIO Pins

Default I2C pins for ESP32:
- **I2C_PORT_0**: SDA = GPIO21, SCL = GPIO22
- **I2C_PORT_1**: SDA = GPIO25, SCL = GPIO26 (but these are also DAC pins)

## Clock Speeds

Common I2C clock speeds:
- `100000` - Standard mode (100 kHz)
- `400000` - Fast mode (400 kHz)
- `1000000` - Fast mode plus (1 MHz, not all devices support)

## Bus Recovery

If the I2C bus gets stuck (e.g., a slave is holding SDA low), use `i2c_driver_reset_bus()`:

```c
esp_err_t ret = i2c_driver_write(I2C_PORT_0, addr, data, len, 100);
if (ret == ESP_ERR_TIMEOUT || ret == ESP_FAIL) {
    ESP_LOGW("APP", "I2C bus stuck, attempting recovery...");
    i2c_driver_reset_bus(I2C_PORT_0);
    // Retry the operation
}
```

The reset function:
1. Deletes all device handles for that port
2. Deletes the bus
3. Toggles SCL up to 9 times to release stuck slaves
4. Generates a STOP condition
5. Recreates the bus with the same configuration

## Address Modes

The driver supports both 7-bit and 10-bit I2C addressing:

- **7-bit addresses**: Range 0x00-0x7F (0-127)
- **10-bit addresses**: Range 0x000-0x3FF (0-1023)

Specify the address length using `I2C_ADDR_BIT_LEN_7` or `I2C_ADDR_BIT_LEN_10` in all function calls.

## Notes

- **Thread safety**: I2C transfers are thread-safe. The mutex only protects shared state (device handle creation).
- **Device handle caching**: Device handles are created on first use and cached for efficiency. Handles are cached per port and address combination.

## API Reference

See `i2c_driver.h` for complete API documentation.
