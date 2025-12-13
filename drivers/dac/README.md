# DAC Driver

ESP32 DAC (Digital-to-Analog Converter) driver abstraction layer for smart home firmware.

## Features

- Thread-safe DAC operations
- DAC channel configuration and control
- 8-bit voltage output (0-255, where 255 = VDD33)
- Channel enable/disable control

## Usage Example

```c
#include "dac_driver.h"

void app_main(void)
{
    // Initialize DAC driver
    dac_driver_init();

    // Configure DAC channel
    dac_driver_channel_config_t config = {
        .channel = DAC_CHAN_0  // GPIO25
    };
    dac_driver_config_channel(&config);

    // Set DAC output voltage (0-255, where 255 = 3.3V)
    dac_driver_set_voltage(DAC_CHAN_0, 128);  // ~1.65V (50% of VDD33)

    // Get current voltage setting
    int voltage = dac_driver_get_voltage(DAC_CHAN_0);
    ESP_LOGI("APP", "DAC voltage: %d/255", voltage);

    // Disable channel
    dac_driver_disable_channel(DAC_CHAN_0);

    // Re-enable channel
    dac_driver_enable_channel(DAC_CHAN_0);

    // Configure another channel
    dac_driver_channel_config_t config2 = {
        .channel = DAC_CHAN_1  // GPIO26
    };
    dac_driver_config_channel(&config2);
    dac_driver_set_voltage(DAC_CHAN_1, 200);  // ~2.59V
}
```

## DAC Channels

ESP32 DAC channels and their corresponding GPIO pins:

- `DAC_CHAN_0` - GPIO25
- `DAC_CHAN_1` - GPIO26

## Voltage Output

The DAC provides 8-bit resolution:
- **0** = 0V (GND)
- **255** = VDD33 (typically 3.3V)
- **128** = ~1.65V (50% of VDD33)

The output voltage is calculated as: `Vout = (voltage_8bit / 255) * VDD33`

## API Reference

See `dac_driver.h` for complete API documentation.

