# ADC Driver

ESP32 ADC (Analog-to-Digital Converter) driver abstraction layer for smart home firmware.

## Features

- Thread-safe ADC operations
- ADC1 channel configuration
- Voltage calibration support
- Raw and calibrated voltage readings
- Configurable attenuation and bit width

## Usage Example

```c
#include "adc_driver.h"

void app_main(void)
{
    // Initialize ADC driver (sets default width to 12-bit)
    adc_driver_init();

    // Optionally change ADC width (global for all channels)
    // adc_driver_set_width(ADC_BITWIDTH_12);  // Default is already 12-bit

    // Configure ADC channel
    adc_driver_channel_config_t config = {
        .channel = ADC_CHANNEL_0,       // GPIO36
        .attenuation = ADC_ATTEN_DB_12 // 0-3.3V range (ADC_ATTEN_DB_11 is deprecated)
    };
    adc_driver_config_channel(&config);

    // Calibrate ADC (optional, improves accuracy)
    adc_driver_calibrate(ADC_CHANNEL_0, 1100);  // 1100mV reference voltage

    // Read raw ADC value (0-4095 for 12-bit)
    int raw = adc_driver_get_raw(ADC_CHANNEL_0);
    ESP_LOGI("APP", "Raw ADC value: %d", raw);

    // Read calibrated voltage in millivolts
    int voltage_mv = adc_driver_get_voltage(ADC_CHANNEL_0);
    ESP_LOGI("APP", "Voltage: %d mV (%.2f V)", voltage_mv, voltage_mv / 1000.0f);

    // Configure another channel (uses same global width setting)
    adc_driver_channel_config_t config2 = {
        .channel = ADC_CHANNEL_3,      // GPIO39
        .attenuation = ADC_ATTEN_DB_6   // 0-2.2V range
    };
    adc_driver_config_channel(&config2);
}
```

## ADC Channels

ESP32 ADC1 channels and their corresponding GPIO pins:

- `ADC_CHANNEL_0` - GPIO36
- `ADC_CHANNEL_1` - GPIO37
- `ADC_CHANNEL_2` - GPIO38
- `ADC_CHANNEL_3` - GPIO39
- `ADC_CHANNEL_4` - GPIO32
- `ADC_CHANNEL_5` - GPIO33
- `ADC_CHANNEL_6` - GPIO34
- `ADC_CHANNEL_7` - GPIO35

## Attenuation Levels

- `ADC_ATTEN_DB_0` - 0-1.1V range
- `ADC_ATTEN_DB_2_5` - 0-1.5V range
- `ADC_ATTEN_DB_6` - 0-2.2V range
- `ADC_ATTEN_DB_12` - 0-3.3V range (recommended for most applications)
  - Note: `ADC_ATTEN_DB_11` is deprecated, use `ADC_ATTEN_DB_12` instead

## Bit Width Options

**Note:** ADC width is global for all ADC1 channels and should be set via `adc_driver_set_width()` before configuring channels. Default is 12-bit.

- `ADC_BITWIDTH_9` - 9-bit resolution (0-511)
- `ADC_BITWIDTH_10` - 10-bit resolution (0-1023)
- `ADC_BITWIDTH_11` - 11-bit resolution (0-2047)
- `ADC_BITWIDTH_12` - 12-bit resolution (0-4095, recommended, default)

## API Reference

See `adc_driver.h` for complete API documentation.

