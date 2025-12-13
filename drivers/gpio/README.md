# GPIO Driver

ESP32 GPIO driver abstraction layer for smart home firmware.

## Features

- Thread-safe GPIO operations
- Simple configuration API
- Interrupt support
- Pin level control (set/get/toggle)

## Usage Example

```c
#include "gpio_driver.h"

void app_main(void)
{
    // Initialize GPIO driver
    gpio_driver_init();

    // Configure GPIO pin as output
    gpio_driver_config_t config = {
        .pin = GPIO_NUM_2,
        .mode = GPIO_MODE_OUTPUT,
        .pull_mode = GPIO_FLOATING,
        .intr_type_enable = false,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_driver_config(&config);

    // Set pin high
    gpio_driver_set_level(GPIO_NUM_2, 1);

    // Toggle pin
    gpio_driver_toggle(GPIO_NUM_2);

    // Configure GPIO pin as input with interrupt
    gpio_driver_config_t input_config = {
        .pin = GPIO_NUM_0,
        .mode = GPIO_MODE_INPUT,
        .pull_mode = GPIO_PULLUP_ONLY,
        .intr_type_enable = true,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_driver_config(&input_config);

    // Install ISR service
    gpio_driver_install_isr(0);

    // Add ISR handler
    gpio_driver_add_isr_handler(GPIO_NUM_0, your_isr_handler, NULL);
}
```

## API Reference

See `gpio_driver.h` for complete API documentation.

