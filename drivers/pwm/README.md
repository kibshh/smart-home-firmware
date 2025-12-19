# PWM Driver

ESP32 PWM (Pulse Width Modulation) driver abstraction layer using LEDC (LED Controller) for smart home firmware.

## Features

- Thread-safe PWM operations
- Multiple independent timers (LEDC_TIMER_0 to LEDC_TIMER_3)
- Multiple channels (up to 8 channels)
- Configurable frequency and duty resolution
- Duty cycle control (raw values or percentage)
- Smooth fade transitions with accurate status tracking
- Non-destructive stop/start (remembers and restores duty cycle)
- Low-speed and high-speed modes
- Active channel/timer tracking

## Usage Example

### Basic PWM Output

```c
#include "pwm_driver.h"

void app_main(void)
{
    // Initialize PWM driver
    pwm_driver_init();

    // Configure PWM timer (1 kHz frequency, 13-bit resolution)
    pwm_driver_timer_config_t timer_config = {
        .timer_num = LEDC_TIMER_0,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,  // 8192 steps (0-8191)
        .freq_hz = 1000,                        // 1 kHz
        .clk_cfg = LEDC_AUTO_CLK,
    };
    pwm_driver_config_timer(&timer_config);

    // Configure PWM channel
    pwm_driver_channel_config_t channel_config = {
        .timer_num = LEDC_TIMER_0,
        .channel = LEDC_CHANNEL_0,
        .gpio_num = GPIO_NUM_2,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty = 4096,        // 50% duty (4096/8191)
        .hpoint = 0,         // No phase shift
        .invert = false,     // No inversion
    };
    pwm_driver_config_channel(&channel_config);

    // Start PWM output
    pwm_driver_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

    // Change duty cycle to 75%
    pwm_driver_set_duty_percent(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 75.0f);

    // Stop PWM (non-destructive: saves current duty cycle)
    pwm_driver_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);  // Output goes to 0%, but 75% is saved

    // Start again - automatically restores the saved 75% duty cycle
    pwm_driver_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);  // Restores to 75%!

    // Deinitialize
    pwm_driver_deinit();
}
```

### LED Brightness Control

```c
// Fade LED from 0% to 100% over 2 seconds
pwm_driver_fade(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 8191, 2000);

// Check if fade is still running
while (pwm_driver_is_fade_running(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0)) {
    vTaskDelay(pdMS_TO_TICKS(100));  // Wait 100ms and check again
}

// Fade back to 0%
pwm_driver_fade(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0, 2000);
```

### Motor Speed Control

```c
// Configure timer for motor control (20 kHz, 10-bit resolution)
pwm_driver_timer_config_t motor_timer = {
    .timer_num = LEDC_TIMER_1,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_10_BIT,  // 1024 steps
    .freq_hz = 20000,                       // 20 kHz (above audible range)
    .clk_cfg = LEDC_AUTO_CLK,
};
pwm_driver_config_timer(&motor_timer);

// Configure motor channel
pwm_driver_channel_config_t motor_channel = {
    .timer_num = LEDC_TIMER_1,
    .channel = LEDC_CHANNEL_1,
    .gpio_num = GPIO_NUM_4,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty = 512,  // 50% speed
    .hpoint = 0,
    .invert = false,
};
pwm_driver_config_channel(&motor_channel);
pwm_driver_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);

// Gradually increase motor speed
for (int speed = 0; speed <= 100; speed += 10) {
    pwm_driver_set_duty_percent(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, (float)speed);
    vTaskDelay(pdMS_TO_TICKS(500));
}
```

### Multiple Channels

```c
// Configure one timer for multiple channels
pwm_driver_timer_config_t timer_config = {
    .timer_num = LEDC_TIMER_0,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_13_BIT,
    .freq_hz = 5000,
    .clk_cfg = LEDC_AUTO_CLK,
};
pwm_driver_config_timer(&timer_config);

// Configure multiple channels on the same timer
pwm_driver_channel_config_t ch0 = {
    .timer_num = LEDC_TIMER_0,
    .channel = LEDC_CHANNEL_0,
    .gpio_num = GPIO_NUM_2,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty = 2048,  // 25%
    .hpoint = 0,
    .invert = false,
};
pwm_driver_config_channel(&ch0);

pwm_driver_channel_config_t ch1 = {
    .timer_num = LEDC_TIMER_0,
    .channel = LEDC_CHANNEL_1,
    .gpio_num = GPIO_NUM_4,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty = 4096,  // 50%
    .hpoint = 0,
    .invert = false,
};
pwm_driver_config_channel(&ch1);

// Start both channels
pwm_driver_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
pwm_driver_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
```

## Timer Configuration

### Duty Resolution

Common resolution options:
- **`LEDC_TIMER_1_BIT`** - 2 steps (0-1)
- **`LEDC_TIMER_8_BIT`** - 256 steps (0-255)
- **`LEDC_TIMER_10_BIT`** - 1024 steps (0-1023) - Good for motors
- **`LEDC_TIMER_13_BIT`** - 8192 steps (0-8191) - Recommended for LEDs
- **`LEDC_TIMER_14_BIT`** - 16384 steps (0-16383)
- **`LEDC_TIMER_15_BIT`** - 32768 steps (0-32767)
- **`LEDC_TIMER_16_BIT`** - 65536 steps (0-65535) - Maximum precision
- **`LEDC_TIMER_17_BIT`** - 131072 steps (0-131071)
- **`LEDC_TIMER_18_BIT`** - 262144 steps (0-262143)
- **`LEDC_TIMER_19_BIT`** - 524288 steps (0-524287)
- **`LEDC_TIMER_20_BIT`** - 1048576 steps (0-1048575)

**Note:** Higher resolution = finer control but lower maximum frequency.

### Frequency Limits

Maximum frequency depends on resolution:
- **13-bit (8192 steps)**: Up to ~5 kHz
- **10-bit (1024 steps)**: Up to ~40 kHz
- **8-bit (256 steps)**: Up to ~160 kHz

**Formula:** `max_freq = APB_CLK / (2^resolution)`

### Speed Modes

- **`LEDC_LOW_SPEED_MODE`** - Low-speed mode (recommended for most applications)
- **`LEDC_HIGH_SPEED_MODE`** - High-speed mode (for higher frequencies, limited GPIOs)

## Duty Cycle Control

### Raw Duty Values

```c
// Set duty to 4096 (50% for 13-bit resolution: 4096/8191 ≈ 50%)
pwm_driver_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 4096);

// Get current duty
uint32_t duty;
pwm_driver_get_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, &duty);
```

### Percentage Duty

```c
// Set to 75% duty cycle
pwm_driver_set_duty_percent(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 75.0f);

// Get duty as percentage
float percentage;
pwm_driver_get_duty_percent(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, &percentage);
```

**Note:** Percentage functions automatically use the exact resolution configured for the timer. The driver tracks timer resolution per channel for accurate percentage calculations.

## Frequency Control

```c
// Change frequency to 2 kHz
pwm_driver_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, 2000);

// Get current frequency
uint32_t freq;
pwm_driver_get_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, &freq);
```

**Note:** Changing frequency affects all channels using that timer.

## Fade Transitions

```c
// Fade from current duty to 100% over 1 second
pwm_driver_fade(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 8191, 1000);

// Fade is non-blocking - do other work while fading
// Check fade status
if (pwm_driver_is_fade_running(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0)) {
    printf("Fade is still in progress\n");
}

// Wait for fade to complete
while (pwm_driver_is_fade_running(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0)) {
    vTaskDelay(pdMS_TO_TICKS(50));  // Check every 50ms
}
printf("Fade completed\n");
```

**Note:** Fade uses hardware acceleration and is smooth. The fade status is tracked automatically and cleared when the fade duration elapses.

## Stop/Start Behavior

The PWM driver implements non-destructive stop/start operations:

```c
// Set duty to 50% and start
pwm_driver_set_duty_percent(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 50.0f);
pwm_driver_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);  // Output at 50%

// Stop PWM - saves the 50% duty cycle
pwm_driver_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);   // Output goes to 0%, 50% saved

// Start again - automatically restores 50%
pwm_driver_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);  // Output back at 50%!
```

This allows you to temporarily stop PWM output without losing the configured duty cycle.

## Common Use Cases

### LED Dimming

```c
// Configure for LED control
pwm_driver_timer_config_t led_timer = {
    .timer_num = LEDC_TIMER_0,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_13_BIT,  // Smooth dimming
    .freq_hz = 5000,                        // 5 kHz (above flicker threshold)
    .clk_cfg = LEDC_AUTO_CLK,
};
pwm_driver_config_timer(&led_timer);

pwm_driver_channel_config_t led_channel = {
    .timer_num = LEDC_TIMER_0,
    .channel = LEDC_CHANNEL_0,
    .gpio_num = GPIO_NUM_2,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty = 0,
    .hpoint = 0,
    .invert = false,
};
pwm_driver_config_channel(&led_channel);
pwm_driver_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

// Smooth fade on
pwm_driver_fade(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 8191, 2000);
```

### Servo Motor Control

```c
// Servos typically use 50 Hz (20 ms period)
pwm_driver_timer_config_t servo_timer = {
    .timer_num = LEDC_TIMER_0,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_14_BIT,  // High precision for servo
    .freq_hz = 50,                          // 50 Hz
    .clk_cfg = LEDC_AUTO_CLK,
};
pwm_driver_config_timer(&servo_timer);

// Servo duty: 1 ms = 0°, 1.5 ms = 90°, 2 ms = 180°
// At 50 Hz with 14-bit: 1 ms = 1638, 1.5 ms = 2458, 2 ms = 3277
pwm_driver_channel_config_t servo_channel = {
    .timer_num = LEDC_TIMER_0,
    .channel = LEDC_CHANNEL_0,
    .gpio_num = GPIO_NUM_2,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty = 2458,  // 90° position
    .hpoint = 0,
    .invert = false,
};
pwm_driver_config_channel(&servo_channel);
pwm_driver_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
```

### DC Motor Speed Control

```c
// Higher frequency for motor (20 kHz to avoid audible noise)
pwm_driver_timer_config_t motor_timer = {
    .timer_num = LEDC_TIMER_1,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_10_BIT,  // 1024 steps sufficient
    .freq_hz = 20000,                       // 20 kHz
    .clk_cfg = LEDC_AUTO_CLK,
};
pwm_driver_config_timer(&motor_timer);
```

## API Reference

### Initialization
- `pwm_driver_init()` - Initialize PWM driver
- `pwm_driver_deinit()` - Deinitialize PWM driver

### Timer Configuration
- `pwm_driver_config_timer()` - Configure PWM timer

### Channel Configuration
- `pwm_driver_config_channel()` - Configure PWM channel

### Duty Cycle Control
- `pwm_driver_set_duty()` - Set duty cycle (raw value)
- `pwm_driver_get_duty()` - Get duty cycle (raw value)
- `pwm_driver_set_duty_percent()` - Set duty cycle (percentage)
- `pwm_driver_get_duty_percent()` - Get duty cycle (percentage)

### Frequency Control
- `pwm_driver_set_freq()` - Set PWM frequency
- `pwm_driver_get_freq()` - Get PWM frequency

### Output Control
- `pwm_driver_start()` - Start PWM output (restores saved duty if previously stopped)
- `pwm_driver_stop()` - Stop PWM output (non-destructive: saves current duty cycle)

### Fade Control
- `pwm_driver_fade()` - Fade duty cycle over time
- `pwm_driver_is_fade_running()` - Check if fade is in progress (accurate tracking with automatic cleanup)

## Thread Safety

- Driver initialization/deinitialization is thread-safe
- Per-channel operations rely on ESP-IDF's LEDC API thread safety
- Multiple channels can operate concurrently without blocking each other
- Active channels/timers are tracked to prevent resource leaks

## Notes

- Always configure timer before configuring channels
- Multiple channels can share the same timer (same frequency)
- Changing timer frequency affects all channels using that timer
- Duty resolution affects maximum achievable frequency
- Fade service is installed during init and uninstalled during deinit
- The `invert` parameter in channel config is noted but may require GPIO-level configuration
- **Stop/Start behavior**: `stop()` saves the current duty cycle. `start()` automatically restores it, making stop/start non-destructive
- **Percentage duty**: Functions automatically use the exact timer resolution configured for each channel
- **Fade tracking**: Fade status is accurately tracked and automatically cleared when fade completes
- Setting duty directly interrupts any ongoing fade

## See Also

See `pwm_driver.h` for complete API documentation.

