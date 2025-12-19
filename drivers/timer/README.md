# Timer Driver

ESP32 General Purpose Timer (GPTimer) driver abstraction layer for smart home firmware.

## Features

- Thread-safe timer operations
- Multiple independent timers support
- Configurable clock source (APB or XTAL)
- Count up or count down modes
- Configurable resolution (1 Hz to 80 MHz)
- Alarm functionality with callbacks
- Periodic and one-shot alarm modes
- Active timer tracking and leak prevention

## Usage Example

### Basic Timer

```c
#include "timer_driver.h"

void app_main(void)
{
    // Initialize timer driver
    timer_driver_init();

    // Create a timer
    timer_driver_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_APB,      // Use APB clock (80 MHz)
        .direction = GPTIMER_COUNT_UP,       // Count up
        .resolution_hz = 1000000,            // 1 MHz resolution
    };
    
    gptimer_handle_t timer_handle;
    timer_driver_create(&timer_config, &timer_handle);

    // Enable and start timer
    timer_driver_enable(timer_handle);
    timer_driver_start(timer_handle);

    // Read timer count
    uint64_t count;
    timer_driver_get_raw_count(timer_handle, &count);
    printf("Timer count: %llu\n", count);

    // Stop and disable timer
    timer_driver_stop(timer_handle);
    timer_driver_disable(timer_handle);

    // Delete timer
    timer_driver_delete(timer_handle);
    
    // Deinitialize driver
    timer_driver_deinit();
}
```

### Periodic Alarm with Callback

```c
#include "timer_driver.h"

// Alarm callback function
bool timer_alarm_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    static int count = 0;
    count++;
    
    printf("Alarm triggered! Count: %d\n", count);
    
    // Return true to continue timer, false to stop
    if (count >= 10) {
        return false;  // Stop after 10 alarms
    }
    return true;
}

void app_main(void)
{
    timer_driver_init();

    // Create timer with 1 MHz resolution
    timer_driver_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_APB,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,  // 1 MHz = 1 microsecond per count
    };
    
    gptimer_handle_t timer_handle;
    timer_driver_create(&timer_config, &timer_handle);

    // Configure alarm: trigger every 1 second (1,000,000 counts at 1 MHz)
    timer_driver_alarm_config_t alarm_config = {
        .alarm_count = 1000000,              // 1 second at 1 MHz
        .reload_count_on_alarm = true,        // Periodic alarm
        .callback = timer_alarm_callback,
        .user_data = NULL,
    };
    timer_driver_set_alarm(timer_handle, &alarm_config);

    // Enable and start timer
    timer_driver_enable(timer_handle);
    timer_driver_start(timer_handle);

    // Wait for alarms (in real application, do other work)
    vTaskDelay(pdMS_TO_TICKS(12000));  // Wait 12 seconds

    // Cleanup
    timer_driver_stop(timer_handle);
    timer_driver_disable(timer_handle);
    timer_driver_delete(timer_handle);
    timer_driver_deinit();
}
```

### One-Shot Alarm

```c
// One-shot alarm (no reload)
timer_driver_alarm_config_t alarm_config = {
    .alarm_count = 5000000,              // 5 seconds at 1 MHz
    .reload_count_on_alarm = false,       // One-shot (no reload)
    .callback = my_callback,
    .user_data = NULL,
};
timer_driver_set_alarm(timer_handle, &alarm_config);
```

### Multiple Timers

```c
gptimer_handle_t timer1, timer2;

// Create first timer
timer_driver_config_t config1 = {
    .clk_src = GPTIMER_CLK_SRC_APB,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 1000000,
};
timer_driver_create(&config1, &timer1);

// Create second timer with different resolution
timer_driver_config_t config2 = {
    .clk_src = GPTIMER_CLK_SRC_APB,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 1000,  // 1 kHz = 1 millisecond per count
};
timer_driver_create(&config2, &timer2);

// Both timers can run independently
timer_driver_enable(timer1);
timer_driver_enable(timer2);
timer_driver_start(timer1);
timer_driver_start(timer2);
```

## Clock Sources

- **`GPTIMER_CLK_SRC_APB`** - APB clock (80 MHz on ESP32). Recommended for most applications.
- **`GPTIMER_CLK_SRC_XTAL`** - Crystal oscillator (40 MHz). Use for lower power or when APB frequency changes.

## Resolution

Timer resolution determines the precision of timing operations:

- **1 Hz** - 1 second per count (coarse timing)
- **1 kHz** - 1 millisecond per count
- **1 MHz** - 1 microsecond per count (recommended for most applications)
- **10 MHz** - 100 nanoseconds per count (high precision)
- **80 MHz** - Maximum resolution (APB clock frequency)

**Example calculations:**
- 1 second at 1 MHz = `1,000,000` counts
- 100 milliseconds at 1 MHz = `100,000` counts
- 1 second at 1 kHz = `1,000` counts

## Count Direction

- **`GPTIMER_COUNT_UP`** - Count from 0 upward (most common)
- **`GPTIMER_COUNT_DOWN`** - Count downward from a set value

## Alarm Modes

### Periodic Alarm
```c
.reload_count_on_alarm = true  // Timer reloads and continues
```
The alarm triggers repeatedly at the specified interval. Useful for periodic tasks.

### One-Shot Alarm
```c
.reload_count_on_alarm = false  // Alarm fires once
```
The alarm triggers once and the timer continues counting. Useful for timeouts or delayed actions.

## Callback Function

The alarm callback has the following signature:
```c
bool callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data);
```

- **Return `true`** - Continue the timer (alarm will trigger again if periodic)
- **Return `false`** - Stop the timer (no more alarms)

**Note:** Callbacks run in interrupt context. Keep them short and avoid blocking operations.

## Common Use Cases

### Precise Delay
```c
// Wait for exactly 500 milliseconds
timer_driver_config_t config = {
    .clk_src = GPTIMER_CLK_SRC_APB,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 1000000,  // 1 MHz
};
gptimer_handle_t timer;
timer_driver_create(&config, &timer);

timer_driver_set_raw_count(timer, 0);
timer_driver_enable(timer);
timer_driver_start(timer);

uint64_t count;
do {
    timer_driver_get_raw_count(timer, &count);
} while (count < 500000);  // Wait for 500ms

timer_driver_stop(timer);
timer_driver_disable(timer);
timer_driver_delete(timer);
```

### Periodic Task
```c
// Execute task every 2 seconds
timer_driver_alarm_config_t alarm = {
    .alarm_count = 2000000,           // 2 seconds at 1 MHz
    .reload_count_on_alarm = true,     // Periodic
    .callback = periodic_task_callback,
    .user_data = NULL,
};
```

### Timeout Detection
```c
// Set timeout for 5 seconds
timer_driver_alarm_config_t alarm = {
    .alarm_count = 5000000,            // 5 seconds at 1 MHz
    .reload_count_on_alarm = false,     // One-shot
    .callback = timeout_callback,
    .user_data = NULL,
};
```

## API Reference

### Initialization
- `timer_driver_init()` - Initialize timer driver
- `timer_driver_deinit()` - Deinitialize timer driver

### Timer Management
- `timer_driver_create()` - Create a new timer
- `timer_driver_delete()` - Delete a timer
- `timer_driver_enable()` - Enable timer
- `timer_driver_disable()` - Disable timer
- `timer_driver_start()` - Start timer counting
- `timer_driver_stop()` - Stop timer counting

### Alarm Configuration
- `timer_driver_set_alarm()` - Configure alarm with callback

### Count Operations
- `timer_driver_get_raw_count()` - Get current count value
- `timer_driver_set_raw_count()` - Set count value

## Thread Safety

- Driver initialization/deinitialization is thread-safe
- Per-timer operations rely on ESP-IDF's GPTimer API thread safety
- Multiple timers can operate concurrently without blocking each other
- Active timer count is tracked to prevent resource leaks

## Notes

- Always delete timers before deinitializing the driver
- The driver tracks active timers and warns if deinit is called with active timers
- Timer handles are valid until explicitly deleted
- Callbacks run in interrupt context - keep them short and non-blocking
- Resolution affects maximum count value and precision

## See Also

See `timer_driver.h` for complete API documentation.

