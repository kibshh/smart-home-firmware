# PCNT Driver (Pulse Counter)

A thread-safe driver abstraction for the ESP32 Pulse Counter (PCNT) peripheral.

## Features

- **Thread-safe**: All operations protected by mutex
- **Multiple units**: Support for up to 4 PCNT units
- **Multiple channels**: Up to 2 channels per unit
- **Quadrature decoding**: Support for rotary encoders
- **Watch points**: Trigger callbacks at specific count values
- **Glitch filtering**: Configurable noise filter
- **Edge/Level actions**: Full control over counting behavior
- **Accumulation mode**: Continue counting across overflow/underflow

## Hardware Overview

The ESP32 PCNT peripheral is designed to count pulses on GPIO pins. Each PCNT unit:
- Has a 16-bit signed counter (-32768 to 32767)
- Supports 2 independent channels
- Can count edges on one GPIO while using another as control
- Has configurable glitch filter
- Can trigger events at specific count values (watch points)

### Use Cases

- **Rotary encoders**: Quadrature decoding with direction detection
- **Frequency measurement**: Count pulses over time
- **Event counting**: Count button presses, sensor triggers
- **Position tracking**: Motor encoder feedback

## API Reference

### Initialization

```c
esp_err_t pcnt_driver_init(void);
esp_err_t pcnt_driver_deinit(void);
```

### Unit Management

```c
esp_err_t pcnt_driver_create_unit(const pcnt_driver_unit_config_t *config, 
                                   pcnt_unit_handle_t *unit_handle);
esp_err_t pcnt_driver_delete_unit(pcnt_unit_handle_t unit_handle);
```

### Channel Management

```c
esp_err_t pcnt_driver_add_channel(pcnt_unit_handle_t unit_handle, 
                                   const pcnt_driver_channel_config_t *config, 
                                   pcnt_channel_handle_t *channel_handle);
esp_err_t pcnt_driver_remove_channel(pcnt_channel_handle_t channel_handle);
```

### Unit Control

```c
esp_err_t pcnt_driver_enable(pcnt_unit_handle_t unit_handle);
esp_err_t pcnt_driver_disable(pcnt_unit_handle_t unit_handle);
esp_err_t pcnt_driver_start(pcnt_unit_handle_t unit_handle);
esp_err_t pcnt_driver_stop(pcnt_unit_handle_t unit_handle);
```

### Counter Operations

```c
esp_err_t pcnt_driver_clear_count(pcnt_unit_handle_t unit_handle);
esp_err_t pcnt_driver_get_count(pcnt_unit_handle_t unit_handle, int *count);
```

### Watch Points

```c
esp_err_t pcnt_driver_add_watch_point(pcnt_unit_handle_t unit_handle, int watch_point);
esp_err_t pcnt_driver_remove_watch_point(pcnt_unit_handle_t unit_handle, int watch_point);
esp_err_t pcnt_driver_register_callback(pcnt_unit_handle_t unit_handle, 
                                         pcnt_driver_watch_cb_t callback, 
                                         void *user_data);
```

### Configuration

```c
esp_err_t pcnt_driver_set_glitch_filter(pcnt_unit_handle_t unit_handle, uint32_t filter_ns);
uint32_t pcnt_driver_get_active_units(void);
```

## Configuration Structures

### Unit Configuration

```c
typedef struct {
    int low_limit;              // Low limit (-32768 to high_limit-1)
    int high_limit;             // High limit (low_limit+1 to 32767)
    uint32_t glitch_filter_ns;  // Glitch filter in nanoseconds (0 = disabled)
    bool accum_count;           // Accumulate across overflow/underflow
} pcnt_driver_unit_config_t;
```

### Channel Configuration

```c
typedef struct {
    gpio_num_t edge_gpio;       // GPIO for edge (pulse) signal
    gpio_num_t level_gpio;      // GPIO for level (control) signal (-1 if not used)
    
    // Edge actions (what to do on rising/falling edge)
    pcnt_channel_edge_action_t pos_edge_action;   // HOLD, INCREASE, or DECREASE
    pcnt_channel_edge_action_t neg_edge_action;   // HOLD, INCREASE, or DECREASE
    
    // Level actions (modify edge action based on level GPIO state)
    pcnt_channel_level_action_t high_level_action; // KEEP, INVERSE, or HOLD
    pcnt_channel_level_action_t low_level_action;  // KEEP, INVERSE, or HOLD
    
    bool invert_edge_input;     // Invert edge GPIO input
    bool invert_level_input;    // Invert level GPIO input
} pcnt_driver_channel_config_t;
```

### Edge Actions

| Action | Description |
|--------|-------------|
| `PCNT_CHANNEL_EDGE_ACTION_HOLD` | Don't change counter |
| `PCNT_CHANNEL_EDGE_ACTION_INCREASE` | Increment counter |
| `PCNT_CHANNEL_EDGE_ACTION_DECREASE` | Decrement counter |

### Level Actions

| Action | Description |
|--------|-------------|
| `PCNT_CHANNEL_LEVEL_ACTION_KEEP` | Keep the edge action unchanged |
| `PCNT_CHANNEL_LEVEL_ACTION_INVERSE` | Invert the edge action (increase↔decrease) |
| `PCNT_CHANNEL_LEVEL_ACTION_HOLD` | Disable counting while at this level |

## Usage Examples

### Simple Pulse Counter

```c
#include "pcnt_driver.h"

void simple_counter_example(void)
{
    // Initialize driver
    pcnt_driver_init();
    
    // Configure unit with limits
    pcnt_driver_unit_config_t unit_config = {
        .low_limit = -1000,
        .high_limit = 1000,
        .glitch_filter_ns = 1000,  // 1 microsecond filter
        .accum_count = false,
    };
    
    pcnt_unit_handle_t unit;
    pcnt_driver_create_unit(&unit_config, &unit);
    
    // Add channel to count rising edges
    pcnt_driver_channel_config_t chan_config = {
        .edge_gpio = GPIO_NUM_4,
        .level_gpio = -1,  // Not used
        .pos_edge_action = PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        .neg_edge_action = PCNT_CHANNEL_EDGE_ACTION_HOLD,
        .high_level_action = PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        .low_level_action = PCNT_CHANNEL_LEVEL_ACTION_KEEP,
    };
    
    pcnt_channel_handle_t channel;
    pcnt_driver_add_channel(unit, &chan_config, &channel);
    
    // Enable and start
    pcnt_driver_enable(unit);
    pcnt_driver_start(unit);
    
    // Read count
    int count;
    pcnt_driver_get_count(unit, &count);
    printf("Count: %d\n", count);
    
    // Cleanup
    pcnt_driver_stop(unit);
    pcnt_driver_disable(unit);
    pcnt_driver_delete_unit(unit);
    pcnt_driver_deinit();
}
```

### Rotary Encoder (Quadrature Decoding)

```c
#include "pcnt_driver.h"

#define ENCODER_A_GPIO  GPIO_NUM_4
#define ENCODER_B_GPIO  GPIO_NUM_5

void rotary_encoder_example(void)
{
    pcnt_driver_init();
    
    // Configure unit
    pcnt_driver_unit_config_t unit_config = {
        .low_limit = -1000,
        .high_limit = 1000,
        .glitch_filter_ns = 1000,
        .accum_count = true,  // Keep counting across limits
    };
    
    pcnt_unit_handle_t unit;
    pcnt_driver_create_unit(&unit_config, &unit);
    
    // Channel A: Count A edges, use B as direction control
    pcnt_driver_channel_config_t chan_a_config = {
        .edge_gpio = ENCODER_A_GPIO,
        .level_gpio = ENCODER_B_GPIO,
        .pos_edge_action = PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        .neg_edge_action = PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        .high_level_action = PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        .low_level_action = PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
    };
    
    pcnt_channel_handle_t chan_a;
    pcnt_driver_add_channel(unit, &chan_a_config, &chan_a);
    
    // Channel B: Count B edges, use A as direction control
    pcnt_driver_channel_config_t chan_b_config = {
        .edge_gpio = ENCODER_B_GPIO,
        .level_gpio = ENCODER_A_GPIO,
        .pos_edge_action = PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        .neg_edge_action = PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        .high_level_action = PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        .low_level_action = PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
    };
    
    pcnt_channel_handle_t chan_b;
    pcnt_driver_add_channel(unit, &chan_b_config, &chan_b);
    
    // Enable and start
    pcnt_driver_enable(unit);
    pcnt_driver_start(unit);
    
    // Read position
    while (1) {
        int position;
        pcnt_driver_get_count(unit, &position);
        printf("Encoder position: %d\n", position);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

### Watch Points with Callback

```c
#include "pcnt_driver.h"

// Callback function (called from ISR context!)
static bool IRAM_ATTR watch_callback(pcnt_unit_handle_t unit, 
                                      const pcnt_driver_watch_event_t *event, 
                                      void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    
    // Signal task that threshold was reached
    TaskHandle_t task = (TaskHandle_t)user_data;
    vTaskNotifyGiveFromISR(task, &high_task_wakeup);
    
    return high_task_wakeup == pdTRUE;
}

void watch_point_example(void)
{
    pcnt_driver_init();
    
    pcnt_driver_unit_config_t unit_config = {
        .low_limit = -100,
        .high_limit = 100,
        .glitch_filter_ns = 1000,
    };
    
    pcnt_unit_handle_t unit;
    pcnt_driver_create_unit(&unit_config, &unit);
    
    // Add channel
    pcnt_driver_channel_config_t chan_config = {
        .edge_gpio = GPIO_NUM_4,
        .level_gpio = -1,
        .pos_edge_action = PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        .neg_edge_action = PCNT_CHANNEL_EDGE_ACTION_HOLD,
        .high_level_action = PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        .low_level_action = PCNT_CHANNEL_LEVEL_ACTION_KEEP,
    };
    
    pcnt_channel_handle_t channel;
    pcnt_driver_add_channel(unit, &chan_config, &channel);
    
    // Add watch points
    pcnt_driver_add_watch_point(unit, 50);   // Trigger at count = 50
    pcnt_driver_add_watch_point(unit, 100);  // Trigger at count = 100 (high limit)
    pcnt_driver_add_watch_point(unit, -100); // Trigger at count = -100 (low limit)
    
    // Register callback
    pcnt_driver_register_callback(unit, watch_callback, xTaskGetCurrentTaskHandle());
    
    // Enable and start
    pcnt_driver_enable(unit);
    pcnt_driver_start(unit);
    
    // Wait for watch point events
    while (1) {
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000))) {
            int count;
            pcnt_driver_get_count(unit, &count);
            printf("Watch point triggered! Count: %d\n", count);
            
            // Reset counter
            pcnt_driver_clear_count(unit);
        }
    }
}
```

### Frequency Measurement

```c
#include "pcnt_driver.h"
#include "esp_timer.h"

void frequency_measurement_example(void)
{
    pcnt_driver_init();
    
    pcnt_driver_unit_config_t unit_config = {
        .low_limit = -32768,
        .high_limit = 32767,
        .glitch_filter_ns = 100,  // 100ns filter for high-frequency signals
        .accum_count = true,
    };
    
    pcnt_unit_handle_t unit;
    pcnt_driver_create_unit(&unit_config, &unit);
    
    pcnt_driver_channel_config_t chan_config = {
        .edge_gpio = GPIO_NUM_4,
        .level_gpio = -1,
        .pos_edge_action = PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        .neg_edge_action = PCNT_CHANNEL_EDGE_ACTION_HOLD,  // Count rising edges only
        .high_level_action = PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        .low_level_action = PCNT_CHANNEL_LEVEL_ACTION_KEEP,
    };
    
    pcnt_channel_handle_t channel;
    pcnt_driver_add_channel(unit, &chan_config, &channel);
    
    pcnt_driver_enable(unit);
    pcnt_driver_start(unit);
    
    // Measure frequency every second
    while (1) {
        pcnt_driver_clear_count(unit);
        int64_t start_time = esp_timer_get_time();
        
        vTaskDelay(pdMS_TO_TICKS(1000));  // Wait 1 second
        
        int count;
        pcnt_driver_get_count(unit, &count);
        int64_t elapsed_us = esp_timer_get_time() - start_time;
        
        // Calculate frequency
        float frequency = (float)count * 1000000.0f / (float)elapsed_us;
        printf("Frequency: %.2f Hz\n", frequency);
    }
}
```

## Thread Safety

All driver functions are protected by a mutex:
- Safe to call from multiple tasks
- Operations are atomic
- Watch point callbacks execute in ISR context (use ISR-safe functions only!)

## Best Practices

1. **Initialize once**: Call `pcnt_driver_init()` once at startup
2. **Enable before start**: Always enable unit before starting
3. **Configure before enable**: Add channels and watch points before enabling
4. **ISR callbacks**: Keep watch callbacks short, use task notifications for heavy work
5. **Glitch filter**: Use appropriate filter values for your signal (1-10μs typical)
6. **Limits**: Set appropriate counter limits to catch overflow/underflow events

## Troubleshooting

### Counter not incrementing
- Check GPIO pin connections
- Verify edge action is not `HOLD`
- Check level GPIO state if level actions are configured
- Ensure unit is both enabled AND started

### Noise/spurious counts
- Increase glitch filter value
- Add hardware filtering (RC low-pass)
- Check for proper grounding

### Overflow issues
- Use `accum_count = true` to continue counting
- Add watch points at limits to detect overflow
- Increase counter range with appropriate limits

### Callback not triggering
- Ensure watch point values are within counter limits
- Check that callback is registered before enabling unit
- Verify callback function is marked `IRAM_ATTR` if accessing flash

## Resource Usage

- **RAM**: ~200 bytes per unit (tracking structure)
- **CPU**: Minimal (hardware peripheral does counting)
- **GPIO**: 1-2 pins per channel

## Limitations

- Maximum 4 units (ESP32 hardware limit)
- Maximum 2 channels per unit
- 16-bit signed counter range (-32768 to 32767)
- Maximum 4 watch points per unit (driver limit)

