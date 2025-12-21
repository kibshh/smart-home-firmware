# RMT Driver

ESP32 RMT (Remote Control Transceiver) driver abstraction layer for smart home firmware. Supports infrared (IR) transmission and reception, as well as other pulse-based protocols.

## Features

- **Thread-safe operations** with per-channel mutex protection
- **Multi-instance support** with context-aware APIs
- **Pre-allocated buffers** (no dynamic allocation during operation)
- **Configurable queue depths** for TX and RX operations
- **Carrier modulation** support for IR transmission
- **Filter configuration** with per-channel resolution tracking
- **Event-based reception** using FreeRTOS queues
- **Proper resource cleanup** with hardened deinit
- **Encoder reuse** for efficient transmission
- **Timeout handling** with proper status checking

## Architecture

### Context-Based Design

The driver supports both global and multi-instance usage:

- **Global instance**: Pass `NULL` as context parameter (backward compatible)
- **Multi-instance**: Create and pass explicit context for isolated driver instances

### Pre-Allocated Buffers

- Per-channel static buffers eliminate dynamic allocation
- Thread-safe buffer access with per-channel mutexes
- Maximum 512 symbols per transaction (`RMT_MAX_SYMBOLS_PER_TRANSACTION`)

### Resource Management

- Automatic cleanup of encoders, carriers, filters, and event queues
- Hardened deinit that forces deletion of all active channels
- Proper mutex cleanup in all scenarios

## Usage Examples

### Basic IR Transmission

```c
#include "rmt_driver.h"

void app_main(void)
{
    // Initialize RMT driver (global instance)
    rmt_driver_init_global();

    // Configure TX channel for IR transmission
    rmt_driver_tx_config_t tx_config = {
        .gpio_num = GPIO_NUM_4,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,        // 1 MHz resolution
        .carrier_freq_hz = 38000,        // 38 kHz carrier (IR standard)
        .carrier_duty_percent = 33.0f,   // 33% duty cycle
        .carrier_en = true,
        .carrier_level = true,
        .loop_en = false,
        .loop_count = 0,
        .mem_block_symbols = 0,          // Auto
        .trans_queue_depth = 4,          // Queue depth
    };
    
    rmt_channel_handle_t tx_handle;
    rmt_driver_config_tx(NULL, 0, &tx_config, &tx_handle);

    // Create NEC protocol pattern (9ms header + 4.5ms space)
    rmt_symbol_t symbols[] = {
        {.level0 = 1, .duration0 = 9000, .level1 = 0, .duration1 = 4500},  // Header
        // Add more symbols for data bits...
    };

    // Transmit symbols
    rmt_driver_transmit(NULL, tx_handle, symbols, sizeof(symbols)/sizeof(symbols[0]), 1000);

    // Cleanup
    rmt_driver_delete_tx(NULL, tx_handle);
    rmt_driver_deinit();
}
```

### IR Reception with Filtering

```c
#include "rmt_driver.h"

void app_main(void)
{
    // Initialize RMT driver
    rmt_driver_init_global();

    // Configure RX channel for IR reception
    rmt_driver_rx_config_t rx_config = {
        .gpio_num = GPIO_NUM_5,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,        // 1 MHz resolution
        .filter_ticks = 100,              // Filter out pulses shorter than 100 ticks
        .filter_en = true,
        .threshold = 100,                 // Minimum duration to be considered a pause/gap (ticks)
        .mem_block_symbols = 0,          // Auto
        .event_queue_depth = 4,          // Event queue depth
    };
    
    rmt_channel_handle_t rx_handle;
    rmt_driver_config_rx(NULL, 0, &rx_config, &rx_handle);

    // Receive symbols
    rmt_symbol_t symbols[64];
    size_t received = 0;
    esp_err_t ret = rmt_driver_receive(NULL, rx_handle, symbols, 64, &received, 5000);
    
    if (ret == ESP_OK) {
        ESP_LOGI("RMT", "Received %zu symbols", received);
        // Process received symbols...
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW("RMT", "Receive timeout");
    }

    // Cleanup
    rmt_driver_delete_rx(NULL, rx_handle);
    rmt_driver_deinit();
}
```

### Multi-Instance Usage

```c
#include "rmt_driver.h"

void app_main(void)
{
    // Create multiple driver instances
    rmt_driver_context_t *ctx1 = NULL;
    rmt_driver_context_t *ctx2 = NULL;
    
    rmt_driver_init(&ctx1);
    rmt_driver_init(&ctx2);

    // Configure channels in different instances
    rmt_channel_handle_t tx1, tx2;
    rmt_driver_tx_config_t config = {
        .gpio_num = GPIO_NUM_4,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .carrier_en = false,
        .trans_queue_depth = 4,
    };
    
    rmt_driver_config_tx(ctx1, 0, &config, &tx1);
    rmt_driver_config_tx(ctx2, 0, &config, &tx2);

    // Use instances independently
    rmt_symbol_t symbols[] = {{1, 1000, 0, 500}};
    rmt_driver_transmit(ctx1, tx1, symbols, 1, 1000);
    rmt_driver_transmit(ctx2, tx2, symbols, 1, 1000);

    // Cleanup
    rmt_driver_delete_tx(ctx1, tx1);
    rmt_driver_delete_tx(ctx2, tx2);
    
    // Deinitialize contexts
    rmt_driver_deinit_context(ctx1);
    rmt_driver_deinit_context(ctx2);
}
```

### Blocking Transmission

```c
// Transmit and wait for completion
rmt_symbol_t symbols[] = {
    {.level0 = 1, .duration0 = 9000, .level1 = 0, .duration1 = 4500},
};

// This will block until transmission completes or timeout
esp_err_t ret = rmt_driver_transmit_wait(NULL, tx_handle, symbols, 
                                         sizeof(symbols)/sizeof(symbols[0]), 1000);
if (ret == ESP_OK) {
    ESP_LOGI("RMT", "Transmission completed");
}
```

## Configuration Guide

### TX Configuration

- **`gpio_num`**: GPIO pin for RMT TX output
- **`clk_src`**: Clock source (`RMT_CLK_SRC_DEFAULT`, `RMT_CLK_SRC_APB`, `RMT_CLK_SRC_REF_TICK`, `RMT_CLK_SRC_XTAL`)
- **`resolution_hz`**: RMT resolution in Hz (e.g., 1000000 for 1 MHz)
- **`carrier_freq_hz`**: Carrier frequency in Hz (0 to disable, typically 38000 for IR)
- **`carrier_duty_percent`**: Carrier duty cycle (0-100, typically 33% for IR)
- **`carrier_en`**: Enable carrier modulation
- **`carrier_level`**: Carrier level (true = high, false = low)
- **`trans_queue_depth`**: Transmission queue depth (default: 4)

### RX Configuration

- **`gpio_num`**: GPIO pin for RMT RX input
- **`clk_src`**: Clock source
- **`resolution_hz`**: RMT resolution in Hz (must match TX for proper decoding)
- **`filter_ticks`**: Filter ticks (0 to disable, typically 100 for IR)
- **`filter_en`**: Enable filter
- **`threshold`**: Signal threshold in ticks (minimum duration to be considered a pause/gap)
- **`event_queue_depth`**: Event queue depth for RX completion events (default: 4)

### Resolution and Filtering

The driver stores resolution per channel and uses it for accurate filter threshold conversion:

- Filter threshold is converted from ticks to nanoseconds: `threshold_ns = (threshold_ticks * 1e9) / resolution_hz`
- Threshold represents the minimum duration to be considered a pause/gap between signals
- Resolution is automatically stored during channel configuration
- Filter settings are applied during receive operations

## Common Use Cases

### IR Remote Control

```c
// Transmit IR command
rmt_driver_tx_config_t tx_config = {
    .gpio_num = GPIO_NUM_4,
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = 1000000,
    .carrier_freq_hz = 38000,
    .carrier_duty_percent = 33.0f,
    .carrier_en = true,
    .trans_queue_depth = 4,
};

// Receive IR command
rmt_driver_rx_config_t rx_config = {
    .gpio_num = GPIO_NUM_5,
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = 1000000,  // Must match TX resolution
    .filter_ticks = 100,
    .filter_en = true,
    .threshold = 100,                 // Minimum pause/gap duration
    .event_queue_depth = 4,
};
```

### Custom Pulse Protocols

```c
// No carrier for non-IR protocols
rmt_driver_tx_config_t tx_config = {
    .gpio_num = GPIO_NUM_4,
    .clk_src = RMT_CLK_SRC_DEFAULT,
    .resolution_hz = 1000000,
    .carrier_en = false,
    .trans_queue_depth = 4,
};
```

## Protocol Examples

### NEC Protocol

```c
// NEC protocol header: 9ms high + 4.5ms low
rmt_symbol_t nec_header = {
    .level0 = 1,
    .duration0 = 9000,   // 9ms at 1 MHz = 9000 ticks
    .level1 = 0,
    .duration1 = 4500,   // 4.5ms at 1 MHz = 4500 ticks
};

// NEC logic 0: 562.5µs high + 562.5µs low
rmt_symbol_t nec_logic0 = {
    .level0 = 1,
    .duration0 = 563,
    .level1 = 0,
    .duration1 = 563,
};

// NEC logic 1: 562.5µs high + 1687.5µs low
rmt_symbol_t nec_logic1 = {
    .level0 = 1,
    .duration0 = 563,
    .level1 = 0,
    .duration1 = 1688,
};
```

## API Reference

### Initialization

- `rmt_driver_init(context)`: Initialize driver and create context
- `rmt_driver_init_global()`: Initialize global instance (backward compatible)

### Channel Configuration

- `rmt_driver_config_tx(context, channel, config, tx_handle)`: Configure TX channel
- `rmt_driver_config_rx(context, channel, config, rx_handle)`: Configure RX channel

### Transmission

- `rmt_driver_transmit(context, tx_handle, symbols, num_symbols, timeout_ms)`: Transmit symbols (non-blocking)
- `rmt_driver_transmit_wait(context, tx_handle, symbols, num_symbols, timeout_ms)`: Transmit and wait for completion

### Reception

- `rmt_driver_receive(context, rx_handle, symbols, num_symbols, received_symbols, timeout_ms)`: Receive symbols

### Channel Control

- `rmt_driver_enable(context, handle)`: Enable channel
- `rmt_driver_disable(context, handle)`: Disable channel

### Cleanup

- `rmt_driver_delete_tx(context, tx_handle)`: Delete TX channel
- `rmt_driver_delete_rx(context, rx_handle)`: Delete RX channel
- `rmt_driver_deinit()`: Deinitialize global instance
- `rmt_driver_deinit_context(context)`: Deinitialize specific context (NULL for global)

## Thread Safety

- **Per-channel mutexes**: Each channel has its own mutex protecting buffer access
- **Driver-wide mutex**: Protects driver state (channel counts, mappings)
- **ESP-IDF thread safety**: RMT API operations are thread-safe
- **Buffer protection**: TX and RX buffers are protected from concurrent access

## Resource Management

### Automatic Cleanup

- Encoders are automatically deleted when channels are deleted
- Carriers are explicitly removed during channel deletion
- Event queues are deleted when RX channels are deleted
- All channel mutexes are cleaned up during deinit

### Hardened Deinit

`rmt_driver_deinit()` performs complete cleanup:

- Waits for all ongoing transmissions to complete
- Removes all carriers
- Deletes all encoders
- Deletes all channels
- Deletes all event queues
- Frees all channel mutexes
- Frees driver context

## Performance Considerations

- **Pre-allocated buffers**: No allocation overhead during operation
- **Encoder reuse**: Encoders are created once per channel and reused
- **Event-based reception**: Deterministic receive completion using FreeRTOS queues
- **Configurable queue depths**: Adjust based on burst requirements

## Error Handling

All functions return `esp_err_t`:

- `ESP_OK`: Success
- `ESP_ERR_INVALID_STATE`: Driver not initialized
- `ESP_ERR_INVALID_ARG`: Invalid parameters
- `ESP_ERR_NO_MEM`: Memory allocation failure
- `ESP_ERR_TIMEOUT`: Operation timeout
- `ESP_ERR_NOT_FOUND`: Resource not found

Error logging uses ESP-IDF logging levels:
- `ESP_LOGE`: Critical errors
- `ESP_LOGW`: Warnings (non-critical failures)
- `ESP_LOGI`: Informational messages
- `ESP_LOGD`: Debug messages

## Limitations

- Maximum 512 symbols per transaction (`RMT_MAX_SYMBOLS_PER_TRANSACTION`)
- Maximum 8 RMT channels (`RMT_CHANNEL_MAX`)
- Filter threshold conversion uses stored resolution (must be configured)
- TX buffer unlock happens immediately after queuing (safe for copy mode, may need adjustment for DMA)

## Best Practices

1. **Match resolutions**: TX and RX should use the same resolution for accurate timing
2. **Configure filters**: Enable filtering for IR reception to reduce noise
3. **Use appropriate queue depths**: Increase for burst scenarios
4. **Clean up resources**: Always delete channels and deinit when done
5. **Handle timeouts**: Check return values and handle `ESP_ERR_TIMEOUT`
6. **Multi-instance isolation**: Use separate contexts for independent driver instances

## Troubleshooting

### Transmission Issues

- Verify carrier configuration matches receiver expectations
- Check GPIO pin configuration
- Ensure resolution matches between TX and RX
- Verify symbol timing is correct for protocol

### Reception Issues

- Enable and configure filter appropriately
- Check threshold value (too high may not detect pauses correctly, too low may cause false pauses)
- Verify event queue depth is sufficient
- Ensure resolution matches transmitter

### Resource Leaks

- Always delete channels before deinit
- Check that all mutexes are properly released
- Verify event queues are deleted

## License

This driver is part of the smart home firmware project and follows the project's license.
