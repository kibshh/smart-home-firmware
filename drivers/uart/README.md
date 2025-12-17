# UART Driver

ESP32 UART (Universal Asynchronous Receiver-Transmitter) driver abstraction layer for smart home firmware.

## Features

- Thread-safe UART operations
- Multiple UART port support (UART_NUM_0, UART_NUM_1, UART_NUM_2)
- Configurable baud rate, data bits, stop bits, parity
- Hardware flow control support (RTS/CTS)
- RX/TX buffer management
- Pattern detection support
- Timeout-based read/write operations

## Usage Example

```c
#include "uart_driver.h"

void app_main(void)
{
    // Initialize UART driver
    uart_driver_init();

    // Configure UART port
    uart_driver_config_t config = {
        .port_num = UART_NUM_1,
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .tx_io_num = GPIO_NUM_17,
        .rx_io_num = GPIO_NUM_16,
        .rts_io_num = -1,  // Not used
        .cts_io_num = -1,  // Not used
        .rx_buffer_size = 1024,
        .tx_buffer_size = 1024,
        .queue_size = 0,  // No event queue
        .intr_alloc_flags = 0
    };
    uart_driver_config(&config);

    // Write data
    const char *message = "Hello, UART!\n";
    uart_driver_write(UART_NUM_1, (const uint8_t *)message, strlen(message), 1000);

    // Read data
    uint8_t rx_buffer[128];
    int bytes_read = uart_driver_read(UART_NUM_1, rx_buffer, sizeof(rx_buffer) - 1, 1000);
    if (bytes_read > 0) {
        rx_buffer[bytes_read] = '\0';
        printf("Received: %s\n", rx_buffer);
    }

    // Check available data
    int available = uart_driver_get_buffered_data_len(UART_NUM_1);
    printf("Bytes available: %d\n", available);

    // Flush buffers
    uart_driver_flush_rx(UART_NUM_1);
    uart_driver_flush_tx(UART_NUM_1);

    // Change baud rate
    uart_driver_set_baudrate(UART_NUM_1, 9600);

    // Cleanup
    uart_driver_deconfig(UART_NUM_1);
    uart_driver_deinit();
}
```

## UART Ports

ESP32 has 3 UART ports:
- `UART_NUM_0` - UART0 (used by USB-to-UART bridge, typically for console/debugging)
- `UART_NUM_1` - UART1 (available for general use)
- `UART_NUM_2` - UART2 (available for general use)

**Note**: UART_NUM_0 is typically used for serial console. Avoid reconfiguring it unless necessary.

## Common Baud Rates

- `9600` - Slow, reliable (common for sensors)
- `115200` - Fast, common default (console, Bluetooth modules)
- `230400` - Very fast
- `460800` - Ultra fast
- `921600` - Maximum (may require short cables)

## Data Bits

- `UART_DATA_5_BITS` - 5 data bits
- `UART_DATA_6_BITS` - 6 data bits
- `UART_DATA_7_BITS` - 7 data bits
- `UART_DATA_8_BITS` - 8 data bits (most common)

## Stop Bits

- `UART_STOP_BITS_1` - 1 stop bit (most common)
- `UART_STOP_BITS_1_5` - 1.5 stop bits
- `UART_STOP_BITS_2` - 2 stop bits

## Parity

- `UART_PARITY_DISABLE` - No parity (most common)
- `UART_PARITY_EVEN` - Even parity
- `UART_PARITY_ODD` - Odd parity

## Flow Control

- `UART_HW_FLOWCTRL_DISABLE` - No flow control (most common)
- `UART_HW_FLOWCTRL_RTS` - RTS (Request To Send) only
- `UART_HW_FLOWCTRL_CTS` - CTS (Clear To Send) only
- `UART_HW_FLOWCTRL_CTS_RTS` - Both RTS and CTS

## Common GPIO Pins

Default UART pins for ESP32:
- **UART_NUM_0**: TX=GPIO1, RX=GPIO3 (USB-to-UART bridge)
- **UART_NUM_1**: TX=GPIO10, RX=GPIO9 (can be remapped)
- **UART_NUM_2**: TX=GPIO17, RX=GPIO16 (can be remapped)

**Note**: GPIO pins can be configured to any available GPIO.

## Buffer Sizes

Recommended buffer sizes:
- **Small (sensors)**: RX=256, TX=256
- **Medium (general use)**: RX=1024, TX=1024
- **Large (high throughput)**: RX=4096, TX=4096

## Timeout Values

- `0` - No timeout (non-blocking, returns immediately)
- `> 0` - Timeout in milliseconds
- `< 0` - Wait forever (blocking)

## Pattern Detection

Pattern detection allows detecting specific byte sequences in the RX stream:

```c
// Enable pattern detection for newline character
uart_driver_set_pattern(UART_NUM_1, '\n', 1, 10, 10, 10);

// Note: Pattern detection requires event queue (set queue_size > 0)
// For full pattern detection support, use ESP-IDF UART API directly
```

## Important Notes

- **Thread safety**: UART operations are thread-safe per port
- **Buffer management**: Use `uart_driver_get_buffered_data_len()` to check available data
- **Flush operations**: Use `uart_driver_flush_rx()` to clear RX buffer, `uart_driver_flush_tx()` to wait for TX completion
- **Baud rate changes**: Can be changed at runtime with `uart_driver_set_baudrate()`
- **UART_NUM_0**: Typically reserved for console/debugging - avoid reconfiguring unless necessary

## API Reference

See `uart_driver.h` for complete API documentation.

### Configuration Functions
- `uart_driver_init()` - Initialize driver
- `uart_driver_config()` - Configure UART port
- `uart_driver_deconfig()` - Deconfigure UART port
- `uart_driver_is_configured()` - Check if port is configured
- `uart_driver_deinit()` - Deinitialize driver

### Data Transfer Functions
- `uart_driver_write()` - Write data to UART
- `uart_driver_read()` - Read data from UART
- `uart_driver_get_buffered_data_len()` - Get available RX data length

### Buffer Management Functions
- `uart_driver_flush_rx()` - Flush RX buffer
- `uart_driver_flush_tx()` - Flush TX buffer (wait for completion)

### Configuration Functions
- `uart_driver_set_baudrate()` - Set baud rate
- `uart_driver_get_baudrate()` - Get baud rate
- `uart_driver_set_pattern()` - Enable pattern detection


