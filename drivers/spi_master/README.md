# SPI Master Driver

ESP32 SPI Master mode driver for smart home firmware.

## Features

- Thread-safe SPI operations
- SPI bus configuration
- Multiple device support per bus (up to 8 per host)
- Transmit/receive operations with timeout
- Full-duplex transactions
- Device handle caching
- Per-device clock speed and mode configuration
- Clock speed validation (1 kHz - 80 MHz)

## Usage

```c
#include "spi_master_driver.h"

void app_main(void)
{
    // Initialize SPI master driver
    spi_master_driver_init();

    // Configure SPI bus
    spi_master_bus_config_t bus_config = {
        .host_id = SPI2_HOST,
        .mosi_io_num = GPIO_NUM_13,
        .miso_io_num = GPIO_NUM_12,
        .sclk_io_num = GPIO_NUM_14,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .dma_chan = SPI_DMA_CH_AUTO,
        .max_transfer_sz = 4096,
        .queue_size = 1
    };
    spi_master_driver_config_bus(&bus_config);

    // Add SPI device
    spi_master_device_config_t device_config = {
        .host_id = SPI2_HOST,
        .cs_io_num = GPIO_NUM_15,
        .clock_speed_hz = 5000000,  // 5 MHz
        .mode = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .input_delay_ns = 0,
        .flags = 0,
        .queue_size = 1
    };
    spi_master_driver_add_device(&device_config);

    // Transmit data
    uint8_t tx_data[] = {0x01, 0x02, 0x03};
    spi_master_driver_transmit(SPI2_HOST, GPIO_NUM_15, tx_data, sizeof(tx_data), 100);

    // Receive data
    uint8_t rx_data[4];
    spi_master_driver_receive(SPI2_HOST, GPIO_NUM_15, rx_data, sizeof(rx_data), 100);

    // Full-duplex transaction
    uint8_t tx_buf[] = {0xAA, 0xBB, 0xCC};
    uint8_t rx_buf[3];
    spi_master_driver_transmit_receive(SPI2_HOST, GPIO_NUM_15, tx_buf, rx_buf, sizeof(tx_buf), 100);

    // Cleanup
    spi_master_driver_remove_device(SPI2_HOST, GPIO_NUM_15);
    spi_master_driver_deinit();
}
```

## SPI Hosts

ESP32 has 3 SPI hosts:
- `SPI1_HOST` - SPI1 (used by flash/PSRAM, **do not use**)
- `SPI2_HOST` - HSPI (recommended for general use)
- `SPI3_HOST` - VSPI (recommended for general use)

**Note**: Master and slave drivers are independent. You can use one host for master and another for slave.

## SPI Modes

SPI has 4 modes based on clock polarity (CPOL) and clock phase (CPHA):
- **Mode 0**: CPOL=0, CPHA=0 (most common)
- **Mode 1**: CPOL=0, CPHA=1
- **Mode 2**: CPOL=1, CPHA=0
- **Mode 3**: CPOL=1, CPHA=1

## Common GPIO Pins

Default SPI pins for ESP32:
- **SPI2_HOST (HSPI)**: MOSI=GPIO13, MISO=GPIO12, SCLK=GPIO14
- **SPI3_HOST (VSPI)**: MOSI=GPIO23, MISO=GPIO19, SCLK=GPIO18

## Clock Speeds

Common SPI clock speeds:
- `1000000` - 1 MHz (slow, reliable)
- `5000000` - 5 MHz (medium)
- `10000000` - 10 MHz (fast)
- `20000000` - 20 MHz (very fast)
- `40000000` - 40 MHz (SPI2 max)
- `80000000` - 80 MHz (SPI3 max, short traces only)

## DMA

- `0` - No DMA (uses CPU, limited to 64 bytes)
- `SPI_DMA_CH_AUTO` - Auto-select DMA channel (recommended)
- `1` or `2` - Specific DMA channel

### DMA Requirements

- Buffers must be in DRAM (not flash)
- 4-byte alignment recommended (use `WORD_ALIGNED_ATTR`)
- Buffer sizes should be multiples of 4 bytes

## API Reference

### Master Functions

- `spi_master_driver_init()` - Initialize master driver
- `spi_master_driver_config_bus()` - Configure SPI bus
- `spi_master_driver_add_device()` - Add device to bus
- `spi_master_driver_remove_device()` - Remove device
- `spi_master_driver_transmit()` - Transmit data
- `spi_master_driver_receive()` - Receive data
- `spi_master_driver_transmit_receive()` - Full-duplex transfer
- `spi_master_driver_is_bus_configured()` - Check bus status
- `spi_master_driver_deinit()` - Deinitialize master driver

