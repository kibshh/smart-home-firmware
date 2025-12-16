# SPI Drivers

ESP32 SPI (Serial Peripheral Interface) drivers for smart home firmware.

## Structure

```
drivers/spi/
├── master/
│   ├── spi_master_driver.h    # Master mode API
│   └── spi_master_driver.c    # Master mode implementation
├── slave/
│   ├── spi_slave_driver.h     # Slave mode API
│   └── spi_slave_driver.c     # Slave mode implementation
└── README.md
```

## Master Mode

See `master/spi_master_driver.h` for API documentation.

### Features
- Thread-safe SPI operations
- SPI bus configuration
- Multiple device support per bus (up to 8 per host)
- Transmit/receive operations with timeout
- Full-duplex transactions
- Device handle caching
- Per-device clock speed and mode configuration
- Clock speed validation (1 kHz - 80 MHz)

### Usage

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

## Slave Mode

See `slave/spi_slave_driver.h` for API documentation.

### Features
- Respond to master-initiated transactions
- Queue-based transaction handling
- Optional callbacks for setup and post-transaction events
- Full-duplex slave transactions
- DMA support for large transfers

### Usage

```c
#include "spi_slave_driver.h"

// Optional callbacks
void my_post_setup_cb(spi_slave_transaction_t *trans)
{
    // Called when transaction is ready
}

void my_post_trans_cb(spi_slave_transaction_t *trans)
{
    // Called when transaction completes
}

void app_main(void)
{
    // Initialize SPI slave driver
    spi_slave_driver_init();

    // Configure SPI slave
    spi_slave_driver_config_t slave_config = {
        .host_id = SPI2_HOST,
        .mosi_io_num = GPIO_NUM_13,
        .miso_io_num = GPIO_NUM_12,
        .sclk_io_num = GPIO_NUM_14,
        .cs_io_num = GPIO_NUM_15,
        .dma_chan = SPI_DMA_CH_AUTO,
        .queue_size = 3,
        .mode = 0,
        .flags = 0,
        .post_setup_cb = my_post_setup_cb,
        .post_trans_cb = my_post_trans_cb,
    };
    spi_slave_driver_config(&slave_config);

    // Prepare buffers (must be DMA-capable, 4-byte aligned)
    WORD_ALIGNED_ATTR uint8_t tx_data[128] = {0xAA, 0xBB};
    WORD_ALIGNED_ATTR uint8_t rx_data[128] = {0};

    // Wait for master transaction (blocking)
    size_t trans_len;
    esp_err_t ret = spi_slave_driver_transmit_receive(
        SPI2_HOST, tx_data, rx_data, sizeof(tx_data), 5000, &trans_len);

    if (ret == ESP_OK) {
        printf("Received %d bytes from master\n", trans_len);
    }

    // Cleanup
    spi_slave_driver_free(SPI2_HOST);
    spi_slave_driver_deinit();
}
```

### Asynchronous Slave Transactions

```c
// Queue multiple transactions
spi_slave_driver_queue_trans(SPI2_HOST, tx_buf1, rx_buf1, 64, 100);
spi_slave_driver_queue_trans(SPI2_HOST, tx_buf2, rx_buf2, 64, 100);

// Wait for transactions to complete
size_t len1, len2;
spi_slave_driver_get_result(SPI2_HOST, 5000, &len1);
spi_slave_driver_get_result(SPI2_HOST, 5000, &len2);
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

## Clock Speeds (Master Mode)

Common SPI clock speeds:
- `1000000` - 1 MHz (slow, reliable)
- `5000000` - 5 MHz (medium)
- `10000000` - 10 MHz (fast)
- `20000000` - 20 MHz (very fast)
- `40000000` - 40 MHz (SPI2 max)
- `80000000` - 80 MHz (SPI3 max, short traces only)

**Note**: Slave mode clock is determined by the master.

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

### Slave Functions
- `spi_slave_driver_init()` - Initialize slave driver
- `spi_slave_driver_config()` - Configure slave
- `spi_slave_driver_free()` - Free slave on host
- `spi_slave_driver_queue_trans()` - Queue transaction (non-blocking)
- `spi_slave_driver_get_result()` - Wait for transaction result
- `spi_slave_driver_transmit_receive()` - Blocking transaction
- `spi_slave_driver_is_configured()` - Check slave status
- `spi_slave_driver_deinit()` - Deinitialize slave driver
