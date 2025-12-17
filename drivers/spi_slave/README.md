# SPI Slave Driver

ESP32 SPI Slave mode driver for smart home firmware.

## Features

- Respond to master-initiated transactions
- Queue-based transaction handling
- Optional callbacks for setup and post-transaction events
- Full-duplex slave transactions
- DMA support for large transfers

## Usage

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

## Asynchronous Slave Transactions

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

### Slave Functions

- `spi_slave_driver_init()` - Initialize slave driver
- `spi_slave_driver_config()` - Configure slave
- `spi_slave_driver_free()` - Free slave on host
- `spi_slave_driver_queue_trans()` - Queue transaction (non-blocking)
- `spi_slave_driver_get_result()` - Wait for transaction result
- `spi_slave_driver_transmit_receive()` - Blocking transaction
- `spi_slave_driver_is_configured()` - Check slave status
- `spi_slave_driver_deinit()` - Deinitialize slave driver

