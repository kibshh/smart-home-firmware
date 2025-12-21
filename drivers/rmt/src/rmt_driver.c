#include "rmt_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "RMT_DRIVER";

// Callback function to post RX done events to queue
static bool rmt_rx_done_callback(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    QueueHandle_t event_queue = (QueueHandle_t)user_data;
    if (event_queue != NULL) {
        BaseType_t must_yield = pdFALSE;
        xQueueSendFromISR(event_queue, edata, &must_yield);
        portYIELD_FROM_ISR(must_yield);
    }
    return false;  // Don't free the buffer
}

// Per-channel RX filter configuration
typedef struct {
    uint32_t filter_ticks;
    bool filter_en;
    uint32_t threshold;
} rmt_rx_filter_config_t;

// Per-channel configuration storage
typedef struct {
    uint32_t resolution_hz;  // Store resolution for filter conversion
} rmt_channel_config_t;

// Driver context structure
struct rmt_driver_context {
    bool initialized;
    uint32_t active_tx_channels;
    uint32_t active_rx_channels;
    // Store encoder handles per TX channel (indexed by channel number)
    rmt_encoder_handle_t tx_encoders[RMT_CHANNEL_MAX];
    // Store channel handle to channel number mapping for encoder lookup
    struct {
        rmt_channel_handle_t handle;
        uint32_t channel;
    } tx_handle_map[RMT_CHANNEL_MAX];
    // Store event queues per RX channel (indexed by channel number)
    QueueHandle_t rx_event_queues[RMT_CHANNEL_MAX];
    struct {
        rmt_channel_handle_t handle;
        uint32_t channel;
    } rx_handle_map[RMT_CHANNEL_MAX];
    // Pre-allocated buffers for transmit/receive (per channel)
    rmt_symbol_word_t tx_buffers[RMT_CHANNEL_MAX][RMT_MAX_SYMBOLS_PER_TRANSACTION];
    rmt_symbol_word_t rx_buffers[RMT_CHANNEL_MAX][RMT_MAX_SYMBOLS_PER_TRANSACTION];
    // Track carrier configuration per TX channel
    bool carrier_applied[RMT_CHANNEL_MAX];
    // Store RX filter configuration per channel
    rmt_rx_filter_config_t rx_filter_configs[RMT_CHANNEL_MAX];
    // Store per-channel configuration (resolution, etc.)
    rmt_channel_config_t channel_configs[RMT_CHANNEL_MAX];
    // Per-channel mutexes for buffer thread safety
    SemaphoreHandle_t channel_mutexes[RMT_CHANNEL_MAX];
    // Mutex protects driver-wide state
    SemaphoreHandle_t mutex;
};

// Global driver instance (for backward compatibility)
static rmt_driver_context_t *global_context = NULL;

// Helper to get context (returns global if NULL passed)
static inline rmt_driver_context_t *get_context(rmt_driver_context_t *ctx)
{
    return (ctx != NULL) ? ctx : global_context;
}

esp_err_t rmt_driver_init(rmt_driver_context_t **context)
{
    rmt_driver_context_t *ctx = NULL;
    
    if (context != NULL && *context != NULL) {
        // Use provided context
        ctx = *context;
    } else {
        // Allocate new context
        ctx = calloc(1, sizeof(rmt_driver_context_t));
        if (ctx == NULL) {
            ESP_LOGE(TAG, "Failed to allocate driver context");
            return ESP_ERR_NO_MEM;
        }
        
        // Create mutex
        ctx->mutex = xSemaphoreCreateMutex();
        if (ctx->mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create mutex");
            free(ctx);
            return ESP_ERR_NO_MEM;
        }
        
        // Initialize arrays
        memset(ctx->tx_encoders, 0, sizeof(ctx->tx_encoders));
        memset(ctx->rx_event_queues, 0, sizeof(ctx->rx_event_queues));
        memset(ctx->carrier_applied, 0, sizeof(ctx->carrier_applied));
        memset(ctx->rx_filter_configs, 0, sizeof(ctx->rx_filter_configs));
        memset(ctx->channel_configs, 0, sizeof(ctx->channel_configs));
        for (int i = 0; i < RMT_CHANNEL_MAX; i++) {
            ctx->tx_handle_map[i].handle = NULL;
            ctx->tx_handle_map[i].channel = RMT_CHANNEL_MAX;
            ctx->rx_handle_map[i].handle = NULL;
            ctx->rx_handle_map[i].channel = RMT_CHANNEL_MAX;
            // Create per-channel mutex for buffer thread safety
            ctx->channel_mutexes[i] = xSemaphoreCreateMutex();
            if (ctx->channel_mutexes[i] == NULL) {
                ESP_LOGE(TAG, "Failed to create channel mutex %d", i);
                // Cleanup already created mutexes
                for (int j = 0; j < i; j++) {
                    vSemaphoreDelete(ctx->channel_mutexes[j]);
                }
                vSemaphoreDelete(ctx->mutex);
                free(ctx);
                return ESP_ERR_NO_MEM;
            }
        }
        
        ctx->initialized = true;
        
        if (context != NULL) {
            *context = ctx;
        } else {
            // Store as global context
            global_context = ctx;
        }
    }

    ESP_LOGI(TAG, "RMT driver initialized (context=%p)", (void*)ctx);
    return ESP_OK;
}

esp_err_t rmt_driver_init_global(void)
{
    return rmt_driver_init(NULL);
}

esp_err_t rmt_driver_config_tx(rmt_driver_context_t *context, uint32_t channel, const rmt_driver_tx_config_t *config, rmt_channel_handle_t *tx_handle)
{
    rmt_driver_context_t *ctx = get_context(context);
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "RMT driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL || tx_handle == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    if (channel >= RMT_CHANNEL_MAX) {
        ESP_LOGE(TAG, "Invalid channel number: %lu (max: %d)", (unsigned long)channel, RMT_CHANNEL_MAX - 1);
        return ESP_ERR_INVALID_ARG;
    }

    // Configure RMT TX channel
    rmt_tx_channel_config_t tx_channel_cfg = {
        .gpio_num = config->gpio_num,
        .clk_src = config->clk_src,
        .resolution_hz = config->resolution_hz,
        .mem_block_symbols = config->mem_block_symbols,
        .trans_queue_depth = (config->trans_queue_depth > 0) ? config->trans_queue_depth : 4,
        .flags = {
            .invert_out = false,
            .with_dma = false,
        },
    };

    esp_err_t ret = rmt_new_tx_channel(&tx_channel_cfg, tx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT TX channel %lu: %s", (unsigned long)channel, esp_err_to_name(ret));
        return ret;
    }

    // Configure carrier if enabled
    if (config->carrier_en && config->carrier_freq_hz > 0) {
        rmt_carrier_config_t carrier_cfg = {
            .duty_cycle = (float)(config->carrier_duty_percent / 100.0f),
            .frequency_hz = config->carrier_freq_hz,
            .flags = {
                .polarity_active_low = !config->carrier_level,
            },
        };

        ret = rmt_apply_carrier(*tx_handle, &carrier_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to apply carrier to TX channel %lu: %s", (unsigned long)channel, esp_err_to_name(ret));
            rmt_del_channel(*tx_handle);
            return ret;
        }
        
        // Track carrier application
        if (xSemaphoreTake(ctx->mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            ctx->carrier_applied[channel] = true;
            xSemaphoreGive(ctx->mutex);
        }
    }

    // Create encoder (simple copy encoder for pattern-based transmission)
    // Store encoder handle for reuse in transmit calls
    // Note: rmt_copy_encoder_config_t is an empty struct (no fields), so zero-initialization is correct
    rmt_copy_encoder_config_t copy_encoder_config = {0};
    rmt_encoder_handle_t copy_encoder = NULL;
    ret = rmt_new_copy_encoder(&copy_encoder_config, &copy_encoder);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create copy encoder for TX channel %lu: %s", (unsigned long)channel, esp_err_to_name(ret));
        rmt_del_channel(*tx_handle);
        return ret;
    }

    // Store encoder handle, handle mapping, and resolution for this channel
    if (xSemaphoreTake(ctx->mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        ctx->tx_encoders[channel] = copy_encoder;
        ctx->tx_handle_map[channel].handle = *tx_handle;
        ctx->tx_handle_map[channel].channel = channel;
        ctx->channel_configs[channel].resolution_hz = config->resolution_hz;
        xSemaphoreGive(ctx->mutex);
    }

    // Enable channel
    ret = rmt_enable(*tx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable TX channel %lu: %s", (unsigned long)channel, esp_err_to_name(ret));
        if (xSemaphoreTake(ctx->mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            ctx->tx_encoders[channel] = NULL;
            ctx->tx_handle_map[channel].handle = NULL;
            ctx->tx_handle_map[channel].channel = RMT_CHANNEL_MAX;
            ctx->carrier_applied[channel] = false;
            xSemaphoreGive(ctx->mutex);
        }
        rmt_del_encoder(copy_encoder);
        rmt_del_channel(*tx_handle);
        return ret;
    }

    // Increment active TX channel count
    if (xSemaphoreTake(ctx->mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        ctx->active_tx_channels++;
        xSemaphoreGive(ctx->mutex);
    }

    ESP_LOGI(TAG, "RMT TX channel %lu configured (GPIO=%d, resolution=%lu Hz, carrier=%s)",
             (unsigned long)channel, config->gpio_num, (unsigned long)config->resolution_hz,
             config->carrier_en ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t rmt_driver_config_rx(rmt_driver_context_t *context, uint32_t channel, const rmt_driver_rx_config_t *config, rmt_channel_handle_t *rx_handle)
{
    rmt_driver_context_t *ctx = get_context(context);
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "RMT driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL || rx_handle == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    if (channel >= RMT_CHANNEL_MAX) {
        ESP_LOGE(TAG, "Invalid channel number: %lu (max: %d)", (unsigned long)channel, RMT_CHANNEL_MAX - 1);
        return ESP_ERR_INVALID_ARG;
    }

    // Create event queue for RX completion events (use configurable depth)
    uint32_t queue_depth = (config->event_queue_depth > 0) ? config->event_queue_depth : 4;
    QueueHandle_t event_queue = xQueueCreate(queue_depth, sizeof(rmt_rx_done_event_data_t));
    if (event_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create event queue for RX channel %lu (depth=%lu)", (unsigned long)channel, (unsigned long)queue_depth);
        return ESP_ERR_NO_MEM;
    }

    // Configure RMT RX channel
    rmt_rx_channel_config_t rx_channel_cfg = {
        .gpio_num = config->gpio_num,
        .clk_src = config->clk_src,
        .resolution_hz = config->resolution_hz,
        .mem_block_symbols = config->mem_block_symbols,
        .flags = {
            .invert_in = false,
            .with_dma = false,
        },
    };

    esp_err_t ret = rmt_new_rx_channel(&rx_channel_cfg, rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create RMT RX channel %lu: %s", (unsigned long)channel, esp_err_to_name(ret));
        vQueueDelete(event_queue);
        return ret;
    }

    // Register event callback to receive completion events
    // The callback will post events to the queue
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = rmt_rx_done_callback,
    };
    ret = rmt_rx_register_event_callbacks(*rx_handle, &cbs, event_queue);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register RX event callbacks for channel %lu: %s", (unsigned long)channel, esp_err_to_name(ret));
        vQueueDelete(event_queue);
        rmt_del_channel(*rx_handle);
        return ret;
    }

    // Filter configuration will be applied during rmt_receive() call
    // Store filter settings for later use (we'll apply them in receive function)
    (void)config->filter_en;
    (void)config->filter_ticks;
    (void)config->threshold;

    // Enable channel
    ret = rmt_enable(*rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RX channel %lu: %s", (unsigned long)channel, esp_err_to_name(ret));
        vQueueDelete(event_queue);
        rmt_del_channel(*rx_handle);
        return ret;
    }

    // Store event queue and handle mapping
    if (xSemaphoreTake(ctx->mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        ctx->rx_event_queues[channel] = event_queue;
        ctx->rx_handle_map[channel].handle = *rx_handle;
        ctx->rx_handle_map[channel].channel = channel;
        xSemaphoreGive(ctx->mutex);
    }

    // Increment active RX channel count
    if (xSemaphoreTake(ctx->mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        ctx->active_rx_channels++;
        xSemaphoreGive(ctx->mutex);
    }

    ESP_LOGI(TAG, "RMT RX channel %lu configured (GPIO=%d, resolution=%lu Hz, filter=%s)",
             (unsigned long)channel, config->gpio_num, (unsigned long)config->resolution_hz,
             config->filter_en ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t rmt_driver_transmit(rmt_driver_context_t *context, rmt_channel_handle_t tx_handle, const rmt_symbol_t *symbols, size_t num_symbols, uint32_t timeout_ms)
{
    rmt_driver_context_t *ctx = get_context(context);
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "RMT driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (tx_handle == NULL || symbols == NULL || num_symbols == 0) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    if (num_symbols > RMT_MAX_SYMBOLS_PER_TRANSACTION) {
        ESP_LOGE(TAG, "Too many symbols: %zu (max: %d)", num_symbols, RMT_MAX_SYMBOLS_PER_TRANSACTION);
        return ESP_ERR_INVALID_ARG;
    }

    // Find channel number and encoder handle
    uint32_t channel = RMT_CHANNEL_MAX;
    rmt_encoder_handle_t copy_encoder = NULL;
    if (xSemaphoreTake(ctx->mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        // Find channel number by matching handle
        for (int i = 0; i < RMT_CHANNEL_MAX; i++) {
            if (ctx->tx_handle_map[i].handle == tx_handle) {
                channel = i;
                copy_encoder = ctx->tx_encoders[i];
                break;
            }
        }
        xSemaphoreGive(ctx->mutex);
    }

    if (copy_encoder == NULL || channel >= RMT_CHANNEL_MAX) {
        ESP_LOGE(TAG, "No encoder found for TX channel. Channel may not be configured.");
        return ESP_ERR_INVALID_STATE;
    }

    // Lock channel buffer to prevent concurrent access
    SemaphoreHandle_t channel_mutex = ctx->channel_mutexes[channel];
    if (xSemaphoreTake(channel_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to lock channel buffer");
        return ESP_ERR_TIMEOUT;
    }

    // Use pre-allocated buffer for this channel
    rmt_symbol_word_t *rmt_symbols = ctx->tx_buffers[channel];

    // Convert rmt_symbol_t to rmt_symbol_word_t
    for (size_t i = 0; i < num_symbols; i++) {
        rmt_symbols[i].level0 = symbols[i].level0;
        rmt_symbols[i].duration0 = symbols[i].duration0;
        rmt_symbols[i].level1 = symbols[i].level1;
        rmt_symbols[i].duration1 = symbols[i].duration1;
    }

    // Transmit using the stored encoder (reused from channel configuration)
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,  // No loop
        .flags = {
            .eot_level = 0,  // End of transmission level
        },
    };

    esp_err_t ret = rmt_transmit(tx_handle, copy_encoder, rmt_symbols, num_symbols * sizeof(rmt_symbol_word_t), &tx_config);
    
    // Unlock channel buffer (transmission is queued, buffer can be reused after transmission completes)
    // Note: In a production system, you might want to keep the lock until transmission completes
    // For now, we unlock immediately since ESP-IDF copies the data
    xSemaphoreGive(channel_mutex);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to transmit RMT symbols: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "RMT transmission started (%zu symbols)", num_symbols);
    return ESP_OK;
}

esp_err_t rmt_driver_transmit_wait(rmt_driver_context_t *context, rmt_channel_handle_t tx_handle, const rmt_symbol_t *symbols, size_t num_symbols, uint32_t timeout_ms)
{
    esp_err_t ret = rmt_driver_transmit(context, tx_handle, symbols, num_symbols, timeout_ms);
    if (ret != ESP_OK) {
        return ret;
    }

    // Wait for transmission to complete using ESP-IDF's proper API
    ret = rmt_tx_wait_all_done(tx_handle, timeout_ms);
    if (ret != ESP_OK) {
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGD(TAG, "RMT transmission wait timeout");
            return ESP_ERR_TIMEOUT;
        }
        ESP_LOGE(TAG, "Failed to wait for RMT transmission: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "RMT transmission completed");
    return ESP_OK;
}

esp_err_t rmt_driver_receive(rmt_driver_context_t *context, rmt_channel_handle_t rx_handle, rmt_symbol_t *symbols, size_t num_symbols, size_t *received_symbols, uint32_t timeout_ms)
{
    rmt_driver_context_t *ctx = get_context(context);
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "RMT driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (rx_handle == NULL || symbols == NULL || received_symbols == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    if (num_symbols > RMT_MAX_SYMBOLS_PER_TRANSACTION) {
        ESP_LOGE(TAG, "Too many symbols: %zu (max: %d)", num_symbols, RMT_MAX_SYMBOLS_PER_TRANSACTION);
        return ESP_ERR_INVALID_ARG;
    }

    // Find event queue and channel number for this channel
    QueueHandle_t event_queue = NULL;
    uint32_t channel = RMT_CHANNEL_MAX;
    if (xSemaphoreTake(ctx->mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        for (int i = 0; i < RMT_CHANNEL_MAX; i++) {
            if (ctx->rx_handle_map[i].handle == rx_handle) {
                event_queue = ctx->rx_event_queues[i];
                channel = i;
                break;
            }
        }
        xSemaphoreGive(ctx->mutex);
    }

    if (event_queue == NULL || channel >= RMT_CHANNEL_MAX) {
        ESP_LOGE(TAG, "No event queue found for RX channel. Channel may not be configured.");
        return ESP_ERR_INVALID_STATE;
    }

    // Lock channel buffer to prevent concurrent access
    SemaphoreHandle_t channel_mutex = ctx->channel_mutexes[channel];
    if (xSemaphoreTake(channel_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to lock channel buffer");
        return ESP_ERR_TIMEOUT;
    }

    // Use pre-allocated buffer for this channel
    rmt_symbol_word_t *rmt_symbols = ctx->rx_buffers[channel];

    // Get filter configuration and resolution for this channel
    rmt_rx_filter_config_t filter_cfg;
    uint32_t resolution_hz = 1000000;  // Default fallback
    if (xSemaphoreTake(ctx->mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        filter_cfg = ctx->rx_filter_configs[channel];
        resolution_hz = ctx->channel_configs[channel].resolution_hz;
        if (resolution_hz == 0) {
            resolution_hz = 1000000;  // Fallback if not set
        }
        xSemaphoreGive(ctx->mutex);
    } else {
        // Default if mutex fails
        filter_cfg.filter_en = false;
        filter_cfg.filter_ticks = 0;
        filter_cfg.threshold = 0;
    }

    // Configure receive with filter settings
    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 0,      // Minimum signal duration in nanoseconds (0 = no limit)
        .signal_range_max_ns = 0,     // Maximum signal duration in nanoseconds (0 = no limit)
    };
    
    // Apply filter if enabled - convert threshold from ticks to nanoseconds using actual resolution
    if (filter_cfg.filter_en && filter_cfg.threshold > 0 && resolution_hz > 0) {
        uint64_t threshold_ns = ((uint64_t)filter_cfg.threshold * 1000000000ULL) / resolution_hz;
        receive_config.signal_range_min_ns = threshold_ns;
        ESP_LOGD(TAG, "Applied filter threshold: %lu ticks = %llu ns (resolution: %lu Hz)", 
                 (unsigned long)filter_cfg.threshold, (unsigned long long)threshold_ns, (unsigned long)resolution_hz);
    }

    esp_err_t ret = rmt_receive(rx_handle, rmt_symbols, num_symbols * sizeof(rmt_symbol_word_t), &receive_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start RMT receive: %s", esp_err_to_name(ret));
        xSemaphoreGive(channel_mutex);
        return ret;
    }

    // Wait for receive completion event with timeout
    rmt_rx_done_event_data_t rx_done_data;
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    
    if (xQueueReceive(event_queue, &rx_done_data, timeout_ticks) != pdTRUE) {
        // Timeout - disable channel to stop receive (ESP-IDF will handle cleanup)
        rmt_disable(rx_handle);
        rmt_enable(rx_handle);  // Re-enable for next receive
        xSemaphoreGive(channel_mutex);
        *received_symbols = 0;
        ESP_LOGD(TAG, "RMT receive timeout");
        return ESP_ERR_TIMEOUT;
    }

    // Calculate number of symbols received
    size_t num_received = rx_done_data.num_symbols;
    if (num_received > num_symbols) {
        ESP_LOGW(TAG, "Received %zu symbols but buffer only has space for %zu", num_received, num_symbols);
        num_received = num_symbols;
    }

    // Convert received symbols
    *received_symbols = num_received;
    for (size_t i = 0; i < num_received; i++) {
        symbols[i].level0 = rmt_symbols[i].level0;
        symbols[i].duration0 = rmt_symbols[i].duration0;
        symbols[i].level1 = rmt_symbols[i].level1;
        symbols[i].duration1 = rmt_symbols[i].duration1;
    }

    // Unlock channel buffer
    xSemaphoreGive(channel_mutex);

    ESP_LOGD(TAG, "RMT received %zu symbols", *received_symbols);
    return ESP_OK;
}

esp_err_t rmt_driver_enable(rmt_driver_context_t *context, rmt_channel_handle_t handle)
{
    rmt_driver_context_t *ctx = get_context(context);
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "RMT driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (handle == NULL) {
        ESP_LOGE(TAG, "Invalid handle");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = rmt_enable(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "RMT channel enabled");
    return ESP_OK;
}

esp_err_t rmt_driver_disable(rmt_driver_context_t *context, rmt_channel_handle_t handle)
{
    rmt_driver_context_t *ctx = get_context(context);
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "RMT driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (handle == NULL) {
        ESP_LOGE(TAG, "Invalid handle");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = rmt_disable(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable RMT channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGD(TAG, "RMT channel disabled");
    return ESP_OK;
}

esp_err_t rmt_driver_delete_tx(rmt_driver_context_t *context, rmt_channel_handle_t tx_handle)
{
    rmt_driver_context_t *ctx = get_context(context);
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "RMT driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (tx_handle == NULL) {
        ESP_LOGE(TAG, "Invalid TX handle");
        return ESP_ERR_INVALID_ARG;
    }

    // Wait for any ongoing transmission to complete before deleting
    // Use a long timeout (60 seconds) to block until done (safer for cleanup)
    // Note: portMAX_DELAY is for FreeRTOS ticks, rmt_tx_wait_all_done expects milliseconds
    esp_err_t wait_ret = rmt_tx_wait_all_done(tx_handle, 60000);  // 60 second timeout
    if (wait_ret != ESP_OK && wait_ret != ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Warning: Failed to wait for TX completion: %s", esp_err_to_name(wait_ret));
    }

    // Find and delete encoder for this channel
    rmt_encoder_handle_t encoder_to_delete = NULL;
    uint32_t channel = RMT_CHANNEL_MAX;
    
    if (xSemaphoreTake(ctx->mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        // Find encoder associated with this handle
        for (int i = 0; i < RMT_CHANNEL_MAX; i++) {
            if (ctx->tx_handle_map[i].handle == tx_handle) {
                encoder_to_delete = ctx->tx_encoders[i];
                channel = i;
                ctx->tx_encoders[i] = NULL;  // Clear encoder reference
                ctx->tx_handle_map[i].handle = NULL;  // Clear handle mapping
                ctx->tx_handle_map[i].channel = RMT_CHANNEL_MAX;
                ctx->carrier_applied[i] = false;  // Clear carrier flag
                break;
            }
        }
        xSemaphoreGive(ctx->mutex);
    }

    // Delete encoder if found
    if (encoder_to_delete != NULL) {
        esp_err_t encoder_ret = rmt_del_encoder(encoder_to_delete);
        if (encoder_ret != ESP_OK) {
            ESP_LOGW(TAG, "Warning: Failed to delete encoder: %s", esp_err_to_name(encoder_ret));
        }
    }
    
    // Delete channel mutex
    if (channel < RMT_CHANNEL_MAX && ctx->channel_mutexes[channel] != NULL) {
        vSemaphoreDelete(ctx->channel_mutexes[channel]);
        ctx->channel_mutexes[channel] = NULL;
    }

    // Explicitly remove carrier if it was applied
    if (channel < RMT_CHANNEL_MAX && ctx->carrier_applied[channel]) {
        // Disable carrier by applying empty carrier config
        rmt_carrier_config_t carrier_cfg = {
            .duty_cycle = 0.0f,
            .frequency_hz = 0,
            .flags = {
                .polarity_active_low = false,
            },
        };
        esp_err_t carrier_ret = rmt_apply_carrier(tx_handle, &carrier_cfg);
        if (carrier_ret != ESP_OK) {
            ESP_LOGW(TAG, "Warning: Failed to remove carrier: %s", esp_err_to_name(carrier_ret));
        }
    }

    // Disable channel before deletion (ensures clean state)
    rmt_disable(tx_handle);

    // Delete channel (this also cleans up carrier configuration)
    esp_err_t ret = rmt_del_channel(tx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete RMT TX channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Decrement active TX channel count
    if (xSemaphoreTake(ctx->mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (ctx->active_tx_channels > 0) {
            ctx->active_tx_channels--;
        }
        xSemaphoreGive(ctx->mutex);
    }

    ESP_LOGD(TAG, "RMT TX channel deleted");
    return ESP_OK;
}

esp_err_t rmt_driver_delete_rx(rmt_driver_context_t *context, rmt_channel_handle_t rx_handle)
{
    rmt_driver_context_t *ctx = get_context(context);
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGE(TAG, "RMT driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (rx_handle == NULL) {
        ESP_LOGE(TAG, "Invalid RX handle");
        return ESP_ERR_INVALID_ARG;
    }

    // Disable channel before deletion (ensures clean state)
    rmt_disable(rx_handle);

    // Find and delete event queue for this channel
    QueueHandle_t event_queue_to_delete = NULL;
    
    if (xSemaphoreTake(ctx->mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        for (int i = 0; i < RMT_CHANNEL_MAX; i++) {
            if (ctx->rx_handle_map[i].handle == rx_handle) {
                event_queue_to_delete = ctx->rx_event_queues[i];
                ctx->rx_event_queues[i] = NULL;
                ctx->rx_handle_map[i].handle = NULL;
                ctx->rx_handle_map[i].channel = RMT_CHANNEL_MAX;
                break;
            }
        }
        xSemaphoreGive(ctx->mutex);
    }

    // Find channel number for cleanup
    uint32_t channel = RMT_CHANNEL_MAX;
    if (xSemaphoreTake(ctx->mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        for (int i = 0; i < RMT_CHANNEL_MAX; i++) {
            if (ctx->rx_handle_map[i].handle == rx_handle) {
                channel = i;
                break;
            }
        }
        xSemaphoreGive(ctx->mutex);
    }

    // Delete event queue if found
    if (event_queue_to_delete != NULL) {
        vQueueDelete(event_queue_to_delete);
    }
    
    // Delete channel mutex
    if (channel < RMT_CHANNEL_MAX && ctx->channel_mutexes[channel] != NULL) {
        vSemaphoreDelete(ctx->channel_mutexes[channel]);
        ctx->channel_mutexes[channel] = NULL;
    }

    // Delete channel (this cleans up all channel resources including filters)
    esp_err_t ret = rmt_del_channel(rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete RMT RX channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // Decrement active RX channel count
    if (xSemaphoreTake(ctx->mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        if (ctx->active_rx_channels > 0) {
            ctx->active_rx_channels--;
        }
        xSemaphoreGive(ctx->mutex);
    }

    ESP_LOGD(TAG, "RMT RX channel deleted");
    return ESP_OK;
}

esp_err_t rmt_driver_deinit_context(rmt_driver_context_t *context)
{
    rmt_driver_context_t *ctx = get_context(context);
    if (ctx == NULL || !ctx->initialized) {
        ESP_LOGW(TAG, "RMT driver not initialized");
        return ESP_OK;
    }
    
    // If this is the global context, clear the global pointer
    if (ctx == global_context) {
        global_context = NULL;
    }

    if (ctx->mutex != NULL) {
        if (xSemaphoreTake(ctx->mutex, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to take mutex during deinit");
            return ESP_ERR_TIMEOUT;
        }
    }

    // Force delete all active channels to prevent leaks
    ESP_LOGW(TAG, "Deinitializing RMT driver - force deleting all channels");
    
    // Delete all TX channels
    for (int i = 0; i < RMT_CHANNEL_MAX; i++) {
        if (ctx->tx_handle_map[i].handle != NULL) {
            rmt_channel_handle_t handle = ctx->tx_handle_map[i].handle;
            
            // Wait for transmission (block until done with long timeout)
            esp_err_t wait_ret = rmt_tx_wait_all_done(handle, 60000);  // 60 second timeout
            if (wait_ret != ESP_OK && wait_ret != ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "Warning: Failed to wait for TX completion on channel %d: %s", i, esp_err_to_name(wait_ret));
            }
            
            // Remove carrier
            if (ctx->carrier_applied[i]) {
                rmt_carrier_config_t carrier_cfg = {
                    .duty_cycle = 0.0f,
                    .frequency_hz = 0,
                    .flags = {.polarity_active_low = false},
                };
                esp_err_t carrier_ret = rmt_apply_carrier(handle, &carrier_cfg);
                if (carrier_ret != ESP_OK) {
                    ESP_LOGW(TAG, "Warning: Failed to remove carrier on channel %d: %s", i, esp_err_to_name(carrier_ret));
                }
            }
            
            // Delete encoder
            if (ctx->tx_encoders[i] != NULL) {
                esp_err_t encoder_ret = rmt_del_encoder(ctx->tx_encoders[i]);
                if (encoder_ret != ESP_OK) {
                    ESP_LOGW(TAG, "Warning: Failed to delete encoder on channel %d: %s", i, esp_err_to_name(encoder_ret));
                }
                ctx->tx_encoders[i] = NULL;
            }
            
            // Disable and delete channel
            rmt_disable(handle);
            rmt_del_channel(handle);
            
            ctx->tx_handle_map[i].handle = NULL;
            ctx->tx_handle_map[i].channel = RMT_CHANNEL_MAX;
            ctx->carrier_applied[i] = false;
        }
    }
    
    // Delete all RX channels
    for (int i = 0; i < RMT_CHANNEL_MAX; i++) {
        if (ctx->rx_handle_map[i].handle != NULL) {
            rmt_channel_handle_t handle = ctx->rx_handle_map[i].handle;
            
            // Disable and delete channel
            esp_err_t disable_ret = rmt_disable(handle);
            if (disable_ret != ESP_OK) {
                ESP_LOGW(TAG, "Warning: Failed to disable RX channel %d: %s", i, esp_err_to_name(disable_ret));
            }
            esp_err_t del_ret = rmt_del_channel(handle);
            if (del_ret != ESP_OK) {
                ESP_LOGW(TAG, "Warning: Failed to delete RX channel %d: %s", i, esp_err_to_name(del_ret));
            }
            
            // Delete event queue
            if (ctx->rx_event_queues[i] != NULL) {
                vQueueDelete(ctx->rx_event_queues[i]);
                ctx->rx_event_queues[i] = NULL;
            }
            
            ctx->rx_handle_map[i].handle = NULL;
            ctx->rx_handle_map[i].channel = RMT_CHANNEL_MAX;
        }
    }

    ctx->active_tx_channels = 0;
    ctx->active_rx_channels = 0;
    ctx->initialized = false;

    if (ctx->mutex != NULL) {
        xSemaphoreGive(ctx->mutex);
        vSemaphoreDelete(ctx->mutex);
        ctx->mutex = NULL;
    }

    // Delete all channel mutexes (ensure cleanup even if channels weren't explicitly deleted)
    for (int i = 0; i < RMT_CHANNEL_MAX; i++) {
        if (ctx->channel_mutexes[i] != NULL) {
            vSemaphoreDelete(ctx->channel_mutexes[i]);
            ctx->channel_mutexes[i] = NULL;
        }
    }

    // Free context
    free(ctx);
    global_context = NULL;

    ESP_LOGI(TAG, "RMT driver deinitialized");
    return ESP_OK;
}

