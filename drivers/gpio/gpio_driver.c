#include "gpio_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "GPIO_DRIVER";
static bool driver_initialized = false;
static SemaphoreHandle_t driver_mutex = NULL;

esp_err_t gpio_driver_init(void)
{
    if (driver_initialized) {
        ESP_LOGW(TAG, "GPIO driver already initialized");
        return ESP_OK;
    }

    // Create mutex for thread-safe operations
    driver_mutex = xSemaphoreCreateMutex();
    if (driver_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    driver_initialized = true;
    ESP_LOGI(TAG, "GPIO driver initialized");
    return ESP_OK;
}

esp_err_t gpio_driver_config(const gpio_driver_config_t *config)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "GPIO driver not initialized. Call gpio_driver_init() first");
        return ESP_ERR_INVALID_STATE;
    }

    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid configuration pointer");
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    // Configure GPIO pin
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->pin),
        .mode = config->mode,
        .pull_up_en = (config->pull_mode == GPIO_PULLUP_ONLY || config->pull_mode == GPIO_PULLUP_PULLDOWN),
        .pull_down_en = (config->pull_mode == GPIO_PULLDOWN_ONLY || config->pull_mode == GPIO_PULLUP_PULLDOWN),
        .intr_type = config->intr_type_enable ? config->intr_type : GPIO_INTR_DISABLE
    };

    esp_err_t ret = gpio_config(&io_conf);
    
    xSemaphoreGive(driver_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO %d: %s", config->pin, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "GPIO %d configured successfully", config->pin);
    return ESP_OK;
}

esp_err_t gpio_driver_set_level(gpio_num_t pin, uint32_t level)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "GPIO driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (level > 1) {
        ESP_LOGE(TAG, "Invalid level value: %lu (must be 0 or 1)", level);
        return ESP_ERR_INVALID_ARG;
    }

    // Note: gpio_set_level() is thread-safe in ESP-IDF, but we protect for consistency
    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = gpio_set_level(pin, level);
    
    xSemaphoreGive(driver_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set GPIO %d level: %s", pin, esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

int gpio_driver_get_level(gpio_num_t pin)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "GPIO driver not initialized");
        return -1;
    }

    // Note: gpio_get_level() is thread-safe in ESP-IDF, but we protect for consistency
    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return -1;
    }

    int level = gpio_get_level(pin);
    
    xSemaphoreGive(driver_mutex);

    return level;
}

esp_err_t gpio_driver_toggle(gpio_num_t pin)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "GPIO driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Protect entire toggle operation atomically
    if (xSemaphoreTake(driver_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take mutex");
        return ESP_ERR_TIMEOUT;
    }

    int current_level = gpio_get_level(pin);
    if (current_level < 0) {
        xSemaphoreGive(driver_mutex);
        ESP_LOGE(TAG, "Failed to get GPIO %d level", pin);
        return ESP_FAIL;
    }

    uint32_t new_level = (current_level == 0) ? 1 : 0;
    esp_err_t ret = gpio_set_level(pin, new_level);
    
    xSemaphoreGive(driver_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to toggle GPIO %d: %s", pin, esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

esp_err_t gpio_driver_install_isr(int intr_alloc_flags)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "GPIO driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = gpio_install_isr_service(intr_alloc_flags);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "GPIO ISR service installed");
    return ESP_OK;
}

esp_err_t gpio_driver_add_isr_handler(gpio_num_t pin, gpio_isr_t isr_handler, void *args)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "GPIO driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (isr_handler == NULL) {
        ESP_LOGE(TAG, "Invalid ISR handler");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = gpio_isr_handler_add(pin, isr_handler, args);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler for GPIO %d: %s", pin, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ISR handler added for GPIO %d", pin);
    return ESP_OK;
}

esp_err_t gpio_driver_remove_isr_handler(gpio_num_t pin)
{
    if (!driver_initialized) {
        ESP_LOGE(TAG, "GPIO driver not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = gpio_isr_handler_remove(pin);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove ISR handler for GPIO %d: %s", pin, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ISR handler removed for GPIO %d", pin);
    return ESP_OK;
}

esp_err_t gpio_driver_deinit(void)
{
    if (!driver_initialized) {
        ESP_LOGW(TAG, "GPIO driver not initialized");
        return ESP_OK;
    }

    // Take mutex to prevent race conditions with other operations
    if (driver_mutex != NULL) {
        if (xSemaphoreTake(driver_mutex, portMAX_DELAY) != pdTRUE) {
            ESP_LOGE(TAG, "Failed to take mutex during deinit");
            return ESP_ERR_TIMEOUT;
        }
    }

    driver_initialized = false;

    // Delete mutex last
    if (driver_mutex != NULL) {
        vSemaphoreDelete(driver_mutex);
        driver_mutex = NULL;
    }

    ESP_LOGI(TAG, "GPIO driver deinitialized");
    return ESP_OK;
}

