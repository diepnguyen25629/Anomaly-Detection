#include <stdio.h>
#include "inmp441_i2s.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "INMP441_I2S"
static i2s_chan_handle_t rx_handle = NULL;

esp_err_t inmp441_i2s_init(inmp441_config_t *config)
{
    esp_err_t ret;

    // Channel configuration  
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ret = i2s_new_channel(&chan_cfg, NULL, &rx_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create channel: %s", esp_err_to_name(ret));
        return ret;
    }

    // I2S standard mode 
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(config->audio_sample_rate),
        .slot_cfg = {
            .data_bit_width = config->data_bit_width,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = I2S_STD_SLOT_LEFT,
            .ws_width = 0,
            .ws_pol = false,
            .bit_shift = true,
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = config->bclk_io_num,
            .ws   = config->ws_io_num,
            .dout = I2S_GPIO_UNUSED,
            .din  = config->data_in_io_num,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ret = i2s_channel_init_std_mode(rx_handle, &std_cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to init I2S std mode: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2s_channel_enable(rx_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable channel: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "INMP441 I2S initialized (port %d)", I2S_NUM_0);
    return ESP_OK;
}

int inmp441_i2s_read(int16_t *buf, size_t buf_size)
{
    size_t bytes_read = 0;
    esp_err_t ret = i2s_channel_read(rx_handle, buf, buf_size, &bytes_read, portMAX_DELAY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read failed: %s", esp_err_to_name(ret));
        return -1;
    }
    return (int)bytes_read;
}

void inmp441_i2s_deinit(void)
{
    if (rx_handle) {
        i2s_channel_disable(rx_handle);
        i2s_del_channel(rx_handle);
        rx_handle = NULL;
        ESP_LOGI(TAG, "INMP441 I2S deinitialized");
    }
}
