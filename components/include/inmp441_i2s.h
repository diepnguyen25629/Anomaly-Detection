#ifndef __INMP441_H
#define __INMP441_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct 
{
    uint32_t audio_sample_rate;       // sample rate (Hz)
    int bclk_io_num;            // I2S_SCK pin
    int ws_io_num;              // I2S_WS pin
    int data_in_io_num;          // I2S_SD pin
    int data_bit_width;         
}inmp441_config_t;

// Khởi tạo I2S 
esp_err_t inmp441_i2s_init(inmp441_config_t *config);
// Lấy dữ liệu cảm biến INMP441
int inmp441_i2s_read(int16_t *buf, size_t buf_size);
// Reset khởi tao I2S
void inmp441_i2s_deinit(void);

#endif