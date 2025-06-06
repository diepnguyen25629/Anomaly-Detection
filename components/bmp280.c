#include "bmp280.h"
#include <math.h>
#include "esp_log.h"

#include "my_i2c.h"
#include "driver/i2c.h"

#define TAG "BMP280"

#define BMP280_I2C_NUM I2C_NUM_0

static bmp280_t bmp280_dev;
static bmp280_config_t bmp280_cfg;

static esp_err_t read_calibration_data(i2c_port_t port, uint8_t addr)
{
    uint8_t calib[24];
    ESP_ERROR_CHECK(i2c_read_bytes(port, addr, BMP280_REG_CALIB, calib, 24));

    bmp280_dev.dig_T1 = (uint16_t)(calib[1] << 8) | calib[0];
    bmp280_dev.dig_T2 = (int16_t)(calib[3] << 8) | calib[2];
    bmp280_dev.dig_T3 = (int16_t)(calib[5] << 8) | calib[4];
    bmp280_dev.dig_P1 = (uint16_t)(calib[7] << 8) | calib[6];
    bmp280_dev.dig_P2 = (int16_t)(calib[9] << 8) | calib[8];
    bmp280_dev.dig_P3 = (int16_t)(calib[11] << 8) | calib[10];
    bmp280_dev.dig_P4 = (int16_t)(calib[13] << 8) | calib[12];
    bmp280_dev.dig_P5 = (int16_t)(calib[15] << 8) | calib[14];
    bmp280_dev.dig_P6 = (int16_t)(calib[17] << 8) | calib[16];
    bmp280_dev.dig_P7 = (int16_t)(calib[19] << 8) | calib[18];
    bmp280_dev.dig_P8 = (int16_t)(calib[21] << 8) | calib[20];
    bmp280_dev.dig_P9 = (int16_t)(calib[23] << 8) | calib[22];  
    
    return ESP_OK;
}

esp_err_t bmp280_init(bmp280_config_t *params)
{
    ESP_LOGI(TAG, "Initializating BMP280");
    vTaskDelay(100 / portTICK_PERIOD_MS);

    if (params == NULL) {
        // Cấu hình mặc định của bmp280 nếu cấu hình truyền vào là NULL
        bmp280_cfg.mode = BMP280_MODE_NORMAL;
        bmp280_cfg.filter = BMP280_FILTER_OFF;
        bmp280_cfg.oversampling_pressure = BMP280_STANDARD;     // x4
        bmp280_cfg.oversampling_temperature = BMP280_STANDARD;  // x4
        bmp280_cfg.stanby = BMP280_STANDBY_250;                 // 250ms
        bmp280_cfg.pre_sample_rate = 5;
    } else {
        bmp280_cfg = *params;
    }
    
    uint8_t id;
    ESP_ERROR_CHECK(i2c_read_byte(BMP280_I2C_NUM, BMP280_I2C_ADDR, BMP280_REG_ID, &id));

    if (id != BMP280_CHIP_ID) 
    {
        ESP_LOGE(TAG, "Device ID mismatch: expected 0x%x, got 0x%x", BMP280_CHIP_ID, id);
        return ESP_FAIL;
    }

    // Reset the device
    ESP_ERROR_CHECK(i2c_write_byte(BMP280_I2C_NUM, BMP280_I2C_ADDR, BMP280_REG_RESET, 0xB6));
    vTaskDelay(10 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(read_calibration_data(BMP280_I2C_NUM, BMP280_I2C_ADDR));

    uint8_t ctrl_meas = (params->oversampling_temperature << 5) | (params->oversampling_pressure << 2) | params->mode;
    uint8_t config = (params->stanby << 5) | (params->filter << 2);

    ESP_ERROR_CHECK(i2c_write_byte(BMP280_I2C_NUM, BMP280_I2C_ADDR, BMP280_REG_CTRL_MEAS, ctrl_meas));
    ESP_ERROR_CHECK(i2c_write_byte(BMP280_I2C_NUM, BMP280_I2C_ADDR, BMP280_REG_CONFIG, config));

    return ESP_OK;

}

static int32_t compenstate_temperature(int32_t adc_T, float *temp)
{
    int32_t var1, var2;
    var1 = ((((adc_T >> 3) - ((int32_t)bmp280_dev.dig_T1 << 1))) * ((int32_t)bmp280_dev.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)bmp280_dev.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp280_dev.dig_T1))) >> 12) *
                    ((int32_t)bmp280_dev.dig_T3)) >> 14;

    bmp280_dev.t_fine = var1 + var2;
    *temp = (bmp280_dev.t_fine * 5 + 128) >> 8;
    *temp /= 100;
    return bmp280_dev.t_fine;
}

static uint32_t compenstate_pressure(int32_t adc_P, float *press) {
    int64_t var1, var2, p;

    var1 = ((int64_t)bmp280_dev.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280_dev.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp280_dev.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280_dev.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280_dev.dig_P3) >> 8) +
           ((var1 * (int64_t)bmp280_dev.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280_dev.dig_P1) >> 33;

    if (var1 == 0) {
        *press = 0;
        return 0; // avoid exception
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280_dev.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp280_dev.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_dev.dig_P7) << 4);
    *press = (float)p / 25600.0;
    return (uint32_t)p;
}

esp_err_t bmp280_read_temp(float *temperature)
{
    uint8_t data[6];
    ESP_ERROR_CHECK(i2c_read_bytes(BMP280_I2C_NUM, BMP280_I2C_ADDR, BMP280_REG_PRESS_MSB, data, 6));
    int32_t adc_T = (int32_t)(((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | (data[5] << 4));
    compenstate_temperature(adc_T, temperature);
    return ESP_OK;
}

esp_err_t bmp280_read_pre(float *pressure)
{
    uint8_t data[6];
    ESP_ERROR_CHECK(i2c_read_bytes(BMP280_I2C_NUM, BMP280_I2C_ADDR, BMP280_REG_PRESS_MSB, data, 6));
    int32_t adc_P = (int32_t)(((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] << 4));
    float dummy;
    if (bmp280_dev.t_fine == 0)
    {
        int32_t adc_T = (int32_t)(((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | (data[5] << 4));
        compenstate_temperature(adc_T, &dummy);
    }
    compenstate_pressure(adc_P, pressure);
    return ESP_OK;
}