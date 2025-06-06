#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "my_i2c.h"
#include "mpu6500.h"

static const char *TAG = "MPU6500";

static mpu6500_config_t *config;

static float gyro_inv_scale = 1.0;
static float accel_inv_scale = 1.0;

#define MPU6500_I2C_NUM I2C_NUM_0

esp_err_t i2c_mpu6500_init(mpu6500_config_t *cfg)
{
    ESP_LOGI(TAG, "Initializating MPU6500");
    vTaskDelay(pdMS_TO_TICKS(100));

    config = cfg;

    ESP_ERROR_CHECK(set_clock_source(MPU6500_CLOCK_PLL_XGYRO));
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_ERROR_CHECK(set_full_scale_gyro_range(cfg->accel_fs));
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_ERROR_CHECK(set_full_scale_accel_range(cfg->gyro_fs));
    vTaskDelay(pdMS_TO_TICKS(10));

    return ESP_OK;

}

esp_err_t set_clock_source(uint8_t adrs)
{
    return i2c_write_bits(MPU6500_I2C_NUM, MPU6500_I2C_ADDR, MPU6500_RA_PWR_MGMT_1, MPU6500_PWR1_CLKSEL_BIT, MPU6500_PWR1_CLKSEL_LENGTH, adrs);
}

// Sacle gyroscope từ dữ liệu số sang degree/s
float get_gyro_inv_scale(uint8_t scale_factor)
{
    switch (scale_factor)
    {
        case MPU6500_GYRO_FS_250:
            return 1.0 / MPU6500_GYRO_SCALE_FACTOR_0;
        case MPU6500_GYRO_FS_500:
            return 1.0 / MPU6500_GYRO_SCALE_FACTOR_1;
        case MPU6500_GYRO_FS_1000:
            return 1.0 / MPU6500_GYRO_SCALE_FACTOR_2;
        case MPU6500_GYRO_FS_2000:
            return 1.0 / MPU6500_GYRO_SCALE_FACTOR_3;
        default:
        ESP_LOGE(TAG, "get_gyro_inv_sacle(): invalid value (%d)", scale_factor);
        return 1;
    }
}

esp_err_t set_full_scale_gyro_range(uint8_t gyro_fs)
{
    gyro_inv_scale = get_gyro_inv_scale(gyro_fs);
    return i2c_write_bits(MPU6500_I2C_NUM, MPU6500_I2C_ADDR, MPU6500_RA_GYRO_CONFIG, MPU6500_GCONFIG_FS_SEL_BIT, MPU6500_GCONFIG_FS_SEL_LENGTH, gyro_fs); 
}

// Sacle accelerometer từ dữ liệu số sang g (gia tốc trọng trường)
float get_accel_inv_scale(uint8_t scale_factor)
{
  switch (scale_factor)
  {
  case MPU6500_ACCEL_FS_2:
    return 1.0 / MPU6500_ACCEL_SCALE_FACTOR_0;
  case MPU6500_ACCEL_FS_4:
    return 1.0 / MPU6500_ACCEL_SCALE_FACTOR_1;
  case MPU6500_ACCEL_FS_8:
    return 1.0 / MPU6500_ACCEL_SCALE_FACTOR_2;
  case MPU6500_ACCEL_FS_16:
    return 1.0 / MPU6500_ACCEL_SCALE_FACTOR_3;
  default:
    ESP_LOGE(TAG, "get_accel_inv_scale(): invalid value (%d)", scale_factor);
    return 1;
  }
}

esp_err_t set_full_scale_accel_range(uint8_t accel_fs)
{
  accel_inv_scale = get_accel_inv_scale(accel_fs);
  return i2c_write_bits(MPU6500_I2C_NUM, MPU6500_I2C_ADDR, MPU6500_RA_ACCEL_CONFIG_1, MPU6500_ACONFIG_FS_SEL_BIT, MPU6500_ACONFIG_FS_SEL_LENGTH, accel_fs);
}

// Hiệu chuẩn accelerometer bằng ma trận 
void calibrate_accel_affine(const vector_t *raw, vector_t *corrected)
{
    float (*M)[3] = config->calibration.accel_cal_matric;
    vector_t *b = &config->calibration.accel_bias_offset;

    float x = raw->x + b->x;
    float y = raw->y + b->y;
    float z = raw->z + b->z;

    corrected->x = M[0][0]*x + M[0][1]*y + M[0][2]*z;
    corrected->y = M[1][0]*x + M[1][1]*y + M[1][2]*z;
    corrected->z = M[2][0]*x + M[2][1]*y + M[2][2]*z;

    
}

void align_accel(uint8_t bytes[6], vector_t *v)
{
    int16_t xi = BYTE_2_INT_BE(bytes, 0);
    int16_t yi = BYTE_2_INT_BE(bytes, 2);
    int16_t zi = BYTE_2_INT_BE(bytes, 4);

    vector_t raw = {
      .x = (float)xi * accel_inv_scale,
      .y = (float)yi * accel_inv_scale,
      .z = (float)zi * accel_inv_scale
    };

    calibrate_accel_affine(&raw, v);
}

esp_err_t get_accel(vector_t *v)
{
    esp_err_t ret;
    uint8_t bytes[6];

    // Lấy lần lượt 6 bytes dữ liệu của accelerometer theo kiểu big endian 
    ret = i2c_read_bytes(MPU6500_I2C_NUM, MPU6500_I2C_ADDR, MPU6500_ACCEL_XOUT_H, bytes, 6);
    if (ret != ESP_OK)
    {
        return ret;
    }

    align_accel(bytes, v);

    return ESP_OK;
}

void align_gyro(uint8_t bytes[6], vector_t *v)
{
  int16_t xi = BYTE_2_INT_BE(bytes, 0);
  int16_t yi = BYTE_2_INT_BE(bytes, 2);
  int16_t zi = BYTE_2_INT_BE(bytes, 4);

  v->x = (float)xi * gyro_inv_scale - config->calibration.gyro_bias_offset.x;
  v->y = (float)yi * gyro_inv_scale - config->calibration.gyro_bias_offset.y;
  v->z = (float)zi * gyro_inv_scale - config->calibration.gyro_bias_offset.z;
}

esp_err_t get_gyro(vector_t *v)
{
  esp_err_t ret;
  uint8_t bytes[6];

  // Lấy lần lượt 6 bytes dữ liệu của gyroscope theo kiểu big endian 
  ret = i2c_read_bytes(MPU6500_I2C_NUM, MPU6500_I2C_ADDR, MPU6500_GYRO_XOUT_H, bytes, 6);
  if (ret != ESP_OK)
  {
    return ret;
  }

  align_gyro(bytes, v);

  return ESP_OK;
}

esp_err_t get_accel_gyro(vector_t *va, vector_t *vg)
{
  esp_err_t ret;
  uint8_t bytes[14];
  ret = i2c_read_bytes(MPU6500_I2C_NUM, MPU6500_I2C_ADDR, MPU6500_ACCEL_XOUT_H, bytes, 14);
  if (ret != ESP_OK)
  {
    return ret;
  }

  // Accelerometer - bytes 0:5
  align_accel(bytes, va);

  // Skip Temperature - bytes 6:7

  // Gyroscope - bytes 8:13s
  align_gyro(&bytes[8], vg);

  return ESP_OK;
}

