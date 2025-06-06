#ifndef __MPU6500_H
#define __MPU6500_H

#include "freertos/FreeRTOS.h"
#include "driver/i2c.h"

#define MPU6500_I2C_ADDR (0x68)
#define MPU6500_WHO_I_AM (0x71)

#define MPU6500_RA_PWR_MGMT_1 (0x6B)
#define MPU6500_PWR1_CLKSEL_BIT (0)
#define MPU6500_PWR1_CLKSEL_LENGTH (3)

#define MPU6500_ACCEL_XOUT_H (0x3B)
#define MPU6500_ACCEL_XOUT_L (0x3C)
#define MPU6500_ACCEL_YOUT_H (0x3D)
#define MPU6500_ACCEL_YOUT_L (0x3E)
#define MPU6500_ACCEL_ZOUT_H (0x3F)
#define MPU6500_ACCEL_ZOUT_L (0x40)
#define MPU6500_TEMP_OUT_H (0x41)
#define MPU6500_TEMP_OUT_L (0x42)
#define MPU6500_GYRO_XOUT_H (0x43)
#define MPU6500_GYRO_XOUT_L (0x44)
#define MPU6500_GYRO_YOUT_H (0x45)
#define MPU6500_GYRO_YOUT_L (0x46)
#define MPU6500_GYRO_ZOUT_H (0x47)
#define MPU6500_GYRO_ZOUT_L (0x48)

#define MPU6500_RA_CONFIG (0x1A)
#define MPU6500_RA_GYRO_CONFIG (0x1B)
#define MPU6500_RA_ACCEL_CONFIG_1 (0x1C)
#define MPU6500_RA_ACCEL_CONFIG_2 (0x1D)

#define MPU6500_GCONFIG_FS_SEL_BIT (3)
#define MPU6500_GCONFIG_FS_SEL_LENGTH (2)
#define MPU6500_GYRO_FS_250 (0x00)
#define MPU6500_GYRO_FS_500 (0x01)
#define MPU6500_GYRO_FS_1000 (0x02)
#define MPU6500_GYRO_FS_2000 (0x03)
#define MPU6500_GYRO_SCALE_FACTOR_0 (131)
#define MPU6500_GYRO_SCALE_FACTOR_1 (65.5)
#define MPU6500_GYRO_SCALE_FACTOR_2 (32.8)
#define MPU6500_GYRO_SCALE_FACTOR_3 (16.4)

#define MPU6500_ACONFIG_FS_SEL_BIT (3)
#define MPU6500_ACONFIG_FS_SEL_LENGTH (2)
#define MPU6500_ACCEL_FS_2 (0x00)
#define MPU6500_ACCEL_FS_4 (0x01)
#define MPU6500_ACCEL_FS_8 (0x02)
#define MPU6500_ACCEL_FS_16 (0x03)
#define MPU6500_ACCEL_SCALE_FACTOR_0 (16384)
#define MPU6500_ACCEL_SCALE_FACTOR_1 (8192)
#define MPU6500_ACCEL_SCALE_FACTOR_2 (4096)
#define MPU6500_ACCEL_SCALE_FACTOR_3 (2048)

#define MPU6500_CLOCK_INTERNAL (0x00)
#define MPU6500_CLOCK_PLL_XGYRO (0x01)
#define MPU6500_CLOCK_PLL_YGYRO (0x02)
#define MPU6500_CLOCK_PLL_ZGYRO (0x03)
#define MPU6500_CLOCK_KEEP_RESET (0x07)
#define MPU6500_CLOCK_PLL_EXT32K (0x04)
#define MPU6500_CLOCK_PLL_EXT19M (0x05)

#define BYTE_2_INT_BE(byte, i) ((int16_t)((byte[i] << 8) + (byte[i + 1])))
#define BYTE_2_INT_LE(byte, i) ((int16_t)((byte[i + 1] << 8) + (byte[i])))

typedef struct {
    float x, y, z;
} vector_t;

typedef enum {
    ACCEL_FS_2G = 0,
    ACCEL_FS_4G,
    ACCEL_FS_8G,
    ACCEL_FS_16G,
}accel_fs_t;

typedef enum {
    GYRO_FS_250DPS = 0,
    GYRO_FS_500DPS,
    GYRO_FS_1000DPS,
    GYRO_FS_2500DPS
} gyro_fs_t;

typedef struct {
    vector_t gyro_bias_offset;          // Bias gyroscope

    vector_t accel_bias_offset;         // Bias accelerometer
    float accel_cal_matric[3][3];       // Calibration matric affine 3x3

} mpu6500_calibration_t;

typedef struct {
    uint16_t imu_sample_rate;

    accel_fs_t accel_fs;
    gyro_fs_t gyro_fs;

    mpu6500_calibration_t calibration;
} mpu6500_config_t;

// Khởi tạo mpu6500
esp_err_t i2c_mpu6500_init(mpu6500_config_t *cal);
// Cấu hình xung clock
esp_err_t set_clock_source(uint8_t adrs);
// Cấu hình phạm vi đo của gỷroscope
esp_err_t set_full_scale_gyro_range(uint8_t adrs);
// Cấu hình phạm vi đo của accelerometer 
esp_err_t set_full_scale_accel_range(uint8_t adrs);

// Lấy dữ liệu accelerometer và gyroscope
esp_err_t get_accel(vector_t *v);
esp_err_t get_gyro(vector_t *v);
esp_err_t get_accel_gyro(vector_t *va, vector_t *vg);

#endif
