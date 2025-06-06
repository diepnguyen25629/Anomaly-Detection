#ifndef __BMP280_H
#define __BMP280_H

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>


#define BMP280_I2C_ADDR 0x76
#define BMP280_CHIP_ID 0x58

// Register addresses
#define BMP280_REG_CALIB 0x88
#define BMP280_REG_ID 0xD0
#define BMP280_REG_RESET 0xE0
#define BMP280_REG_STATUS 0xF3
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG 0xF5
#define BMP280_REG_PRESS_MSB 0xF7

// Chế độ hoạt động của BMP280
typedef enum{
    BMP280_MODE_SLEEP = 0x00,
    BMP280_MODE_FORCED = 0x01,
    BMP280_MODE_NORMAL = 0x03
}BMP280_Mode;

// Bộ lọc của BMP280 (lọc càng cao dữ liệu cang mượt nhưng độ phản hồi chậm hơn)
typedef enum {
    BMP280_FILTER_OFF = 0,
    BMP280_FILTER_2 = 1,
    BMP280_FILTER_4 = 2,
    BMP280_FILTER_8 = 3,
    BMP280_FILTER_16 = 4
}BMP280_Filter;

// Oversampling của preesure 
typedef enum {
    BMP280_SKIPPED = 0,          // no measurement
    BMP280_ULTRA_LOW_POWER = 1,  // oversampling x1
    BMP280_LOW_POWER = 2,        // oversampling x2
    BMP280_STANDARD = 3,         // oversampling x4
    BMP280_HIGH_RES = 4,         // oversampling x8
    BMP280_ULTRA_HIGH_RES = 5    // oversampling x16
} BMP280_Oversampling;

// Thời gian nghỉ giữa các lần đo 
typedef enum {
    BMP280_STANDBY_05 = 0,      //!< stand by time 0.5ms
    BMP280_STANDBY_62 = 1,      //!< stand by time 62.5ms
    BMP280_STANDBY_125 = 2,     //!< stand by time 125ms
    BMP280_STANDBY_250 = 3,     //!< stand by time 250ms
    BMP280_STANDBY_500 = 4,     //!< stand by time 500ms
    BMP280_STANDBY_1000 = 5,    //!< stand by time 1s
    BMP280_STANDBY_2000 = 6,    //!< stand by time 2s BMP280
    BMP280_STANDBY_4000 = 7,    //!< stand by time 4s BMP280
} BMP280_StandbyTime;

typedef struct {
    BMP280_Mode mode;
    BMP280_Filter filter;
    BMP280_Oversampling oversampling_pressure;
    BMP280_Oversampling oversampling_temperature;
    BMP280_StandbyTime stanby;
    uint8_t pre_sample_rate;
} bmp280_config_t;

// Hệ số hiệu chuẩn của nhà sản xuất được lưu trong cảm biến 
typedef struct {
    int32_t t_fine;
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

} bmp280_t;    

// Khởi tạo cảm biến BMP280
esp_err_t bmp280_init(bmp280_config_t *params);
// Lấy dữ liệu nhiệt độ của cảm biến BMP280
esp_err_t bmp280_read_temp(float *temperature);
// Lấy dữ liệu áp suất của cảm biến BMP280
esp_err_t bmp280_read_pre(float *pressure);

#endif
