#ifndef __MY_I2C_H
#define __MY_I2C_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"

// Khởi tạo giao tiếp I2C 
esp_err_t i2c_master_init(uint8_t i2c_num, uint8_t gpio_sda, uint8_t gpio_scl);

// Ghi byte dũ liệu vào trong thanh ghi 
esp_err_t i2c_write_bytes(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t *data, size_t data_len);
esp_err_t i2c_write_byte(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t data);

// Lấy byte dữ liệu từ thanh ghi 
esp_err_t i2c_read_bytes(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t *data, size_t data_len);
esp_err_t i2c_read_byte(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t *data);

// Ghi bit dũ liệu vào trong thanh ghi  
esp_err_t i2c_write_bits(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t bit, uint8_t length, uint8_t value);
esp_err_t i2c_write_bit(i2c_port_t i2c_num, uint8_t periph_address, uint8_t reg_address, uint8_t bit, uint8_t value);

#endif

