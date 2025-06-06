#ifndef __SPI_SD_CARD_H
#define __SPI_SD_CARD_H

#include "esp_err.h"

typedef struct {
    int pin_miso;
    int pin_mosi;
    int pin_clk;
    int pin_cs;
    const char *mount_point;
} sd_spi_config_t;

// Khởi tạo sd card và mount FAT filesystem
esp_err_t sd_spi_mount(sd_spi_config_t *cfg);
// Unmount  FAT file system và ngắt giao tiếp SPI 
esp_err_t sd_spi_unmount(void);
// Sẵn sàng ghi dữ liệu vào file 
FILE* sd_spi_start_write(char *path);
// Dừng ghi 
esp_err_t sd_spi_stop_write(FILE *sd_file);
// Ghi dữ liệu nhị phân 
esp_err_t sd_spi_write_binary_buffer(FILE *sd_file, const void *data, size_t element_size, size_t count);
// Đọc dữ liệu trong file 
esp_err_t sd_spi_read_file(char *path, char *out_buf, size_t buf_size);
// Đặt tên không trùng nhau cho các file 
void sd_spi_generate_unique_filename(const char *dir, const char *prefix, const char *ext, char *out_path, size_t max_len);

#endif