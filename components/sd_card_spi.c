#include "sd_card_spi.h"
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "sdmmc_cmd.h"
#include <errno.h>
#include <dirent.h>

static const char *TAG = "SD_CARD_SPI";

static sdmmc_card_t *sd_card = NULL;
static const char *mount_path = NULL;
static int sd_host_slot = -1;
 
esp_err_t sd_spi_mount(sd_spi_config_t *cfg)
{   
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = cfg->pin_mosi,
        .miso_io_num = cfg->pin_miso,
        .sclk_io_num = cfg->pin_clk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = cfg->pin_cs;
    slot_config.host_id = SPI2_HOST;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;

    ESP_LOGI(TAG, "Mounting SD card at %s", cfg->mount_point);
    ret = esp_vfs_fat_sdspi_mount(cfg->mount_point, &host, &slot_config, &mount_config, &sd_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card: %s", esp_err_to_name(ret));
        return ret;
    }

    sdmmc_card_print_info(stdout, sd_card);
    mount_path = cfg->mount_point;
    sd_host_slot = host.slot;  // <-- thêm dòng này
    return ESP_OK;
}

esp_err_t sd_spi_unmount(void)
{
    if (!sd_card || !mount_path) return ESP_FAIL;

    esp_vfs_fat_sdcard_unmount(mount_path, sd_card);
    spi_bus_free(sd_host_slot);
    ESP_LOGI(TAG, "SD card unmounted from %s", mount_path);

    // Reset trạng thái
    sd_card = NULL;
    mount_path = NULL;
    sd_host_slot = -1;

    return ESP_OK;
}

FILE* sd_spi_start_write(char *path)
{
    FILE *sd_file = fopen(path, "wb");
    if (!sd_file) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s (errno = %d)", path, errno );
        return NULL;
    }
    ESP_LOGI(TAG, "Started writing to file: %s", path);
    return sd_file;
}

esp_err_t sd_spi_stop_write(FILE *sd_file)
{
    fflush(sd_file);
    fclose(sd_file);
    ESP_LOGI(TAG, "File closed");
    return ESP_OK;
}

esp_err_t sd_spi_write_binary_buffer(FILE *sd_file, const void *data, size_t element_size, size_t count)
{
    if (!sd_file || !data || element_size == 0 || count == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t written = fwrite(data, element_size, count, sd_file);

    if (written != count) {
        if (ferror(sd_file)) {
            ESP_LOGE(TAG, "Write error: expected %d, got %d, errno=%d (%s)",
                     count, written, errno, strerror(errno));
        } else {
            ESP_LOGE(TAG, "Partial write: expected %d, got %d", count, written);
        }
        return ESP_FAIL;
    }
    return ESP_OK;
}


esp_err_t sd_spi_read_file(char *path, char *out_buf, size_t buf_size)
{
    ESP_LOGI(TAG, "Reading from file: %s", path);
    FILE *file = fopen(path, "r");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }

    if (fgets(out_buf, buf_size, file) == NULL) {
        ESP_LOGW(TAG, "No content read or file empty.");
        fclose(file);
        return ESP_FAIL;
    }

    char *pos = strchr(out_buf, '\n');
    if (pos) {
        *pos = '\0';
    }
    fclose(file);
    return ESP_OK;
}

void sd_spi_generate_unique_filename(const char *dir, const char *prefix, const char *ext, char *out_path, size_t max_len)
{
    int index = 1;
    while (index < 1000) {
        snprintf(out_path, max_len, "%s/%s%d.%s", dir, prefix, index, ext);
        FILE *file = fopen(out_path, "r");
        if (file) {
            fclose(file);
            index++;  
        } else {
            break;  
        }
    }
}