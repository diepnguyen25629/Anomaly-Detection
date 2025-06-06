#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

#include "driver/i2c.h"

#include "my_i2c.h"
#include "mpu6500.h"
#include "sd_card_spi.h"

static const char *TAG = "MAIN"; 

// Khởi tạo các chân giao tiếp I2C
#define I2C_MASTER_NUM I2C_NUM_0  
#define I2C_SDA_GPIO GPIO_NUM_17
#define I2C_SCL_GPIO GPIO_NUM_16

// Khởi tạo các chân button, led ghi dũ liệu và led kết nối đến GUI
#define BUTTON_GPIO GPIO_NUM_0
#define LED_RECORD_GPIO GPIO_NUM_15

#define imu_BUF_LEN 32

mpu6500_config_t mpu6500_cfg = {
    .calibration = {
        .gyro_bias_offset = {.x = -2.07176219, .y = 9.80366151, .z = 0.37255191},
        .accel_bias_offset = {.x = -0.00132555 , .y = -0.00647594, .z = 0.01120966},
        .accel_cal_matric = {
            {1.00089023, 0.00616087, -0.007825},
            {-0.00973511, 1.00038651, 0.00564757},
            { 0.035241 , -0.00492608,  0.98840621}
        }
    },
    .imu_sample_rate = 20,
    .accel_fs = ACCEL_FS_16G,
    .gyro_fs = GYRO_FS_2500DPS

};
/*

// Cấu hình các chân giao tiếp SPI của sd card 
sd_spi_config_t sd_cfg = {
    .pin_miso = 32,
    .pin_mosi = 23,
    .pin_clk = 33,
    .pin_cs = 5,
    .mount_point = "/data"
};

typedef struct {
    vector_t accel_data, gyro_data;
}imu_data_t;

QueueHandle_t imu_queue;

// Trạng thái record vào sd card 
volatile bool is_recording = false;
int file_index = 0;

void read_imu_task(void *pvParameter)
{   
    imu_data_t imu_data; 
    esp_err_t ret;

    // Khởi tạo giao tiếp I2C
    ret = i2c_master_init(I2C_MASTER_NUM, I2C_SDA_GPIO, I2C_SCL_GPIO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C");
        vTaskDelete(NULL);
        return;
    }

    // Khởi tạo MPU6500 với các tham số hiệu chỉnh 
    ret = i2c_mpu6500_init(&mpu6500_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6500");
        vTaskDelete(NULL);
        return;
    }

    // Tính chu kỳ delay theo sample rate (ms)
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / mpu6500_cfg.imu_sample_rate);
    TickType_t xLastWakeTime = xTaskGetTickCount();  // Mốc thời gian lần đầu

    while (1) {
        // Đọc dữ liệu cảm biến MPU6500
        ret = get_accel_gyro(&imu_data.accel_data, &imu_data.gyro_data);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read MPU6500");
        } else {

            // Gửi vào hàng đợi
            if (xQueueSend(imu_queue, &imu_data, 0) != pdTRUE) {
                imu_data_t discarded;
                xQueueReceive(imu_queue, &discarded, 0);
                xQueueSend(imu_queue, &imu_data, 0);
            }
        }

        // Delay cho đến mốc thời gian kế tiếp (đảm bảo tần số cố định)
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void write_imu_to_sd_task(void *pvParameter)
{
    imu_data_t imu_data_recv;
    FILE *imu_file = NULL;
    static char imu_file_path[64];

    while (1) {
        // Nếu button được nhấn thì bắt đầu ghi vào sd card 
        if (is_recording && imu_file == NULL) {
            sd_spi_generate_unique_filename("/data", "imu", "csv", imu_file_path, sizeof(imu_file_path));
            imu_file = sd_spi_start_write(imu_file_path);
            if (!imu_file) {
                ESP_LOGE(TAG, "Failed to open imu file");
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
        }
        // Nếu button được nhấn lần nữa thì kết thúc ghi và ghi dữ liệu còn lại vào sd card 
        if (!is_recording && imu_file != NULL) {
            sd_spi_stop_write(imu_file);
            imu_file = NULL;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Ghi dữ liệu từ hàng đợi vào buffer đến khi đầy thì ghi vào sd card 
        if (is_recording && imu_file != NULL) {
            if (xQueueReceive(imu_queue, &imu_data_recv, pdMS_TO_TICKS(100)) == pdTRUE) {
                char line[128];
                snprintf(line, sizeof(line), "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
                    imu_data_recv.accel_data.x,
                    imu_data_recv.accel_data.y,
                    imu_data_recv.accel_data.z,
                    imu_data_recv.gyro_data.x,
                    imu_data_recv.gyro_data.y,
                    imu_data_recv.gyro_data.z);
                sd_spi_write_line(imu_file, line);
            } else {
            vTaskDelay(pdMS_TO_TICKS(50));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Khởi tạo các chân gpio của LED và button 
void gpio_init()
{
    gpio_config_t button_io_cfg = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&button_io_cfg);

    gpio_config_t led_record_io_cfg = {
        .pin_bit_mask = (1ULL << LED_RECORD_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };

    gpio_config(&led_record_io_cfg);
}

// Kiểm tra trạng thái của button 
void button_monitor_task(void *pvParameter)
{
    bool last_button_state = true;
    while (1) {
        bool button_state = gpio_get_level(BUTTON_GPIO);

        // Nút được nhấn (LOW)
        if (!button_state && last_button_state) {
            is_recording = !is_recording;
            xQueueReset(imu_queue);   // Xoá dữ liệu cũ
            ESP_LOGI(TAG, "Recording: %s", is_recording ? "ON" : "OFF");
            vTaskDelay(pdMS_TO_TICKS(200));  // Chống dội nút
        }

        last_button_state = button_state;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void led_blink_task(void *pvParameter)
{
    while (1) {
        if (is_recording) {
            gpio_set_level(LED_RECORD_GPIO, 0);  // LED ON (kéo xuống)
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else {
            gpio_set_level(LED_RECORD_GPIO, 1);  // LED luôn OFF khi không ghi
            vTaskDelay(pdMS_TO_TICKS(1000));
        }      
    }
}

void app_main(void)
{
    if (sd_spi_mount(&sd_cfg) != ESP_OK){
        return;
    }

    imu_queue = xQueueCreate(100, sizeof(imu_data_t));

    if (!imu_queue ) {
    ESP_LOGE(TAG, "Failed to create queues!");
    return;
    }

    gpio_init();
    xTaskCreate(button_monitor_task, "Button", 2048, NULL, 4, NULL);
    xTaskCreate(led_blink_task, "LED", 1024, NULL, 4, NULL);

    xTaskCreate(read_imu_task, "ReadIMUPT", 4096, NULL, 5, NULL);

    xTaskCreate(write_imu_to_sd_task, "WriteIMUPT", 4096, NULL, 6, NULL);
}

*/

vector_t accel_data, gyro_data;
bool is_read_data = false;

void app_main(void)
{
    gpio_config_t button_io_cfg = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&button_io_cfg);

    gpio_config_t led_io_cfg = {
        .pin_bit_mask = (1ULL << LED_RECORD_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };

    gpio_config(&led_io_cfg);

    gpio_set_level(LED_RECORD_GPIO, 1); // LED OFF (nguồn ngoài, LOW là bật)

    // Khởi tạo giao tiếp I2C
    esp_err_t ret = i2c_master_init(I2C_MASTER_NUM, I2C_SDA_GPIO, I2C_SCL_GPIO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C: %s", esp_err_to_name(ret));
        return;
    }

    // Khởi tạo MPU6500
    ret = i2c_mpu6500_init(&mpu6500_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6500: %s", esp_err_to_name(ret));
        return;
    }


    bool last_button_state = true;
    while (1) {
        bool button_state = gpio_get_level(BUTTON_GPIO);

        // Nút được nhấn (LOW)
        if (!button_state && last_button_state) {
            is_read_data = !is_read_data;
            printf("Read accel and gyro mpu6500: %s\n", is_read_data ? "ON" : "OFF");
            vTaskDelay(pdMS_TO_TICKS(300));  // Chống dội nút
            gpio_set_level(LED_RECORD_GPIO, !is_read_data);  // LED ON (kéo xuống)

        }

        if(is_read_data){
        // Ghi log ra serial
        ret = get_accel_gyro(&accel_data, &gyro_data);
        if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data MPU6500: %s", esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(200)); 
        return;
        }
        printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                    accel_data.x, accel_data.y, accel_data.z,
                    gyro_data.x, gyro_data.y, gyro_data.z);
        }
        last_button_state = button_state;
        vTaskDelay(pdMS_TO_TICKS(20));
    }    
}
    