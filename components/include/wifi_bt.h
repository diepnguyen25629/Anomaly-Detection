#ifndef _WIFI_BT_H
#define _WIFI_BT_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

typedef struct {
    char ssid[32];
    char password[64];
    char gui_ip[16];
} wifi_bt_config_t;

void wifi_bt_init(void); // Khởi tạo Bluetooth Classic và SPP server 
void wifi_bt_start_wifi(const char *ssid, const char *password); // Khởi tạo wifi 
int wifi_bt_tcp_connect(const char *ip, uint16_t port); // Kết nối TCP Socket để gửi dữ liệu cảm biến đến GUI

SemaphoreHandle_t wifi_bt_get_semaphore(void); // Khởi tạo Semaphore để chờ khi GUI gửi dữ liệu cấu hình
wifi_bt_config_t wifi_bt_get_config(void);

#endif