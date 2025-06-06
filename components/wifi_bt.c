#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h> 

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"       
#include "esp_spp_api.h"

#include "wifi_bt.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define MAX_RETRY 5

static const char *TAG = "WIFI_UART";

static EventGroupHandle_t s_wifi_event_group;
static SemaphoreHandle_t bt_config_sem = NULL;
static uint32_t spp_client_handle = 0;
static int retry_num = 0;

static const char *local_device_name = "ESP32_SPP_SERVER"; // Tên thiết bị ESP32

static wifi_bt_config_t bt_wifi_config;

wifi_bt_config_t wifi_bt_get_config(void) {
    return bt_wifi_config;
}

SemaphoreHandle_t wifi_bt_get_semaphore(void) {
    return bt_config_sem;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_num < MAX_RETRY) {
            esp_wifi_connect();
            retry_num++;
            ESP_LOGI(TAG, "Retry to connect...");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Connect to AP failed.");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_bt_start_wifi(const char *ssid, const char *password) {
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    wifi_config_t wifi_config = { 0 };
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(TAG, "WiFi connecting to %s", ssid);
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
}

static void spp_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
        case ESP_SPP_INIT_EVT:
            esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, local_device_name);
            break;
        case ESP_SPP_SRV_OPEN_EVT:
            spp_client_handle = param->srv_open.handle;
            break;
        case ESP_SPP_DATA_IND_EVT: {
            char *buf = malloc(param->data_ind.len + 1);
            if (!buf) return;

            memcpy(buf, param->data_ind.data, param->data_ind.len);
            buf[param->data_ind.len] = '\0';

            // Tìm các dòng chứa SSID, PASS, IP trong chuỗi nhận được
            char *ssid_line = strstr(buf, "SSID:");
            char *pass_line = strstr(buf, "PASS:");
            char *ip_line = strstr(buf, "IP:");

            if (ssid_line && pass_line && ip_line) {
                char ssid[64] = {0};
                char pass[64] = {0};
                char ip[64] = {0};

                sscanf(ssid_line, "SSID:%63[^\n]", ssid);
                sscanf(pass_line, "PASS:%63[^\n]", pass);
                sscanf(ip_line, "IP:%63[^\n]", ip);
    
            if (strlen(ssid) > 0 && strlen(pass) > 0 && strlen(ip) > 0) {
                strncpy(bt_wifi_config.ssid, ssid, sizeof(bt_wifi_config.ssid) - 1);
                strncpy(bt_wifi_config.password, pass, sizeof(bt_wifi_config.password) - 1);
                strncpy(bt_wifi_config.gui_ip, ip, sizeof(bt_wifi_config.gui_ip) - 1);

                ESP_LOGI("WIFI_UART", "SSID: %s\n PASS: %s\n IP: %s\n", bt_wifi_config.ssid, bt_wifi_config.password, bt_wifi_config.gui_ip);

                // Báo đã nhận được cấu hình
                xSemaphoreGive(bt_config_sem);

                // Gửi phản hồi thành công cho GUI
                const char *response = "OK: WiFi config received\n";
                esp_spp_write(spp_client_handle, strlen(response), (uint8_t *)response);
                }            
            }
            free(buf);
            break;
        }
        default:
            break;
    }
}

static void bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
    ESP_LOGI(TAG, "GAP event: %d", event);
}

void wifi_bt_init(void) {
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    esp_bluedroid_init();
    esp_bluedroid_enable();

    esp_bt_gap_register_callback(bt_gap_cb);
    esp_spp_register_callback(spp_callback);

    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = true,
        .tx_buffer_size = 0
    };
    esp_spp_enhanced_init(&spp_cfg);

    esp_bt_dev_set_device_name(local_device_name);
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    if (!bt_config_sem) {
        bt_config_sem = xSemaphoreCreateBinary();
    }

    ESP_LOGI(TAG, "Bluetooth ready");
}

int wifi_bt_tcp_connect(const char *ip, uint16_t port) {
    struct sockaddr_in dest_addr = {
        .sin_addr.s_addr = inet_addr(ip),
        .sin_family = AF_INET,
        .sin_port = htons(port),
    };

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE("TCP_CONNECT", "Unable to create socket: errno %d", errno);
        return -1;
    }

    // Thiết lập timeout 5 giây cho connect
    struct timeval timeout;
    timeout.tv_sec = 5;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    ESP_LOGI("TCP_CONNECT", "Connecting to %s:%d...", ip, port);
    if (connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr)) != 0) {
        ESP_LOGE("TCP_CONNECT", "Socket unable to connect: errno %d", errno);
        close(sock);
        return -1;
    }

    ESP_LOGI("TCP_CONNECT", "Successfully connected to %s:%d", ip, port);
    return sock;
}


