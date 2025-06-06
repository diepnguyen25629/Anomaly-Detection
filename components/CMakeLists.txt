idf_component_register(
    SRCS "my_i2c.c"
         "mpu6500.c"
         "bmp280.c"
         "inmp441_i2s.c"
         "sd_card_spi.c"
         "ds18b20_1_wire.c"
         "wifi_bt.c"
    INCLUDE_DIRS "include"
    REQUIRES driver fatfs vfs esp_rom esp_timer esp_wifi esp_event nvs_flash bt 
)
