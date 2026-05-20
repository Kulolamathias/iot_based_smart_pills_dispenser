idf_component_register(
    SRCS 
        "src/wifi_manager.c"
        "src/mqtt_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES 
        nvs_flash 
        esp_netif 
        esp_event 
        esp_wifi 
        efuse
        esp_timer
        mqtt
)
