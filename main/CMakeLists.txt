# Main component CMakeLists
idf_component_register(
    SRCS 
        "main.c"
    PRIV_REQUIRES 
        stepper_motor 
        time_service
        nvs_flash
        rtc 
        lcd 
        led 
        buzzer 
        ultrasonic 
        relay 
        keypad 
        wifi_mqtt 
        scheduler 
        config 
        utils
        esp_timer
        json
        esp_netif esp_event time_service
)
