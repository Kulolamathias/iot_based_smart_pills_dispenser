# Main component CMakeLists
idf_component_register(
    SRCS 
        "main.c"
    PRIV_REQUIRES 
        motor 
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
)
