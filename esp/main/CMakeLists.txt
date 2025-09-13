idf_component_register(
    SRCS "bluedroid_ble.c" "ultrasonic.c" "i2c_master.c" "ina.c" "encoder.c" "imu.c" "utils.c" "motor_pwm.c" "jetson_uart.c" "main.c"
    INCLUDE_DIRS "."
    REQUIRES driver nvs_flash bt
)