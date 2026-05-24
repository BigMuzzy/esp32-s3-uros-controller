idf_component_register(
    SRCS "main.c"
         "vesc_can.c"
         "diff_drive.c"
         "rc_failsafe.c"
         "can_task.c"
         "uros_task.c"
         "uros_transport_usb_jtag.c"
         "tune_cli.c"
         "tune_transport_uart.c"
         "tune_transport_wifi.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer micro_ros_espidf_component
             esp_wifi esp_event esp_netif nvs_flash lwip
)
