# Backend-agnostic sources (HAL consumers + RC failsafe + micro-ROS).
set(MAIN_SRCS
    "main.c"
    "diff_drive.c"
    "rc_failsafe.c"
    "motor_task.c"
    "uros_task.c"
    "uros_transport_usb_jtag.c"
)

# Backend-specific sources, gated by the Kconfig MOTOR_DRIVER_BACKEND choice.
# Exactly one of these blocks contributes.
if(CONFIG_MOTOR_DRIVER_VESC)
    list(APPEND MAIN_SRCS
        "vesc_can.c"
        "motor_driver_vesc.c"
        # tune_cli depends on motor_driver_vesc.h (ERPM, per-VESC health),
        # so it is also gated to the VESC backend for now.
        "tune_cli.c"
        "tune_transport_uart.c"
        "tune_transport_wifi.c"
    )
endif()

if(CONFIG_MOTOR_DRIVER_ZLAC8015D)
    list(APPEND MAIN_SRCS
        "canopen.c"
        "cia402.c"
        "motor_driver_zlac8015d.c"
    )
endif()

idf_component_register(
    SRCS ${MAIN_SRCS}
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer micro_ros_espidf_component
             esp_wifi esp_event esp_netif nvs_flash lwip
)
