idf_component_register(SRCS "motor_pwm.c" "motor_can.c" "motor_control.c"
                       INCLUDE_DIRS "include"
                       REQUIRES can_manager driver freertos)
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wall" "-Wextra" "-Wpedantic")
