idf_component_register(SRCS "throttle.c"
                       INCLUDE_DIRS "include"
                       REQUIRES common driver freertos esp_adc)
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wall" "-Wextra")