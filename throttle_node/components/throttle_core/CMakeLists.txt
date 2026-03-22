idf_component_register(SRCS "throttle.c"
                       INCLUDE_DIRS "include"
                       REQUIRES can_manager driver freertos)
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wall" "-Wextra" "-Wpedantic")
