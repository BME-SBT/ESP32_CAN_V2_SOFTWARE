idf_component_register(SRCS "accubox.c"
                       INCLUDE_DIRS "include"
                       REQUIRES common freertos can_manager)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wall" "-Wextra" "-Wpedantic")
