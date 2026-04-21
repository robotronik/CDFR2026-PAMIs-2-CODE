file(GLOB_RECURSE app_sources "main.cpp" "src/*.cpp" "src/*.c")

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_mcpwm esp_driver_pcnt esp_driver_ledc
)
