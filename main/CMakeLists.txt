file(GLOB_RECURSE app_sources "main.cpp" "src/*.cpp" "src/*.c")

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_driver_mcpwm esp_driver_pcnt esp_driver_ledc esp_timer esp_wifi lwip nvs_flash esp_http_server esp_adc
    EMBED_TXTFILES "src/wireless/index.html"
)

# Keep compiler diagnostics visible without failing the build on warnings.
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error)
