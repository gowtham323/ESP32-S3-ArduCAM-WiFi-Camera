idf_component_register(SRCS "hello_world_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver led_strip esp_timer spi_flash esp_http_server esp_wifi nvs_flash)
