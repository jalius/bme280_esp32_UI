idf_component_register(SRCS "main.c"
                    REQUIRES u8g2-hal-esp-idf nvs_flash spi_flash bme280_esp32 esp_timer
                    INCLUDE_DIRS "")
