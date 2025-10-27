idf_component_register(SRCS "main.c" "fonts/ink_free_12.c"
                    INCLUDE_DIRS "." "fonts"
                    REQUIRES lvgl driver esp_timer esp_lcd esp_lcd_ssd1681 esp_lvgl_port)
