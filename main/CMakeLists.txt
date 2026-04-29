idf_component_register(SRC_DIRS "."
                       PRIV_INCLUDE_DIRS "."
                       PRIV_REQUIRES vfs nvs_flash esp_timer led_strip)

if(CONFIG_UNITY)
    idf_component_register(SRCS "../test/test_valve_driver.c" INCLUDE_DIRS ".." REQUIRES unity)
endif()
