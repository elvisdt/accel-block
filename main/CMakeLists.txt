
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        nvs_flash 
        driver 
        esp-idf-lib__mcp4725
        esp-idf-lib__i2cdev
        esp-idf-lib__esp_idf_lib_helpers
        button
)
