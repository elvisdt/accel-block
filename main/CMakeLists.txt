set(srcs "main.c")

set(ota_ble_srcs  
    "ble/gap.c"
    "ble/gatt_svr.c")

idf_component_register(
    SRCS ${srcs} ${ota_ble_srcs}
    INCLUDE_DIRS "." "ble"
    REQUIRES 
        nvs_flash 
        driver 
        app_update
        bt
        esp-idf-lib__mcp4725
        esp-idf-lib__i2cdev
        esp-idf-lib__esp_idf_lib_helpers
        button
)
