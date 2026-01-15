#include "MCP4725.h"
#include "esp_log.h"

#define TAG "MCP4725";

esp_err_t i2c_bus_init(i2c_master_bus_handle_t *bus_handle) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false, // Tu PCB ya tiene R7/R10 
    };
    return i2c_new_master_bus(&bus_config, bus_handle);
}

esp_err_t mcp4725_add_device(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *dev_handle, uint8_t dev_addr) {
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    return i2c_master_bus_add_device(bus_handle, &dev_config, dev_handle);
}

/**
 * Establece el voltaje usando el "Fast Mode" del MCP4725
 */
esp_err_t mcp4725_set_voltage(i2c_master_dev_handle_t dev_handle, uint16_t voltage_12bit) {
    if (voltage_12bit > 4095) voltage_12bit = 4095;

    // Trama de 2 bytes para modo rápido (Fast Mode)
    // Byte 1: [0][0][PD1][PD0][D11][D10][D9][D8]
    // Byte 2: [D7][D6][D5][D4][D3][D2][D1][D0]
    uint8_t write_buf[2];
    write_buf[0] = (uint8_t)((voltage_12bit >> 8) & 0x0F); 
    write_buf[1] = (uint8_t)(voltage_12bit & 0xFF);

    return i2c_master_transmit(dev_handle, write_buf, 2, I2C_MASTER_TIMEOUT_MS);
}