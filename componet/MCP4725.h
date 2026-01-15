#ifndef MCP4725_CONTROL_H
#define MCP4725_CONTROL_H

#include "driver/i2c_master.h"

// Configuración de pines según tu esquemático
#define I2C_MASTER_SCL_IO           7    // Pin SCL
#define I2C_MASTER_SDA_IO           6    // Pin SDA
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TIMEOUT_MS       1000

// Direcciones I2C de tus dos chips MCP4725
#define MCP4725_U1_ADDR             0x60 // ADDR pin a GND
#define MCP4725_U2_ADDR             0x61 // ADDR pin a VCC

// Prototipos de funciones
esp_err_t i2c_bus_init(i2c_master_bus_handle_t *bus_handle);
esp_err_t mcp4725_add_device(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t *dev_handle, uint8_t dev_addr);
esp_err_t mcp4725_set_voltage(i2c_master_dev_handle_t dev_handle, uint16_t voltage_12bit);

#endif