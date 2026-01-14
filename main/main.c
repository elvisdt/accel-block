/*
 * SPDX-FileCopyrightText: 2020-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "MCP4725.h"


#include "esp_log.h"



#define GPIO_OUT_RELE       GPIO_NUM_4
#define GPIO_OUT_LED        GPIO_NUM_3
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUT_RELE) | (1ULL<<GPIO_OUT_LED))


#define TAG "GPIO"

void app_main(void)
{
    ESP_LOGI(TAG, "Init config GPIO...");
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);


    // i2c_master_bus_config_t i2c_bus_config = {
    //     .i2c_port = I2C_MASTER_NUM,
    //     .sda_io_num = I2C_MASTER_SDA_IO,
    //     .scl_io_num = I2C_MASTER_SCL_IO,
    //     .clk_source = I2C_CLK_SRC_DEFAULT,
    //     .glitch_ignore_cnt = 7,
    //     .flags.enable_internal_pullup = false, // Tu PCB ya tiene R7/R10 
    // };

    i2c_master_bus_handle_t i2c_bus_handle = NULL;



    esp_err_t ret = i2c_bus_init(&i2c_bus_handle);
    ESP_LOGI(TAG, "i2c_bus_init: %s", esp_err_to_name(ret));

    i2c_master_dev_handle_t mcp4725_u1;
    ret = mcp4725_add_device(i2c_bus_handle, &mcp4725_u1, MCP4725_U1_ADDR);
    ESP_LOGI(TAG, "mcp4725_add_device U1: %s", esp_err_to_name(ret));


    ret = mcp4725_set_voltage(mcp4725_u1, 2048);
    ESP_LOGI(TAG, "mcp4725_set_voltage U1: %s", esp_err_to_name(ret));


    // int cnt = 0;
    // while (1) {
    //     ESP_LOGI(TAG,"cnt: %d", cnt++);
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    //     gpio_set_level(GPIO_OUT_RELE, cnt % 2);
    //     // gpio_set_level(GPIO_OUT_LED, cnt % 2);
    // }
}