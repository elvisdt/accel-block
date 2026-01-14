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

#include "nvs_flash.h"


#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "MCP4725.h"


#include "esp_log.h"



#define GPIO_OUT_RELE       GPIO_NUM_4
#define GPIO_OUT_LED        GPIO_NUM_3
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUT_RELE) | (1ULL<<GPIO_OUT_LED))


#define TAG "GPIO"


//--------------

void app_main(void)
{

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);



    ESP_LOGI(TAG, "Init config GPIO...");
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);




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