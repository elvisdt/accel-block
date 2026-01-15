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
#include "nvs.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "mcp4725.h"
#include "button.h"




#define GPIO_OUT_RELE       GPIO_NUM_4
#define GPIO_OUT_LED        GPIO_NUM_3
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUT_RELE) | (1ULL<<GPIO_OUT_LED))


#define GPIO_IN_BTN_AVL     GPIO_NUM_5
#define GPIO_IN_BTN_CFG     GPIO_NUM_10
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_IN_BTN_AVL | 1ULL<<GPIO_IN_BTN_CFG)




// Configuración de pines según tu esquemático
#define I2C_MASTER_SCL_IO           7    // Pin SCL
#define I2C_MASTER_SDA_IO           6    // Pin SDA
#define I2C_MASTER_NUM              I2C_NUM_0

// Direcciones I2C de tus dos chips MCP4725
#define MCP4725_U1_ADDR             0x60 
#define MCP4725_U2_ADDR             0x61 

#define NVS_NAMESPACE               "app"
#define NVS_KEY_U1_V                "dac_u1_v"
#define NVS_KEY_U2_V                "dac_u2_v"



#define TAG "GPIO"

#define VDD_MAX     5.0f  // Voltaje de referencia del DAC
#define VECU_HIGH   1.25f
#define VECU_LOW    0.50f


static i2c_dev_t dev_u1;
static i2c_dev_t dev_u2;
static bool dac_ready = false;

static button_t btn_avl, btn_cfg;

//------------------------------------------------------------//
static float clamp_voltage(float v){
    if (v < 0.0f) return 0.0f;
    if (v > VDD_MAX) return VDD_MAX;
    return v;
}
//------------------------------------------------------------//

static void init_dac(i2c_dev_t *dev, uint8_t addr)
{
    memset(dev, 0, sizeof(*dev));

    ESP_ERROR_CHECK(mcp4725_init_desc(dev, addr, I2C_MASTER_NUM,
                                      I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));

    mcp4725_power_mode_t pm;
    ESP_ERROR_CHECK(mcp4725_get_power_mode(dev, false, &pm));
    if (pm != MCP4725_PM_NORMAL)
    {
        printf("DAC 0x%02x estaba en sleep, despertando...\n", addr);
        ESP_ERROR_CHECK(mcp4725_set_power_mode(dev, false, MCP4725_PM_NORMAL));
    }
}

static void apply_dac_values(float v_u1, float v_u2, bool save_nvs)
{
    float u1 = clamp_voltage(v_u1);
    float u2 = clamp_voltage(v_u2);

    ESP_ERROR_CHECK(mcp4725_set_voltage(&dev_u1, VDD_MAX, u1, false));
    ESP_ERROR_CHECK(mcp4725_set_voltage(&dev_u2, VDD_MAX, u2, false));


}

void app_set_dac_voltage(float u1, float u2)
{
    if (!dac_ready) {
        ESP_LOGW(TAG, "DAC no listo todavia");
        return;
    }

    apply_dac_values(u1, u2, true);
}
//------------------------------------------------------------//

void on_sensor_callback(button_t *btn, button_state_t state) {

    ESP_LOGW("SEN", "BTN[%02d] -> %02d ", btn->gpio, state);


    if(btn->gpio == GPIO_IN_BTN_AVL) {
        ESP_LOGI("SEN", "BTN AVL");
        if(state == BUTTON_PRESSED) {
            ESP_LOGW("SEN", "BTN AVL PRESSED - RELE ON");
            ESP_LOGI("SEN", "V01:%.02f, V02:%.02f", VECU_LOW, VECU_HIGH);
            gpio_set_level(GPIO_OUT_RELE, 1);
        } else if(state == BUTTON_RELEASED) {
            ESP_LOGW("SEN", "BTN AVL RELEASED - RELE OFF");
            gpio_set_level(GPIO_OUT_RELE, 0);
        }
    }

    if(btn->gpio == GPIO_IN_BTN_CFG) {
        ESP_LOGI("SEN", "BTN CFG");
        if(state == BUTTON_PRESSED_LONG) {
            ESP_LOGE("SEN", "BTN CFG LONG PRESS - TOGGLE LED");
        }
    }
}


//------------------------------------------------------------//
void init_inputs_and_outputs() {

    ESP_LOGI(TAG, "Init config GPIO...");
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_set_level(GPIO_OUT_RELE, 0);
    gpio_set_level(GPIO_OUT_LED, 0);

    ESP_LOGI(TAG, "Init config BUTTON...");
    btn_avl.gpio = GPIO_IN_BTN_AVL;
    btn_avl.pressed_level = 0;
    btn_avl.internal_pull = true;
    btn_avl.autorepeat = false;
    btn_avl.callback = on_sensor_callback;

    btn_cfg.gpio = GPIO_IN_BTN_CFG;
    btn_cfg.pressed_level = 0;
    btn_cfg.internal_pull = true;
    btn_cfg.autorepeat = false;
    btn_cfg.callback = on_sensor_callback;


    button_init(&btn_avl);
    button_init(&btn_cfg);

}

// Tarea principal
void task_mcpu_demo(void *pvParameters)
{
    (void)pvParameters;

    init_dac(&dev_u1, MCP4725_U1_ADDR);
    init_dac(&dev_u2, MCP4725_U2_ADDR);

    apply_dac_values(VECU_LOW, VECU_HIGH, false);
    dac_ready = true;
    ESP_LOGE(TAG, "DAC init: U1=%.02f V, U2=%.02f V", VECU_LOW, VECU_HIGH);

    while (1){
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

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
    init_inputs_and_outputs();



    // Init i2cdev library
    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(task_mcpu_demo, "demo-task", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
    
}
