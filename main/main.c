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
#include <mcp4725.h>



#define GPIO_OUT_RELE       GPIO_NUM_4
#define GPIO_OUT_LED        GPIO_NUM_3
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUT_RELE) | (1ULL<<GPIO_OUT_LED))


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

#define VDD 5.0f  // Voltaje de referencia del DAC



akek
static i2c_dev_t dev_u1;
static i2c_dev_t dev_u2;
static bool dac_ready = false;



static float clamp_voltage(float v)
{
    if (v < 0.0f) return 0.0f;
    if (v > VDD) return VDD;
    return v;
}

static esp_err_t nvs_load_float(const char *key, float *out, float default_value)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return err;

    size_t size = sizeof(float);
    err = nvs_get_blob(handle, key, out, &size);
    if (err == ESP_ERR_NVS_NOT_FOUND || size != sizeof(float)) {
        *out = default_value;
        err = nvs_set_blob(handle, key, out, sizeof(float));
        if (err == ESP_OK)
            err = nvs_commit(handle);
    }

    nvs_close(handle);
    return err;
}

static esp_err_t nvs_save_float(const char *key, float value)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
        return err;

    err = nvs_set_blob(handle, key, &value, sizeof(value));
    if (err == ESP_OK)
        err = nvs_commit(handle);

    nvs_close(handle);
    return err;
}

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

    ESP_ERROR_CHECK(mcp4725_set_voltage(&dev_u1, VDD, u1, false));
    ESP_ERROR_CHECK(mcp4725_set_voltage(&dev_u2, VDD, u2, false));

    if (save_nvs) {
        esp_err_t err = nvs_save_float(NVS_KEY_U1_V, u1);
        if (err != ESP_OK)
            ESP_LOGE(TAG, "NVS save U1 failed: %s", esp_err_to_name(err));
        err = nvs_save_float(NVS_KEY_U2_V, u2);
        if (err != ESP_OK)
            ESP_LOGE(TAG, "NVS save U2 failed: %s", esp_err_to_name(err));
    }
}

void app_set_dac_voltage(float u1, float u2)
{
    if (!dac_ready) {
        ESP_LOGW(TAG, "DAC no listo todavia");
        return;
    }

    apply_dac_values(u1, u2, true);
}


void task(void *pvParameters)
{
    (void)pvParameters;

    init_dac(&dev_u1, MCP4725_U1_ADDR);
    init_dac(&dev_u2, MCP4725_U2_ADDR);

    float v_u1 = 0.0f;
    float v_u2 = 0.0f;
    esp_err_t err = nvs_load_float(NVS_KEY_U1_V, &v_u1, 0.0f);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "NVS load U1 failed: %s", esp_err_to_name(err));
    err = nvs_load_float(NVS_KEY_U2_V, &v_u2, 0.0f);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "NVS load U2 failed: %s", esp_err_to_name(err));

    apply_dac_values(v_u1, v_u2, false);
    dac_ready = true;
    printf("DAC init: U1=%.02f V, U2=%.02f V\n", v_u1, v_u2);

    while (1)
        vTaskDelay(pdMS_TO_TICKS(1000));
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



    ESP_LOGI(TAG, "Init config GPIO...");
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);


    // Init i2cdev library
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreate(task, "test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
    
}
