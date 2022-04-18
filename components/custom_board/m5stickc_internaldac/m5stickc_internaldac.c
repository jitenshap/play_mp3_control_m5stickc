/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2020 <ESPRESSIF SYSTEMS (SHANGHAI) CO., LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "board.h"
#include "i2c_bus.h"

#include "m5stickc_internaldac.h"

static const char *TAG = "m5stk_idac";
static i2c_bus_handle_t i2c_handle;

static bool codec_init_flag;

audio_hal_func_t AUDIO_NEW_CODEC_DEFAULT_HANDLE = {
    .audio_codec_initialize = new_codec_init,
    .audio_codec_deinitialize = new_codec_deinit,
    .audio_codec_ctrl = new_codec_ctrl_state,
    .audio_codec_config_iface = new_codec_config_i2s,
    .audio_codec_set_mute = new_codec_set_voice_mute,
    .audio_codec_set_volume = new_codec_set_voice_volume,
    .audio_codec_get_volume = new_codec_get_voice_volume,
};

bool new_codec_initialized()
{
    return codec_init_flag;
}

static int i2c_init()
{
    int res;
    i2c_config_t es_i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    res = get_i2c_pins(I2C_NUM_0, &es_i2c_cfg);
    i2c_handle = i2c_bus_create(I2C_NUM_0, &es_i2c_cfg);
    return res;
}

esp_err_t new_codec_init(audio_hal_codec_config_t *cfg)
{
    const char* TAG = "SPK_INIT";
    ESP_LOGI(TAG, "new_codec init");
    gpio_reset_pin(GPIO_NUM_0);
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_OUTPUT); 
    gpio_set_level(GPIO_NUM_0, 0);

    gpio_reset_pin(GPIO_NUM_37);
    gpio_set_direction(GPIO_NUM_37, GPIO_MODE_INPUT); 
    gpio_pullup_en(GPIO_NUM_37);
    gpio_reset_pin(GPIO_NUM_39);
    gpio_set_direction(GPIO_NUM_39, GPIO_MODE_INPUT);
    gpio_pullup_en(GPIO_NUM_39);

    i2c_init();

    int i2c_master_port = 0;

    //0x34 0x28 0xcc
    ESP_LOGI(TAG, "10");
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AXP_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x10, true);
    i2c_master_write_byte(cmd, 0xff, true);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        ESP_LOGE(TAG, "FAIL %d", ret);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "28");
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AXP_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x28, true);
    i2c_master_write_byte(cmd, 0xff, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        ESP_LOGE(TAG, "FAIL %d", ret);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "82");
     cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AXP_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x82, true);
    i2c_master_write_byte(cmd, 0xff, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        ESP_LOGE(TAG, "FAIL %d", ret);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "33");
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AXP_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x33, true);
    i2c_master_write_byte(cmd, 0xc0, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        ESP_LOGE(TAG, "FAIL %d", ret);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "b8");
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AXP_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0xb8, true);
    i2c_master_write_byte(cmd, 0x80, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        ESP_LOGE(TAG, "FAIL %d", ret);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); 

/*
    ESP_LOGI(TAG, "12");
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AXP_ADDR << 1) | 0x12, true);
    uint8_t rxdata;
    i2c_master_read_byte(cmd, &rxdata, 0x0);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        ESP_LOGE(TAG, "FAIL %d", ret);
    }    
    ESP_LOGI(TAG, "RX=%02x", rxdata);
*/

    ESP_LOGI(TAG, "12");
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AXP_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x12, true);
    i2c_master_write_byte(cmd, (0x4d), true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        ESP_LOGE(TAG, "FAIL %d", ret);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);    

    ESP_LOGI(TAG, "36");
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AXP_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x36, true);
    i2c_master_write_byte(cmd, 0x5c, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        ESP_LOGE(TAG, "FAIL %d", ret);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);    

    ESP_LOGI(TAG, "90");
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AXP_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x90, true);
    i2c_master_write_byte(cmd, 0x07, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        ESP_LOGE(TAG, "FAIL %d", ret);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "31");
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AXP_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x31, true);
    i2c_master_write_byte(cmd, 0x04, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        ESP_LOGE(TAG, "FAIL %d", ret);
    }
    
    vTaskDelay(100 / portTICK_PERIOD_MS); 

    gpio_set_level(GPIO_NUM_0, 1);



    return ESP_OK;
}

esp_err_t new_codec_deinit(void)
{
    return ESP_OK;
}

esp_err_t new_codec_ctrl_state(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state)
{
    return ESP_OK;
}

esp_err_t new_codec_config_i2s(audio_hal_codec_mode_t mode, audio_hal_codec_i2s_iface_t *iface)
{
    return ESP_OK;
}

esp_err_t new_codec_set_voice_mute(bool mute)
{
    return ESP_OK;
}

esp_err_t new_codec_set_voice_volume(int volume)
{
    return ESP_OK;
}

esp_err_t new_codec_get_voice_volume(int *volume)
{
    return ESP_OK;
}
