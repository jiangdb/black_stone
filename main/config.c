/* Non-Volatile Storage (NVS) Read and Write a Value - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "config.h"

#define TAG "CONFIG"

#define KEY_ZERO_TRACK          "zero track"
#define KEY_ALARM_ENABLE        "alarm enable"
#define KEY_ALARM_TIME          "alarm time"
#define KEY_ALARM_WEIGHT        "alarm weight"
#define KEY_ALARM_WEIGHT_UNIT   "alarm weight unit"
#define KEY_WEIGHT_UNIT         "weight unit"
#define KEY_WIFI_NAME           "wifi name"
#define KEY_WIFI_PASS           "wifi pass"

typedef struct {
    uint8_t zero_track;
    uint8_t alarm_enable;
    uint16_t alarm_time;
    uint16_t alarm_weight;
    uint8_t alarm_weight_unit;
    uint8_t weight_unit;
    char* wifi_name;
    char* wifi_pass;
    firmware_t firmware;
}system_settings_t;

static nvs_handle config_handle;
static system_settings_t system_settings;

int32_t config_read(char* name, int32_t default_value)
{
    int32_t value = default_value;
    nvs_get_i32(config_handle, name, &value);
    return value;
}

bool config_write(char* name, int32_t value)
{
    esp_err_t err;
    err = nvs_set_i32(config_handle, name, value);
    printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    err = nvs_commit(config_handle);
    return err == ESP_OK;
}

uint8_t config_get_zero_track()
{
    return system_settings.zero_track;
}

// num must be 7 bytes
void config_get_serial_num(char* serial_num, int len)
{
    //Use mac as serial number
    uint8_t mac[6];
    memset(mac, 0, sizeof(mac));
    esp_efuse_mac_get_default(mac);
    memset(serial_num, 0, len);
    sprintf(serial_num, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

bool config_set_zero_track(uint8_t enable)
{
    ESP_LOGD(TAG, "%s: %d\n", __func__, enable);
    if (enable != system_settings.zero_track) {
        system_settings.zero_track = enable;
        esp_err_t err = nvs_set_u8(config_handle, KEY_ZERO_TRACK, enable);
        if (err != ESP_OK) return false;
    }
    return true ;
}

uint8_t config_get_alarm_enable()
{
    return system_settings.alarm_enable;
}

bool config_set_alarm_enable(uint8_t enable)
{
    ESP_LOGD(TAG, "%s: %d\n", __func__, enable);
    if (enable != system_settings.alarm_enable) {
        system_settings.alarm_enable = enable;
        esp_err_t err = nvs_set_u8(config_handle, KEY_ALARM_ENABLE, enable);
        if (err != ESP_OK) return false;
    }
    return true ;
}

uint16_t config_get_alarm_time()
{
    return system_settings.alarm_time;
}

bool config_set_alarm_time(uint16_t timeInSeconds)
{
    ESP_LOGD(TAG, "%s: %d\n", __func__, timeInSeconds);
    if (timeInSeconds != system_settings.alarm_time) {
        system_settings.alarm_time = timeInSeconds;
        esp_err_t err = nvs_set_u16(config_handle, KEY_ALARM_TIME, timeInSeconds);
        if (err != ESP_OK) return false;
    }
    return true;
}

uint16_t config_get_alarm_weight()
{
    //should be consistant with unit;
    return system_settings.alarm_weight * 10;
}

bool config_set_alarm_weight(uint16_t weight)
{
    ESP_LOGD(TAG, "%s: %d\n", __func__, weight);
    if (weight != system_settings.alarm_weight) {
        system_settings.alarm_weight = weight;
        esp_err_t err = nvs_set_u16(config_handle, KEY_ALARM_WEIGHT, weight);
        if (err != ESP_OK) return false;
    }
    return true;
}

uint8_t config_get_weight_unit()
{
    return system_settings.weight_unit;
}

bool config_set_weight_unit(uint8_t unit)
{
    ESP_LOGD(TAG, "%s: %d\n", __func__, unit);
    if (unit != system_settings.weight_unit) {
        system_settings.weight_unit = unit;
        esp_err_t err = nvs_set_u8(config_handle, KEY_WEIGHT_UNIT, unit);
        if (err != ESP_OK) return false;
    }
    return true;
}

char* config_get_wifi_name()
{
    return system_settings.wifi_name;
}

bool config_set_wifi_name(char* name, size_t len)
{
    free(system_settings.wifi_name);
    system_settings.wifi_name = malloc(len+1);
    memset(system_settings.wifi_name, 0, len+1);      //make sure end up with 0
    memcpy(system_settings.wifi_name, name, len);
    ESP_LOGD(TAG, "%s: %s\n", __func__, system_settings.wifi_name);
    esp_err_t err = nvs_set_str(config_handle, KEY_WIFI_NAME, system_settings.wifi_name);
    return err == ESP_OK;
}

char* config_get_wifi_pass()
{
    return system_settings.wifi_pass;
}

bool config_set_wifi_pass(char* pass, size_t len)
{
    free(system_settings.wifi_pass);
    system_settings.wifi_pass = malloc(len+1);
    memset(system_settings.wifi_pass, 0, len+1);      //make sure end up with 0
    memcpy(system_settings.wifi_pass, pass, len);
    esp_err_t err = nvs_set_str(config_handle, KEY_WIFI_PASS, system_settings.wifi_pass);
    return err == ESP_OK;
}

firmware_t* config_get_firmware_upgrade()
{
    return &system_settings.firmware;
}

bool config_set_firmware_upgrade(char* host, uint8_t host_len, uint8_t port, char* path, uint8_t path_len)
{
    free(system_settings.firmware.host);
    free(system_settings.firmware.path);
    system_settings.firmware.host = malloc(host_len);
    system_settings.firmware.path = malloc(path_len);
    memset(system_settings.firmware.host, 0, host_len+1);      //make sure end up with 0
    memset(system_settings.firmware.path, 0, path_len+1);      //make sure end up with 0
    memcpy(system_settings.firmware.host, host, host_len);
    memcpy(system_settings.firmware.path, path, path_len);
    system_settings.firmware.port = port;

    ESP_LOGD(TAG, "%s set firmware host: %s, port: %d, path:%s\n", 
            __func__, system_settings.firmware.host, system_settings.firmware.port, system_settings.firmware.path);
    return true;
}

void config_close()
{
    free(system_settings.wifi_name);
    free(system_settings.wifi_pass);
    nvs_commit(config_handle);
    nvs_close(config_handle);
}

void config_init()
{
    ESP_LOGD(TAG, "%s\n", __func__);
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        // NVS partition was truncated and needs to be erased
        const esp_partition_t* nvs_partition = esp_partition_find_first(
                ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
        assert(nvs_partition && "partition table must have an NVS partition");
        ESP_ERROR_CHECK( esp_partition_erase_range(nvs_partition, 0, nvs_partition->size) );
        // Retry nvs_flash_init
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // Opening
    err = nvs_open("storage", NVS_READWRITE, &config_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "(%d) opening NVS handle!\n", err);
    }

    //Device Info
    char serial_num[20];
    config_get_serial_num(serial_num, 20);
    ESP_LOGI(TAG, "%s: serial_num: %s", __func__, serial_num);

    //System Setting
    memset(&system_settings,0,sizeof(system_settings));
    system_settings.firmware.host = NULL;
    system_settings.firmware.path = NULL;

    //zero track enable
    err = nvs_get_u8(config_handle, KEY_ZERO_TRACK, &system_settings.zero_track);
    assert(err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND);
    ESP_LOGI(TAG, "%s: zero_track: %d", __func__, system_settings.zero_track);

    //alarm enable
    err = nvs_get_u8(config_handle, KEY_ALARM_ENABLE, &system_settings.alarm_enable);
    assert(err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND);
    ESP_LOGI(TAG, "%s: alarm_enable: %d", __func__, system_settings.alarm_enable);

    //alarm time in seconds
    err = nvs_get_u16(config_handle, KEY_ALARM_TIME, &system_settings.alarm_time);
    assert(err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND);
    ESP_LOGI(TAG, "%s: alarm_time: %d", __func__, system_settings.alarm_time);

    //alarm weight
    err = nvs_get_u16(config_handle, KEY_ALARM_WEIGHT, &system_settings.alarm_weight);
    assert(err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND);
    ESP_LOGI(TAG, "%s: alarm_weight: %d", __func__, system_settings.alarm_weight);

    //alarm weight uint
    err = nvs_get_u8(config_handle, KEY_ALARM_WEIGHT_UNIT, &system_settings.alarm_weight_unit);
    assert(err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND);
    ESP_LOGI(TAG, "%s: alarm_weight_unit: %d", __func__, system_settings.alarm_weight_unit);

    //weight uint
    err = nvs_get_u8(config_handle, KEY_WEIGHT_UNIT, &system_settings.weight_unit);
    assert(err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND);
    ESP_LOGI(TAG, "%s: weight_unit: %d", __func__, system_settings.weight_unit);

    //wifi name
    size_t required_size = 0;
    err = nvs_get_str(config_handle, KEY_WIFI_NAME, NULL, &required_size);
    assert(err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND);
    if (err == ESP_OK) {
        system_settings.wifi_name = malloc(required_size);
        nvs_get_str(config_handle, KEY_WIFI_NAME, system_settings.wifi_name, &required_size);
        ESP_LOGI(TAG, "%s: wifi_name: %s", __func__, system_settings.wifi_name);
    }else{
        system_settings.wifi_name = NULL;
        ESP_LOGI(TAG, "%s: wifi_name is empty", __func__);
    }

    //wifi pass
    required_size = 0;
    err = nvs_get_str(config_handle, KEY_WIFI_PASS, NULL, &required_size);
    assert(err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND);
    if (err == ESP_OK) {
        system_settings.wifi_pass = malloc(required_size);
        nvs_get_str(config_handle, KEY_WIFI_PASS, system_settings.wifi_pass, &required_size);
        ESP_LOGI(TAG, "%s: wifi_pass: %s\n", __func__, system_settings.wifi_pass);
    }else{
        system_settings.wifi_pass = NULL;
        ESP_LOGI(TAG, "%s: wifi_pass is empty\n", __func__);
    }
}
