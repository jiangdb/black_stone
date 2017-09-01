/* Non-Volatile Storage (NVS) Read and Write a Value - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "config.h"

static nvs_handle config_handle;

void config_init()
{
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
    printf("\n");
    printf("Opening Non-Volatile Storage (NVS) handle... \n");
    err = nvs_open("storage", NVS_READWRITE, &config_handle);
    if (err != ESP_OK) {
        printf("Error (%d) opening NVS handle!\n", err);
    }
}

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
    // printf("Committing updates in NVS ... \n");
    err = nvs_commit(config_handle);
    return err == ESP_OK;
}

void config_close()
{
    nvs_close(config_handle);
}