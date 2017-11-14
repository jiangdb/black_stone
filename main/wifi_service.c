/* 
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "wifi_service.h"
#include "gatts_service.h"
#include "display.h"
#include "config.h"

#define TAG "WIFI_SERVICE"

static uint8_t wifi_status = WIFI_STATUS_UNSTART;
static TaskHandle_t xHandle = NULL;
static SemaphoreHandle_t connectSem = NULL;

esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            ESP_LOGD(TAG, "SYSTEM_EVENT_STA_START");
            wifi_status = WIFI_STATUS_STARTED;
            ESP_ERROR_CHECK(esp_wifi_connect());
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            if (wifi_status != WIFI_STATUS_CONNECTED) {
                wifi_status = WIFI_STATUS_CONNECTED;
                bt_notify_wifi_status(wifi_status);
                //setWifiSound(0, true);
                display_seticon(ICON_WIFI, true);
            }
            ESP_LOGD(TAG, "SYSTEM_EVENT_STA_GOT_IP");
            ESP_LOGD(TAG, "got ip:%s\n",ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            if (wifi_status != WIFI_STATUS_DISCONNECTED) {
                wifi_status = WIFI_STATUS_DISCONNECTED;
                bt_notify_wifi_status(wifi_status);
                //setWifiSound(0, false);
                display_seticon(ICON_WIFI, false);
            }
            ESP_LOGD(TAG, "SYSTEM_EVENT_STA_DISCONNECTED");
            xSemaphoreGive(connectSem);
            break;
        default:
            break;
    }
    return ESP_OK;
}

void ws_connect_loop()
{
    while(1) {
        xSemaphoreTake( connectSem, portMAX_DELAY );

        vTaskDelay(3000/portTICK_RATE_MS);
        if (wifi_status == WIFI_STATUS_STARTED) {
            ESP_ERROR_CHECK(esp_wifi_connect());
        }
    }
}

static bool ws_valid_config(char* ssid, char* pass)
{
    if (ssid == NULL || pass == NULL) return false;

    wifi_sta_config_t sta;
    if ((strlen(ssid)+1) > sizeof(sta.ssid) || (strlen(pass)+1) > sizeof(sta.password)) {
        return false;
    }
    return true;
}

static void ws_config(char* ssid, char* pass)
{
    esp_err_t err;
    wifi_config_t sta_config = {
        .sta = {
            .bssid_set = false
        }
    };

    err = esp_wifi_get_config(WIFI_IF_STA, &sta_config);
    if (err == ESP_OK) {
        ESP_LOGD(TAG, "current ssid: %s, pass %s", sta_config.sta.ssid, sta_config.sta.password);
        if ((strcmp((char*)sta_config.sta.ssid, ssid) == 0) && (strcmp((char*)sta_config.sta.password, pass) == 0)){
            ESP_LOGD(TAG, "same config");
            return;
        }
    }

    memset(sta_config.sta.ssid,0,sizeof(sta_config.sta.ssid));
    memcpy(sta_config.sta.ssid,ssid,strlen(ssid));

    memset(sta_config.sta.password,0,sizeof(sta_config.sta.password));
    memcpy(sta_config.sta.password,pass,strlen(pass));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
}

void ws_connect(char* ssid, char* pass)
{
    if (!ws_valid_config(ssid, pass)) return;

    ws_config(ssid, pass);
    if (wifi_status == WIFI_STATUS_UNSTART) {
        ESP_ERROR_CHECK( esp_wifi_start() );
    } else if(wifi_status == WIFI_STATUS_STARTED) {
        ESP_ERROR_CHECK(esp_wifi_connect());
    } else if(wifi_status == WIFI_STATUS_CONNECTED) {
        ESP_ERROR_CHECK(esp_wifi_disconnect());
    } else if(wifi_status == WIFI_STATUS_DISCONNECTED) {
        ESP_ERROR_CHECK(esp_wifi_connect());
    }
}

uint8_t ws_get_status()
{
    return wifi_status;
}

void ws_stop()
{
    if( xHandle != NULL )
    {
        vTaskDelete( xHandle );
    }
    esp_wifi_stop();
}

void ws_init()
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );

    char* wifi_name = config_get_wifi_name();
    char* wifi_pass = config_get_wifi_pass();
    ws_connect(wifi_name, wifi_pass);

    connectSem=xSemaphoreCreateBinary();
    //Create connect task
    xTaskCreate(&ws_connect_loop, "wifi_service_connect_loop", 2048, NULL, 8, &xHandle);
}
