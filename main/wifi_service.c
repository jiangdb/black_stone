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

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;
static const int CONNECTED_BIT = BIT0;
static const int DISCONNECTED_BIT = BIT1;

static uint8_t wifi_status = WIFI_STATUS_UNSTARTED;
static TaskHandle_t xHandle = NULL;
static SemaphoreHandle_t reconnectSem = NULL;

static void ws_start();

esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            ESP_LOGD(TAG, "SYSTEM_EVENT_STA_START");
            wifi_status = WIFI_STATUS_CONNECTING;
            ESP_ERROR_CHECK(esp_wifi_connect());
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            ESP_LOGD(TAG, "SYSTEM_EVENT_STA_GOT_IP");
            ESP_LOGD(TAG, "got ip:%s\n",ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
            if (wifi_status != WIFI_STATUS_CONNECTED) {
                wifi_status = WIFI_STATUS_CONNECTED;
                bt_notify_wifi_status(wifi_status);
                display_seticon(ICON_WIFI, true);
            }
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            {
                system_event_sta_disconnected_t *disconnected = &event->event_info.disconnected;
                ESP_LOGD(TAG, "SYSTEM_EVENT_STA_DISCONNECTED, ssid:%s, ssid_len:%d, bssid:" MACSTR ", reason:%d", \
                       disconnected->ssid, disconnected->ssid_len, MAC2STR(disconnected->bssid), disconnected->reason);
                if (wifi_status != WIFI_STATUS_DISCONNECTED) {
                    wifi_status = WIFI_STATUS_DISCONNECTED;
                    bt_notify_wifi_status(wifi_status);
                    display_seticon(ICON_WIFI, false);
                }
                //manually disconnected, do reconnect immediately
                if (disconnected->reason == WIFI_REASON_ASSOC_LEAVE) {
                    wifi_status = WIFI_STATUS_CONNECTING;
                    ESP_ERROR_CHECK(esp_wifi_connect());
                }else{
                    xSemaphoreGive(reconnectSem);
                }
            }
            break;
        default:
            break;
    }
    return ESP_OK;
}

void ws_connect_loop()
{
    while(1) {
        xSemaphoreTake( reconnectSem, portMAX_DELAY );

        //reconnect every 30s
        vTaskDelay(30000/portTICK_RATE_MS);
        if (wifi_status == WIFI_STATUS_DISCONNECTED) {
            wifi_status = WIFI_STATUS_CONNECTING;
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

static bool ws_same_config(char* ssid, char* pass)
{
    wifi_config_t sta_config = {
        .sta = {
            .bssid_set = false
        }
    };
    esp_err_t err = esp_wifi_get_config(WIFI_IF_STA, &sta_config);
    if (err == ESP_OK) {
        ESP_LOGD(TAG, "current ssid: %s, pass %s", sta_config.sta.ssid, sta_config.sta.password);
        if ((strcmp((char*)sta_config.sta.ssid, ssid) == 0) && (strcmp((char*)sta_config.sta.password, pass) == 0)){
            ESP_LOGD(TAG, "same config");
            return true;
        }
    }
    return false;
}

static void ws_config(char* ssid, char* pass)
{
    wifi_config_t sta_config = {
        .sta = {
            .bssid_set = false
        }
    };

    memset(sta_config.sta.ssid,0,sizeof(sta_config.sta.ssid));
    memcpy(sta_config.sta.ssid,ssid,strlen(ssid));

    memset(sta_config.sta.password,0,sizeof(sta_config.sta.password));
    memcpy(sta_config.sta.password,pass,strlen(pass));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
}

void ws_connect(char* ssid, char* pass)
{
    if (!ws_valid_config(ssid, pass)) return;

    switch(wifi_status) {
        case WIFI_STATUS_UNSTARTED:
            ws_start();
            ws_config(ssid, pass);
            break;
        case WIFI_STATUS_STARTING:
            ws_config(ssid, pass);
            break;
        case WIFI_STATUS_DISCONNECTED:
            ws_config(ssid, pass);
            wifi_status = WIFI_STATUS_CONNECTING;
            ESP_ERROR_CHECK(esp_wifi_connect());
            break;
        case WIFI_STATUS_CONNECTING:
        case WIFI_STATUS_CONNECTED:
            if (!ws_same_config(ssid, pass)) {
                ws_config(ssid, pass);
                wifi_status = WIFI_STATUS_DISCONNECTING;
                ESP_ERROR_CHECK(esp_wifi_disconnect());
            }
            break;
        default:
            break;
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

static void ws_start()
{
    ESP_LOGD(TAG, "%s", __func__);
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_ps(WIFI_PS_MODEM) );
    wifi_status = WIFI_STATUS_STARTING;
    ESP_ERROR_CHECK( esp_wifi_start() );

    //Create re-connect task
    reconnectSem=xSemaphoreCreateBinary();
    xTaskCreate(&ws_connect_loop, "wifi_service_connect_loop", 2048, NULL, 8, &xHandle);
}

void ws_init()
{
    //char* ssid = config_get_wifi_name();
    //char* pass = config_get_wifi_pass();
    char* ssid = "8Sian-K";
    char* pass = "8sianmedianetwork";
    if (!ws_valid_config(ssid, pass)) return;

    ws_start();
    ws_config(ssid, pass);
}
