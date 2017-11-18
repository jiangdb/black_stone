/* 
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "wifi_service.h"
#include "gatts_service.h"
#include "http_request.h"
#include "display.h"
#include "config.h"

#define TAG "WIFI_SERVICE"

#define CONNECTED_BIT       BIT0
#define DISCONNECTED_BIT    BIT1
#define RECONNECT_BIT       BIT2
#define EVENT_GROUP  (CONNECTED_BIT | DISCONNECTED_BIT | RECONNECT_BIT)

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;
static uint8_t wifi_status = WIFI_STATUS_UNSTARTED;
static TaskHandle_t xHandle = NULL;

void ws_task_loop();
static void ws_start();

esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            ESP_LOGD(TAG, "SYSTEM_EVENT_STA_START");
            wifi_event_group = xEventGroupCreate();
            xTaskCreate(&ws_task_loop, "wifi_service_task_loop", 8192, NULL, 8, &xHandle);
            wifi_status = WIFI_STATUS_CONNECTING;
            ESP_ERROR_CHECK(esp_wifi_connect());
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            ESP_LOGD(TAG, "SYSTEM_EVENT_STA_GOT_IP");
            ESP_LOGD(TAG, "got ip:%s\n",ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            {
                system_event_sta_disconnected_t *disconnected = &event->event_info.disconnected;
                ESP_LOGD(TAG, "SYSTEM_EVENT_STA_DISCONNECTED, ssid:%s, ssid_len:%d, bssid:" MACSTR ", reason:%d", \
                       disconnected->ssid, disconnected->ssid_len, MAC2STR(disconnected->bssid), disconnected->reason);
                if (disconnected->reason == WIFI_REASON_ASSOC_LEAVE) {
                    //manually disconnected, do reconnect immediately
                    xEventGroupSetBits(wifi_event_group, RECONNECT_BIT);
                }else{
                    xEventGroupSetBits(wifi_event_group, DISCONNECTED_BIT);
                }
            }
            break;
        default:
            break;
    }
    return ESP_OK;
}

void ws_task_loop()
{
    EventBits_t uxBits;
    TickType_t tickToWait = portMAX_DELAY;

    while(1) {
        uxBits = xEventGroupWaitBits(wifi_event_group, EVENT_GROUP, pdTRUE, pdFALSE, tickToWait); 
        tickToWait = portMAX_DELAY;

        ESP_LOGD(TAG, "get Event group %08x", uxBits);
        if(uxBits & CONNECTED_BIT) {
            ESP_LOGD(TAG, "CONNECTED_BIT");
            wifi_status = WIFI_STATUS_CONNECTED;
            display_seticon(ICON_WIFI, true);
            bt_notify_wifi_status(wifi_status);
            device_login();
        } else if (uxBits & DISCONNECTED_BIT) {
            ESP_LOGD(TAG, "DISCONNECTED_BIT");
            wifi_status = WIFI_STATUS_DISCONNECTED;
            display_seticon(ICON_WIFI, false);
            bt_notify_wifi_status(wifi_status);
            //reconnect every 30s
            tickToWait = 30000/portTICK_RATE_MS;
        } else if (uxBits & RECONNECT_BIT) {
            ESP_LOGD(TAG, "RECONNECT_BIT");
            wifi_status = WIFI_STATUS_DISCONNECTED;
            display_seticon(ICON_WIFI, false);
            bt_notify_wifi_status(wifi_status);
            wifi_status = WIFI_STATUS_CONNECTING;
            ESP_ERROR_CHECK(esp_wifi_connect());
        } else {
            ESP_LOGD(TAG, "TIMEOUT");
            if (wifi_status == WIFI_STATUS_DISCONNECTED) {
                wifi_status = WIFI_STATUS_CONNECTING;
                ESP_ERROR_CHECK(esp_wifi_connect());
            }
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
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_ps(WIFI_PS_MODEM) );
    wifi_status = WIFI_STATUS_STARTING;
    ESP_ERROR_CHECK( esp_wifi_start() );
}

void ws_init()
{
    char* ssid = config_get_wifi_name();
    char* pass = config_get_wifi_pass();
    if (!ws_valid_config(ssid, pass)) return;

    ws_start();
    ws_config(ssid, pass);
}
