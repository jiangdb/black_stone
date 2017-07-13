/* 
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "bt.h"

static const char *TAG = "black_stone";

esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_START:
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
            ESP_ERROR_CHECK(esp_wifi_connect());
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
            ESP_LOGI(TAG, "got ip:%s\n",ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "SYSTEM_EVENT_STA_DISCONNECTED");
            ESP_ERROR_CHECK(esp_wifi_connect());
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void wifi_init()
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = "8SIAN2F",
            .password = "8sianmedianetwork",
            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

static void bs_main_task(void *pvParameters)
{

}

static void bt_init()
{
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    
    if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
        ESP_LOGI(TAG, "Bluetooth controller initialize failed");
        return;
    }

    if (esp_bt_controller_enable(ESP_BT_MODE_BTDM) != ESP_OK) {
        ESP_LOGI(TAG, "Bluetooth controller enable failed");
        return;
    }
}

void app_main()
{
    printf("BLACK STONE!!!\n");

    /* Initialise wifi */
    wifi_init();

    /* Initialise bluetooth */
    //bt_init();

    /* Start main task */
    //xTaskCreate(&bs_main_task, "bs_main_task", 4096, NULL, 5, NULL);

}