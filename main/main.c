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

extern void display_init();
extern void adc_init();
extern void bt_init();
extern void setDisplayInteger(uint8_t displayNum, uint32_t value);
extern uint32_t channel_values[2];

static const char *TAG = "black_stone";

void app_main()
{
    // printf("BLACK STONE!!!\n");
	ESP_LOGD(TAG, "Start!!!");

    /* Initialise wifi */
    //wifi_init();

    /* Initialise bluetooth */
    bt_init();

    /* Initialise adc */
    adc_init();

    /* Initialise display */
    display_init();

    while(1) {

    	for (int i=0; i<2; i++) {
       		setDisplayInteger(i , channel_values[i]);
    	}

    	vTaskDelay(50/portTICK_RATE_MS);
    }
}