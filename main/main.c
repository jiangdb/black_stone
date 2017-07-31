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

void app_main()
{
    printf("BLACK STONE!!!\n");

    /* Initialise wifi */
    //wifi_init();

    /* Initialise bluetooth */
    //bt_init();

    /* Start main task */
    //xTaskCreate(&bs_main_task, "bs_main_task", 4096, NULL, 5, NULL);

}