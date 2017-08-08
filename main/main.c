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
#include "display.h"
#include "timer.h"
#include "queue_buffer.h"

#define GPIO_LED_IO         19

extern void display_init();
extern void adc_init();
extern void bt_init();
extern int32_t channel_values[2];
extern queue_buffer_t dataQueueBuffer[2];
extern void gpio_key_init();
extern int32_t get_weight(int32_t adcValue);

static const char *TAG = "black_stone";

void led_on()
{
    printf("%s: led gpio_init !!!\n", TAG);
    //GPIO config for the data line.
    gpio_config_t io_conf={
        .intr_type=GPIO_PIN_INTR_DISABLE,
        .mode=GPIO_MODE_OUTPUT,
        .pull_down_en=0,
        .pull_up_en=0,
        .pin_bit_mask=(1<<GPIO_LED_IO)
    };

    //Set up handshake line interrupt.
    gpio_config(&io_conf);
    gpio_set_level(GPIO_LED_IO, 1);
}


void app_main()
{
    printf("BLACK STONE!!!\n");
	ESP_LOGI(TAG, "Start!!!");

    /* led */
    led_on();

    /* Initialise wifi */
    //wifi_init();

    /* Initialise bluetooth */
    // bt_init();

    /* Initialise adc */
    adc_init();

    /* Initialise display */
    display_init();

    /* Initialise key */
    gpio_key_init();

    /* Initialise timer */
    bs_timer_init();

    while(1) {
        vTaskDelay(100/portTICK_RATE_MS);
        // printf("%d\n", queue_average(&dataQueueBuffer[0]));
        int32_t weight = get_weight(queue_average(&dataQueueBuffer[0])/100);
        // int32_t weight = get_weight(channel_values[0]/100);
        setDisplayNumber(0, weight, 0);
        /*
        for (int i = 0; i < 2; ++i)
        {
            int32_t value = queue_average(&dataQueueBuffer[i]);
            printf("%d\n", value);
        }
        */
    }
}