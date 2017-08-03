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

#define GPIO_LED_IO         19


extern void display_init();
extern void adc_init();
extern void bt_init();
extern void setDisplayInteger(uint8_t displayNum, uint32_t value);
extern void spi_trassfer_display();
extern int32_t channel_values[2];

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

    while(1) {
        vTaskDelay(300/portTICK_RATE_MS);

        for (int i=0; i<2; i++) {
            int32_t value = channel_values[i] + 50;
            if (value < 0) {
                value = 0;
            }
            // printf("%d: %d\n", i, value);
            setDisplayInteger(i , value);
        }

        spi_trassfer_display();
    }
}