/*
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "display.h"
#include "key_event.h"

#define ADC1_CHANNEL (6)
#define ADC1_REF     3650
#define BATTERY_PERCENTAGE_75		3900
#define BATTERY_PERCENTAGE_50		3800
#define BATTERY_PERCENTAGE_25		3700
#define BATTERY_PERCENTAGE_0 		3500
#define BATTERY_WARNING				3500

extern void handle_key_event(key_event_t keyEvent);

void battery_task(void* arg)
{
    int adcValue = 0;
    int voltage = 0;
    while(1){
    	adcValue = adc1_get_voltage(ADC1_CHANNEL);
        voltage = (adcValue*ADC1_REF*2)/512;
        voltage = (voltage+50)/100*100;

        printf("The adc1 value:%d\n",adcValue);
        printf("battery voltage:%d\n",voltage);

        if (voltage >= BATTERY_PERCENTAGE_75) {
        	setBatteryLevel(BATTERY_LEVEL_3);
        }else if (voltage >= BATTERY_PERCENTAGE_50) {
        	setBatteryLevel(BATTERY_LEVEL_2);
        }else if (voltage >= BATTERY_PERCENTAGE_25) {
        	setBatteryLevel(BATTERY_LEVEL_1);
        }else if (voltage >= BATTERY_PERCENTAGE_0) {
        	setBatteryLevel(BATTERY_LEVEL_EMPTY);
        }else{
            key_event_t keyEvent;
            keyEvent.key_type = SLEEP_KEY;
            keyEvent.key_value = KEY_DOWN;
            handle_key_event(keyEvent);
        }
        vTaskDelay(30000/portTICK_PERIOD_MS);
    }
}

static bool battery_check_low_voltage()
{
    int adcValue = adc1_get_voltage(ADC1_CHANNEL);
    int voltage = (adcValue*ADC1_REF*2)/512;
    voltage = (voltage+50)/100*100;
    return voltage <= BATTERY_WARNING? true: false;
}

bool battery_init()
{
    // initialize ADC
    adc1_config_width(ADC_WIDTH_9Bit);
    adc1_config_channel_atten(ADC1_CHANNEL,ADC_ATTEN_11db);

    if (battery_check_low_voltage()) {
        return false;
    }

    xTaskCreate(battery_task, "battery_task", 1024*3, NULL, 10, NULL);

    return true;
}