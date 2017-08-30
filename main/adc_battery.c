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

#define ADC1_CHANNEL (6)
#define BATTERY_PERCENTAGE_75		2190
#define BATTERY_PERCENTAGE_50		2120
#define BATTERY_PERCENTAGE_25		2060
#define BATTERY_PERCENTAGE_0 		2000
#define BATTERY_WARNING				1975

void battery_task(void* arg)
{
    // initialize ADC
    adc1_config_width(ADC_WIDTH_9Bit);
    adc1_config_channel_atten(ADC1_CHANNEL,ADC_ATTEN_11db);
    int adcValue=0;
    while(1){
    	adcValue = adc1_get_voltage(ADC1_CHANNEL);
        printf("The adc1 value:%d\n",adcValue);
        printf("battery voltage:%d\n",(adcValue*3600*2)/512);

        if (adcValue > BATTERY_PERCENTAGE_75) {
        	setBatteryLevel(BATTERY_LEVEL_3);
        }else if (adcValue > BATTERY_PERCENTAGE_50) {
        	setBatteryLevel(BATTERY_LEVEL_2);
        }else if (adcValue > BATTERY_PERCENTAGE_25) {
        	setBatteryLevel(BATTERY_LEVEL_1);
        }else if (adcValue > BATTERY_PERCENTAGE_0) {
        	setBatteryLevel(BATTERY_LEVEL_EMPTY);
        }
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void battery_init()
{
    xTaskCreate(battery_task, "battery_task", 1024*3, NULL, 10, NULL);
}