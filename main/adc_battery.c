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

#define ADC1_CHANNEL (6)

void battery_task(void* arg)
{
    // initialize ADC
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_CHANNEL,ADC_ATTEN_11db);
    while(1){
        printf("The adc1 value:%d\n",adc1_get_voltage(ADC1_CHANNEL));
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void battery_init()
{
    xTaskCreate(battery_task, "battery_task", 1024*3, NULL, 10, NULL);
}