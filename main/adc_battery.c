/*
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "battery.h"
#include "display.h"
#include "config.h"
#include "key_event.h"
#include "gatts_service.h"

#define TAG  "BATTERY"

#define ADC1_CHANNEL (6)
#define CONFIG_ADC_REF              "adc_battery_ref"
#define BATTERY_PERCENTAGE_75       3900
#define BATTERY_PERCENTAGE_50       3800
#define BATTERY_PERCENTAGE_25       3700
#define BATTERY_PERCENTAGE_0        3500
#define BATTERY_CHARGE_START        3800
#define BATTERY_WARNING             3500
#define BATTERY_SHUTDOWN            3400

extern void handle_key_event(key_event_t keyEvent);

static TaskHandle_t xHandle = NULL;
static uint8_t batterLevel = 255;
static uint32_t adc_ref = 3900;

static int read_voltage()
{
    int adcValue = adc1_get_voltage(ADC1_CHANNEL);
    int voltage = (adcValue * adc_ref * 2 ) / 512;

    //ESP_LOGD(TAG, "The adc1 value:%d\n",adcValue");
    //ESP_LOGD(TAG, "battery voltage:%d\n",(voltage+50)/100*100);

    return (voltage+50)/100*100;
}

static void setDisplay(uint8_t level)
{
    if (batterLevel != level) {
        setBatteryLevel(level);
        bt_notify_battery_level(level);
        batterLevel = level;
    }
}

void battery_task(void* arg)
{
    int voltage = 0;

    while(1){
        voltage = read_voltage();

        if (voltage >= BATTERY_PERCENTAGE_75) {
            setDisplay(BATTERY_LEVEL_3);
        }else if (voltage >= BATTERY_PERCENTAGE_50) {
            setDisplay(BATTERY_LEVEL_2);
        }else if (voltage >= BATTERY_PERCENTAGE_25) {
            setDisplay(BATTERY_LEVEL_1);
        }else if (voltage >= BATTERY_PERCENTAGE_0) {
            setDisplay(BATTERY_LEVEL_EMPTY);
        }else{
            key_event_t keyEvent;
            keyEvent.key_type = SLEEP_KEY;
            keyEvent.key_value = KEY_DOWN;
            send_key_event(keyEvent,false);
        }
        vTaskDelay(30000/portTICK_PERIOD_MS);
    }
}

uint8_t get_battery_level()
{
    int voltage = read_voltage();

    if (voltage >= BATTERY_PERCENTAGE_75) {
        return BATTERY_LEVEL_3;
    }else if (voltage >= BATTERY_PERCENTAGE_50) {
        return BATTERY_LEVEL_2;
    }else if (voltage >= BATTERY_PERCENTAGE_25) {
        return BATTERY_LEVEL_1;
    }else if (voltage >= BATTERY_PERCENTAGE_0) {
        return BATTERY_LEVEL_EMPTY;
    }
    return BATTERY_LEVEL_EMPTY;
}

bool can_start_up()
{
    int voltage = read_voltage();
    return (voltage < BATTERY_CHARGE_START) ? false: true;
}

bool is_battery_extremely_low()
{
    int voltage = read_voltage();
    return (voltage < BATTERY_SHUTDOWN) ? true: false;
}

bool is_battery_level_low()
{
    int voltage = read_voltage();
    return (voltage <= BATTERY_WARNING) ? true: false;
}

void battery_calibration()
{
    int adc_sum = 0;
    for(int i=0; i<10; i++) {
        adc_sum += adc1_get_voltage(ADC1_CHANNEL);
        vTaskDelay(1/portTICK_PERIOD_MS);
    }
    int adc_mean = adc_sum/10;
    //921600 == 1800*512
    adc_ref = 921600 / adc_mean;
    config_write(CONFIG_ADC_REF, adc_ref);
    ESP_LOGD(TAG, "%s: adc_ref: %d",__func__, adc_ref);
}

void battery_start()
{
    xTaskCreate(battery_task, "battery_task", 1024*3, NULL, 10, &xHandle);   
}

void battery_stop()
{
    if( xHandle != NULL )
    {
        vTaskDelete( xHandle );
    }
}

void battery_init()
{
    // initialize ADC
    adc1_config_width(ADC_WIDTH_9Bit);
    adc1_config_channel_atten(ADC1_CHANNEL,ADC_ATTEN_11db);

    adc_ref = config_read(CONFIG_ADC_REF, 3900);
    ESP_LOGD(TAG, "%s: read adc_ref: %d",__func__, adc_ref);
}
