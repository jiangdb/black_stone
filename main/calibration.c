#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "calibration.h"
#include "config.h"


#define TAG "CAL"

#define CALIBRATION_WEIGHT             100        //100g
#define CONFIG_CHANNEL_0_ZERO          "channel_0_zero"
#define CONFIG_CHANNEL_1_ZERO          "channel_1_zero"
#define CONFIG_CHANNEL_0_CALIBRATION   "channel_0_cal"
#define CONFIG_CHANNEL_1_CALIBRATION   "channel_1_cal"

int32_t calibrations[2][2] = {
    {
        0,
        0,
    },
    {
        0,
        0,
    }
};

char* config_zero_name[2] = {
    CONFIG_CHANNEL_0_ZERO,
    CONFIG_CHANNEL_1_ZERO
};

static int32_t zero[2] = {0,0};
static int32_t cal[2] = {1,1};
static int32_t precision[2] = {0,0};

void cal_set_zero(int channel, int32_t adcValue)
{
    if(channel<0||channel>1) return;

    zero[channel] = adcValue;

    config_write(config_zero_name[channel], zero[channel]);
    ESP_LOGD(TAG,"set zero[%d]: %d\n", channel, zero[channel]);
}

int32_t get_zero(int channel)
{
    return zero[channel];
}

int get_zero_offset(int channel, int32_t adcValue)
{
    return ((adcValue - zero[channel]) * precision[channel]);
}

/*******************************************************************************
**
** Function         convert_weight
**
** Description      convert adcvalue to weight, unit is 100mg.
**
** Parameter        channel: adc channel.
**                  adcValue: adc value .
**
** Returns          weight in 0.1g.
**
*******************************************************************************/
int32_t convert_weight(int8_t channel, int32_t adcValue)
{

    if (channel<0||channel>CALIBRATION_NUMS) return 0;
    if (cal[channel] == 0) return 0;

    int32_t weight = ((adcValue - zero[channel]) * CALIBRATION_WEIGHT * 100 )/cal[channel];
    if (weight >= 100000 || weight <= -10000) {
        return (weight+50)/10;
    } else{
        return (weight+5)/10;
    }
}

void set_calibration(int index, int32_t channel0, int32_t channel1)
{
    if (index<0 || index>1) return;

    calibrations[0][index] = channel0;
    calibrations[1][index] = channel1;

    ESP_LOGD(TAG,"set cal[%d]: %d, %d\n", index, channel0, channel1);
    if (index == 1) {
        cal[0] = calibrations[0][1]-calibrations[0][0];
        cal[1] = calibrations[1][1]-calibrations[1][0];
        config_write(CONFIG_CHANNEL_0_CALIBRATION, cal[0]);
        config_write(CONFIG_CHANNEL_1_CALIBRATION, cal[1]);
        ESP_LOGD(TAG,"calibrations: %d, %d --  %d, %d\n", calibrations[0][0], calibrations[0][1],calibrations[1][0],calibrations[1][1]);
        ESP_LOGD(TAG,"write cal config: %d, %d\n", cal[0], cal[1]);

        if (cal[0] != 0 ) {
            precision[0] = (CALIBRATION_WEIGHT * 1000) / cal[0];
        }
        if (cal[1] != 0 ) {
            precision[1] = (CALIBRATION_WEIGHT * 1000) / cal[1];
        }
    }
}

void calibration_init()
{
    zero[0] = config_read(CONFIG_CHANNEL_0_ZERO, 0);
    zero[1] = config_read(CONFIG_CHANNEL_1_ZERO, 0);
    ESP_LOGD(TAG,"get zero: %d, %d\n", zero[0], zero[1]);

    cal[0] = config_read(CONFIG_CHANNEL_0_CALIBRATION, 10000);
    cal[1] = config_read(CONFIG_CHANNEL_1_CALIBRATION, 10000);
    ESP_LOGD(TAG,"get cal: %d, %d\n", cal[0], cal[1]);

    if (cal[0] != 0 ) {
        precision[0] = (CALIBRATION_WEIGHT * 1000) / cal[0];
    }
    if (cal[1] != 0 ) {
        precision[1] = (CALIBRATION_WEIGHT * 1000) / cal[1];
    }
}
