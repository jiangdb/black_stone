#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "calibration.h"
#include "config.h"


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

void set_zero(int channel, int32_t adcValue)
{
    if(channel<0||channel>1) return;

    zero[channel] = adcValue;

    config_write(config_zero_name[channel], zero[channel]);
    printf("set zero[%d]: %d\n", channel, zero[channel]);
}

int32_t get_zero(int channel)
{
    return zero[channel];
}

int get_zero_offset(int channel, int32_t adcValue)
{
    return ((adcValue - zero[channel]) * precision[channel]);
}

int32_t convert_weight(int32_t adcValue, int8_t channel, int8_t *precision)
{

    if (channel<0||channel>CALIBRATION_NUMS) return 0;
    if (cal[channel] == 0) return 0;

    int32_t weight = ((adcValue - zero[channel]) * CALIBRATION_WEIGHT * 100 )/cal[channel];
    if (weight >= 100000 || weight <= -10000) {
        *precision = 0;
        return (weight+50)/10;
    } else{
        *precision = 1;
        return (weight+5)/10;
    }
}

void set_calibration(int index, int32_t channel0, int32_t channel1)
{
    if (index<0 || index>1) return;

    calibrations[0][index] = channel0;
    calibrations[1][index] = channel1;

    printf("set cal[%d]: %d, %d\n", index, channel0, channel1);
    if (index == 1) {
        cal[0] = calibrations[0][1]-calibrations[0][0];
        cal[1] = calibrations[1][1]-calibrations[1][0];
        config_write(CONFIG_CHANNEL_0_CALIBRATION, cal[0]);
        config_write(CONFIG_CHANNEL_1_CALIBRATION, cal[1]);
        printf("calibrations: %d, %d --  %d, %d\n", calibrations[0][0], calibrations[0][1],calibrations[1][0],calibrations[1][1]);
        printf("write cal config: %d, %d\n", cal[0], cal[1]);

        precision[0] = (CALIBRATION_WEIGHT * 1000) / cal[0];
        precision[1] = (CALIBRATION_WEIGHT * 1000) / cal[1];
    }
}

void calibration_init()
{
    zero[0] = config_read(CONFIG_CHANNEL_0_ZERO, 0);
    zero[1] = config_read(CONFIG_CHANNEL_1_ZERO, 0);
    printf("get zero: %d, %d\n", zero[0], zero[1]);

    cal[0] = config_read(CONFIG_CHANNEL_0_CALIBRATION, 10000);
    cal[1] = config_read(CONFIG_CHANNEL_1_CALIBRATION, 10000);
    printf("get cal: %d, %d\n", cal[0], cal[1]);

    precision[0] = (CALIBRATION_WEIGHT * 1000) / cal[0];
    precision[1] = (CALIBRATION_WEIGHT * 1000) / cal[1];
}
