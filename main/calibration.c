#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "calibration.h"
#include "config.h"


#define CALIBRATION_WEIGHT      1900        //2000g-100g
#define CONFIG_CHANNEL_0_ZERO   "channel_0_zero"
#define CONFIG_CHANNEL_1_ZERO   "channel_1_zero"
#define CONFIG_CHANNEL_0_CALIBRATION   "channel_0_cal"
// #define CONFIG_CHANNEL_0_CALIBRATION_1   "channel_0_cal_1"
#define CONFIG_CHANNEL_1_CALIBRATION   "channel_1_cal"
// #define CONFIG_CHANNEL_1_CALIBRATION_1   "channel_1_cal_0"

#define CONVERT_WEIGHT(x)   (x*100/22500)

int32_t labels[5] = {
    100,
    500,
    1000,
    2000,
    3000
};

int32_t calibrations[2][5] = {
    {
        100,
        500,
        1000,
        2000,
        3000,
    },
    {
        100,
        500,
        1000,
        2000,
        3000,
    }
};

char* calibration_name[2][5] = {
    {
        "channel_0_cal_0",
        "channel_0_cal_1",
        "channel_0_cal_2",
        "channel_0_cal_3",
        "channel_0_cal_4",
    },
    {
        "channel_1_cal_0",
        "channel_1_cal_1",
        "channel_1_cal_2",
        "channel_1_cal_3",
        "channel_1_cal_4",
    },
};

static int32_t zero[2] = {0,0};
static int32_t cal[2] = {1,1};

void set_zero(int32_t adcValue0, int32_t adcValue1)
{
    /*
    zero[0] = calibrations[0][0]-adcValue0;
    zero[1] = calibrations[1][0]-adcValue1;
    */
    /*
    zero[0] = (adcValue0/100)*100;
    zero[1] = (adcValue1/100)*100;
    */
    zero[0] = adcValue0;
    zero[1] = adcValue1;

    config_write(CONFIG_CHANNEL_0_ZERO, zero[0]);
    config_write(CONFIG_CHANNEL_1_ZERO, zero[1]);
    printf("set sezo: %d, %d\n", zero[0], zero[1]);
}

void get_zero()
{
    zero[0] = config_read(CONFIG_CHANNEL_0_ZERO, 378);
    zero[1] = config_read(CONFIG_CHANNEL_0_ZERO, 712);
    printf("get zero: %d, %d\n", zero[0], zero[1]);
}

void get_calibrations()
{
    cal[0] = config_read(CONFIG_CHANNEL_0_CALIBRATION, 10000);
    cal[1] = config_read(CONFIG_CHANNEL_1_CALIBRATION, 10000);
    printf("get cal: %d, %d\n", cal[0], cal[1]);
    /*
    for (int i = 0; i < 5; ++i)
    {
        calibrations[0][i] = config_read(calibration_name[0][i], i);
        calibrations[1][i] = config_read(calibration_name[1][i], i);
    }
    */
}

int32_t get_weight(int32_t adcValue, int8_t channel, int8_t *precision)
{

    if (channel<0||channel>CALIBRATION_NUMS) return 0;

    // int value = (adcValue/100)*100-zero[channel];

    /*
    if (channel == 0) {
        return (value*2900/(calibrations[channel][1]-calibrations[channel][0]));
    }else{
        return (value*2900/(calibrations[channel][1]-calibrations[channel][0]));
    }
    */

    if (cal[channel] == 0) return 0;

    float rtn = (float)((adcValue - zero[channel]) * CALIBRATION_WEIGHT )/cal[channel];
    if (rtn > 1000) {
        *precision = 0;
        return (int32_t)rtn;
    }else{
        *precision = 1;
        return (int32_t)(rtn*10);
    }
    // return ((adcValue - zero[channel]) * CALIBRATION_WEIGHT )/(cal[channel]);
    // return (( (adcValue/100) * 100 - zero[channel]) * CALIBRATION_WEIGHT )/(cal[channel]);
    // return CONVERT_WEIGHT(value);
    /*
    int i;
    //find range
	for (i = 0; i < CALIBRATION_NUMS; ++i)
	{
		if (adcValue<=calibrations[channel][i]) {
			break;
		}
	}

	if(i==0) i = 1;
    if (i == CALIBRATION_NUMS) i = CALIBRATION_NUMS-1;

    if (adcValue == calibrations[channel][i]) return labels[i];

	return (labels[i]-labels[i-1])*(adcValue-calibrations[channel][i-1])/(calibrations[channel][i]-calibrations[channel][i-1])+labels[i-1];
    */
}

void set_calibration(int index, int32_t channel0, int32_t channel1)
{
    if (index<0 || index>1) return;

    calibrations[0][index] = channel0;
    calibrations[1][index] = channel1;

    if (index == 1) {
        // cal[0] = (calibrations[0][1]-calibrations[0][0])*100/100;
        // cal[1] = (calibrations[1][1]-calibrations[1][0])*100/100;
        cal[0] = (calibrations[0][1]-calibrations[0][0]);
        cal[1] = (calibrations[1][1]-calibrations[1][0]);
        config_write(CONFIG_CHANNEL_0_CALIBRATION, cal[0]);
        config_write(CONFIG_CHANNEL_1_CALIBRATION, cal[1]);
        printf("write cal config: %d, %d\n", cal[0], cal[1]);
    }
    printf("set cal[%d]: %d, %d\n", index, channel0, channel1);
}

/*
void set_calibration(int index, int32_t channel0, int32_t channel1)
{
    if (index<0 || index>4) return;

    calibrations[index][0] = channel0;
    calibrations[index][1] = channel1;
    config_write(calibration_name[0][index], channel0);
    config_write(calibration_name[1][index], channel1);
    printf("set calibrations[%d]: %d, %d\n", index, channel0, channel1);
}
*/