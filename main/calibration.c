#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "calibration.h"

#define CALIBRATION_NUMS 6

int32_t labels[CALIBRATION_NUMS] = {
    0,
    100,
    200,
    300,
    400,
    500
};

int32_t calibrations[2][CALIBRATION_NUMS] = {
    {
        378,
        598,
        818,
        1037,
        1257,
        1477,
    },
    {
        712,
        928,
        1147,
        1368,
        1593,
        1839
    }
};

static int32_t zero[2] = {0,0};

void setZero(int32_t adcValue0, int32_t adcValue1)
{
    zero[0] = calibrations[0][0]-adcValue0;
    zero[1] = calibrations[1][0]-adcValue1;
    printf("%d: %d\n", zero[0], zero[1]);
}

int32_t get_weight(int32_t adcValue, int8_t channel)
{
	int i;

    if (channel<0||channel>2) return 0;

    //add zero
    adcValue+=zero[channel];

    //find range
	for (i = 0; i < CALIBRATION_NUMS; ++i)
	{
		if (adcValue<=calibrations[channel][i]) {
			break;
		}
	}

	if(i==0) return 0;
    if (i == CALIBRATION_NUMS) i = CALIBRATION_NUMS-1;

    if (adcValue == calibrations[channel][i]) return labels[i];

	return (labels[i]-labels[i-1])*(adcValue-calibrations[channel][i-1])/(calibrations[channel][i]-calibrations[channel][i-1])+labels[i-1];
}

