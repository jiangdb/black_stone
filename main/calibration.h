#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

#include <stdio.h>

#define CALIBRATION_NUMS 2

int32_t get_weight(int32_t adcValue, int8_t channel, int8_t *precision);
void set_zero(int32_t adcValue0, int32_t adbValue1);
void get_zero();
void get_calibrations();

#endif  /*_CALIBRATION_H_*/
