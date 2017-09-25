#ifndef _CALIBRATION_H_
#define _CALIBRATION_H_

#include <stdio.h>

#define CALIBRATION_NUMS 2

void calibration_init();
int32_t convert_weight(int32_t adcValue, int8_t channel, int8_t *precision);
void set_zero(int channel, int32_t adcValue);
int32_t get_zero(int channel);
int get_zero_offset(int channel, int32_t adcValue);

#endif  /*_CALIBRATION_H_*/
