/*
 * =====================================================================================
 *
 *       Filename:  zero_track.c
 *
 *    Description:  If in specific time, changes not more than predefined tolerance,
 *                  set zero to current value
 *        Version:  1.0
 *        Created:  09/24/2017 11:34:48
 *       Compiler:  gcc
 *
 *         Author:  Jiang Dong Bin, 
 * =====================================================================================
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "zero_track.h"
#include "calibration.h"
#include "config.h"

#define TRACE_TIME_THRESHOLD      1000    //1s
#define TRACE_VALUE_THRESHOLD     5       //500mg

static int total_time[2] = {0,0};

/*
 * zero trace
 *
 * @param ch channel
 * @weight weight in 100mg
 * @adcValue adc value
 * @interval intreval between two trace, in ms 
 * @return bool indicate set zero or not
 */
bool zero_trace(int ch, int32_t adcValue, int32_t weight, int interval)
{
    int enable = config_get_zero_trace();
    if (enable) {
        int zero = get_zero(ch);
        
        if ((adcValue == zero) || (abs(weight) > TRACE_VALUE_THRESHOLD) ) {
            total_time[ch] = 0;
        } else {
            total_time[ch] += interval;
            if (total_time[ch] >= TRACE_TIME_THRESHOLD) {
                cal_set_zero(ch, adcValue);
                return true;
            }
        }
    }else{
        if (total_time[ch] != 0) {
            total_time[ch] = 0;
        }
    }
    return false;
}
