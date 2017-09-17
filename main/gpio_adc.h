#ifndef _GPIO_ADC_H_
#define _GPIO_ADC_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

// extern queue_buffer_t qb_GpioAdcData;
// extern queue_buffer_t qb_GpioAdcCalibration;

void gpio_adc_init();
void gpio_adc_calibration(bool enable);
void gpio_adc_shutdown();
int32_t gpio_adc_get_value();

#endif  /*_GPIO_ADC_H_*/