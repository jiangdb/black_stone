#ifndef _GPIO_ADC_H_
#define _GPIO_ADC_H_

#include <stdio.h>

void gpio_adc_init();
void gpio_adc_calibration(bool enable);
void gpio_adc_shutdown();
int32_t gpio_adc_get_value(int ch);

#endif  /*_GPIO_ADC_H_*/