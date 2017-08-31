#ifndef _SPI_ADC_H_
#define _SPI_ADC_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"


void adc_init();
void adc_calibration(bool enable);
void adc_shutdown();

#endif  /*_SPI_ADC_H_*/