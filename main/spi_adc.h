#ifndef _SPI_ADC_H_
#define _SPI_ADC_H_

#if 0
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

// extern queue_buffer_t qb_SpiAdcData;
// extern queue_buffer_t qb_SpiAdcCalibration;

void spi_adc_init();
void spi_adc_calibration(bool enable);
void spi_adc_shutdown();
int32_t spi_adc_get_value();
#endif
#endif  /*_SPI_ADC_H_*/