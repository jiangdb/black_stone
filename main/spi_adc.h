#ifndef _SPI_ADC_H_
#define _SPI_ADC_H_
#include <stdio.h>

void spi_adc_init();
void spi_adc_calibration(bool enable);
void spi_adc_shutdown();
int32_t spi_adc_get_value();

#endif  /*_SPI_ADC_H_*/