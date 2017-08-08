#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"

int32_t labels[] = {
    0,
    50,
    100,
    150,
    200,
    250,
    300,
    350,
    400,
    450,
    500,
    600,
    700,
    800,
    900,
    1000
};

int32_t calibrations[] = {
    515,
    625,
    735,
    845,
    955,
    1065,
    1175,
    1285,
    1395,
    1505,
    1615,
    1835,
    2055,
    2275,
    2495,
    2715
};

int32_t get_weight(int32_t adcValue)
{
	int8_t count = sizeof(labels);
	int i;
	for (i = 0; i < count; ++i)
	{
		if (adcValue<=calibrations[i]) {
			break;
		}
	}

	if(i==0) return 0;

    // printf("%d\n", i);

    if (adcValue == calibrations[i]) return labels[i];

	return (labels[i]-labels[i-1])*(adcValue-calibrations[i-1])/(calibrations[i]-calibrations[i-1])+labels[i-1];
}

