#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "util.h"

void print_bin(int32_t value, int bytes)
{
	if (bytes > 4 || bytes < 0) return;

    for (int i = 8*bytes-1; i >=0; i--)
    {
        if(value & (1 << i))
            printf("1");
        else
            printf("0");

        if (i%8 == 0){
            printf("    ");
        }
    }
    printf("\n");
}