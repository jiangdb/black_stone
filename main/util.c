#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "util.h"

void print_bin(int8_t value)
{
    for (int i = 7; i >=0; i--)
    {
        if(value & (1 << i))
            printf("1");
        else
            printf("0");
    }
}