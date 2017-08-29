#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#define BATTERY_LEVEL_EMPTY			0
#define BATTERY_LEVEL_1				1
#define BATTERY_LEVEL_2				2
#define BATTERY_LEVEL_3				3

void display_init();
void setDisplayNumber(uint8_t displayNum, int32_t value, int8_t precision);
void setDisplayTime(uint32_t seconds);
void setBatteryLevel(int batteryLevel);
void setWifiSound(bool wifi, bool sound);
void spi_trassfer_display();

#endif  /*_DISPLAY_H_*/
