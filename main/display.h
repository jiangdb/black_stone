#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

enum {
    BATTERY_LEVEL_EMPTY,
    BATTERY_LEVEL_1,
    BATTERY_LEVEL_2,
    BATTERY_LEVEL_3
};

void display_init();
void display_start();
void display_stop();
void display_indicate_charge_only();
void display_indicate_charging_only();
void display_indicate_charging();
void display_disable_charging();
void setDisplayNumber(uint8_t displayNum, int32_t value);
void setDisplayTime(uint32_t seconds);
void setBatteryLevel(int batteryLevel);
void setWifiSound(bool wifi, bool sound);
void spi_trassfer_display();

#endif  /*_DISPLAY_H_*/
