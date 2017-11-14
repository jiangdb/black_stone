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

enum {
    OPERATION_CALIBRATION,
    OPERATION_UPGRADE,
};

enum {
    DISPLAY_CHANNEL_UP,
    DISPLAY_CHANNEL_DOWN,
    DISPLAY_CHANNEL_MAX,
};

enum {
    ICON_WIFI,
    ICON_SOUND,
    ICON_ALL,
}

void display_init();
void display_start();
void display_stop();
void display_indicate_charge_only();
void display_indicate_charging_only();
void display_indicate_charging();
void display_disable_charging();
void display_backup();
void display_restore();
void display_seticon(int icon, bool on);
void setDisplayNumber(uint8_t displayNum, int32_t value);
void setDisplayTime(uint32_t seconds);
void setBatteryLevel(int batteryLevel);
void setWifiSound(int wifiSound, bool enable);
void display_setOperation(int opation, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
void setError(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);
void alarmNumber();

#endif  /*_DISPLAY_H_*/
