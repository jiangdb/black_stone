#ifndef _BS_BATTERY_H_
#define _BS_BATTERY_H_

#include <stdio.h>

void battery_init();
void battery_start();
void battery_stop();
void battery_calibration();
bool can_start_up();
bool is_battery_level_low();
bool is_battery_extremely_low();
uint8_t get_battery_level();

#endif  /*_BS_BATTERY_H_*/
