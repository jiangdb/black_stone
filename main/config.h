#ifndef _BS_CONFIG_H_
#define _BS_CONFIG_H_

#include <stdio.h>

typedef enum{
    WEIGHT_UNIT_G,
    WEIGHT_UNIT_KG,
}weight_unit_e;

void config_init();
void config_close();

int32_t config_read(char* name, int32_t default_value);
bool config_write(char* name, int32_t value);

uint8_t config_get_zero_track();
bool config_set_zero_track(uint8_t enable);

uint8_t config_get_alarm_enable();
bool config_set_alarm_enable(uint8_t enable);

uint16_t config_get_alarm_time();
bool config_set_alarm_time(uint16_t timeInSeconds);

uint16_t config_get_alarm_weight();
bool config_set_alarm_weight(uint16_t weight);

uint8_t config_get_weight_unit();
bool config_set_weight_unit(uint8_t unit);

char* config_get_wifi_name();
bool config_set_wifi_name(char* name, size_t len);

char* config_get_wifi_pass();
bool config_set_wifi_pass(char* name, size_t len);

#endif  /*_BS_CONFIG_H_*/
