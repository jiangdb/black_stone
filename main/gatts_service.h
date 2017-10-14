/*
 * =====================================================================================
 *
 *       Filename:  gatts_service.h
 *
 *    Description:  header file for gatts_service
 *
 *        Version:  1.0
 *        Created:  10/08/2017 20:00:22
 *       Revision:  none
 *       Compiler:  gcc
 *
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef _GATTS_SERVICE_H
#define _GATTS_SERVICE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void bt_init();
void bt_stop();
void bt_set_weight(int channel, int32_t value);
void bt_notify_wifi_status(uint8_t wifi_status);
void bt_notify_battery_level(uint8_t level);

#endif  /*_GATTS_SERVICE_H*/
