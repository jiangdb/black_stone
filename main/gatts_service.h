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

void bt_init();
void bt_stop();
void ble_send_notification(uint8_t displayNum, int32_t value, int8_t precision);

#endif  /*_GATTS_SERVICE_H*/


