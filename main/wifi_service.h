/*
 * =====================================================================================
 *
 *       Filename:  wifi_service.h
 *
 *    Description:  header file for wifi_service
 *
 *        Version:  1.0
 *        Created:  10/13/2017 22:52:26
 *       Revision:  none
 *       Compiler:  gcc
 *
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef _WIFI_SERVICE_H_
#define _WIFI_SERVICE_H_

#include <stdio.h>

enum {
    WIFI_STATUS_UNSTART,
    WIFI_STATUS_STARTED,
    WIFI_STATUS_DISCONNECTED,
    WIFI_STATUS_CONNECTED,
};

void ws_init();
void ws_stop();
void ws_connect(char* ssid, char* pass);
uint8_t ws_get_status();

#endif  /*_WIFI_SERVICE_H_*/
