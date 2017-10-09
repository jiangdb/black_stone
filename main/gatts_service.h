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

/*
 * DEFINES
 ****************************************************************************************
 */

#define WSPS_WEIGHT_MEAS_MAX_LEN        (15)

#define HRPS_MANDATORY_MASK             (0x0F)
#define HRPS_BODY_SENSOR_LOC_MASK       (0x30)
#define HRPS_HR_CTNL_PT_MASK            (0xC0)


///Attributes State Machine
enum
{
    WSS_IDX_SVC,

    WSS_IDX_WS_MEAS_CHAR,
    WSS_IDX_WS_MEAS_VAL,
    WSS_IDX_WS_MEAS_NTF_CFG,

    WSS_IDX_WS_FEATURE_CHAR,
    WSS_IDX_WS_FEATURE_VAL,

    WSS_IDX_NB,
};

void bt_init();
void bt_stop();
void ble_send_notification(int32_t bottom, int32_t total);

#endif  /*_GATTS_SERVICE_H*/
