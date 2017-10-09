// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/*
 * DB: 
 * GATT Services: 
 *    https://www.bluetooth.com/specifications/gatt/services
 *    Weight Scale    0x181D
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "bt.h"
#include "bta_api.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "sdkconfig.h"
#include "gatts_service.h"

#define GATTS_TAG           "GATTS_WEIGHT"
#define DEVICE_NAME         "Weight Scale"

///Declare the static function 
static void gatts_weight_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define GATTS_WEIGHT_SERVICE_UUID               0x181D
#define GATTS_CHAR_WEIGHT_MEASUREMENT_UUID      0x2A9D
#define GATTS_CHAR_WEIGHT_SCALE_FEATURE_UUID    0x2A9E
#define GATTS_WEIGHT_SERVICE_NUM_HANDLE         6           //4 can hole 1 Char, 5 hold 2

#define TEST_MANUFACTURER_DATA_LEN  17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024

uint8_t weight_scale_feature_val[] = {0x38,0x00,0x00,0x00};
esp_attr_value_t char_weight_scale_feature_val = 
{
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(weight_scale_feature_val),
    .attr_value   = weight_scale_feature_val,
};
uint8_t weight_measurement_val[] = {
    0x00,           //flags: SI, No Timestamp, No UserID, No BMI and Height
    0xFF,0xFF       //weight: 0xFFFF can be used to indicate ‘Measurement Unsuccessful’
};
esp_attr_value_t char_weight_measurement_val = 
{
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(weight_measurement_val),
    .attr_value   = weight_measurement_val,
};

#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
        0x02, 0x01, 0x06,
        0x02, 0x0a, 0xeb, 0x03, 0x03, 0xab, 0xcd
};
static uint8_t raw_scan_rsp_data[] = {
        0x0f, 0x09, 0x45, 0x53, 0x50, 0x5f, 0x47, 0x41, 0x54, 0x54, 0x53, 0x5f, 0x44,
        0x45, 0x4d, 0x4f
};
#else
static uint8_t weight_scale_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x1D, 0x18, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x1D, 0x18, 0x1D, 0x18,
};

//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid = weight_scale_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t test_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define PROFILE_NUM                 1
#define PROFILE_WEIGHT_APP_ID       0

#define WEIGHT_SERVICE_CHAR_NUM         2
#define CHAR_WEIGHT_SCALE_FEATURE_ID    0
#define CHAR_WEIGHT_MEASUREMENT_ID      1

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;

    uint16_t weight_service_char_handle[WEIGHT_SERVICE_CHAR_NUM];
    esp_bt_uuid_t weight_service_char_uuid[WEIGHT_SERVICE_CHAR_NUM];
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_WEIGHT_APP_ID] = {
        .gatts_cb = gatts_weight_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;

typedef union {
    struct {
        uint8_t bmi_height_present:     1;                /* BMI and Height present: 0 false 1 true */
        uint8_t userid_present:         1;                /* User ID present: 0 false 1 true */
        uint8_t timestamp_present:      1;                /* Time stamp present: 0 false 1 true */
        uint8_t measurement_units:      1;                /* Measurement Units: 0 SI 1 Imperial*/
        uint8_t second_channel_present: 1;                /* Custom: second channel present: 0 false 1 true */
        uint8_t one_decimal:            1;                /* Custom: one decimal: 0 false 1 true */
        uint8_t reserved:               2;                /* Reserved */
    };
    uint8_t val;
} weight_measurement_flags_t;

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_send_notification(esp_gatt_if_t gatts_if, uint16_t conn_id, uint16_t attr_handle);

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&test_adv_params);
        break;
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&test_adv_params);
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&test_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising start failed\n");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed\n");
        }
        else {
            ESP_LOGI(GATTS_TAG, "Stop adv successfully\n");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTS_TAG, "update connetion params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    esp_gatt_status_t status = ESP_GATT_OK;
    ESP_LOGI(GATTS_TAG, "write need rsp %d\n", param->write.need_rsp);
    ESP_LOGI(GATTS_TAG, "write is prep %d\n", param->write.is_prep);
    ESP_LOGI(GATTS_TAG, "write value len %d\n", param->write.len);
    ESP_LOGI(GATTS_TAG, "write value ");
    for(int i = 0; i < param->write.len; i++){
        ESP_LOGI(GATTS_TAG, "%x\n", param->write.value[i]);
    }

    if (param->write.need_rsp){
        if (param->write.is_prep){
            if (prepare_write_env->prepare_buf == NULL) {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    LOG_ERROR("Gatt_server prep no mem\n");
                    status = ESP_GATT_NO_RESOURCES;
                }
            } else {
                if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_OFFSET;
                } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_ATTR_LEN;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               LOG_ERROR("Send response error\n");
            }
            free(gatt_rsp);
            if (status != ESP_GATT_OK){
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;

        }else{
            //wechart program has no prepare
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){
        esp_log_buffer_hex(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

void example_send_notification(esp_gatt_if_t gatts_if, uint16_t conn_id, uint16_t attr_handle){
    uint8_t value[4] = { 0x11, 0x22, 0x33, 0x44 };
    uint8_t len = 4;

    ESP_LOGI(GATTS_TAG, "send notification, value len %d, value %08x\n", len, *(uint32_t *)value);
    esp_ble_gatts_send_indicate(gatts_if, conn_id, attr_handle, len, value, false);
}

static void gatts_weight_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        if (param->reg.status != ESP_GATT_OK) break;
        gl_profile_tab[PROFILE_WEIGHT_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_WEIGHT_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_WEIGHT_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_WEIGHT_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_WEIGHT_SERVICE_UUID;

        esp_ble_gap_set_device_name(DEVICE_NAME);
#ifdef CONFIG_SET_RAW_ADV_DATA
        esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
        esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
#else
        esp_ble_gap_config_adv_data(&adv_data);
#endif
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_WEIGHT_APP_ID].service_id, GATTS_WEIGHT_SERVICE_NUM_HANDLE);
        break;
    case ESP_GATTS_READ_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0xde;
        rsp.attr_value.value[1] = 0xed;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d\n", param->write.conn_id, param->write.trans_id, param->write.handle);
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value %08x\n", param->write.len, *(uint32_t *)param->write.value);
        example_write_event_env(gatts_if, &a_prepare_write_env, param);
        if (param->write.value[0] == 0xaa && param->write.value[1] == 0x55) {
            example_send_notification(gatts_if, param->write.conn_id, param->write.handle);
        }
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(GATTS_TAG,"ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&a_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
    case ESP_GATTS_CONF_EVT:
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        if (param->reg.status != ESP_GATT_OK) break;

        gl_profile_tab[PROFILE_WEIGHT_APP_ID].service_handle = param->create.service_handle;
        /*
        gl_profile_tab[PROFILE_WEIGHT_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_WEIGHT_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_WEIGHT_MEASUREMENT_UUID;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_WEIGHT_APP_ID].service_handle);

        esp_ble_gatts_add_char(gl_profile_tab[PROFILE_WEIGHT_APP_ID].service_handle, &gl_profile_tab[PROFILE_WEIGHT_APP_ID].char_uuid,
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY, 
                               &char_weight_measurement_val, NULL);
         */
        gl_profile_tab[PROFILE_WEIGHT_APP_ID].weight_service_char_uuid[CHAR_WEIGHT_MEASUREMENT_ID].len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_WEIGHT_APP_ID].weight_service_char_uuid[CHAR_WEIGHT_MEASUREMENT_ID].uuid.uuid16 = GATTS_CHAR_WEIGHT_MEASUREMENT_UUID;

        gl_profile_tab[PROFILE_WEIGHT_APP_ID].weight_service_char_uuid[CHAR_WEIGHT_SCALE_FEATURE_ID].len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_WEIGHT_APP_ID].weight_service_char_uuid[CHAR_WEIGHT_SCALE_FEATURE_ID].uuid.uuid16 = GATTS_CHAR_WEIGHT_SCALE_FEATURE_UUID;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_WEIGHT_APP_ID].service_handle);

        esp_ble_gatts_add_char(gl_profile_tab[PROFILE_WEIGHT_APP_ID].service_handle, &(gl_profile_tab[PROFILE_WEIGHT_APP_ID].weight_service_char_uuid[CHAR_WEIGHT_SCALE_FEATURE_ID]),
                               ESP_GATT_PERM_READ,
                               ESP_GATT_CHAR_PROP_BIT_READ, 
                               &char_weight_scale_feature_val, NULL);

        esp_ble_gatts_add_char(gl_profile_tab[PROFILE_WEIGHT_APP_ID].service_handle, &(gl_profile_tab[PROFILE_WEIGHT_APP_ID].weight_service_char_uuid[CHAR_WEIGHT_MEASUREMENT_ID]),
                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                               ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY, 
                               NULL, NULL);
                               //&char_weight_measurement_val, NULL);
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT: {
        uint16_t length = 0;
        const uint8_t *prf_char;

        ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d, char_uuid %02x\n",
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle, param->add_char.char_uuid.uuid.uuid16);
        if (param->reg.status != ESP_GATT_OK) break;

        //gl_profile_tab[PROFILE_WEIGHT_APP_ID].char_handle = param->add_char.attr_handle;

        if (param->add_char.char_uuid.uuid.uuid16 == GATTS_CHAR_WEIGHT_MEASUREMENT_UUID) {
            gl_profile_tab[PROFILE_WEIGHT_APP_ID].weight_service_char_handle[CHAR_WEIGHT_MEASUREMENT_ID] = param->add_char.attr_handle;

            ESP_LOGI(GATTS_TAG, "weight measurement handles %d\n", gl_profile_tab[PROFILE_WEIGHT_APP_ID].weight_service_char_handle[CHAR_WEIGHT_MEASUREMENT_ID]);
        }else if (param->add_char.char_uuid.uuid.uuid16 == GATTS_CHAR_WEIGHT_SCALE_FEATURE_UUID) {
            gl_profile_tab[PROFILE_WEIGHT_APP_ID].weight_service_char_handle[CHAR_WEIGHT_SCALE_FEATURE_ID] = param->add_char.attr_handle;
            esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
            ESP_LOGI(GATTS_TAG, "weight scale handles %d\n", gl_profile_tab[PROFILE_WEIGHT_APP_ID].weight_service_char_handle[CHAR_WEIGHT_SCALE_FEATURE_ID]);
            ESP_LOGI(GATTS_TAG, "the gatts char length = %x\n", length);
            for(int i = 0; i < length; i++){
                ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x\n",i,prf_char[i]);
            }
        }

        if (param->add_char.char_uuid.uuid.uuid16 == GATTS_CHAR_WEIGHT_MEASUREMENT_UUID) {
            gl_profile_tab[PROFILE_WEIGHT_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_WEIGHT_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
            esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_WEIGHT_APP_ID].service_handle, &gl_profile_tab[PROFILE_WEIGHT_APP_ID].descr_uuid,
                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        }
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d\n",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT: {
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x50;    // max_int = 0x50*1.25ms = 100ms
        conn_params.min_int = 0x30;    // min_int = 0x30*1.25ms = 60ms
        conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:, is_conn %d\n",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5],
                 param->connect.is_connected);
        gl_profile_tab[PROFILE_WEIGHT_APP_ID].conn_id = param->connect.conn_id;
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        esp_ble_gap_start_advertising(&test_adv_params);
        break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id, 
                    param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void ble_send_notification(uint8_t displayNum, int32_t value, int8_t precision)
{
#if 0
    ESP_LOGI(GATTS_TAG, "%s(%d, %d, %d)", __func__, displayNum, value, precision);

    uint8_t buffer[5];
    weight_measurement_flags_t flag = {
        .bmi_height_present = 0,
        .userid_present = 0,
        .timestamp_present = 0,
        .measurement_units = 0,
        .second_channel_present = (displayNum == 1),
        .one_decimal = (precision == 1),
        .reserved = 0,
    };
    buffer[0] = flag.val;
    memcpy(&buffer[1], &value, sizeof(int32_t));

    ESP_LOGI(GATTS_TAG, "send notification: ");
    for (int i=0;i<sizeof(buffer);i++) {
        ESP_LOGI(GATTS_TAG, "[%d]: %02x", i, buffer[i]);
    }

    /*
        ESP_LOGI(GATTS_TAG, "gatts_if %d, conn id %d, char handle %d\n",
                gl_profile_tab[PROFILE_WEIGHT_APP_ID].gatts_if,
                gl_profile_tab[PROFILE_WEIGHT_APP_ID].conn_id,
                gl_profile_tab[PROFILE_WEIGHT_APP_ID].weight_service_char_handle[CHAR_WEIGHT_MEASUREMENT_ID]
                );
     */
    esp_ble_gatts_send_indicate(
            gl_profile_tab[PROFILE_WEIGHT_APP_ID].gatts_if,
            gl_profile_tab[PROFILE_WEIGHT_APP_ID].conn_id,
            gl_profile_tab[PROFILE_WEIGHT_APP_ID].weight_service_char_handle[CHAR_WEIGHT_MEASUREMENT_ID],
            sizeof(buffer), buffer, false);
#endif
}

void bt_stop()
{
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable(ESP_BT_MODE_BTDM);
    esp_bt_controller_deinit();
}

void bt_init()
{
    esp_err_t ret;

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed\n", __func__);
        return;
    }
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed\n", __func__);
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed\n", __func__);
        return;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(PROFILE_WEIGHT_APP_ID);

    return;
}
