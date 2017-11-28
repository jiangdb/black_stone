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


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "nvs_flash.h"
#include "bt.h"
#include "bta_api.h"
#include "gatts_service.h"
#include "wifi_service.h"
#include "bs_timer.h"
#include "config.h"
#include "battery.h"
#include "display.h"
#include "key_event.h"

/*
 * DEFINES
 */

#define GATTS_SERVICE_TAG               "GATTS_SERVICE"

#define WEIGHT_SCALE_PROFILE_NUM        1
#define WEIGHT_SCALE_PROFILE_APP_IDX    0
#define WEIGHT_SCALE_APP_ID             0x55
#define DEVICE_NAME                     "Timemore Scale"
#define WEIGHT_SCALE_SVC_INST_ID        0
#define DEVICE_INFORMATION_SVC_INST_ID  1

 /// Weight Scale Measurement
#define CHAR_UUID_WEIGHT_MEAS           0x2A9D
#define CHAR_UUID_WEIGHT_SCALE_FEATURE  0x2A9E

#define CONTROL_OPT_READ         0
#define CONTROL_OPT_WRITE        1
#define CONTROL_OPT_NOTIFY       2
#define CONTROL_STATUS_SUCCESS   0
#define CONTROL_STATUS_FAIL      1
#define CONTROL_MAX_LEN          128

#define CHAR_DECLARATION_SIZE           (sizeof(uint8_t))

/*
 * ENUMS
 */
enum {
    CONTROL_NOTIFY_WIFI,
    CONTROL_NOTIFY_BATTERY,
};

enum {
    CONTROL_SET_ZERO,
    CONTROL_ZERO_TRACE,
    CONTROL_ALARM,
    CONTROL_WEIGHT_UNIT,
    CONTROL_ALARM_TIME,
    CONTROL_ALARM_WEIGHT,
    CONTROL_WIFI,
    CONTROL_FW_UPGRADE,
    CONTROL_START_TIMER,
    CONTROL_PAUSE_TIMER,
    CONTROL_RESET_TIMER,
    CONTROL_BATTERY_CALIBRATION,
};

enum
{
    WSS_IDX_SVC,

    WSS_IDX_WS_MEAS_CHAR,
    WSS_IDX_WS_MEAS_VAL,
    WSS_IDX_WS_MEAS_NTF_CFG,

    WSS_IDX_WS_FEATURE_CHAR,
    WSS_IDX_WS_FEATURE_VAL,

    WSS_IDX_WS_CTNL_PT_CHAR,
    WSS_IDX_WS_CTNL_PT_VAL,
    WSS_IDX_WS_CTNL_NTF_CFG,

    WSS_IDX_NB,
};

enum
{
    DIS_IDX_SVC,
    DIS_IDX_MANU_NAME_CHAR,
    DIS_IDX_MANU_NAME_VAL,
    
    DIS_IDX_MODEL_NUMBER_CHAR,
    DIS_IDX_MODEL_NUMBER_VAL,

    DIS_IDX_SERIAL_NUMBER_CHAR,
    DIS_IDX_SERIAL_NUMBER_VAL,

    DIS_IDX_FW_REVISION_CHAR,
    DIS_IDX_FW_REVISION_VAL,

    DIS_IDX_NB,
};

static uint8_t weight_scale_service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x1D, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t weight_scale_adv_config = {
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
    .service_uuid_len = sizeof(weight_scale_service_uuid),
    .p_service_uuid = weight_scale_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t weight_scale_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    bool     connected;
    uint16_t *handle_table;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, 
                    esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

uint16_t weight_scale_handle_table[WSS_IDX_NB];
uint16_t device_information_handle_table[DIS_IDX_NB];
/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst weight_scale_profile_tab[WEIGHT_SCALE_PROFILE_NUM] = {
    [WEIGHT_SCALE_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
        .app_id = WEIGHT_SCALE_APP_ID,
        .connected = false,
        .handle_table = weight_scale_handle_table,
    },
};

typedef union {
    struct {
        uint8_t bmi_height_present:     1;                /* BMI and Height present: 0 false 1 true */
        uint8_t userid_present:         1;                /* User ID present: 0 false 1 true */
        uint8_t timestamp_present:      1;                /* Time stamp present: 0 false 1 true */
        uint8_t measurement_units:      1;                /* Measurement Units: 0 SI 1 Imperial*/
        uint8_t two_channel:            1;                /* Custom: 0 single channel, 1 two channel*/
        uint8_t reserved:               3;                /* Reserved */
    };
    uint8_t val;
} weight_measurement_flags_t;

static int32_t weight_value[2] = {-1,0};
static TaskHandle_t xHandle = NULL;
//static SemaphoreHandle_t connectedSem = NULL;
static bool meas_notify_enable = false;
static bool control_notify_enable = false;

/*
 *  Weight Scale PROFILE ATTRIBUTES
 ****************************************************************************************
 */

/// Weight Scale Service
static const uint16_t weight_scale_svc = ESP_GATT_UUID_WEIGHT_SCALE_SVC;
/// Device Information Service
static const uint16_t device_information_svc = ESP_GATT_UUID_DEVICE_INFO_SVC;

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_indicate = ESP_GATT_CHAR_PROP_BIT_INDICATE;
static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE|ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_WRITE|ESP_GATT_CHAR_PROP_BIT_READ|ESP_GATT_CHAR_PROP_BIT_NOTIFY;

/// Weight Scale Service - Weight Measurement Characteristic, indicate
static const uint16_t weight_measurement_uuid = CHAR_UUID_WEIGHT_MEAS;
static const uint8_t weight_measurement_ccc[2] ={ 0x00, 0x00};

/// Weight Scale Service - Weight Scale Feature characteristic, read
static const uint16_t weight_scale_feature_uuid = CHAR_UUID_WEIGHT_SCALE_FEATURE;
static const uint32_t weight_scale_feature_val = 0x38;

/// Weight Scale Service - Custom: Weight Scale Control Point characteristic, write&read
//custom 128bit UUID 0x00001234800000000000000000000000
static const uint8_t weight_scale_ctrl_point_uuid[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x08,0x34,0x12,0x00,0x00};
static const uint8_t weight_scale_ctrl_point_ccc[2] = {0x00, 0x00};

static const uint16_t manufacturer_name_uuid = ESP_GATT_UUID_MANU_NAME;
static const uint16_t model_number_uuid = ESP_GATT_UUID_MODEL_NUMBER_STR;
static const uint16_t serial_number_uuid = ESP_GATT_UUID_SERIAL_NUMBER_STR;
static uint8_t serial_number_val[13] = {0};
static const uint16_t firmware_revision_uuid = ESP_GATT_UUID_FW_VERSION_STR;

/// Full WSS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t weight_scale_gatt_db[WSS_IDX_NB] =
{
    // Weight Scale Service Declaration
    [WSS_IDX_SVC] = 
        {
            { ESP_GATT_AUTO_RSP },
            {
                ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
                sizeof(uint16_t), sizeof(weight_scale_svc), (uint8_t *)&weight_scale_svc
            }
        },

    // Weight Measurement Characteristic Declaration
    [WSS_IDX_WS_MEAS_CHAR] = 
        {
            {ESP_GATT_AUTO_RSP},
            {
                ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_indicate
            }
        },
      
    // Weight Measurement Characteristic Value
    [WSS_IDX_WS_MEAS_VAL] =
        {
            {ESP_GATT_AUTO_RSP},
            {
                ESP_UUID_LEN_16, (uint8_t *)&weight_measurement_uuid, ESP_GATT_PERM_READ,
                15,0, NULL
            }
        },

    // Weight Measurement Characteristic - Client Characteristic Configuration Descriptor
    [WSS_IDX_WS_MEAS_NTF_CFG] = 
        {
            {ESP_GATT_AUTO_RSP},
            {
                ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
                sizeof(uint16_t),sizeof(weight_measurement_ccc), (uint8_t *)weight_measurement_ccc
            }
        },

    // Weight Scale Feature Characteristic Declaration
    [WSS_IDX_WS_FEATURE_CHAR] =
        {
            {ESP_GATT_AUTO_RSP},
            {
                ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read
            }
        },

    // Weight Scale Feature Characteristic Value
    [WSS_IDX_WS_FEATURE_VAL] =
        {
            {ESP_GATT_AUTO_RSP},
            {
                ESP_UUID_LEN_16, (uint8_t *)&weight_scale_feature_uuid , ESP_GATT_PERM_READ,
                sizeof(uint32_t), sizeof(weight_scale_feature_val), (uint8_t *)&weight_scale_feature_val
            }
        },

    // Custom: Weight Scale Control Point Characteristic Declaration
    [WSS_IDX_WS_CTNL_PT_CHAR] =
        {
            {ESP_GATT_AUTO_RSP},
            {
                ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify
            }
        },
                                         			
    // Custom: Weight Scale Control Point Characteristic Value
    [WSS_IDX_WS_CTNL_PT_VAL] = 
        {
            {ESP_GATT_RSP_BY_APP},
            {
                ESP_UUID_LEN_128, (uint8_t *)&weight_scale_ctrl_point_uuid, ESP_GATT_PERM_WRITE|ESP_GATT_PERM_READ,
                255, NULL, NULL
            }
        },

    // Custom: Weight Scale Control Point Characteristic - Client Characteristic Configuration Descriptor
    [WSS_IDX_WS_CTNL_NTF_CFG] = 
        {
            {ESP_GATT_AUTO_RSP},
            {
                ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
                sizeof(uint16_t),sizeof(weight_scale_ctrl_point_ccc), (uint8_t *)weight_scale_ctrl_point_ccc
            }
        },
};

static const esp_gatts_attr_db_t device_information_gatt_db[DIS_IDX_NB] = {
    // Device Information Service Declaration
    [DIS_IDX_SVC] = 
        {
            { ESP_GATT_AUTO_RSP },
            {
                ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
                sizeof(uint16_t), sizeof(device_information_svc), (uint8_t *)&device_information_svc
            }
        },

    // Manufacturer Name String Characteristic Declaration
    [DIS_IDX_MANU_NAME_CHAR] =
        {
            {ESP_GATT_AUTO_RSP},
            {
                ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read
            }
        },

    // Manufacturer Name String Characteristic Value
    [DIS_IDX_MANU_NAME_VAL] =
        {
            {ESP_GATT_AUTO_RSP},
            {
                ESP_UUID_LEN_16, (uint8_t *)&manufacturer_name_uuid, ESP_GATT_PERM_READ,
                50, strlen(MANUFACTURER), (uint8_t *)MANUFACTURER
            }
        },

    // Model Number String Characteristic Declaration
    [DIS_IDX_MODEL_NUMBER_CHAR] =
        {
            {ESP_GATT_AUTO_RSP},
            {
                ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read
            }
        },

    // Model Number String Characteristic Value
    [DIS_IDX_MODEL_NUMBER_VAL] =
        {
            {ESP_GATT_AUTO_RSP},
            {
                ESP_UUID_LEN_16, (uint8_t *)&model_number_uuid, ESP_GATT_PERM_READ,
                50, strlen(MODEL_NUMBER), (uint8_t *)MODEL_NUMBER
            }
        },

    // Serial Number String Characteristic Declaration
    [DIS_IDX_SERIAL_NUMBER_CHAR] =
        {
            {ESP_GATT_AUTO_RSP},
            {
                ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read
            }
        },

    // Serial Number String Characteristic Value
    [DIS_IDX_SERIAL_NUMBER_VAL] =
        {
            {ESP_GATT_AUTO_RSP},
            {
                ESP_UUID_LEN_16, (uint8_t *)&serial_number_uuid, ESP_GATT_PERM_READ,
                20, sizeof(serial_number_val)-1, (uint8_t *)serial_number_val
            }
        },

    // Firmware Revision String Characteristic Declaration
    [DIS_IDX_FW_REVISION_CHAR] =
        {
            {ESP_GATT_AUTO_RSP},
            {
                ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read
            }
        },

    // Firmware Revision String Characteristic Value
    [DIS_IDX_FW_REVISION_VAL] =
        {
            {ESP_GATT_AUTO_RSP},
            {
                ESP_UUID_LEN_16, (uint8_t *)&firmware_revision_uuid, ESP_GATT_PERM_READ,
                10, strlen(FW_VERSION), (uint8_t *)FW_VERSION
            }
        },
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGD(GATTS_SERVICE_TAG, "GAP_EVT, event %d", event);

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&weight_scale_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_SERVICE_TAG, "Advertising start failed\n");
        }
        break;
    default:
        break;
    }
}

static void init_weight_control_value(uint8_t* value, uint16_t* len)
{
    int index=0;
    value[index++] = CONTROL_OPT_READ;           //operation type
    value[index++] = CONTROL_STATUS_SUCCESS;     //operation status
    value[index++] = config_get_zero_trace();
    value[index++] = config_get_alarm_enable();
    value[index++] = config_get_weight_unit();
    uint16_t alarm_time = config_get_alarm_time();
    value[index++] = (uint8_t)(alarm_time>>8);
    value[index++] = (uint8_t)alarm_time;
    uint16_t alarm_weight = config_get_alarm_weight();
    value[index++] = (uint8_t)(alarm_weight>>8);
    value[index++] = (uint8_t)alarm_weight;
    value[index++] = get_battery_level();
    value[index++] = ws_get_status();
    char* wifi_name = config_get_wifi_name();
    if (wifi_name == NULL) {
        value[index++] = 0;
        *len = index;
    }else{
        value[index++] = strlen(wifi_name);
        memcpy(&value[index], wifi_name, strlen(wifi_name));
        *len = index + strlen(wifi_name);
    }
}

static void handle_weight_control_write(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    //prepare response
    uint8_t reply[2];
    memset(reply, 0, sizeof(reply));
    reply[0] = CONTROL_OPT_WRITE;
    reply[1] = CONTROL_STATUS_SUCCESS;

    uint8_t *pData = (uint8_t *)param->write.value;
    ESP_LOGD(GATTS_SERVICE_TAG, "GATT_WRITE_EVT, write control %d", pData[0]);
    switch(pData[0]) {
        case CONTROL_SET_ZERO:
            {
                key_event_t keyEvent;
                keyEvent.key_type = CLEAR_KEY;
                keyEvent.key_value = KEY_UP;
                send_key_event(keyEvent,false);
            }
            break;
        case CONTROL_ZERO_TRACE:
            config_set_zero_trace(pData[1]);
            break;
        case CONTROL_ALARM:
            config_set_alarm_enable(pData[1]);
            if (pData[1]) {
                display_seticon(ICON_SOUND,true);
            }else{
                display_seticon(ICON_SOUND,false);
            }
            break;
        case CONTROL_WEIGHT_UNIT:
            config_set_weight_unit(pData[1]);
            break;
        case CONTROL_ALARM_TIME:
            {
                uint16_t time = pData[1]<<8|pData[2];
                config_set_alarm_time(time);
            }
            break;
        case CONTROL_ALARM_WEIGHT:
            {
                uint16_t weight = pData[1]<<8|pData[2];
                config_set_alarm_weight(weight);
            }
            break;
        case CONTROL_WIFI:
            {
                uint8_t name_len = pData[1];
                config_set_wifi_name((char*)&pData[2],name_len);
                uint16_t pass_offset = 2+name_len;
                uint8_t pass_len = pData[pass_offset];
                config_set_wifi_pass((char*)&pData[pass_offset+1],pass_len);
                char* wifi_name = config_get_wifi_name();
                char* wifi_pass = config_get_wifi_pass();
                ws_connect(wifi_name, wifi_pass);
            }
            break;
        case CONTROL_FW_UPGRADE:
            {
                uint8_t host_len = pData[1];
                uint16_t port_offset = 2+host_len;
                uint8_t port = pData[port_offset];
                uint16_t path_offset = port_offset+1;
                uint8_t path_len = pData[path_offset++];
                config_set_firmware_upgrade((char*)&pData[2],host_len, port, (char*)&pData[path_offset],path_len);
                key_event_t keyEvent;
                keyEvent.key_type = FIRMWARE_UPGRADE_KEY;
                keyEvent.key_value = KEY_UP;
                send_key_event(keyEvent,false);
            }
            break;
        case CONTROL_START_TIMER:
            if (!bs_timer_status(TIMER_STOPWATCH)) {
                key_event_t keyEvent;
                keyEvent.key_type = TIMER_KEY;
                keyEvent.key_value = KEY_DOWN;
                send_key_event(keyEvent,false);
            }
            break;
        case CONTROL_PAUSE_TIMER:
            if (bs_timer_status(TIMER_STOPWATCH)) {
                key_event_t keyEvent;
                keyEvent.key_type = TIMER_KEY;
                keyEvent.key_value = KEY_DOWN;
                send_key_event(keyEvent,false);
            }
            break;
        case CONTROL_RESET_TIMER:
            {
                key_event_t keyEvent;
                keyEvent.key_type = TIMER_KEY;
                keyEvent.key_value = KEY_HOLD;
                send_key_event(keyEvent,false);
            }
            break;
        case CONTROL_BATTERY_CALIBRATION:
            battery_calibration();
            break;
        default:
            ESP_LOGE(GATTS_SERVICE_TAG, "GATT_WRITE_EVT, write control unknown setting %d", pData[0])
            reply[1] = CONTROL_STATUS_FAIL;
            break;
    }

    esp_ble_gatts_send_indicate(gatts_if,param->write.conn_id,param->write.handle,
            sizeof(reply), reply, false);
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, 
                                           esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) 
{
    //ESP_LOGE(GATTS_SERVICE_TAG, "PROFILE event = %x\n",event);
    switch (event) {
        case ESP_GATTS_REG_EVT:
            esp_ble_gap_set_device_name(DEVICE_NAME);
            esp_ble_gap_config_adv_data(&weight_scale_adv_config);
            esp_ble_gatts_create_attr_tab(weight_scale_gatt_db, gatts_if, 
                                    WSS_IDX_NB, WEIGHT_SCALE_SVC_INST_ID);
            esp_ble_gatts_create_attr_tab(device_information_gatt_db, gatts_if, 
                                    DIS_IDX_NB, DEVICE_INFORMATION_SVC_INST_ID);
            break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGD(GATTS_SERVICE_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
            if (param->read.handle == weight_scale_handle_table[WSS_IDX_WS_CTNL_PT_VAL]) {
                ESP_LOGD(GATTS_SERVICE_TAG, "GATT_READ_EVT, read control\n");
                esp_gatt_rsp_t rsp;
                memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
                rsp.attr_value.handle = param->read.handle;
                init_weight_control_value(rsp.attr_value.value, &rsp.attr_value.len);
                esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                            ESP_GATT_OK, &rsp);
            }
            break;
        case ESP_GATTS_WRITE_EVT: 
            ESP_LOGD(GATTS_SERVICE_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
            ESP_LOGD(GATTS_SERVICE_TAG, "GATT_WRITE_EVT, value len %d, value %08x", param->write.len, *(uint8_t *)param->write.value);
            ESP_LOGD(GATTS_SERVICE_TAG, "GATT_WRITE_EVT, need_rsp %d, is_prep %d", param->write.need_rsp, param->write.is_prep);

            if (param->write.handle == weight_scale_handle_table[WSS_IDX_WS_CTNL_PT_VAL]) {
                ESP_LOGD(GATTS_SERVICE_TAG, "GATT_WRITE_EVT, write control");
                if (param->write.need_rsp && !param->write.is_prep){
                    esp_gatt_status_t status = ESP_GATT_OK;
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
                    handle_weight_control_write(gatts_if, param);
                }
            } else if (param->write.handle == weight_scale_handle_table[WSS_IDX_WS_MEAS_NTF_CFG]) {
                uint8_t* value = (uint8_t *)param->write.value;
                ESP_LOGD(GATTS_SERVICE_TAG, "turn on/off measurement notify: %08x", *value);
                if (value[0] & 0x01) {
                    meas_notify_enable = true;
                }else{
                    meas_notify_enable = false;
                }
            } else if (param->write.handle == weight_scale_handle_table[WSS_IDX_WS_CTNL_NTF_CFG]) {
                uint8_t* value = (uint8_t *)param->write.value;
                ESP_LOGD(GATTS_SERVICE_TAG, "turn on/off control notify: %08x", *value);
                if (value[0] & 0x01) {
                    control_notify_enable = true;
                }else{
                    control_notify_enable = false;
                }
            }

            break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            break;
        case ESP_GATTS_MTU_EVT:
            break;
        case ESP_GATTS_CONF_EVT:
            break;
        case ESP_GATTS_UNREG_EVT:
            break;
        case ESP_GATTS_DELETE_EVT:
            break;
        case ESP_GATTS_START_EVT:
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
                ESP_LOGD(GATTS_SERVICE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:, is_conn %d",
                         param->connect.conn_id,
                         param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                         param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5],
                         param->connect.is_connected);
                weight_scale_profile_tab[WEIGHT_SCALE_PROFILE_APP_IDX].conn_id = param->connect.conn_id;
                weight_scale_profile_tab[WEIGHT_SCALE_PROFILE_APP_IDX].connected = true;
                //start sent the update connection parameters to the peer device.
                esp_ble_gap_update_conn_params(&conn_params);
                //xSemaphoreGive(connectedSem);
            }
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            weight_scale_profile_tab[WEIGHT_SCALE_PROFILE_APP_IDX].connected = false;
            esp_ble_gap_start_advertising(&weight_scale_adv_params);
            break;
        case ESP_GATTS_OPEN_EVT:
            break;
        case ESP_GATTS_CANCEL_OPEN_EVT:
            break;
        case ESP_GATTS_CLOSE_EVT:
            break;
        case ESP_GATTS_LISTEN_EVT:
            break;
        case ESP_GATTS_CONGEST_EVT:
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            ESP_LOGD(GATTS_SERVICE_TAG, "[%x]: The number handle =%x",param->add_attr_tab.svc_uuid.uuid.uuid16, param->add_attr_tab.num_handle);
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(GATTS_SERVICE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else {
                if (param->add_attr_tab.svc_uuid.uuid.uuid16 == weight_scale_svc) {
                    memcpy(weight_scale_handle_table, param->add_attr_tab.handles, sizeof(weight_scale_handle_table));
                    esp_ble_gatts_start_service(weight_scale_handle_table[WSS_IDX_SVC]);
                }
                if (param->add_attr_tab.svc_uuid.uuid.uuid16 == device_information_svc) {
                    memcpy(device_information_handle_table, param->add_attr_tab.handles, sizeof(device_information_handle_table));
                    esp_ble_gatts_start_service(device_information_handle_table[DIS_IDX_SVC]);
                }
            }
            break;
        default:
            break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, 
                                    esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGD(GATTS_SERVICE_TAG, "GATTS_EVT %d, gatts if %d", event, gatts_if);

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            weight_scale_profile_tab[WEIGHT_SCALE_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_SERVICE_TAG, "Reg app failed, app_id %04x, status %d\n",
                    param->reg.app_id, 
                    param->reg.status);
            return;
        }
    }
    
    do {
        int idx;
        for (idx = 0; idx < WEIGHT_SCALE_PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == weight_scale_profile_tab[idx].gatts_if) {
                if (weight_scale_profile_tab[idx].gatts_cb) {
                    weight_scale_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}


static void notify_control_point(uint8_t key, uint8_t value)
{
    if (!control_notify_enable) return;

    ESP_LOGD(GATTS_SERVICE_TAG, "%s(%d, %d)", __func__, key, value);
    uint8_t notify[4];
    memset(notify, 0, sizeof(notify));

    notify[0] = CONTROL_OPT_NOTIFY;
    notify[1] = key;
    notify[2] = value;

    esp_ble_gatts_send_indicate(
            weight_scale_profile_tab[WEIGHT_SCALE_PROFILE_APP_IDX].gatts_if,
            weight_scale_profile_tab[WEIGHT_SCALE_PROFILE_APP_IDX].conn_id,
            weight_scale_handle_table[WSS_IDX_WS_CTNL_PT_VAL],
            sizeof(notify), notify, false);
}

void bt_notify_wifi_status(uint8_t wifi_status)
{
    notify_control_point(CONTROL_NOTIFY_WIFI, wifi_status);
}

void bt_notify_battery_level(uint8_t level)
{
    notify_control_point(CONTROL_NOTIFY_BATTERY, level);
}

static void ble_notify_measurement()
{
    //ESP_LOGI(GATTS_SERVICE_TAG, "%s()", __func__);
    uint8_t buffer[9];
    size_t bufferSize = 5;
    weight_measurement_flags_t flag = {
        .bmi_height_present = 0,
        .userid_present = 0,
        .timestamp_present = 0,
        .measurement_units = 0,
        .two_channel = 0,
        .reserved = 0,
    };
    buffer[0] = flag.val;
    memcpy(&buffer[1], &weight_value[DISPLAY_CHANNEL_DOWN], sizeof(int32_t));

    if (weight_value[DISPLAY_CHANNEL_UP] != -1) {
        flag.two_channel = 1;
        memcpy(&buffer[5], &weight_value[DISPLAY_CHANNEL_UP], sizeof(int32_t));
        bufferSize = sizeof(buffer);
    }

    esp_ble_gatts_send_indicate(
            weight_scale_profile_tab[WEIGHT_SCALE_PROFILE_APP_IDX].gatts_if,
            weight_scale_profile_tab[WEIGHT_SCALE_PROFILE_APP_IDX].conn_id,
            weight_scale_handle_table[WSS_IDX_WS_MEAS_VAL],
            bufferSize, buffer, false);
}

void weight_measurement_indicate_loop()
{
    while(1) {
        /*
        if (!weight_scale_profile_tab[WEIGHT_SCALE_PROFILE_APP_IDX].connected) {
            xSemaphoreTake( connectedSem, portMAX_DELAY );
        }
        */
        if ( meas_notify_enable ) {
            ble_notify_measurement();
        }
        vTaskDelay(100/portTICK_RATE_MS);
    }
}

void bt_set_weight(int channel, int32_t value)
{
    if (channel<0||channel>1) return;
    weight_value[channel] = value;
}

void bt_stop()
{
    if( xHandle != NULL )
    {
        vTaskDelete( xHandle );
    }
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable(ESP_BT_MODE_BTDM);
    esp_bt_controller_deinit();
}

void bt_init()
{
    ESP_LOGD(GATTS_SERVICE_TAG, "%s init bluetooth\n", __func__);

    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_SERVICE_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret) {
        ESP_LOGE(GATTS_SERVICE_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_SERVICE_TAG, "%s init bluetooth failed\n", __func__);
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_SERVICE_TAG, "%s enable bluetooth failed\n", __func__);
        return;
    }

    //init device info
    config_get_serial_num((char*)serial_number_val,sizeof(serial_number_val));

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(WEIGHT_SCALE_APP_ID);

    //connectedSem=xSemaphoreCreateBinary();
    //Create task
    xTaskCreate(&weight_measurement_indicate_loop, "weight_mesa_indicate_task", 2048, NULL, 5, &xHandle);
    return;
}
