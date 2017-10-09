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
#include "nvs_flash.h"
#include "bt.h"
#include "bta_api.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "gatts_service.h"

#define GATTS_WEIGHT_TAG "GATTS_WEIGHT"

#define WEIGHT_SCALE_PROFILE_NUM       1
#define WEIGHT_SCALE_PROFILE_APP_IDX   0
#define WEIGHT_SCALE_APP_ID            0x55
#define DEVICE_NAME                    "Timemore Scale"
#define WEIGHT_SCALE_SVC_INST_ID       0
#define GATTS_CHAR_VAL_LEN_MAX         0x40

 /// Weight Scale Measurement
#define CHAR_UUID_WEIGHT_MEAS            0x2A9D
#define CHAR_UUID_WEIGHT_SCALE_FEATURE   0x2A9E

uint8_t char1_str[] ={0x11,0x22,0x33};

uint16_t weight_scale_handle_table[WSS_IDX_NB];

esp_attr_value_t gatts_demo_char1_val = 
{
    .attr_max_len = GATTS_CHAR_VAL_LEN_MAX,
    .attr_len       = sizeof(char1_str),
    .attr_value     = char1_str,
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
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, 
                    esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst weight_scale_profile_tab[WEIGHT_SCALE_PROFILE_NUM] = {
    [WEIGHT_SCALE_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
        .connected = false,
    },
};

typedef union {
    struct {
        uint8_t bmi_height_present:     1;                /* BMI and Height present: 0 false 1 true */
        uint8_t userid_present:         1;                /* User ID present: 0 false 1 true */
        uint8_t timestamp_present:      1;                /* Time stamp present: 0 false 1 true */
        uint8_t measurement_units:      1;                /* Measurement Units: 0 SI 1 Imperial*/
        uint8_t reserved:               4;                /* Reserved */
    };
    uint8_t val;
} weight_measurement_flags_t;

/*
 *  Weight Scale PROFILE ATTRIBUTES
 ****************************************************************************************
 */

/// Weight Scale Service
static const uint16_t weight_scale_svc = ESP_GATT_UUID_WEIGHT_SCALE_SVC;

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_indicate = ESP_GATT_CHAR_PROP_BIT_INDICATE;
static const uint8_t char_prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE|ESP_GATT_CHAR_PROP_BIT_READ;

/// Weight Scale Service - Weight Measurement Characteristic, indicate
static const uint16_t weight_measurement_uuid = CHAR_UUID_WEIGHT_MEAS;
static const uint8_t weight_measurement_ccc[2] ={ 0x00, 0x00};

/// Weight Scale Service - Weight Scale Feature characteristic, read
static const uint16_t weight_scale_feature_uuid = CHAR_UUID_WEIGHT_SCALE_FEATURE;
static const uint32_t weight_scale_feature_val[1] = {0x38};

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
                CHAR_DECLARATION_SIZE,CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_notify
            }
        },
      
    // Weight Measurement Characteristic Value
    [WSS_IDX_WS_MEAS_VAL] =
        {
            {ESP_GATT_AUTO_RSP},
            {
                ESP_UUID_LEN_16, (uint8_t *)&weight_measurement_uuid, ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
                WSPS_WEIGHT_MEAS_MAX_LEN,0, NULL
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
                sizeof(uint32_t), sizeof(weight_scale_feature_val), (uint8_t *)weight_scale_feature_val
            }
        },
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGE(GATTS_WEIGHT_TAG, "GAP_EVT, event %d\n", event);

    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&weight_scale_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_WEIGHT_TAG, "Advertising start failed\n");
        }
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, 
                                           esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) 
{
    ESP_LOGE(GATTS_WEIGHT_TAG, "event = %x\n",event);
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(GATTS_WEIGHT_TAG, "REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
            esp_ble_gap_set_device_name(DEVICE_NAME);
            esp_ble_gap_config_adv_data(&weight_scale_adv_config);
            esp_ble_gatts_create_attr_tab(weight_scale_gatt_db, gatts_if, 
                                    WSS_IDX_NB, WEIGHT_SCALE_SVC_INST_ID);
            break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_WEIGHT_TAG, "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
            /*
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
             */
            break;
        case ESP_GATTS_WRITE_EVT: 
            ESP_LOGI(GATTS_WEIGHT_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d\n", param->write.conn_id, param->write.trans_id, param->write.handle);
            ESP_LOGI(GATTS_WEIGHT_TAG, "GATT_WRITE_EVT, value len %d, value %08x\n", param->write.len, *(uint32_t *)param->write.value);
            /*
                example_write_event_env(gatts_if, &a_prepare_write_env, param);
                if (param->write.value[0] == 0xaa && param->write.value[1] == 0x55) {
                    example_send_notification(gatts_if, param->write.conn_id, param->write.handle);
                }
             */
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
            ESP_LOGI(GATTS_WEIGHT_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:, is_conn %d\n",
                     param->connect.conn_id,
                     param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                     param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5],
                     param->connect.is_connected);
            weight_scale_profile_tab[WEIGHT_SCALE_PROFILE_APP_IDX].conn_id = param->connect.conn_id;
            weight_scale_profile_tab[WEIGHT_SCALE_PROFILE_APP_IDX].connected = true;
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
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
            ESP_LOGI(GATTS_WEIGHT_TAG, "The number handle =%x\n",param->add_attr_tab.num_handle);
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(GATTS_WEIGHT_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != WSS_IDX_NB){
                ESP_LOGE(GATTS_WEIGHT_TAG, "Create attribute table abnormally, num_handle (%d) \
                        doesn't equal to WSS_IDX_NB(%d)", param->add_attr_tab.num_handle, WSS_IDX_NB);
            }
            else {
                memcpy(weight_scale_handle_table, param->add_attr_tab.handles, sizeof(weight_scale_handle_table));
                esp_ble_gatts_start_service(weight_scale_handle_table[WSS_IDX_SVC]);
            }
            break;
        default:
            break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, 
                                    esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_WEIGHT_TAG, "EVT %d, gatts if %d\n", event, gatts_if);

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            weight_scale_profile_tab[WEIGHT_SCALE_PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_WEIGHT_TAG, "Reg app failed, app_id %04x, status %d\n",
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

void ble_send_notification(int32_t bottom, int32_t total)
{
    //ESP_LOGI(GATTS_WEIGHT_TAG, "%s(%d, %d, %d)", __func__, displayNum, value, precision);

    if (!weight_scale_profile_tab[WEIGHT_SCALE_PROFILE_APP_IDX].connected) return;

    uint8_t buffer[9];
    weight_measurement_flags_t flag = {
        .bmi_height_present = 0,
        .userid_present = 0,
        .timestamp_present = 0,
        .measurement_units = 0,
        .reserved = 0,
    };
    buffer[0] = flag.val;
    memcpy(&buffer[1], &bottom, sizeof(int32_t));
    memcpy(&buffer[5], &total, sizeof(int32_t));

    /*
    ESP_LOGI(GATTS_WEIGHT_TAG, "send notification: ");
    for (int i=0;i<sizeof(buffer);i++) {
        ESP_LOGI(GATTS_WEIGHT_TAG, "[%d]: %02x", i, buffer[i]);
    }
    */

    esp_ble_gatts_send_indicate(
            weight_scale_profile_tab[WEIGHT_SCALE_PROFILE_APP_IDX].gatts_if,
            weight_scale_profile_tab[WEIGHT_SCALE_PROFILE_APP_IDX].conn_id,
            weight_scale_handle_table[WSS_IDX_WS_MEAS_VAL],
            sizeof(buffer), buffer, false);
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
        ESP_LOGE(GATTS_WEIGHT_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (ret) {
        ESP_LOGE(GATTS_WEIGHT_TAG, "%s enable controller failed\n", __func__);
        return;
    }

    ESP_LOGI(GATTS_WEIGHT_TAG, "%s init bluetooth\n", __func__);
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_WEIGHT_TAG, "%s init bluetooth failed\n", __func__);
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_WEIGHT_TAG, "%s enable bluetooth failed\n", __func__);
        return;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(WEIGHT_SCALE_APP_ID);
    return;
}
