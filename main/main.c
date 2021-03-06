/* 
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "bt.h"
#include "battery.h"
#include "display.h"
#include "gatts_service.h"
#include "wifi_service.h"
#include "bs_timer.h"
#include "queue_buffer.h"
#include "key.h"
#include "key_event.h"
#include "calibration.h"
#include "spi_adc.h"
#include "gpio_adc.h"
#include "config.h"
#include "zero_track.h"
#include "ota_service.h"

#define TAG  "MAIN"

#define GPIO_LED_IO                         19
#define DISPLAY_LOCK_THRESHOLD_200MG        2       //0.2g
#define DISPLAY_LOCK_THRESHOLD_300MG        3       //0.3g
#define DISPLAY_LOCK_THRESHOLD_500MG        5       //0.5g
#define DOUBLE_SCALE_THRESHOLD_G            -300    //-30g
#define DOUBLE_SCALE_THRESHOLD_OZ           -11     //-1.1oz
#define REPEAT_COUNT_CALIBRATION            7
#define SLEEP_COUNT                         3
#define SET_ZERO_COUNT                      4       //125*4 = 500ms
#define FORCE_RESTART_COUNT                 20      //500*20 = 10s
#define DISPLAY_LOCK_COUNT                  24      //125*24 = 3s
#define SCALE_UP_MAX_WEIGHT_G               10000   //1kg
#define SCALE_UP_MAX_WEIGHT_OZ              352     //35.2oz
#define SCALE_DOWN_MAX_WEIGHT_G             20000   //2kg
#define SCALE_DOWN_MAX_WEIGHT_OZ            704     //70.4oz

enum {
    WROK_STATUS_INIT,
    WORK_STATUS_CHARGING_SLEEP,
    WORK_STATUS_PRE_NORMAL,
    WORK_STATUS_NORMAL,
    WORK_STATUS_WORKING,
    WORK_STATUS_CALIBRATION,
    WORK_STATUS_SHUTDOWN,
};

enum {
    SCALE_UP,
    SCALE_DOWN,
    SCALE_NUM
};

enum {
    CALIBRATION_STEP_INIT,
    CALIBRATION_STEP_ZERO,
    CALIBRATION_STEP_ONE,
    CALIBRATION_STEP_TWO,
    CALIBRATION_STEP_NUM,
};

typedef enum {
    DISPLAY_LOCKED,
    DISPLAY_UNLOCKING,
    DISPLAY_UNLOCKED,
}en_display_lock;

static int calibrate_tick = -1;
static int calibrate_step = 0;
static int display_lock[2] = {DISPLAY_LOCKED, DISPLAY_LOCKED};
static int display_lock_count[2] = {0,0};
static int32_t lock_weight[2] = {0,0};
static int set_zero_count = -1;
static bool done = false;
static bool auto_shutdown = false;
static int trigger_sleep_count = 0;
static int force_restart_count = 0;
static int cal_key_repeat_count = -1;
static int work_status = WROK_STATUS_INIT;
static bool charging = true;
static bool no_key_start = true;
static bool alarmed = false;
static xQueueHandle eventQueue;
static TaskHandle_t xKeyTaskHandler = NULL;

static void led_init()
{
    //GPIO config for the data line.
    gpio_config_t io_conf={
        .intr_type=GPIO_PIN_INTR_DISABLE,
        .mode=GPIO_MODE_OUTPUT,
        .pull_down_en=0,
        .pull_up_en=0,
        .pin_bit_mask=(1<<GPIO_LED_IO)
    };

    //Set up handshake line interrupt.
    gpio_config(&io_conf);

    if (config_get_weight_unit() == WEIGHT_UNIT_OZ) {
        gpio_set_level(GPIO_LED_IO, 1);
    }
}

void led_enable(bool enable)
{
    if (enable) 
        gpio_set_level(GPIO_LED_IO, 1);
    else
        gpio_set_level(GPIO_LED_IO, 0);
}

static void lock_display(int channel, en_display_lock lock)
{
    display_lock[channel] = lock;
    if (lock == DISPLAY_UNLOCKED) {
        display_lock_count[channel] = 0;
    }
    ESP_LOGD(TAG,"lock_display[%d]: %d!!!\n", channel, lock);
}

static void enter_sleep()
{
    const int ext_wakeup_pin_1 = GPIO_INPUT_IO_KEY_RIGHT;               //power
    const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;
    const int ext_wakeup_pin_2 = GPIO_INPUT_IO_STATE1;                  //charging
    const uint64_t ext_wakeup_pin_2_mask = 1ULL << ext_wakeup_pin_2;

    //enable gpio wakeup
    esp_deep_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask | ext_wakeup_pin_2_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

    ESP_LOGD(TAG,"enter sleep now\n");
    esp_deep_sleep_start();
}

static bool is_wakeup_by_key()
{
    if (esp_deep_sleep_get_wakeup_cause() == ESP_DEEP_SLEEP_WAKEUP_EXT1) {
        uint64_t wakeup_pin_mask = esp_deep_sleep_get_ext1_wakeup_status();
        if (wakeup_pin_mask != 0) {
            int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
            ESP_LOGD(TAG,"Wake up from GPIO %d\n", pin);
            if (pin == 1) {
                return true;
            }
        }
    }
    return false;
}

void send_key_event(key_event_t keyEvent, bool fromIsr)
{
    if (fromIsr) {
        xQueueSendToBackFromISR(eventQueue, &keyEvent, ( TickType_t ) 0 );
    }else{
        xQueueSendToBack(eventQueue, &keyEvent, ( TickType_t ) 0 );
    }
}

static void handle_key_event(void *arg)
{
    bool timer_hold = false;
    while(1) {
        key_event_t keyEvent;
        xQueueReceive(eventQueue, &keyEvent, portMAX_DELAY);

        ESP_LOGD(TAG,"%s: handle key evnet(%d, %d) work_status: %d!!!\n", TAG, keyEvent.key_type, keyEvent.key_value, work_status);
        //reset timeout timer
        bs_timer_reset(TIMER_TIMEOUT);

        //force restart count
        if (keyEvent.key_type == CLEAR_KEY && keyEvent.key_value == KEY_HOLD) {
            force_restart_count++;
            if (force_restart_count >= FORCE_RESTART_COUNT) {
                ESP_LOGI(TAG, "force restart");
                esp_restart();
            }
        }else{
            force_restart_count = 0;
        }
        
        //handle key event
        switch(work_status) {
            case WORK_STATUS_CHARGING_SLEEP:
                if (keyEvent.key_type == NOT_CHARGE_KEY) {
                    charging = false;
                }else if (keyEvent.key_type == CLEAR_KEY) {
                    if (keyEvent.key_value == KEY_UP) {
                        if (can_start_up()) {
                            no_key_start = false;
                        }
                    }
                }
                break;
            case WORK_STATUS_NORMAL:
                switch(keyEvent.key_type){
                    case TIMER_KEY:
                        if (keyEvent.key_value == KEY_HOLD) {
                            if (cal_key_repeat_count < 0) {
                                cal_key_repeat_count = 0;
                            }
                        }else if (keyEvent.key_value == KEY_UP) {
                            if (cal_key_repeat_count >= 0) {
                                cal_key_repeat_count = -1;
                            }else if (bs_timer_toggle(TIMER_STOPWATCH)) {
                                work_status = WORK_STATUS_WORKING;
                            }
                        }
                        break;
                    case CLEAR_KEY:
                        if (keyEvent.key_value == KEY_UP) {
                            if (cal_key_repeat_count >= 0) {
                                cal_key_repeat_count++;
                                if (cal_key_repeat_count == REPEAT_COUNT_CALIBRATION) {
                                    //do calibration
                                    set_zero_count = -1; 
                                    ESP_LOGD(TAG,"enter calibration mode\n");
                                    work_status = WORK_STATUS_CALIBRATION;
                                    calibrate_tick = 0;
                                    calibrate_step = CALIBRATION_STEP_INIT;
                                }
                            }else{
                                //start count for set zero
                                set_zero_count = 0; 
                            }
                        } else if (keyEvent.key_value == KEY_HOLD) {
                            trigger_sleep_count++;
                            if (trigger_sleep_count >= SLEEP_COUNT) {
                                done = true;
                            }
                        }
                        break;
                    case SLEEP_KEY:
                    case FIRMWARE_UPGRADE_KEY:
                        done = true;
                        auto_shutdown = true;
                        break;
                    case CHARGE_KEY:
                        display_indicate_charging();
                        break;
                    case NOT_CHARGE_KEY:
                        display_disable_charging();
                        break;
                    default:
                        break;
                }
                break;
            case WORK_STATUS_WORKING:
                switch(keyEvent.key_type){
                    case TIMER_KEY:
                        if (keyEvent.key_value == KEY_UP) {
                            if (!timer_hold) {
                                bs_timer_toggle(TIMER_STOPWATCH);
                            }else{
                                work_status = WORK_STATUS_NORMAL;
                            }
                            timer_hold = false;
                        }else if (keyEvent.key_value == KEY_HOLD) {
                            bs_timer_stop(TIMER_STOPWATCH);
                            timer_hold = true;
                        }
                        break;
                    case CLEAR_KEY:
                        if (keyEvent.key_value == KEY_UP) {
                            if (trigger_sleep_count < SLEEP_COUNT) {
                                //start count for set zero
                                set_zero_count = 0; 
                            }
                        } else if (keyEvent.key_value == KEY_HOLD) {
                            trigger_sleep_count++;
                            if (trigger_sleep_count >= SLEEP_COUNT) {
                                done = true;
                            }
                        }
                        break;
                    case SLEEP_KEY:
                    case FIRMWARE_UPGRADE_KEY:
                        done = true;
                        auto_shutdown = true;
                        break;
                    case CHARGE_KEY:
                        display_indicate_charging();
                        break;
                    case NOT_CHARGE_KEY:
                        display_disable_charging();
                        break;
                    default:
                        break;
                }
                break;
            case WORK_STATUS_CALIBRATION:
                if (keyEvent.key_type == CLEAR_KEY && keyEvent.key_value == KEY_DOWN) {
                    if (cal_key_repeat_count < 0) {
                        calibrate_tick = 0;
                    }
                }else if (keyEvent.key_type == TIMER_KEY && keyEvent.key_value == KEY_UP) {
                    if (cal_key_repeat_count > 0) {
                        cal_key_repeat_count = -1;
                    } else {
                        //exit calibration mode
                        ESP_LOGD(TAG,"exit calibration mode\n");
                        display_restore();
                        work_status = WORK_STATUS_NORMAL;
                    }
                }
                break;
            case WORK_STATUS_SHUTDOWN:
                if (keyEvent.key_value == KEY_UP) {
                    done = true;
                }
                break;
            default:
                break;
        }
    }
}

static int32_t getAdcValue(uint8_t scaleChannel)
{
    switch(scaleChannel) {
        case SCALE_UP:
            return gpio_adc_get_value();
            break;
        case SCALE_DOWN:
            return spi_adc_get_value();
            break;
        default:
            break;
    }
    return 0;
}

static int32_t parseAdcValue(uint8_t scaleChannel, int32_t adcValue)
{
    int32_t weight = convert_weight(scaleChannel, adcValue, false);
    uint8_t weight_unit = config_get_weight_unit();
    int32_t rtn=0;

    //trace zero
    bool traced = zero_trace(scaleChannel, adcValue, weight, 100);
    if (traced) return 0;       //traced zero

    //handle lock
    if (display_lock[scaleChannel] == DISPLAY_UNLOCKED) {
        //not locked
        if (abs(lock_weight[scaleChannel]-weight) >= DISPLAY_LOCK_THRESHOLD_200MG){
            display_lock_count[scaleChannel] = 0;
            lock_weight[scaleChannel] = weight;
            //reset timeout timer
            bs_timer_reset(TIMER_TIMEOUT);
        }else{
            display_lock_count[scaleChannel]++;
            //3s not change, lock display
            if (display_lock_count[scaleChannel] == DISPLAY_LOCK_COUNT) {
                lock_display(scaleChannel, DISPLAY_LOCKED);
                lock_weight[scaleChannel] = weight;
            }
        }
        //return unlock value
        rtn = weight;
    } else if(display_lock[scaleChannel] == DISPLAY_UNLOCKING) {
        static int unlocking_count = 0;
        unlocking_count++;
        //reset timeout timer
        bs_timer_reset(TIMER_TIMEOUT);
        /*
        if (unlocking_count == 2 ) {
            lock_weight[scaleChannel] += (weight-lock_weight[scaleChannel])/2;
        } else if (unlocking_count == 3) {
            lock_display(scaleChannel, DISPLAY_UNLOCKED);
            unlocking_count = 0;
        }
        */
        if (unlocking_count == 2 ) {
            lock_display(scaleChannel, DISPLAY_UNLOCKED);
            unlocking_count = 0;
        }
        rtn = lock_weight[scaleChannel];
    } else {
        //locked, check if unlock
        if (abs(lock_weight[scaleChannel]-weight) >= DISPLAY_LOCK_THRESHOLD_200MG){
            lock_display(scaleChannel, DISPLAY_UNLOCKING);
            lock_display(SCALE_DOWN-scaleChannel, DISPLAY_UNLOCKED);
            //we need unlocking status
            //lock_weight[scaleChannel] += (weight-lock_weight[scaleChannel])/3;
            lock_weight[scaleChannel] += (weight-lock_weight[scaleChannel])/2;
            //reset timeout timer
            bs_timer_reset(TIMER_TIMEOUT);
        }
        //return locked value
        rtn = lock_weight[scaleChannel];
    }
    return (weight_unit == WEIGHT_UNIT_G)?rtn:(rtn * 100 / 2835);
}

static bool detectDualScale()
{
    int32_t upAdcValue = getAdcValue(SCALE_UP);
    int32_t absWeight = convert_weight(SCALE_UP, upAdcValue, true);
    uint8_t weightUnit = config_get_weight_unit();
    if (((weightUnit == WEIGHT_UNIT_G) && (absWeight > DOUBLE_SCALE_THRESHOLD_G))
            || ((weightUnit == WEIGHT_UNIT_OZ) && (absWeight > DOUBLE_SCALE_THRESHOLD_OZ))) {
        display_seticon(ICON_ALL, true);
        return true;
    }else{
        //turn off display
        display_seticon(ICON_ALL, false);
        setDisplayNumberOff(DISPLAY_CHANNEL_UP);
    }
    return false;
}

static bool scaleOverFlow(uint8_t scaleChannel, int32_t scaleWeight)
{
    uint8_t weightUnit = config_get_weight_unit();
    int32_t maxWeight = 0;

    switch(scaleChannel) {
        case SCALE_UP:
            if (weightUnit == WEIGHT_UNIT_G) {
                maxWeight = SCALE_UP_MAX_WEIGHT_G;
            }else{
                maxWeight = SCALE_UP_MAX_WEIGHT_OZ;
            }
            break;
        case SCALE_DOWN:
            if (weightUnit == WEIGHT_UNIT_G) {
                maxWeight = SCALE_DOWN_MAX_WEIGHT_G;
            }else{
                maxWeight = SCALE_DOWN_MAX_WEIGHT_OZ;
            }
            break;
        default:
            return false;
    }
    return scaleWeight > maxWeight;
}

void app_main()
{
    ESP_LOGI(TAG, "BLACK STONE!!!");
    ESP_LOGI(TAG, "version: %s", FW_VERSION);
    ESP_LOGI(TAG, "model: %s", MODEL_NUMBER);

    /* config */
    config_init();
    //init battery first
    battery_init();

    /* start event queue */
    eventQueue = xQueueCreate(10, sizeof(key_event_t));
    xTaskCreate(&handle_key_event, "key_event_task", 4096, NULL, 2, &xKeyTaskHandler);

    /* Initialise key */
    gpio_key_init();

    /* No Power */
    if (!is_charging() && is_battery_extremely_low()) {
        ESP_LOGD(TAG, "not charging and battery extremely low\n");
        //enter sleep
        enter_sleep();
    }

    /* Initialise display */
    display_init();

    /* indicate user to charge*/
    if (!is_charging() && is_battery_level_low()) {
        ESP_LOGD(TAG, "not charging and battery low\n");
        //indicate charge
        display_indicate_charge_only();
        //enter sleep
        display_stop();
        enter_sleep();
    }

    // wake up by use means not by key
    if (!is_wakeup_by_key()) {
        ESP_LOGD(TAG,"not wake up by power key\n");

        //enter charging sleep loop
        work_status = WORK_STATUS_CHARGING_SLEEP;

        charging = is_charging();
        ESP_LOGD(TAG,"charging: %d\n", charging);

        //loop to get power key and charge event
        while(charging && no_key_start) {
            display_indicate_charging_only();
            vTaskDelay(1000/portTICK_RATE_MS);
        }

        if (!charging) {
            //enter sleep
            display_stop();
            enter_sleep();
        }else{
            //reset status to init
            work_status = WROK_STATUS_INIT;
        }
    }

    /* start battery */
    battery_start();

    /* start keyboard */
    gpio_key_start();

    /* config */
    config_load();
    calibration_init();

    /* Init weight unit LED */
    led_init();

    /* Initialise adc */
    spi_adc_init();
    gpio_adc_init();
    vTaskDelay(1/portTICK_RATE_MS);

    /* Initialise wifi service*/
    ws_init();

    /* Initialise gatts service */
    bt_init();

    /* Initialise timer */
    bs_timer_init();

    /* start display */
    display_start();
    if (is_charging()) {
        display_indicate_charging();    
    }

    ESP_LOGD(TAG,"enter main loop!!!\n");
    work_status = WORK_STATUS_NORMAL;

    cal_set_zero(SCALE_UP, getAdcValue(SCALE_UP));
    cal_set_zero(SCALE_DOWN, getAdcValue(SCALE_DOWN));

    int32_t calibration_data[SCALE_NUM][CALIBRATION_NUMS] = {0};
    bool dualScale = false;
    while(!done) {
        vTaskDelay(125/portTICK_RATE_MS);
        if (work_status == WORK_STATUS_NORMAL || work_status == WORK_STATUS_WORKING) {
            //ESP_LOGI(TAG, "%d ----- %d", getAdcValue(SCALE_UP), getAdcValue(SCALE_DOWN));
            dualScale = detectDualScale();
            int32_t upAdcValue = 0;
            if (dualScale) {
                upAdcValue = getAdcValue(SCALE_UP);
            }
            int32_t downAdcValue = getAdcValue(SCALE_DOWN);
            if (set_zero_count >= 0 ) {
                set_zero_count++;
            }
            //hanlde set zero first
            if (set_zero_count > SET_ZERO_COUNT) {
                if (dualScale) {
                    cal_set_zero(SCALE_UP, upAdcValue);
                    setDisplayNumber(DISPLAY_CHANNEL_UP, 0);
                    bt_set_weight(DISPLAY_CHANNEL_UP, 0);
                    lock_weight[SCALE_UP] = 0;
                }
                cal_set_zero(SCALE_DOWN, downAdcValue);
                setDisplayNumber(DISPLAY_CHANNEL_DOWN, 0);
                bt_set_weight(DISPLAY_CHANNEL_DOWN, 0);
                lock_weight[SCALE_DOWN] = 0;
                set_zero_count = -1;
                continue;
            }

            int32_t upWight = 0;
            int32_t downWeight = parseAdcValue(SCALE_DOWN, downAdcValue);
            if (scaleOverFlow(SCALE_DOWN, downWeight)) {
                downWeight = 100000;
            }
            if (dualScale) {
                upWight = parseAdcValue(SCALE_UP, upAdcValue);
                if (scaleOverFlow(SCALE_UP, upWight)) {
                    upWight = 100000;
                }
                setDisplayNumber(DISPLAY_CHANNEL_UP, downWeight);
                bt_set_weight(DISPLAY_CHANNEL_UP, downWeight);
            }else{
                //make sure gatt not indicate 
                bt_set_weight(DISPLAY_CHANNEL_UP, -1);
            }
            //set display
            int32_t totalWeight = upWight+downWeight;
            setDisplayNumber(DISPLAY_CHANNEL_DOWN, totalWeight);
            bt_set_weight(DISPLAY_CHANNEL_DOWN, totalWeight);

            //check if we need do alarm
            if ((config_get_alarm_enable()==1) && (work_status == WORK_STATUS_WORKING)) {
                if (downWeight >= config_get_alarm_weight()) {
                    if (!alarmed ) {
                        alarmNumber();
                        alarmed = true;
                    }
                }else{
                    alarmed = false;
                }
            }
        } else if (work_status == WORK_STATUS_CALIBRATION && calibrate_tick >=0 ) {
            calibrate_tick++;
            if (calibrate_step == CALIBRATION_STEP_INIT && calibrate_tick >= 2) {
                ESP_LOGD(TAG,"calibration_init done\n");
                //calibration init
                display_backup();
                display_setOperation(OPERATION_CALIBRATION, 255, 255, 255, 3);
                beap(100, 200);
                beap(100, 400);
                calibrate_tick = -1;
                calibrate_step++;
                memset(calibration_data,0,sizeof(calibration_data));
            }
            if (calibrate_step < CALIBRATION_STEP_NUM && calibrate_tick >= 10) {
                //set calibration
                int32_t up = getAdcValue(SCALE_UP);
                int32_t down = getAdcValue(SCALE_DOWN);
                ESP_LOGD(TAG,"%d: %d\n", up, down);
                calibration_data[SCALE_UP][calibrate_step-CALIBRATION_STEP_ZERO] = up;
                calibration_data[SCALE_DOWN][calibrate_step-CALIBRATION_STEP_ZERO] = down;
                display_setOperation(OPERATION_CALIBRATION, 255, 255, 255, CALIBRATION_STEP_NUM-calibrate_step-1);
                beap(0, 200);
                calibrate_step++;
                calibrate_tick = -1;
            }
            if (calibrate_step >= CALIBRATION_STEP_NUM) {
                //exit calibration mode
                ESP_LOGD(TAG,"exist calibration mode\n");
                set_calibration(calibration_data[SCALE_UP], calibration_data[SCALE_DOWN]);
                display_restore();
                work_status = WORK_STATUS_NORMAL;
                beap(100,400);
            }
        }
    }

    ESP_LOGD(TAG,"quit main loop\n");
    work_status = WORK_STATUS_SHUTDOWN;
    if (!auto_shutdown) {
        done = false;       //should wait for up
    }

    spi_adc_shutdown();
    gpio_adc_shutdown();
    bs_timer_deinit();
    bt_stop();
    //check ota
    if (ws_get_status() == WIFI_STATUS_CONNECTED) {
        firmware_t* firmware = config_get_firmware_upgrade();
        if (firmware->host != NULL) {
            if (!ota_task(firmware)){
                done = false;
                while(!done) {
                    vTaskDelay(100/portTICK_RATE_MS);
                }
            }
        }
    }
    display_stop();
    ws_stop();
    gpio_key_pre_stop();
    // wait for power key up
    while(!done) {
        vTaskDelay(100/portTICK_RATE_MS);
    }
    gpio_key_stop();
    if( xKeyTaskHandler != NULL ) vTaskDelete( xKeyTaskHandler );
    battery_stop();
    if (is_charging()) {
        esp_restart();
    }else{
        enter_sleep();
    }
}
