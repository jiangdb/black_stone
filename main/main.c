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

#define GPIO_LED_IO                 19
#define DISPLAY_LOCK_THRESHOLD      5       //0.5g
#define DOUBLE_SCALE_THRESHOLD_G    500     //50g
#define DOUBLE_SCALE_THRESHOLD_OZ   17      //1.7oz
#define REPEAT_COUNT_CALIBRATION    6

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
    UP_SCALE,
    DOWN_SCALE,
    SCALE_NUM
};

enum {
    CALIBRATION_STEP_INIT,
    CALIBRATION_STEP_ZERO,
    CALIBRATION_STEP_ONE,
    CALIBRATION_STEP_TWO,
    CALIBRATION_STEP_NUM,
};

static int calibrate_tick = -1;
static int calibrate_step = 0;
static bool display_lock[2] = {false, false};
static int display_lock_count[2] = {0,0};
static int set_zero_count = -1;
static int32_t last_weight[2] = {0,0};
static bool done = false;
static int trigger_sleep_count = 0;
static int key_repeat_count = 0;
static int work_status = WROK_STATUS_INIT;
static bool charging = true;
static bool no_key_start = true;
static bool alarmed = false;
static bool doubleScale = false;
static xQueueHandle eventQueue;
static TaskHandle_t xHandle = NULL;

static void lock_display(int channel, bool lock)
{
    display_lock[channel] = lock;
    if (!lock) {
        display_lock_count[channel] = 0;
    }
    ESP_LOGD(TAG,"lock_display[%d]: %d!!!\n", channel, lock);
}

static void led_on()
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
    gpio_set_level(GPIO_LED_IO, 1);
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

static int count_key_repeat ()
{
    static uint32_t lastDataReadyTime = 0;

    uint32_t currtime=xthal_get_ccount();
    uint32_t diff=currtime-lastDataReadyTime;

    //consider 500ms (240000*500) as 
    if ( diff < 120000000 ) {
        key_repeat_count++;
    } else {
        key_repeat_count = 0;
    }
    lastDataReadyTime=currtime;

    ESP_LOGD(TAG,"repeat count %d\n", key_repeat_count);
    return key_repeat_count;
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
    while(1) {
        key_event_t keyEvent;
        xQueueReceive(eventQueue, &keyEvent, portMAX_DELAY);

        ESP_LOGD(TAG,"%s: handle key evnet(%d, %d) work_status: %d!!!\n", TAG, keyEvent.key_type, keyEvent.key_value, work_status);
        //reset timeout timer
        bs_timer_reset(TIMER_TIMEOUT);
        //handle key event
        switch(work_status) {
            case WORK_STATUS_CHARGING_SLEEP:
                if (keyEvent.key_type == NOT_CHARGE_KEY) {
                    charging = false;
                }else if (keyEvent.key_type == CLEAR_KEY) {
                    if (keyEvent.key_value == KEY_UP) {
                        no_key_start = false;
                    }
                }
                break;
            case WORK_STATUS_NORMAL:
                switch(keyEvent.key_type){
                    case TIMER_KEY:
                        if (keyEvent.key_value == KEY_DOWN) {
                            if (bs_timer_toggle(TIMER_STOPWATCH)) {
                                work_status = WORK_STATUS_WORKING;
                            }
                        }
                        break;
                    case CLEAR_KEY:
                        if (keyEvent.key_value == KEY_DOWN) {
                            count_key_repeat();
                        }else if (keyEvent.key_value == KEY_UP) {
                            if (trigger_sleep_count >= 1) {
                                trigger_sleep_count = 0;
                                // enter_sleep();
                                done = true;
                            } else if (key_repeat_count == REPEAT_COUNT_CALIBRATION) {
                                //do calibration
                                set_zero_count = -1; 
                                ESP_LOGD(TAG,"enter calibration mode\n");
                                work_status = WORK_STATUS_CALIBRATION;
                                calibrate_tick = 0;
                                calibrate_step = CALIBRATION_STEP_INIT;
                            } else {
                                //start count for set zero
                                set_zero_count = 0; 
                            }
                        } else if (keyEvent.key_value == KEY_HOLD) {
                            trigger_sleep_count++;
                        }
                        break;
                    case SLEEP_KEY:
                    case FIRMWARE_UPGRADE_KEY:
                        done = true;
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
                        if (keyEvent.key_value == KEY_DOWN) {
                            bs_timer_toggle(TIMER_STOPWATCH);
                        }else if (keyEvent.key_value == KEY_HOLD) {
                            bs_timer_stop(TIMER_STOPWATCH);
                            work_status = WORK_STATUS_NORMAL;
                        }
                        break;
                    case CLEAR_KEY:
                        if (keyEvent.key_value == KEY_UP) {
                            if (trigger_sleep_count >= 1) {
                                trigger_sleep_count = 0;
                                // enter_sleep();
                                done = true;
                            } else {
                                //start count for set zero
                                set_zero_count = 0; 
                            }
                        } else if (keyEvent.key_value == KEY_HOLD) {
                            trigger_sleep_count++;
                        }
                        break;
                    case SLEEP_KEY:
                    case FIRMWARE_UPGRADE_KEY:
                        done = true;
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
                    if (count_key_repeat() > REPEAT_COUNT_CALIBRATION && calibrate_step == CALIBRATION_STEP_INIT) {
                        //exit calibration mode
                        ESP_LOGD(TAG,"exist calibration mode\n");
                        work_status = WORK_STATUS_NORMAL;
                    }else{
                        calibrate_tick = 0;
                    }
                }else if (keyEvent.key_type == TIMER_KEY && keyEvent.key_value == KEY_DOWN) {
                    //exit calibration mode
                    ESP_LOGD(TAG,"exist calibration mode\n");
                    display_restore();
                    work_status = WORK_STATUS_NORMAL;
                }
                break;
            case WORK_STATUS_SHUTDOWN:
                done = true;
                break;
            default:
                break;
        }
    }
}

int get_work_status()
{
    return work_status;
}

static int32_t processAdcValue(int channel, int32_t value, int32_t weightAddValue)
{
    int32_t weight = convert_weight(channel, value, false);

    // auto track zero
    bool tracked = zero_track(channel, value, weight, 100);
    if (tracked) weight = 0;

    // add any weight if we want, e.g UP_SCALE weight
    weight += weightAddValue;

    // change more than 1.5g, unlock display
    if (display_lock[channel] && (abs(last_weight[channel]-weight) > DISPLAY_LOCK_THRESHOLD)) {
        lock_display(channel, false);
        last_weight[channel] = weight;
    }else if (!display_lock[channel]) {
        // change more than 0.5g, clear count
        if (abs(last_weight[channel]-weight) > 5){
            display_lock_count[channel] = 0;
            last_weight[channel] = weight;
        }else{
            display_lock_count[channel]++;
            //3s not change, lock display
            if (display_lock_count[channel] == 30) {
                lock_display(channel, true);
            }
        }
    }
    if (!display_lock[channel] || tracked) {
        //check if we need do alarm
        if ((config_get_alarm_enable()==1) && (channel==DOWN_SCALE) && (work_status == WORK_STATUS_WORKING)) {
            uint16_t alarmWeight = config_get_alarm_weight(); 
            if (weight >= alarmWeight) {
                if (!alarmed ) {
                    alarmNumber();
                    alarmed = true;
                }
            }else{
                alarmed = false;
            }
        }
        setDisplayNumber(channel, weight);
        bt_set_weight(channel, weight);
    }
    return weight;
}

void app_main()
{
    ESP_LOGI(TAG, "BLACK STONE!!!");
    ESP_LOGI(TAG, "version: %s", FW_VERSION);
    ESP_LOGI(TAG, "model: %s", MODEL_NUMBER);

    //init battery first
    battery_init();

    /* start event queue */
    eventQueue = xQueueCreate(10, sizeof(key_event_t));
    xTaskCreate(&handle_key_event, "key_event_task", 4096, NULL, 2, &xHandle);

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

    /* led */
    // led_on();

    /* start battery */
    battery_start();

    /* start keyboard */
    gpio_key_start();

    /* config */
    config_init();
    calibration_init();

    /* Initialise wifi service*/
    ws_init();

    /* Initialise gatts service */
    bt_init();

    /* Initialise timer */
    bs_timer_init();

    /* Initialise adc */
    spi_adc_init();
    gpio_adc_init();

    /* start display */
    display_start();
    if (is_charging()) {
        display_indicate_charging();    
    }

    ESP_LOGD(TAG,"enter main loop!!!\n");
    work_status = WORK_STATUS_NORMAL;

    int32_t calibration_data[2][3] = {0};
    while(!done) {
        vTaskDelay(100/portTICK_RATE_MS);
        if (work_status == WORK_STATUS_NORMAL || work_status == WORK_STATUS_WORKING) {
            int32_t upAdcValue = gpio_adc_get_value();
            if (!doubleScale) {
                int32_t absWeight = convert_weight(0, upAdcValue, true);
                uint8_t weightUnit = config_get_weight_unit();
                if (((weightUnit == WEIGHT_UNIT_G) && (absWeight > DOUBLE_SCALE_THRESHOLD_G))
                        || ((weightUnit == WEIGHT_UNIT_OZ) && (absWeight > DOUBLE_SCALE_THRESHOLD_OZ))) {
                    doubleScale = true;
                }
            }
            int32_t upWight = 0;
            if (doubleScale) {
                upWight = processAdcValue(UP_SCALE, upAdcValue, 0);
            }
            int32_t downAdcValue = spi_adc_get_value();
            processAdcValue(DOWN_SCALE, downAdcValue, upWight);

            //need hanlde zero
            if (set_zero_count >= 0 ) {
                set_zero_count++;
                if (set_zero_count > 10) {
                    if (doubleScale) {
                        cal_set_zero(0, upAdcValue);
                        setDisplayNumber(0, 0);
                        bt_set_weight(0, 0);
                    }
                    cal_set_zero(1, downAdcValue);
                    setDisplayNumber(1, 0);
                    bt_set_weight(1, 0);
                    set_zero_count = -1;
                }
            } 
        } else if (work_status == WORK_STATUS_CALIBRATION && calibrate_tick >=0 ) {
            calibrate_tick++;
            if (calibrate_step == CALIBRATION_STEP_INIT && calibrate_tick >= 5) {
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
            if (calibrate_step < CALIBRATION_STEP_NUM && calibrate_tick >= 20) {
                //set calibration
                int32_t cal1 = gpio_adc_get_value();
                int32_t cal2 = spi_adc_get_value();
                ESP_LOGD(TAG,"%d: %d\n", cal1, cal2);
                calibration_data[0][calibrate_step-CALIBRATION_STEP_ZERO] = cal1;
                calibration_data[1][calibrate_step-CALIBRATION_STEP_ZERO] = cal2;
                display_setOperation(OPERATION_CALIBRATION, 255, 255, 255, CALIBRATION_STEP_NUM-calibrate_step-1);
                beap(0, 200);
                calibrate_step++;
                calibrate_tick = -1;
            }
            if (calibrate_step >= CALIBRATION_STEP_NUM) {
                //exit calibration mode
                ESP_LOGD(TAG,"exist calibration mode\n");
                set_calibration(calibration_data[0], calibration_data[1]);
                display_restore();
                work_status = WORK_STATUS_NORMAL;
                beap(100,400);
            }
        }
    }

    ESP_LOGD(TAG,"quit main loop\n");
    work_status = WORK_STATUS_SHUTDOWN;

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
    gpio_key_stop();
    if( xHandle != NULL ) vTaskDelete( xHandle );
    battery_stop();
    if (is_charging()) {
        esp_restart();
    }else{
        enter_sleep();
    }
}
