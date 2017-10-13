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
#include "bs_timer.h"
#include "queue_buffer.h"
#include "key.h"
#include "key_event.h"
#include "calibration.h"
#include "spi_adc.h"
#include "gpio_adc.h"
#include "config.h"
#include "zero_track.h"

#define GPIO_LED_IO                 19
#define DISPLAY_LOCK_THRESHOLD      15      //1.5g
#define REPEAT_COUNT_CALIBRATION    6

enum {
    WROK_STATUS_INIT,
    WORK_STATUS_CHARGING_SLEEP,
    WORK_STATUS_PRE_NORMAL,
    WORK_STATUS_NORMAL,
    WORK_STATUS_CALIBRATION
};

extern void bs_wifi_init();
extern void bs_wifi_stop();

static const char *TAG = "BS";
static int calibrate_tick = -1;
static int calibrate_index = 0;
static bool display_lock[2] = {false, false};
static int display_lock_count[2] = {0,0};
static int32_t last_weight[2] = {0,0};
static bool done = false;
static int trigger_sleep_count = 0;
static int key_repeat_count = 0;
static int work_status = WROK_STATUS_INIT;
static bool charging = true;
static bool no_key_start = true;

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

    return key_repeat_count;
}

void set_zero()
{
    //set zero
    int32_t adcValue[2];
    adcValue[0] = spi_adc_get_value();
    adcValue[1] = gpio_adc_get_value();
    for (int i = 0; i < 2; ++i)
    {
        cal_set_zero(i,adcValue[i]);
        setDisplayNumber(i, 0);
        bt_set_weight(i, 0);
    }
}


void handle_key_event(key_event_t keyEvent)
{
    ESP_LOGD(TAG,"%s: handle key evnet(%d, %d) work_status: %d!!!\n", TAG, keyEvent.key_type, keyEvent.key_value, work_status);
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
                        bs_timer_toggle();
                    }else if (keyEvent.key_value == KEY_HOLD) {
                        bs_timer_stop();
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
                        }else if (key_repeat_count >= REPEAT_COUNT_CALIBRATION) {
                            //do calibration
                            ESP_LOGD(TAG,"enter calibration mode\n");
                            beap(0, 400);
                            work_status = WORK_STATUS_CALIBRATION;
                        }else if (key_repeat_count == 0){
                            //set zero
                            set_zero();
                        }
                    } else if (keyEvent.key_value == KEY_HOLD) {
                        trigger_sleep_count++;
                    }
                    break;
                case SLEEP_KEY:
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
                calibrate_tick = 0;
            }
            break;
        default:
            break;
    }
}

int get_work_status()
{
    return work_status;
}

void app_main()
{
	ESP_LOGI(TAG, "BLACK STONE!!!");

    //init battery first
    battery_init();

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

    /* Initialise adc */
    spi_adc_init();
    gpio_adc_init();

    /* Initialise wifi */
    // bs_wifi_init();

    /* Initialise bluetooth */
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

    while(!done) {
        vTaskDelay(100/portTICK_RATE_MS);
        if (work_status == WORK_STATUS_NORMAL) {
            int32_t adcValue[2];
            adcValue[0] = spi_adc_get_value();
            adcValue[1] = gpio_adc_get_value();
            int32_t weight[2];
            for (int i = 0; i < 2; ++i)
            {
                weight[i] = convert_weight(i, adcValue[i]);

                // auto track zero
                bool tracked = zero_track(i, adcValue[i], weight[i], 100);
                if (tracked) weight[i] = 0;

                if (i == 1) weight[i]+=weight[0];
                // change more than 1.5g, unlock display
                if (display_lock[i] && (abs(last_weight[i]-weight[i]) > DISPLAY_LOCK_THRESHOLD)) {
                    lock_display(i, false);
                    last_weight[i] = weight[i];
                }else if (!display_lock[i]) {
                    // change more than 0.5g, clear count
                    if (abs(last_weight[i]-weight[i]) > 5){
                        display_lock_count[i] = 0;
                        last_weight[i] = weight[i];
                    }else{
                        display_lock_count[i]++;
                        //3s not change, lock display
                        if (display_lock_count[i] == 30) {
                            lock_display(i, true);
                        }
                    }
                }
                if (!display_lock[i] || tracked) {
                    setDisplayNumber(i, weight[i]);
                    bt_set_weight(i, weight[i]);
                }
            }
        } else if (work_status == WORK_STATUS_CALIBRATION && calibrate_tick >=0 ) {
            calibrate_tick++;
            if (calibrate_tick >= 30) {
                int32_t cal1 = spi_adc_get_value();
                int32_t cal2 = gpio_adc_get_value();
                ESP_LOGD(TAG,"%d: %d\n", cal1, cal2);
                set_calibration(calibrate_index++, cal1, cal2);
                beap(0, 200);
                calibrate_tick = -1;
                //exit calibration mode
                if (calibrate_index >= CALIBRATION_NUMS) {
                    ESP_LOGD(TAG,"exist calibration mode\n");
                    work_status = WORK_STATUS_NORMAL;
                    calibrate_index = 0;
                    beap(100,400);
                }
            }
        }
    }

    ESP_LOGD(TAG,"quit main loop\n");

    display_stop();
    spi_adc_shutdown();
    gpio_adc_shutdown();
    bs_timer_stop();
    bt_stop();
    // bs_wifi_stop();
    battery_stop();
    if (is_charging()) {
        esp_restart();
    }else{
        enter_sleep();
    }
}
