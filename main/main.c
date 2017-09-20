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
#include "display.h"
#include "bs_timer.h"
#include "delay_timer.h"
#include "queue_buffer.h"
#include "key_event.h"
#include "calibration.h"
#include "spi_adc.h"
#include "gpio_adc.h"
#include "config.h"

#define GPIO_LED_IO                 19
#define WORKING_MODE_NORMAL         0
#define WORKING_MODE_CALIBRATION    1
#define DISPLAY_LOCK_THRESHOLD      15      //1.5g
#define REPEAT_COUNT_CALIBRATION    6

extern void bt_init();
extern void gpio_key_init();
extern void bs_wifi_init();
extern bool battery_init();
extern void beap(int wait, int duration);
extern void set_calibration(int index, int32_t channel0, int32_t channel1);

static const char *TAG = "black_stone";
static int working_mode = WORKING_MODE_NORMAL;
static int calibrate_tick = -1;
static int calibrate_index = 0;
static bool display_lock[2] = {false, false};
static int display_lock_count[2] = {0,0};
static int32_t last_weight[2] = {0,0};
static bool done = false;
static int trigger_sleep_count = 0;
static int key_repeat_count = 0;
/*
static queue_buffer_t* dataQueueBuffer[2] = { &qb_SpiAdcData, &qb_GpioAdcData};
static queue_buffer_t* calibrationQueueBuffer[2] = { &qb_SpiAdcCalibration, &qb_GpioAdcCalibration};
*/

static void lock_display(int channel, bool lock)
{
    display_lock[channel] = lock;
    if (!lock) {
        display_lock_count[channel] = 0;
    }
    printf("lock_display[%d]: %d!!!\n", channel, lock);
}

static void led_on()
{
    printf("%s: led gpio_init !!!\n", TAG);
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

static void do_sleep()
{
    const int ext_wakeup_pin_1 = 33;
    const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;

    //enable gpio wakeup
    esp_deep_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

    printf("enter sleep now\n");
    esp_deep_sleep_start();
}

static void enter_sleep()
{
    printf("enter sleep\n");

    done = true;
    vTaskDelay(100/portTICK_RATE_MS);       //wait main loop to exist


    //turn off display
    display_shutdown();

    //turn off adc
    // spi_adc_shutdown();
    gpio_adc_shutdown();

    //do sleep
    do_sleep();
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

void handle_key_event(key_event_t keyEvent)
{
    if (working_mode == WORKING_MODE_NORMAL) {
        printf("%s: handle key evnet(%d, %d) !!!\n", TAG, keyEvent.key_type, keyEvent.key_value);
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
                        enter_sleep();
                    }else if (key_repeat_count >= REPEAT_COUNT_CALIBRATION) {
                        //do calibration
                        printf("enter calibration mode\n");
                        beap(0, 400);
                        working_mode = WORKING_MODE_CALIBRATION;
                        // spi_adc_calibration(true);
                        gpio_adc_calibration(true);
                    }else if (key_repeat_count == 0){
                        //set zero
                        int32_t adcValue[2];
                        // adcValue[0] = spi_adc_get_value();
                        adcValue[0] = gpio_adc_get_value(0);
                        adcValue[1] = gpio_adc_get_value(1);
                        for (int i = 0; i < 2; ++i)
                        {
                            set_zero(i,adcValue[i]);
                            setDisplayNumber(i, 0, 0);
                            // lock_display(i, false);
                        }
                    }
                } else if (keyEvent.key_value == KEY_HOLD) {
                    trigger_sleep_count++;
                }
                break;
            case SLEEP_KEY:
                enter_sleep();
                break;
            default:
                break;
        }
    } else {
        printf("%s: handle key evnet(%d, %d) !!!\n", TAG, keyEvent.key_type, keyEvent.key_value);
        if (keyEvent.key_type == CLEAR_KEY && keyEvent.key_value == KEY_DOWN) {
            calibrate_tick = 0;
        }
    }
}

void app_main()
{
    printf("BLACK STONE!!!\n");
	ESP_LOGI(TAG, "Start!!!");

    //init battery first
    if (!battery_init()) {
        do_sleep();
    }

    /* led */
    led_on();

    /* config */
    config_init();
    get_zero();
    get_calibrations();

    /* Initialise wifi */
    // bs_wifi_init();

    /* Initialise bluetooth */
    // bt_init();

    /* Initialise timer */
    bs_timer_init();
    // delay_timer_init();

    /* Initialise adc */
    // spi_adc_init();
    gpio_adc_init();

    /* Initialise display */
    display_init();

    /* Initialise key */
    gpio_key_init();

    printf("working mode :%s!!!\n", working_mode == WORKING_MODE_NORMAL? "normal":"calibration");

    while(!done) {
        vTaskDelay(100/portTICK_RATE_MS);

        /*
        if (working_mode == WORKING_MODE_CALIBRATION) {
            printf("0: %d ---- 1: %d\n", queue_average(&calibrationQueueBuffer[0]), queue_average(&calibrationQueueBuffer[1]));
        }else{
            printf("0: %d ---- 1: %d\n", queue_average(&dataQueueBuffer[0]), queue_average(&dataQueueBuffer[1]));
        }
        */

        if (working_mode == WORKING_MODE_NORMAL) {
            int32_t adcValue[2];
            // adcValue[0] = spi_adc_get_value();
            adcValue[0] = gpio_adc_get_value(0);
            adcValue[1] = gpio_adc_get_value(1);
            for (int i = 0; i < 2; ++i)
            {
                int8_t precision = 0;
                int32_t weight = get_weight(adcValue[i], i, &precision);

                // change more than 1.5g, unlock display
                if (display_lock[i] && (abs(last_weight[i]-weight) > DISPLAY_LOCK_THRESHOLD)) {
                    lock_display(i, false);
                    last_weight[i] = weight;
                }else if (!display_lock[i]) {
                    // change more than 0.5g, clear count
                    if (abs(last_weight[i]-weight) > 5){
                        display_lock_count[i] = 0;
                        last_weight[i] = weight;
                    }else{
                        display_lock_count[i]++;
                        //1s not change, and weight < 0.5g, clear zero
                        if ( (abs(weight) < 10) && (display_lock_count[i] == 10)) {
                            set_zero(i, adcValue[i]);
                        }
                        //3s not change, lock display
                        if (display_lock_count[i] == 30) {
                            lock_display(i, true);                            
                        }
                    }
                }
                if (!display_lock[i]) {
                    if (weight >10000 || weight<-1000) weight/=10;
                    setDisplayNumber(i==0?1:0 , weight, precision);
                }
            }
        } else if (working_mode == WORKING_MODE_CALIBRATION && calibrate_tick >=0 ) {
            calibrate_tick++;
            // printf("tick: %d\n", calibrate_tick);
            if (calibrate_tick >= 30) {
                // int32_t cal1 = spi_adc_get_value();
                int32_t cal1 = gpio_adc_get_value(0);
                int32_t cal2 = gpio_adc_get_value(1);
                printf("%d: %d\n", cal1, cal2);
                set_calibration(calibrate_index++, cal1, cal2);
                beap(0, 200);
                calibrate_tick = -1;
                //exit calibration mode
                if (calibrate_index >= CALIBRATION_NUMS) {
                    printf("exist calibration mode\n");
                    working_mode = WORKING_MODE_NORMAL;
                    // spi_adc_calibration(false);
                    gpio_adc_calibration(false);
                    calibrate_index = 0;
                    beap(100,400);
                }
            }
        }
    }
    printf("quit main loop\n");
}