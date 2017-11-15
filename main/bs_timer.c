/* Timer group-hardware timer example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include "esp_types.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "bs_timer.h"
#include "display.h"
#include "key_event.h"

#define TAG  "TIMER"
/*
 * DEFINES
 */
#define TIMER_INTR_SEL          TIMER_INTR_LEVEL  /*!< Timer level interrupt */
#define TIMER_DIVIDER           80               /*!< Hardware timer clock divider */
#define TIMER_SCALE             (TIMER_BASE_CLK / TIMER_DIVIDER)  /*!< used to calculate counter value */
#define TIMER_INTERVAL_1_SEC    (1)     /*!< 1s */
#define TIMER_INTERVAL_1_MIN    (60)    /*!< 1m */
#define TIMER_INTERVAL_5_MIN    (300)   /*!< 5m */

/*
 * STRUCTS and ENUMS
 */
typedef struct {
    int timer_group;
    int timer_idx;
    bool timer_enabled;
}timer_def_t;

static uint32_t seconds=0;

static timer_def_t sTimers[TIMER_NUM] = {
    [TIMER_TIMEOUT] = {
        .timer_group = TIMER_GROUP_0,
        .timer_idx = TIMER_0,
        .timer_enabled = false
    },
    [TIMER_STOPWATCH] = {
        .timer_group = TIMER_GROUP_0,
        .timer_idx = TIMER_1,
        .timer_enabled = false
    },
};

/*
 * @brief timer group0 ISR handler
 */
void IRAM_ATTR timer_group_isr(void *para)
{
    int timer_idx = (int) para;
    uint32_t intr_status = TIMERG0.int_st_timers.val;
    if((intr_status & BIT(timer_idx))) {
        TIMERG0.hw_timer[timer_idx].update = 1;
        switch (timer_idx) {
            case TIMER_0:
                /*We don't call a API here because they are not declared with IRAM_ATTR*/
                TIMERG0.int_clr_timers.t0 = 1;
                if (sTimers[timer_idx].timer_enabled) {
                    /*For a auto-reload timer, we still need to set alarm_en bit if we want to enable alarm again.*/
                    TIMERG0.hw_timer[timer_idx].config.alarm_en = 1;

                    key_event_t keyEvent;
                    keyEvent.key_type = SLEEP_KEY;
                    keyEvent.key_value = KEY_DOWN;
                    send_key_event(keyEvent, true);
                }
                break;
            case TIMER_1:
                /*We don't call a API here because they are not declared with IRAM_ATTR*/
                TIMERG0.int_clr_timers.t1 = 1;
                if (sTimers[timer_idx].timer_enabled) {
                    /* set time in display */
                    setDisplayTime(++seconds);
                    /*For a auto-reload timer, we still need to set alarm_en bit if we want to enable alarm again.*/
                    TIMERG0.hw_timer[timer_idx].config.alarm_en = 1;
                }
                break;
            default:
                break;
        }
    }
}

/*
 * @brief timer group0 hardware timer0 init
 */
static void operation_timer_init()
{
    timer_config_t config;
    config.alarm_en = 1;
    config.auto_reload = 1;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = TIMER_DIVIDER;
    config.intr_type = TIMER_INTR_SEL;
    config.counter_en = TIMER_PAUSE;
    /*Configure timer*/
    timer_init(sTimers[TIMER_TIMEOUT].timer_group, sTimers[TIMER_TIMEOUT].timer_idx, &config);
    /*Stop timer counter*/
    timer_pause(sTimers[TIMER_TIMEOUT].timer_group, sTimers[TIMER_TIMEOUT].timer_idx);
    /*Load counter value */
    timer_set_counter_value(sTimers[TIMER_TIMEOUT].timer_group, sTimers[TIMER_TIMEOUT].timer_idx, 0x00000000ULL);
    /*Set alarm value*/
    timer_set_alarm_value(sTimers[TIMER_TIMEOUT].timer_group, sTimers[TIMER_TIMEOUT].timer_idx, TIMER_INTERVAL_5_MIN * TIMER_SCALE);
    /*Enable timer interrupt*/
    timer_enable_intr(sTimers[TIMER_TIMEOUT].timer_group, sTimers[TIMER_TIMEOUT].timer_idx);
    /*Set ISR handler*/
    timer_isr_register(sTimers[TIMER_TIMEOUT].timer_group, sTimers[TIMER_TIMEOUT].timer_idx, timer_group_isr, (void*) sTimers[TIMER_TIMEOUT].timer_idx, ESP_INTR_FLAG_IRAM, NULL);
}

/*
 * @brief timer group0 hardware timer1 init
 */
static void stopwatch_timer_init()
{
    timer_config_t config;
    config.alarm_en = 1;
    config.auto_reload = 1;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = TIMER_DIVIDER;
    config.intr_type = TIMER_INTR_SEL;
    config.counter_en = TIMER_PAUSE;
    /*Configure timer*/
    timer_init(sTimers[TIMER_STOPWATCH].timer_group, sTimers[TIMER_STOPWATCH].timer_idx, &config);
    /*Stop timer counter*/
    timer_pause(sTimers[TIMER_STOPWATCH].timer_group, sTimers[TIMER_STOPWATCH].timer_idx);
    /*Load counter value */
    timer_set_counter_value(sTimers[TIMER_STOPWATCH].timer_group, sTimers[TIMER_STOPWATCH].timer_idx, 0x00000000ULL);
    /*Set alarm value*/
    timer_set_alarm_value(sTimers[TIMER_STOPWATCH].timer_group, sTimers[TIMER_STOPWATCH].timer_idx, TIMER_INTERVAL_1_SEC * TIMER_SCALE);
    /*Enable timer interrupt*/
    timer_enable_intr(sTimers[TIMER_STOPWATCH].timer_group, sTimers[TIMER_STOPWATCH].timer_idx);
    /*Set ISR handler*/
    timer_isr_register(sTimers[TIMER_STOPWATCH].timer_group, sTimers[TIMER_STOPWATCH].timer_idx, timer_group_isr, (void*) sTimers[TIMER_STOPWATCH].timer_idx, ESP_INTR_FLAG_IRAM, NULL);
}

/*
 * main function
 */
void bs_timer_start(bs_timer_e timer)
{
    timer_start(sTimers[timer].timer_group, sTimers[timer].timer_idx);
    sTimers[timer].timer_enabled = true;
}

void bs_timer_stop(bs_timer_e timer)
{
    timer_pause(sTimers[timer].timer_group, sTimers[timer].timer_idx);
    timer_set_counter_value(sTimers[timer].timer_group, sTimers[timer].timer_idx, 0x00000000ULL);
    sTimers[timer].timer_enabled = false;
    if (timer == TIMER_STOPWATCH) {
        seconds = 0;
        ESP_LOGI(TAG,"set display time to 0 \n");
        setDisplayTime(seconds);
    }
}

void bs_timer_pause(bs_timer_e timer)
{
    timer_pause(sTimers[timer].timer_group, sTimers[timer].timer_idx);
    sTimers[timer].timer_enabled = false;
}

bool bs_timer_status(bs_timer_e timer)
{
    return sTimers[timer].timer_enabled;
}

bool bs_timer_toggle(bs_timer_e timer)
{
    if (!sTimers[timer].timer_enabled){
        bs_timer_start(timer);
        return true;
    }else{
        bs_timer_pause(timer);
        return false;;
    }
}

void bs_timer_reset(bs_timer_e timer)
{
    if (sTimers[timer].timer_enabled) {
        timer_set_counter_value(sTimers[timer].timer_group, sTimers[timer].timer_idx, 0x00000000ULL);
    }
}

void bs_timer_deinit()
{
    bs_timer_stop(TIMER_TIMEOUT);
    bs_timer_stop(TIMER_STOPWATCH);
}

void bs_timer_init()
{
    stopwatch_timer_init();
    operation_timer_init();
    bs_timer_start(TIMER_TIMEOUT);
}
