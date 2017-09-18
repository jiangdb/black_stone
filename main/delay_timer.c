/* Timer group-hardware timer example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_types.h"
#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "delay_timer.h"

#define TIMER_INTR_SEL TIMER_INTR_LEVEL  /*!< Timer level interrupt */
#define TIMER_DIVIDER   80               /*!< Hardware timer clock divider */
#define TIMER_SCALE    (TIMER_BASE_CLK / TIMER_DIVIDER)  /*!< used to calculate counter value */
#define TIMER_INTERVAL_5_US   (5)   /*!< test interval for timer 1 */

#define DELAY_TIMER_GROUP       TIMER_GROUP_1
#define DELAY_TIMER_INDEX       TIMER_0

static SemaphoreHandle_t semDelay5Us = NULL;

/*
 * @brief timer group0 ISR handler
 */
void IRAM_ATTR delay_timer_isr(void *para)
{
    int timer_idx = (int) para;
    uint32_t intr_status = TIMERG1.int_st_timers.val;
    if((intr_status & BIT(timer_idx)) && timer_idx == DELAY_TIMER_INDEX) {
        /*Timer1 is an example that will reload counter value*/
        TIMERG1.hw_timer[timer_idx].update = 1;
        /*We don't call a API here because they are not declared with IRAM_ATTR*/
        TIMERG1.int_clr_timers.t0 = 1;
        TIMERG1.hw_timer[timer_idx].config.alarm_en = 1;
        // release semaphore
        BaseType_t mustYield=false;
        xSemaphoreGiveFromISR(semDelay5Us, &mustYield);
        if (mustYield) portYIELD_FROM_ISR();
    }
}

void delay_5us()
{
    timer_set_counter_value(DELAY_TIMER_GROUP, DELAY_TIMER_INDEX, 0x00000000ULL);
    timer_start(DELAY_TIMER_GROUP, DELAY_TIMER_INDEX);
    xSemaphoreTake( semDelay5Us, portMAX_DELAY );
}

void delay_timer_stop()
{
    timer_pause(DELAY_TIMER_GROUP, DELAY_TIMER_INDEX);
    timer_set_counter_value(DELAY_TIMER_GROUP, DELAY_TIMER_INDEX, 0x00000000ULL);
}

void delay_timer_init()
{
    //Create the semaphore.
    semDelay5Us=xSemaphoreCreateBinary();

    timer_config_t config;
    config.alarm_en = 1;
    config.auto_reload = 1;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = TIMER_DIVIDER;
    config.intr_type = TIMER_INTR_SEL;
    config.counter_en = TIMER_PAUSE;
    /*Configure timer*/
    timer_init(DELAY_TIMER_GROUP, DELAY_TIMER_INDEX, &config);
    /*Stop timer counter*/
    timer_pause(DELAY_TIMER_GROUP, DELAY_TIMER_INDEX);
    /*Load counter value */
    timer_set_counter_value(DELAY_TIMER_GROUP, DELAY_TIMER_INDEX, 0x00000000ULL);
    /*Set alarm value*/
    timer_set_alarm_value(DELAY_TIMER_GROUP, DELAY_TIMER_INDEX, TIMER_INTERVAL_5_US);
    // timer_set_alarm_value(DELAY_TIMER_GROUP, DELAY_TIMER_INDEX, 2*TIMER_SCALE);

    /*Enable timer interrupt*/
    timer_enable_intr(DELAY_TIMER_GROUP, DELAY_TIMER_INDEX);
    /*Set ISR handler*/
    timer_isr_register(DELAY_TIMER_GROUP, DELAY_TIMER_INDEX, delay_timer_isr, (void*) DELAY_TIMER_INDEX, ESP_INTR_FLAG_IRAM, NULL);
}
