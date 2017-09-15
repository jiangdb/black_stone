/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "key_event.h"
#include "key.h"

/**
 * Externs:
 *
 */
extern void handle_key_event(key_event_t keyEvent);
extern int get_work_status();

static xQueueHandle gpio_evt_queue = NULL;
static bool enable_beep_vibrate = false;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

void beap(int wait, int duration)
{
    if (wait > 0) {
        vTaskDelay(wait/portTICK_RATE_MS);
    }
    gpio_set_level(GPIO_OUTPUT_IO_SPEAKER, 1);
    vTaskDelay(duration/portTICK_RATE_MS);
    gpio_set_level(GPIO_OUTPUT_IO_SPEAKER, 0);
}

static void beap_vibrate()
{
    gpio_set_level(GPIO_OUTPUT_IO_SPEAKER, 1);
    gpio_set_level(GPIO_OUTPUT_IO_VIBRATE, 1);
    vTaskDelay(100/portTICK_RATE_MS);
    gpio_set_level(GPIO_OUTPUT_IO_SPEAKER, 0);
    gpio_set_level(GPIO_OUTPUT_IO_VIBRATE, 0);
}

bool is_charging()
{
    int state1 = gpio_get_level(GPIO_INPUT_IO_STATE1);
    int state2 = gpio_get_level(GPIO_INPUT_IO_STATE2);

    if (state1 && state2) {
        // not charging
        return true;
    }
    return false;
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    int tick[2]= {-1, -1};
    TickType_t tick_type = portMAX_DELAY;

    while(1) {
        if(xQueueReceive(gpio_evt_queue, &io_num, tick_type)) {
            int val = gpio_get_level(io_num);
            // printf("gpio_key(%d): %d !!!\n", io_num, val);
            if (io_num == GPIO_INPUT_IO_STATE1 || io_num == GPIO_INPUT_IO_STATE2 ) {
                //got charging status change
                int state1 = gpio_get_level(GPIO_INPUT_IO_STATE1);
                int state2 = gpio_get_level(GPIO_INPUT_IO_STATE2);

                key_event_t keyEvent;
                if (state1 && state2) {
                    keyEvent.key_type = CHARGE_KEY;
                    handle_key_event(keyEvent);
                }else if (state2) {
                    keyEvent.key_type = NOT_CHARGE_KEY;
                }else if (state1) {
                }else{
                    keyEvent.key_type = NOT_CHARGE_KEY;
                    handle_key_event(keyEvent);
                }
            }

            if (io_num == GPIO_INPUT_IO_KEY_LEFT || io_num == GPIO_INPUT_IO_KEY_RIGHT ) {
                int tick_value = -1;

                //beap and vibrate
                if (val == 1) {
                    if (enable_beep_vibrate){
                        beap_vibrate();
                    }
                    tick_value = 0;
                }

                // detect key
                key_event_t keyEvent;
                keyEvent.key_value = val==1?KEY_DOWN:KEY_UP;
                if (io_num == GPIO_INPUT_IO_KEY_LEFT) {
                    keyEvent.key_type = TIMER_KEY;
                    tick[0] = tick_value;
                } else {
                    keyEvent.key_type = CLEAR_KEY;
                    tick[1] = tick_value;
                }
                handle_key_event(keyEvent);

                if (tick[0] == -1 && tick[1] == -1) {
                    //stop tick
                    tick_type = portMAX_DELAY;
                } else {
                    //start tick
                    tick_type = ( TickType_t ) 0;
                }
            }
        }
        for (int i = 0; i < 2; ++i)
        {
            if (tick[i] >= 0) {
                tick[i]++;
                if (tick[i] > 10) {
                    //50*10 ms
                    key_event_t keyEvent;
                    keyEvent.key_value = KEY_HOLD;
                    keyEvent.key_type = (i==0?TIMER_KEY:CLEAR_KEY);
                    handle_key_event(keyEvent);
                    tick[i] = 1;
                }
            }
        }
        vTaskDelay(50/portTICK_RATE_MS);
    }
}

void gpio_key_start()
{
    gpio_set_level(GPIO_OUTPUT_IO_LED0, 1);
    gpio_set_level(GPIO_OUTPUT_IO_LED1, 1);
    enable_beep_vibrate = true;
}

void gpio_key_init()
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //interrupt of both edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull_down_en mode
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_KEY_LEFT, gpio_isr_handler, (void*) GPIO_INPUT_IO_KEY_LEFT);
    gpio_isr_handler_add(GPIO_INPUT_IO_KEY_RIGHT, gpio_isr_handler, (void*) GPIO_INPUT_IO_KEY_RIGHT);
    gpio_isr_handler_add(GPIO_INPUT_IO_STATE1, gpio_isr_handler, (void*) GPIO_INPUT_IO_STATE1);
    gpio_isr_handler_add(GPIO_INPUT_IO_STATE2, gpio_isr_handler, (void*) GPIO_INPUT_IO_STATE2);

    //set output level
    gpio_set_level(GPIO_OUTPUT_IO_LED0, 0);
    gpio_set_level(GPIO_OUTPUT_IO_LED1, 0);
    gpio_set_level(GPIO_OUTPUT_IO_SPEAKER, 0);
    gpio_set_level(GPIO_OUTPUT_IO_VIBRATE, 0);
}

