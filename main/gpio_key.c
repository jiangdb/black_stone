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

/**
 * Externs:
 *
 */
extern void handle_key_event(key_event_t keyEvent);

/**
 * Defines:
 *
 */
#define GPIO_OUTPUT_IO_SPEAKER       21
#define GPIO_OUTPUT_IO_VIBRATE       22
#define GPIO_OUTPUT_IO_LED0          25
#define GPIO_OUTPUT_IO_LED1          26
#define GPIO_OUTPUT_PIN_SEL  ((1<<GPIO_OUTPUT_IO_LED0) | (1<<GPIO_OUTPUT_IO_LED1) | (1<<GPIO_OUTPUT_IO_SPEAKER) | (1<<GPIO_OUTPUT_IO_VIBRATE))
#define GPIO_INPUT_IO_KEY_LEFT       32
#define GPIO_INPUT_IO_KEY_RIGHT      33
#define GPIO_INPUT_PIN_SEL  (uint64_t)(((uint64_t)1<<GPIO_INPUT_IO_KEY_LEFT) | ((uint64_t)1<<GPIO_INPUT_IO_KEY_RIGHT))
#define ESP_INTR_FLAG_DEFAULT        0

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    while(1) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            int val = gpio_get_level(io_num);
            key_event_t keyEvent;
            keyEvent.key_value = val==1?KEY_DOWN:KEY_UP;
            // printf("GPIO[%d] intr, val: %d\n", io_num, val);
            if (io_num == GPIO_INPUT_IO_KEY_LEFT || io_num == GPIO_INPUT_IO_KEY_RIGHT ) {
                if (val == 1) {
                    gpio_set_level(GPIO_OUTPUT_IO_SPEAKER, 1);
                    gpio_set_level(GPIO_OUTPUT_IO_VIBRATE, 1);
                    vTaskDelay(100/portTICK_RATE_MS);
                    gpio_set_level(GPIO_OUTPUT_IO_SPEAKER, 0);
                    gpio_set_level(GPIO_OUTPUT_IO_VIBRATE, 0);
                }
                if (io_num == GPIO_INPUT_IO_KEY_LEFT) {
                    keyEvent.key_type = TIMER_KEY;
                }else{
                    keyEvent.key_type = CLEAR_KEY;
                }
                handle_key_event(keyEvent);
            }
        }
    }
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

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
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
    // gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_KEY_LEFT, gpio_isr_handler, (void*) GPIO_INPUT_IO_KEY_LEFT);
    gpio_isr_handler_add(GPIO_INPUT_IO_KEY_RIGHT, gpio_isr_handler, (void*) GPIO_INPUT_IO_KEY_RIGHT);

    //set output level
    gpio_set_level(GPIO_OUTPUT_IO_LED0, 1);
    gpio_set_level(GPIO_OUTPUT_IO_LED1, 1);
    gpio_set_level(GPIO_OUTPUT_IO_SPEAKER, 0);
    gpio_set_level(GPIO_OUTPUT_IO_VIBRATE, 0);
}

