#ifndef _BS_KEY_H_
#define _BS_KEY_H_

#include <stdio.h>

/**
 * Defines:
 *
 */
#define GPIO_OUTPUT_IO_SPEAKER       21
#define GPIO_OUTPUT_IO_VIBRATE       22
#define GPIO_OUTPUT_IO_LED0          25
#define GPIO_OUTPUT_IO_LED1          26
#define GPIO_OUTPUT_PIN_SEL  		((1<<GPIO_OUTPUT_IO_LED0) | (1<<GPIO_OUTPUT_IO_LED1) | (1<<GPIO_OUTPUT_IO_SPEAKER) | (1<<GPIO_OUTPUT_IO_VIBRATE))
#define GPIO_INPUT_IO_KEY_LEFT       32
#define GPIO_INPUT_IO_KEY_RIGHT      33
#define GPIO_INPUT_IO_STATE1         35
#define GPIO_INPUT_IO_STATE2         27
#define GPIO_INPUT_PIN_SEL  		(uint64_t)(((uint64_t)1<<GPIO_INPUT_IO_KEY_LEFT) | ((uint64_t)1<<GPIO_INPUT_IO_KEY_RIGHT) | ((uint64_t)1<<GPIO_INPUT_IO_STATE1) | ((uint64_t)1<<GPIO_INPUT_IO_STATE2))
#define ESP_INTR_FLAG_DEFAULT        0

void gpio_key_init();
void gpio_key_start();
void beap(int wait, int duration);
bool is_charging();

#endif  /*_BS_KEY_H_*/