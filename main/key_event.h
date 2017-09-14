#ifndef _BS_KEY_EVENT_H_
#define _BS_KEY_EVENT_H_

#include <stdio.h>

/* KEY TYPE */
enum {
	TIMER_KEY,
	CLEAR_KEY,
	SLEEP_KEY,
	CHARGE_KEY,
	NOT_CHARGE_KEY
};

// #define TIMER_KEY		 1
// #define CLEAR_KEY   	 2
// #define SLEEP_KEY   	 3
// #define CHARGE_KEY   	 4
// #define NOT_CHARGE_KEY   5

/* KEY VALUE */
#define KEY_UP		0
#define KEY_HOLD	1
#define KEY_DOWN	2

typedef struct {
	int8_t key_type;
	int8_t key_value;
} key_event_t;

#endif  /*_BS_KEY_EVENT_H_*/