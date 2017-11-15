#ifndef _BS_KEY_EVENT_H_
#define _BS_KEY_EVENT_H_

#include <stdio.h>

/* KEY TYPE */
enum {
	TIMER_KEY,
	CLEAR_KEY,
	SLEEP_KEY,
	CHARGE_KEY,
	NOT_CHARGE_KEY,
	FIRMWARE_UPGRADE_KEY,
    KEY_TYPE_MAX
};

/* KEY VALUE */
enum {
    KEY_UP,
    KEY_HOLD,
    KEY_DOWN,
    KEY_VALUE_MAX
};

typedef struct {
	int8_t key_type;
	int8_t key_value;
} key_event_t;

void send_key_event(key_event_t keyEvent, bool fromIsr);

#endif  /*_BS_KEY_EVENT_H_*/
