#include <stdio.h>

/* KEY TYPE */
#define TIMER_KEY	1
#define CLEAR_KEY   2
#define SLEEP_KEY   3


/* KEY VALUE */
#define KEY_UP		0
#define KEY_HOLD	1
#define KEY_DOWN	2

typedef struct {
	int8_t key_type;
	int8_t key_value;
} key_event_t;