#ifndef _BS_TIMER_H_
#define _BS_TIMER_H_

#include <stdio.h>

typedef enum {
	TIMER_TIMEOUT,
	TIMER_STOPWATCH,
    TIMER_NUM
}bs_timer_e;

void bs_timer_init();
void bs_timer_deinit();
void bs_timer_start(bs_timer_e timer);
void bs_timer_stop(bs_timer_e timer);
void bs_timer_pause(bs_timer_e timer);
bool bs_timer_status(bs_timer_e timer);
bool bs_timer_toggle(bs_timer_e timer);     //return timer status: true for started: flase for paused
void bs_timer_reset(bs_timer_e timer);

#endif  /*_BS_TIMER_H_*/
