#ifndef _BS_TIMER_H_
#define _BS_TIMER_H_

#include <stdio.h>

enum {
	TIMER_COUNTDOWN,
	TIMER_OPERATION
};

void bs_timer_init();
void bs_timer_start();
void bs_timer_pause();
void bs_timer_stop();
void bs_timer_toggle();

#endif  /*_BS_TIMER_H_*/
