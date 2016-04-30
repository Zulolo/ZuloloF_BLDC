#ifndef __IT_H__
#define __IT_H__

#include "global.h"

#ifdef __USED_BY_IT_C__
	#define EXTERNAL_IT

	#define BATTERY_LOW_MIN_INTERVAL		10	// if two time's measurement of battery low is less than 10 ms 
												// we think it is real low
	#define MAX_BATT_LOW_CNT				50	// 9.6ms*50=480ms	//1000	// 0.5ms*1000=500ms
	#define MAX_ZX_MATCH_IN_PWM				36
	#define PWM_ZX_FILTER_TIME				40

#else 
	#define EXTERNAL_IT extern
#endif

#endif
