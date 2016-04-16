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
	
//	__IO uint32_t iPhaseChangeTime;
	__IO uint32_t iZXMatchCNT = 0;
//	__IO uint32_t iTestZXContinueCNT = 0;
//	__IO uint32_t iTestEnter_TMR1_MATCH = 0;
//	__IO uint32_t iTestEnter_ACMP_MATCH = 0;
//	__IO uint32_t iTestDetectedZX = 0;
//	__IO uint32_t iTestACMPIntEachPhaseCNT = 0;
//	__IO uint32_t iTestZXDPeriod = 0;
//	#define TEST_ARRAY_LEN		200
//	__IO uint32_t iTestZXDPeriodArray[TEST_ARRAY_LEN];
//	__IO uint8_t iTestNewPeriodIndex = 0;
//	__IO uint32_t iTestTIM0CNTArray[TEST_ARRAY_LEN];
//	__IO uint8_t iTestTIM0CNTIndex = 0;
//	__IO uint32_t iTestZXDPeriodMax;
//	__IO uint32_t iTestZXDPeriodMin;
//	__IO uint32_t iTestTIM0CNTMax;
//	__IO uint32_t iTestTIM0CNTMin;
//	#define RECORD_TEST_VALUE(INDEX, ARRAY, VALUE)	(ARRAY[(INDEX)] = (VALUE)); \
//													(INDEX_INCREASE((INDEX), TEST_ARRAY_LEN))
#else 
	#define EXTERNAL_IT extern
#endif

#endif
