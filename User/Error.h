#ifndef __ERROR_H__
#define __ERROR_H__

#include "global.h"

#define LED_PORT					P5
#define LED_PIN						BIT4
#define LED_OUTPUT					P54

#define GET_SPECIFIED_EEROR(x)		((unErrorMaster & (1L << (x)) == 0) ? FALSE : TRUE)
#define IS_ANY_EEROR				((unErrorMaster == 0) ? FALSE : TRUE)
/*
	ERR_NULL = 0,
	ERR_CURRENT_WARNING,	// Current > 5A
	ERR_LOCATE_FAIL,
	ERR_RAMPUP_FAIL,
	ERR_BATTERY_LOW,
	ERR_INTERNAL,	// Loss lock, single phase duration too long
	ERR_CURRENT_BURNING,	// Current > 15A
	ERR_BRD_FAULT
	*/
#define MOTOR_ERROR_MSK				0xFFFFFFFEul
#define NO_MOTOR_EEROR				(((unErrorMaster & MOTOR_ERROR_MSK) == 0) ? TRUE : FALSE)

// Max 32 type of error because used uint32_t's bit to mark each error
typedef enum {
	ERR_NULL = 0,
	ERR_CURRENT_WARNING,	// Current > 5A
	ERR_COMMUNICATION_FAIL,
	ERR_LOCATE_FAIL,
	ERR_RAMPUP_FAIL,
	ERR_BATTERY_LOW,
	ERR_INTERNAL,	// Loss lock, single phase duration too long
	ERR_CURRENT_BURNING,	// Current > 15A
	ERR_BRD_FAULT
} ENUM_ERROR_LEVEL;

#ifdef __USED_BY_ERROR_C__
	#define EXTERNAL_ERROR  

	#define LED_ON					(LED_OUTPUT = 0)
	#define LED_OFF 				(LED_OUTPUT = 1)
	#define LED_TOGGLE 				(GPIO_TOGGLE(LED_OUTPUT))
	#define LED_PATTERN_INTERVAL	2000  //2s
	#define LED_BLINK_ON_TIME		200  //0.2s
	#define LED_BLINK_OFF_TIME		200  //0.2s, so 3 times is (0.4x3 = 1.2s)
	#define LED_BLINK_INTERVAL		(LED_BLINK_ON_TIME + LED_BLINK_OFF_TIME)
	// Since LED can not blink to fast (for human watchable), 3 type of blink make sense
	// So this table is introduced. Since ERR_BRD_FAULT is constantly ON, we can use it as array size
	const uint32_t unLED_BLINK_PATTERN_TABLE[ERR_BRD_FAULT] = {1 * LED_BLINK_INTERVAL,
			1 * LED_BLINK_INTERVAL,
			2 * LED_BLINK_INTERVAL,
			2 * LED_BLINK_INTERVAL,
			2 * LED_BLINK_INTERVAL,
			2 * LED_BLINK_INTERVAL,
			3 * LED_BLINK_INTERVAL};
#else 
	#define EXTERNAL_ERROR extern
#endif

EXTERNAL_ERROR uint32_t unErrorMaster;
EXTERNAL_ERROR void delay(uint32_t unDelayMs);
EXTERNAL_ERROR void resetError(ENUM_ERROR_LEVEL enumErrorType);
EXTERNAL_ERROR void setError(ENUM_ERROR_LEVEL enumErrorType);
EXTERNAL_ERROR void clearError(void);
EXTERNAL_ERROR void ErrorManager(void);

#endif
