/******************************************************************************
 * @file     Error.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/12/15 21:13p $ 
 * @brief    BLDC Sensor Less controller for Mini51 series MCU
 *
 * @note
 * Copyright (C) 2014 Zulolo Technology Corp. All rights reserved.
*****************************************************************************/
#define __USED_BY_ERROR_C__
#include "Error.h"

void delay(uint32_t unDelayMs)
{
	uint32_t unEntryTime = unSystemTick;
	while (((uint32_t)(unSystemTick - unEntryTime)) < unDelayMs)
	{
		__NOP();
	}
}

void clearError(void)
{
	unErrorMaster = 0;
}

void resetError(ENUM_ERROR_LEVEL enumErrorType)
{
	if (ERR_NULL != enumErrorType)
	{	
		unErrorMaster &= ~(1UL << (enumErrorType - 1));
	}
}

void setError(ENUM_ERROR_LEVEL enumErrorType)
{
	if (ERR_NULL == enumErrorType)
	{
		unErrorMaster = 0;
	}
	else
	{
		unErrorMaster |= 1UL << (enumErrorType - 1); 
	}
}

ENUM_ERROR_LEVEL getPrecedenceError(void)
{
	register ENUM_ERROR_LEVEL iVernier = ERR_BRD_FAULT;
	while (iVernier)
	{
		iVernier--; 
		if (unErrorMaster >> iVernier)
		{
			return (iVernier + 1);
		}		
	}
	return iVernier;
}

void LEDBlinkHandler(ENUM_ERROR_LEVEL tErrorType, uint32_t unErrorStartTime)
{
	uint16_t iLEDTime;
	if (ERR_BRD_FAULT == tErrorType)
	{
		// Always ON
		LED_ON;
	}
	else if (ERR_NULL == tErrorType)
	{
		// Always off
		LED_OFF;
	}
	else
	{
		iLEDTime = (uint16_t)(((uint32_t)(unSystemTick - unErrorStartTime)) % LED_PATTERN_INTERVAL);
		if (iLEDTime >= unLED_BLINK_PATTERN_TABLE[tErrorType])
		{
			LED_OFF;
		}
		else
		{
			iLEDTime %= LED_BLINK_INTERVAL;
			if (iLEDTime < LED_BLINK_ON_TIME)
			{
				LED_ON;
			}
			else
			{
				LED_OFF;
			}
		}
	}
}

// Take charge of all error reporting via LED/UART
// Shut down all MOSFET was handled as soon as the critical error was found
// to make sure then responding time will not burn the board
void ErrorManager(void)
{
	static uint32_t unLastErrorChangeTime;
	static ENUM_ERROR_LEVEL tLastErrorType = ERR_NULL;
	ENUM_ERROR_LEVEL tErrorFetched;
	tErrorFetched = getPrecedenceError();

	if (tLastErrorType != tErrorFetched)
	{
		tLastErrorType = tErrorFetched;
		// error type changed (maybe changed to no error)
		unLastErrorChangeTime = unSystemTick;
		// No matter there is error or not, process!!
		LEDBlinkHandler(tErrorFetched, unLastErrorChangeTime);
	}
	else
	{
		// Still same error (maybe no error)
		// Only process when there is some error
		if (ERR_NULL != tErrorFetched)
		{
			LEDBlinkHandler(tErrorFetched, unLastErrorChangeTime);
		}	
	}
	
}

