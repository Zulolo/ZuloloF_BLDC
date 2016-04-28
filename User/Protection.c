/******************************************************************************
 * @file     Protection.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 16/04/20 21:13p $
 * @brief    Protection for Mini51 series MCU
 *
 * @note
 * Copyright (C) 2016 Zulolo Technology Corp. All rights reserved.
*****************************************************************************/

#define __USED_BY_PTC_C__
#include "Protection.h"

#define MOTOR_TEST_MOSFET_ON_DURATION	50	// 50ms

void PTC_checkMotor(void)
{
	uint8_t unCheckIndex;
	clearError();
	// Battery check
	// Battery voltage check will be done in ADC interrupt on the fly, so no need to check here.

	// LED check
	// Check what?

	// MOSFET check
	// Open each MOSFET one by one to see if there is any current.
	// If yes means some MOSFET is short
	// Set ADC Current threshold to very low
	// Set ADC Voltage threshold to very high
    // Configure and enable Comperator 0 to monitor channel 0(current) input greater or euqal to 93
    ADC_ENABLE_CMP0(ADC, ADC_CURRENT_CHN_IDX, ADC_CMP_GREATER_OR_EQUAL_TO, ADC_CURRENT_HIGH_THRS_MT, ADC_CURRENT_HIGH_CNT);
    // Configure and enable Comperator 1 to monitor channel 7(battery) input less than 0x200
    ADC_ENABLE_CMP1(ADC, ADC_BATTERY_CHN_IDX, ADC_CMP_LESS_THAN, ADC_BAT_LOW_THRS_MT, ADC_BAT_LOW_CNT);
	for (unCheckIndex = 0; unCheckIndex < (sizeof(unMosfetTestTable) / sizeof(uint32_t*)); unCheckIndex++)
	{
		SET_MOSFET_ON_MANUAL(unMosfetTestTable[unCheckIndex]);
		delay(MOTOR_TEST_MOSFET_ON_DURATION);
		SET_MOSFET_OFF_MANUAL(unMosfetTestTable[unCheckIndex]);
		if (IS_ANY_EEROR == TRUE)
		{
			BLDC_stopMotor();
			while (1)
			{
				ERR_Manager();
			}
		}
	}
    // Configure and enable Comperator 0 to monitor channel 0(current) input greater or euqal to 93
    ADC_ENABLE_CMP0(ADC, ADC_CURRENT_CHN_IDX, ADC_CMP_GREATER_OR_EQUAL_TO, ADC_CURRENT_HIGH_THRS, ADC_CURRENT_HIGH_CNT);
    // Configure and enable Comperator 1 to monitor channel 7(battery) input less than 0x200
    ADC_ENABLE_CMP1(ADC, ADC_BATTERY_CHN_IDX, ADC_CMP_LESS_THAN, ADC_BAT_LOW_THRS, ADC_BAT_LOW_CNT);
}
