/******************************************************************************
 * @file     it.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 15/01/01 11:50a $ 
 * @brief    all interrupt handler for Mini51 series MCU
 *
 * @note
 * Copyright (C) 2014 Zulolo Technology Corp. All rights reserved.
*****************************************************************************/
#define __USED_BY_IT_C__
#include "it.h"

// Used to change phase
void TMR0_IRQHandler(void)                                   
{
	TIMER0->TISR  = TIMER0->TISR;	//~0;    // Clear interrupt flag

	//iPhaseChangeCNT4Period++;
	FLAG_PHASE_CHANGED = SET;
//	iTestACMPIntEachPhaseCNT = 0;

	// Disable comparator's interrupt in case after change phase it triggers
	//ACMP0_INT_DISABLE; 

	// Disable PWM's interrupt in case after change phase it triggers
	PWM_INT_DISABLE;


	if (TRUE == tMotor.structMotor.MSR.bZeroCrossDetecting)
	{
		// In case TIM1 interrupt happened but preeempted by TIM0
		TIMER_DisableInt(TIMER1);
		TIMER1->TISR  = ~0;    // Clear interrupt flag
		TIMER_SET_CMP_VALUE(TIMER1, GET_TIM1_CMP_VALUE(TIMER1->TDR + AVOID_ZXD_AFTER_PHCHG));	
		//GET_TIMER_DIFF(PWM_ZX_FILTER_TIME, GET_TIM1_CMP_VALUE(TIMER1->TDR + (tMotor.structMotor.ACT_PERIOD >> 1))));	
		//(tMotor.structMotor.ACT_PERIOD >> 2)));
		FLAG_TIM1_USEAGE = ENUM_TIM1_AVOID_ZXD;
		TIMER_EnableInt(TIMER1);
	}
}

//uint32_t getTIM0atThisPWMHigh(uint16_t iPWMHighPeriod)
//{
//	// PMW is 11MHz, TIM0 is 2MHz
//	// Get current TIM0 CNT
//	uint32_t iCurrentTIM0CNT;
//	iCurrentTIM0CNT = TIMER_GetCounter(TIMER0);
//
//}
// Only detecting if zx during PWM is high
// Using while to read ACMP output
// So a lot of CPU resource will be occupied
// Other routine will be handle during the time between phase change and this kind of detect start
// ALso during PWM low CPU can do other thing, but if PWM duty is heavy this will be short
//PWM:  _______________             _______________
//_____|       |       |___________|
//		    ZX found
uint32_t DetectdTimeWhenPWMHigh(void)
{
	

//	uint32_t iMaxTIM0atThisPWMHigh;
//	iMaxTIM0atThisPWMHigh = getTIM0atThisPWMHigh(tMotor.structMotor.ACT_DUTY);

	// ALready done when enter PWM interrupt
	//PWM->PIIR |= PWM_PIIR_PWMPIF1_Msk;

	while ((PWM->PIIR & PWM_PIIR_PWMPIF1_Msk) == 0)
	{
//		BRG_DISABLE;
//		BLDC_stopMotor();
		if (unZXMatchCNT > MAX_ZX_MATCH_IN_PWM)
		{
			return TIMER_GetCounter(TIMER1);	//GET_TIMER_DIFF(PWM_ZX_FILTER_TIME, TIMER_GetCounter(TIMER1));
		}

		if (ACMP0_EDGE_MATCH)
		{
			unZXMatchCNT++; 
		}
		else
		{
			unZXMatchCNT = 0;
		}
		
	}
	return TIMER_INVALID_CNT;
}

//uint32_t getMax(uint32_t *pArray, uint8_t iArrayLength)
//{
//	uint32_t iMax = 0;
//	while (iArrayLength)
//	{
//		if (iMax < *pArray)
//		{
//			iMax = *pArray;
//		}
//		pArray++;
//		iArrayLength--;
//	}
//	return iMax;
//}
//
//uint32_t getMin(uint32_t *pArray, uint8_t iArrayLength)
//{
//	uint32_t iMin = 0xFFFFFFFF;
//	while (iArrayLength)
//	{
//		if (iMin > *pArray)
//		{
//			iMin = *pArray;
//		}
//		pArray++;
//		iArrayLength--;
//	}
//	return iMin;
//}

int32_t PhaseZXDedHandler(uint32_t iThisZXDetectedTime)
{
	static uint32_t iTempDeltaZXD = 0;
	static uint32_t iHalfPeriod = 0;
//	static uint32_t iThisZXDetectedTime = 0;

	//iThisZXDetectedTime = TIMER_GetCounter(TIMER1);

	iTempDeltaZXD = GET_TIMER_DIFF(unLastZXDetectedTime, iThisZXDetectedTime);

	if ((iTempDeltaZXD > MIN_PHASE_TIME) && (iTempDeltaZXD < MAX_PHASE_TIME))
	{
		unLastZXDetectedTime = iThisZXDetectedTime;
		tMotor.structMotor.MSR.bThisPhaseDetectedZX = TRUE;
//		iTestDetectedZX++;
		if (TRUE == tMotor.structMotor.MSR.bLocked)
		{
			tMotor.structMotor.unACT_PERIOD = (iTempDeltaZXD + tMotor.structMotor.unACT_PERIOD) >> 1;
			iHalfPeriod = tMotor.structMotor.unACT_PERIOD >> 1;
			TIMER_SET_CMP_VALUE(TIMER0, TIMER0->TDR + (iHalfPeriod > TIME_DEBT) ? (iHalfPeriod - TIME_DEBT) : ZXD_BEFORE_PHCHG);
		}
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

// Used to set time reference and filter ZXD
void TMR1_IRQHandler(void)                                   
{
//	static uint32_t iZXTimeDuringPWMHigh;
//	static uint32_t iTempDeltaZXD = 0;
//	static uint32_t iTempPhaseChange2ZX = 0;
//	static uint32_t iThisZXDetectedTime = 0;
//	static uint32_t iTargetNewPeriod = 0;

//	TIMER_DisableInt(TIMER1);
	TIMER1->TISR = ~0;    // Clear interrupt flag
//	unZXMatchCNT = 0;

/*		iZXTimeDuringPWMHigh = DetectdTimeWhenPWMHigh();
		if (iZXTimeDuringPWMHigh == TIMER_INVALID_CNT)
		{
			// Start PWM duty interrupt, during each PWM high, check is there is ZX
			PWM->PIIR = ~0;
			PWM_INT_ENABLE;
		}
		else
		{
			if (PhaseZXDedHandler(iZXTimeDuringPWMHigh) == FALSE)
			{	// The value has some problem, still open interrupt of PWM to detect ZX when PWM high
				PWM->PIIR = ~0;
				PWM_INT_ENABLE;
			}
		}


	if (ENUM_TIM1_START_ZXD == FLAG_TIM1_USEAGE)
	{
		// First check if ACMP already match
		if (ACMP0_EDGE_MATCH)
		{
			iTestEnter_TMR1_MATCH++;
			PhaseZXDedHandler();
		}
		else
		{
			// ACMPF0 may already be set
			if (ACMP->CMPSR & ACMP_CMPSR_ACMPF0_Msk)
			{
				ACMP->CMPSR |= ACMP_CMPSR_ACMPF0_Msk;
			}
			// If not match, open ACMP interrupt to monitor ZX
			ACMP0_INT_ENABLE;
		}
	}
*/

	if (ENUM_TIM1_AVOID_ZXD == FLAG_TIM1_USEAGE)
	{
		TIMER_DisableInt(TIMER1);	// ACMP interrupt will re-open TIM1's interrupt
		FLAG_TIM1_USEAGE = ENUM_TIM1_ZXD_FILTER;
		// Clear all ACMP changed flag happened before
		ACMP->CMPSR |= ACMP_CMPSR_ACMPF0_Msk;
		ACMP0_INT_ENABLE;
	}
	else
	{	// for now can only be ENUM_TIM1_ZXD_FILTER
		if (ACMP0_EDGE_MATCH)
		{	
			// No need to find the real ZXD time, we only care the delta 
			if (TRUE == PhaseZXDedHandler(TIMER_GetCounter(TIMER1)))	//GET_TIMER_DIFF(ZXD_FILTER_TIME, TIMER_GetCounter(TIMER1))))
			{
				ACMP0_INT_DISABLE;  // This phase ACMP job done
				TIMER_DisableInt(TIMER1);
			}

//			iThisZXDetectedTime = TIMER_GetCounter(TIMER1) - ZXD_FILTER_TIME;
//			//iTempDeltaZXD = GET_TIMER_DIFF(unLastZXDetectedTime, iThisZXDetectedTime) - ZXD_FILTER_TIME - ACMP_HYS_AVG_TIME;
//			iTempDeltaZXD = GET_TIMER_DIFF(unLastZXDetectedTime, iThisZXDetectedTime);
////			iTempPhaseChange2ZX = GET_TIMER_DIFF(iPhaseChangeTime, iThisZXDetectedTime) - ZXD_FILTER_TIME;
//			//iTargetNewPeriod = (iTempDeltaZXD >> 1) + iTempPhaseChange2ZX;
//			iTargetNewPeriod = iTempDeltaZXD;	// >> 1 + iTempPhaseChange2ZX;	//(iTempDeltaZXD + tMotor.structMotor.ACT_PERIOD) >> 1;
////			iTargetNewPeriod = (iTargetNewPeriod + tMotor.structMotor.ACT_PERIOD) >> 1;
//			iTestEnter_TMR1_MATCH++;
////			RECORD_TEST_VALUE(iTestNewPeriodIndex, iTestZXDPeriodArray, iTargetNewPeriod);
////			RECORD_TEST_VALUE(iTestTIM0CNTIndex, iTestTIM0CNTArray, TIMER0->TCMPR);
////			if ((iTestNewPeriodIndex == 0) && (iTestZXDPeriodArray[0] != 0))
////			{
////				BRG_DISABLE;
////				BLDC_stopMotor();
////				iTestZXDPeriodMax = getMax(iTestZXDPeriodArray, TEST_ARRAY_LEN);
////				iTestZXDPeriodMin = getMin(iTestZXDPeriodArray, TEST_ARRAY_LEN);
////				iTestTIM0CNTMax = getMax(iTestTIM0CNTArray, TEST_ARRAY_LEN);
////				iTestTIM0CNTMin = getMin(iTestTIM0CNTArray, TEST_ARRAY_LEN);
////			}
////			RECORD_TEST_DELTA_ZXD(iTestNewPeriodIndex, iThisZXDetectedTime);
//			// Only when iTempDeltaZXD larger than minimum phase time and
//			// smaller then max phase time we consider it is valid
//			if ((iTargetNewPeriod > MIN_PHASE_TIME) && (iTargetNewPeriod < MAX_PHASE_TIME))
//			{
////				ACMP0_INT_DISABLE;	// This phase job done
//				TIMER_DisableInt(TIMER1);	// TIM1 interrupt will be re-open in TIM0 interrupt (phase change) to implement AVOID_ZXD_AFTER_PHCHG
//				iTestEnter_ACMP_MATCH++;
//				// We can consider ZX detected, and USE IT!!!!
//				// To get stable and solide ZXD, more filter will be added in the TIM0 (phase change) interrupt
//				// Only after continuously some number of ThisPhaseDetectedZX or miss ThisPhaseDetectedZX, it will enter or loss lock
//				tMotor.structMotor.MSR.ThisPhaseDetectedZX = TRUE;
//				unLastZXDetectedTime = iThisZXDetectedTime;
//
//				// Incase the counter of TIM0 is already in front of iTempDeltaZXD
//				if ((TRUE == tMotor.structMotor.MSR.Locked))	// && (iTargetNewPeriod > (TIMER_GetCounter(TIMER0) + ZXD_BEFORE_PHCHG)))
//				{	// Still have time to change CMP in TIM0
//					// You can have a rest, now the only thing left is T0 trigger phase change
//
////					BRG_DISABLE;
//					iTestDetectedZX++;
//					//iTempDeltaZXD = (tMotor.structMotor.ACT_PERIOD * 3 + iTempDeltaZXD) >> 2;
//					if (iTargetNewPeriod > (TIMER_GetCounter(TIMER0) + ZXD_BEFORE_PHCHG))
//					{
//						tMotor.structMotor.ACT_PERIOD = iTargetNewPeriod;   //GET_TIMER_DIFF(unLastZXDetectedTime, TIMER_GetCounter(TIMER1));
//					}
//					else
//					{
//						tMotor.structMotor.ACT_PERIOD = TIMER_GetCounter(TIMER0) + (ZXD_BEFORE_PHCHG << 1);
//					}
//					TIMER_SET_CMP_VALUE(TIMER0, tMotor.structMotor.ACT_PERIOD);	//TIMER0->TDR + (iTargetNewPeriod >> 2));	//(iTempPhaseChange2ZX >> 1));
////					if (TIMER0->TISR & 0x01)
////					{	// If TIM0 interrupt was already triggered
////						// Clear interrupt flag
////						TIMER0->TISR |= 0x01;
////					}
//					//TIMER_SET_CMP_VALUE(TIMER0, tMotor.structMotor.ACT_PERIOD);
//				}
//			}
		}
	}
}

//void PWM_IRQHandler(void)
//{
//	static uint32_t iZXTimeDuringPWMHigh;
//
//	// Clear Flag
//	PWM->PIIR = PWM->PIIR;	//~0;
//	if (IS_PWM_IRQ_ENABLED)
//	{
//		iZXTimeDuringPWMHigh = DetectdTimeWhenPWMHigh();
//		if (iZXTimeDuringPWMHigh != TIMER_INVALID_CNT)
//		{
//			if (PhaseZXDedHandler(iZXTimeDuringPWMHigh) == TRUE)
//			{
//				// Found it
//				PWM_INT_DISABLE;
//			}
//		}
//	}
//}

void ACMP_IRQHandler(void)
{
	ACMP->CMPSR |= ACMP_CMPSR_ACMPF0_Msk;
//	iTestACMPIntEachPhaseCNT++;
//	// maybe before enter TIM0 interrupt this ACMP int was triggered
//	if (ACMP->CMPCR[0] & ACMP_CMPCR_ACMPIE_Msk)
//	{
//		if (ACMP0_EDGE_MATCH)
//		{
////			RECORD_TEST_VALUE(iTestNewPeriodIndex, iTestZXDPeriodArray, TIMER_GetCounter(TIMER1) - TIMER1->TCMPR);
////			if (TIMER_GetCounter(TIMER1) < TIMER1->TCMPR)//((iTestNewPeriodIndex == 0) && (iTestZXDPeriodArray[0] != 0))
////			{
////				BRG_DISABLE;
////				BLDC_stopMotor();
////				iTestZXDPeriodMax = getMax(iTestZXDPeriodArray, TEST_ARRAY_LEN);
////				iTestZXDPeriodMin = getMin(iTestZXDPeriodArray, TEST_ARRAY_LEN);
////			}
////			iTestEnter_ACMP_MATCH++;
//			ACMP0_INT_DISABLE;
//			PhaseZXDedHandler(TIMER_GetCounter(TIMER1));
//		}
//	}
//}

	// In case this was triggered by last phase
	// and in TIM0 interrupt didin't successfully disabled this interrupt
	if (ENUM_TIM1_ZXD_FILTER == FLAG_TIM1_USEAGE)
	{	 	
		// Rising or falling edge will be put into TIM1 Interrupt handler because the following situdation will have problem:
		// First a matched edge happened, then an un-matched edge happened
		// So NOT check ACMP0_EDGE_MATCH here
		//	if (ACMP0_EDGE_MATCH)
		//	{
		// Start count, if ACMP level can be stable for ZXD_FILTER_TIME
		// we can consider real ZX
		// If ACMP_IRQHandler was re-entered before ZXD_FILTER_TIME, postpond 
		TIMER_SET_CMP_VALUE(TIMER1, GET_TIM1_CMP_VALUE(TIMER1->TDR + ZXD_FILTER_TIME));
		TIMER_EnableInt(TIMER1);
//		if (ACMP0_EDGE_MATCH)
//		{
//			ACMP0_INT_DISABLE;	// This phase job done
//			iThisZXDetectedTime = TIMER_GetCounter(TIMER1);	// - ZXD_FILTER_TIME;
//			//iTempDeltaZXD = GET_TIMER_DIFF(unLastZXDetectedTime, iThisZXDetectedTime) - ZXD_FILTER_TIME - ACMP_HYS_AVG_TIME;
//			iTempDeltaZXD = GET_TIMER_DIFF(unLastZXDetectedTime, iThisZXDetectedTime);
////			iTempPhaseChange2ZX = GET_TIMER_DIFF(iPhaseChangeTime, iThisZXDetectedTime) - ZXD_FILTER_TIME;
//			//iTargetNewPeriod = (iTempDeltaZXD >> 1) + iTempPhaseChange2ZX;
//			iTargetNewPeriod = (iTempDeltaZXD + tMotor.structMotor.ACT_PERIOD) >> 1;
////			iTargetNewPeriod = (iTargetNewPeriod + tMotor.structMotor.ACT_PERIOD) >> 1;
//			iTestEnter_TMR1_MATCH++;
////			RECORD_TEST_VALUE(iTestNewPeriodIndex, iTestZXDPeriodArray, iTargetNewPeriod);
////			RECORD_TEST_VALUE(iTestTIM0CNTIndex, iTestTIM0CNTArray, TIMER0->TCMPR);
////			if ((iTestNewPeriodIndex == 0) && (iTestZXDPeriodArray[0] != 0))
////			{
////				BRG_DISABLE;
////				BLDC_stopMotor();
////				iTestZXDPeriodMax = getMax(iTestZXDPeriodArray, TEST_ARRAY_LEN);
////				iTestZXDPeriodMin = getMin(iTestZXDPeriodArray, TEST_ARRAY_LEN);
////				iTestTIM0CNTMax = getMax(iTestTIM0CNTArray, TEST_ARRAY_LEN);
////				iTestTIM0CNTMin = getMin(iTestTIM0CNTArray, TEST_ARRAY_LEN);
////			}
////			RECORD_TEST_DELTA_ZXD(iTestNewPeriodIndex, iThisZXDetectedTime);
//			// Only when iTempDeltaZXD larger than minimum phase time and
//			// smaller then max phase time we consider it is valid
//			if ((iTargetNewPeriod > MIN_PHASE_TIME) && (iTargetNewPeriod < MAX_PHASE_TIME))
//			{
////				ACMP0_INT_DISABLE;	// This phase job done
//				TIMER_DisableInt(TIMER1);	// TIM1 interrupt will be re-open in TIM0 interrupt (phase change) to implement AVOID_ZXD_AFTER_PHCHG
//				iTestEnter_ACMP_MATCH++;
//				// We can consider ZX detected, and USE IT!!!!
//				// To get stable and solide ZXD, more filter will be added in the TIM0 (phase change) interrupt
//				// Only after continuously some number of ThisPhaseDetectedZX or miss ThisPhaseDetectedZX, it will enter or loss lock
//				tMotor.structMotor.MSR.ThisPhaseDetectedZX = TRUE;
//				unLastZXDetectedTime = iThisZXDetectedTime;
//
//				// Incase the counter of TIM0 is already in front of iTempDeltaZXD
//				if ((TRUE == tMotor.structMotor.MSR.Locked))	// && (iTargetNewPeriod > (TIMER_GetCounter(TIMER0) + ZXD_BEFORE_PHCHG)))
//				{	// Still have time to change CMP in TIM0
//					// You can have a rest, now the only thing left is T0 trigger phase change
//
////					BRG_DISABLE;
//					iTestDetectedZX++;
//					//iTempDeltaZXD = (tMotor.structMotor.ACT_PERIOD * 3 + iTempDeltaZXD) >> 2;
//
//					tMotor.structMotor.ACT_PERIOD = iTargetNewPeriod;   //GET_TIMER_DIFF(unLastZXDetectedTime, TIMER_GetCounter(TIMER1));
//					TIMER_SET_CMP_VALUE(TIMER0, TIMER0->TDR + (iTargetNewPeriod >> 2));	//(iTempPhaseChange2ZX >> 1));
////					if (TIMER0->TISR & 0x01)
////					{	// If TIM0 interrupt was already triggered
////						// Clear interrupt flag
////						TIMER0->TISR |= 0x01;
////					}
//					//TIMER_SET_CMP_VALUE(TIMER0, tMotor.structMotor.ACT_PERIOD);
//				}
//			}
//		}
//	}
	}
}

void ADC_IRQHandler(void)
{
    static uint32_t iADC_ComparatorFlag;
	static uint32_t iSystemTickTemp;
	static uint16_t iBatteryLowCNT = 0;
    static uint32_t iBatteryLowLastTimeRCD = 0;

    // Get ADC comparator interrupt flag
    iADC_ComparatorFlag = ADC_GET_INT_FLAG(ADC, ADC_CURRENT_CMP_MSK | ADC_BATTERY_CMP_MSK | ADC_ADF_MSK);
    if(iADC_ComparatorFlag & ADC_ADF_MSK)
	{
		// Change ADC channel
		if (ADC->ADCHER & ADC_CURRENT_CHN_MSK)
		{
			tMotor.structMotor.unCURRENT = (uint16_t)(ADC_GET_CONVERSION_DATA(ADC, WHAT_EVER_DO_NOT_CARE));
			ADC_SET_INPUT_CHANNEL(ADC, ADC_BATTERY_CHN_MSK);		
		}
		else if (ADC->ADCHER & ADC_BATTERY_CHN_MSK)
		{
			tMotor.structMotor.unBATTERY = (uint16_t)(ADC_GET_CONVERSION_DATA(ADC, WHAT_EVER_DO_NOT_CARE));
			ADC_SET_INPUT_CHANNEL(ADC, ADC_CURRENT_CHN_MSK);
		}
		ADC_START_CONV(ADC);
	}    
    if(iADC_ComparatorFlag & ADC_CURRENT_CMP_MSK)
	{
		// current too big
		MOTOR_SHUT_DOWN;
		setError(ERR_CURRENT_BURNING);
	}
    if(iADC_ComparatorFlag & ADC_BATTERY_CMP_MSK)
	{
    	// Longer filter for battery voltage
		// if this time's battery low is near to the last one
		// 0.64ms interval of each measurement
		iSystemTickTemp = unSystemTick;
		if ((uint32_t)(iSystemTickTemp - iBatteryLowLastTimeRCD) < BATTERY_LOW_MIN_INTERVAL)
		{
			if (iBatteryLowCNT < MAX_BATT_LOW_CNT)
			{
				iBatteryLowCNT++;
			}
			else
			{
				// battery really too low
				MOTOR_SHUT_DOWN;
				setError(ERR_BATTERY_LOW);
			}
		}
		else
		{
			iBatteryLowCNT = 0;
		}
		iBatteryLowLastTimeRCD = iSystemTickTemp; 
	}
    
    ADC_CLR_INT_FLAG(ADC, iADC_ComparatorFlag);
}

// Update PWM duty after each phase change in locked state
// In other state update on demand
//void PWM_IRQHandler(void)
//{
//  PWM->PIER &= ~1;
//  PWM->CMR[2] = iPWMDutyCurrent;       // Update PWM duty
//  PWM->CMR[1] = iPWMDutyCurrent;
//  PWM->CMR[0] = iPWMDutyCurrent;
//  PWM->PIIR = 1;
//}

void SPI_IRQHandler(void)
{
	static uint8_t unValueIndex;
	if ((SPI->STATUS & SPI_STATUS_RX_INTSTS_Msk) != 0)
	{
		if (SPI_GET_RX_FIFO_COUNT(SPI) != COMM_LENGTH)
		{
			unCOM_SPI_TransErrCNT++;
		}
		else
		{
			// Received 4 uint16, copy to RAM buffer and infor communication manager
			for (unValueIndex = 0; unValueIndex < COMM_LENGTH; unValueIndex++)
			{
				unCOM_SPI_ReadData[unValueIndex] = SPI_READ_RX(SPI);
			}
			tMotor.structMotor.MSR.bNewComFrameReceived = TRUE;
		}

		// Clear Receive FIFO interrupt
		// I don't know how to clear. Maybe just after I read out the data in FIFO and it is below threshold it will be OK
		SPI_ClearRxFIFO(SPI);
		SPI_ClearTxFIFO(SPI);
	}
	else
	{	// Receive FIFO interrupt should be the only interrupt enabled for SPI
		// So something strange happened
		unCOM_SPI_TransErrCNT++;
	}
}

void SysTick_Handler(void)
{
	unSystemTick += 5;
}
