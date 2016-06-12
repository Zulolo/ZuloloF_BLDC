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

// Timer 0 is used to change phase
void TMR0_IRQHandler(void)                                   
{
	TIMER0->TISR  = TIMER0->TISR;	//~0;    // Write 1 to clear interrupt flag

	FLAG_PHASE_CHANGED = SET;

	if (TRUE == tMotor.structMotor.MSR.bZeroCrossDetecting)
	{
		// In case TIM1 interrupt was triggered but before TIM0 interrupt (phase change)
		TIMER_DisableInt(TIMER1);
		TIMER1->TISR  = ~0;    // Clear interrupt flag
		TIMER_SET_CMP_VALUE(TIMER1, GET_TIM1_CMP_VALUE(TIMER1->TDR + AVOID_ZXD_AFTER_PHCHG));	
		// ****!! In future AVOID_ZXD_AFTER_PHCHG can be made to dynamic !!****
		// ****!! which means at lower RPM, the AVOID_ZXD_AFTER_PHCHG will be longer than at higher RPM !!****
		//GET_TIMER_DIFF(PWM_ZX_FILTER_TIME, GET_TIM1_CMP_VALUE(TIMER1->TDR + (tMotor.structMotor.ACT_PERIOD >> 1))));	
		//(tMotor.structMotor.ACT_PERIOD >> 2)));
		FLAG_TIM1_USEAGE = ENUM_TIM1_AVOID_ZXD;
		TIMER_EnableInt(TIMER1);
	}
}

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
			tMotor.structMotor.unActualPeriod = (iTempDeltaZXD + tMotor.structMotor.unActualPeriod) >> 1;
			iHalfPeriod = tMotor.structMotor.unActualPeriod >> 1;
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
//	TIMER_DisableInt(TIMER1);
	TIMER1->TISR = ~0;    // Clear interrupt flag

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
		}
	}
}

void ACMP_IRQHandler(void)
{
	ACMP->CMPSR |= ACMP_CMPSR_ACMPF0_Msk;

	// In case this was triggered by last phase
	// and in TIM0 interrupt didin't successfully disabled this interrupt
	if (ENUM_TIM1_ZXD_FILTER == FLAG_TIM1_USEAGE)
	{	 	
		// Rising or falling edge will be put into TIM1 Interrupt handler because the following situation will have problem:
		// First a matched edge happened, then an un-matched edge happened
		// So do NOT check ACMP0_EDGE_MATCH here.

		// Start count, if ACMP level can be stable for ZXD_FILTER_TIME
		// we can consider real ZX happened
		// If ACMP_IRQHandler was re-entered before ZXD_FILTER_TIME, TIM1's interrupt will be postponed
		TIMER_SET_CMP_VALUE(TIMER1, GET_TIM1_CMP_VALUE(TIMER1->TDR + ZXD_FILTER_TIME));
		TIMER_EnableInt(TIMER1);
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
			tMotor.structMotor.unCurrent = (uint16_t)(ADC_GET_CONVERSION_DATA(ADC, WHAT_EVER_DO_NOT_CARE));
			ADC_SET_INPUT_CHANNEL(ADC, ADC_BATTERY_CHN_MSK);		
		}
		else if (ADC->ADCHER & ADC_BATTERY_CHN_MSK)
		{
			tMotor.structMotor.unBattery = (uint16_t)(ADC_GET_CONVERSION_DATA(ADC, WHAT_EVER_DO_NOT_CARE));
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
	
void SPI_IRQHandler(void)
{
	static ENUM_SPI_RECEIVE_STATE tSPI_LastState = SPI_RCV_IDLE;
	static uint16_t unSPI_RX_Value;
//	static uint8_t unSelectedReg = 0;
	
	// Check if it is really finished one unit transfer
	if ((SPI->SSR & SPI_SSR_LTRIG_FLAG_Msk) == SPI_SSR_LTRIG_FLAG_Msk)
	{
		unSPI_RX_Value = SPI_READ_RX(SPI);
		
		if (tMotor.structMotor.MSR.bNewComFrameReceived == FALSE)
		{
			switch(tSPI_LastState)
			{	
				case SPI_RCV_IDLE:
				case SPI_RCV_CRC:
					if (MTR_INVALID_MOTOR_CMD == unSPI_RX_Value)
					{
//						SPI_WRITE_TX(SPI, unReadValueCRC);
						SPI_TRIGGER(SPI);
						tSPI_LastState = SPI_RCV_IDLE;
					}
					else
					{
						if (IS_COMM_RD_CMD(unSPI_RX_Value))
						{
							unCOM_SPI_ReadData[0] = unSPI_RX_Value;
//							tMotor.structMotor.MSR.bNewComFrameReceived = TRUE;
							SPI_TRIGGER(SPI);
							tSPI_LastState = SPI_RCV_RD_CMD;
						}
						else
						{
							unCOM_SPI_ReadData[0] = unSPI_RX_Value;
//							SPI_WRITE_TX(SPI, 0);
							SPI_TRIGGER(SPI);
							tSPI_LastState = SPI_RCV_WR_CMD;
						}						
					}	
				break;
			
				case SPI_RCV_RD_CMD:
					// If last time is read command, this time must be read CRC and next time must be 0xFFFF on MOSI to read
					// So the data received is the CRC of read command, now the slave doesn't care
					unCOM_SPI_ReadData[1] = unSPI_RX_Value;
//					SPI_WRITE_TX(SPI, unReadValueCRC);
//					SPI_TRIGGER(SPI);		
					tMotor.structMotor.MSR.bNewComFrameReceived = TRUE;				
					tSPI_LastState = SPI_RCV_CRC;	
				break;

				case SPI_RCV_WR_CMD:
					// No need to comment
//					SPI_WRITE_TX(SPI, 0);
					SPI_TRIGGER(SPI);
					unCOM_SPI_ReadData[1] = unSPI_RX_Value;
					tSPI_LastState = SPI_RCV_WR_DATA;	
				break;
				
				case SPI_RCV_WR_DATA:
					// No need to comment
					unCOM_SPI_ReadData[2] = unSPI_RX_Value;
					tMotor.structMotor.MSR.bNewComFrameReceived = TRUE;
					tSPI_LastState = SPI_RCV_CRC;	
				break;
			
				default:
					unCOM_SPI_TransErrCNT++;
//					SPI_WRITE_TX(SPI, 0);
					SPI_TRIGGER(SPI);
				break;
			}
		}
		else
		{
			SPI_TRIGGER(SPI);			
		}		
	}
	else
	{
//		SPI_WRITE_TX(SPI, 0);
		SPI_TRIGGER(SPI);
	}

	SPI_CLR_UNIT_TRANS_INT_FLAG(SPI);
}



//	if ((SPI->CNTRL & SPI_STATUS_IF_Msk) == SPI_STATUS_IF_Msk)
//	{
//		// Check if it is really finished one unit transfer
//		if ((SPI->SSR & SPI_SSR_LTRIG_FLAG_Msk) == SPI_SSR_LTRIG_FLAG_Msk)
//		{
//			SPI_WRITE_TX(SPI, unSPI_RX_Value);
//			SPI_TRIGGER(SPI);
//			
//			unSPI_RX_Value = SPI_READ_RX(SPI);
//			if (tMotor.structMotor.MSR.bNewComFrameReceived == FALSE)
//			{
//				switch(tSPI_LastState)
//				{
//				case SPI_RCV_IDLE:
//				case SPI_RCV_RD_CMD:
//				case SPI_RCV_WR_DATA:
//					unCOM_SPI_ReadData[0] = (uint16_t)(unSPI_RX_Value >> 16);
//					unCOM_SPI_ReadData[1] = (uint16_t)unSPI_RX_Value;
//					if (MTR_INVALID_MOTOR_CMD == unCOM_SPI_ReadData[0] )
//					{
//						// If it is dummy command, means to clear communication.
//						// What need to be read is already done in last transaction during receiving this dummy command
//						// SO next time no matter read or write command is transmitting, a fresh new frame with 0 data and 0 CRC is responsing
//						SPI_WRITE_TX(SPI, 0);
//						SPI_TRIGGER(SPI);
//						tSPI_LastState = SPI_RCV_IDLE;
//					}
//					else
//					{
//						if (IS_COMM_RD_CMD(unCOM_SPI_ReadData[0]))
//						{
//							tSPI_LastState = SPI_RCV_RD_CMD;
//							// Read command received, go to main procedure to handle
//							tMotor.structMotor.MSR.bNewComFrameReceived = TRUE;
//						}
//						else
//						{
//							SPI_WRITE_TX(SPI, 0);
//							SPI_TRIGGER(SPI);
//							tSPI_LastState = SPI_RCV_WR_CMD;
//						}
//					}
//					break;

//				case SPI_RCV_WR_CMD:
//					unCOM_SPI_ReadData[2] = (uint16_t)(unSPI_RX_Value >> 16);
//					unCOM_SPI_ReadData[3] = (uint16_t)unSPI_RX_Value;
//					// Write command and data received, go to main procedure to handle
//					tMotor.structMotor.MSR.bNewComFrameReceived = TRUE;
//					tSPI_LastState = SPI_RCV_WR_DATA;
//					break;

//				default:
//					SPI_WRITE_TX(SPI, 0);
//					SPI_TRIGGER(SPI);
//					unCOM_SPI_TransErrCNT++;
//					break;
//				}
//			}
//			else
//			{
//				SPI_WRITE_TX(SPI, 0);
//				SPI_TRIGGER(SPI);
//				unCOM_SPI_TransErrCNT++;
//			}
//		}
//		else
//		{
//			unWTF = SPI->SSR;
//			SPI_WRITE_TX(SPI, 0);
//			SPI_TRIGGER(SPI);
//			unCOM_SPI_TransErrCNT++;
//		}
//			unFIFO_RX_CNT = SPI_GET_RX_FIFO_COUNT(SPI);
//			if (unFIFO_RX_CNT == COMM_RD_CMD_CNT_IN_32BIT)
//			{
//				// Received 2 uint16, copy to RAM buffer and infor communication manager
//				unSPI_RX_Value = SPI_READ_RX(SPI);
//				unCOM_SPI_ReadData[0] = (uint16_t)(unSPI_RX_Value >> 16);
//				unCOM_SPI_ReadData[1] = (uint16_t)unSPI_RX_Value;

//				if (unCOM_SPI_ReadData[0] != MTR_INVALID_MOTOR_CMD)
//				{
//					// If it is dummy command, master just want to retrieve last time read's data.
//					// For slave, do nothing
//					if (IS_COMM_RD_CMD(unCOM_SPI_ReadData[0]))
//					{
//						tMotor.structMotor.MSR.bNewComFrameReceived = TRUE;
//					}
//					else
//					{
//						unCOM_SPI_TransErrCNT++;
//					}
//				}
//			}
//			else if (unFIFO_RX_CNT == COMM_WR_CMD_CNT_IN_32BIT)
//			{
//				// Received 4 uint16, copy to RAM buffer and infor communication manager
//				unSPI_RX_Value = SPI_READ_RX(SPI);
//				unCOM_SPI_ReadData[0] = (uint16_t)(unSPI_RX_Value >> 16);
//				unCOM_SPI_ReadData[1] = (uint16_t)unSPI_RX_Value;
//				unSPI_RX_Value = SPI_READ_RX(SPI);
//				unCOM_SPI_ReadData[2] = (uint16_t)(unSPI_RX_Value >> 16);
//				unCOM_SPI_ReadData[3] = (uint16_t)unSPI_RX_Value;
//				if (IS_COMM_WR_CMD(unCOM_SPI_ReadData[0]))
//				{
//					tMotor.structMotor.MSR.bNewComFrameReceived = TRUE;
//				}
//				else
//				{
//					unCOM_SPI_TransErrCNT++;
//				}
//			}
//			else
//			{
//				unCOM_SPI_TransErrCNT++;
//			}
//		}
//		else
//		{
//			unCOM_SPI_TransErrCNT++;
//		}
			// Clear Receive FIFO interrupt
			// I don't know how to clear. Maybe just after I read out the data in FIFO and it is below threshold it will be OK
//		else
//		{	// Receive FIFO interrupt should be the only interrupt enabled for SPI
//			// So something strange happened
//			unCOM_SPI_TransErrCNT++;
//			SPI_WRITE_TX(SPI, 0);
//			SPI_WRITE_TX(SPI, 0);
//			SPI_TRIGGER(SPI);
//		}		
//		SPI_ClearRxFIFO(SPI);
//		SPI_ClearTxFIFO(SPI);
//		SPI_CLR_UNIT_TRANS_INT_FLAG(SPI);

//	}
//	else
//	{
//		SPI_WRITE_TX(SPI, 0);
//		SPI_TRIGGER(SPI);
//		unCOM_SPI_TransErrCNT++;
//	}
	
//}

void SysTick_Handler(void)
{
	unSystemTick += 5;
}
