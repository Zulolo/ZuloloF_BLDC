/******************************************************************************
 * @file     BLDCSensorLess.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/12/10 21:13p $ 
 * @brief    BLDC Sensor Less controller for Mini51 series MCU
 *
 * @note
 * Copyright (C) 2014 Zulolo Technology Corp. All rights reserved.
*****************************************************************************/
#define __USED_BY_BLDC_SENSOR_LESS_C__
#include "BLDCSensorLess.h"

/* After change phase and wait a filter time,
   the zero cross detect window will be opened.
   1. In this window the comparator's interrupt will be enabled.
   2. The first ACMP INT will start TIM1 and enable its interrupt.
   3. TIM1 INT will be set as current time + filter time (100us).
   4. Each new ACMP INT will reset TIM1 INT as current time + filter time (100us).
   5. So when TIM1 INT really happened, it means ACMP was stable and really zero crossed.                                                            
   6. In the TIM1 INT we set TIM0's phase change time.*/

/* Some limitation:
   1. Because the phase duration variable is uint16_t, max one phase is 32ms,
	  so min rotation speed is about 180RMP (12 e-phase).*/

__INLINE PhaseChangedRoutine(void)
{
	FLAG_PHASE_CHANGED = RESET;
	mMotor.structMotor.PHASE_CHANGE_CNT++;
	
	if (TRUE == mMotor.structMotor.MSR.ZeroCrossDetecting)
	{
//		iPhaseChangeTime = TIMER_GetCounter(TIMER1);
		// Miss ZXD or ZXD success filter
		// If continuously detected more than MIN_SUCC_ZXD_THRESHOLD ZX, OK! GOOD!!
		if (TRUE == mMotor.structMotor.MSR.ThisPhaseDetectedZX)
		{
			mMotor.structMotor.MSR.MissedZXD_CNT = 0;

			if (mMotor.structMotor.MSR.SuccessZXD_CNT > MIN_SUCC_ZXD_THRESHOLD)
			{
				mMotor.structMotor.MSR.Locked = TRUE;
//				BRG_DISABLE;
//				P50 = 1;
//				stopMotor();

//				iTestZXContinueCNT++;
//				iTestZXDPeriod = mMotor.structMotor.ACT_PERIOD;
			}
			else
			{
				mMotor.structMotor.MSR.SuccessZXD_CNT++;
			}
		}
		else	// If continuously missing detected more than MAX_MISS_ZXD_THRESHOLD ZX, loss lock
		{
			mMotor.structMotor.MSR.SuccessZXD_CNT = 0;
			// If ZX was not detected in last phase, iLastZXDetectedTime was also not updated
			// Guess one value
			iLastZXDetectedTime = GET_TIMER_DIFF((mMotor.structMotor.ACT_PERIOD >> 2), TIMER_GetCounter(TIMER1));
			if (mMotor.structMotor.MSR.MissedZXD_CNT > MAX_MISS_ZXD_THRESHOLD)
			{
				if (TRUE == mMotor.structMotor.MSR.Locked)
				{	
					mMotor.structMotor.MSR.Locked = FALSE;
					MOTOR_SHUT_DOWN;
					setError(ERR_INTERNAL);
				}
			}
			else
			{
				mMotor.structMotor.MSR.MissedZXD_CNT++;
			}
		}

	}

	if (TRUE == mMotor.structMotor.MSR.Locked)
	{
		// Set a rough next phase change time as the same with last phase
		// After detected ZX in TIM1 interrupt, next phase change time will be re-configured
		TIMER_SET_CMP_VALUE(TIMER0, mMotor.structMotor.ACT_PERIOD << 1);
	}

	mMotor.structMotor.MSR.ThisPhaseDetectedZX = FALSE;
	// For debug
	GPIO_TOGGLE(P50);
}

void checkMotor(void)
{
	clearError();
	// Battery check
	// Battery voltage check will be done in ADC interrupt on the fly, so no need to check here.

	// LED check
	// Check what?

	// MOSFET check
	// Open each MOSFET one by one to see if there is any current.
	// If yes means some MOSFET is short
	if (IS_ANY_EEROR == TRUE)
	{
		while (1)
		{
			ErrorManager();
		}
	}
}

/* return STATUS_WORKING: still detecting
   0: no rotation detected
   1-65534: phase time */
uint16_t canMotorContinueRunning(void)
{
	uint16_t iPhaseDuration = 0;
	static uint32_t iStateEnterTime;
// Later implement this when motor can rotate
// Then stop it while rotating to measure the waveform
// Manually rotate it is too slow 
	return 0;

	if ((uint32_t)(iSystemTick - iRotateDetectStartTime) > MAX_ALREADY_ROTATING_DETECT_TIME)
	{
		return 0;
	}
	switch (enumRotateDetectState)
	{
	case DETECT_START: 
		iStateEnterTime = iSystemTick;
		enumRotateDetectState = DETECT_PHASE_1_P;
		break;

	case DETECT_PHASE_1_P:
		if ((uint32_t)(iSystemTick - iStateEnterTime) > MAX_ROTATING_DETECT_PHASE_TIME)
		{
			return (uint16_t)0;
		}
		else
		{

		}
		break; 

	case DETECT_PHASE_1_A:

		break;

	case DETECT_PHASE_2_P:

		break;

	case DETECT_PHASE_2_A:

		break;

	case DETECT_PHASE_3_P:

		break;

	case DETECT_PHASE_3_A:

		break;

	default:
		break;
	}

	return iPhaseDuration;
}

// Mainly PWM duty increase/decrease
void BLDCSpeedManager(void)
{
	if (SET == FLAG_PHASE_CHANGED)
	{
		PhaseChangedRoutine();

		if (mMotor.structMotor.ACT_DUTY != mMotor.structMotor.TGT_DUTY)
		{
//				mMotor.structMotor.ACT_DUTY = mMotor.structMotor.TGT_DUTY;
				// Change PWM duty after each x phase change
			if (iPhaseChangeCNT4Duty > CHANGE_DUTY_CNT_THR)
			{
				iPhaseChangeCNT4Duty = 0;
				if (mMotor.structMotor.ACT_DUTY < mMotor.structMotor.TGT_DUTY)
				{
					mMotor.structMotor.ACT_DUTY++;
				}
				else
				{
					mMotor.structMotor.ACT_DUTY--;
				}
				MOTOR_SET_DUTY(mMotor.structMotor.ACT_DUTY);
			}
			iPhaseChangeCNT4Duty++;
		}
		
		PHASE_INCREASE(iCurrentPhase);
		// Modify PWM->PHCHGNXT at last because I don't know how long needed to reload PHCH with PHCHNEXT after TIM0 time-out
		PWM->PHCHGNXT = GET_PHASE_VALUE(iCurrentPhase);
	}
}
//
//void BLDCStarterManager(void)
//{
//
//}
//
//void BLDCPhaseChangeManager(void)
//{
//
//}
__INLINE void stopMotor(void)
{
	MOTOR_SHUT_DOWN;
	mMotor.structMotor.MCR.MotorNeedToRun = FALSE;
	mMotor.structMotor.MSR.MotorPowerOn = FALSE;
	enumMotorState = MOTOR_IDLE;
}

__INLINE void setPhaseManually(uint16_t iPWMDuty, uint8_t iPhase)
{
    MOTOR_SET_DUTY(iPWMDuty);
	PWM->PHCHG = GET_PHASE_VALUE(iPhase);
}

ENUM_STATUS BLDCLocatingManager(void)
{
	if ((uint32_t)(iSystemTick - iLastPhaseChangeTime) > mMotor.structMotor.LCT_PERIOD)
	{
		if (iLocateIndex < (sizeof(iLocatePhaseSequencyTable)/sizeof(uint8_t)))
		{
			//iLastPhaseChangeTime = iSystemTick; 
			setPhaseManually(mMotor.structMotor.LCT_DUTY, iLocatePhaseSequencyTable[iLocateIndex]);
			iLocateIndex++;
		}
		else
		{
			MOTOR_SHUT_DOWN;
			mMotor.structMotor.MSR.MotorPowerOn = FALSE;
			iCurrentPhase = iLocatePhaseSequencyTable[iLocateIndex - 1];
			return STATUS_FINISHED;
		}
	}
	return STATUS_WORKING;
}

__INLINE void BLDCRampUp_Manager(void)
{
	if (SET == FLAG_PHASE_CHANGED)
	{
		PhaseChangedRoutine();
		if (iPhaseChangeCNT4Period > CHANGE_DUTY_PERIOD_THR)
		{
			iPhaseChangeCNT4Period = 0;
			// Change duty and period 
//			MOTOR_RAMPUP_DT_INCR(mMotor.structMotor.ACT_DUTY);			
			MOTOR_RAMPUP_PR_DCR(mMotor.structMotor.ACT_PERIOD);	
			if (mMotor.structMotor.ACT_PERIOD <= MOTOR_RAMPUP_PR_MIN)
			{
				iRampUpPeriodMiniCNT++;
			}
		}
		iPhaseChangeCNT4Period++;
//		MOTOR_SET_DUTY(mMotor.structMotor.ACT_DUTY);
		TIMER_SET_CMP_VALUE(TIMER0, mMotor.structMotor.ACT_PERIOD);
		PHASE_INCREASE(iCurrentPhase);
		// Modify PWM->PHCHGNXT at last because I don't know how long needed to reload PHCH with PHCHNEXT after TIM0 time-out
		PWM->PHCHGNXT = GET_PHASE_VALUE(iCurrentPhase);
	}
}

// Take charge of all Motot control
void BLDCSensorLessManager(void)
{
	uint16_t iMotorAlreadyRotatingPhaseTime;
	static uint32_t iEnterTimeBeforeWait;

	// Duty too big protection
	if ((mMotor.structMotor.ACT_DUTY > MAX_MOTOR_PWR_DUTY) || (PWM->CMR[1] > MAX_MOTOR_PWR_DUTY))
	{
		stopMotor();
		setError(ERR_INTERNAL);
	}

	// Single phase duration too long protection 
	if (TRUE == mMotor.structMotor.MSR.MotorPowerOn)
	{
		if (iCurrentPHCHG != PWM->PHCHG)
		{
			iCurrentPHCHG = PWM->PHCHG;
			iLastPhaseChangeTime = iSystemTick;
		}
		else
		{
			if ((uint32_t)(iSystemTick - iLastPhaseChangeTime) > MAX_SINGLE_PHASE_DURATION) 
			{
				stopMotor();
				setError(ERR_INTERNAL);
			}
		}
	}

	switch (enumMotorState)
	{
	case MOTOR_IDLE:
		if (mMotor.structMotor.MCR.MotorNeedToRun && NO_MOTOR_EEROR)
		{
			iRotateDetectStartTime = iSystemTick;
			enumRotateDetectState = DETECT_START;
			enumMotorState = MOTOR_START;
		}
		break;
		
	case MOTOR_START:
		if (mMotor.structMotor.MCR.MotorNeedToRun && NO_MOTOR_EEROR)
		{
			// Later implement this when motor can rotate
			// Then stop it while rotating to measure the waveform
			// Manually rotate it is too slow 
			iMotorAlreadyRotatingPhaseTime = canMotorContinueRunning();
			if (iMotorAlreadyRotatingPhaseTime != ALREADY_ROTATING_DETECTING)
			{
				if (iMotorAlreadyRotatingPhaseTime)
				{
					// 1 to 65534
					enumMotorState = MOTOR_LOCKED;
				}
				else
				{
					// When back to Idle state the motor was already shut down
					// MOTOR_SHUT_DOWN;
					iCurrentPhase = 0;
					iLocateIndex = 0;
					mMotor.structMotor.MSR.MissedZXD_CNT = 0;
					iLastPhaseChangeTime = iSystemTick;
					mMotor.structMotor.MSR.MotorPowerOn = TRUE;
					// Clear start detect zero cross flag
					mMotor.structMotor.MSR.ZeroCrossDetecting = FALSE;
					mMotor.structMotor.MSR.Locked = FALSE;
					//setPhaseManually(mMotor.structMotor.LCT_DUTY, iCurrentPhase);
					BRG_ENABLE;
					enumMotorState = MOTOR_LOCATE;
				}
			}
		}
		else
		{
			stopMotor();
		}
		break;

	case MOTOR_LOCATE:
		if (mMotor.structMotor.MCR.MotorNeedToRun && NO_MOTOR_EEROR)
		{
			if (BLDCLocatingManager() == STATUS_FINISHED)
			{
				iEnterTimeBeforeWait = iSystemTick;
				enumMotorState = MOTOR_WAIT_AFTER_LOCATE;
			}
		}
		else
		{
			stopMotor();
		}
		break;

	case MOTOR_WAIT_AFTER_LOCATE:
		if (mMotor.structMotor.MCR.MotorNeedToRun && NO_MOTOR_EEROR)
		{
			if ((uint32_t)(iSystemTick - iEnterTimeBeforeWait) >= WAIT_AFTER_LOCATE_TIME)
			{
				mMotor.structMotor.ACT_DUTY = mMotor.structMotor.RU_DUTY;
				mMotor.structMotor.ACT_PERIOD = mMotor.structMotor.RU_PERIOD;
				mMotor.structMotor.MSR.MotorPowerOn = TRUE;
				PHASE_INCREASE(iCurrentPhase);
				setPhaseManually(mMotor.structMotor.ACT_DUTY, iCurrentPhase);
				BRG_ENABLE;
				// Set timer 0 valure, use timer 0 to change phase automatically
				// ************************************************************************
				// ----==== From here current iCurrentPhase is actually next phase ====----
				// What to get real current phase value? Read PWM->PHCHG.
				// ************************************************************************
				PHASE_INCREASE(iCurrentPhase);
				PWM->PHCHGNXT = GET_PHASE_VALUE(iCurrentPhase);
				// !!!! Need to make sure CPU run to here every min mMotor.structMotor.ACT_PERIOD time !!!
				// !!!! If not , timer counter may already passed mMotor.structMotor.ACT_PERIOD, !!!!
				// !!!! then need to count to 2^24, go back to 0 and triger interrupt when reach ACT_PERIOD !!!!
				TIMER_SET_CMP_VALUE(TIMER0, mMotor.structMotor.ACT_PERIOD);
				TIMER_Start(TIMER0);	// Once started, running and interrupting until Motor stop
				TIMER_EnableInt(TIMER0);
				iRampUpPeriodMiniCNT = 0;
				iPhaseChangeCNT4Duty = 0;
				iPhaseChangeCNT4Period = 0;
				enumMotorState = MOTOR_RAMPUP_WO_ZXD;
			}
		}
		else
		{
			stopMotor();
		}
		break;

	case MOTOR_RAMPUP_WO_ZXD:	// without zero cross detection
		if (mMotor.structMotor.MCR.MotorNeedToRun && NO_MOTOR_EEROR)
		{
			BLDCRampUp_Manager();
			if (mMotor.structMotor.ACT_PERIOD <= MOTOR_START_ZXD_SPEED)	//(iRampUpPeriodMiniCNT > MOTOR_START_ZXD_MINROT_CNT)  //
			{
				mMotor.structMotor.MSR.ThisPhaseDetectedZX = FALSE;
				mMotor.structMotor.MSR.ZeroCrossDetecting = TRUE;
				// Speed is enough for zero cross detecting
				// Prepare everything
				// T0 used to change phase automatically -- already configured
				// T1 used to filter ZX
				//ACMP->CMPCR[0]

//				TIMER_SET_CMP_VALUE(TIMER1, GET_TIM1_CMP_VALUE(TIMER1->TDR + AVOID_ZXD_AFTER_PHCHG));
//				FLAG_TIM1_USEAGE = ENUM_TIM1_AVOID_ZXD;
				ACMP0_ENABLE;
				TIMER_Start(TIMER1);	// Once started, running until Motor stop
//				TIMER_EnableInt(TIMER1);
				// Suppose last ZX detected time 
//				iLastZXDetectedTime = MINI51_TIM_CNT_MAX - mMotor.structMotor.ACT_PERIOD / 2;
				enumMotorState = MOTOR_RAMPUP_W_ZXD;
			}
		}
		else
		{
			stopMotor();
		}
		break;

	case MOTOR_RAMPUP_W_ZXD:	// with zero cross detection
		if (mMotor.structMotor.MCR.MotorNeedToRun && NO_MOTOR_EEROR)
		{
			if (TRUE == mMotor.structMotor.MSR.Locked)
			{
				// Finally, everything was prepared:
				// T0 used to change phase automatically
				// T1 used to filter ZX
				enumMotorState = MOTOR_LOCKED;
			}
			else
			{
				if (iRampUpPeriodMiniCNT < RAMP_UP_MIN_PERIOD_NUM_THRS)
				{
					BLDCRampUp_Manager(); 
				}
				else
				{
					setError(ERR_RAMPUP_FAIL);
				}
			}
		}
		else
		{
			stopMotor();
		}
		break;

	case MOTOR_LOCKED:
		if (mMotor.structMotor.MCR.MotorNeedToRun && NO_MOTOR_EEROR)
		{
			BLDCSpeedManager();	// Mainly PWM duty increase/decrease
		}
		else
		{
			stopMotor();
		}
		break;

	default:
		break;
	}
}

