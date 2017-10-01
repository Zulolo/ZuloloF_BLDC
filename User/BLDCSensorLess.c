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
 5. So when TIM1 INT really happened, and the rising/falling edge is correct, it means ACMP was stable and really zero crossed.
 6. In the TIM1 INT we set TIM0's phase change time.*/

/* Some limitation:
 1. Because the phase duration variable is uint16_t, max one phase is 32ms,
 so min rotation speed is about 180RMP (12 e-phase).*/

__INLINE void PhaseChangedRoutine(void) {
	FLAG_PHASE_CHANGED = RESET;
	tMotor.structMotor.unPhaseChangeCNT++;

	if (TRUE == tMotor.structMotor.MSR.bZeroCrossDetecting) {
//		iPhaseChangeTime = TIMER_GetCounter(TIMER1);
		// Miss ZXD or ZXD success filter
		// If continuously detected more than MIN_SUCC_ZXD_THRESHOLD ZX, OK! GOOD!!
		if (TRUE == tMotor.structMotor.MSR.bThisPhaseDetectedZX) {
			tMotor.structMotor.unMissedZXD_CNT = 0;

			if (tMotor.structMotor.unSuccessZXD_CNT > MIN_SUCC_ZXD_THRESHOLD) {
				tMotor.structMotor.MSR.bLocked = TRUE;
			} else {
				tMotor.structMotor.unSuccessZXD_CNT++;
			}
		} else		// If continuously missing detected more than MAX_MISS_ZXD_THRESHOLD ZX, loss lock
		{
			tMotor.structMotor.unSuccessZXD_CNT = 0;
			// If ZX was not detected in last phase, unLastZXDetectedTime was also not updated
			// Guess one value
			unLastZXDetectedTime = GET_TIMER_DIFF((tMotor.structMotor.unActualPeriod >> 2), TIMER_GetCounter(TIMER1));
			if (tMotor.structMotor.unMissedZXD_CNT > MAX_MISS_ZXD_THRESHOLD) {
				if (TRUE == tMotor.structMotor.MSR.bLocked) {
					tMotor.structMotor.MSR.bLocked = FALSE;
					MOTOR_SHUT_DOWN;
					setError(ERR_INTERNAL);
				}
			} else {
				tMotor.structMotor.unMissedZXD_CNT++;
			}
		}

	}

	if (TRUE == tMotor.structMotor.MSR.bLocked) {
		// Set a rough next phase change time as the same with last phase
		// After detected ZX in TIM1 interrupt, next phase change time will be re-configured
		TIMER_SET_CMP_VALUE(TIMER0, tMotor.structMotor.unActualPeriod << 1);
	}

	tMotor.structMotor.MSR.bThisPhaseDetectedZX = FALSE;
	// For debug
	GPIO_TOGGLE(P50);
}

/* return STATUS_WORKING: still detecting
 0: no rotation detected
 1-65534: phase time */
uint16_t canMotorContinueRunning(void) {
// Later implement this when motor can rotate
// Then stop it while rotating to measure the waveform
// Manually rotate it is too slow 
#ifdef FOR_CAR
	return 0;
#else
	uint16_t unPhaseDuration = 0;
	static uint32_t unStateEnterTime;
	if ((uint32_t) (unSystemTick - unRotateDetectStartTime) > MAX_ALREADY_ROTATING_DETECT_TIME) {
		return 0;
	}
	switch (tRotateDetectState) {
	case DETECT_START:
		unStateEnterTime = unSystemTick;
		tRotateDetectState = DETECT_PHASE_1_P;
		break;

	case DETECT_PHASE_1_P:
		if ((uint32_t) (unSystemTick - unStateEnterTime) > MAX_ROTATING_DETECT_PHASE_TIME) {
			return (uint16_t) 0;
		} else {

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

	return unPhaseDuration;
#endif
}

// Mainly PWM duty increase/decrease
void BLDCSpeedManager(void) {
	if (SET == FLAG_PHASE_CHANGED) {
		PhaseChangedRoutine();

		if (tMotor.structMotor.unActualDuty != tMotor.structMotor.unTargetDuty) {
//				tMotor.structMotor.ACT_DUTY = tMotor.structMotor.TGT_DUTY;
			// Change PWM duty after each x phase change
			if (unPhaseChangeCNT_AtCurrentDuty > CHANGE_DUTY_AFTER_PHASE_CHANGED_NUM) {
				unPhaseChangeCNT_AtCurrentDuty = 0;
				if (tMotor.structMotor.unActualDuty < tMotor.structMotor.unTargetDuty) {
					tMotor.structMotor.unActualDuty++;
				} else {
					tMotor.structMotor.unActualDuty--;
				}
				MOTOR_SET_DUTY(tMotor.structMotor.unActualDuty);
			}
			unPhaseChangeCNT_AtCurrentDuty++;
		}

		PHASE_INCREASE(unCurrentPhase);
		// Modify PWM->PHCHGNXT at last because I don't know how long needed to reload PHCH with PHCHNEXT after TIM0 time-out
		PWM->PHCHGNXT = GET_PHASE_VALUE(unCurrentPhase);
	}
}

__INLINE void BLDC_stopMotor(void) {
	MOTOR_SHUT_DOWN;
	tMotor.structMotor.MCR.bMotorNeedToRun = FALSE;
	tMotor.structMotor.MSR.bMotorPowerOn = FALSE;
	tMotorState = MOTOR_IDLE;
}

__INLINE void setPhaseManually(uint16_t iPWMDuty, uint8_t iPhase) {
	MOTOR_SET_DUTY(iPWMDuty);
	PWM->PHCHG = GET_PHASE_VALUE(iPhase);
}

ENUM_STATUS BLDC_LocatingManager(void) {
	if ((uint32_t) (unSystemTick - unLastPhaseChangeTime) > tMotor.structMotor.unLocatingPeriod) {
		if (unLocateIndex < (sizeof(unLocatePhaseSequencyTable) / sizeof(uint8_t))) {
			//iLastPhaseChangeTime = unSystemTick; 
			setPhaseManually(tMotor.structMotor.unLocatingDuty, unLocatePhaseSequencyTable[unLocateIndex]);
			unLocateIndex++;
		} else {
			MOTOR_SHUT_DOWN;
			tMotor.structMotor.MSR.bMotorPowerOn = FALSE;
			unCurrentPhase = unLocatePhaseSequencyTable[unLocateIndex - 1];
			return STATUS_FINISHED;
		}
	}
	return STATUS_WORKING;
}

__INLINE void BLDCRampUp_Manager(void) {
	if (SET == FLAG_PHASE_CHANGED) {
		PhaseChangedRoutine();
		if (unPhaseChangeCNT_AtCurrentPeriod > CHANGE_DT_PR_AFTER_PHASE_CHANGED_NUM) {
			unPhaseChangeCNT_AtCurrentPeriod = 0;
			// Change duty and period 
//			MOTOR_RAMPUP_DT_INCR(tMotor.structMotor.ACT_DUTY);			
			MOTOR_RAMPUP_PR_DCR(tMotor.structMotor.unActualPeriod);
			if (tMotor.structMotor.unActualPeriod <= MOTOR_RAMPUP_PR_MIN) {
				unPeriodChangeCNT_AfterPR_ReachMini++;
			}
		}
		unPhaseChangeCNT_AtCurrentPeriod++;
//		MOTOR_SET_DUTY(tMotor.structMotor.ACT_DUTY);
		TIMER_SET_CMP_VALUE(TIMER0, tMotor.structMotor.unActualPeriod);
		PHASE_INCREASE(unCurrentPhase);
		// Modify PWM->PHCHGNXT at last because I don't know how long needed to reload PHCH with PHCHNEXT after TIM0 time-out
		PWM->PHCHGNXT = GET_PHASE_VALUE(unCurrentPhase);
	}
}

__INLINE void dutyProtection(void) {
	// Duty too big protection
	if ((tMotor.structMotor.unActualDuty > MAX_MOTOR_PWR_DUTY) || (PWM->CMR[1] > MAX_MOTOR_PWR_DUTY)) {
		BLDC_stopMotor();
		setError(ERR_INTERNAL);
	}
}

__INLINE void phaseDurationProtection(uint32_t unLastPhaseChangeTime) {
	static uint32_t unCurrentPHCHG;
	// Single phase duration too long protection
	if (TRUE == tMotor.structMotor.MSR.bMotorPowerOn) {
		if (unCurrentPHCHG != PWM->PHCHG) {
			unCurrentPHCHG = PWM->PHCHG;
			unLastPhaseChangeTime = unSystemTick;
		} else {
			if ((uint32_t) (unSystemTick - unLastPhaseChangeTime) > MAX_SINGLE_PHASE_DURATION) {
				BLDC_stopMotor();
				setError(ERR_INTERNAL);
			}
		}
	}
}

// Take charge of all Motot control
void BLDC_SensorLessManager(void) {
	uint16_t unMotorAlreadyRotatingPhaseTime;
#ifdef WAIT_AFTER_LOCATE
	static uint32_t iEnterTimeBeforeWait;
#endif
	dutyProtection();
	phaseDurationProtection(unLastPhaseChangeTime);

	switch (tMotorState) {
	case MOTOR_IDLE:
		if (tMotor.structMotor.MCR.bMotorNeedToRun && NO_MOTOR_EEROR) {
			unRotateDetectStartTime = unSystemTick;
			tRotateDetectState = DETECT_START;
			tMotorState = MOTOR_START;
		}
		break;

	case MOTOR_START:
		if (tMotor.structMotor.MCR.bMotorNeedToRun && NO_MOTOR_EEROR) {
			// Later implement this when motor can rotate
			// Then stop it while rotating to measure the waveform
			// Manually rotate it is too slow 
			unMotorAlreadyRotatingPhaseTime = canMotorContinueRunning();
			if (unMotorAlreadyRotatingPhaseTime != IS_ROTATING_DETECTING) {
				if (unMotorAlreadyRotatingPhaseTime > 0) {
					// 1 to 65534
					tMotorState = MOTOR_LOCKED;
				} else {
					// When back to Idle state the motor was already shut down
					// MOTOR_SHUT_DOWN;
					unCurrentPhase = 0;
					unLocateIndex = 0;
					tMotor.structMotor.unMissedZXD_CNT = 0;
					unLastPhaseChangeTime = unSystemTick;
					tMotor.structMotor.MSR.bMotorPowerOn = TRUE;
					// Clear start detect zero cross flag
					tMotor.structMotor.MSR.bZeroCrossDetecting = FALSE;
					tMotor.structMotor.MSR.bLocked = FALSE;
					//setPhaseManually(tMotor.structMotor.LCT_DUTY, unCurrentPhase);
					BRG_ENABLE;
					tMotorState = MOTOR_LOCATE;
				}
			}
		} else {
			BLDC_stopMotor();
		}
		break;

	case MOTOR_LOCATE:
		if (tMotor.structMotor.MCR.bMotorNeedToRun && NO_MOTOR_EEROR) {
			if (BLDC_LocatingManager() == STATUS_FINISHED) {
#ifdef WAIT_AFTER_LOCATE
				iEnterTimeBeforeWait = unSystemTick;
#endif
				tMotorState = MOTOR_WAIT_AFTER_LOCATE;
			}
		} else {
			BLDC_stopMotor();
		}
		break;

	case MOTOR_WAIT_AFTER_LOCATE:
		if (tMotor.structMotor.MCR.bMotorNeedToRun && NO_MOTOR_EEROR) {
#ifdef WAIT_AFTER_LOCATE
			if ((uint32_t) (unSystemTick - iEnterTimeBeforeWait) >= WAIT_AFTER_LOCATE_TIME) {
#endif					
				tMotor.structMotor.unActualDuty = tMotor.structMotor.unRampUpDuty;
				tMotor.structMotor.unActualPeriod = tMotor.structMotor.unRampUpPeriod;
				tMotor.structMotor.MSR.bMotorPowerOn = TRUE;
				PHASE_INCREASE(unCurrentPhase);
				setPhaseManually(tMotor.structMotor.unActualDuty, unCurrentPhase);
				BRG_ENABLE;
				// Set timer 0 valure, use timer 0 to change phase automatically
				// ************************************************************************
				// ----==== From here current unCurrentPhase is actually next phase ====----
				// ----==== Because we want to use the HW auto phase changer (PWM->PHCHGNXT) ====----
				// So increase unCurrentPhase again. Want to get real current phase value? Read PWM->PHCHG.
				// ************************************************************************
				PHASE_INCREASE(unCurrentPhase);
				PWM->PHCHGNXT = GET_PHASE_VALUE(unCurrentPhase);
				// !!!! Need to make sure CPU runs to here every min tMotor.structMotor.ACT_PERIOD time !!!
				// !!!! If not , timer counter may already passed tMotor.structMotor.ACT_PERIOD, !!!!
				// !!!! then need to count to max timer counter number (which is 2^24), !!!!
				// !!!! go back to 0 and triger interrupt when reach ACT_PERIOD again !!!!
				TIMER_SET_CMP_VALUE(TIMER0, tMotor.structMotor.unActualPeriod);
				TIMER_Start(TIMER0);				// Once started, running and interrupting until Motor stop
				TIMER_EnableInt(TIMER0);
				unPeriodChangeCNT_AfterPR_ReachMini = 0;
				unPhaseChangeCNT_AtCurrentDuty = 0;
				unPhaseChangeCNT_AtCurrentPeriod = 0;
				tMotorState = MOTOR_RAMPUP_WO_ZXD;
#ifdef WAIT_AFTER_LOCATE
			}
#endif
		} else {
			BLDC_stopMotor();
		}
		break;

	case MOTOR_RAMPUP_WO_ZXD:	// without zero cross detection
		if (tMotor.structMotor.MCR.bMotorNeedToRun && NO_MOTOR_EEROR) {
			BLDCRampUp_Manager();
			if (tMotor.structMotor.unActualPeriod <= MOTOR_START_ZXD_SPEED)	//(iRampUpPeriodMiniCNT > MOTOR_START_ZXD_MINROT_CNT)  //
			{
				tMotor.structMotor.MSR.bThisPhaseDetectedZX = FALSE;
				tMotor.structMotor.MSR.bZeroCrossDetecting = TRUE;
				// Speed is enough for zero cross detecting
				// Prepare everything
				// T0 used to change phase automatically -- already configured
				// T1 used to filter ZX

//				TIMER_SET_CMP_VALUE(TIMER1, GET_TIM1_CMP_VALUE(TIMER1->TDR + AVOID_ZXD_AFTER_PHCHG));
//				FLAG_TIM1_USEAGE = ENUM_TIM1_AVOID_ZXD;
				ACMP0_ENABLE;
				TIMER_Start(TIMER1);				// Once started, running until Motor stop
//				TIMER_EnableInt(TIMER1);
						// Suppose last ZX detected time
//				unLastZXDetectedTime = MINI51_TIM_CNT_MAX - tMotor.structMotor.ACT_PERIOD / 2;
				tMotorState = MOTOR_RAMPUP_W_ZXD;
			}
		} else {
			BLDC_stopMotor();
		}
		break;

	case MOTOR_RAMPUP_W_ZXD:	// with zero cross detection
		if (tMotor.structMotor.MCR.bMotorNeedToRun && NO_MOTOR_EEROR) {
			if (TRUE == tMotor.structMotor.MSR.bLocked) {
				// Finally, everything was prepared:
				// T0 used to change phase automatically
				// T1 used to filter ZX
				tMotorState = MOTOR_LOCKED;
			} else {
				if (unPeriodChangeCNT_AfterPR_ReachMini < RAMP_UP_MIN_PERIOD_NUM_THRS) {
					BLDCRampUp_Manager();
				} else {
					setError(ERR_RAMPUP_FAIL);
				}
			}
		} else {
			BLDC_stopMotor();
		}
		break;

	case MOTOR_LOCKED:
		if (tMotor.structMotor.MCR.bMotorNeedToRun && NO_MOTOR_EEROR) {
			BLDCSpeedManager();	// Mainly PWM duty increase/decrease
		} else {
			BLDC_stopMotor();
		}
		break;

	default:
		break;
	}
}

