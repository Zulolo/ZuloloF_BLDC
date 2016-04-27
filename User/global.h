#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include "Mini51Series.h"

#define WHAT_EVER_DO_NOT_CARE		1

#define DEBUG_ACMP_OUT_PORT			P3
#define DEBUG_ACMP_OUT_PIN			BIT6
#define DEBUG_GPIO_PORT				P5
#define DEBUG_GPIO_PIN				BIT0
#define DEBUG_TX_PORT				P0
#define DEBUG_TX_PIN				BIT0
#define DEBUG_RX_PORT				P1
#define DEBUG_RX_PIN				BIT2

#define TIMER_INVALID_CNT 			0xFFFFFFFF

//#define unSystemTick 				SysTick->VAL

#define SET           			(1)     
#define RESET          			(0)      

#define INDEX_INCREASE(INDEX, MAX)	((INDEX) = (((INDEX) < ((MAX) - 1)) ? ((INDEX) + 1) : 0))

extern __IO uint32_t unSystemTick;

typedef struct
{
	struct
	{
		__IO uint16_t bMotorNeedToRun:1;
		__IO uint16_t bRotateDirection:1;
	}MCR;
	struct
	{
		__IO uint16_t bMotorPowerOn:1;
		__IO uint16_t bZeroCrossDetecting:1;
		__IO uint16_t bLocked:1;
		__IO uint16_t bThisPhaseDetectedZX:1;
		__IO uint16_t bNewComFrameReceived:1;
		__IO uint16_t unMissedZXD_CNT:8;
		__IO uint16_t unSuccessZXD_CNT:8;
	}MSR;
	__IO uint16_t  unLocatingDuty;		/*!<  PWM Locating Duty  */
	__IO uint16_t  unRampUpDuty;		/*!<  PWM Ramp Up Start Duty  */
	__IO uint16_t  unTargetDuty;		/*!<  PWM Target (Locked State) Duty  */
	__IO uint16_t  unActualDuty;		/*!<  PWM Actual Duty  */
	__IO uint16_t  unLocatingPeriod;	/*!<  Locating State One Phase Period  */
	__IO uint16_t  unRESERVE_1;			/*!<  for 4 bytes align */
	__IO uint32_t  unRampUpPeriod;		/*!<  Ramp Up Start One Phase Period  */
	__IO uint32_t  unActualPeriod;		/*!<  Actual One Phase Period  */
	__IO uint32_t  unPhaseChangeCNT;	/*!<  Phase changed counter  */
	__IO uint16_t  unRPM;				/*!<  Actual RPM  */
	__IO uint16_t  unBattery;			/*!<  Battery Voltage  */
	__IO uint16_t  unCurrent;			/*!<  Current  */
	__IO uint16_t  unRESERVE_2;			//*!<  for 4 bytes align */
} MOTOR_T;

typedef union
{
	uint16_t unValue[sizeof(MOTOR_T)/sizeof(uint16_t)];
	MOTOR_T structMotor;
} MOTOR_UNION_T;

#include "BLDCSensorLess.h"
#include "Communication.h"
#include "Error.h"
#include "Protection.h"
#endif 
