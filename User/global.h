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
//		__IO uint16_t  MCR;		/*!<  Motor Control  */
//		__IO uint16_t  MSR;		/*!<  Motor Status  */
	//  __IO uint32_t  ACNR;       /*!<  PWM Actual Counter Register  */
	//  __IO uint32_t  STCNR;        /*!<  PWM Start Counter Register  */
	//  __IO uint32_t  SBCNR;        /*!<  PWM Stable Counter Register  */
	__IO uint16_t  unLCT_DUTY;	/*!<  PWM Locating Duty  */
	__IO uint16_t  unRU_DUTY;		/*!<  PWM Ramp Up Start Duty  */
	__IO uint16_t  unTGT_DUTY;	/*!<  PWM Target (Locked State) Duty  */
	__IO uint16_t  unACT_DUTY;	/*!<  PWM Actual Duty  */
	__IO uint16_t  unLCT_PERIOD;	/*!<  Locating State One Phase Period  */
	__IO uint32_t  unRU_PERIOD;	/*!<  Ramp Up Start One Phase Period  */
	__IO uint32_t  unACT_PERIOD;	/*!<  Actual One Phase Period  */
	__IO uint32_t  unPHASE_CHANGE_CNT;	/*!<  Phase changed counter  */
	__IO uint16_t  unRPM;			/*!<  Actual RPM  */
	__IO uint16_t  unRESERVE;		/*!<  Reserve for future use (round up 32bits) */
	__IO uint16_t  unBATTERY;		/*!<  Battery Voltage  */
	__IO uint16_t  unCURRENT;		/*!<  Current  */
} MOTOR_T;

#include "BLDCSensorLess.h"
#include "Communication.h"
#include "Error.h"
#include "Protection.h"
#endif 
