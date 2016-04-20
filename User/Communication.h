#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#include "global.h"

	#ifdef __USED_BY_COMMUNICATION_C__
		#define EXTERNAL_COMM  

	#else 
		#define EXTERNAL_COMM extern
	#endif
typedef enum{
	MOTOR_MCR = 0,	/*!<  Motor Control  */
	MOTOR_MSR,		/*!<  Motor Status  */
	MOTOR_LCT_DUTY,		/*!<  PWM Locating Duty  */
	MOTOR_RU_DUTY,		/*!<  PWM Ramp Up Start Duty  */
	MOTOR_TGT_DUTY,		/*!<  PWM Target (Locked State) Duty  */
	MOTOR_ACT_DUTY,		/*!<  PWM Actual Duty  */
	MOTOR_LCT_PERIOD,	/*!<  Locating State One Phase Period  */
	MOTOR_RU_PERIOD_LOW,	/*!<  Ramp Up Start One Phase Period  */
	MOTOR_RU_PERIOD_HIGH,	/*!<  Ramp Up Start One Phase Period  */
	MOTOR_ACT_PERIOD_LOW,	/*!<  Actual One Phase Period  */
	MOTOR_ACT_PERIOD_HIGH,	/*!<  Actual One Phase Period  */
	MOTOR_RPM,			/*!<  Actual RPM  */
	MOTOR_RESERVE,		/*!<  Reserve for future use (round up 32bits) */
	MOTOR_BATTERY,		/*!<  Battery Voltage  */
	MOTOR_CURRENT		/*!<  Current  */
} ENUM_COMM_REG;
EXTERNAL_COMM uint16_t iSPI_ReadData;	// 0 or 0xFFFF means no data
EXTERNAL_COMM uint16_t iRegisterValue;	// 0 or 0xFFFF means no data
EXTERNAL_COMM ENUM_COMM_REG enumRegister;
EXTERNAL_COMM uint8_t FlagRegisterNeedWrite;

EXTERNAL_COMM void CommunicationManager(void);
#endif
