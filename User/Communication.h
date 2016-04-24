#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#include "global.h"

	#ifdef __USED_BY_COMMUNICATION_C__
		#define EXTERNAL_COMM  
		#define COM_SPI_TRANS_ERR_THRESHOLD		6

	#else 
		#define EXTERNAL_COMM extern
	#endif

#define COMM_PORT					P0
#define COMM_CLK_PIN				BIT7
#define COMM_CS_PIN					BIT1
#define COMM_TX_PIN					BIT6
#define COMM_RX_PIN					BIT5

#define COMM_FIFO_LENGTH			4
#define COMM_BIT_LENTH				16	// Because STM32F407 can only configure SPI to be 8bits/16bits width
										// highest bit in first half word is used to indicate command or data
#define COMM_RD_CMD_CNT				2	// R(0):Address | CRC
#define COMM_WR_CMD_CNT				4	// W(1):Address | Data Low | Data High | CRC

#define COMM_RW_MASK				(0x8000)
#define IS_COMM_RD(value)			((value) & COMM_RW_MASK)
#define COMM_DATA_MASK				(0x7FFF)
#define COMM_BAUT_RATE				5000000	// 5MHz

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
EXTERNAL_COMM uint32_t unCOM_SPI_TransCNT;
EXTERNAL_COMM uint32_t unCOM_SPI_TransErrCNT;
EXTERNAL_COMM uint16_t unCOM_SPI_ReadData[COMM_FIFO_LENGTH];	// 0 or 0xFFFF means no data
EXTERNAL_COMM uint16_t unRegisterValue;	// 0 or 0xFFFF means no data
EXTERNAL_COMM ENUM_COMM_REG tRegister;
EXTERNAL_COMM uint8_t FlagRegisterNeedWrite;

EXTERNAL_COMM void COMM_Manager(void);
#endif
