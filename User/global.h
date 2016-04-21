#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include "Mini51Series.h"

#define WHAT_EVER_DO_NOT_CARE		1

// GPIOs
#define BRG_FAULT_PORT				P3
#define BRG_FAULT_PIN_NUM			2
#define BRG_FAULT_PIN				BIT2
#define BRG_EN_PORT					P3
#define BRG_EN_PIN					BIT0
#define BRG_ENABLE					(P30 = 1)
#define BRG_DISABLE					(P30 = 0)

#define LED_PORT					P5
#define LED_PIN						BIT4
#define LED_OUTPUT					P54

#define BATTERY_V_PORT				P3
#define BATTERY_V_PIN				BIT1
#define CURRENT_PORT				P5
#define CURRENT_PIN					BIT3

#define DEBUG_ACMP_OUT_PORT			P3
#define DEBUG_ACMP_OUT_PIN			BIT6
#define DEBUG_GPIO_PORT				P5
#define DEBUG_GPIO_PIN				BIT0
#define DEBUG_TX_PORT				P0
#define DEBUG_TX_PIN				BIT0
#define DEBUG_RX_PORT				P1
#define DEBUG_RX_PIN				BIT2

#define COMM_PORT					P0
#define COMM_CLK_PIN				BIT7
#define COMM_CS_PIN					BIT1
#define COMM_TX_PIN					BIT6
#define COMM_RX_PIN					BIT5

#define MOSFET_DRV_0_4_PORT			P2
#define MOSFET_DRV_5_PORT			P0
#define MOSFET_DRV_0_PIN			BIT2
#define MOSFET_DRV_1_PIN			BIT3
#define MOSFET_DRV_2_PIN			BIT4
#define MOSFET_DRV_3_PIN			BIT5
#define MOSFET_DRV_4_PIN			BIT6
#define MOSFET_DRV_5_PIN			BIT4

#define MOSFET_AS_PIN_ADDR      	GPIO_PIN_ADDR(2, 2)
#define MOSFET_BS_PIN_ADDR     		GPIO_PIN_ADDR(2, 4)
#define MOSFET_CS_PIN_ADDR     		GPIO_PIN_ADDR(2, 6)
#define MOSFET_AD_PIN_ADDR      	GPIO_PIN_ADDR(2, 3)
#define MOSFET_BD_PIN_ADDR          GPIO_PIN_ADDR(2, 5)
#define MOSFET_CD_PIN_ADDR			GPIO_PIN_ADDR(0, 4)

#define GPIO_OFFD_OFF_SET			16
#define ZERO_DETECT_PORT			P1
#define ZERO_DETECT_A_PIN			BIT0
#define ZERO_DETECT_B_PIN			BIT3
#define ZERO_DETECT_C_PIN			BIT5
#define ZERO_DETECT_M_PIN			BIT4

#define TIMER_INVALID_CNT 			0xFFFFFFFF

#define COMM_BIT_LENTH				17	// highest bit is used to indicate command or data
#define COMM_MASK					(0x1FFFFul)	
#define IS_COMM_CMD(x)				((x)&(0x10000ul))
#define IS_COMM_CMD_WR(x)			((x)&(0x8000ul))	
#define COMM_VALUE_MASK				(0x7FFFul)
#define COMM_BAUT_RATE				5000000 

//#define unSystemTick 				SysTick->VAL

#define PWM_PHCHG_PWM_MASK			(0x00003F00ul)
#define PWM_PHCHG_D0_7_MASK			(0x000000FFul) 
#define PWM_PERIOD 					(884-1)	// PWM T=0.08ms, if target is 3K PRM, 42 E-Circle per M-Circle,
											// each PC should less than 0.5 ms. 
											// So each E-Circle at least has 6 PWM circle

#define ROTATE_CLOCKWISE		0
#define ROTATE_ANTICLOCKWISE	1

#define ADC_CURRENT_CHN_IDX		0
#define ADC_BATTERY_CHN_IDX		7
#define ADC_CURRENT_CHN_MSK		(0x01 << ADC_CURRENT_CHN_IDX)
#define ADC_BATTERY_CHN_MSK		(0x01 << ADC_BATTERY_CHN_IDX)
#define ADC_CURRENT_CMP_MSK		ADC_CMP0_INT
#define ADC_BATTERY_CMP_MSK		ADC_CMP1_INT
#define ADC_ADF_MSK				ADC_ADF_INT

#define SET           			(1)     
#define RESET          			(0)      


#define PWM_INT_ENABLE				(PWM_EnableDutyInt(PWM, 1, WHAT_EVER_DO_NOT_CARE))

#define PWM_INT_DISABLE				(PWM->PIER = 0)

#define IS_PWM_IRQ_ENABLED			(PWM->PIER)

// To increase shut down speed we can just directly write a constant number into PHCHG register
// But since we also have protection form driver IC, it is not so critical here
// So better just shut down, not change other bit in the register
// First write PHCHGNXT because between write PHCHG and PHCHGNXT time, PHCHG may be updated by PHCHGNXT                                 
//#define	MOSFET_ALL_OFF			(PWM->PHCHGNXT = (PWM->PHCHGNXT & (~PWM_PHCHG_PWM_MASK)) | PWM_PHCHG_D0_7_MASK); /
//								(PWM->PHCHG = (PWM->PHCHG & (~PWM_PHCHG_PWM_MASK)) | PWM_PHCHG_D0_7_MASK);
// After shut down all MOSFET you need to re-initialize all PHCHG register bits
#define MOSFET_SHUT_DOWN_VAL		(0x000000FFul)	
#define	MOTOR_SHUT_DOWN				BRG_DISABLE; \
									PWM_INT_DISABLE; \
									(TIMER_Stop(TIMER0)); \
									(TIMER_Stop(TIMER1)); \
									(TIMER_DisableInt(TIMER0)); \
									(TIMER_DisableInt(TIMER1)); \
									(PWM->PHCHGNXT = MOSFET_SHUT_DOWN_VAL); \
									(PWM->PHCHG = MOSFET_SHUT_DOWN_VAL)

//(ACMP->CMPCR[0] &= ~(ACMP_CMPCR_ACMPIE_Msk | ACMP_CMPCR_ACMPEN_Msk)); \

#define MOTOR_SET_DUTY(x)			(PWM->CMR[1] = (x)); \
									(PWM->CMR[3] = (x)); \
									(PWM->CMR[5] = (x))
//#define MOTOR_SET_DUTY(x)			(PWM->CMR[0] = (x)); \
//									(PWM->CMR[2] = (x)); \
//									(PWM->CMR[4] = (x))
//#define MOTOR_SET_DUTY(x)			(PWM->CMR[0] = (x)); \
//									(PWM->CMR[1] = (x)); \
//									(PWM->CMR[2] = (x)); \
//									(PWM->CMR[3] = (x)); \
//									(PWM->CMR[4] = (x)); \
//									(PWM->CMR[5] = (x))

#define INDEX_INCREASE(INDEX, MAX)	((INDEX) = (((INDEX) < ((MAX) - 1)) ? ((INDEX) + 1) : 0))
// Max 32 type of error because used uint32_t's bit to mark each error
// Since LED can not blink to fast (for human watchable), 6 type of blink make sense
typedef enum {	
	ERR_NULL = 0,
	ERR_CURRENT_WARNING,	// Current > 5A
	ERR_LOCATE_FAIL,
	ERR_RAMPUP_FAIL,
	ERR_BATTERY_LOW,
	ERR_INTERNAL,	// Loss lock, single phase duration too long
	ERR_CURRENT_BURNING,	// Current > 15A
	ERR_BRD_FAULT
} ENUM_ERROR_LEVEL;

extern __IO uint32_t unSystemTick;

#include "BLDCSensorLess.h"
#include "Communication.h"
#include "Error.h"

#endif 
