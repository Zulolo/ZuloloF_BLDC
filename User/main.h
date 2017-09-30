#ifndef __MAIN_H__
#define __MAIN_H__

#include "global.h"

#define ADC_CLK_DIVIDER				221	// 100KHz
//sample clock 16, total ADC time is (16+14)/100=300us,
//
//11 //22.1184/11=2MHz,
//sample clock 16, total ADC time is (16+14)/2=15us,
//trigger ADC after every ADC finished

#define TIMER0_PRESCALE				10	//22.1184M/(10+1)=2M
#define TIMER1_PRESCALE				10	//22.1184M/(10+1)=2M

#define PWM_CHN01_PRESCALER			1
#define PWM_CHN23_PRESCALER			1
#define PWM_CHN45_PRESCALER			1
#define PWM_CHN_ALL_MSK				(0x3Ful)
#define PWM_CHN_135_MSK				(0x2Aul)	
#define PWM_CHN_USED_MSK			(0x2Aul)	// Enable PWM 1 3 5
#define PCR_DEBUG_MODE				0x00000002
#define PCR_CLR_COUNTER				0x08000000
#define PCR_CH1_INV_EN_MSK			0x40
#define PCR_CH3_INV_EN_MSK			0x4000
#define PCR_CH5_INV_EN_MSK			0x400000
#define PCR_CH0_INV_EN_MSK			0x04
#define PCR_CH2_INV_EN_MSK			0x0400
#define PCR_CH4_INV_EN_MSK			0x040000
#define PCR_PERIOD_MODE(CHANNEL)	(8ul<<(4*(CHANNEL))) 
#define PCR_INV_EN(CHANNEL)			(4ul<<(4*(CHANNEL))) 
#define PCR_CH_EN(CHANNEL)			(1ul<<(4*(CHANNEL)))
#define PHCHG_CTL_CMP0				0x00000100

#define UART_CLK_DIVIDER			1
#define SYS_TICK_RELOAD_VALUE		(110592 - 1) // system tick's frequency is 22.1184M
// 110592 means every 5ms the CNT return to 0 and cause interrupt
uint8_t iTestSpeedSequenIndex = 0;
uint32_t iTestSpeedLastTime = 0;
const uint16_t iTestSpeedSequence[] = { 250, 300, 350, 400, 450, 300, 400, 250, 200, 450, 200 };
#define TEST_SPEED_SEQUENCE_NUM		(sizeof(iTestSpeedSequence)/sizeof(uint16_t))
__IO uint32_t unSystemTick = 0;

//extern void BLDC_SensorLessManager(void);
//extern void CommunicationManager(void);
//extern void ErrorManager(void);
//extern void checkMotor(void);
#endif 
