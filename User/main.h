#ifndef __MAIN_H__
#define __MAIN_H__

#include "global.h"

#define ADC_CLK_DIVIDER			221	// 100KHz	
									//sample clock 16, total ADC time is (16+14)/100=300us, 
									//
									//11 //22.1184/11=2MHz, 
								  //sample clock 16, total ADC time is (16+14)/2=15us, 
								  //trigger ADC after every ADC finished

// 5x5 m ohm R = 10m ohm, 30A*0.01=0.3V, VCC=3V, (0.3/3)*1024=102
// 5x5 m ohm R = 10m ohm, 30A*0.01=0.3V, VCC=5V, (0.3/5)*1024=61
#define ADC_CURRENT_HIGH_THRS	68
#define ADC_CURRENT_HIGH_CNT	12	// 300us*8*2(one current, one battery)=4.8ms	
									//16	// 15us*16*2(one current, one battery)=0.48ms interval of measurement

// Set 6.8V as min, (6.8/13.3)*3.3=1.687V, (1.687/3.3)*1024=524	// Actually using 10K and 3.3K R
// Set 10.2V as min, (10.2/13.3)*3.3=2.53V, (2.53/3)*1024=864	// Actually using 10K and 3.3K R
// Set 10.2V as min, (10.2/13.3)*3.3=2.53V, (2.53/5)*1024=518	// Actually using 10K and 3.3K R
// Set 10.2V as min, (10.2/12)*2=1.7V, (1.7/5)*1024=348	// Actually using 10K and 2K R
#define ADC_BAT_LOW_THRS		348
#define ADC_BAT_LOW_CNT			16	// Battery may need further filter, 300us*16*2(one current, one battery)=9.6ms
									
#define TIMER0_PRESCALE			10	//22.1184M/(10+1)=2M
#define TIMER1_PRESCALE			10	//22.1184M/(10+1)=2M

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
const uint16_t iTestSpeedSequence[] = {250, 300, 350, 400, 450, 300, 400, 250, 200, 450, 200};
#define TEST_SPEED_SEQUENCE_NUM		(sizeof(iTestSpeedSequence)/sizeof(uint16_t))
__IO uint32_t iSystemTick = 0;

//extern void BLDCSensorLessManager(void);
//extern void CommunicationManager(void);
//extern void ErrorManager(void);
//extern void checkMotor(void);
#endif 
