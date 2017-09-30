#ifndef __PROTECTION_H__
#define __PROTECTION_H__

#include "global.h"

#define BATTERY_V_PORT				P3
#define BATTERY_V_PIN				((uint32_t)BIT1)
#define CURRENT_PORT				P5
#define CURRENT_PIN					((uint32_t)BIT3)

#define ADC_CURRENT_CHN_IDX			0
#define ADC_BATTERY_CHN_IDX			7
#define ADC_CURRENT_CHN_MSK			(0x01 << ADC_CURRENT_CHN_IDX)
#define ADC_BATTERY_CHN_MSK			(0x01 << ADC_BATTERY_CHN_IDX)
#define ADC_CURRENT_CMP_MSK			ADC_CMP0_INT
#define ADC_BATTERY_CMP_MSK			ADC_CMP1_INT
#define ADC_ADF_MSK					ADC_ADF_INT

// 5x50 m ohm R = 10m ohm, 30A*0.01=0.3V, VCC=3V, (0.3/3)*1024=102
// 5x50 m ohm R = 10m ohm, 30A*0.01=0.3V, VCC=5V, (0.3/5)*1024=61
#define ADC_CURRENT_HIGH_THRS			68
#define ADC_CURRENT_HIGH_CNT			12	// 300us*8*2(one current, one battery)=4.8ms
//16	// 15us*16*2(one current, one battery)=0.48ms interval of measurement
// 5x50 m ohm R = 10m ohm, 1A*0.01=0.01V, VCC=5V, (0.01/5)*1024=2
#define ADC_CURRENT_HIGH_THRS_MT		5	// For MOSFET check at start up, but 2 can be noise, so maybe 5 is OK
// If it is higher than 5 also maybe ADC is wrong

// Set 6.8V as min, (6.8/13.3)*3.3=1.687V, (1.687/3.3)*1024=524	// Actually using 10K and 3.3K R
// Set 10.2V as min, (10.2/13.3)*3.3=2.53V, (2.53/3)*1024=864	// Actually using 10K and 3.3K R
// Set 10.2V as min, (10.2/13.3)*3.3=2.53V, (2.53/5)*1024=518	// Actually using 10K and 3.3K R
// Set 10.2V as min, (10.2/12)*2=1.7V, (1.7/5)*1024=348	// Actually using 10K and 2K R
#define ADC_BAT_LOW_THRS				348
#define ADC_BAT_LOW_CNT					16	// Battery may need further filter, 300us*16*2(one current, one battery)=9.6ms
// Set 11.5V as min, (11.5/12)*2=1.92V, (1.92/5)*1024=393	// Actually using 10K and 2K R
#define ADC_BAT_LOW_THRS_MT				393

#ifdef __USED_BY_PTC_C__
#define EXTERNAL_PTC

volatile uint32_t* unMosfetTestTable[] = {&MOSFET_AS_PIN_ADDR, &MOSFET_BS_PIN_ADDR, &MOSFET_CS_PIN_ADDR,
	&MOSFET_AD_PIN_ADDR, &MOSFET_BD_PIN_ADDR, &MOSFET_CD_PIN_ADDR};
#define SET_MOSFET_ON_MANUAL(pinAddr)		(*(pinAddr) = 0)
#define SET_MOSFET_OFF_MANUAL(pinAddr)		(*(pinAddr) = 1)

#else
#define EXTERNAL_PTC extern
#endif

EXTERNAL_PTC void PTC_checkMotor(void);

#endif
