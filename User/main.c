/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 13/10/07 3:59p $ 
 * @brief    Template project for Mini51 series MCU
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/   
#include <stdio.h>
//#include "Mini51Series.h"
//#include "Mini5xxDE.h"  
#include "main.h"

void CLK_Init()
{
    /* Enable internal 22.1184MHz */
    CLK->PWRCON |= CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_IRC22M_STB_Msk);

    // Configure HCLK to use 22.1184MHz HIRC and div by 1
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_IRC22M, CLK_CLKDIV_HCLK(1));
//	CLK_DisableXtalRC(CLK_PWRCON_XTLCLK_EN_Msk);

    // Configure SysTick to use HIRC
//	SetSysTickClockSrc(CLK_CLKSEL0_STCLK_S_IRC22M_DIV2);

//    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLK_S_HCLK_DIV2);
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDT_S_HCLK_DIV2048, WHAT_EVER_DO_NOT_CARE);
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL1_ADC_S_IRC22M, CLK_CLKDIV_ADC(ADC_CLK_DIVIDER));
    CLK_SetModuleClock(PWM01_MODULE, CLK_CLKSEL1_PWM01_S_HCLK, WHAT_EVER_DO_NOT_CARE);
    CLK_SetModuleClock(PWM23_MODULE, CLK_CLKSEL1_PWM23_S_HCLK, WHAT_EVER_DO_NOT_CARE);
    CLK_SetModuleClock(PWM45_MODULE, CLK_CLKSEL2_PWM45_S_HCLK, WHAT_EVER_DO_NOT_CARE);
    CLK_SetModuleClock(UART_MODULE, CLK_CLKSEL1_UART_S_IRC22M, CLK_CLKDIV_UART(UART_CLK_DIVIDER));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HCLK , WHAT_EVER_DO_NOT_CARE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1_S_HCLK , WHAT_EVER_DO_NOT_CARE);
    CLK_SetModuleClock(SPI_MODULE, CLK_CLKSEL1_SPI_S_HCLK  , WHAT_EVER_DO_NOT_CARE);

    // Nai nai de seems can not use | | | to put all peripheral enable into one invoke
    CLK_EnableModuleClock(WDT_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_EnableModuleClock(SPI_MODULE);
    CLK_EnableModuleClock(UART_MODULE);
    CLK_EnableModuleClock(PWM01_MODULE);
    CLK_EnableModuleClock(PWM23_MODULE);
    CLK_EnableModuleClock(PWM45_MODULE);
    CLK_EnableModuleClock(ADC_MODULE);
    CLK_EnableModuleClock(ACMP_MODULE);
}

void IRQ_Init()
{
    NVIC_EnableIRQ(TMR0_IRQn);
    NVIC_EnableIRQ(TMR1_IRQn);
    NVIC_EnableIRQ(SPI_IRQn);
    NVIC_EnableIRQ(ACMP_IRQn);
    NVIC_EnableIRQ(EINT0_IRQn);
    NVIC_EnableIRQ(ADC_IRQn);
//    NVIC_EnableIRQ(PWM_IRQn);

    NVIC_SetPriority(TMR0_IRQn, 1);
    NVIC_SetPriority(TMR1_IRQn, 1);
    NVIC_SetPriority(SPI_IRQn, 3);
    NVIC_SetPriority(ACMP_IRQn, 2);
    NVIC_SetPriority(EINT0_IRQn, 0);
    NVIC_SetPriority(ADC_IRQn, 3);
//    NVIC_SetPriority(PWM_IRQn, 2);

    GPIO_EnableEINT0(BRG_FAULT_PORT, BRG_FAULT_PIN, GPIO_INT_FALLING);
}

void GPIO_Init()
{
/*---------------------------------------------------------------------------------------------------------*/
/* GPIO configuration                                                                                 	   */
/*---------------------------------------------------------------------------------------------------------*/
    // LED Pin
    GPIO_SetMode(LED_PORT, LED_PIN, GPIO_PMD_OUTPUT);

    // Bridge Enable Pin
    GPIO_SetMode(BRG_EN_PORT, BRG_EN_PIN, GPIO_PMD_OUTPUT);
    BRG_DISABLE;

    // Bridge Fault Pin
    GPIO_SetMode(BRG_FAULT_PORT, BRG_FAULT_PIN, GPIO_PMD_INPUT);

    // PWM Pin
    GPIO_SetMode(MOSFET_DRV_0_4_PORT, MOSFET_DRV_0_PIN | MOSFET_DRV_1_PIN | MOSFET_DRV_2_PIN | 
			     MOSFET_DRV_3_PIN | MOSFET_DRV_4_PIN, GPIO_PMD_OUTPUT);
    GPIO_SetMode(MOSFET_DRV_5_PORT, MOSFET_DRV_5_PIN, GPIO_PMD_OUTPUT);

    // SPI, I am slave
    GPIO_SetMode(COMM_PORT, COMM_CLK_PIN | COMM_CS_PIN | COMM_RX_PIN, GPIO_PMD_INPUT);
    GPIO_SetMode(COMM_PORT, COMM_TX_PIN, GPIO_PMD_OUTPUT);

    // UART Pin
    GPIO_SetMode(DEBUG_TX_PORT, DEBUG_TX_PIN, GPIO_PMD_OUTPUT);
    GPIO_SetMode(DEBUG_RX_PORT, DEBUG_RX_PIN, GPIO_PMD_INPUT);

    // DEBUG for ACMP Output Pin
    GPIO_SetMode(DEBUG_ACMP_OUT_PORT, DEBUG_ACMP_OUT_PIN, GPIO_PMD_OUTPUT);
	GPIO_SetMode(DEBUG_GPIO_PORT, DEBUG_GPIO_PIN, GPIO_PMD_OUTPUT);
	P50 = 0;

    // ADC for current Pin
    GPIO_DISABLE_DIGITAL_PATH(CURRENT_PORT, CURRENT_PIN << GPIO_OFFD_OFF_SET);
    GPIO_SetMode(CURRENT_PORT, CURRENT_PIN, GPIO_PMD_INPUT);
    // ADC for battery Pin
    GPIO_DISABLE_DIGITAL_PATH(BATTERY_V_PORT, BATTERY_V_PIN << GPIO_OFFD_OFF_SET);
    GPIO_SetMode(BATTERY_V_PORT, BATTERY_V_PIN, GPIO_PMD_INPUT);
	    
    // ACMP Pin
    GPIO_DISABLE_DIGITAL_PATH(ZERO_DETECT_PORT, (ZERO_DETECT_A_PIN | ZERO_DETECT_B_PIN |
			      ZERO_DETECT_C_PIN | ZERO_DETECT_M_PIN) << GPIO_OFFD_OFF_SET);
    GPIO_SetMode(ZERO_DETECT_PORT, ZERO_DETECT_A_PIN | ZERO_DETECT_B_PIN |
		 ZERO_DETECT_C_PIN | ZERO_DETECT_M_PIN, GPIO_PMD_INPUT);
/*---------------------------------------------------------------------------------------------------------*/
/* Init I/O Multi-function                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART RXD and TXD */
    SYS->P0_MFP &= ~SYS_MFP_P00_Msk;
    SYS->P0_MFP |= SYS_MFP_P00_TXD;  
    SYS->P1_MFP &= ~SYS_MFP_P12_Msk;
    SYS->P1_MFP |= SYS_MFP_P12_RXD; 
	 
	/* Set multi-function pins for SPI */
    SYS->P0_MFP &= ~(SYS_MFP_P01_Msk | SYS_MFP_P05_Msk | SYS_MFP_P06_Msk | SYS_MFP_P07_Msk);
    SYS->P0_MFP |= (SYS_MFP_P01_SPISS | SYS_MFP_P05_MOSI | SYS_MFP_P06_MISO | SYS_MFP_P07_SPICLK); 

	/* Set multi-function pins for ADC for current */
    SYS->P5_MFP &= ~SYS_MFP_P53_Msk;
    SYS->P5_MFP |= SYS_MFP_P53_AIN0;  

	/* Set multi-function pins for ADC for battery */
    SYS->P3_MFP &= ~SYS_MFP_P31_Msk;
    SYS->P3_MFP |= SYS_MFP_P31_AIN7;  

	/* Set multi-function pins for ACMP output for debug */
    SYS->P3_MFP &= ~SYS_MFP_P36_Msk;
    SYS->P3_MFP |= SYS_MFP_P36_CPO0;
    SYS->P5_MFP &= ~SYS_MFP_P50_Msk;
    SYS->P5_MFP |= SYS_MFP_P50_GPIO;

	/* Set multi-function pins for ACMP */
    SYS->P1_MFP &= ~SYS_MFP_P14_Msk;
    SYS->P1_MFP |= SYS_MFP_P14_CPN0;  
	// pp will changed in the 
//    SYS->P1_MFP &= ~(SYS_MFP_P10_Msk | SYS_MFP_P13_Msk | SYS_MFP_P14_Msk | SYS_MFP_P15_Msk);
//    SYS->P1_MFP |= (SYS_MFP_P10_CPP0 | SYS_MFP_P13_CPP0 | SYS_MFP_P14_CPN0 | SYS_MFP_P15_CPP0); 
// 	          
    /* Set multi-function pins for PWM */
    SYS->P2_MFP &= ~(SYS_MFP_P22_Msk | SYS_MFP_P23_Msk | SYS_MFP_P24_Msk | SYS_MFP_P25_Msk | SYS_MFP_P26_Msk);
    SYS->P2_MFP = SYS_MFP_P22_PWM0 | SYS_MFP_P23_PWM1 | SYS_MFP_P24_PWM2 | SYS_MFP_P25_PWM3 |SYS_MFP_P26_PWM4;
    SYS->P0_MFP &= ~SYS_MFP_P04_Msk;
    SYS->P0_MFP |= SYS_MFP_P04_PWM5;  

	/* Set multi-function pins for EINT0 */
    SYS->P3_MFP &= ~SYS_MFP_P32_Msk;
    SYS->P3_MFP |= SYS_MFP_P32_INT0;  
	
}

void TIM_Config(void)
{
	// T0 used to change phase automatically
	// T1 used to filter ZX
    TIMER0->TCSR  =  TIMER_TCSR_CRST_Msk | TIMER_PERIODIC_MODE | TIMER_TCSR_PERIODIC_SEL_Msk | TIMER_TCSR_TDR_EN_Msk + TIMER0_PRESCALE;   
    TIMER1->TCSR  =  TIMER_TCSR_CRST_Msk | TIMER_CONTINUOUS_MODE | TIMER_TCSR_TDR_EN_Msk + TIMER1_PRESCALE; 
    // TIMER1->TCSR |=  TIMER_TCSR_CEN_Msk ;    
	//TIMER_EnableInt(TIMER0);                         
    //TIMER_EnableInt(TIMER1);
}

void ADC_Config(void)
{

    ADC_SetExtraSampleTime(ADC, 0 , ADC_SAMPLE_CLOCK_16);

    // Enable channel 0 and 7 (Current and Battery)
//    ADC_Open(ADC, 0, 0, ADC_BATTERY_CHN_MSK);	//ADC_CURRENT_CHN_MSK | ADC_BATTERY_CHN_MSK);	
// Do NOT use this, it will clear all bit in ADCR


    // Power on ADC
    ADC_POWER_ON(ADC);

    // ADC start triggered by TIM and take turn between current and battery
    // Use two ADC comparator to hardware trace the big cuurent or battery low
 
    // Configure and enable Comperator 0 to monitor channel 0(current) input greater or euqal to 93
    ADC_ENABLE_CMP0(ADC, ADC_CURRENT_CHN_IDX, ADC_CMP_GREATER_OR_EQUAL_TO, ADC_CURRENT_HIGH_THRS, ADC_CURRENT_HIGH_CNT);
    // Configure and enable Comperator 1 to monitor channel 7(battery) input less than 0x200	
    ADC_ENABLE_CMP1(ADC, ADC_BATTERY_CHN_IDX, ADC_CMP_LESS_THAN, ADC_BAT_LOW_THRS, ADC_BAT_LOW_CNT);    

    // Enable ADC comparator 0 and 1 interrupt
    ADC_EnableInt(ADC, ADC_ADF_INT);
    ADC_EnableInt(ADC, ADC_CMP0_INT);
    ADC_EnableInt(ADC, ADC_CMP1_INT);

    ADC_SET_INPUT_CHANNEL(ADC, ADC_BATTERY_CHN_MSK);
    ADC_START_CONV(ADC);
}

void SPI_Config(void)
{
/*---------------------------------------------------------------------------------------------------------*/
/* Init SPI                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a slave, clock idle low, falling clock edge Tx, rising edge Rx and 32-bit transaction */
    /* Set IP clock divider. SPI clock rate = 10MHz */
    SPI_Close(SPI);
    SPI_ClearRxFIFO(SPI);
    SPI_ClearTxFIFO(SPI);
    // peripheral clock frequency of slave device must be faster than the bus clock frequency of the master
    SPI_Open(SPI, SPI_SLAVE, SPI_MODE_0, COMM_BIT_LENTH, COMM_BAUT_RATE);

    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_EnableAutoSS(SPI, SPI_SS, SPI_SS_ACTIVE_LOW);

    /* Use FIFO */
    //SPI_EnableFIFO(SPI, SPI_TX_FIFO_THRE, SPI_RX_FIFO_THRE);
    /* Enable SPI Unit Transfer Interrupt */
    SPI_EnableInt(SPI, SPI_IE_MASK);
}

//void ACMP_Config(void)
//{
//    ACMP->CMPCR[0] = ACMP_CMPCR_HYSEN_Msk | ACMP_CMPCR_ACMPIE_Msk | ACMP_CMPCR_ACMPEN_Msk;
//    //ACMP->CR1 = ACMP_CPP1_P31_0 | ACMP_HYST_EN | ACMP_IE | ACMP_EN ;
//}

void PWM_Init(void)
{
    PWM_Stop(PWM, PWM_CHN_ALL_MSK);
    PWM_SET_PRESCALER(PWM, 0, PWM_CHN01_PRESCALER);
//    PWM_SET_PRESCALER(PWM, 1, PWM_CHN01_PRESCALER);
    PWM_SET_PRESCALER(PWM, 2, PWM_CHN23_PRESCALER);
//    PWM_SET_PRESCALER(PWM, 3, PWM_CHN23_PRESCALER);
    PWM_SET_PRESCALER(PWM, 4, PWM_CHN45_PRESCALER);
//    PWM_SET_PRESCALER(PWM, 5, PWM_CHN45_PRESCALER);
    PWM_SET_DIVIDER(PWM, 0, PWM_CLK_DIV_1);
    PWM_SET_DIVIDER(PWM, 1, PWM_CLK_DIV_1);
    PWM_SET_DIVIDER(PWM, 2, PWM_CLK_DIV_1);
    PWM_SET_DIVIDER(PWM, 3, PWM_CLK_DIV_1);
    PWM_SET_DIVIDER(PWM, 4, PWM_CLK_DIV_1);
    PWM_SET_DIVIDER(PWM, 5, PWM_CLK_DIV_1);

//    PWM->PCR = PCR_CLR_COUNTER | PCR_DEBUG_MODE | PCR_CH_EN(0) | PCR_PERIOD_MODE(0) |
//		PCR_CH_EN(1) | PCR_PERIOD_MODE(1) | PCR_CH_EN(2) | PCR_PERIOD_MODE(2) |
//		PCR_CH_EN(3) | PCR_PERIOD_MODE(3) | PCR_CH_EN(4) | PCR_PERIOD_MODE(4) |
//		PCR_CH_EN(5) | PCR_PERIOD_MODE(5) |
//		PCR_INV_EN(0) | PCR_INV_EN(2) | PCR_INV_EN(4) |
//		PCR_INV_EN(1) | PCR_INV_EN(3) | PCR_INV_EN(5);
    PWM->PCR = PCR_CLR_COUNTER | PCR_DEBUG_MODE |
		PCR_CH_EN(0) | PCR_PERIOD_MODE(0) |  
		PCR_CH_EN(1) | PCR_PERIOD_MODE(1) | 
		PCR_CH_EN(2) | PCR_PERIOD_MODE(2) | 
		PCR_CH_EN(3) | PCR_PERIOD_MODE(3) | 
		PCR_CH_EN(4) | PCR_PERIOD_MODE(4) | 
		PCR_CH_EN(5) | PCR_PERIOD_MODE(5) |
		PCR_INV_EN(0) | PCR_INV_EN(2) | PCR_INV_EN(4) |
		PCR_INV_EN(1) | PCR_INV_EN(3) | PCR_INV_EN(5);
//    PWM_SET_ALIGNED_TYPE(PWM_EDGE_ALIGNED);
//    PWM_ENABLE_GROUP_MODE(PWM);
    PWM->CMR[0] = 0;
    PWM->CMR[1] = 0;
    PWM->CMR[2] = 0;
    PWM->CMR[3] = 0;
    PWM->CMR[4] = 0;
    PWM->CMR[5] = 0;
    PWM->CNR[0] = PWM_PERIOD; 
    PWM->CNR[1] = PWM_PERIOD;    
    PWM->CNR[2] = PWM_PERIOD;                                    
    PWM->CNR[3] = PWM_PERIOD;
    PWM->CNR[4] = PWM_PERIOD; 
    PWM->CNR[5] = PWM_PERIOD;
    PWM_EnableOutput(PWM, PWM_CHN_ALL_MSK);
    PWM_INT_DISABLE;	// Disable all PWM interrupt 
    stopMotor();
    //MOTOR_SHUT_DOWN;

    // PWM duty change every each phase for both ramp up and locked state
//    PWM->INTACCUCTL = 0x41; // Every 4 PWM periods change duty
			    // this will be used in locked time
			    // During startup ramp, duty change every x E-Circle
    PWM->PHCHGMASK = PHCHG_CTL_CMP0;	// Input of ACMP0 is controlled by PHCHG
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    
    /* Clock initialization, Enable PWM, ADC, TIM, UART clock */
    CLK_Init();
    //CLK->APBCLK = CLK_APBCLK_UART_EN_Msk;
    
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate(); 

    /* System tick Configuration */
    // SYS TICK interrupt will be entered every 5ms
    SysTick_Config(SYS_TICK_RELOAD_VALUE);
    
    /* PWM Configuration */
    // Close all MOSFET here first, then output GPIO
    PWM_Init();
      
    /* IO Configuration */
    GPIO_Init();

    /* ACMP initialization */
    //ACMP_Config();

    /* ADC initialization */
    ADC_Config();

    /* TIM initialization */
    TIM_Config();

    /* SPI initialization */
    SPI_Config();

    /* Enable all interrupt from NVIC */
    IRQ_Init();

    /* Lock protected registers */
    SYS_LockReg();
                         
}    

//    uint32_t imsTest;
//	uint32_t iEntreTime;
//	uint32_t iLeaveTime;
int main()
{

    SYS_Init();
//	TIMER_Start(TIMER1);
//	imsTest = 0;
//
//	iEntreTime = TIMER_GetCounter(TIMER1);
//	while ((PWM->PIIR & PWM_PIIR_PWMPIF1_Msk))
//	{
////		BRG_DISABLE;
////		stopMotor();
//		if (imsTest > 1000)
//		{
//			break;
//		}
//
//		if (ACMP0_EDGE_MATCH)
//		{
//			imsTest = 0;
//		}
//		else
//		{
//			imsTest++;
//		}
//
//	}
//	iLeaveTime = TIMER_GetCounter(TIMER1);
    /* ---=== Interrupt already enabled so communication can start ===--- */

    // reset motor state machine to stop
    // power was already stopped providing to motor and 
    // enumMotorState was already initialized in PWM_Init()
    //stopMotor();

    /* Init UART to 115200-8n1 for print message */
    /* This MCU don't have DMA, need to use interrupt to make un-polling UART TX */
    /* Now better not to use printf after motor was started, in future if needed will write one */
//    UART_Open(UART, 115200);
//   
//    printf("Hello World\n");

    checkMotor();

    /* ----==== Here is the parameter used for test ====----*/
    // Max PWM Duty is: PWM_PERIOD = 441
    // Max Period is:
    MOTOR_SHUT_DOWN;

    mMotor.structMotor.LCT_DUTY = 200;
    mMotor.structMotor.LCT_PERIOD = 10;	// Unit ms
    mMotor.structMotor.RU_DUTY = 320;
    mMotor.structMotor.RU_PERIOD = 8000;	// Unit 2MH, 20ms, 500rpm
	mMotor.structMotor.TGT_DUTY = 400;
	mMotor.structMotor.MCR.RotateDirection = ROTATE_CLOCKWISE;	// Clockwise
    mMotor.structMotor.MCR.MotorNeedToRun = TRUE;
    /* ----=============== Test End ================---- */

    while(1)
    {
//	if (TIMER_GetCounter(TIMER0) > 100 && TIMER_GetCounter(TIMER0) < 1000)
//	{
//	    imsTest = iSystemTick;
//	}
//	if (TIMER_GetCounter(TIMER0) > 30000 && TIMER_GetCounter(TIMER0) < 31000)
//	{
//	    imsTest = iSystemTick;
//	}
	BLDCSensorLessManager(); 
		//	CommunicationManager();
	ErrorManager();

		// For test
	if (TRUE == mMotor.structMotor.MSR.Locked)
	{
		if (iSystemTick%5000 == 0)
		{
			if (iTestSpeedLastTime != iSystemTick)
			{
				iTestSpeedLastTime = iSystemTick;
				mMotor.structMotor.TGT_DUTY = iTestSpeedSequence[iTestSpeedSequenIndex];
				INDEX_INCREASE(iTestSpeedSequenIndex, TEST_SPEED_SEQUENCE_NUM);
			}
		}
	}
	}

}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
