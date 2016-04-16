/**************************************************************************//**
 * @file     clk.c
 * @version  V1.00
 * $Revision: 21 $
 * $Date: 13/10/01 9:05a $ 
 * @brief    MINI51 series CLK driver source file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/ 
#include "Mini51Series.h"
/** @addtogroup MINI51_Device_Driver MINI51 Device Driver
  @{
*/

/** @addtogroup MINI51_CLK_Driver CLK Driver
  @{
*/


/** @addtogroup MINI51_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

/**
  * @brief  This function disable frequency output function.
  * @return None
  */
void CLK_DisableCKO(void)
{ 
  /* Disable CKO clock source */
  CLK->APBCLK &= (~CLK_APBCLK_FDIV_EN_Msk); 
}

/**
  * @brief  This function enable frequency divider module clock, 
  *         enable frequency divider clock function and configure frequency divider.  
  * @param  u32ClkSrc is frequency divider function clock source
  *           - \ref CLK_CLKSEL2_FRQDIV_XTAL        
  *           - \ref CLK_CLKSEL2_FRQDIV_HCLK       
  *           - \ref CLK_CLKSEL2_FRQDIV_IRC22M 
  * @param  u32ClkDiv is system reset source
  * @param  u32ClkDivBy1En is frequency divided by one enable.
  * @return None
  *
  * @details    Ouput selected clock to CKO. The output clock frequency is divided by u32ClkDiv. 
  *             The formula is:
  *                 CKO frequency = (Clock source frequency) / 2^(u32ClkDiv + 1)
  *             This function is just used to set CKO clock.
  *             User must enable I/O for CKO clock output pin by themselves.
  */
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En)
{
  /* CKO = clock source / 2^(u32ClkDiv + 1) */
  CLK->FRQDIV = CLK_FRQDIV_DIVIDER_EN_Msk | u32ClkDiv | u32ClkDivBy1En<<CLK_FRQDIV_DIVIDER1_Pos;

  /* Enable CKO clock source */
  CLK->APBCLK |= CLK_APBCLK_FDIV_EN_Msk;

  /* Select CKO clock source */
  CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_FRQDIV_S_Msk)) | u32ClkSrc;  
}

/**
  * @brief  This function let sytem enter to Power-down mode.  
  * @return None
  */
void CLK_PowerDown(void)
{
  SCB->SCR = SCB_SCR_SLEEPDEEP_Msk;
  CLK->PWRCON |= (CLK_PWRCON_PWR_DOWN_EN_Msk | CLK_PWRCON_PD_WU_STS_Msk);
  __WFI();
}

/**
  * @brief  This function let sytem enter to Idle mode   
  * @return None
  */
void CLK_Idle(void)
{
  CLK->PWRCON |= (CLK_PWRCON_PWR_DOWN_EN_Msk | CLK_PWRCON_PD_WU_STS_Msk);
  __WFI();
}

/**
  * @brief  This function get external high frequency crystal frequency. The frequency unit is Hz.  
  * @return None
  */
uint32_t CLK_GetHXTFreq(void)
{  
  if(CLK->PWRCON & CLK_PWRCON_XTL12M )
    return __XTAL12M;
  else 
    return 0;
}

/**
  * @brief  This function get external low frequency crystal frequency. The frequency unit is Hz.  
  * @return LXT frequency
  */
uint32_t CLK_GetLXTFreq(void)
{
  if(CLK->PWRCON & CLK_PWRCON_XTL32K )
    return __XTAL32K;
  else 
    return 0;
}

/**
  * @brief  This function get HCLK frequency. The frequency unit is Hz.  
  * @return HCLK frequency
  */
uint32_t CLK_GetHCLKFreq(void)
{
  SystemCoreClockUpdate();
  return SystemCoreClock;
}


/**
  * @brief  This function get CPU frequency. The frequency unit is Hz.  
  * @return CPU frequency
  */
uint32_t CLK_GetCPUFreq(void)
{
  SystemCoreClockUpdate();
  return SystemCoreClock;
}

/**
  * @brief  This function set HCLK clock source and HCLK clock divider
  * @param  u32ClkSrc is HCLK clock source. Including :
  *           - \ref CLK_CLKSEL0_HCLK_S_XTAL
  *           - \ref CLK_CLKSEL0_HCLK_S_IRC10K
  *           - \ref CLK_CLKSEL0_HCLK_S_IRC22M
  * @param  u32ClkDiv is HCLK clock divider. Including :
  *           - \ref CLK_CLKDIV_HCLK(x)
  * @return None
  */
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
  CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLK_S_Msk) | u32ClkSrc; 
  CLK->CLKDIV = (CLK->CLKDIV & ~CLK_CLKDIV_HCLK_N_Msk) | u32ClkDiv; 
}

/**
  * @brief  This function set selected module clock source and module clock divider
  * @param  u32ModuleIdx is module index. 
  * @param  u32ClkSrc is module clock source.  
  * @param  u32ClkDiv is module clock divider.
  * @return None
  * @details Valid parameter combinations listed in following table: 
  *
  * |Module index        |Clock source                          |Divider                 |
  * | :----------------  | :----------------------------------- | :--------------------- |
  * |\ref WDT_MODULE     |\ref CLK_CLKSEL1_WDT_S_XTAL           | x                      |
  * |\ref WDT_MODULE     |\ref CLK_CLKSEL1_WDT_S_HCLK_DIV2048   | x                      |
  * |\ref WDT_MODULE     |\ref CLK_CLKSEL1_WDT_S_IRC10K         | x                      |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0_S_XTAL          | x                      |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0_S_IRC10K        | x                      |
  * |\ref TMR0_MODULE    |\ref CLK_CLKSEL1_TMR0_S_HCLK          | x                      |
  * |\ref TMR0_MODULE    | \ref CLK_CLKSEL1_TMR0_S_IRC22M       | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1_S_XTAL          | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1_S_IRC10K        | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1_S_HCLK          | x                      |
  * |\ref TMR1_MODULE    |\ref CLK_CLKSEL1_TMR1_S_IRC22M        | x                      |
  * |\ref FDIV_MODULE    |\ref CLK_CLKSEL2_FRQDIV_XTAL          | x                      |
  * |\ref FDIV_MODULE    |\ref CLK_CLKSEL2_FRQDIV_HCLK          | x                      |
  * |\ref FDIV_MODULE    |\ref CLK_CLKSEL2_FRQDIV_IRC22M        | x                      |
  * |\ref I2C_MODULE     | x                                    | x                      |
  * |\ref SPI_MODULE     |\ref CLK_CLKSEL1_SPI_S_HXTorLXT       | x                      |
  * | \ref SPI_MODULE    |\ref CLK_CLKSEL1_SPI_S_HCLK           | x                      |
  * |\ref UART_MODULE    |\ref CLK_CLKSEL1_UART_S_XTAL          |\ref CLK_CLKDIV_UART(x) |
  * |\ref UART_MODULE    |\ref CLK_CLKSEL1_UART_S_IRC22M        |\ref CLK_CLKDIV_UART(x) |
  * |\ref PWM01_MODULE   |\ref CLK_CLKSEL1_PWM01_S_HCLK         | x                      |
  * |\ref PWM23_MODULE   |\ref CLK_CLKSEL1_PWM23_S_HCLK         | x                      |
  * |\ref PWM45_MODULE   |\ref CLK_CLKSEL2_PWM45_S_HCLK         | x                      |
  * |\ref ADC_MODULE     |\ref CLK_CLKSEL1_ADC_S_XTAL           |\ref CLK_CLKDIV_ADC(x)  |
  * |\ref ADC_MODULE     |\ref CLK_CLKSEL1_ADC_S_HCLK           |\ref CLK_CLKDIV_ADC(x)  |
  * | \ref ADC_MODULE    |\ref CLK_CLKSEL1_ADC_S_IRC22M         |\ref CLK_CLKDIV_ADC(x)  |
  * | \ref ACMP_MODULE   | x                                    | x                      | 
  */
  
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv)
{
  uint32_t u32tmp=0,u32sel=0,u32div=0;
    
  if(MODULE_CLKSEL_Msk(u32ModuleIdx)!=MODULE_NoMsk)
  {
    u32sel = (uint32_t)(&(CLK->CLKSEL0))+((MODULE_CLKSEL(u32ModuleIdx))*4);
    u32tmp = *(volatile uint32_t *)(u32sel);
    u32tmp = (u32tmp & (~(MODULE_CLKSEL_Msk(u32ModuleIdx)<<MODULE_CLKSEL_Pos(u32ModuleIdx)))) | u32ClkSrc;    
    *(volatile uint32_t *)(u32sel) = u32tmp;
  }

  if(MODULE_CLKDIV_Msk(u32ModuleIdx)!=MODULE_NoMsk)
  {
    u32div =(uint32_t)(&(CLK->CLKDIV))+((MODULE_CLKDIV(u32ModuleIdx))*4);
    u32tmp = *(volatile uint32_t *)(u32div);
    u32tmp = (u32tmp & (~(MODULE_CLKDIV_Msk(u32ModuleIdx)<<MODULE_CLKDIV_Pos(u32ModuleIdx)))) | u32ClkDiv;  
    *(volatile uint32_t *)(u32div) = u32tmp;    
  }
}

/**
  * @brief  This function set systick clock source
  * @param  u32ClkSrc is module clock source. Including
  *           - \ref CLK_CLKSEL0_STCLK_S_XTAL
  *           - \ref CLK_CLKSEL0_STCLK_S_XTAL_DIV2
  *           - \ref CLK_CLKSEL0_STCLK_S_HCLK_DIV2
  *           - \ref CLK_CLKSEL0_STCLK_S_IRC22M_DIV2
  * @return None
  */
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc)
{
  CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_STCLK_S_Msk) | u32ClkSrc;
}

/**
  * @brief  This function enable clock source
  * @param  u32ClkMask is clock source mask. Including :
  *         - \ref CLK_PWRCON_XTL12M or CLK_PWRCON_XTL32K,
  *         - \ref CLK_PWRCON_OSC10K_EN_Msk
  *         - \ref CLK_PWRCON_OSC22M_EN_Msk
  * @return None
  */
void CLK_EnableXtalRC(uint32_t u32ClkMask)
{
  CLK->PWRCON |=u32ClkMask; 
}

/**
  * @brief  This function disable clock source
  * @param  u32ClkMask is clock source mask. Including :
  *         - \ref CLK_PWRCON_XTLCLK_EN_Msk,
  *         - \ref CLK_PWRCON_OSC10K_EN_Msk,
  *         - \ref CLK_PWRCON_OSC22M_EN_Msk,
  * @return None
  */
void CLK_DisableXtalRC(uint32_t u32ClkMask)
{
  CLK->PWRCON &=~u32ClkMask;
}

/**
  * @brief  This function enable module clock
  * @param  u32ModuleIdx is module index. Including :
  *   - \ref WDT_MODULE    
  *   - \ref TMR0_MODULE   
  *   - \ref TMR1_MODULE   
  *   - \ref FDIV_MODULE   
  *   - \ref I2C_MODULE    
  *   - \ref SPI_MODULE    
  *   - \ref UART_MODULE   
  *   - \ref PWM01_MODULE  
  *   - \ref PWM23_MODULE  
  *   - \ref PWM45_MODULE  
  *   - \ref ADC_MODULE    
  *   - \ref ACMP_MODULE     
  * @return None
  */
void CLK_EnableModuleClock(uint32_t u32ModuleIdx)
{
  *(volatile uint32_t *)((uint32_t)&CLK->APBCLK+(MODULE_APBCLK(u32ModuleIdx)*4))  |= 1<<MODULE_IP_EN_Pos(u32ModuleIdx);
}

/**
  * @brief  This function disable module clock
  * @param  u32ModuleIdx is module index
  *   - \ref WDT_MODULE    
  *   - \ref TMR0_MODULE   
  *   - \ref TMR1_MODULE   
  *   - \ref FDIV_MODULE   
  *   - \ref I2C_MODULE    
  *   - \ref SPI_MODULE    
  *   - \ref UART_MODULE   
  *   - \ref PWM01_MODULE  
  *   - \ref PWM23_MODULE  
  *   - \ref PWM45_MODULE  
  *   - \ref ADC_MODULE    
  *   - \ref ACMP_MODULE
  * @return None
  */
void CLK_DisableModuleClock(uint32_t u32ModuleIdx)
{
  *(volatile uint32_t *)((uint32_t)&CLK->APBCLK+(MODULE_APBCLK(u32ModuleIdx)*4))  &= ~(1<<MODULE_IP_EN_Pos(u32ModuleIdx));
}

/**
  * @brief  This function execute delay function. 
  * @param  us  Delay time. The Max value is 2^24 / CPU Clock(MHz). Ex:
  *                             50MHz => 335544us, 48MHz => 349525us, 28MHz => 699050us ...
  * @return None
  * @details    Use the SysTick to generate the delay time and the UNIT is in us. 
  *             The SysTick clock source is from HCLK, i.e the same as system core clock.
  */
void CLK_SysTickDelay(uint32_t us)
{
    SysTick->LOAD = us * CyclesPerUs;
    SysTick->VAL  =  (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
}

/**
  * @brief  This function check selected clock source status
  * @param  u32ClkMask is selected clock source. Including
  *           - \ref CLK_CLKSTATUS_CLK_SW_FAIL_Msk
  *           - \ref CLK_CLKSTATUS_IRC22M_STB_Msk
  *           - \ref CLK_CLKSTATUS_IRC10K_STB_Msk
  *           - \ref CLK_CLKSTATUS_XTL_STB_Msk
  *
  * @return None
  */
void CLK_WaitClockReady(uint32_t u32ClkMask)
{
    int32_t i32TimeOutCnt;

    i32TimeOutCnt = __HSI / 200; /* About 5ms */
    
    while((CLK->CLKSTATUS & u32ClkMask) != u32ClkMask)
    {
        if(i32TimeOutCnt-- <= 0)
            break;    
    } 
}


/*@}*/ /* end of group MINI51_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group MINI51_CLK_Driver */

/*@}*/ /* end of group MINI51_Device_Driver */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
