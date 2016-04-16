/*****************************************************************************
 * @file     Mini5xxBN.h      
 * @author   NUVOTON Shang-Hai                                                                              	                 
 * @date     26. January 2013                                           		  

 * Copyright (C) 2011 Nuvoton Technology Corp. All rights reserved.   
 ******************************************************************************/
//=======================================================================================
//"外设数据结构体"按字母的先后顺序定义,  后面是各寄存器"位常量"定义 
//查寻寄存器的名字,可找到该寄存器位常量定义,如查"GPIO0->PMD"可找到它的位常量定义
//#define常量定义, 后缀字符说明:
//  _EN   :  Enable,使能
//  _DIS  :  Disable,禁止
//  _IE   :  Interrupt Enable, 中断使能
//  _IF   :  Interrupt Flag,   中断标志位
//  _F    :  Flag,状态标志位
//  _RD   :  Read,  读
//  _WR   ： Write, 写
//  _RST  :  Reset, 清零,或复位
//  _MASK :  有效位全被定义为1, 取反后做与运算, 可清除有效位 
//  _nbit :  此定义有n位
//  _0    :  定义值为0, 即常量所有位都为0
//  最后有个下杠“_”, 表示受保护位, 写之前要解锁
//=======================================================================================

#ifndef __MINI51_H__
#define __MINI51_H__

#ifdef __cplusplus
  extern "C" {
#endif

typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

typedef  volatile unsigned char        vu8;
typedef  volatile unsigned short       vu16;
typedef  volatile unsigned long        vu32;

#define  FALSE     (0)
#define  TRUE      (~0)
#define  NULL      (0)
#define  _OK_      (0)
#define  _ERR_     (~0)

#define  SRAM_BASE                     ((uint32_t)0x20000000)
#define  APB1PERIPH_BASE               ((uint32_t)0x40000000)
#define  APB2PERIPH_BASE               ((uint32_t)0x40100000)
#define  AHBPERIPH_BASE                ((uint32_t)0x50000000) 

#define  UNLOCKREG(x) {GCR->RegLockAddr  = 0x59; GCR->RegLockAddr = 0x16; GCR->RegLockAddr = 0x88;}
#define  LOCKREG(x)   GCR->RegLockAddr = 0x00;      

#define  INTID             ((INTID_TypeDef *)(AHBPERIPH_BASE + 0x00300))

//=======================================================================================
// ---------- Interrupt Number Definition --------------------------------------------
//=======================================================================================
typedef enum IRQn{
/******  Cortex-M0 Processor Exceptions Numbers *****************************************/
  NonMaskableInt_IRQn	  = -14,	/*!< 2 Non Maskable Interrupt                           */
  HardFault_IRQn		    = -13,	/*!< 3 Cortex-M0 Hard Fault Interrupt                   */
  SVCall_IRQn			      = -5,	  /*!< 11 Cortex-M0 SV Call Interrupt                     */
  PendSV_IRQn			      = -2,	  /*!< 14 Cortex-M0 Pend SV Interrupt                     */
  SysTick_IRQn			    = -1,	  /*!< 15 Cortex-M0 System Tick Interrupt                 */
/****** Mini5x Interrupt Numbers **********************************************************/	
  BOD_IRQn              = 0,	/*!< Brownout low voltage detected interrupt              */
  WDT_IRQn              = 1,	/*!< Watch Dog Timer interrupt                            */
  EINT0_IRQn            = 2,	/*!< External signal interrupt from P3.2 pin              */
  EINT1_IRQn            = 3,	/*!< External signal interrupt from P3.3 pin              */
  GPIO01_IRQn           = 4,	/*!< External signal interrupt from P0/P1                 */
  GPIO234_IRQn          = 5,	/*!< External interrupt from P2/P3/P4                     */
  PWM_IRQn              = 6,	/*!< PWM interrupt                                        */
  FB_IRQn               = 7,	/*!< Fault break interrupt                                */
  TMR0_IRQn             = 8,	/*!< Timer 0 interrupt                                    */
  TMR1_IRQn             = 9,	/*!< Timer 1 interrupt                                    */
  UART0_IRQn            = 12,	/*!< UART0 interrupt                                      */
  SPI_IRQn              = 14,	/*!< SPI0 interrupt                                       */
  GPIO5_IRQn            = 16,	/*!< External interrupt from P5                           */
  HIRC_IRQn             = 17, /*!< HFIRC trim) interrupt source identify                */
  I2C_IRQn              = 18,	/*!< I2C interrupt                                        */
  ACMP_IRQn				      = 25,	/*!< ACMP interrupt                                       */
  PDWU_IRQn             = 28,	/*!< Power Down Wake up interrupt                         */
  ADC_IRQn              = 29	/*!< ADC interrupt                                        */
} IRQn_Type;

//=======================================================================================
// ----------- Processor and Core Peripheral Section ---------------------------------
//=======================================================================================
/* Configuration of the Cortex-M0 Processor and Core Peripherals */
#define __MPU_PRESENT             0         /*!< MPU present or not                               */
#define __NVIC_PRIO_BITS          2         /*!< Number of Bits used for Priority Levels          */
#define __Vendor_SysTickConfig    0         /*!< Set to 1 if different SysTick Config is used     */

#include "core_cm0.h"                       /* Cortex-M0 processor and core peripherals           */

#if defined ( __CC_ARM  )
  #pragma anon_unions
#endif

//=======================================================================================
//                Device Specific Peripheral registers structures            
//=======================================================================================

//----------------------------- ADC Controller -------------------------------
typedef struct
{
  volatile  uint32_t  DR;                                  //AD结果
	          uint32_t  Res0[7];
  volatile  uint32_t  CR;                                  //控制寄存器
  volatile  uint32_t  CHER;                                //AD通道寄存器
  volatile  uint32_t  CMPR0;                               //比较
  volatile  uint32_t  CMPR1;
  volatile  uint32_t  SR;                                  //状态寄存器
} ADC_TypeDef;
#define    ADC               ((ADC_TypeDef *)0x400E0000)

//---------------------- Analog Comparator Controller -------------------------
typedef struct
{
  volatile uint32_t  CR0;                                  //控制
  volatile uint32_t  CR1;
  volatile uint32_t  SR;                                   //状态
  volatile uint32_t  RVCR;                                 //负端参考电压选择
} ACMP_TypeDef;
#define    ACMP              ((ACMP_TypeDef *)(APB1PERIPH_BASE + 0xD0000))

//---------------------------- Clock Controller ------------------------------
typedef struct
{
  volatile uint32_t  PWRCON;                               //受保护,位6除外
  volatile uint32_t  AHBCLK;
  volatile uint32_t  APBCLK;                               //位0: WTD_EN 受保护
  volatile uint32_t  CLKSTATUS;
  volatile uint32_t  CLKSEL0;                              //受保护,STCLK_S[5:3],HCLK_S[2:0]
  volatile uint32_t  CLKSEL1;                              //位[1:0]WTD_S 受保护
  volatile uint32_t  CLKDIV;
  volatile uint32_t  CLKSEL2;
           uint32_t  Res0;
  volatile uint32_t  FRQDIV;
} CLK_TypeDef;
#define    CLK               ((CLK_TypeDef *)(0x50000200))

//-------------------------- FLASH Memory Controller -------------------------
typedef struct
{
  volatile uint32_t  ISPCON;                         //受保护寄存器
  volatile uint32_t  ISPADR;
  volatile uint32_t  ISPDAT;
  volatile uint32_t  ISPCMD;
  volatile uint32_t  ISPTRG;                         //受保护寄存器
  volatile uint32_t  DFBADR;                         //Data Flash起始地址,位[8:0]必须为0
} FMC_TypeDef;
#define    FMC               ((FMC_TypeDef *)(0x5000C000))

//---------------------------- Global Controller -----------------------------
typedef struct
{
  volatile uint32_t  PDID;
  volatile uint32_t  RST_SRC;
  volatile uint32_t  IPRST_CTL1;
  volatile uint32_t  IPRST_CTL2;
  volatile uint32_t  LDOCR ;     
           uint32_t  Res0[1];
  volatile uint32_t  BODCTL;
           uint32_t  Res1[2];
  volatile uint32_t  PORCTL;
           uint32_t  Res2[2];
  volatile uint32_t  P0_MFP;
  volatile uint32_t  P1_MFP;
  volatile uint32_t  P2_MFP;
  volatile uint32_t  P3_MFP;
  volatile uint32_t  P4_MFP;
  volatile uint32_t  P5_MFP;
           uint32_t  Res3[14];
  volatile uint32_t  IRCTRIMCTL;
  volatile uint32_t  IRCTRIMIER;
  volatile uint32_t  IRCTRIMISR;
           uint32_t  Res4[29];
  volatile uint32_t  RegLockAddr;
           uint32_t  Res5[3];
  volatile uint32_t  RCADJ;	   
} GCR_TypeDef;
#define    GCR               ((GCR_TypeDef *)( 0x50000000))

//---------------------------- GPIO ---------------------------------------
typedef struct
{
  volatile uint32_t  PMD;
  volatile uint32_t  OFFD;
  volatile uint32_t  DOUT;
  volatile uint32_t  DMASK;
  volatile uint32_t  PIN;
  volatile uint32_t  DBEN;
  volatile uint32_t  IMD;
  volatile uint32_t  IER;
  volatile uint32_t  ISR;
} GPIO_TypeDef;
#define    GPIO0             ((GPIO_TypeDef *)(0x50004000))
#define    GPIO1             ((GPIO_TypeDef *)(0x50004040))
#define    GPIO2             ((GPIO_TypeDef *)(0x50004080))
#define    GPIO3             ((GPIO_TypeDef *)(0x500040C0))
#define    GPIO4             ((GPIO_TypeDef *)(0x50004100))
#define    GPIO5             ((GPIO_TypeDef *)(0x50004140))
#define    GPIODBNCE         (*(volatile uint32_t *)(0x50004180))

#define    GPIO00            (*((volatile uint32_t *)(0x50004200)))
#define    GPIO01            (*((volatile uint32_t *)(0x50004204)))
#define    GPIO04            (*((volatile uint32_t *)(0x50004210)))
#define    GPIO05            (*((volatile uint32_t *)(0x50004214)))
#define    GPIO06            (*((volatile uint32_t *)(0x50004218)))
#define    GPIO07            (*((volatile uint32_t *)(0x5000421C)))

#define    GPIO10            (*((volatile uint32_t *)(0x50004220)))
#define    GPIO12            (*((volatile uint32_t *)(0x50004228)))
#define    GPIO13            (*((volatile uint32_t *)(0x5000422C)))
#define    GPIO14            (*((volatile uint32_t *)(0x50004230)))
#define    GPIO15            (*((volatile uint32_t *)(0x50004234)))

#define    GPIO22            (*((volatile uint32_t *)(0x50004248)))
#define    GPIO23            (*((volatile uint32_t *)(0x5000424C)))
#define    GPIO24            (*((volatile uint32_t *)(0x50004250)))
#define    GPIO25            (*((volatile uint32_t *)(0x50004254)))
#define    GPIO26            (*((volatile uint32_t *)(0x50004258)))
#define    GPIO27            (*((volatile uint32_t *)(0x5000425C)))

#define    GPIO30            (*((volatile uint32_t *)(0x50004260)))
#define    GPIO31            (*((volatile uint32_t *)(0x50004264)))
#define    GPIO32            (*((volatile uint32_t *)(0x50004268)))
#define    GPIO34            (*((volatile uint32_t *)(0x50004270)))
#define    GPIO35            (*((volatile uint32_t *)(0x50004274)))
#define    GPIO36            (*((volatile uint32_t *)(0x50004278)))

#define    GPIO46            (*((volatile uint32_t *)(0x50004298)))
#define    GPIO47            (*((volatile uint32_t *)(0x5000429C)))

#define    GPIO50            (*((volatile uint32_t *)(0x500042A0)))
#define    GPIO51            (*((volatile uint32_t *)(0x500042A4)))
#define    GPIO52            (*((volatile uint32_t *)(0x500042A8)))
#define    GPIO53            (*((volatile uint32_t *)(0x500042AC)))
#define    GPIO54            (*((volatile uint32_t *)(0x500042B0)))
#define    GPIO55            (*((volatile uint32_t *)(0x500042B4)))

//------------------------------ I2C Controller ------------------------------
typedef struct
{
  volatile uint32_t  CON;
  volatile uint32_t  SADDR0;
  volatile uint32_t  DATA;
  volatile uint32_t  STATUS ;
  volatile uint32_t  DIV;
  volatile uint32_t  TOUT;
  volatile uint32_t  SADDR1;
  volatile uint32_t  SADDR2;
  volatile uint32_t  SADDR3;
  volatile uint32_t  SAMASK0;
  volatile uint32_t  SAMASK1;
  volatile uint32_t  SAMASK2;
  volatile uint32_t  SAMASK3;
} I2C_TypeDef;
#define    I2C               ((I2C_TypeDef *)(0x40020000))

//----------------------------- PWM Controller -------------------------------
typedef struct
{
  volatile uint32_t  PPR;
  volatile uint32_t  CSR;
  volatile uint32_t  PCR;
  volatile uint32_t  CNR0;
  volatile uint32_t  CNR1;
  volatile uint32_t  CNR2;
  volatile uint32_t  CNR3;
  volatile uint32_t  CNR4;
  volatile uint32_t  CNR5;
  volatile uint32_t  CMR0;
  volatile uint32_t  CMR1;
  volatile uint32_t  CMR2;
  volatile uint32_t  CMR3;
  volatile uint32_t  CMR4;
  volatile uint32_t  CMR5;
  volatile uint32_t  TDR0;
  volatile uint32_t  TDR1;
  volatile uint32_t  TDR2;
  volatile uint32_t  TDR3;
  volatile uint32_t  TDR4;
  volatile uint32_t  TDR5;
  volatile uint32_t  PIER;
  volatile uint32_t  PIIR;
  volatile uint32_t  POE;
  volatile uint32_t  PFBCON;
  volatile uint32_t  PDZIR;
  volatile uint32_t  TRGCON0 ;
  volatile uint32_t  TRGCON1 ;
  volatile uint32_t  TRGSTS0 ;
  volatile uint32_t  TRGSTS1 ;
  volatile uint32_t  PHCHG ;
  volatile uint32_t  PHCHGNXT ;
  volatile uint32_t  PHCHGMASK ;
  volatile uint32_t  INTACCUCTL ;
} PWM_TypeDef;
#define    PWM               ((PWM_TypeDef *)(0x40040000))

//------------------------- SPI Interface Controller -------------------------
typedef struct
{
  volatile uint32_t  CNTRL;
  volatile uint32_t  DIVIDER;
  volatile uint32_t  SSR;
           uint32_t  Res0;
  volatile uint32_t  RX0;
  volatile uint32_t  RX1;
           uint32_t  Res1;
           uint32_t  Res2;
  volatile uint32_t  TX0;
  volatile uint32_t  TX1;
           uint32_t  Res3;
           uint32_t  Res4;
           uint32_t  Res5;
  volatile uint32_t  VARCLK;
           uint32_t  Res6;
  volatile uint32_t  CNTRL2;
  volatile uint32_t  FIFO_STS_CLR;
   
} SPI_TypeDef;
#define    SPI               ((SPI_TypeDef *)(0x40030000))

//----------------------------- Timer Controller -----------------------------
typedef struct
{
  volatile  uint32_t  CSR;
  volatile  uint32_t  CMPR;
  volatile  uint32_t  ISR;
  volatile  uint32_t  DR;
  volatile  uint32_t  CAP;
  volatile  uint32_t  EXCON;
  volatile  uint32_t  EXISR;
} TIMER_TypeDef;
#define    TIMER0          ((TIMER_TypeDef *)(0x40010000))
#define    TIMER1   	     ((TIMER_TypeDef *)(0x40010020))

//------------------------- UART Interface Controller ------------------------
typedef struct
{
  union {
    volatile   uint32_t  RBR;
    volatile   uint32_t  THR;
  };
  volatile uint32_t  IER;
  volatile uint32_t  FCR;
  volatile uint32_t  LCR;
  volatile uint32_t  MCR;
  volatile uint32_t  MSR;
  volatile uint32_t  FSR;
  volatile uint32_t  ISR;
  volatile uint32_t  TOR;
  volatile uint32_t  BAUD;
  volatile uint32_t  IRCR;
  volatile uint32_t  ALT_CSR;
  volatile uint32_t  FUN_SEL;
} UART_TypeDef;
#define    UART0             ((UART_TypeDef *)(0x40050000))

//----------------------------- WDT Controller -----------------------------
typedef struct
{
  volatile uint32_t  CR ;
} WDT_TypeDef;
#define    WDT               ((WDT_TypeDef *)(0x40004000))

#if defined ( __CC_ARM   )
  #pragma no_anon_unions
#endif

//=======================================================================================
// ----------- BIT CONSTANT DEFINE USED FOR REGISTER---------------------------------
//=======================================================================================
//------------------- BIT CONSTANT for Analog Comparator---------------------------------
//// ACMP->CR0, ACMP->CR1, //////////////////////////////////////////////////////////////
#define  ACMP_CPP0_P15_0           0                      //PIN+输入PIN选择
#define  ACMP_CPP0_P10             0x20000000
#define  ACMP_CPP0_P12             0x40000000
#define  ACMP_CPP0_P13             0x60000000
#define  ACMP_FALL_TRG_TIM_EN      0x00000200              //下沿触发Timer使能
#define  ACMP_RISE_TRG_TIM_EN      0x00000100
#define  ACMP_CN_VREF              0x00000010              //负端接内部梯级电压
#define  ACMP_HYST_EN              0x00000004              //20mV 回差使能
#define  ACMP_IE                   0x00000002       
#define  ACMP_EN                   0x00000001

#define  ACMP_CPP1_P31_0           0                      //PIN+输入PIN选择
#define  ACMP_CPP1_P32             0x20000000
#define  ACMP_CPP1_P34             0x40000000
#define  ACMP_CPP1_P35             0x60000000  

//// ACMP->SR, /////////////////////////////////////////////////////////////////////////
#define  ACMP_OUT1                 0x00000008               //OUT输出位
#define  ACMP_OUT0                 0x00000004    
#define  ACMP_CF1                  0x40000002               //中断标志
#define  ACMP_CF0                  0x60000001

//------------------- BIT CONSTANT for ADC ----------------------------------------------
//// ADC->CR ////////////////////////////////////////////////////////////////////////////
#define  ADCR_START_BUSY           0x00000800              //软件触发ADC开始
#define  ADCR_TRG_EN               0x00000100              //PIN触发ADC使能
#define  ADCR_RISE_TRG             0x00000040              //PIN上沿触发
#define  ADCR_IE                   0x00000002              
#define  ADCR_EN                   0x00000001

//// ADC->SR ////////////////////////////////////////////////////////////////////////////
#define  ADSR_OVERRUN              0x00010000              //数据覆蓋
#define  ADSR_VALID                0x00000100              //数据可用
#define  ADSR_CHANNEL_MASK         0x00000070
#define  ADSR_BUSY                 0x00000008              //正在转换
#define  ADSR_CMPF1                0x00000004              //比较器1中断标志
#define  ADSR_CMPF0                0x00000002
#define  ADSR_ADF                  0x00000001              //ADC完成中断标志

//------------------- BIT CONSTANT for Clock --------------------------------------------
////CLK->PWRCON , 电源控制位 ////////////////////////////////////////////////////////////
#define  PWR_DOWN_32K_WORK_        0x00000200    //掉电后32768依然工作
#define  PWR_DOWN_EN_              0x00000080    //立即进入掉电状态,否则等_WFI()指令
#define  PWR_WAKEUP_F              0x00000040    //唤醒标志,写1清0
#define  PWR_WAKEUP_INT_EN_        0x00000020    //唤醒后中断使能
#define  PWR_WAKEUP_DELAY_         0x00000010    //唤醒后延时4096(外)或16(内)个时钟
#define  PWR_10K_ON_               0x00000008    //片内10K上电工作
#define  PWR_22M_ON_               0x00000004    //片内22.1184MHz起振
#define  PWR_EXT_IN_               0x00000003    //XTAL1做时钟输入，XTAL2-GPIO
#define  PWR_EXT_32768_            0x00000002    //32768起振
#define  PWR_EXT_HI_               0x00000001    //4M~24M起振
#define  PWR_EXT_OFF_0             0             //关闭外时钟,pin做GPIO用


////CLK->AHPCLK /////////////////////////////////////////////////////////////////////////
#define  CLK_ISP_EN                0x00000004    //ISP时钟使能,mini51的AHBCLK只有这一位

////CLK->APBCLK , 外设时钟使能位/////////////////////////////////////////////////////////
#define  CLK_WDT_EN                0x00000001    
#define  CLK_TMR0_EN               0x00000004
#define  CLK_TMR1_EN               0x00000008    //各外设时钟使能 , 定义中包含外设名字
#define  CLK_FDIV_EN               0x00000040
#define  CLK_I2C_EN                0x00000100         
#define  CLK_SPI_EN                0x00001000
#define  CLK_UART_EN               0x00010000
#define  CLK_PWM01_EN              0x00100000
#define  CLK_PWM23_EN              0x00200000
#define  CLK_PWM45_EN              0x00400000
#define  CLK_ADC_EN                0x10000000
#define  CLK_ACMP_EN               0x40000000

////CLK->CLKSTATUS , 时钟状态位 /////////////////////////////////////////////////////////
#define  CLKSTA_EXT_STABLE         0x00000001    //外部时钟已稳定
#define  CLKSTA_10K_STABLE         0x00000008    //10K时钟已稳定
#define  CLKSTA_22M_STABLE         0x00000010    //22M时钟已稳定
#define  CLKSTA_SWITCH_FAIL        0x00000080    //时钟切换失败

////CLK->CLKSEL0 , 时钟选择位 ///////////////////////////////////////////////////////////
#define  CLKS0_HCLK_EXT_0_         0             //HCLK选外接时钟
#define  CLKS0_HCLK_10K_           0x00000003    //HCLK选10K时钟
#define  CLKS0_HCLK_22M_           0x00000007    //HCLK选22.1184M时钟

#define  CLKS0_TICK_EXT_0_         0             //SysTick选外接时钟
#define  CLKS0_TICK_EXT_DIV2_      0x00000010    //SysTick选外接时钟的二分频
#define  CLKS0_TICK_HCLK_DIV2_     0x00000018    //SysTick选HCLK的二分频
#define  CLKS0_TICK_22M_DIV2_      0x00000038    //SysTick选22M的二分频

////CLK->CLKSEL1 , 时钟选择位 ///////////////////////////////////////////////////////////
#define  CLKS1_WDT_EXT_0_          0             //WDT 选外部时钟源
#define  CLKS1_WDT_HCLK2048_       0x00000002  
#define  CLKS1_WDT_10K_            0x00000003
#define  CLKS1_WDTCLK_MASK_        0x00000003

#define  CLKS1_SPI_12M_0           0
#define  CLKS1_SPI_22M             0x00000010

#define  CLKS1_ADC_EXT_0           0 
#define  CLKS1_ADC_HCLK            0x00000008
#define  CLKS1_ADC_22M             0x0000000C
#define  CLKS1_ADCCLK_MASK         0x0000000C

#define  CLKS1_TMR0_EXT_0          0             //外接晶体
#define  CLKS1_TMR0_10K            0x00000100    //片内10K
#define  CLKS1_TMR0_HCLK           0x00000200    //HCLK
#define  CLKS1_TMR0_CLK_PIN        0x00000300    //定时器时钟输入引脚
#define  CLKS1_TMR0_22M            0x00000700    //片内22M
#define  CLKS1_TMR0CLK_MASK        0x00000700

#define  CLKS1_TMR1_EXT_0          0 
#define  CLKS1_TMR1_10K            0x00001000
#define  CLKS1_TMR1_HCLK           0x00002000
#define  CLKS1_TMR1_CLK_PIN        0x00003000
#define  CLKS1_TMR1_22M            0x00007000
#define  CLKS1_TMR1CLK_MASK        0x00007000

#define  CLKS1_UART_EXT_0          0 
#define  CLKS1_UART_22M            0x02000000
#define  CLKS1_UARTCLK_MASK        0x02000000

#define  CLKS1_PWM01_EXT_0         0 
#define  CLKS1_PWM01_HCLK          0x20000000
#define  CLKS1_PWM01_22M           0x30000000
#define  CLKS1_PWM01CLK_MASK       0x30000000

#define  CLKS1_PWM23_EXT_0         0 
#define  CLKS1_PWM23_HCLK          0x80000000
#define  CLKS1_PWM23_22M           0xC0000000
#define  CLKS1_PWM23CLK_MASK       0xC0000000

////CLK->CLKSEL2 , 时钟选择位 ///////////////////////////////////////////////////////////
#define  CLKS2_FDIV_EXT_0          0 
#define  CLKS2_FDIV_HCLK           0x00000008
#define  CLKS2_FDIV_22M            0x0000000C
#define  CLKS2_FDIV_MASK           0x0000000C

#define  CLKS2_PWM45_EXT_0         0 
#define  CLKS2_PWM45_HCLK          0x00000020
#define  CLKS2_PWM45_22M           0x00000030
#define  CLKS2_PWM45_MASK          0x00000030

////CLK->CLKDIV /////////////////////////////////////////////////////////////////////////
#define  HCLK_DIV_4bit(x)          ((x##ul)-1)             //x的值 1~16
#define  UART_CLK_DIV_4bit(x)      (((x##ul)-1)<<8)        //x的值 1~16
#define  ADC_CLK_DIV_8bit(x)       (((x##ul)-1)<<16)       //x的值 1~256

////CLK->FRQDIV /////////////////////////////////////////////////////////////////////////
#define  FQRDIV_DIV_POWER(x)       ((x##ul)-1)             //分频系数:2的幂指数,1~16
#define  FQRDIV_EN                 0x00000010              //时钟分频输出使能
  
//------------------- BIT CONSTANT for FLASH Memory--------------------------------------
////FMC->ISPCON //////受保护寄存器///////////////////////////////////////////////////////
#define  FMC_EREASE_TIM_20ms_0     0
#define  FMC_EREASE_TIM_25ms       0x00001000
#define  FMC_EREASE_TIM_30ms       0x00002000
#define  FMC_EREASE_TIM_35ms       0x00003000
#define  FMC_EREASE_TIM_3ms        0x00004000
#define  FMC_EREASE_TIM_5ms        0x00005000
#define  FMC_EREASE_TIM_10ms       0x00006000
#define  FMC_EREASE_TIM_15ms       0x00007000

#define  FMC_PRG_TIM_40us_0        0
#define  FMC_PRG_TIM_45us          0x00000100
#define  FMC_PRG_TIM_50us          0x00000200
#define  FMC_PRG_TIM_55us          0x00000300
#define  FMC_PRG_TIM_20us          0x00000400
#define  FMC_PRG_TIM_25us          0x00000500
#define  FMC_PRG_TIM_30us          0x00000600
#define  FMC_PRG_TIM_35us          0x00000700

#define  FMC_SW_RST	               0x00000080              //软件复位
#define  FMC_FAIL                  0x00000040              //操作失败
#define  FMC_LDROM_WR_EN           0x00000020              //APROM代码可写LDROM
#define  FMC_CFG_WR_EN             0x00000010              //APROM代码可写config参数据
#define  FMC_APUEN_                0x00000008
#define  FMC_BOOT_LDROM            0x00000002              //从LDROM起动
#define  FMC_EN_                   0x00000001

////FMC->ISPCMD /////////////////////////////////////////////////////////////////////////
#define  FMC_READ_0                0
#define  FMC_WRITE                 0x00000021
#define  FMC_ERASE                 0x00000022

#define  FMC_RD_CID                0x0000000B
#define  FMC_RD_DID                0x0000000C
#define  FMC_RD_UID                0x00000004

////FMC->ISPGO /////////////////////////////////////////////////////////////////////////
#define  FMC_GO_BUSY_              0x00000001	//此寄存器仅此一位,受保护位

//------------------- BIT CONSTANT for Global Controller --------------------------------
////GCR->RST_SRC /////////////////////////////////////////////////////////////////////////
#define  RST_SRC_CPU               0x00000080 
#define  RST_SRC_MCU               0x00000020
#define  RST_SRC_BOD               0x00000010
#define  RST_SRC_WTD               0x00000004
#define  RST_SRC_PIN               0x00000002
#define  RST_SRC_POR               0x00000001 

////GCR->IPRST_CTL1 //////////////////////////////////////////////////////////////////////
#define  RST_CPU                   0x00000002 
#define  RST_CHIP                  0x00000001

////GCR->IPRST_CTL2 //////////////////////////////////////////////////////////////////////
#define  RST_ADC                   (1<<28) 
#define  RST_ACMP                  (1<<22)
#define  RST_PWM                   (1<<20) 
#define  RST_UART                  (1<<16)
#define  RST_SPI                   (1<<12) 
#define  RST_IIC                   (1<<8)
#define  RST_T1                    0x00000008
#define  RST_T0                    0x00000004 
#define  RST_GPIO                  0x00000002

////GCR->BODCTL //////////////////////////////////////////////////////////////////////
#define  BOD_OUT_VOL_LOW           0x00000004 
#define  BOD_IF                    0x00000010
#define  BOD_RST_EN                0x00000008 
#define  BOD_DIS                   0x00000006
#define  BOD_V38                   0x00000004
#define  BOD_V27                   0x00000002

////GCR->RegLockAddr /////////////////////////////////////////////////////////////////
#define  REG_UNLOCK                1
 
//------------------- BIT CONSTANT for GPIO ---------------------------------------------
////GPIO0->PMD,GPIO1->PMD,GPIO2->PMD,GPIO3->PMD,GPIO4->PMD,GPIO5->PMD ///////////////////
#define  Px0_IN_0                  0 
#define  Px0_OUT                   0x00000001
#define  Px0_OD                    0x00000002
#define  Px0_QB                    0x00000003
#define  Px0_PMD_MASK              0x00000003 

#define  Px1_IN_0                  0 
#define  Px1_OUT                   0x00000004
#define  Px1_OD                    0x00000008
#define  Px1_QB                    0x0000000C
#define  Px1_PMD_MASK              0x0000000C

#define  Px2_IN_0                  0 
#define  Px2_OUT                   0x00000010
#define  Px2_OD                    0x00000020
#define  Px2_QB                    0x00000030
#define  Px2_PMD_MASK              0x00000030

#define  Px3_IN_0                  0
#define  Px3_OUT                   0x00000040
#define  Px3_OD                    0x00000080
#define  Px3_QB                    0x000000C0
#define  Px3_PMD_MASK              0x000000C0

#define  Px4_IN_0                  0
#define  Px4_OUT                   0x00000100
#define  Px4_OD                    0x00000200
#define  Px4_QB                    0x00000300
#define  Px4_PMD_MASK              0x00000300

#define  Px5_IN_0                  0
#define  Px5_OUT                   0x00000400
#define  Px5_OD                    0x00000800
#define  Px5_QB                    0x00000C00
#define  Px5_PMD_MASK              0x00000C00

#define  Px6_IN_0                  0 
#define  Px6_OUT                   0x00001000
#define  Px6_OD                    0x00002000
#define  Px6_QB                    0x00003000
#define  Px6_PMD_MASK              0x00003000

#define  Px7_IN_0                  0 
#define  Px7_OUT                   0x00004000
#define  Px7_OD                    0x00008000
#define  Px7_QB                    0x0000C000
#define  Px7_PMD_MASK              0x0000C000
//------------------- BIT CONSTANT for I2C ----------------------------------------------
////I2C->CON ////////////////////////////////////////////////////////////////////////////
#define  I2C_IE                    0x00000080
#define  I2C_EN				             0x00000040
#define  I2C_STA				           0x00000020
#define  I2C_STO				           0x00000010 
#define  I2C_SI					           0x00000008 
#define  I2C_AA					           0x00000004
////I2C->TOUT ///////////////////////////////////////////////////////////////////////////
#define  I2C_TO_EN                 0x00000004
#define  I2C_TO_DIV4               0x00000002              //时钟四分频
#define  I2C_TO_IF				         0x00000001

////I2C->SADDR0, I2C->SADDR1, I2C->SADDR2, I2C->SADDR3, ////////////////////////////////
#define  I2C_ADRESS_7bit(x)        ((x##ul)<<1)
#define  I2C_GC                    0x00000001			   //广播使能

//------------------- BIT CONSTANT for PWM ----------------------------------------------
//// PWM->PPR //////////////////////////////////////////////////////////////////////////
#define  PPR_DIV45(x)              ((x##ul-1)<<16)
#define  PPR_DIV23(x)              ((x##ul-1)<<8)
#define  PPR_DIV01(x)              (x##ul-1)

//// PWM->CSR //////////////////////////////////////////////////////////////////////////
#define  CSR_DIV1_CH(CHANNEL)         (4ul<<(4*(CHANNEL)))
#define  CSR_DIV2(CHANNEL)         (0ul<<(4*(CHANNEL)))
#define  CSR_DIV4(CHANNEL)         (1ul<<(4*(CHANNEL)))
#define  CSR_DIV8(CHANNEL)         (2ul<<(4*(CHANNEL)))
#define  CSR_DIV16(CHANNEL)        (3ul<<(4*(CHANNEL)))

//// PWM->PCR //////////////////////////////////////////////////////////////////////////
#define  PCR_CENTER_TYPE           0x80000000
#define  PCR_GROUP_MODE            0x40000000
#define  PCR_INDEP_0               0
#define  PCR_COMPLE                0x10000000
#define  PCR_SYHC                  0x20000000
#define  PCR_CLR_COUNTER           0x08000000
#define  PCR_DZEN45_EN             0x04000000
#define  PCR_DZEN23_EN             0x02000000
#define  PCR_DZEN01_EN             0x01000000
#define  PCR_PERIOD_MODE(CHANNEL)  (8ul<<(4*(CHANNEL))) 
#define  PCR_INV_EN(CHANNEL)       (4ul<<(4*(CHANNEL))) 
#define  PCR_CH_EN(CHANNEL)        (1ul<<(4*(CHANNEL)))
#define  PCR_DEBUG_MODE            0x00000002

//// PWM->PIER //////////////////////////////////////////////////////////////////////////
#define  PIER_CENTER_INT           0x00020000
#define  PIER_BRKE_IE              0x00010000
#define  PIER_DIE(CHANNEL)         (1<<((CHANNEL)+8))
#define  PIER_PIE(CHANNEL)         (1<<(CHANNEL))

//// PWM->PIIR //////////////////////////////////////////////////////////////////////////
#define  PIIR_BKF1                 0x00020000
#define  PIIR_BKF0                 0x00010000
#define  PIIR_DIF(CHANNEL)         (1<<((CHANNEL)+8))
#define  PIIR_PIF(CHANNEL)         (1<<(CHANNEL))

//// PWM->POE //////////////////////////////////////////////////////////////////////////
#define  POE_EN(CHANNEL)         (1<<(CHANNEL))

//// PWM->PFBCON ///////////////////////////////////////////////////////////////////////
#define  BRK_OUT_HIGH(CHANNEL)     (1<<((CHANNEL)+24))
#define  BRK_OUT_FAULT             0x00000080
#define  BRK_SOURCE_CPO0           0x00000004
#define  BRK1_EN                   0x00000002
#define  BRK0_EN                   0x00000001

//// PWM->PDZIR ///////////////////////////////////////////////////////////////////////
#define  DZEN45_TIM(x)             ((x##UL)<<16)
#define  DZEN23_TIM(x)             ((x##UL)<<8)
#define  DZEN01_TIM(x)             (x##UL)

//// PWM->TRGCON0  ////// 只适用于 PWM0123, CHANNEL的取值为0,1,2,3 ////////////////////
#define  TRG_AD_BOTTOM(CHANNEL)    (1<<(((CHANNEL)*8)+3))
#define  TRG_AD_DOWN(CHANNEL)      (1<<(((CHANNEL)*8)+2))
#define  TRG_AD_TOP(CHANNEL)       (1<<(((CHANNEL)*8)+1))
#define  TRG_AD_UP(CHANNEL)        (1<<((CHANNEL)*8))

//// PWM->TRGCON1  ///// 只适用于 PWM45,即CHANNEL的值只能取4或5 ///////////////////////
#define  TRG45_AD_BOTTOM(CHANNEL)  (1<<((((CHANNEL)-4)*8)+3))
#define  TRG45_AD_DOWN(CHANNEL)    (1<<((((CHANNEL)-4)*8)+2))
#define  TRG45_AD_TOP(CHANNEL)     (1<<((((CHANNEL)-4)*8)+1))
#define  TRG45_AD_UP(CHANNEL)      (1<<(((CHANNEL)-4)*8))

//// PWM->PHCHG, PWM->PHCHGNXT ////////////////////////////////////////////////////////
#define  CMP0_CAP_TIM_EN           0x80000000
#define  T0_TRG_PWM_EN             0x40000000  // Enable "PWM->PHCHGNXT write into PWM->PHCHG" when Timer0 expire. 
#define  CMP0_PIN_P13              0x30000000  // P13 is selected as CMPP0 pin. 
#define  CMP0_PIN_P12              0x20000000  // P12 is selected as CMPP0 pin. 
#define  CMP0_PIN_P10              0x10000000  // P10 is selected as CMPP0 pin. 
#define  CMP0_PIN_P15_0            0
#define  CMP0_OFF_PWM3             0x08000000
#define  CMP0_OFF_PWM2             0x04000000
#define  CMP0_OFF_PWM1             0x02000000
#define  CMP0_OFF_PWM0             0x01000000

#define  CMP1_CAP_TIM_EN           0x00800000
#define  T1_TRG_PWM_EN             0x00400000 // Enable "PWM->PHCHGNXT write into PWM->PHCHG" when Timer1 expire. 
#define  CMP1_PIN_P34              0x00300000
#define  CMP1_PIN_P33              0x00200000
#define  CMP1_PIN_P32              0x00100000
#define  CMP1_PIN_P31_0            0
#define  CMP1_OFF_PWM3             0x00080000
#define  CMP1_OFF_PWM2             0x00040000
#define  CMP1_OFF_PWM1             0x00020000
#define  CMP1_OFF_PWM0             0x00010000

#define  CMP1_MULT_CAP             0x00008000              //ACMP1 多次捕获
#define  CMP0_MULT_CAP             0x00004000
#define  PIN_OUT_PWM5              0x00002000
#define  PIN_OUT_PWM4              0x00001000
#define  PIN_OUT_PWM3              0x00000800
#define  PIN_OUT_PWM2              0x00000400
#define  PIN_OUT_PWM1              0x00000200
#define  PIN_OUT_PWM0              0x00000100  

//// PWM->PHCHGMASK /////////////////////////////////////////////////////////////////////
#define  PHCHG_CTL_CMP1            0x00000200
#define  PHCHG_CTL_CMP0            0x00000100
#define  P00_OUT_D7                0x00000080
#define  P01_OUT_D6                0x00000040

//// PWM->INTACCUCTL /////////////////////////////////////////////////////////////////////
#define  PWM0_IF_CNT_EN            0x00000001
#define  PWM0IF_CNT(x)             ((x##ul)<<4)

//------------------- BIT CONSTANT for SPI ----------------------------------------------
//// SPI->CNTRL /////////////////////////////////////////////////////////////////////////
#define  SPI_TX_FULL               (1<<27)
#define  SPI_TX_EMPTY              (1<<26)
#define  SPI_RX_FULL               (1<<25)
#define  SPI_RX_EMPTY              (1<<24)
#define  SPI_FIFO_EN               (1<<21)
#define  SPI_RECORD_EN             (1<<19)
#define  SPI_SLAVE_MODE            (1<<18)
#define  SPI_IE                    0x00020000
#define  SPI_IF                    0x00010000
#define  SPI_INTERVAL_4bit(x)      ((x##ul)<<12)
#define  SPI_CLK_NEGTIVE           (1<<11)       //CLK为负脉冲,空闲时为高,此位为0，CLK正脉冲
#define  SPI_LSB                   (1<<10)       //低位在前, 否则高位在前
#define  SPI_BIT_LEN_5bit(x)       ((x##ul)<<3)  //Range from 8 to 32 
#define  SPI_CLK_FALL_TX           4
#define  SPI_CLK_FALL_RX           2
#define  SPI_TX_GO_BUSY            1

//// SPI->SSR ///////////////////////////////////////////////////////////////////////////
#define  SPI_TRANSFER_FINISH       0x00000020    //仅从模式, 一次传输完成
#define  SPI_SS_LEVEL_TRG          0x00000010    //仅从模式，此位为0,SS边沿有较
#define  SPI_SS_AUTO_EN            0x00000008    //位0=1且在不发送数据时,置SS无效, 仅主模式
#define  SPI_SS_HIGH               0x00000004    //从选高或上沿有效,否则低或下沿有效, Only master
#define  SPI_SS_ASSERT             0x00000001    //SS有效,否则无效, Only master,

//// SPI->CNTRL2 /////////////////////////////////////////////////////////////////////////
#define  SPI_SS_DISASSERT_IF       (1<<16)       //仅从模式, 一次传输完成
#define  SPI_SS_LEVEL_TRG          0x00000010    //仅从模式，此位为0,SS边沿有较
#define  SPI_SS_AUTO_EN            0x00000008    //位0=1且在不发送数据时,置SS无效, 仅主模式
#define  SPI_SS_HIGH               0x00000004    //从选高或上沿有效,否则低或下沿有效, Only master
#define  SPI_SS_ASSERT             0x00000001    //SS有效,否则无效, Only master,

//------------------- BIT CONSTANT for TIMER --------------------------------------------
//// TIMER0->CSR, TIMER1->CSR////////////////////////////////////////////////////////////
#define  T_DEBUG_WORK              0x80000000
#define  T_CEN                     0x40000000
#define  T_IE                      0x20000000

#define  T_MODE_MASK               0x18000000
#define  T_MODE_ONCE               0x00000000
#define  T_MODE_PERIODIC           0x08000000
#define  T_MODE_TOGGLE             0x10000000
#define  T_MODE_CONTINUOUS		     0x18000000

#define  T_CRST                    0x04000000
#define  T_CNT_MODE                0x01000000
#define  T_WAKE_EN                 0x00800000
#define  T_CAP_ACMP                0x00080000
#define  T_TOGGLE_TEX              0x00040000
#define  T_WR_TCMP_NO_RST          0x00020000
#define  T_TDR_EN                  0x00010000
                                                 //位[15:8]没用,位[7:1]为预分频系数
//// TIMER0->ISR, TIMER1->ISR ///////////////////////////////////////////////////////
#define  T_WAKUP_F                 0x000000002
#define  T_IF                      0x000000001   //使能中断此标志才会置1

//// TIMER0->EXCON, TIMER1->EXCON ///////////////////////////////////////////////////////
#define  TEX_TRIG_COUNT            0x00000100
#define  T_CNT_DEBO_EN             0x00000080
#define  TEX_DEBO_EN               0x00000040
#define  TEX_IE                    0x00000020
#define  TEX_CAP_RST_MODE          0x00000010

#define  TEX_EN                    0x00000008
#define  TEX_CAP_FALL_RISE         0x00000004  
#define  TEX_CAP_RISE              0x00000002 
#define  TEX_CAP_FALL_0            0x00000000

#define  T_CNT_RISE                0x00000001

//// TIMER0->EXISR, TIMER1->EXISR ///////////////////////////////////////////////////////
#define  TEX_IF                    0x00000001

//------------------- BIT CONSTANT for UART ---------------------------------------------
//// UART0->IER /////////////////////////////////////////////////////////////////////////
#define  CTS_EN                    (1<<13)
#define  RTS_EN                    (1<<12)
#define  TO_EN                     (1<<11) 
#define  CTS_WAKEUP_EN             0x00000040
#define  BUF_ERR_IE                0x00000020
#define  RX_TO_IE                  0x00000010
#define  MODEM_IE                  0x00000008
#define  RX_LINE_IE                0x00000004
#define  TX_EMPTY_IE               0x00000002
#define  RX_DATA_IE                0x00000001

//// UART0->FCR /////////////////////////////////////////////////////////////////////////
#define  RTS_1BYTE_TRG_0           0
#define  RTS_4BYTE_TRG             0x00010000
#define  RTS_8BYTE_TRG             0x00020000
#define  RTS_14BYTE_TRG            0x00030000
#define  RX_DIS                    0x00000100
#define  RX_FIFO_1BYTE_INT_0       0
#define  RX_FIFO_4BYTE_INT         0x00000010
#define  RX_FIFO_8BYTE_INT         0x00000020
#define  RX_FIFO_14BYTE_INT        0x00000030
#define  TX_RESET                  0x00000004
#define  RX_RESET                  0x00000002

//// UART0->LCR /////////////////////////////////////////////////////////////////////////
#define  BREAK_ONE_FRAME           0x00000040
#define  RTS_4BYTE_TRG             0x00010000
#define  RTS_8BYTE_TRG             0x00020000
#define  RTS_14BYTE_TRG            0x00030000
#define  PARITY_STICK_EN           0x00000020
#define  PARITY_EVEN_EN            0x00000010
#define  PARITY_EN                 0x00000008
#define  STOP_2BIT                 0x00000004
#define  WORD_LEN_8BIT             0x00000003
#define  WORD_LEN_7BIT             0x00000002
#define  WORD_LEN_6BIT             0x00000001
#define  WORD_LEN_5BIT_0           0

//// UART0->MCR /////////////////////////////////////////////////////////////////////////
#define  RTS_PIN                   0x00002000              //RTS 引脚输出值
#define  RTS_POSITIVE              0x00000200              //RTS输出不反相
#define  RTS_VALUE                 0x00000002              //RTS的值

//// UART0->MSR /////////////////////////////////////////////////////////////////////////
#define  CTS_HIGH_TRG              0x00000100
#define  CTS_PIN                   0x00800010
#define  CTS_CHANGE                0x00000001

//// UART0->FSR /////////////////////////////////////////////////////////////////////////
#define  TX_EMPTY                  0x10000000
#define  TX_FIFO_OVER              0x01000000
#define  TX_FIFO_FULL              0x00800000
#define  TX_FIFO_EMPTY             0x00400000
#define  TX_FIFO_NUM_MASK          0x003F0000
#define  RX_FIFO_FULL              0x00008000
#define  RX_FIFO_EMPTY             0x00004000
#define  RX_FIFO_NUM_MASK          0x00003F00
#define  RX_BREAK_IF               0x00000040
#define  RX_FRAME_ERR              0x00000020
#define  RX_PARITY_ERR             0x00000010
#define  RX_ADRESS_MATCH           0x00000008
#define  RX_FIFO_OVER              0x00000001

//// UART0->ISR /////////////////////////////////////////////////////////////////////////
#define  BUF_ERR_IF                0x00000020
#define  TO_IF                     0x00000010
#define  MODEM_IF                  0x00000008
#define  RX_LINE_IF                0x00000004
#define  TX_EMPTY_IF               0x00000002
#define  RX_DATA_IF                0x00000001

//// UART0->BAUD /////////////////////////////////////////////////////////////////////////
#define  DIV_X_EN                  0x20000000
#define  DIV_X_ONE                 0x10000000
#define  DIV_X(x)                  ((x##ul)<<24)

//// UART0->IRCR /////////////////////////////////////////////////////////////////////////
#define  IRCR_RX_INV               0x00000040
#define  IRCR_TX_INV               0x00000020
#define  IRCR_MODE_EN              0x00000002

//// UART0->ALT_CSR //////////////////////////////////////////////////////////////////////
#define  M485_ADDR_8bit(x)         ((x##ul)<<24)
#define  M485_ADDR_EN              0x00008000
#define  M485_AUTO_DIR             0x00000400
#define  M485_DETEC_ADDR           0x00000200
#define  M485_NORMAL_MODE          0x00000100

//// UART0->FUN_SEL //////////////////////////////////////////////////////////////////////
#define  SIO_MODE_UART_0           0
#define  SIO_MODE_IR               0x00000002
#define  SIO_MODE_485              0x00000003
#define  SIO_MODE_MASK             0x00000003

//------------------- BIT CONSTANT for WDT ----------------------------------------------
#define  WTD_DEBUG_EN_             (1<<31)
#define  WTD_TIM_3bit_(x)          ((x##ul)<<8) 
#define  WTD_EN_                   0x80
#define  WTD_IE_                   0x40
#define  WTD_WKUP_F_               0x20
#define  WTD_WKUP_EN_              0x10
#define  WTD_IF_                   0x08
#define  WTD_RST_F_                0x04
#define  WTD_RST_EN_               0x02
#define  WTD_CLEAR_                0x01

#ifdef __cplusplus
  }
#endif

#endif  // __MINI51_H__
