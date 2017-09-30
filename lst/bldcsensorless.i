#line 1 "User\\BLDCSensorLess.c"









 
#line 1 "User\\BLDCSensorLess.h"



#line 1 "User\\global.h"



#line 1 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"
 










 






































 















 

 
 
 



 



 
typedef enum IRQn {
     

    NonMaskableInt_IRQn   = -14,     
    HardFault_IRQn        = -13,     
    SVCall_IRQn           = -5,      
    PendSV_IRQn           = -2,      
    SysTick_IRQn          = -1,      

     

    BOD_IRQn              = 0,       
    WDT_IRQn              = 1,       
    EINT0_IRQn            = 2,       
    EINT1_IRQn            = 3,       
    GPIO01_IRQn           = 4,       
    GPIO234_IRQn          = 5,       
    PWM_IRQn              = 6,       
    FB_IRQn               = 7,       
    TMR0_IRQn             = 8,       
    TMR1_IRQn             = 9,       
    UART_IRQn             = 12,      
    SPI_IRQn              = 14,      
    GPIO5_IRQn            = 16,      
    HIRC_IRQn             = 17,      
    I2C_IRQn              = 18,      
    ACMP_IRQn             = 25,      
    PDWU_IRQn             = 28,      
    ADC_IRQn              = 29       

} IRQn_Type;






 


 






   


#line 1 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\5.1.1\\CMSIS\\Include\\core_cm0.h"
 




 
















 










#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 35 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\5.1.1\\CMSIS\\Include\\core_cm0.h"

















 




 



 

#line 1 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\5.1.1\\CMSIS\\Include\\cmsis_version.h"
 




 
















 










 
#line 64 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\5.1.1\\CMSIS\\Include\\core_cm0.h"
 
 









 







#line 114 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\5.1.1\\CMSIS\\Include\\core_cm0.h"

#line 1 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\5.1.1\\CMSIS\\Include\\cmsis_compiler.h"
 




 
















 




#line 29 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\5.1.1\\CMSIS\\Include\\cmsis_compiler.h"



 
#line 1 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\5.1.1\\CMSIS\\Include\\cmsis_armcc.h"
 




 
















 









 













   
   


 
#line 100 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\5.1.1\\CMSIS\\Include\\cmsis_armcc.h"

 



 





 
 






 
 





 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}






 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}






 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}






 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}






 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}






 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}






 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}






 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}






 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}






 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}






 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}


#line 335 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\5.1.1\\CMSIS\\Include\\cmsis_armcc.h"


#line 373 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\5.1.1\\CMSIS\\Include\\cmsis_armcc.h"



 


 



 




 






 







 






 








 










 










 











 








 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}









 









 








 




__attribute__((always_inline)) static __inline uint32_t __RBIT(uint32_t value)
{
  uint32_t result;
  int32_t s = (4   * 8) - 1;  

  result = value;                       
  for (value >>= 1U; value; value >>= 1U)
  {
    result <<= 1U;
    result |= value & 1U;
    s--;
  }
  result <<= s;                         
  return(result);
}








 



#line 729 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\5.1.1\\CMSIS\\Include\\cmsis_armcc.h"

   


 



 

#line 811 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\5.1.1\\CMSIS\\Include\\cmsis_armcc.h"
 


#line 35 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\5.1.1\\CMSIS\\Include\\cmsis_compiler.h"




 
#line 350 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\5.1.1\\CMSIS\\Include\\cmsis_compiler.h"




#line 116 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\5.1.1\\CMSIS\\Include\\core_cm0.h"

















 
#line 150 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\5.1.1\\CMSIS\\Include\\core_cm0.h"

 






 
#line 166 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\5.1.1\\CMSIS\\Include\\core_cm0.h"

 




 










 



 






 



 
typedef union
{
  struct
  {
    uint32_t _reserved0:28;               
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;

 















 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;

 






 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:15;               
    uint32_t T:1;                         
    uint32_t _reserved1:3;                
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;

 





















 
typedef union
{
  struct
  {
    uint32_t _reserved0:1;                
    uint32_t SPSEL:1;                     
    uint32_t _reserved1:30;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 



 







 



 
typedef struct
{
  volatile uint32_t ISER[1U];                
        uint32_t RESERVED0[31U];
  volatile uint32_t ICER[1U];                
        uint32_t RSERVED1[31U];
  volatile uint32_t ISPR[1U];                
        uint32_t RESERVED2[31U];
  volatile uint32_t ICPR[1U];                
        uint32_t RESERVED3[31U];
        uint32_t RESERVED4[64U];
  volatile uint32_t IP[8U];                  
}  NVIC_Type;

 







 



 
typedef struct
{
  volatile const  uint32_t CPUID;                   
  volatile uint32_t ICSR;                    
        uint32_t RESERVED0;
  volatile uint32_t AIRCR;                   
  volatile uint32_t SCR;                     
  volatile uint32_t CCR;                     
        uint32_t RESERVED1;
  volatile uint32_t SHP[2U];                 
  volatile uint32_t SHCSR;                   
} SCB_Type;

 















 



























 















 









 






 



 







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t LOAD;                    
  volatile uint32_t VAL;                     
  volatile const  uint32_t CALIB;                   
} SysTick_Type;

 












 



 



 









 








 
 







 






 







 


 







 

 










 









 


 



 





 

#line 575 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\5.1.1\\CMSIS\\Include\\core_cm0.h"
 
 
#line 583 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\5.1.1\\CMSIS\\Include\\core_cm0.h"
 





#line 598 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\5.1.1\\CMSIS\\Include\\core_cm0.h"




 
 










 
static __inline void __NVIC_EnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
  }
}









 
static __inline uint32_t __NVIC_GetEnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0U] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}







 
static __inline void __NVIC_DisableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
    do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);
  }
}









 
static __inline uint32_t __NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0U] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}







 
static __inline void __NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
  }
}







 
static __inline void __NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0U] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
  }
}










 
static __inline void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )]  = ((uint32_t)(((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )]  & ~(0xFFUL << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL))) |
       (((priority << (8U - 2)) & (uint32_t)0xFFUL) << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL)));
  }
  else
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] = ((uint32_t)(((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] & ~(0xFFUL << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL))) |
       (((priority << (8U - 2)) & (uint32_t)0xFFUL) << ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL)));
  }
}










 
static __inline uint32_t __NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( (((uint32_t)(int32_t)(IRQn)) >> 2UL) )] >> ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL) ) & (uint32_t)0xFFUL) >> (8U - 2)));
  }
  else
  {
    return((uint32_t)(((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( (((((uint32_t)(int32_t)(IRQn)) & 0x0FUL)-8UL) >> 2UL) )] >> ( ((((uint32_t)(int32_t)(IRQn)) ) & 0x03UL) * 8UL) ) & (uint32_t)0xFFUL) >> (8U - 2)));
  }
}










 
static __inline void __NVIC_SetVector(IRQn_Type IRQn, uint32_t vector)
{
  uint32_t *vectors = (uint32_t *)0x0U;
  vectors[(int32_t)IRQn + 16] = vector;
}









 
static __inline uint32_t __NVIC_GetVector(IRQn_Type IRQn)
{
  uint32_t *vectors = (uint32_t *)0x0U;
  return vectors[(int32_t)IRQn + 16];
}





 
static __inline void __NVIC_SystemReset(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                          
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FAUL << 16U) |
                 (1UL << 2U));
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                           

  for(;;)                                                            
  {
    __nop();
  }
}

 


 





 








 
static __inline uint32_t SCB_GetFPUType(void)
{
    return 0U;            
}


 



 





 













 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);                                                    
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)(ticks - 1UL);                          
  __NVIC_SetPriority (SysTick_IRQn, (1UL << 2) - 1UL);  
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                              
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2U) |
                   (1UL << 1U)   |
                   (1UL );                          
  return (0UL);                                                      
}



 










#line 131 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\system_Mini51Series.h"
 








 









 
 
 



 

#line 34 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\system_Mini51Series.h"


extern uint32_t SystemCoreClock;         
extern uint32_t CyclesPerUs;             









 

extern void SystemCoreClockUpdate (void);








 
#line 132 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"
#line 133 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"

 
 
 



 


#pragma anon_unions




 



 


typedef struct {

    
































































 

    volatile uint32_t CMPCR[2];       
    volatile uint32_t CMPSR;          
    volatile uint32_t CMPRVCR;        

} ACMP_T;






 








































   
   


 



 


typedef struct {
    

















































































































































































 

    volatile const  uint32_t ADDR;           
    
    volatile const  uint32_t RESERVE0[7];
    
    volatile uint32_t ADCR;           
    volatile uint32_t ADCHER;         
    volatile uint32_t ADCMPR[2];      
    volatile uint32_t ADSR;           
    
    volatile const  uint32_t RESERVE1[4];
    
    volatile uint32_t ADTDCR;         
    volatile uint32_t ADSAMP;         

} ADC_T;






 







































































































   
   


 



 


typedef struct {

    







































































































































































































































 

    volatile uint32_t PWRCON;         
    volatile uint32_t AHBCLK;         
    volatile uint32_t APBCLK;         
    volatile uint32_t CLKSTATUS;      
    volatile uint32_t CLKSEL0;        
    volatile uint32_t CLKSEL1;        
    volatile uint32_t CLKDIV;         
    volatile uint32_t CLKSEL2;        
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile uint32_t FRQDIV;         

} CLK_T;






 




#line 871 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"

#line 878 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"























































#line 939 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"

#line 946 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"

#line 953 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"

















































   
   


 



 


typedef struct {

    




































































































 

    volatile uint32_t ISPCON;         
    volatile uint32_t ISPADR;         
    volatile uint32_t ISPDAT;         
    volatile uint32_t ISPCMD;         
    volatile uint32_t ISPTRG;         
    volatile const  uint32_t DFBADR;         
    
    volatile const  uint32_t RESERVE0[10];
    
    volatile const  uint32_t ISPSTA;         

} FMC_T;






 














































   
   


 



 


typedef struct {

    











































































































































































































































































































 

    volatile const  uint32_t PDID;           
    volatile uint32_t RSTSRC;         
    volatile uint32_t IPRSTC1;        
    volatile uint32_t IPRSTC2;        
    
    volatile const  uint32_t RESERVE0[2];
    
    volatile uint32_t BODCTL;          
    
    volatile const  uint32_t RESERVE1[5];
    
    volatile uint32_t P0_MFP;         
    volatile uint32_t P1_MFP;         
    volatile uint32_t P2_MFP;         
    volatile uint32_t P3_MFP;         
    volatile uint32_t P4_MFP;         
    volatile uint32_t P5_MFP;         
    
    volatile const  uint32_t RESERVE2[14];
    
    volatile uint32_t IRCTRIMCTL;     
    volatile uint32_t IRCTRIMIER;     
    volatile uint32_t IRCTRIMISR;     
    
    volatile uint32_t RESERVE3[29];
    
    volatile uint32_t RegLockAddr;    

} GCR_T;






 




















































































































































































































































   
   


 



 


typedef struct {

    


















































































































































































































































































 
    volatile uint32_t PMD;            
    volatile uint32_t OFFD;           
    volatile uint32_t DOUT;           
    volatile uint32_t DMASK;          
    volatile const  uint32_t PIN;            
    volatile uint32_t DBEN;           
    volatile uint32_t IMD;            
    volatile uint32_t IEN;            
    volatile uint32_t ISRC;           
} GPIO_T;



 
typedef struct {
    





























 
    volatile uint32_t DBNCECON;       
} GPIO_DBNCECON_T;






 





























































   
   


 



 


typedef struct {

    









































































































































































































 

    volatile uint32_t I2CON;          
    volatile uint32_t I2CADDR0;       
    volatile uint32_t I2CDAT;         
    volatile const  uint32_t I2CSTATUS;      
    volatile uint32_t I2CLK;          
    volatile uint32_t I2CTOC;         
    volatile uint32_t I2CADDR1;       
    volatile uint32_t I2CADDR2;       
    volatile uint32_t I2CADDR3;       
    volatile uint32_t I2CADM0;        
    volatile uint32_t I2CADM1;        
    volatile uint32_t I2CADM2;        
    volatile uint32_t I2CADM3;        
    
    volatile const  uint32_t RESERVE0[2];
    
    volatile uint32_t I2CON2;         
    volatile uint32_t I2CSTATUS2;     

} I2C_T;






 












































































   
   


 



 



typedef struct {

    





















































































































































 


    volatile const  uint32_t SRC0;            
    volatile const  uint32_t SRC1;            
    volatile const  uint32_t SRC2;            
    volatile const  uint32_t SRC3;            
    volatile const  uint32_t SRC4;            
    volatile const  uint32_t SRC5;            
    volatile const  uint32_t SRC6;            
    volatile const  uint32_t SRC7;            
    volatile const  uint32_t SRC8;            
    volatile const  uint32_t SRC9;            
    
    volatile const  uint32_t RESERVED0[2];
    
    volatile const  uint32_t SRC12;            
    
    volatile const  uint32_t RESERVED1;
    
    volatile const  uint32_t SRC14;            
    
    volatile const  uint32_t RESERVED2;
    
    volatile const  uint32_t SRC16;            
    volatile const  uint32_t SRC17;            
    volatile const  uint32_t SRC18;            
    
    volatile const  uint32_t RESERVED3[6];
    
    volatile const  uint32_t SRC25;            
    
    volatile const  uint32_t RESERVED4[2];
    
    volatile const  uint32_t SRC28;            
    volatile const  uint32_t SRC29;            
    
    volatile const  uint32_t RESERVED5[2];
    
    volatile uint32_t NMICON;           
    volatile uint32_t MCUIRQ;           

} INT_T;






 













   
   


 



 


typedef struct {

    






























































































































































































































































































































































































































































































































































































































































































































































































































 

    volatile uint32_t PPR;            
    volatile uint32_t CSR;            
    volatile uint32_t PCR;            
    volatile uint32_t CNR[6];         
    volatile uint32_t CMR[6];         
    
    volatile const  uint32_t RESERVE0[6];
    
    volatile uint32_t PIER;           
    volatile uint32_t PIIR;           
    volatile uint32_t POE;            
    volatile uint32_t PFBCON;         
    volatile uint32_t PDZIR;          
    volatile uint32_t TRGCON0;        
    volatile uint32_t TRGCON1;        
    volatile uint32_t TRGSTS0;        
    volatile uint32_t TRGSTS1;        
    volatile uint32_t PHCHG;          
    volatile uint32_t PHCHGNXT;       
    volatile uint32_t PHCHGMASK;      
    volatile uint32_t INTACCUCTL;     

} PWM_T;






 































































































































































































































































































































































































































































































































































































































   
   


 



 


typedef struct {

    








































































































































































































































































 

    volatile uint32_t CNTRL;          
    volatile uint32_t DIVIDER;        
    volatile uint32_t SSR;            
    
    volatile const  uint32_t RESERVE0[1];
    
    volatile const  uint32_t RX;             
    
    volatile const  uint32_t RESERVE1[3];
    
    volatile  uint32_t TX;             
    
    volatile const  uint32_t RESERVE2[6];
    
    volatile uint32_t CNTRL2;         
    volatile uint32_t FIFO_CTL;       
    volatile uint32_t STATUS;         

} SPI_T;






 























































































































































   
   


 



 


typedef struct {

    
























































































































































 

    volatile uint32_t TCSR;          
    volatile uint32_t TCMPR;         
    volatile uint32_t TISR;          
    volatile const  uint32_t TDR;           
    volatile const  uint32_t TCAP;          
    volatile uint32_t TEXCON;        
    volatile uint32_t TEXISR;        
} TIMER_T;






 





















































































   
   


 



 


typedef struct {

    































































































































































































































































































































































































 

    union {
        volatile const  uint32_t RBR;            
        volatile  uint32_t THR;            
    };
    volatile uint32_t IER;            
    volatile uint32_t FCR;            
    volatile uint32_t LCR;            
    volatile uint32_t MCR;            
    volatile uint32_t MSR;            
    volatile uint32_t FSR;            
    volatile uint32_t ISR;            
    volatile uint32_t TOR;            
    volatile uint32_t BAUD;           
    volatile uint32_t IRCR;           
    volatile uint32_t ALT_CSR;        
    volatile uint32_t FUN_SEL;        

} UART_T;






 
















































































































































































































   
   


 



 


typedef struct {

    























































 

    volatile uint32_t WTCR;           

} WDT_T;






 































   
   



#pragma no_anon_unions





 
 






 
#line 5679 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"

#line 5698 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"

   





 
#line 5718 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"

#line 5736 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"

   
   




 

typedef volatile unsigned char  vu8;        
typedef volatile unsigned short vu16;       
typedef volatile unsigned long  vu32;       





 







 







 








 







 








 







 







 






 








 







 








 







 







 






 



   

 
 
 



 











 
#line 5924 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"

 










   

   






 
 
 
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\sys.h"
 








  







    


 



 



     
 
 
 
#line 42 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\sys.h"
    
    
 
 
 
#line 54 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\sys.h"


 
 
 








        





        




        



        



        



        




        





        





        




        




        



        



        



        



        




        










#line 155 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\sys.h"






















        

























    


   



 






 








 








 








 








 









 








 








 








 












 








 








 








 








 








 








 








 








 













 


    
void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
void SYS_LockReg(void);
void SYS_UnlockReg(void);
uint32_t  SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
void SYS_DisableBOD(void);    

   

   

   





#line 5949 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\clk.h"
 








  











 



 





 

 
 
 





 
 
 
#line 56 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\clk.h"


 
 
 
#line 88 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\clk.h"


 
 
 
#line 100 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\clk.h"

       
 
 
 




 
 
  
#line 122 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\clk.h"

#line 131 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\clk.h"
         
 
 
#line 146 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\clk.h"

   




 

void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En);
void CLK_PowerDown(void);
void CLK_Idle(void);
uint32_t CLK_GetHXTFreq(void);
uint32_t CLK_GetLXTFreq(void);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetPCLKFreq(void);
uint32_t CLK_GetCPUFreq(void);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count);
void CLK_DisableSysTick(void);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
void CLK_SysTickDelay(uint32_t us);
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask);



   

   

   







 
#line 5950 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\acmp.h"
 








 











 



 



 

 
 
 
#line 63 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\acmp.h"


   




 




























 
#line 108 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\acmp.h"







 








 








 








 









 








 








 








 








 













 























 






 






 








 








 








 







 


void ACMP_Open(ACMP_T *acmp, uint32_t u32ChNum, uint32_t u32NegSrc, uint32_t u32HysteresisEn);
void ACMP_Close(ACMP_T *acmp, uint32_t u32ChNum);

   

   

   







 
#line 5951 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\adc.h"
 








  











 



 



 

#line 55 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\adc.h"

   




 











 








 











 











 











 

     








      










   







 







 
















  
#line 183 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\adc.h"
                                                                   




   
















                      
#line 215 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\adc.h"





                           










    







 







 


void ADC_Open(ADC_T *adc,
               uint32_t u32InputMode, 
               uint32_t u32OpMode,  
               uint32_t u32ChMask);
void ADC_Close(ADC_T *adc);
void ADC_EnableHWTrigger(ADC_T *adc,
                         uint32_t u32Source,
                         uint32_t u32Param);
void ADC_DisableHWTrigger(ADC_T *adc);
void ADC_SetExtraSampleTime(ADC_T *adc,
                            uint32_t u32ChNum,
                            uint32_t u32SampleTime);
void ADC_EnableInt(ADC_T *adc, uint32_t u32Mask);
void ADC_DisableInt(ADC_T *adc, uint32_t u32Mask);
                   
                   
                   
   

   

   







 
#line 5952 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\fmc.h"
 








 







    


 



 



 
 
 
 









 
 
 
#line 52 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\fmc.h"


   



 

#line 72 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\fmc.h"


extern void FMC_Close(void);
extern int32_t FMC_Erase(uint32_t u32PageAddr);
extern int32_t FMC_GetBootSource(void);
extern void FMC_Open(void);
extern uint32_t FMC_Read (uint32_t u32Addr);
extern uint32_t FMC_ReadCID(void);
extern uint32_t FMC_ReadPID(void);
extern uint32_t FMC_ReadUCID(uint32_t u32Index);
extern uint32_t FMC_ReadUID(uint32_t u32Index);
extern uint32_t FMC_ReadDataFlashBaseAddr(void);
extern void FMC_SetVectorPageAddr(uint32_t u32PageAddr);
extern uint32_t FMC_GetVectorPageAddr(void);
extern void FMC_Write(uint32_t u32Addr, uint32_t u32Data);
extern int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count);
extern int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count);


   

   

   








#line 5953 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\gpio.h"
 








  











 



 



 


 
 
 





 
 
 






 
 
 



 
 
 






#line 81 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\gpio.h"














 
#line 143 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\gpio.h"

   



 










 











 











 











 











 











 











 












 





















 










 











 










 













 












 














 












 



void GPIO_SetMode(GPIO_T *gpio, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_EnableInt(GPIO_T *gpio, uint32_t u32Pin, uint32_t u32IntAttribs);
void GPIO_DisableInt(GPIO_T *gpio, uint32_t u32Pin);



   

   

   







 
#line 5954 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\i2c.h"
 








  











 



 




 






   




 






 






 






 










 






 






 






 






 






 






 






 






 






 







 






 








 








 






 


uint32_t I2C_Open(I2C_T *i2c, uint32_t u32BusClock);
void I2C_Close(I2C_T *i2c);
void I2C_ClearTimeoutFlag(I2C_T *i2c);
void I2C_Trigger(I2C_T *i2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Si, uint8_t u8Ack);
void I2C_DisableInt(I2C_T *i2c);
void I2C_EnableInt(I2C_T *i2c);
uint32_t I2C_GetBusClockFreq(I2C_T *i2c);
uint32_t I2C_SetBusClockFreq(I2C_T *i2c, uint32_t u32BusClock);
uint32_t I2C_GetIntFlag(I2C_T *i2c);
uint32_t I2C_GetStatus(I2C_T *i2c);
uint32_t I2C_GetData(I2C_T *i2c);
void I2C_SetData(I2C_T *i2c, uint8_t u8Data);
void I2C_SetSlaveAddr(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddr, uint8_t u8GCMode);
void I2C_SetSlaveAddrMask(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddrMask);
void I2C_EnableTimeout(I2C_T *i2c, uint8_t u8LongTimeout);
void I2C_DisableTimeout(I2C_T *i2c);
void I2C_EnableWakeup(I2C_T *i2c);
void I2C_DisableWakeup(I2C_T *i2c);

   

   

   







 
#line 5955 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\pwm.h"
 








  











 



 



 
#line 49 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\pwm.h"

   




 






 







 







 







 







 

 





 









 
#line 122 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\pwm.h"










 















 











 











 











 




uint32_t PWM_ConfigOutputChannel(PWM_T *pwm,
                                  uint32_t u32ChannelNum, 
                                  uint32_t u32Frequency, 
                                  uint32_t u32DutyCycle);
void PWM_Start(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_Stop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_ForceStop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
void PWM_DisableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearADCTriggerFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
uint32_t PWM_GetADCTriggerFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableFaultBrake(PWM_T *pwm, 
                           uint32_t u32ChannelMask, 
                           uint32_t u32LevelMask, 
                           uint32_t u32BrakeSource);
void PWM_ClearFaultBrakeFlag(PWM_T *pwm, uint32_t u32BrakeSource);
void PWM_EnableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_DisableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void PWM_DisableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableDutyInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void PWM_DisableDutyInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearDutyIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetDutyIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableFaultBrakeInt(PWM_T *pwm, uint32_t u32BrakeSource);
void PWM_DisableFaultBrakeInt(PWM_T *pwm, uint32_t u32BrakeSource);
void PWM_ClearFaultBrakeIntFlag(PWM_T *pwm, uint32_t u32BrakeSource);
uint32_t PWM_GetFaultBrakeIntFlag(PWM_T *pwm, uint32_t u32BrakeSource);
void PWM_EnablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType);
void PWM_DisablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);



   

   

   







 
#line 5956 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\spi.h"
 








  











 



 



 













#line 50 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\spi.h"


   




 





 






 






 






 






 






 








 








 








 






 







 






 
static __inline void SPI_SET_SS_HIGH(SPI_T *spi)
{
  spi->SSR &= ~(0x1ul << (3));  
  spi->SSR |= ((0x1ul << (5)) | (0x1ul << (2)) | (0x1ul << (0)));  
}





 
static __inline void SPI_SET_SS_LOW(SPI_T *spi)
{
  spi->SSR &= ~(0x1ul << (3));
  spi->SSR |= (0x1ul << (5));  
  spi->SSR &= ~(0x1ul << (2));
  spi->SSR |= (0x1ul << (0));
}





 






 







 






 






 







 
static __inline void SPI_SET_DATA_WIDTH(SPI_T *spi, uint32_t u32Width)
{
   if(u32Width == 32)
        u32Width = 0;
        
   spi->CNTRL = (spi->CNTRL & ~(0x1ful << (3))) | (u32Width << (3));
}







 






 


uint32_t SPI_Open(SPI_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void SPI_Close(SPI_T *spi);
void SPI_ClearRxFIFO(SPI_T *spi);
void SPI_ClearTxFIFO(SPI_T *spi);
void SPI_DisableAutoSS(SPI_T *spi);
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock);
void SPI_EnableFIFO(SPI_T *spi, uint32_t u32TxThreshold, uint32_t u32RxThreshold);
void SPI_DisableFIFO(SPI_T *spi);
uint32_t SPI_GetBusClock(SPI_T *spi);
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask);
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask);

   

   

   







 
#line 5957 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\timer.h"
 








 











 



 



 

#line 47 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\timer.h"

   




 







 









 









 










 








 
static __inline void TIMER_Start(TIMER_T *timer)
{
    timer->TCSR |= (0x1ul << (30));
}





 
static __inline void TIMER_Stop(TIMER_T *timer)
{
    timer->TCSR &= ~(0x1ul << (30));
}






 
static __inline void TIMER_EnableWakeup(TIMER_T *timer)
{
    timer->TCSR |= (0x1ul << (23));
}





 
static __inline void TIMER_DisableWakeup(TIMER_T *timer)
{
    timer->TCSR &= ~(0x1ul << (23));
}






 
static __inline void TIMER_EnableCaptureDebounce(TIMER_T *timer)
{
    timer->TEXCON |= (0x1ul << (6));
}





 
static __inline void TIMER_DisableCaptureDebounce(TIMER_T *timer)
{
    timer->TEXCON &= ~(0x1ul << (6));
}






 
static __inline void TIMER_EnableEventCounterDebounce(TIMER_T *timer)
{
    timer->TEXCON |= (0x1ul << (7));
}





 
static __inline void TIMER_DisableEventCounterDebounce(TIMER_T *timer)
{
    timer->TEXCON &= ~(0x1ul << (7));
}





 
static __inline void TIMER_EnableInt(TIMER_T *timer)
{
    timer->TCSR |= (0x1ul << (29));
}





 
static __inline void TIMER_DisableInt(TIMER_T *timer)
{
    timer->TCSR &= ~(0x1ul << (29));
}





 
static __inline void TIMER_EnableCaptureInt(TIMER_T *timer)
{
    timer->TEXCON |= (0x1ul << (5));
}





 
static __inline void TIMER_DisableCaptureInt(TIMER_T *timer)
{
    timer->TEXCON &= ~(0x1ul << (5));
}







 
static __inline uint32_t TIMER_GetIntFlag(TIMER_T *timer)
{
    return(timer->TISR & (0x1ul << (0)) ? 1 : 0);
}





 
static __inline void TIMER_ClearIntFlag(TIMER_T *timer)
{
    timer->TISR = (0x1ul << (0));
}







 
static __inline uint32_t TIMER_GetCaptureIntFlag(TIMER_T *timer)
{
    return timer->TEXISR;
}





 
static __inline void TIMER_ClearCaptureIntFlag(TIMER_T *timer)
{
    timer->TEXISR = (0x1ul << (0));
}







 
static __inline uint32_t TIMER_GetWakeupFlag(TIMER_T *timer)
{
    return (timer->TISR & (0x1ul << (1)) ? 1 : 0);
}





 
static __inline void TIMER_ClearWakeupFlag(TIMER_T *timer)
{
    timer->TISR = (0x1ul << (1));
}





 
static __inline uint32_t TIMER_GetCaptureData(TIMER_T *timer)
{
    return timer->TCAP;
}





 
static __inline uint32_t TIMER_GetCounter(TIMER_T *timer)
{
    return timer->TDR;
}

uint32_t TIMER_Open(TIMER_T *timer, uint32_t u32Mode, uint32_t u32Freq);
void TIMER_Close(TIMER_T *timer);
void TIMER_Delay(TIMER_T *timer, uint32_t u32Usec);
void TIMER_EnableCapture(TIMER_T *timer, uint32_t u32CapMode, uint32_t u32Edge);
void TIMER_DisableCapture(TIMER_T *timer);
void TIMER_EnableEventCounter(TIMER_T *timer, uint32_t u32Edge);
void TIMER_DisableEventCounter(TIMER_T *timer);
uint32_t TIMER_GetModuleClock(TIMER_T *timer);


   

   

   







 
#line 5958 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\uart.h"
 








  











 



 



 

 
 
 











 
 
 
















 
 
 



 
 
 





   




 








 









 











 









 










 








 









 









 








 









 

                                     








 











 











 









 









 


















 

















 




















                                                                                       








  
__inline void UART_CLEAR_RTS(UART_T* uart)  
{
    uart->MCR |= (0x1ul << (9));
    uart->MCR &= ~(0x1ul << (1));
}






  
__inline void UART_SET_RTS(UART_T* uart)
{
    uart->MCR |= (0x1ul << (9)) | (0x1ul << (1));
}






                                                                                                                                  








     



void UART_ClearIntFlag(UART_T* uart , uint32_t u32InterruptFlag);
void UART_Close(UART_T* uart );
void UART_DisableFlowCtrl(UART_T* uart );
void UART_DisableInt(UART_T*  uart, uint32_t u32InterruptFlag );
void UART_EnableFlowCtrl(UART_T* uart );
void UART_EnableInt(UART_T*  uart, uint32_t u32InterruptFlag );
void UART_Open(UART_T* uart, uint32_t u32baudrate);
uint32_t UART_Read(UART_T* uart, uint8_t *pu8RxBuf, uint32_t u32ReadBytes);
void UART_SetLine_Config(UART_T* uart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t  u32stop_bits);
void UART_SetTimeoutCnt(UART_T* uart, uint32_t u32TOC);
void UART_SelectIrDAMode(UART_T* uart, uint32_t u32Buadrate, uint32_t u32Direction);
void UART_SelectRS485Mode(UART_T* uart, uint32_t u32Mode, uint32_t u32Addr);
uint32_t UART_Write(UART_T* uart,uint8_t *pu8TxBuf, uint32_t u32WriteBytes);


   

   

   







 








#line 5959 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\wdt.h"
 








  











 



 



 
#line 39 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Driver\\wdt.h"

   
    
    


 




 





 





 








 








 








 







 



void  WDT_Open(uint32_t u32TimeoutInterval,
                  uint32_t u32ResetDelay,
                  uint32_t u32EnableReset,
                  uint32_t u32EnableWakeup);  
void WDT_Close(void);

void WDT_EnableInt(void);
void WDT_DisableInt(void);

   

   

   







 
#line 5960 "C:\\Keil_v5\\ARM\\PACK\\Nuvoton\\NuMicro_DFP\\1.1.0\\Device\\Mini51\\Include\\Mini51Series.h"



 
#line 5 "User\\global.h"



#line 18 "User\\global.h"











extern volatile uint32_t unSystemTick;

typedef struct {
	volatile uint16_t unNULL;
	struct {
		volatile uint16_t bMotorNeedToRun :1;
		volatile uint16_t bRotateDirection :1;
	} MCR;
	struct {
		volatile uint16_t bMotorPowerOn :1;
		volatile uint16_t bZeroCrossDetecting :1;
		volatile uint16_t bLocked :1;
		volatile uint16_t bThisPhaseDetectedZX :1;
		volatile uint16_t bNewComFrameReceived :1;
	} MSR;
	volatile uint16_t unMissedZXD_CNT;
	volatile uint16_t unSuccessZXD_CNT;
	volatile uint16_t unLocatingDuty;  
	volatile uint16_t unRampUpDuty;  
	volatile uint16_t unTargetDuty;  
	volatile uint16_t unActualDuty;  
	volatile uint16_t unLocatingPeriod;  
	volatile uint16_t unSpeedADC;  
	volatile uint16_t unReserved1;  
	volatile uint32_t unRampUpPeriod;  
	volatile uint32_t unActualPeriod;  
	volatile uint32_t unPhaseChangeCNT;  
	volatile uint16_t unRPM;  
	volatile uint16_t unBattery;  
	volatile uint16_t unCurrent;  
	volatile uint16_t unReserved2;  
	volatile uint32_t unCommOK_CNT;  
	volatile uint32_t unCommErrCNT;  
} MOTOR_T;

typedef union {
	uint16_t unValue[sizeof(MOTOR_T) / sizeof(uint16_t)];
	MOTOR_T structMotor;
} MOTOR_UNION_T;

#line 1 "User\\BLDCSensorLess.h"
#line 70 "User\\global.h"
#line 1 "User\\Communication.h"



#line 1 "User\\global.h"
#line 5 "User\\Communication.h"

#line 81 "User\\Communication.h"

















#line 106 "User\\Communication.h"




















extern uint16_t unRegisterValue;	

extern uint8_t FlagRegisterNeedWrite;
extern uint16_t unReadValueCRC;
extern void COMM_Manager(void);
#line 71 "User\\global.h"
#line 1 "User\\Error.h"



#line 5 "User\\Error.h"
















 




typedef enum {
	ERR_NULL = 0, ERR_CURRENT_WARNING,	
	ERR_COMMUNICATION_FAIL,
	ERR_LOCATE_FAIL,
	ERR_RAMPUP_FAIL,
	ERR_BATTERY_LOW,
	ERR_INTERNAL,	
	ERR_CURRENT_BURNING,	
	ERR_BRD_FAULT
} ENUM_ERROR_LEVEL;





#line 60 "User\\Error.h"

extern uint32_t unErrorMaster;
extern void delay(uint32_t unDelayMs);
extern void resetError(ENUM_ERROR_LEVEL enumErrorType);
extern void setError(ENUM_ERROR_LEVEL enumErrorType);
extern void clearError(void);
extern void ERR_Manager(void);

#line 72 "User\\global.h"
#line 1 "User\\Protection.h"



#line 5 "User\\Protection.h"






#line 18 "User\\Protection.h"



















#line 48 "User\\Protection.h"

extern void PTC_checkMotor(void);

#line 73 "User\\global.h"
#line 5 "User\\BLDCSensorLess.h"

typedef enum {
	ENUM_TIM1_AVOID_ZXD = 0, ENUM_TIM1_ZXD_FILTER 

} ENUM_TIM1_USAGE;


#line 19 "User\\BLDCSensorLess.h"




#line 37 "User\\BLDCSensorLess.h"







#line 52 "User\\BLDCSensorLess.h"

#line 59 "User\\BLDCSensorLess.h"

#line 66 "User\\BLDCSensorLess.h"























#line 98 "User\\BLDCSensorLess.h"



















#line 123 "User\\BLDCSensorLess.h"






















const uint32_t PHASE_TAB_CLOCKWISE[] = {
	(0x1ul << (30)) | 0x00000000 | (0x0239ul),
	(0x1ul << (30)) | 0x30000000 | (0x022Dul) | (1ul << 7),
	(0x1ul << (30)) | 0x10000000 | (0x0827ul),
	(0x1ul << (30)) | 0x00000000 | (0x0836ul) | (1ul << 7),
	(0x1ul << (30)) | 0x30000000 | (0x201Eul),
	(0x1ul << (30)) | 0x10000000 | (0x201Bul) | (1ul << 7)






};
const uint32_t PHASE_TAB_ANTICLOCKWISE[] = {
	(0x1ul << (30)) | 0x00000000 | (0x0239ul) | (1ul << 7),
	(0x1ul << (30)) | 0x10000000 | (0x201Bul) ,
	(0x1ul << (30)) | 0x30000000 | (0x201Eul) | (1ul << 7),
	(0x1ul << (30)) | 0x00000000 | (0x0836ul) ,
	(0x1ul << (30)) | 0x10000000 | (0x0827ul) | (1ul << 7),
	(0x1ul << (30)) | 0x30000000 | (0x022Dul)
};

typedef enum {
	MOTOR_IDLE = 0,

	MOTOR_START,
	MOTOR_LOCATE,
	MOTOR_WAIT_AFTER_LOCATE,
	MOTOR_RAMPUP_WO_ZXD,
	MOTOR_RAMPUP_W_ZXD,
	MOTOR_LOCKED
}ENUM_MOTOR_STATE;

typedef enum {
	DETECT_START = 0,
	DETECT_PHASE_1_P,
	DETECT_PHASE_1_A,
	DETECT_PHASE_2_P,
	DETECT_PHASE_2_A,
	DETECT_PHASE_3_P,
	DETECT_PHASE_3_A
}ENUM_ROTATE_DETECT_STATE;







typedef enum {
	STATUS_ERROR = 0,
	STATUS_FINISHED,
	STATUS_WORKING = 0xFFFF
}ENUM_STATUS;

const uint8_t unLocatePhaseSequencyTable[] = {0, 1, 2, 1};

#line 209 "User\\BLDCSensorLess.h"
















































static ENUM_MOTOR_STATE tMotorState = MOTOR_IDLE;
static ENUM_ROTATE_DETECT_STATE tRotateDetectState = DETECT_START;
static uint8_t unLocateIndex;
static uint8_t unPhaseChangeCNT_AtCurrentPeriod; 

 
static uint8_t unPhaseChangeCNT_AtCurrentDuty; 
 
static uint16_t unPeriodChangeCNT_AfterPR_ReachMini; 

 
static uint32_t unLastPhaseChangeTime;
static uint32_t unRotateDetectStartTime;	




 volatile MOTOR_UNION_T tMotor;	
 volatile ENUM_TIM1_USAGE FLAG_TIM1_USEAGE;
 volatile uint32_t unLastZXDetectedTime;
 volatile uint32_t unZXMatchCNT;




 uint8_t unCurrentPhase;
 uint8_t FLAG_PHASE_CHANGED;
 __inline void BLDC_stopMotor(void);
 void BLDC_SensorLessManager(void);
#line 13 "User\\BLDCSensorLess.c"








 



 

__inline void PhaseChangedRoutine(void) {
	FLAG_PHASE_CHANGED = (0);
	tMotor.structMotor.unPhaseChangeCNT++;

	if ((1) == tMotor.structMotor.MSR.bZeroCrossDetecting) {

		
		
		if ((1) == tMotor.structMotor.MSR.bThisPhaseDetectedZX) {
			tMotor.structMotor.unMissedZXD_CNT = 0;

			if (tMotor.structMotor.unSuccessZXD_CNT > 4) {
				tMotor.structMotor.MSR.bLocked = (1);
			} else {
				tMotor.structMotor.unSuccessZXD_CNT++;
			}
		} else		
		{
			tMotor.structMotor.unSuccessZXD_CNT = 0;
			
			
			unLastZXDetectedTime = (((TIMER_GetCounter(((TIMER_T *) (((uint32_t)0x40000000) + 0x10020)))) > ((tMotor . structMotor . unActualPeriod >> 2))) ? ((TIMER_GetCounter(((TIMER_T *) (((uint32_t)0x40000000) + 0x10020)))) - ((tMotor . structMotor . unActualPeriod >> 2))) : ((TIMER_GetCounter(((TIMER_T *) (((uint32_t)0x40000000) + 0x10020)))) + (0xFFFFFF - ((tMotor . structMotor . unActualPeriod >> 2)))));
			if (tMotor.structMotor.unMissedZXD_CNT > 12) {
				if ((1) == tMotor.structMotor.MSR.bLocked) {
					tMotor.structMotor.MSR.bLocked = (0);
					((*((volatile uint32_t *)(((((uint32_t)0x50000000) + 0x04200)+(0x20*(3))) + ((0)<<2)))) = 0); (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PIER = 0); (TIMER_Stop(((TIMER_T *) (((uint32_t)0x40000000) + 0x10000)))); (TIMER_Stop(((TIMER_T *) (((uint32_t)0x40000000) + 0x10020)))); (TIMER_DisableInt(((TIMER_T *) (((uint32_t)0x40000000) + 0x10000)))); (TIMER_DisableInt(((TIMER_T *) (((uint32_t)0x40000000) + 0x10020)))); (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PHCHGNXT = (0x000000FFul)); (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PHCHG = (0x000000FFul));
					setError(ERR_INTERNAL);
				}
			} else {
				tMotor.structMotor.unMissedZXD_CNT++;
			}
		}

	}

	if ((1) == tMotor.structMotor.MSR.bLocked) {
		
		
		((((TIMER_T *) (((uint32_t)0x40000000) + 0x10000)))->TCMPR = (tMotor . structMotor . unActualPeriod << 1));
	}

	tMotor.structMotor.MSR.bThisPhaseDetectedZX = (0);
	
	(((*((volatile uint32_t *)(((((uint32_t)0x50000000) + 0x04200)+(0x20*(5))) + ((0)<<2))))) ^= 1);
}



 
uint16_t canMotorContinueRunning(void) {




	return 0;
#line 128 "User\\BLDCSensorLess.c"
}


void BLDCSpeedManager(void) {
	if ((1) == FLAG_PHASE_CHANGED) {
		PhaseChangedRoutine();

		if (tMotor.structMotor.unActualDuty != tMotor.structMotor.unTargetDuty) {

			
			if (unPhaseChangeCNT_AtCurrentDuty > 5) {
				unPhaseChangeCNT_AtCurrentDuty = 0;
				if (tMotor.structMotor.unActualDuty < tMotor.structMotor.unTargetDuty) {
					tMotor.structMotor.unActualDuty++;
				} else {
					tMotor.structMotor.unActualDuty--;
				}
				(((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CMR[1] = (tMotor . structMotor . unActualDuty)); (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CMR[3] = (tMotor . structMotor . unActualDuty)); (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CMR[5] = (tMotor . structMotor . unActualDuty));
			}
			unPhaseChangeCNT_AtCurrentDuty++;
		}

		(((unCurrentPhase)) = ((((unCurrentPhase)) < (((sizeof(PHASE_TAB_CLOCKWISE)/sizeof(uint32_t))) - 1)) ? (((unCurrentPhase)) + 1) : 0));
		
		((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PHCHGNXT = ((tMotor . structMotor . MCR . bRotateDirection == 0) ? PHASE_TAB_CLOCKWISE[(unCurrentPhase)] : PHASE_TAB_ANTICLOCKWISE[(unCurrentPhase)]);
	}
}

__inline void BLDC_stopMotor(void) {
	((*((volatile uint32_t *)(((((uint32_t)0x50000000) + 0x04200)+(0x20*(3))) + ((0)<<2)))) = 0); (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PIER = 0); (TIMER_Stop(((TIMER_T *) (((uint32_t)0x40000000) + 0x10000)))); (TIMER_Stop(((TIMER_T *) (((uint32_t)0x40000000) + 0x10020)))); (TIMER_DisableInt(((TIMER_T *) (((uint32_t)0x40000000) + 0x10000)))); (TIMER_DisableInt(((TIMER_T *) (((uint32_t)0x40000000) + 0x10020)))); (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PHCHGNXT = (0x000000FFul)); (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PHCHG = (0x000000FFul));
	tMotor.structMotor.MCR.bMotorNeedToRun = (0);
	tMotor.structMotor.MSR.bMotorPowerOn = (0);
	tMotorState = MOTOR_IDLE;
}

__inline void setPhaseManually(uint16_t iPWMDuty, uint8_t iPhase) {
	(((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CMR[1] = (iPWMDuty)); (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CMR[3] = (iPWMDuty)); (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CMR[5] = (iPWMDuty));
	((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PHCHG = ((tMotor . structMotor . MCR . bRotateDirection == 0) ? PHASE_TAB_CLOCKWISE[(iPhase)] : PHASE_TAB_ANTICLOCKWISE[(iPhase)]);
}

ENUM_STATUS BLDC_LocatingManager(void) {
	if ((uint32_t) (unSystemTick - unLastPhaseChangeTime) > tMotor.structMotor.unLocatingPeriod) {
		if (unLocateIndex < (sizeof(unLocatePhaseSequencyTable) / sizeof(uint8_t))) {
			
			setPhaseManually(tMotor.structMotor.unLocatingDuty, unLocatePhaseSequencyTable[unLocateIndex]);
			unLocateIndex++;
		} else {
			((*((volatile uint32_t *)(((((uint32_t)0x50000000) + 0x04200)+(0x20*(3))) + ((0)<<2)))) = 0); (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PIER = 0); (TIMER_Stop(((TIMER_T *) (((uint32_t)0x40000000) + 0x10000)))); (TIMER_Stop(((TIMER_T *) (((uint32_t)0x40000000) + 0x10020)))); (TIMER_DisableInt(((TIMER_T *) (((uint32_t)0x40000000) + 0x10000)))); (TIMER_DisableInt(((TIMER_T *) (((uint32_t)0x40000000) + 0x10020)))); (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PHCHGNXT = (0x000000FFul)); (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PHCHG = (0x000000FFul));
			tMotor.structMotor.MSR.bMotorPowerOn = (0);
			unCurrentPhase = unLocatePhaseSequencyTable[unLocateIndex - 1];
			return STATUS_FINISHED;
		}
	}
	return STATUS_WORKING;
}

__inline void BLDCRampUp_Manager(void) {
	if ((1) == FLAG_PHASE_CHANGED) {
		PhaseChangedRoutine();
		if (unPhaseChangeCNT_AtCurrentPeriod > 9) {
			unPhaseChangeCNT_AtCurrentPeriod = 0;
			

			((tMotor . structMotor . unActualPeriod) = (((tMotor . structMotor . unActualPeriod) < (1000 - 1)) ? (tMotor . structMotor . unActualPeriod) : (uint16_t)((tMotor . structMotor . unActualPeriod) * (0.98))));
			if (tMotor.structMotor.unActualPeriod <= (1000 - 1)) {
				unPeriodChangeCNT_AfterPR_ReachMini++;
			}
		}
		unPhaseChangeCNT_AtCurrentPeriod++;

		((((TIMER_T *) (((uint32_t)0x40000000) + 0x10000)))->TCMPR = (tMotor . structMotor . unActualPeriod));
		(((unCurrentPhase)) = ((((unCurrentPhase)) < (((sizeof(PHASE_TAB_CLOCKWISE)/sizeof(uint32_t))) - 1)) ? (((unCurrentPhase)) + 1) : 0));
		
		((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PHCHGNXT = ((tMotor . structMotor . MCR . bRotateDirection == 0) ? PHASE_TAB_CLOCKWISE[(unCurrentPhase)] : PHASE_TAB_ANTICLOCKWISE[(unCurrentPhase)]);
	}
}

__inline void dutyProtection(void) {
	
	if ((tMotor.structMotor.unActualDuty > ((884-1) - 150)) || (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CMR[1] > ((884-1) - 150))) {
		BLDC_stopMotor();
		setError(ERR_INTERNAL);
	}
}

__inline void phaseDurationProtection(uint32_t unLastPhaseChangeTime) {
	static uint32_t unCurrentPHCHG;
	
	if ((1) == tMotor.structMotor.MSR.bMotorPowerOn) {
		if (unCurrentPHCHG != ((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PHCHG) {
			unCurrentPHCHG = ((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PHCHG;
			unLastPhaseChangeTime = unSystemTick;
		} else {
			if ((uint32_t) (unSystemTick - unLastPhaseChangeTime) > 80) {
				BLDC_stopMotor();
				setError(ERR_INTERNAL);
			}
		}
	}
}


void BLDC_SensorLessManager(void) {
	uint16_t unMotorAlreadyRotatingPhaseTime;
	static uint32_t iEnterTimeBeforeWait;

	dutyProtection();
	phaseDurationProtection(unLastPhaseChangeTime);

	switch (tMotorState) {
	case MOTOR_IDLE:
		if (tMotor.structMotor.MCR.bMotorNeedToRun && (((unErrorMaster & 0xFFFFFFFEul) == 0) ? (1) : (0))) {
			unRotateDetectStartTime = unSystemTick;
			tRotateDetectState = DETECT_START;
			tMotorState = MOTOR_START;
		}
		break;

	case MOTOR_START:
		if (tMotor.structMotor.MCR.bMotorNeedToRun && (((unErrorMaster & 0xFFFFFFFEul) == 0) ? (1) : (0))) {
			
			
			
			unMotorAlreadyRotatingPhaseTime = canMotorContinueRunning();
			if (unMotorAlreadyRotatingPhaseTime != 0xFFFF) {
				if (unMotorAlreadyRotatingPhaseTime > 0) {
					
					tMotorState = MOTOR_LOCKED;
				} else {
					
					
					unCurrentPhase = 0;
					unLocateIndex = 0;
					tMotor.structMotor.unMissedZXD_CNT = 0;
					unLastPhaseChangeTime = unSystemTick;
					tMotor.structMotor.MSR.bMotorPowerOn = (1);
					
					tMotor.structMotor.MSR.bZeroCrossDetecting = (0);
					tMotor.structMotor.MSR.bLocked = (0);
					
					((*((volatile uint32_t *)(((((uint32_t)0x50000000) + 0x04200)+(0x20*(3))) + ((0)<<2)))) = 1);
					tMotorState = MOTOR_LOCATE;
				}
			}
		} else {
			BLDC_stopMotor();
		}
		break;

	case MOTOR_LOCATE:
		if (tMotor.structMotor.MCR.bMotorNeedToRun && (((unErrorMaster & 0xFFFFFFFEul) == 0) ? (1) : (0))) {
			if (BLDC_LocatingManager() == STATUS_FINISHED) {
				iEnterTimeBeforeWait = unSystemTick;
				tMotorState = MOTOR_WAIT_AFTER_LOCATE;
			}
		} else {
			BLDC_stopMotor();
		}
		break;

	case MOTOR_WAIT_AFTER_LOCATE:
		if (tMotor.structMotor.MCR.bMotorNeedToRun && (((unErrorMaster & 0xFFFFFFFEul) == 0) ? (1) : (0))) {



				tMotor.structMotor.unActualDuty = tMotor.structMotor.unRampUpDuty;
				tMotor.structMotor.unActualPeriod = tMotor.structMotor.unRampUpPeriod;
				tMotor.structMotor.MSR.bMotorPowerOn = (1);
				(((unCurrentPhase)) = ((((unCurrentPhase)) < (((sizeof(PHASE_TAB_CLOCKWISE)/sizeof(uint32_t))) - 1)) ? (((unCurrentPhase)) + 1) : 0));
				setPhaseManually(tMotor.structMotor.unActualDuty, unCurrentPhase);
				((*((volatile uint32_t *)(((((uint32_t)0x50000000) + 0x04200)+(0x20*(3))) + ((0)<<2)))) = 1);
				
				
				
				
				
				
				(((unCurrentPhase)) = ((((unCurrentPhase)) < (((sizeof(PHASE_TAB_CLOCKWISE)/sizeof(uint32_t))) - 1)) ? (((unCurrentPhase)) + 1) : 0));
				((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PHCHGNXT = ((tMotor . structMotor . MCR . bRotateDirection == 0) ? PHASE_TAB_CLOCKWISE[(unCurrentPhase)] : PHASE_TAB_ANTICLOCKWISE[(unCurrentPhase)]);
				
				
				
				
				((((TIMER_T *) (((uint32_t)0x40000000) + 0x10000)))->TCMPR = (tMotor . structMotor . unActualPeriod));
				TIMER_Start(((TIMER_T *) (((uint32_t)0x40000000) + 0x10000)));				
				TIMER_EnableInt(((TIMER_T *) (((uint32_t)0x40000000) + 0x10000)));
				unPeriodChangeCNT_AfterPR_ReachMini = 0;
				unPhaseChangeCNT_AtCurrentDuty = 0;
				unPhaseChangeCNT_AtCurrentPeriod = 0;
				tMotorState = MOTOR_RAMPUP_WO_ZXD;



		} else {
			BLDC_stopMotor();
		}
		break;

	case MOTOR_RAMPUP_WO_ZXD:	
		if (tMotor.structMotor.MCR.bMotorNeedToRun && (((unErrorMaster & 0xFFFFFFFEul) == 0) ? (1) : (0))) {
			BLDCRampUp_Manager();
			if (tMotor.structMotor.unActualPeriod <= (1600 - 1))	
			{
				tMotor.structMotor.MSR.bThisPhaseDetectedZX = (0);
				tMotor.structMotor.MSR.bZeroCrossDetecting = (1);
				
				
				
				



				(((ACMP_T *) (((uint32_t)0x40000000) + 0xD0000))->CMPCR[0] |= (0x1ul << (2)) | (0x1ul << (0)));
				TIMER_Start(((TIMER_T *) (((uint32_t)0x40000000) + 0x10020)));				

						

				tMotorState = MOTOR_RAMPUP_W_ZXD;
			}
		} else {
			BLDC_stopMotor();
		}
		break;

	case MOTOR_RAMPUP_W_ZXD:	
		if (tMotor.structMotor.MCR.bMotorNeedToRun && (((unErrorMaster & 0xFFFFFFFEul) == 0) ? (1) : (0))) {
			if ((1) == tMotor.structMotor.MSR.bLocked) {
				
				
				
				tMotorState = MOTOR_LOCKED;
			} else {
				if (unPeriodChangeCNT_AfterPR_ReachMini < 300) {
					BLDCRampUp_Manager();
				} else {
					setError(ERR_RAMPUP_FAIL);
				}
			}
		} else {
			BLDC_stopMotor();
		}
		break;

	case MOTOR_LOCKED:
		if (tMotor.structMotor.MCR.bMotorNeedToRun && (((unErrorMaster & 0xFFFFFFFEul) == 0) ? (1) : (0))) {
			BLDCSpeedManager();	
		} else {
			BLDC_stopMotor();
		}
		break;

	default:
		break;
	}
}

