#line 1 "User\\Communication.c"









    
#line 1 "User\\Communication.h"



#line 1 "User\\global.h"



#line 1 ".\\CMSIS\\Mini51Series.h"
 










  


















 















 

 
 
 



 



 
typedef enum IRQn
{
 

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






 


 






   


#line 1 ".\\CMSIS\\core_cm0.h"
 




















 













 












 




 


 

 













#line 89 ".\\CMSIS\\core_cm0.h"


 







#line 114 ".\\CMSIS\\core_cm0.h"

#line 1 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
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




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 116 ".\\CMSIS\\core_cm0.h"
#line 1 ".\\CMSIS\\core_cmInstr.h"
 




















 






 


 



 


 









 







 







 






 








 







 







 









 









 
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









 



#line 268 ".\\CMSIS\\core_cmInstr.h"



#line 619 ".\\CMSIS\\core_cmInstr.h"

   

   

#line 117 ".\\CMSIS\\core_cm0.h"
#line 1 ".\\CMSIS\\core_cmFunc.h"
 




















 






 

 



 


 





 
 






 
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


#line 260 ".\\CMSIS\\core_cmFunc.h"


#line 296 ".\\CMSIS\\core_cmFunc.h"


#line 615 ".\\CMSIS\\core_cmFunc.h"

 

   

#line 118 ".\\CMSIS\\core_cm0.h"








 
#line 143 ".\\CMSIS\\core_cm0.h"

 






 
#line 159 ".\\CMSIS\\core_cm0.h"

 










 


 





 


 
typedef union
{
  struct
  {

    uint32_t _reserved0:27;               





    uint32_t Q:1;                         
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
    uint32_t IT:2;                        
    uint32_t Q:1;                         
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
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[1];                  
       uint32_t RESERVED0[31];
  volatile uint32_t ICER[1];                  
       uint32_t RSERVED1[31];
  volatile uint32_t ISPR[1];                  
       uint32_t RESERVED2[31];
  volatile uint32_t ICPR[1];                  
       uint32_t RESERVED3[31];
       uint32_t RESERVED4[64];
  volatile uint32_t IP[8];                    
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
  volatile uint32_t SHP[2];                   
  volatile uint32_t SHCSR;                    
} SCB_Type;

 















 



























 















 









 






 



 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 








 
 






 

 










 









 

 



 




 

 
 










 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}











 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}







 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}










 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] = (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] = (((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
}












 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) >> (8 - 2)));  }  
  else {
    return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( ((uint32_t)(IRQn) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) >> (8 - 2)));  }  
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FA << 16)      |
                 (1UL << 2));
  __dsb(0xF);                                                      
  while(1);                                                     
}

 



 




 

















 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if (ticks > (0xFFFFFFUL << 0))  return (1);             

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (ticks & (0xFFFFFFUL << 0)) - 1;       
  NVIC_SetPriority (SysTick_IRQn, (1<<2) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2) |
                   (1UL << 1)   |
                   (1UL << 0);                     
  return (0);                                                   
}



 








   

#line 112 ".\\CMSIS\\Mini51Series.h"
#line 1 ".\\CMSIS\\system_Mini51Series.h"
 








   
 








 
 
 



 

#line 34 ".\\CMSIS\\system_Mini51Series.h"


extern uint32_t SystemCoreClock;         
extern uint32_t CyclesPerUs;             









 

extern void SystemCoreClockUpdate (void);








 
#line 113 ".\\CMSIS\\Mini51Series.h"
#line 114 ".\\CMSIS\\Mini51Series.h"

 
 
 



 


#pragma anon_unions



 




  


    
typedef struct
{
    








 
  volatile const  uint32_t  PDID;

    

































     
  volatile uint32_t  RSTSRC;

    
























     
  volatile uint32_t  IPRSTC1;

    

































     
  volatile uint32_t  IPRSTC2;

    




     
       uint32_t  RESERVED0[2];

    


































     
  volatile uint32_t  BODCTL;

    




     
       uint32_t  RESERVED1[5];

    





















































     
  volatile uint32_t  P0_MFP;

    















































     
  volatile uint32_t  P1_MFP;

    















































     
  volatile uint32_t  P2_MFP;

    

























































     
  volatile uint32_t  P3_MFP;

    
























     
  volatile uint32_t  P4_MFP;

    
















































     
  volatile uint32_t  P5_MFP;

    




     
       uint32_t  RESERVED3[14];

    

























     
  volatile uint32_t  IRCTRIMCTL;

    




















     
  volatile uint32_t  IRCTRIMIER;

    























     
  volatile uint32_t  IRCTRIMISR;

    




     
       uint32_t  RESERVED4[29];

    





















     
  volatile uint32_t  RegLockAddr;
} GCR_T;

 


















 






 



























 


















 









 









 









 









 









 









 






 






 









 



   


 




  


    
typedef struct
{
    


















































     
  volatile uint32_t  PWRCON;

    









     
  volatile uint32_t  AHBCLK;

    













































     
  volatile uint32_t  APBCLK;

    



















     
  volatile uint32_t  CLKSTATUS;

    



























     
  volatile uint32_t  CLKSEL0;

    








































     
  volatile uint32_t  CLKSEL1;

    












     
  volatile uint32_t  CLKDIV;

    














     
  volatile uint32_t  CLKSEL2;
    

 
       uint32_t  RESERVED0; 

    














     
  volatile uint32_t  FRQDIV;
} CLK_T;

 




























 



 





























                                                






 
















 






 





















 






 









 









   


 



 


    
typedef struct
{
  volatile uint32_t  CMPCR[2];       
  volatile uint32_t  CMPSR;          
  volatile uint32_t  CMPRVCR;        
} ACMP_T; 

 





















 


















   


 




  


    
typedef struct
{
  volatile uint32_t  ADDR;            
       uint32_t  RESERVED0[7];    
  volatile uint32_t  ADCR;            
  volatile uint32_t  ADCHER;          
  volatile uint32_t  ADCMPR[2];       
  volatile uint32_t  ADSR;            
  volatile uint32_t  ADTDCR;          
  volatile uint32_t  ADSAMP;            
} ADC_T; 

 









 


















 






 


















 





















 



 



   


 




  


    
typedef struct
{
    




















































     
    volatile uint32_t ISPCON;

    









     
    volatile uint32_t ISPADR;

    









     
    volatile uint32_t ISPDAT;

    















     
    volatile uint32_t ISPCMD;

    











     
    volatile uint32_t ISPTRG;

    













     
    volatile const  uint32_t DFBADR;

} FMC_T;


 














                                                                                        



 









 




   





  


    
typedef struct
{
  volatile uint32_t  PMD;                          
  volatile uint32_t  OFFD;                        
  volatile uint32_t  DOUT;                        
  volatile uint32_t  DMASK;                       
  volatile const  uint32_t  PIN;                         
  volatile uint32_t  DBEN;                        
  volatile uint32_t  IMD;                         
  volatile uint32_t  IEN;                         
  volatile uint32_t  ISRC;                        
} GPIO_T;  



   
typedef struct                                  
{                                               
  volatile uint32_t  DBNCECON;                                  
} GPIO_DBNCECON_T; 

 
























 



                                            



 



 



 



 



 






 



 











   
typedef struct
{
  volatile uint32_t  GP_BIT0;        
  volatile uint32_t  GP_BIT1;        
  volatile uint32_t  GP_BIT2;        
  volatile uint32_t  GP_BIT3;        
  volatile uint32_t  GP_BIT4;        
  volatile uint32_t  GP_BIT5;        
  volatile uint32_t  GP_BIT6;        
  volatile uint32_t  GP_BIT7;        
} GPIOBIT_T; 

   


 




  


    
typedef struct
{
  volatile uint32_t  I2CON;             
  volatile uint32_t  I2CADDR0;          
  volatile uint32_t  I2CDAT;            
  volatile uint32_t  I2CSTATUS;         
  volatile uint32_t  I2CLK;             
  volatile uint32_t  I2CTOC;            
  volatile uint32_t  I2CADDR1;          
  volatile uint32_t  I2CADDR2;          
  volatile uint32_t  I2CADDR3;          
  volatile uint32_t  I2CADM0;           
  volatile uint32_t  I2CADM1;           
  volatile uint32_t  I2CADM2;           
  volatile uint32_t  I2CADM3;           
       uint32_t  RESERVED0;         
       uint32_t  RESERVED1;         
  volatile uint32_t  I2CON2;            
  volatile uint32_t  I2CSTATUS2;        
} I2C_T; 


 


















 






 



 



 



 









 



 















 















   




 


   
typedef struct
{   
    volatile const  uint32_t IRQSRC[32];  
    volatile uint32_t NMICNO;      
    volatile uint32_t MCUIRQ;      

} INT_T;
   

 




  


    
typedef struct
{
  volatile uint32_t  PPR;                   
  volatile uint32_t  CSR;                   
  volatile uint32_t  PCR;                   
  volatile uint32_t  CNR[6];                
  volatile uint32_t  CMR[6];                
       uint32_t  RESERVED0[6];          
  volatile uint32_t  PIER;                  
  volatile uint32_t  PIIR;                  
  volatile uint32_t  POE;                   
  volatile uint32_t  PFBCON;                
  volatile uint32_t  PDZIR;                 
  volatile uint32_t  TRGCON0;               
  volatile uint32_t  TRGCON1;               
  volatile uint32_t  TRGSTS0;               
  volatile uint32_t  TRGSTS1;               
  volatile uint32_t  PHCHG;                 
  volatile uint32_t  PHCHGNXT;              
  volatile uint32_t  PHCHGMASK;             
  volatile uint32_t  INTACCUCTL;            
} PWM_T;

 









 


















 














































































 



 




 










































 










































 


















 




































 









 



































                        





                        






 











                        





                        






 



































                        


                        


                        






 











                        


                        


                        






 


























































































 


























































































 









 







   


 




 


    
typedef struct
{
    volatile uint32_t CNTRL;           
    volatile uint32_t DIVIDER;         
    volatile uint32_t SSR;             
         uint32_t RESERVED0;       
    volatile const  uint32_t RX;              
         uint32_t RESERVED1[3];    
    volatile  uint32_t TX;              
         uint32_t RESERVED2[6];    
    volatile uint32_t CNTRL2;          
    volatile uint32_t FIFO_CTL;        
    volatile uint32_t STATUS;          
} SPI_T; 

 
















































 



 















 


















 
























 



































   


 



  


    
typedef struct
{
  volatile uint32_t  TCSR;        
  volatile uint32_t  TCMPR;       
  volatile uint32_t  TISR;        
  volatile const  uint32_t  TDR;         
  volatile const  uint32_t  TCAP;        
  volatile uint32_t  TEXCON;      
  volatile uint32_t  TEXISR;      
} TIMER_T;

 







































 



 






 



 



 
























 



   


 




  


    
typedef struct
{
  union {
  volatile const   uint32_t  RBR;          
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
} UART_T; 



 



 



 






























 















 


















 










 










 







































 

















           



















 






 












 









 















 





   

 




  


    
typedef struct
{
  volatile uint32_t  WTCR;           
} WDT_T; 

 






























   



#pragma no_anon_unions





 
 






 
#line 3192 ".\\CMSIS\\Mini51Series.h"

#line 3211 ".\\CMSIS\\Mini51Series.h"

   





 
#line 3228 ".\\CMSIS\\Mini51Series.h"

#line 3246 ".\\CMSIS\\Mini51Series.h"

   
    




 

typedef volatile unsigned char  vu8;        
typedef volatile unsigned short vu16;       
typedef volatile unsigned long  vu32;       





 







 







 








 







 








 







 







 






 








 







 








 







 







 






 



   

 
 
 



 











 
#line 3434 ".\\CMSIS\\Mini51Series.h"

 










   

   






 
 
  
#line 1 ".\\StdDriver\\inc\\sys.h"
 








  







    


 



 



     
 
 
 
#line 44 ".\\StdDriver\\inc\\sys.h"
    
    
 
 
 
#line 56 ".\\StdDriver\\inc\\sys.h"


 
 
 








        





        




        



        



        



        




        





        





        




        




        



        



        



        



        




        





































        

























    


   



 
#line 224 ".\\StdDriver\\inc\\sys.h"

#line 232 ".\\StdDriver\\inc\\sys.h"

    
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

   

   

   





#line 3459 ".\\CMSIS\\Mini51Series.h"
#line 1 ".\\StdDriver\\inc\\clk.h"
 








  











 



 





 

 
 
 



 
 
 
#line 50 ".\\StdDriver\\inc\\clk.h"


 
 
 
#line 75 ".\\StdDriver\\inc\\clk.h"


 
 
 





       
 
 
 




 
 
  
#line 105 ".\\StdDriver\\inc\\clk.h"
         
 
 
#line 120 ".\\StdDriver\\inc\\clk.h"

   




 

void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En);
void CLK_PowerDown(void);
void CLK_Idle(void);
uint32_t CLK_GetHXTFreq(void);
uint32_t CLK_GetLXTFreq(void);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetPCLKFreq(void);
uint32_t CLK_GetCPUFreq(void);
uint32_t CLK_GetPLLClockFreq(void);
uint32_t CLK_SetCoreClock(uint32_t u32Hclk);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq);
void CLK_DisablePLL(void);
void CLK_SysTickDelay(uint32_t us);
void CLK_WaitClockReady(uint32_t u32ClkMask);



   

   

   







 
#line 3460 ".\\CMSIS\\Mini51Series.h"

#line 1 ".\\StdDriver\\inc\\adc.h"
 








  











 



 



 

#line 55 ".\\StdDriver\\inc\\adc.h"

   




 










 







 










 










 










 

     







      









   






 






 















  
#line 173 ".\\StdDriver\\inc\\adc.h"
                                                                   



   















                      
#line 203 ".\\StdDriver\\inc\\adc.h"




                           









    






 






 


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
                   
                   
                   
   

   

   







 
#line 3462 ".\\CMSIS\\Mini51Series.h"
#line 1 ".\\StdDriver\\inc\\fmc.h"
 








 







    


 



 



 
 
 
 









 
 
 
#line 52 ".\\StdDriver\\inc\\fmc.h"


   



 

#line 71 ".\\StdDriver\\inc\\fmc.h"


extern void FMC_SetBootSource(int32_t i32BootSrc);
extern void FMC_Close(void);
extern void FMC_DisableAPUpdate(void);
extern void FMC_DisableConfigUpdate(void);
extern void FMC_DisableLDUpdate(void);
extern void FMC_EnableAPUpdate(void);
extern void FMC_EnableConfigUpdate(void);
extern void FMC_EnableLDUpdate(void);
extern int32_t FMC_Erase(uint32_t u32PageAddr);
extern int32_t FMC_GetBootSource(void);
extern void FMC_Open(void);
extern uint32_t FMC_Read (uint32_t u32Addr);
extern uint32_t FMC_ReadCID(void);
extern uint32_t FMC_ReadDID(void);
extern uint32_t FMC_ReadPID(void);
extern uint32_t FMC_ReadUCID(uint32_t u32Index);
extern uint32_t FMC_ReadUID(uint32_t u32Index);
extern uint32_t FMC_ReadDataFlashBaseAddr(void);
extern void FMC_SetVectorPageAddr(uint32_t u32PageAddr);
extern void FMC_Write(uint32_t u32Addr, uint32_t u32Data);
extern int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count);
extern int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count);


   

   

   








#line 3463 ".\\CMSIS\\Mini51Series.h"
#line 1 ".\\StdDriver\\inc\\gpio.h"
 








  











 



 



 


 
 
 





 
 
 






 
 
 



 
 
 






#line 81 ".\\StdDriver\\inc\\gpio.h"














 
#line 144 ".\\StdDriver\\inc\\gpio.h"

   



 










 











 











 











 











 











 











 












 



















 










 











 










 













 












 














 












 



void GPIO_SetMode(GPIO_T *PORT, uint32_t pin_mask, uint32_t mode);
void GPIO_EnableInt(GPIO_T *PORT, uint32_t u32Pin, uint32_t u32IntAttribs);
void GPIO_DisableInt(GPIO_T *PORT, uint32_t u32Pin);



   

   

   







 
#line 3464 ".\\CMSIS\\Mini51Series.h"
#line 1 ".\\StdDriver\\inc\\i2c.h"
 








  











 



 




 






   




 






 
static __inline void I2C_SET_CONTROL_REG(I2C_T *i2c, uint8_t u8Ctrl)
{
    i2c->I2CON = (i2c->I2CON & ~0x3c) | u8Ctrl;
}





 
static __inline void I2C_START(I2C_T *i2c)
{
    i2c->I2CON = (i2c->I2CON & ~(1ul << 3)) | (1ul << 5);
}





 
static __inline void I2C_STOP(I2C_T *i2c)
{
    i2c->I2CON = (i2c->I2CON & ~(1ul << 3)) | (1ul << 4);
}





 
static __inline void I2C_WAIT_READY(I2C_T *i2c)
{
    while(!(i2c->I2CON & (1ul << 3)));
}





 
static __inline void I2C_DISABLE_FIFO(I2C_T *i2c)
{
    i2c->I2CON2 &= ~(1ul << 1);
}





 
static __inline void I2C_ENABLE_FIFO(I2C_T *i2c)
{
    i2c->I2CON2 |= (1ul << 1);
}





 
static __inline void I2C_DISABLE_CLOCK_STRETCH(I2C_T *i2c)
{
    i2c->I2CON2 |= (1ul << 2);
}





 
static __inline void I2C_ENABLE_CLOCK_STRETCH(I2C_T *i2c)
{
    i2c->I2CON2 &= ~(1ul << 2);
}





 
static __inline void I2C_DISABLE_OVERRUN_INT(I2C_T *i2c)
{
    i2c->I2CON2 &= ~(1ul << 3);
}





 
static __inline void I2C_ENABLE_OVERRUN_INT(I2C_T *i2c)
{
    i2c->I2CON2 |= (1ul << 3);
}





 
static __inline void I2C_ENABLE_UNDERRUN_INT(I2C_T *i2c)
{
    i2c->I2CON2 |= (1ul << 4);
}





 
static __inline void I2C_DISABLE_UNDERRUN_INT(I2C_T *i2c)
{
    i2c->I2CON2 &= ~(1ul << 4);
}





 
static __inline uint32_t I2C_GET_DATA(I2C_T *i2c)
{
    return ( i2c->I2CDAT );
}






 
static __inline void I2C_SET_DATA(I2C_T *i2c, uint8_t u8Data)
{
    i2c->I2CDAT = u8Data;
}





 
static __inline uint32_t I2C_GET_STATUS(I2C_T *i2c)
{
    return ( i2c->I2CSTATUS );
}







 
static __inline uint32_t I2C_GET_TIMEOUT_FLAG(I2C_T *i2c)
{
    return ( (i2c->I2CTOC & (1ul << 0)) == (1ul << 0) ? 1:0  );
}







 
static __inline uint32_t I2C_GET_WAKEUP_FLAG(I2C_T *i2c)
{
    return ( (i2c->I2CSTATUS2 & (1ul << 0)) == (1ul << 0) ? 1:0  );
}

uint32_t I2C_Open(I2C_T *i2c, uint32_t u32BusClock);
void I2C_Close(I2C_T *i2c);
void I2C_ClearTimeoutFlag(I2C_T *i2c);
void I2C_Trigger(I2C_T *i2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Si, uint8_t u8Ack);
void I2C_DisableInt(I2C_T *i2c);
void I2C_EnableInt(I2C_T *i2c);
uint32_t I2C_GetClockBusFreq(I2C_T *i2c);
uint32_t I2C_SetClockBusFreq(I2C_T *i2c, uint32_t u32BusClock);
uint32_t I2C_GetIntFlag(I2C_T *i2c);
uint32_t I2C_GetStatus(I2C_T *i2c);
uint32_t I2C_GetData(I2C_T *i2c);
void I2C_SetData(I2C_T *i2c, uint8_t u8Data);
void I2C_SetSlaveAddr(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddr, uint8_t u8GCMode);
void I2C_SetSlaveMask(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddrMask);
void I2C_EnableTimeout(I2C_T *i2c, uint8_t u8LongTimeout);
void I2C_DisableTimeout(I2C_T *i2c);
void I2C_EnableWakeup(I2C_T *i2c);
void I2C_DisableWakeup(I2C_T *i2c);

   

   

   







 
#line 3465 ".\\CMSIS\\Mini51Series.h"
#line 1 ".\\StdDriver\\inc\\pwm.h"
 








  











 



 



 
#line 49 ".\\StdDriver\\inc\\pwm.h"

   




 





 






 






 






 






 

 




 








 
#line 114 ".\\StdDriver\\inc\\pwm.h"









 














 










 










 










 




uint32_t PWM_ConfigOutputChannel(PWM_T *pwm,
                                  uint32_t u32ChannelNum, 
                                  uint32_t u32Frequncy, 
                                  uint32_t u32DutyCycle);
void PWM_Start(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_Stop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_ForceStop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
void PWM_DisableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearADCTriggerFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
uint32_t PWM_GetADCTriggerFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableFaultBreak(PWM_T *pwm, 
                           uint32_t u32ChannelMask, 
                           uint32_t u32LevelMask, 
                           uint32_t u32BreakSource);
void PWM_ClearFaultBreakFlag(PWM_T *pwm, uint32_t u32BreakSource);
void PWM_EnableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_DisableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void PWM_DisableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableDutyInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void PWM_DisableDutyInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearDutyIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetDutyIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableFaultBreakInt(PWM_T *pwm, uint32_t u32BreakSource);
void PWM_DisableFaultBreakInt(PWM_T *pwm, uint32_t u32BreakSource);
void PWM_ClearFaultBreakIntFlag(PWM_T *pwm, uint32_t u32BreakSource);
uint32_t PWM_GetFaultBreakIntFlag(PWM_T *pwm, uint32_t u32BreakSource);
void PWM_EnablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType);
void PWM_DisablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);



   

   

   







 
#line 3466 ".\\CMSIS\\Mini51Series.h"
#line 1 ".\\StdDriver\\inc\\spi.h"
 








  











 



 



 













#line 51 ".\\StdDriver\\inc\\spi.h"


   




 





 
static __inline void SPI_ABORT_3WIRE_TRANSFER(SPI_T *spi)
{
  spi->CNTRL2 |= (1ul << 9);
}





 
static __inline void SPI_CLR_3WIRE_START_INT_FLAG(SPI_T *spi)
{
  spi->STATUS |= (1ul << 11);
}





 
static __inline void SPI_CLR_UNIT_TRANS_INT_FLAG(SPI_T *spi)
{
  spi->STATUS |= (1ul << 16);
}





 
static __inline void SPI_DISABLE_3WIRE_MODE(SPI_T *spi)
{
  spi->CNTRL2 &= ~(1ul << 8);
}





 
static __inline void SPI_ENABLE_3WIRE_MODE(SPI_T *spi)
{
  spi->CNTRL2 |= (1ul << 8);
}





 
static __inline uint32_t SPI_GET_RX_FIFO_COUNT(SPI_T *spi)
{
  return ( ((spi->STATUS & (0xFul << 12)) >> 12) & 0xf );
}







 
static __inline uint32_t SPI_GET_RX_FIFO_EMPTY_FLAG(SPI_T *spi)
{
  return ( (spi->STATUS & (1ul << 24)) == (1ul << 24) ? 1:0);
}







 
static __inline uint32_t SPI_GET_TX_FIFO_EMPTY_FLAG(SPI_T *spi)
{
  return ( (spi->STATUS & (1ul << 26)) == (1ul << 26) ? 1:0);
}





 
static __inline uint32_t SPI_READ_RX(SPI_T *spi)
{
  return (spi->RX);
}






 
static __inline void SPI_WRITE_TX(SPI_T *spi, uint32_t u32TxData)
{
  spi->TX = u32TxData;
}





 
static __inline void SPI_SET_SS_HIGH(SPI_T *spi)
{
  spi->SSR &= ~(1ul << 3);  
  spi->SSR |= (1ul << 5);
  spi->SSR &= ~((1ul << 2) | (1ul << 0));
}





 
static __inline void SPI_SET_SS_LOW(SPI_T *spi)
{
  spi->SSR &= ~(1ul << 3);  
  spi->SSR |= (1ul << 5) | (1ul << 2) | (1ul << 0);  
}





 
static __inline void SPI_ENABLE_BYTE_REORDER(SPI_T *spi)
{
   spi->CNTRL |= (3ul << 19);
}





 
static __inline void SPI_DISABLE_BYTE_REORDER(SPI_T *spi)
{
   spi->CNTRL &= ~(3ul << 19);
}






 
static __inline void SPI_SET_SUSPEND_CYCLE(SPI_T *spi, uint32_t u32SuspCycle)
{
   spi->CNTRL = (spi->CNTRL & ~(0xFul << 12)) | (u32SuspCycle << 12);
}





 
static __inline void SPI_SET_LSB_FIRST(SPI_T *spi)
{
   spi->CNTRL |= (1ul << 10);
}





 
static __inline void SPI_SET_MSB_FIRST(SPI_T *spi)
{
   spi->CNTRL &= ~(1ul << 10);
}






 
static __inline void SPI_SET_DATA_WIDTH(SPI_T *spi, uint32_t u32Width)
{
   if(u32Width == 32)
        u32Width = 0;
        
   spi->CNTRL = (spi->CNTRL & ~(0x1Ful << 3)) | (u32Width << 3);
}







 
static __inline uint32_t SPI_IS_BUSY(SPI_T *spi)
{
  return ( (spi->CNTRL & (1ul << 0)) == (1ul << 0) ? 1:0);
}





 
static __inline void SPI_TRIGGER(SPI_T *spi)
{
  spi->CNTRL |= (1ul << 0);
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

   

   

   







 
#line 3467 ".\\CMSIS\\Mini51Series.h"
#line 1 ".\\StdDriver\\inc\\timer.h"
 








 











 



 



 

#line 45 ".\\StdDriver\\inc\\timer.h"


   




 






 








 







 







 
static __inline void TIMER_Start(TIMER_T *timer)
{
    timer->TCSR |= (1ul << 30);
}





 
static __inline void TIMER_Stop(TIMER_T *timer)
{
    timer->TCSR &= ~(1ul << 30);
}






 
static __inline void TIMER_EnableWakeup(TIMER_T *timer)
{
    timer->TCSR |= (1ul << 23);
}





 
static __inline void TIMER_DisableWakeup(TIMER_T *timer)
{
    timer->TCSR &= ~(1ul << 23);
}






 
static __inline void TIMER_EnableCaptureDebounce(TIMER_T *timer)
{
    timer->TEXCON |= (1ul << 6);
}





 
static __inline void TIMER_DisableCaptureDebounce(TIMER_T *timer)
{
    timer->TEXCON &= ~(1ul << 6);
}






 
static __inline void TIMER_EnableEventCounterDebounce(TIMER_T *timer)
{
    timer->TEXCON |= (1ul << 7);
}





 
static __inline void TIMER_DisableEventCounterDebounce(TIMER_T *timer)
{
    timer->TEXCON &= ~(1ul << 7);
}





 
static __inline void TIMER_EnableInt(TIMER_T *timer)
{
    timer->TCSR |= (1ul << 29);
}





 
static __inline void TIMER_DisableInt(TIMER_T *timer)
{
    timer->TCSR &= ~(1ul << 29);
}





 
static __inline void TIMER_EnableCaptureInt(TIMER_T *timer)
{
    timer->TEXCON |= (1ul << 5);
}





 
static __inline void TIMER_DisableCaptureInt(TIMER_T *timer)
{
    timer->TEXCON &= ~(1ul << 5);
}







 
static __inline uint32_t TIMER_GetIntFlag(TIMER_T *timer)
{
    return(timer->TISR & (1ul << 0) ? 1 : 0);
}





 
static __inline void TIMER_ClearIntFlag(TIMER_T *timer)
{
    timer->TISR = (1ul << 0);
}







 
static __inline uint32_t TIMER_GetCaptureIntFlag(TIMER_T *timer)
{
    return timer->TEXISR;
}





 
static __inline void TIMER_ClearCaptureIntFlag(TIMER_T *timer)
{
    timer->TEXISR = (1ul << 0);
}







 
static __inline uint32_t TIMER_GetWakeupFlag(TIMER_T *timer)
{
    return (timer->TISR & (1ul << 1) ? 1 : 0);
}





 
static __inline void TIMER_ClearWakeupFlag(TIMER_T *timer)
{
    timer->TISR = (1ul << 1);
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


   

   

   







 
#line 3468 ".\\CMSIS\\Mini51Series.h"
#line 1 ".\\StdDriver\\inc\\uart.h"
 








  











 



 



 

 
 
 











 
 
 
















 
 
 



 
 
 





   




 








 









 











 









 










 








 









 









 








 









 

                                     








 











 











 









 









 


















 

















 




















                                                                                       








  
__inline void UART_CLEAR_RTS(UART_T* uart)  
{
    uart->MCR |= (1ul << 9);
    uart->MCR &= (1ul << 1);
}






  
__inline void UART_SET_RTS(UART_T* uart)
{
    uart->MCR |= (1ul << 9) | (1ul << 1);
}






                                                                                                                                  








     



void UART_ClearIntFlag(UART_T* uart , uint32_t u32InterruptFlag);
void UART_Close(UART_T* uart );
void UART_DisableFlowCtrl(UART_T* uart );
void UART_DisableInt(UART_T*  uart, uint32_t u32InterruptFlag );
void UART_EnableFlowCtrl(UART_T* uart );
void UART_EnableInt(UART_T*  uart, uint32_t u32InterruptFlag );
void UART_Open(UART_T* uart, uint32_t u32baudrate);
int32_t UART_ReadBytes(UART_T* uart, uint8_t *pu8RxBuf, uint32_t u32ReadBytes);
void UART_SetLine_Config(UART_T* uart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t  u32stop_bits);
void UART_SetTimeoutCnt(UART_T* uart, uint32_t u32TOC);
void UART_SelectIrDAMode(UART_T* uart, uint32_t u32Buadrate, uint32_t u32Direction);
void UART_SelectRS485Mode(UART_T* uart, uint32_t u32Mode, uint32_t u32Addr);
uint32_t UART_WriteBytes(UART_T* uart,uint8_t *pu8TxBuf, uint32_t u32WriteBytes);


   

   

   







 








#line 3469 ".\\CMSIS\\Mini51Series.h"
#line 1 ".\\StdDriver\\inc\\wdt.h"
 








  











 



 



 
#line 39 ".\\StdDriver\\inc\\wdt.h"

   
    
    


 



 




 


 







 







 







 






 



void  WDT_Open(uint32_t u32TimeoutInterval,
                  uint32_t u32ResetDelay,
                  uint32_t u32EnableReset,
                  uint32_t u32EnableWakeup);  
void WDT_Close(void);

void WDT_EnableInt(void);
void WDT_DisableInt(void);

   

   

   







 
#line 3470 ".\\CMSIS\\Mini51Series.h"



 
#line 5 "User\\global.h"



#line 16 "User\\global.h"











extern volatile uint32_t unSystemTick;

typedef struct
{
	struct
	{
		volatile uint16_t bMotorNeedToRun:1;
		volatile uint16_t bRotateDirection:1;
	}MCR;
	struct
	{
		volatile uint16_t bMotorPowerOn:1;
		volatile uint16_t bZeroCrossDetecting:1;
		volatile uint16_t bLocked:1;
		volatile uint16_t bThisPhaseDetectedZX:1;
		volatile uint16_t bNewComFrameReceived:1;
	}MSR;
	volatile uint16_t	unMissedZXD_CNT;
	volatile uint16_t	unSuccessZXD_CNT;
	volatile uint16_t unLocatingDuty;		 
	volatile uint16_t unRampUpDuty;			 
	volatile uint16_t unTargetDuty;			 
	volatile uint16_t unActualDuty;			 
	volatile uint16_t unLocatingPeriod;	 
	volatile uint16_t unRESERVE_1;				 
	volatile uint32_t unRampUpPeriod;		 
	volatile uint32_t unActualPeriod;		 
	volatile uint32_t unPhaseChangeCNT;	 
	volatile uint16_t unRPM;							 
	volatile uint16_t unBattery;					 
	volatile uint16_t unCurrent;					 
	volatile uint16_t unRESERVE_2;				 
} MOTOR_T;

typedef union
{
	uint16_t unValue[sizeof(MOTOR_T)/sizeof(uint16_t)];
	MOTOR_T structMotor;
} MOTOR_UNION_T;


#line 1 "User\\BLDCSensorLess.h"



#line 1 "User\\global.h"
#line 5 "User\\BLDCSensorLess.h"

typedef enum {
	ENUM_TIM1_AVOID_ZXD = 0,
	ENUM_TIM1_ZXD_FILTER

}ENUM_TIM1_USAGE;


#line 20 "User\\BLDCSensorLess.h"




#line 38 "User\\BLDCSensorLess.h"







#line 53 "User\\BLDCSensorLess.h"

#line 60 "User\\BLDCSensorLess.h"

#line 67 "User\\BLDCSensorLess.h"




											
											

















#line 99 "User\\BLDCSensorLess.h"
















#line 277 "User\\BLDCSensorLess.h"
extern volatile MOTOR_UNION_T tMotor;	
extern volatile ENUM_TIM1_USAGE FLAG_TIM1_USEAGE;
extern volatile uint32_t unLastZXDetectedTime;
extern volatile uint32_t unZXMatchCNT;

											
											

extern uint8_t unCurrentPhase;
extern uint8_t FLAG_PHASE_CHANGED;
extern __inline void BLDC_stopMotor(void);
extern void BLDC_SensorLessManager(void);
#line 69 "User\\global.h"
#line 1 "User\\Communication.h"
#line 70 "User\\global.h"
#line 1 "User\\Error.h"



#line 5 "User\\Error.h"
















 




typedef enum {
	ERR_NULL = 0,
	ERR_CURRENT_WARNING,	
	ERR_COMMUNICATION_FAIL,
	ERR_LOCATE_FAIL,
	ERR_RAMPUP_FAIL,
	ERR_BATTERY_LOW,
	ERR_INTERNAL,	
	ERR_CURRENT_BURNING,	
	ERR_BRD_FAULT
} ENUM_ERROR_LEVEL;





#line 62 "User\\Error.h"

extern uint32_t unErrorMaster;
extern void delay(uint32_t unDelayMs);
extern void resetError(ENUM_ERROR_LEVEL enumErrorType);
extern void setError(ENUM_ERROR_LEVEL enumErrorType);
extern void clearError(void);
extern void ERR_Manager(void);

#line 71 "User\\global.h"
#line 1 "User\\Protection.h"



#line 5 "User\\Protection.h"






#line 18 "User\\Protection.h"





									


											










#line 48 "User\\Protection.h"

extern void PTC_checkMotor(void);

#line 72 "User\\global.h"
#line 5 "User\\Communication.h"





		uint16_t CRC_TABLE16[] = {0x0000,0x8005,0x800F,0x000A,0x801B,0x001E,0x0014,0x8011,0x8033,0x0036,0x003C,0x8039,0x0028,0x802D,0x8027,0x0022,
		0x8063,0x0066,0x006C,0x8069,0x0078,0x807D,0x8077,0x0072,0x0050,0x8055,0x805F,0x005A,0x804B,0x004E,0x0044,0x8041,
		0x80C3,0x00C6,0x00CC,0x80C9,0x00D8,0x80DD,0x80D7,0x00D2,0x00F0,0x80F5,0x80FF,0x00FA,0x80EB,0x00EE,0x00E4,0x80E1,
		0x00A0,0x80A5,0x80AF,0x00AA,0x80BB,0x00BE,0x00B4,0x80B1,0x8093,0x0096,0x009C,0x8099,0x0088,0x808D,0x8087,0x0082,
		0x8183,0x0186,0x018C,0x8189,0x0198,0x819D,0x8197,0x0192,0x01B0,0x81B5,0x81BF,0x01BA,0x81AB,0x01AE,0x01A4,0x81A1,
		0x01E0,0x81E5,0x81EF,0x01EA,0x81FB,0x01FE,0x01F4,0x81F1,0x81D3,0x01D6,0x01DC,0x81D9,0x01C8,0x81CD,0x81C7,0x01C2,
		0x0140,0x8145,0x814F,0x014A,0x815B,0x015E,0x0154,0x8151,0x8173,0x0176,0x017C,0x8179,0x0168,0x816D,0x8167,0x0162,
		0x8123,0x0126,0x012C,0x8129,0x0138,0x813D,0x8137,0x0132,0x0110,0x8115,0x811F,0x011A,0x810B,0x010E,0x0104,0x8101,
		0x8303,0x0306,0x030C,0x8309,0x0318,0x831D,0x8317,0x0312,0x0330,0x8335,0x833F,0x033A,0x832B,0x032E,0x0324,0x8321,
		0x0360,0x8365,0x836F,0x036A,0x837B,0x037E,0x0374,0x8371,0x8353,0x0356,0x035C,0x8359,0x0348,0x834D,0x8347,0x0342,
		0x03C0,0x83C5,0x83CF,0x03CA,0x83DB,0x03DE,0x03D4,0x83D1,0x83F3,0x03F6,0x03FC,0x83F9,0x03E8,0x83ED,0x83E7,0x03E2,
		0x83A3,0x03A6,0x03AC,0x83A9,0x03B8,0x83BD,0x83B7,0x03B2,0x0390,0x8395,0x839F,0x039A,0x838B,0x038E,0x0384,0x8381,
		0x0280,0x8285,0x828F,0x028A,0x829B,0x029E,0x0294,0x8291,0x82B3,0x02B6,0x02BC,0x82B9,0x02A8,0x82AD,0x82A7,0x02A2,
		0x82E3,0x02E6,0x02EC,0x82E9,0x02F8,0x82FD,0x82F7,0x02F2,0x02D0,0x82D5,0x82DF,0x02DA,0x82CB,0x02CE,0x02C4,0x82C1,
		0x8243,0x0246,0x024C,0x8249,0x0258,0x825D,0x8257,0x0252,0x0270,0x8275,0x827F,0x027A,0x826B,0x026E,0x0264,0x8261,
		0x0220,0x8225,0x822F,0x022A,0x823B,0x023E,0x0234,0x8231,0x8213,0x0216,0x021C,0x8219,0x0208,0x820D,0x8207,0x0202};
		
		typedef enum {
			COMM_READ_MCR = 0,
			COMM_READ_MSR,
			COMM_READ_MISS_ZXD_CNT,
			COMM_READ_SCS_ZXD_CNT,
			COMM_READ_LOCATING_DUTY,
			COMM_READ_RAMP_UP_DUTY,
			COMM_READ_TARGET_DUTY,
			COMM_READ_ACTUAL_DUTY,
			COMM_READ_LOCATING_PERIOD,
			COMM_READ_RESERVED_1,
			COMM_READ_RAMP_UP_PERIOD_LOW,
			COMM_READ_RAMP_UP_PERIOD_HIGH,
			COMM_READ_ACTUAL_PERIOD_LOW,
			COMM_READ_ACTUAL_PERIOD_HIGH,
			COMM_READ_PHASE_CHANGE_CNT_LOW,
			COMM_READ_PHASE_CHANGE_CNT_HIGH,
			COMM_READ_RPM,
			COMM_READ_BATTERY,
			COMM_READ_CURRENT,
			COMM_READ_RESERVE_2,
			COMM_READ_MAX
		} ENUM_COMM_READ_CMD;

		typedef enum {
			COMM_WRITE_DUMMY = 0,
			COMM_WRITE_MOTOR_NEED_TO_RUN,
			COMM_WRITE_ROTATE_DIRECTION,
			COMM_WRITE_LOCATING_DUTY,
			COMM_WRITE_RAMP_UP_DUTY,
			COMM_WRITE_TARGET_DUTY,
			COMM_WRITE_LOCATING_PERIOD,
			COMM_WRITE_RAMP_UP_PERIOD,
			COMM_WRITE_CMD_MAX
		} ENUM_COMM_WRITE_CMD;
		uint32_t unValidFrameCNT;












										






		
#line 89 "User\\Communication.h"




















 uint32_t unCOM_SPI_TransCNT;
 uint32_t unCOM_SPI_TransErrCNT;
 uint16_t unCOM_SPI_ReadData[4];	
 uint16_t unRegisterValue;	

 uint8_t FlagRegisterNeedWrite;

 void COMM_Manager(void);
#line 13 "User\\Communication.c"
#line 1 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
 
 
 
 




 








 












#line 38 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"


  



    typedef unsigned int size_t;    
#line 54 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"




extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 193 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 209 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 232 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 247 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 270 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 502 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"



 

#line 14 "User\\Communication.c"













 







 

uint16_t calCRC16(uint8_t* pBytes, uint32_t unLength)
{
	uint16_t crc = 0;
	uint32_t unIndex;
	uint8_t unPosInTable;
	
	for (unIndex = 0; unIndex < unLength; unIndex++)
	{
		 
		unPosInTable = (uint8_t)((crc >> 8) ^ (*(pBytes + ((((unIndex) & 0x00000001) == 0) ? ((unIndex) + 1) : ((unIndex) - 1)))));  
		 
		crc = (uint16_t)((crc << 8) ^ (uint16_t)(CRC_TABLE16[unPosInTable]));
	}

	return crc;
}

uint32_t unSPI_TX_WR_CNT = 0;
int32_t nReadCommandHandler(uint16_t* pCOM_Buff)
{
	if (((pCOM_Buff[0]) & (0x7FFF)) < COMM_READ_MAX)
	{
		SPI_WRITE_TX(((SPI_T *) (((uint32_t)0x40000000) + 0x30000)), (tMotor.unValue[((pCOM_Buff[0]) & (0x7FFF))] << 16) + 
			calCRC16((uint8_t*)(&(tMotor.unValue[((pCOM_Buff[0]) & (0x7FFF))])), 2) );
		unSPI_TX_WR_CNT++;
		return 0;
	}
	else
	{
		return -1;
	}
}

int32_t nWriteCommandHandler(uint16_t* pCOM_Buff)
{
	switch(((pCOM_Buff[0]) & (0x7FFF)))
	{
	case COMM_WRITE_MOTOR_NEED_TO_RUN:
	case COMM_WRITE_ROTATE_DIRECTION:
		tMotor.structMotor.MCR.bMotorNeedToRun = pCOM_Buff[1];
		break;
	case COMM_WRITE_LOCATING_DUTY:
		tMotor.structMotor.unLocatingDuty = pCOM_Buff[1];
		break;
	case COMM_WRITE_RAMP_UP_DUTY:
		tMotor.structMotor.unRampUpDuty = pCOM_Buff[1];
		break;
	case COMM_WRITE_TARGET_DUTY:
		tMotor.structMotor.unTargetDuty = pCOM_Buff[1];
		break;
	case COMM_WRITE_LOCATING_PERIOD:
		tMotor.structMotor.unLocatingPeriod = pCOM_Buff[1];
		break;
	case COMM_WRITE_RAMP_UP_PERIOD:
		tMotor.structMotor.unRampUpPeriod = pCOM_Buff[1] + (pCOM_Buff[2] << 16);
			break;
	default:
		return -1;
	}
	return 0;
}


void COMM_Manager(void)
{
	static uint16_t unCOM_Buff[4];
	static uint32_t unLastFrameCNT = 0;
	static uint32_t unLastCheckTime = 0;
	
	if (tMotor.structMotor.MSR.bNewComFrameReceived == (1))
	{
		memcpy(unCOM_Buff, unCOM_SPI_ReadData, 4);
		tMotor.structMotor.MSR.bNewComFrameReceived = (0);
		if (calCRC16((uint8_t *)unCOM_Buff, ((((unCOM_Buff[0]) & (0x8000)) == (0x8000)) ? ((2 - 1) << 1) : ((4 - 1) << 1))) ==
				((((unCOM_Buff[0]) & (0x8000)) == (0x8000)) ? unCOM_Buff[2 - 1] : unCOM_Buff[4 - 1]))
		{
			unValidFrameCNT++;
			
			if ((((unCOM_Buff[0]) & (0x8000)) == (0x8000)))
			{
				nReadCommandHandler(unCOM_Buff);
			}
			else
			{
				nWriteCommandHandler(unCOM_Buff);
			}
		}
		else
		{
			unCOM_SPI_TransErrCNT++;
		}
	}
	
	
	if ((uint32_t)(unSystemTick - unLastCheckTime) > 500)
	{
		unLastCheckTime = unSystemTick;
		if ((uint32_t)(unValidFrameCNT - unLastFrameCNT) < 1)
		{
			BLDC_stopMotor();
			setError(ERR_COMMUNICATION_FAIL);
		}
		unLastFrameCNT = unValidFrameCNT;
	}
	
	if (unCOM_SPI_TransErrCNT > 6)
	{
		BLDC_stopMotor();
		setError(ERR_COMMUNICATION_FAIL);
	}
}
