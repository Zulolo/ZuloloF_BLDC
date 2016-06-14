#line 1 "User\\main.c"









    
#line 1 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));


#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int __ARM_vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int __ARM_vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int __ARM_vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));
   








 

extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 1021 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"



 

#line 12 "User\\main.c"


#line 1 "User\\main.h"



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



#line 18 "User\\global.h"











extern volatile uint32_t unSystemTick;

typedef struct
{
	volatile uint16_t	unNULL;
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
#line 75 "User\\global.h"
#line 1 "User\\Communication.h"



#line 5 "User\\Communication.h"

#line 82 "User\\Communication.h"









										






		
#line 107 "User\\Communication.h"




















extern uint16_t unRegisterValue;	

extern uint8_t FlagRegisterNeedWrite;
extern uint16_t unReadValueCRC;
extern void COMM_Manager(void);
#line 76 "User\\global.h"
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

#line 77 "User\\global.h"
#line 1 "User\\Protection.h"



#line 5 "User\\Protection.h"






#line 18 "User\\Protection.h"





									


											










#line 48 "User\\Protection.h"

extern void PTC_checkMotor(void);

#line 78 "User\\global.h"
#line 5 "User\\main.h"


									
									
									
								  
								  

									



#line 35 "User\\main.h"



												
uint8_t iTestSpeedSequenIndex = 0;
uint32_t iTestSpeedLastTime = 0;
const uint16_t iTestSpeedSequence[] = {250, 300, 350, 400, 450, 300, 400, 250, 200, 450, 200};

volatile uint32_t unSystemTick = 0;





#line 15 "User\\main.c"

void initClk()
{
	 
	((CLK_T *) (((uint32_t)0x50000000) + 0x00200))->PWRCON |= (1ul << 2);

	 
	CLK_WaitClockReady((1ul << 4));

	
	CLK_SetHCLK(0x07UL, ((1)-1));
	

	
	CLK_SetSysTickClockSrc(0x38UL);

	
	CLK_SetModuleClock(((0x0<<31)|(0x1<<29)|(0x3<<25)|( 0<<20)|(0x3<<18)|(0x0<<10)|(31<<5)|0 ), 0x00000002UL, 1);
	CLK_SetModuleClock(((0x0<<31)|(0x1<<29)|(0x3<<25)|( 2<<20)|(0x0<<18)|(0xFF<<10)|(16<<5)|28), 0x0000000CUL, (((221)-1) << 16));
	CLK_SetModuleClock(((0x0<<31)|(0x1<<29)|(0x3<<25)|(28<<20)|(0x3<<18)|(0x0<<10)|(31<<5)|20), 0x20000000UL, 1);
	CLK_SetModuleClock(((0x0<<31)|(0x1<<29)|(0x3<<25)|(30<<20)|(0x3<<18)|(0x0<<10)|(31<<5)|21), 0x80000000UL, 1);
	CLK_SetModuleClock(((0x0<<31)|(0x3<<29)|(0x3<<25)|( 4<<20)|(0x3<<18)|(0x0<<10)|(31<<5)|22), 0x00000020UL, 1);
	CLK_SetModuleClock(((0x0<<31)|(0x1<<29)|(0x3<<25)|(24<<20)|(0x0<<18)|(0x0F<<10)|( 8<<5)|16), 0x02000000UL, (((1)-1) << 8));
	CLK_SetModuleClock(((0x0<<31)|(0x1<<29)|(0x7<<25)|( 8<<20)|(0x3<<18)|(0x0<<10)|(31<<5)|2), 0x00000200UL , 1);
	CLK_SetModuleClock(((0x0<<31)|(0x1<<29)|(0x7<<25)|(12<<20)|(0x3<<18)|(0x0<<10)|(31<<5)|3), 0x00002000UL , 1);

	((CLK_T *) (((uint32_t)0x50000000) + 0x00200))->CLKSEL1 |= (0x01 << 4);

	
	CLK_EnableModuleClock(((0x0<<31)|(0x1<<29)|(0x3<<25)|( 0<<20)|(0x3<<18)|(0x0<<10)|(31<<5)|0 ));
	CLK_EnableModuleClock(((0x0<<31)|(0x1<<29)|(0x7<<25)|( 8<<20)|(0x3<<18)|(0x0<<10)|(31<<5)|2));
	CLK_EnableModuleClock(((0x0<<31)|(0x1<<29)|(0x7<<25)|(12<<20)|(0x3<<18)|(0x0<<10)|(31<<5)|3));
	CLK_EnableModuleClock(((0x0<<31)|(0x1<<29)|(0x1<<25)|( 4<<20)|(0x3<<18)|(0x0<<10)|(31<<5)|12));
	CLK_EnableModuleClock(((0x0<<31)|(0x1<<29)|(0x3<<25)|(24<<20)|(0x0<<18)|(0x0F<<10)|( 8<<5)|16));
	CLK_EnableModuleClock(((0x0<<31)|(0x1<<29)|(0x3<<25)|(28<<20)|(0x3<<18)|(0x0<<10)|(31<<5)|20));
	CLK_EnableModuleClock(((0x0<<31)|(0x1<<29)|(0x3<<25)|(30<<20)|(0x3<<18)|(0x0<<10)|(31<<5)|21));
	CLK_EnableModuleClock(((0x0<<31)|(0x3<<29)|(0x3<<25)|( 4<<20)|(0x3<<18)|(0x0<<10)|(31<<5)|22));
	CLK_EnableModuleClock(((0x0<<31)|(0x1<<29)|(0x3<<25)|( 2<<20)|(0x0<<18)|(0xFF<<10)|(16<<5)|28));
	CLK_EnableModuleClock(((0x0<<31)|(0x3<<29)|(0x0<<25)|(31<<20)|(0x3<<18)|(0x0<<10)|(31<<5)|30));

}

void initIRQ()
{
	NVIC_EnableIRQ(TMR0_IRQn);
	NVIC_EnableIRQ(TMR1_IRQn);
	NVIC_EnableIRQ(SPI_IRQn);
	NVIC_EnableIRQ(ACMP_IRQn);
	NVIC_EnableIRQ(EINT0_IRQn);
	NVIC_EnableIRQ(ADC_IRQn);
	

	NVIC_SetPriority(TMR0_IRQn, 1);
	NVIC_SetPriority(TMR1_IRQn, 1);
	NVIC_SetPriority(SPI_IRQn, 3);
	NVIC_SetPriority(ACMP_IRQn, 2);
	NVIC_SetPriority(EINT0_IRQn, 0);
	NVIC_SetPriority(ADC_IRQn, 3);
	

	GPIO_EnableInt(((GPIO_T *) (((uint32_t)0x50000000) + 0x040C0)), (0x00000004), 0x00000001UL);
}

void initGPIO()
{
 
 
 
    
    GPIO_SetMode(((GPIO_T *) (((uint32_t)0x50000000) + 0x04140)), (0x00000010), 0x1UL);

    
    GPIO_SetMode(((GPIO_T *) (((uint32_t)0x50000000) + 0x040C0)), (0x00000001), 0x1UL);
    ((*((volatile uint32_t *)(((((uint32_t)0x50000000) + 0x04200)+(0x20*(3))) + ((0)<<2)))) = 0);

    
    GPIO_SetMode(((GPIO_T *) (((uint32_t)0x50000000) + 0x040C0)), (0x00000004), 0x0UL);

    
    GPIO_SetMode(((GPIO_T *) (((uint32_t)0x50000000) + 0x04080)), (0x00000004) | (0x00000008) | (0x00000010) | 
			     (0x00000020) | (0x00000040), 0x1UL);
    GPIO_SetMode(((GPIO_T *) (((uint32_t)0x50000000) + 0x04000)), (0x00000010), 0x1UL);

    
    GPIO_SetMode(((GPIO_T *) (((uint32_t)0x50000000) + 0x04000)), (0x00000080) | (0x00000002) | (0x00000020), 0x0UL);
    GPIO_SetMode(((GPIO_T *) (((uint32_t)0x50000000) + 0x04000)), (0x00000040), 0x1UL);

    
    GPIO_SetMode(((GPIO_T *) (((uint32_t)0x50000000) + 0x04000)), (0x00000001), 0x1UL);
    GPIO_SetMode(((GPIO_T *) (((uint32_t)0x50000000) + 0x04040)), (0x00000004), 0x0UL);

    
    GPIO_SetMode(((GPIO_T *) (((uint32_t)0x50000000) + 0x040C0)), (0x00000040), 0x1UL);
		GPIO_SetMode(((GPIO_T *) (((uint32_t)0x50000000) + 0x04140)), (0x00000001), 0x1UL);
		(*((volatile uint32_t *)(((((uint32_t)0x50000000) + 0x04200)+(0x20*(5))) + ((0)<<2)))) = 0;

    
    ((((GPIO_T *) (((uint32_t)0x50000000) + 0x04140)))->OFFD |= (0x00000008) << 16);
    GPIO_SetMode(((GPIO_T *) (((uint32_t)0x50000000) + 0x04140)), (0x00000008), 0x0UL);
    
    ((((GPIO_T *) (((uint32_t)0x50000000) + 0x040C0)))->OFFD |= (0x00000002) << 16);
    GPIO_SetMode(((GPIO_T *) (((uint32_t)0x50000000) + 0x040C0)), (0x00000002), 0x0UL);
	    
    
    ((((GPIO_T *) (((uint32_t)0x50000000) + 0x04040)))->OFFD |= ((0x00000001) | (0x00000008) | (0x00000020) | (0x00000010)) << 16);

    GPIO_SetMode(((GPIO_T *) (((uint32_t)0x50000000) + 0x04040)), (0x00000001) | (0x00000008) |
		 (0x00000020) | (0x00000010), 0x0UL);
 
 
 
     
    ((GCR_T *) (((uint32_t)0x50000000) + 0x00000))->P0_MFP &= ~0x01000101UL;
    ((GCR_T *) (((uint32_t)0x50000000) + 0x00000))->P0_MFP |= 0x00000101UL;  
    ((GCR_T *) (((uint32_t)0x50000000) + 0x00000))->P1_MFP &= ~0x00000404UL;
    ((GCR_T *) (((uint32_t)0x50000000) + 0x00000))->P1_MFP |= 0x00000400UL; 
	 
	 

    ((GCR_T *) (((uint32_t)0x50000000) + 0x00000))->P0_MFP |= (0x00000002UL | 0x00002000UL | 0x00004000UL | 0x00008000UL); 

	 
    ((GCR_T *) (((uint32_t)0x50000000) + 0x00000))->P5_MFP &= ~0x00000808UL;
    ((GCR_T *) (((uint32_t)0x50000000) + 0x00000))->P5_MFP |= 0x00000008UL;  

	 
    ((GCR_T *) (((uint32_t)0x50000000) + 0x00000))->P3_MFP &= ~0x00000202UL;
    ((GCR_T *) (((uint32_t)0x50000000) + 0x00000))->P3_MFP |= 0x00000202UL;  

	 
    ((GCR_T *) (((uint32_t)0x50000000) + 0x00000))->P3_MFP &= ~0x00004040UL;
    ((GCR_T *) (((uint32_t)0x50000000) + 0x00000))->P3_MFP |= 0x00004040UL;
    ((GCR_T *) (((uint32_t)0x50000000) + 0x00000))->P5_MFP &= ~0x00000101UL;
    ((GCR_T *) (((uint32_t)0x50000000) + 0x00000))->P5_MFP |= 0x00000000UL;

	 
    ((GCR_T *) (((uint32_t)0x50000000) + 0x00000))->P1_MFP &= ~0x00001010UL;
    ((GCR_T *) (((uint32_t)0x50000000) + 0x00000))->P1_MFP |= 0x00001010UL;  
	



     
    ((GCR_T *) (((uint32_t)0x50000000) + 0x00000))->P2_MFP &= ~(0x00000404UL | 0x00000808UL | 0x00001010UL | 0x00002020UL | 0x00004040UL);
    ((GCR_T *) (((uint32_t)0x50000000) + 0x00000))->P2_MFP = 0x00000400UL | 0x00000800UL | 0x00001000UL | 0x00002000UL |0x00004000UL;
    ((GCR_T *) (((uint32_t)0x50000000) + 0x00000))->P0_MFP &= ~0x00001010UL;
    ((GCR_T *) (((uint32_t)0x50000000) + 0x00000))->P0_MFP |= 0x00001010UL;  

	 
    ((GCR_T *) (((uint32_t)0x50000000) + 0x00000))->P3_MFP &= ~0x00000404UL;
    ((GCR_T *) (((uint32_t)0x50000000) + 0x00000))->P3_MFP |= 0x00000004UL;  
	
}

void configTIM(void)
{
	
	
    ((TIMER_T *) (((uint32_t)0x40000000) + 0x10000))->TCSR  =  (1ul << 26) | (1UL << 27) | (1ul << 17) | (1ul << 16) + 10;   
    ((TIMER_T *) (((uint32_t)0x40000000) + 0x10020))->TCSR  =  (1ul << 26) | (3UL << 27) | (1ul << 16) + 10; 
    
	
    
}

void configADC(void)
{

	ADC_SetExtraSampleTime(((ADC_T *) (((uint32_t)0x40000000) + 0xE0000)), 0 , (5UL));

	




	
	(((ADC_T *) (((uint32_t)0x40000000) + 0xE0000))->ADCR |= (1ul << 0));

	
	

	
	(((ADC_T *) (((uint32_t)0x40000000) + 0xE0000))->ADCMPR[0] = ((0) << 3) | (((1ul << 2))) | ((68) << 16) | (((12) - 1) << 8) | (1ul << 0));
	
	(((ADC_T *) (((uint32_t)0x40000000) + 0xE0000))->ADCMPR[1] = ((7) << 3) | ((0UL)) | ((348) << 16) | ((16 - 1) << 8) | (1ul << 0));    

	
	ADC_EnableInt(((ADC_T *) (((uint32_t)0x40000000) + 0xE0000)), ((1ul << 0)));
	ADC_EnableInt(((ADC_T *) (((uint32_t)0x40000000) + 0xE0000)), ((1ul << 1)));
	ADC_EnableInt(((ADC_T *) (((uint32_t)0x40000000) + 0xE0000)), ((1ul << 2)));

	(((ADC_T *) (((uint32_t)0x40000000) + 0xE0000))->ADCHER = (((ADC_T *) (((uint32_t)0x40000000) + 0xE0000))->ADCHER & ~(0xFFul << 0)) | ((0x01 << 7)));
	(((ADC_T *) (((uint32_t)0x40000000) + 0xE0000))->ADCR |= (1ul << 11));
}

void configSPI(void)
{
 
 
 
	
	


 
	
	 
	 
	SPI_Close(((SPI_T *) (((uint32_t)0x40000000) + 0x30000)));

	
	SPI_Open(((SPI_T *) (((uint32_t)0x40000000) + 0x30000)), ((1ul << 18)), ((1ul << 2)), 16, 0);




	SPI_SET_MSB_FIRST(((SPI_T *) (((uint32_t)0x40000000) + 0x30000)));

	
	
	((SPI_T *) (((uint32_t)0x40000000) + 0x30000))->SSR |= (1ul << 4);
	
	((SPI_T *) (((uint32_t)0x40000000) + 0x30000))->SSR &= (~(1ul << 2));

	 


	 
	SPI_EnableInt(((SPI_T *) (((uint32_t)0x40000000) + 0x30000)), (0x01));
		


	

	SPI_TRIGGER(((SPI_T *) (((uint32_t)0x40000000) + 0x30000)));
}







void initPWM(void)
{
    PWM_Stop(((PWM_T *) (((uint32_t)0x40000000) + 0x40000)), (0x3Ful));
    (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PPR = (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PPR & ~((0xFFul << 0) << (((0) >> 1) * 8))) | ((1) << (((0) >> 1) * 8)));

    (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PPR = (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PPR & ~((0xFFul << 0) << (((2) >> 1) * 8))) | ((1) << (((2) >> 1) * 8)));

    (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PPR = (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PPR & ~((0xFFul << 0) << (((4) >> 1) * 8))) | ((1) << (((4) >> 1) * 8)));

    (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CSR = (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CSR & ~((7ul << 0) << ((0) * 4))) | (((4UL)) << ((0) * 4)));
    (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CSR = (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CSR & ~((7ul << 0) << ((1) * 4))) | (((4UL)) << ((1) * 4)));
    (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CSR = (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CSR & ~((7ul << 0) << ((2) * 4))) | (((4UL)) << ((2) * 4)));
    (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CSR = (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CSR & ~((7ul << 0) << ((3) * 4))) | (((4UL)) << ((3) * 4)));
    (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CSR = (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CSR & ~((7ul << 0) << ((4) * 4))) | (((4UL)) << ((4) * 4)));
    (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CSR = (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CSR & ~((7ul << 0) << ((5) * 4))) | (((4UL)) << ((5) * 4)));







    ((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PCR = 0x08000000 | 0x00000002 |
		(1ul<<(4*(0))) | (8ul<<(4*(0))) |  
		(1ul<<(4*(1))) | (8ul<<(4*(1))) | 
		(1ul<<(4*(2))) | (8ul<<(4*(2))) | 
		(1ul<<(4*(3))) | (8ul<<(4*(3))) | 
		(1ul<<(4*(4))) | (8ul<<(4*(4))) | 
		(1ul<<(4*(5))) | (8ul<<(4*(5))) |
		(4ul<<(4*(0))) | (4ul<<(4*(2))) | (4ul<<(4*(4))) |
		(4ul<<(4*(1))) | (4ul<<(4*(3))) | (4ul<<(4*(5)));


    ((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CMR[0] = 0;
    ((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CMR[1] = 0;
    ((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CMR[2] = 0;
    ((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CMR[3] = 0;
    ((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CMR[4] = 0;
    ((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CMR[5] = 0;
    ((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CNR[0] = (884-1); 
    ((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CNR[1] = (884-1);    
    ((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CNR[2] = (884-1);                                    
    ((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CNR[3] = (884-1);
    ((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CNR[4] = (884-1); 
    ((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->CNR[5] = (884-1);
    PWM_EnableOutput(((PWM_T *) (((uint32_t)0x40000000) + 0x40000)), (0x3Ful));
    (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PIER = 0);	
    BLDC_stopMotor();
    

    

			    
			    
    ((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PHCHGMASK = 0x00000100;	
}

void initSys(void)
{
     
    SYS_UnlockReg();
    
     
    initClk();
    
    
     
     
    SystemCoreClockUpdate(); 

     
    
    SysTick_Config((110592 - 1));
    
     
    
    initPWM();
      
     
    initGPIO();

     
    

     
    configADC();

     
    configTIM();

     
    configSPI();

     
    initIRQ();

     
    SYS_LockReg();
                         
}    

void initEnv(void)
{
	tMotor.structMotor.unCommOK_CNT = 0;
	tMotor.structMotor.unCommErrCNT = 0;
	unZXMatchCNT = 0;
	tMotor.structMotor.MSR.bNewComFrameReceived = (0);

}
uint32_t unWTF_Value = 0xA5D4;
int main()
{

	initSys();
	initEnv();

	PTC_checkMotor();

	 
	
	
	((*((volatile uint32_t *)(((((uint32_t)0x50000000) + 0x04200)+(0x20*(3))) + ((0)<<2)))) = 0); (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PIER = 0); (TIMER_Stop(((TIMER_T *) (((uint32_t)0x40000000) + 0x10000)))); (TIMER_Stop(((TIMER_T *) (((uint32_t)0x40000000) + 0x10020)))); (TIMER_DisableInt(((TIMER_T *) (((uint32_t)0x40000000) + 0x10000)))); (TIMER_DisableInt(((TIMER_T *) (((uint32_t)0x40000000) + 0x10020)))); (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PHCHGNXT = (0x000000FFul)); (((PWM_T *) (((uint32_t)0x40000000) + 0x40000))->PHCHG = (0x000000FFul));

    







     

	while(1)
	{

		COMM_Manager();

	}

}

 
