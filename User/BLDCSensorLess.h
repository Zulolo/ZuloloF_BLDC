#ifndef __BLDC_SENSOR_LESS_H__
#define __BLDC_SENSOR_LESS_H__

#include "global.h"
	typedef struct
	{
		struct
		{
			__IO uint16_t MotorNeedToRun:1;
			__IO uint16_t RotateDirection:1;
		}MCR;
		struct
		{
			__IO uint16_t MotorPowerOn:1;
			__IO uint16_t ZeroCrossDetecting:1;
			__IO uint16_t Locked:1;
			__IO uint16_t ThisPhaseDetectedZX:1;
			__IO uint16_t MissedZXD_CNT:8;
			__IO uint16_t SuccessZXD_CNT:8;
		}MSR;
//		__IO uint16_t  MCR;		/*!<  Motor Control  */
//		__IO uint16_t  MSR;		/*!<  Motor Status  */
		//  __IO uint32_t  ACNR;       /*!<  PWM Actual Counter Register  */
		//  __IO uint32_t  STCNR;        /*!<  PWM Start Counter Register  */
		//  __IO uint32_t  SBCNR;        /*!<  PWM Stable Counter Register  */
		__IO uint16_t  LCT_DUTY;	/*!<  PWM Locating Duty  */
		__IO uint16_t  RU_DUTY;		/*!<  PWM Ramp Up Start Duty  */
		__IO uint16_t  TGT_DUTY;	/*!<  PWM Target (Locked State) Duty  */
		__IO uint16_t  ACT_DUTY;	/*!<  PWM Actual Duty  */
		__IO uint16_t  LCT_PERIOD;	/*!<  Locating State One Phase Period  */
		__IO uint32_t  RU_PERIOD;	/*!<  Ramp Up Start One Phase Period  */
		__IO uint32_t  ACT_PERIOD;	/*!<  Actual One Phase Period  */
		__IO uint32_t  PHASE_CHANGE_CNT;	/*!<  Phase changed counter  */
		__IO uint16_t  RPM;			/*!<  Actual RPM  */
		__IO uint16_t  RESERVE;		/*!<  Reserve for future use (round up 32bits) */
		__IO uint16_t  BATTERY;		/*!<  Battery Voltage  */
		__IO uint16_t  CURRENT;		/*!<  Current  */
	} MOTOR_T;

	typedef union
	{
		uint16_t iValue[sizeof(MOTOR_T)/sizeof(uint16_t)];
		MOTOR_T structMotor;
	} MOTOR_UNION_T;

	typedef enum {
		ENUM_TIM1_AVOID_ZXD = 0,
		ENUM_TIM1_ZXD_FILTER//,
//		ENUM_TIM1_START_ZXD
	}ENUM_TIM1_USAGE;

//	#define ACMP0_FALLING_ENABLE		(ACMP->CMPCR[0] = ACMP_CMPCR_FALLING_Msk | ACMP_CMPCR_HYSEN_Msk | ACMP_CMPCR_ACMPIE_Msk | ACMP_CMPCR_ACMPEN_Msk)
//	#define ACMP0_RISING_ENABLE			(ACMP->CMPCR[0] = ACMP_CMPCR_RISING_Msk | ACMP_CMPCR_HYSEN_Msk | ACMP_CMPCR_ACMPIE_Msk | ACMP_CMPCR_ACMPEN_Msk)
//	#define ACMP0_EDGE_SELECT			((PWM->PHCHG & DETEC_UP) ? (ACMP_CMPCR_RISING_Msk) : (ACMP_CMPCR_FALLING_Msk))
	#define DETEC_UP_POS				7
	#define DETEC_UP					(1ul << DETEC_UP_POS)
	#define ACMP0_EDGE_MATCH			(((PWM->PHCHG & DETEC_UP) >> DETEC_UP_POS) == (( ACMP->CMPSR & ACMP_CMPSR_ACMPCO0_Msk) >> ACMP_CMPSR_ACMPCO0_Pos))
	#define ACMP0_ENABLE				(ACMP->CMPCR[0] |= ACMP_CMPCR_HYSEN_Msk | ACMP_CMPCR_ACMPEN_Msk)
	#define ACMP0_INT_ENABLE			(ACMP->CMPCR[0] |= ACMP_CMPCR_ACMPIE_Msk)
	#define ACMP0_INT_DISABLE			(ACMP->CMPCR[0] &= (~ACMP_CMPCR_ACMPIE_Msk))
	#define AVOID_ZXD_AFTER_PHCHG		160	// 160/2M = 80us
	#define ZXD_FILTER_TIME				200	// 200/2M = 100us
	#define CALC_TIME_BT_ZXD_SET_TIM	60	// calculation time between after confirmed ZX and set TIMER0
	#define TIME_DEBT					(ZXD_FILTER_TIME + CALC_TIME_BT_ZXD_SET_TIM)	// Included ZXD filter time and calculation time between after confirmed ZX and set TIMER0
	#define ZXD_BEFORE_PHCHG			50	// 50/2M = 25us	// If it already need change phase when ZX was confirmed ZX, give some time to TIM0 to response
	#define ACMP_HYS_AVG_TIME			20	// Average Hysteresis time 20/2M = 10us
	#define MIN_PHASE_TIME				500	//(AVOID_ZXD_AFTER_PHCHG + ZXD_FILTER_TIME + 100)	// 1200/2M=0.6ms, 8333RPM if 42PC=1MC==7EC
	#define MAX_PHASE_TIME				(10000)	// Unit 2MH, 5ms, 286rpm if 42PC=1MC==7EC

	#define MINI51_TIM_CNT_MAX			0xFFFFFF
	#define GET_TIM1_CMP_VALUE(x)		(((x) >= MINI51_TIM_CNT_MAX) ? ((x) - MINI51_TIM_CNT_MAX) : (x))
	#define GET_TIMER_DIFF(iLast, iThis)	(((iThis) > (iLast)) ? ((iThis) - (iLast)) : ((iThis) + (MINI51_TIM_CNT_MAX - (iLast))))
	#define MAX_MISS_ZXD_THRESHOLD		12
	#define MIN_SUCC_ZXD_THRESHOLD		4

	#ifdef __USED_BY_BLDC_SENSOR_LESS_C__
		#define EXTERNAL_BLDC  

	// UP side PWM
		#define PHASE_AB_PIN      		(0x0239ul)	// 0000 0010 0011 1001
		#define PHASE_AC_PIN     		(0x022Dul)	// 0000 0010 0010 1101
		#define PHASE_BC_PIN     		(0x0827ul)	// 0000 1000 0010 0111
		#define PHASE_BA_PIN      		(0x0836ul)  // 0000 1000 0011 0110
		#define PHASE_CA_PIN           	(0x201Eul)  // 0010 0000 0001 1110
		#define PHASE_CB_PIN			(0x201Bul)	// 0010 0000 0001 1011

	// Down side PWM
//		#define PHASE_AB_PIN      		(0x0439ul)	// 0000 0100 0011 1001
//		#define PHASE_AC_PIN     		(0x102Dul)	// 0001 0000 0010 1101
//		#define PHASE_BC_PIN     		(0x1027ul)	// 0001 0000 0010 0111
//		#define PHASE_BA_PIN      		(0x0136ul)  // 0000 0001 0011 0110
//		#define PHASE_CA_PIN           	(0x011Eul)  // 0000 0001 0001 1110
//		#define PHASE_CB_PIN			(0x041Bul)	// 0000 0100 0001 1011

	// Both sides PWM
//		#define PHASE_AB_PIN      		(0x0339ul)	// 0000 0110 0011 1001
//		#define PHASE_AC_PIN     		(0x122Dul)	// 0001 0010 0010 1101
//		#define PHASE_BC_PIN     		(0x1827ul)	// 0001 1000 0010 0111
//		#define PHASE_BA_PIN      		(0x0936ul)  // 0000 1001 0011 0110
//		#define PHASE_CA_PIN           	(0x211Eul)  // 0010 0001 0001 1110
//		#define PHASE_CB_PIN			(0x241Bul)	// 0010 0100 0001 1011
													//													//
		#define CMP0_PIN_P13			0x30000000  // P13 is selected as CMPP0 pin. 
		#define CMP0_PIN_P12			0x20000000  // P12 is selected as CMPP0 pin. 
		#define CMP0_PIN_P10			0x10000000  // P10 is selected as CMPP0 pin. 
		#define CMP0_PIN_P15			0x00000000  // P15 is selected as CMPP0 pin.

		const uint32_t PHASE_TAB_CLOCKWISE[] = { 
			PWM_PHCHG_T0_Msk | CMP0_PIN_P15 | PHASE_AB_PIN,
			PWM_PHCHG_T0_Msk | CMP0_PIN_P13 | PHASE_AC_PIN | DETEC_UP,
			PWM_PHCHG_T0_Msk | CMP0_PIN_P10 | PHASE_BC_PIN,
			PWM_PHCHG_T0_Msk | CMP0_PIN_P15 | PHASE_BA_PIN | DETEC_UP,
			PWM_PHCHG_T0_Msk | CMP0_PIN_P13 | PHASE_CA_PIN,
			PWM_PHCHG_T0_Msk | CMP0_PIN_P10 | PHASE_CB_PIN | DETEC_UP
//			PWM_PHCHG_T0_Msk | CMP0_PIN_P15 | PHASE_AB_PIN | DETEC_UP,
//			PWM_PHCHG_T0_Msk | CMP0_PIN_P13 | PHASE_AC_PIN,
//			PWM_PHCHG_T0_Msk | CMP0_PIN_P10 | PHASE_BC_PIN | DETEC_UP,
//			PWM_PHCHG_T0_Msk | CMP0_PIN_P15 | PHASE_BA_PIN,
//			PWM_PHCHG_T0_Msk | CMP0_PIN_P13 | PHASE_CA_PIN | DETEC_UP,
//			PWM_PHCHG_T0_Msk | CMP0_PIN_P10 | PHASE_CB_PIN
		};
		const uint32_t PHASE_TAB_ANTICLOCKWISE[] = {                        
			PWM_PHCHG_T0_Msk | CMP0_PIN_P15 | PHASE_AB_PIN | DETEC_UP,  
			PWM_PHCHG_T0_Msk | CMP0_PIN_P10 | PHASE_CB_PIN ,      
			PWM_PHCHG_T0_Msk | CMP0_PIN_P13 | PHASE_CA_PIN | DETEC_UP,             
			PWM_PHCHG_T0_Msk | CMP0_PIN_P15 | PHASE_BA_PIN , 
			PWM_PHCHG_T0_Msk | CMP0_PIN_P10 | PHASE_BC_PIN | DETEC_UP,            
			PWM_PHCHG_T0_Msk | CMP0_PIN_P13 | PHASE_AC_PIN 
		};

		typedef enum {	
			MOTOR_IDLE = 0,
//			MOTOR_STOP, 
			MOTOR_START,	// if not already rotating then just jumo to MOTOR_LOCATE
			MOTOR_LOCATE,
			MOTOR_WAIT_AFTER_LOCATE,
			MOTOR_RAMPUP_WO_ZXD,
			MOTOR_RAMPUP_W_ZXD,
			MOTOR_LOCKED
		} ENUM_MOTOR_STATE;

		typedef enum {	
			DETECT_START = 0,
			DETECT_PHASE_1_P,
			DETECT_PHASE_1_A,
			DETECT_PHASE_2_P,
			DETECT_PHASE_2_A,
			DETECT_PHASE_3_P,
			DETECT_PHASE_3_A
		} ENUM_ROTATE_DETECT_STATE;

//		typedef enum {
//			ENUM_MOTOR_POWER_ON = 0,
//			ENUM_MOTOR_ZX_DETECTING,
//			ENUM_MOTOR_LOCKED
//		}ENUM_MOTOR_SR_BIT;

		typedef enum {	
			STATUS_ERROR = 0,
			STATUS_FINISHED,
			STATUS_WORKING = 0xFFFF
		} ENUM_STATUS;

		const uint8_t unLocatePhaseSequencyTable[] = {0, 1, 2, 1};
		volatile uint32_t* unMosfetTestTable[] = {MOSFET_AS_PIN_ADDR, MOSFET_BS_PIN_ADDR, MOSFET_CS_PIN_ADDR,
				MOSFET_AD_PIN_ADDR, MOSFET_BD_PIN_ADDR, MOSFET_CD_PIN_ADDR};

		#define ALREADY_ROTATING_DETECTING			0xFFFF
		#define MAX_ROTATING_DETECT_PHASE_TIME		30	// half phase max 30ms
		#define MAX_ALREADY_ROTATING_DETECT_TIME	200	// 200ms used to detect is motor is already rotating
		#define MAX_SINGLE_PHASE_DURATION			80  // 80ms, 
		#define RAMP_UP_MIN_PERIOD_NUM_THRS			300	// After ramp up to minimum period, force continue rotate these phases
		#define MAX_MOTOR_PWR_DUTY 					(PWM_PERIOD - 150)
		#define CHANGE_DUTY_PERIOD_THR				9		// Used in ramp up
		#define CHANGE_DUTY_CNT_THR					5		// Used after locked

//		#define MOTOR_RUNNING_MSK					(0x01ul)
//		#define MOTOR_R_DIRECTION_MSK				(0x02ul)
//		#define IS_MOTOR_NEED_TO_RUN(x)				((x) & MOTOR_RUNNING_MSK)

//		#define IS_MOTOR_R_CLOCKWISE				(((mMotor.structMotor.MCR & MOTOR_R_DIRECTION_MSK) == 0) ? TRUE : FALSE)
		#define GET_PHASE_VALUE(x)					((mMotor.structMotor.MCR.RotateDirection == ROTATE_CLOCKWISE) ? PHASE_TAB_CLOCKWISE[(x)] : PHASE_TAB_ANTICLOCKWISE[(x)])
		#define PHASE_NUMBER						(sizeof(PHASE_TAB_CLOCKWISE)/sizeof(uint32_t))
		#define PHASE_INCREASE(x) 					INDEX_INCREASE((x), PHASE_NUMBER) //((x) = (((x) == (PHASE_NUMBER - 1)) ? 0 : ((x) + 1)))

//		#define RESET_MOTOR_SR_BIT(x)				(mMotor.structMotor.MSR &= (~(1ul << (x))))
//		#define SET_MOTOR_SR_BIT(x)					(mMotor.structMotor.MSR |= (1ul << (x)))
//		#define IS_MOTOR_STATUS_SET(x)				(mMotor.structMotor.MSR & (1ul << (x)))

		#define WAIT_AFTER_LOCATE_TIME				0	// ms

		#define MOTOR_RAMPUP_DT_MAX					(PWM_PERIOD - 200)
		#define MOTOR_RAMPUP_DT_FACTOR				(1.02)
//		#define MOTOR_RAMPUP_DT_INCR(x)				((x) = (((x) > MOTOR_RAMPUP_DT_MAX) ? (x) : (uint16_t)((x) * MOTOR_RAMPUP_DT_FACTOR)))
		#define MOTOR_RAMPUP_DT_INCR(x)				((x) = (((x) > MOTOR_RAMPUP_DT_MAX) ? (x) : ((x) + 1)))

		#define MOTOR_RAMPUP_PR_MIN					(1000 - 1)	// frequency 2M, 2857RPM if 1MC==7EC==42PC
		#define MOTOR_RAMPUP_PR_FACTOR				(0.98)
		#define MOTOR_RAMPUP_PR_DCR(x)				((x) = (((x) < MOTOR_RAMPUP_PR_MIN) ? (x) : (uint16_t)((x) * MOTOR_RAMPUP_PR_FACTOR)))
//		#define MOTOR_RAMPUP_PR_DCR(x)				((x) = (((x) < MOTOR_RAMPUP_PR_MIN) ? (x) : ((x) - 100)))
		#define MOTOR_START_ZXD_SPEED				(1600 - 1)	// frequency 2M,
//		#define MOTOR_START_ZXD_MINROT_CNT			200	// After phase change xxx times at max speed of rampup, start to detect ZX
		#define SET_MOSFET_ON_MANUAL(pinAddr)		(*(pinAddr) = 0)
		#define SET_MOSFET_OFF_MANUAL(pinAddr)		(*(pinAddr) = 1)
//		typedef enum {
//			PHASE_AB = 0,
//			PHASE_AC,
//			PHASE_BC,
//			PHASE_BA,
//			PHASE_CA,
//			PHASE_CB
//		} ENUM_MOTOR_PHASE;

//		typedef enum {
//			PHASE_AB = 0,
//			PHASE_CB,
//			PHASE_CA,
//			PHASE_BA,
//			PHASE_BC,
//			PHASE_AC
//		} ENUM_MOTOR_ANTICLOCKWISE_PHASE;


		static ENUM_MOTOR_STATE enumMotorState = MOTOR_IDLE;
		static ENUM_ROTATE_DETECT_STATE enumRotateDetectState = DETECT_START;
		static uint8_t iLocateIndex;
		static uint8_t iPhaseChangeCNT4Period;
		static uint8_t iPhaseChangeCNT4Duty;
		static uint16_t iRampUpPeriodMiniCNT;
		static uint32_t iCurrentPHCHG;
		static uint32_t iLastPhaseChangeTime;
		static uint32_t iRotateDetectStartTime;	// Used to record when enter motor start, 
												// ALready rotating detect end time will be compared with this time
	#else 
		#define EXTERNAL_BLDC extern
	#endif
EXTERNAL_BLDC MOTOR_UNION_T mMotor;	// Motor control register
EXTERNAL_BLDC ENUM_TIM1_USAGE FLAG_TIM1_USEAGE;
EXTERNAL_BLDC uint32_t iLastZXDetectedTime;
//EXTERNAL_BLDC uint32_t iPhaseChangeCNT4Period;	// Used to check if phase has been changed after every 60ms.
											// If not, force MOSFET OFF. 
											// 60ms=1.39rps=83.33rpm, we should make sure 1st startup ramp phase < 60ms

EXTERNAL_BLDC uint8_t iCurrentPhase;
EXTERNAL_BLDC uint8_t FLAG_PHASE_CHANGED;
EXTERNAL_BLDC __INLINE void stopMotor(void);
EXTERNAL_BLDC void checkMotor(void);
EXTERNAL_BLDC void BLDCSensorLessManager(void);
#endif
