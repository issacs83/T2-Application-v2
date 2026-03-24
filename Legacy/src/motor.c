/*
*********************************************************************************************
* motor.c :
*********************************************************************************************
* Copyright (C) 2014-2016 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date :
* @ Brief :
*
* @ Revision History :
*       1) initial creation. -------------------------------------- 2014-11-16
*       2) V1.1.0  ------------------------------------------- 2015-03-20
*       3) V1.2.0  ------------------------------------------- 2015-09-18
*       4) V1.3.0  ------------------------------------------- 2015-10-28
*********************************************************************************************
*/

/* Include files -------------------------------------------------------- */
#include "extern.h"
#include "system.h"
#include "timer.h"
#include "error_code.h"
#include "motor.h"
#include "tube.h"
#include "serial.h"
#include "misc1.h"
#include <math.h>
#ifdef USE_TUBE_PPS_TYPE_IO
#include "sensor.h"
#endif /* USE_TUBE_PPS_TYPE_IO */


/* Private typedef ----------------------------------------------------- */
/* Private define ------------------------------------------------------ */
#define CEPH_2ND_COLI_ALIGN_POSITION 40.31 * (MOTOR_S_MICROSTEP / (MOTOR_S_PULLEY_DIAMETER * pi)) // default

#define CT_ALIGN_POS_RAXIS_DEGREE         30.0
#define CT_ALIGN_POS_RAXIS_OFFSET         (CT_ALIGN_POS_RAXIS_DEGREE / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))


#define	MOTOR_DIR_GPIO_MODE							GPIO_Mode_OUT
#define	MOTOR_DIR_GPIO_A_PORT						GPIOA
#define	MOTOR_DIR_GPIO_A_CLOCK						RCC_AHB1Periph_GPIOA
#define	MOTOR_DIR_GPIO_C_PORT						GPIOC
#define	MOTOR_DIR_GPIO_C_CLOCK		        	    RCC_AHB1Periph_GPIOC
#define	MOTOR_DIR_GPIO_D_PORT						GPIOD
#define	MOTOR_DIR_GPIO_D_CLOCK						RCC_AHB1Periph_GPIOD

#define	MOTOR_ORG_GPIO_MODE							GPIO_Mode_IN
#define	MOTOR_ORG_GPIO_D_PORT						GPIOD
//#define	MOTOR_ORG_GPIO_D_CLOCK						RCC_AHB1Periph_GPIOD

#define	MOTOR_R										TIM8
#define	MOTOR_R_CLOCK								RCC_APB2Periph_TIM8
#define	MOTOR_R_GPIO								GPIOC
#define	MOTOR_R_GPIO_CLOCK							RCC_AHB1Periph_GPIOC
#define	MOTOR_R_GPIO_PIN							GPIO_Pin_8
#define	MOTOR_R_GPIO_PinSource						GPIO_PinSource8
#define	MOTOR_R_GPIO_AF								GPIO_AF_TIM8
#define	MOTOR_R_INIT_STATUS							TIM_OCPolarity_High
#define	MOTOR_R_INTERRUPT							TIM8_CC_IRQn
#define	MOTOR_R_SOURCE								TIM_IT_CC3
#define	MOTOR_R_CHANNEL								TIM_Channel_3

#define	MOTOR_R_ORG_GPIO							GPIOD
//#define	MOTOR_R_ORG_GPIO_CLOCK						RCC_AHB1Periph_GPIOD
#define	MOTOR_R_ORG_GPIO_PIN						GPIO_Pin_9
#define	MOTOR_R_ORG2_GPIO							GPIOD
//#define	MOTOR_R_ORG2_GPIO_CLOCK						RCC_AHB1Periph_GPIOD
#define	MOTOR_R_ORG2_GPIO_PIN						GPIO_Pin_10
#define	MOTOR_R_ORG_GPIO_MODE						GPIO_Mode_IN

//#define	MOTOR_R_DIR_GPIO_CLOCK						RCC_AHB1Periph_GPIOD
#define	MOTOR_R_DIR_GPIO							GPIOD
#define	MOTOR_R_DIR_GPIO_PIN						GPIO_Pin_1

#define	MOTOR_R_ORG_TYPE							ORG1HIGH_ORG2LOW
#define	MOTOR_R_TO_ORG_DIR							ORGDIR_LOW

 //FAST MODE
#define	MOTOR_R_MIN_FREQ							1500//500	
#define	MOTOR_R_DELAY_UNIT_FREQ						1000
#define	MOTOR_R_TO_ORG_RUN_STEP						4000 //16 * 200
#define	MOTOR_R_TO_ORG_RUN_FREQ						10000 // MOTOR_R_TO_ORG_RUN_STEP * 3 //  2
#define	MOTOR_R_TO_ORG_ACC_STEP						4000
#define	MOTOR_R_TO_ORG_DEC_STEP						4000
#define	MOTOR_R_TO_LIMIT_RUN_FREQ					10000
#define	MOTOR_R_TO_LIMIT_RUN_STEP					9600 // 4000
#define	MOTOR_R_TO_LIMIT_ACC_STEP					4000
#define	MOTOR_R_TO_LIMIT_DEC_STEP					4000

/*
#define	MOTOR_R_MIN_FREQ							1500
#define	MOTOR_R_DELAY_UNIT_FREQ						1000
#define	MOTOR_R_TO_ORG_RUN_STEP						4000
#define	MOTOR_R_TO_ORG_RUN_FREQ						4800
#define	MOTOR_R_TO_ORG_ACC_STEP						2000
#define	MOTOR_R_TO_ORG_DEC_STEP						4000
#define	MOTOR_R_TO_LIMIT_RUN_FREQ					4800
#define	MOTOR_R_TO_LIMIT_RUN_STEP					9600
#define	MOTOR_R_TO_LIMIT_ACC_STEP					2000
#define	MOTOR_R_TO_LIMIT_DEC_STEP					1920
*/



#define	MOTOR_V										TIM1
#define	MOTOR_V_CLOCK								RCC_APB2Periph_TIM1
#define	MOTOR_V_GPIO								GPIOA
#define	MOTOR_V_GPIO_CLOCK							RCC_AHB1Periph_GPIOA
#define	MOTOR_V_GPIO_PIN							GPIO_Pin_8
#define	MOTOR_V_GPIO_PinSource						GPIO_PinSource8
#define	MOTOR_V_GPIO_AF								GPIO_AF_TIM1
#define	MOTOR_V_INIT_STATUS							TIM_OCPolarity_High
#define	MOTOR_V_INTERRUPT							TIM1_CC_IRQn
#define	MOTOR_V_SOURCE								TIM_IT_CC1
#define	MOTOR_V_CHANNEL								TIM_Channel_1

#define	MOTOR_V_ORG_GPIO							GPIOD
//#define	MOTOR_V_ORG_GPIO_CLOCK						RCC_AHB1Periph_GPIOD
#define	MOTOR_V_ORG_GPIO_PIN						GPIO_Pin_8
//#define	MOTOR_V_ORG_GPIO_MODE						GPIO_Mode_IN

#define	MOTOR_V_DIR_GPIO							GPIOD
//#define	MOTOR_V_DIR_GPIO_CLOCK						RCC_AHB1Periph_GPIOD
#define	MOTOR_V_DIR_GPIO_PIN						GPIO_Pin_0

#define	MOTOR_V_ORG_TYPE							ORG1_HIGH
#define	MOTOR_V_TO_ORG_DIR							ORGDIR_HIGH
#define	MOTOR_V_MIN_FREQ							3000//500			
#define	MOTOR_V_DELAY_UNIT_FREQ						1000
#define	MOTOR_V_TO_ORG_RUN_STEP						16 * 200
#define	MOTOR_V_TO_ORG_RUN_FREQ						15000//MOTOR_V_TO_ORG_RUN_STEP * 2
#define	MOTOR_V_TO_ORG_ACC_STEP						3000
#define	MOTOR_V_TO_ORG_DEC_STEP						3200
#define	MOTOR_V_TO_LIMIT_RUN_FREQ					15000 //2000
#define	MOTOR_V_TO_LIMIT_ACC_STEP					1000
#define	MOTOR_V_TO_LIMIT_DEC_STEP					1000


//20190618_myshin : SCB Rev 2.0
#if 1
#define	MOTOR_C										TIM2
#define	MOTOR_C_CLOCK								RCC_APB1Periph_TIM2
#define	MOTOR_C_GPIO								GPIOA
#define	MOTOR_C_GPIO_CLOCK							RCC_AHB1Periph_GPIOA
#define	MOTOR_C_GPIO_PIN							GPIO_Pin_1
#define	MOTOR_C_GPIO_PinSource						GPIO_PinSource1
#define	MOTOR_C_GPIO_AF								GPIO_AF_TIM2
#define	MOTOR_C_INIT_STATUS							TIM_OCPolarity_High
#define	MOTOR_C_INTERRUPT							TIM2_IRQn
#define	MOTOR_C_SOURCE								TIM_IT_CC2
#define	MOTOR_C_CHANNEL								TIM_Channel_2
#else
#define	MOTOR_C										TIM4
#define	MOTOR_C_CLOCK								RCC_APB1Periph_TIM4
#define	MOTOR_C_GPIO								GPIOB
#define	MOTOR_C_GPIO_CLOCK							RCC_AHB1Periph_GPIOB
#define	MOTOR_C_GPIO_PIN							GPIO_Pin_6
#define	MOTOR_C_GPIO_PinSource						GPIO_PinSource6
#define	MOTOR_C_GPIO_AF								GPIO_AF_TIM4
#define	MOTOR_C_INIT_STATUS							TIM_OCPolarity_High
#define	MOTOR_C_INTERRUPT							TIM4_IRQn
#define	MOTOR_C_SOURCE								TIM_IT_CC1
#define	MOTOR_C_CHANNEL								TIM_Channel_1
#endif

#define	MOTOR_C_ORG_GPIO							GPIOD
#define	MOTOR_C_ORG_GPIO_CLOCK						RCC_AHB1Periph_GPIOD
#define	MOTOR_C_ORG_GPIO_PIN						GPIO_Pin_11
#define	MOTOR_C_ORG_GPIO_MODE						GPIO_Mode_IN

#define	MOTOR_C_DIR_GPIO				            GPIOC
#define	MOTOR_C_DIR_GPIO_PIN			            GPIO_Pin_15

#define	MOTOR_C_ORG_TYPE							ORG1_HIGH
#define	MOTOR_C_TO_ORG_DIR 							ORGDIR_HIGH

#define	MOTOR_C_MIN_FREQ							500			
#define	MOTOR_C_DELAY_UNIT_FREQ						1000
#define	MOTOR_C_TO_ORG_RUN_STEP						400
#define	MOTOR_C_TO_ORG_RUN_FREQ						2500
#define	MOTOR_C_TO_ORG_ACC_STEP						400
#define	MOTOR_C_TO_ORG_DEC_STEP						400
#define	MOTOR_C_TO_LIMIT_RUN_FREQ					2500
#define	MOTOR_C_TO_LIMIT_RUN_STEP					400
#define	MOTOR_C_TO_LIMIT_ACC_STEP					200
#define	MOTOR_C_TO_LIMIT_DEC_STEP					200


#define	MOTOR_S										TIM10
#define	MOTOR_S_CLOCK								RCC_APB2Periph_TIM10
#define	MOTOR_S_GPIO								GPIOB
#define	MOTOR_S_GPIO_CLOCK							RCC_AHB1Periph_GPIOB
#define	MOTOR_S_GPIO_PIN							GPIO_Pin_8
#define	MOTOR_S_GPIO_PinSource						GPIO_PinSource8
#define	MOTOR_S_GPIO_AF								GPIO_AF_TIM10
#define	MOTOR_S_INIT_STATUS							TIM_OCPolarity_High
#define	MOTOR_S_INTERRUPT							TIM1_UP_TIM10_IRQn
#define	MOTOR_S_SOURCE								TIM_IT_CC1
#define	MOTOR_S_CHANNEL								TIM_Channel_1

#define	MOTOR_S_ORG_GPIO							GPIOD
#define	MOTOR_S_ORG_GPIO_CLOCK						RCC_AHB1Periph_GPIOD
#define	MOTOR_S_ORG_GPIO_PIN						GPIO_Pin_12
#define	MOTOR_S_ORG_GPIO_MODE						GPIO_Mode_IN

#define	MOTOR_S_DIR_GPIO							GPIOD
//#define	MOTOR_S_DIR_GPIO_CLOCK						RCC_AHB1Periph_GPIOD
#define	MOTOR_S_DIR_GPIO_PIN						GPIO_Pin_3

#define	MOTOR_S_ORG_TYPE							ORG1_HIGH
#define	MOTOR_S_TO_ORG_DIR							ORGDIR_LOW

#define	MOTOR_S_MIN_FREQ							500			
#define	MOTOR_S_DELAY_UNIT_FREQ						1000
#define	MOTOR_S_TO_ORG_RUN_STEP						400
#define	MOTOR_S_TO_ORG_RUN_FREQ						2500
#define	MOTOR_S_TO_ORG_ACC_STEP						400
#define	MOTOR_S_TO_ORG_DEC_STEP						400
#define	MOTOR_S_TO_LIMIT_RUN_FREQ					2500
#define	MOTOR_S_TO_LIMIT_RUN_STEP					400
#define	MOTOR_S_TO_LIMIT_ACC_STEP					200
#define	MOTOR_S_TO_LIMIT_DEC_STEP					200

#define	MOTOR_T										TIM5
#define	MOTOR_T_CLOCK								RCC_APB1Periph_TIM5
#define	MOTOR_T_GPIO								GPIOC
#define	MOTOR_T_GPIO_CLOCK							RCC_AHB1Periph_GPIOC
#define	MOTOR_T_GPIO_PIN							GPIO_Pin_6
#define	MOTOR_T_GPIO_PinSource						GPIO_PinSource6
//#define	MOTOR_T_GPIO_AF								GPIO_AF_TIM5
//#define	MOTOR_T_INIT_STATUS							TIM_OCPolarity_High
#define	MOTOR_T_INTERRUPT							TIM5_IRQn
#define	MOTOR_T_SOURCE								TIM_IT_Update
//#define	MOTOR_T_CHANNEL								TIM_Channel_1

#define	MOTOR_T_ORG_GPIO							GPIOD
#define	MOTOR_T_ORG_GPIO_CLOCK						RCC_AHB1Periph_GPIOD
#define	MOTOR_T_ORG_GPIO_PIN						GPIO_Pin_14
#define	MOTOR_T_ORG_GPIO_MODE						GPIO_Mode_IN

#define	MOTOR_T_DIR_GPIO							GPIOA
//#define	MOTOR_T_DIR_GPIO_CLOCK						RCC_AHB1Periph_GPIOD
#define	MOTOR_T_DIR_GPIO_PIN						GPIO_Pin_3

#define	MOTOR_T_ORG_TYPE 							ORG1_HIGH
#define	MOTOR_T_TO_ORG_DIR 							ORGDIR_HIGH
#define	MOTOR_T_MIN_FREQ 							500			
#define	MOTOR_T_DELAY_UNIT_FREQ 					1000
#define	MOTOR_T_TO_ORG_RUN_STEP 					16 * 200//1500
#define	MOTOR_T_TO_ORG_RUN_FREQ 					3000//MOTOR_T_TO_ORG_RUN_STEP//3000
#define	MOTOR_T_TO_ORG_DEC_STEP 					2000//500
#define	MOTOR_T_TO_ORG_ACC_STEP 					1000//500
#define	MOTOR_T_TO_LIMIT_RUN_FREQ 					3000//2500
#define	MOTOR_T_TO_LIMIT_RUN_STEP 					1000//200
#define	MOTOR_T_TO_LIMIT_ACC_STEP 					500
#define	MOTOR_T_TO_LIMIT_DEC_STEP 					500


#define	MOTOR_A										TIM9
#define	MOTOR_A_CLOCK								RCC_APB2Periph_TIM9
#define	MOTOR_A_GPIO								GPIOE
#define	MOTOR_A_GPIO_CLOCK							RCC_AHB1Periph_GPIOE
#define	MOTOR_A_GPIO_PIN							GPIO_Pin_5
#define	MOTOR_A_GPIO_PinSource						GPIO_PinSource5
#define	MOTOR_A_GPIO_AF								GPIO_AF_TIM9
#define	MOTOR_A_INIT_STATUS							TIM_OCPolarity_High
#define	MOTOR_A_INTERRUPT							TIM1_BRK_TIM9_IRQn
#define	MOTOR_A_SOURCE								TIM_IT_CC1
#define	MOTOR_A_CHANNEL								TIM_Channel_1

#define	MOTOR_A_ORG_GPIO							GPIOD
#define	MOTOR_A_ORG_GPIO_CLOCK						RCC_AHB1Periph_GPIOD
#define	MOTOR_A_ORG_GPIO_PIN						GPIO_Pin_13
#define	MOTOR_A_ORG_GPIO_MODE						GPIO_Mode_IN

#define	MOTOR_A_DIR_GPIO							GPIOD
#define	MOTOR_A_DIR_GPIO_CLOCK						RCC_AHB1Periph_GPIOD
#define	MOTOR_A_DIR_GPIO_PIN						GPIO_Pin_4

#define	MOTOR_A_ORG_TYPE							ORG1_HIGH
#define	MOTOR_A_TO_ORG_DIR							ORGDIR_HIGH
#define	MOTOR_A_MIN_FREQ							7500			
#define	MOTOR_A_DELAY_UNIT_FREQ						1000
#define	MOTOR_A_TO_ORG_RUN_STEP						16 * 200
#define	MOTOR_A_TO_ORG_RUN_FREQ						12000 //MOTOR_A_TO_ORG_RUN_STEP			
#define	MOTOR_A_TO_ORG_ACC_STEP						3000
#define	MOTOR_A_TO_ORG_DEC_STEP						3200
#define	MOTOR_A_TO_LIMIT_RUN_FREQ					12000 //3000
#define	MOTOR_A_TO_LIMIT_RUN_STEP					2000
#define	MOTOR_A_TO_LIMIT_ACC_STEP					1000
#define	MOTOR_A_TO_LIMIT_DEC_STEP					1000

//jehun
#ifdef USE_MOTOR_CHINREST_VER
/*//#define	MOTOR_CNS									TIM3
#define	MOTOR_CNS_CLOCK								RCC_APB1Periph_TIM5
#define	MOTOR_CNS_GPIO								GPIOB
#define	MOTOR_CNS_GPIO_CLOCK						RCC_AHB1Periph_GPIOB
#define	MOTOR_CNS_GPIO_PIN							GPIO_Pin_0
#define	MOTOR_CNS_GPIO_PinSource					GPIO_PinSource0
//#define	MOTOR_CNS_GPIO_AF							GPIO_AF_TIM3
//#define	MOTOR_CNS_INIT_STATUS						TIM_OCPolarity_High

#define	MOTOR_CNS_INTERRUPT							TIM5_IRQn
#define	MOTOR_CNS_SOURCE							TIM_IT_Update //TIM_IT_CC3
#define	MOTOR_CNS_CHANNEL							TIM_Channel_1
*/

//#define	MOTOR_CNS									TIM3
#define	MOTOR_CNS_CLOCK								RCC_APB1Periph_TIM3
#define	MOTOR_CNS_GPIO								GPIOB
#define	MOTOR_CNS_GPIO_CLOCK						RCC_AHB1Periph_GPIOB
#define	MOTOR_CNS_GPIO_PIN							GPIO_Pin_0
#define	MOTOR_CNS_GPIO_PinSource					GPIO_PinSource0
#define	MOTOR_CNS_GPIO_AF							GPIO_AF_TIM3
#define	MOTOR_CNS_INIT_STATUS						TIM_OCPolarity_High
#define	MOTOR_CNS_INTERRUPT							TIM3_IRQn
#define	MOTOR_CNS_SOURCE							TIM_IT_CC3
#define	MOTOR_CNS_CHANNEL							TIM_Channel_3


#define	MOTOR_CNS_ORG_GPIO							GPIOA
#define	MOTOR_CNS_ORG_GPIO_CLOCK					RCC_AHB1Periph_GPIOA
#define	MOTOR_CNS_ORG_GPIO_PIN						GPIO_Pin_6 //GPIO_Pin_11
#define	MOTOR_CNS_ORG_GPIO_MODE						GPIO_Mode_IN

#define	MOTOR_CNS_DIR_GPIO							GPIOC
#define	MOTOR_CNS_DIR_GPIO_CLOCK					RCC_AHB1Periph_GPIOC
#define	MOTOR_CNS_DIR_GPIO_PIN						GPIO_Pin_3

#define	MOTOR_CNS_ORG_TYPE							ORG1_HIGH
#define	MOTOR_CNS_TO_ORG_DIR						ORGDIR_LOW
//jehun - 20200721 CNS ���� �ȿ����� min FREQ�� ����
#define	MOTOR_CNS_MIN_FREQ							500 // 500
//#define	MOTOR_CNS_MIN_FREQ							5000 // 500
#define	MOTOR_CNS_DELAY_UNIT_FREQ					1000
//jehun

#define	MOTOR_CNS_TO_ORG_RUN_STEP					1000
#define	MOTOR_CNS_TO_ORG_RUN_FREQ					30000  //30000
#define	MOTOR_CNS_TO_ORG_ACC_STEP					1000
#define	MOTOR_CNS_TO_ORG_DEC_STEP					1000
#define	MOTOR_CNS_TO_LIMIT_RUN_FREQ					30000  // 30000	//8000
#define	MOTOR_CNS_TO_LIMIT_RUN_STEP					10000 //4000
#define	MOTOR_CNS_TO_LIMIT_ACC_STEP					1000
#define	MOTOR_CNS_TO_LIMIT_DEC_STEP					1000

/*
#define	MOTOR_CNS_TO_ORG_RUN_STEP					8000
#define	MOTOR_CNS_TO_ORG_RUN_FREQ					14000
#define	MOTOR_CNS_TO_ORG_ACC_STEP					4000
#define	MOTOR_CNS_TO_ORG_DEC_STEP					8000
#define	MOTOR_CNS_TO_LIMIT_RUN_FREQ					14000	//8000
#define	MOTOR_CNS_TO_LIMIT_RUN_STEP					10000//146141 //4000
#define	MOTOR_CNS_TO_LIMIT_ACC_STEP					4000
#define	MOTOR_CNS_TO_LIMIT_DEC_STEP					4000
*/
#endif /* USE_MOTOR_CHINREST_VER */

#ifdef USE_MOTOR_CHINREST_HOR
#define	MOTOR_CWE									TIM11
#define	MOTOR_CWE_CLOCK								RCC_APB2Periph_TIM11
#define	MOTOR_CWE_GPIO								GPIOB
#define	MOTOR_CWE_GPIO_CLOCK						RCC_AHB1Periph_GPIOB
#define	MOTOR_CWE_GPIO_PIN							GPIO_Pin_9
#define	MOTOR_CWE_GPIO_PinSource					GPIO_PinSource9
#define	MOTOR_CWE_GPIO_AF							GPIO_AF_TIM11
#define	MOTOR_CWE_INIT_STATUS						TIM_OCPolarity_High
#define	MOTOR_CWE_INTERRUPT							TIM1_TRG_COM_TIM11_IRQn
#define	MOTOR_CWE_SOURCE							TIM_IT_CC1
#define	MOTOR_CWE_CHANNEL							TIM_Channel_1

#define	MOTOR_CWE_ORG_GPIO							GPIOE
#define	MOTOR_CWE_ORG_GPIO_CLOCK					RCC_AHB1Periph_GPIOE
#define	MOTOR_CWE_ORG_GPIO_PIN						GPIO_Pin_12
#define	MOTOR_CWE_ORG_GPIO_MODE						GPIO_Mode_IN

#define	MOTOR_CWE_DIR_GPIO							GPIOD
#define	MOTOR_CWE_DIR_GPIO_CLOCK					RCC_AHB1Periph_GPIOD
#define	MOTOR_CWE_DIR_GPIO_PIN						GPIO_Pin_15

#define	MOTOR_CWE_ORG_TYPE							ORG1_HIGH
#define	MOTOR_CWE_TO_ORG_DIR						ORGDIR_HIGH
#define	MOTOR_CWE_MIN_FREQ							2000 // 500			
#define	MOTOR_CWE_DELAY_UNIT_FREQ					1000
#define	MOTOR_CWE_TO_ORG_RUN_STEP					2000
#define	MOTOR_CWE_TO_ORG_RUN_FREQ					20000 //4000
#define	MOTOR_CWE_TO_ORG_ACC_STEP					400
#define	MOTOR_CWE_TO_ORG_DEC_STEP					2000
#define	MOTOR_CWE_TO_LIMIT_RUN_FREQ					20000 //2000
#define	MOTOR_CWE_TO_LIMIT_RUN_STEP					4000 //14720//400
#define	MOTOR_CWE_TO_LIMIT_ACC_STEP					2000
#define	MOTOR_CWE_TO_LIMIT_DEC_STEP					2000
#endif /* USE_MOTOR_CHINREST_HOR */

#define	MOTOR_DELAY_TIME							500

#define CT_SCAN_MIN_TIME						10.0 // Sec
#define MOTOR_R_FAST_MIN_FREQ					4000

#define BOOT_ROT_ALIGN_ANGLE			40.0 // degree
#define BOOT_ROT_CHECK_STEP 			BOOT_ROT_ALIGN_ANGLE / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO)
#define ROTATOR_ALIGN_STEP		 		PANO_ROTATE_ALIGN_ANGLE / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO)


#define CEPH_REVERSE_INIT_POS 			260.0 * (MOTOR_S_MICROSTEP / (MOTOR_S_PULLEY_DIAMETER * pi))

#define	TIM3_COUNT_CLOCK			1200000
#define TIM3_PERIOD 				1200 // Hz PWM


#define ROT_SPI_CS_GPIO					GPIOE
#define ROT_SPI_CS_GPIO_PIN				GPIO_Pin_13

#define PAN_SPI_CS_GPIO					GPIOA
#define PAN_SPI_CS_GPIO_PIN				GPIO_Pin_0

#define CS_SPI_CS_GPIO					GPIOA
#define CS_SPI_CS_GPIO_PIN				GPIO_Pin_12

#define CC_SPI_CS_GPIO					GPIOB
#define CC_SPI_CS_GPIO_PIN				GPIO_Pin_1

#define MS_SPI_CS_GPIO					GPIOD
#define MS_SPI_CS_GPIO_PIN				GPIO_Pin_2

#define ER_SPI_CS_GPIO					GPIOC
#define ER_SPI_CS_GPIO_PIN				GPIO_Pin_2

#define CNS_SPI_CS_GPIO					GPIOE
#define CNS_SPI_CS_GPIO_PIN				GPIO_Pin_1

#define CWE_SPI_CS_GPIO					GPIOE
#define CWE_SPI_CS_GPIO_PIN				GPIO_Pin_10

#define USE_MOTOR_PANO_AXIS

#define Priority_0						0
#define Priority_1						1
#define Priority_2						2

/* Select : Chip Select pin low  */
#define SPI_CS_LOW(port, pin) 		GPIO_ResetBits(port, pin)
/* Deselect : Chip Select pin high */
#define SPI_CS_HIGH(port, pin)      GPIO_SetBits(port, pin)



#define	PANO_ROTATE_ALIGN_ANGLE				30.0 // degree
#define	PANO_ROTATE_START_ANGLE				120.0 // degree			
#define	PANO_AUTO_ANGLE						20.0 // degree

#define GEOMETRY_ALIGN_RAXIS_START_ANGLE 			(30.0 / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))
#define GEOMETRY_ALIGN_VAXIS_START_ANGLE 			(10.0 / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP))

#define	CEPH_ROTATE_ANGLE           		90.0 // degree
#define	CEPH_VERTICAL_POSITION       		100.0 // mm
#define	CEPH_AUTO_ANGLE						-20.0
#define CEPH_2ND_COLLIMATOR_RATIO (double)0.726 //20190711_myshin : Safety ?�비


/* Private macro ---------------------------------------------------- */
/* Private variables -------------------------------------------------- */
Motor_Typedef Motor_R;
Motor_Typedef Motor_V;
Motor_Typedef Motor_C;
Motor_Typedef Motor_S;
#ifdef USE_MOTOR_CHINREST_TS
TempleSupporStatus Temple_Status;
Motor_Typedef Motor_T;
#endif /* USE_MOTOR_CHINREST_TS */
#ifdef USE_MOTOR_GANTRY_MS
Motor_Typedef Motor_A;
#endif /* USE_MOTOR_GANTRY_MS */
#ifdef USE_MOTOR_CHINREST_VER
Motor_Typedef Motor_CNS;
#endif /* USE_MOTOR_CHINREST_VER */
#ifdef USE_MOTOR_CHINREST_HOR
Motor_Typedef Motor_CWE;
#endif /* USE_MOTOR_CHINREST_HOR */

static bool bIsOrgSeq2 = RESET;

static uint32_t gInitTimeOut;


/* Private function prototypes ------------------------------------------*/
static void Motor_InitParam(void);
static void Motor_InitOrgParam(Motor_Typedef* motor);
static void Motor_InitOrgParam_R(Motor_Typedef* motor);
static void Motor_InitRunParam(RunParam_Typedef* param, double ccr);
static void Motor_InitRunParam_R(RunParam_Typedef* param, double ccr);
static double Motor_CalcCCRValue(double ccr1, double ccr2, uint32_t step);
static void Motor_SetStatus(Motor_Typedef* motor, MotorStatus_Typedef status);
static bool Motor_CheckOrgType(Motor_Typedef* motor);
static bool Motor_Reverse_CheckOrgType(Motor_Typedef* motor);
static void Motor_SetDir(Motor_Typedef* motor, MotorDir_Typedef direction);
static void Motor_ProcPanoMode(void);
inline static void Motor_MoveAbsoluteDistanceAccDec(Motor_Typedef* motor, double freq, uint32_t acc, int32_t run, uint32_t dec);
static void MotorR_MoveAbsoluteDistanceAccDec_CT(Motor_Typedef* motor, double freq, uint32_t acc, int32_t run, uint32_t dec, double TimClk);
static void Motor_MoveAbsoluteDistance(Motor_Typedef* motor, double freq, uint32_t acc, int32_t run, uint32_t dec);
static bool Motor_InitBootPosition(void);
static void SPI_Initialize(void);
static uint8_t SPI_SendByte(uint8_t byte);
static void TMC2660_SPI_SendData(GPIO_TypeDef *pGpio, uint16_t gpio_pin, uint32_t spiData);
static void TMC2660_SPI_Init(void);
static void Motor_SetCurrent(char bStart);
void TMC2660_SPI_Resolution_Change(unsigned char resolution);


/* Private functions ---------------------------------------------------- */
//jehun
/*
void Motor_Stop_CNS(Motor_Typedef* motor)
{
	TIM_Cmd(motor->Timer.Periph, DISABLE);	

	//TMC2660_SetCurrent(motor->Timer.Periph, 0);
	Motor_SetStatus(motor, STATUS_STOP);
}


void Motor_RunOrgCheck_CNS(Motor_Typedef* motor)
{
	if (!Motor_GetStatus(motor, STATUS_STOP))
		Motor_Stop(motor);

	if (Motor_CheckOrgType(motor)) {
		Motor_SetStatus(motor, STATUS_LIMIT);
		Motor_SetDir(motor, DIR_LIMIT);
	} else {
		Motor_SetStatus(motor, STATUS_ORG);
		Motor_SetDir(motor, DIR_ORG);
	}

	motor->ToOrgStep = 0;
	motor->OrgOverStep = 0;
	motor->DelayCnt = 0;

	Motor_Start_CNS(motor);

}

void Motor_Start_CNS(Motor_Typedef* motor)
{
    TIM_SetCounter(motor->Timer.Periph, 0);

	//if (!Motor_GetStatus(motor, STATUS_ARCH))
		motor->NextCCR = motor->MinFreqCCR;
	else 		
        motor->NextCCR = motor->ArchParam.IndexCcr;
	//
    if (Motor_GetStatus(motor, STATUS_ARCH))
        motor->NextCCR = motor->ArchParam.IndexCcr;
    else if(Motor_GetStatus(motor, STATUS_CT))
		//motor->NextCCR = motor->MinFreqCCR;
		//motor->NextCCR = motor->RunParam.RunCCR*(motor->RunParam.AccStep);
		motor->NextCCR = (motor->RunParam.RunCCR*motor->RunParam.AccStep*5)/(motor->RunParam.AccStep+4);
	else if(Motor_GetStatus(motor, STATUS_ORG))
	{		
		if(motor == &Motor_R)
		{
			if(motor->MotorDir == DIR_LIMIT)
				motor->NextCCR = motor->ToLimitParam.RunCCR*((double)(motor->ToLimitParam.AccStep));
			else
				motor->NextCCR = motor->ToOrgParam.RunCCR*((double)(motor->ToOrgParam.AccStep));
		}
		else
		{
			motor->NextCCR = motor->MinFreqCCR;
		}		
	}
	else 
	{
		if(motor == &Motor_R)
		{
			motor->NextCCR = motor->RunParam.RunCCR*((double)(motor->RunParam.AccStep));
		}
		else
		{
			motor->NextCCR = motor->MinFreqCCR;
			//motor->NextCCR = (motor->RunParam.RunCCR*motor->RunParam.AccStep*3)/(motor->RunParam.AccStep+2);
		}		
	}

	TIM_ITConfig(MOTOR_T, MOTOR_T_SOURCE, ENABLE);

	//Motor_SetDir(motor, DIR_ORG);
    TIM_Cmd(motor->Timer.Periph, ENABLE);
}*/
/**
* @ Function Name : MotorR_InitMinFreq
* @ Desc :
* @ Param :
* @ Return :
*/
void MotorR_InitMinFreq(uint32_t freq)
{
	Motor_R.MinFreqCCR = Hz_To_CCR(freq);
}

/**
* @ Function Name : SPI_SendByte
* @ Desc :
* @ Param :
* @ Return :
*/
static uint8_t SPI_SendByte(uint8_t byte)
{
	/* Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);

	/* Send byte through the SPI1 peripheral */
	SPI_I2S_SendData(SPI3, byte);

	/* Wait to receive a byte */
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET);

	/* Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI3);
}

/**
* @ Function Name : SPI_Initialize
* @ Desc :
* @ Param :
* @ Return :
*/
static void SPI_Initialize(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	if(sysInfo.bShowLog==SET)
	{
		printUart(DBG_MSG_PC, "SPI_Initialize");
	}

	/* Enable SPI3 clocks */
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);


	/* Enable GPIO clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE, ENABLE);
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	/* configure pins used by SPI3
		 * PC10 = SCK3_SCK
		 * PC11 = SCK3_MISO
		 * PC12 = SCK3_MOSI
		 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);

	/* Configure I/O for Chip select */
	GPIO_InitStructure.GPIO_Pin = ROT_SPI_CS_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //GPIO_Mode_Out_PP;
	GPIO_Init(ROT_SPI_CS_GPIO, &GPIO_InitStructure);


	/* Deselect : Chip Select high */
	SPI_CS_HIGH(ROT_SPI_CS_GPIO, ROT_SPI_CS_GPIO_PIN);

	GPIO_InitStructure.GPIO_Pin = PAN_SPI_CS_GPIO_PIN;
	GPIO_Init(PAN_SPI_CS_GPIO, &GPIO_InitStructure);
	SPI_CS_HIGH(PAN_SPI_CS_GPIO, PAN_SPI_CS_GPIO_PIN);

	GPIO_InitStructure.GPIO_Pin = CC_SPI_CS_GPIO_PIN;
	GPIO_Init(CC_SPI_CS_GPIO, &GPIO_InitStructure);
	SPI_CS_HIGH(CC_SPI_CS_GPIO, CC_SPI_CS_GPIO_PIN);

	GPIO_InitStructure.GPIO_Pin = CS_SPI_CS_GPIO_PIN;
	GPIO_Init(CS_SPI_CS_GPIO, &GPIO_InitStructure);
	SPI_CS_HIGH(CS_SPI_CS_GPIO, CS_SPI_CS_GPIO_PIN);

	GPIO_InitStructure.GPIO_Pin = MS_SPI_CS_GPIO_PIN;
	GPIO_Init(MS_SPI_CS_GPIO, &GPIO_InitStructure);
	SPI_CS_HIGH(MS_SPI_CS_GPIO, MS_SPI_CS_GPIO_PIN);

	GPIO_InitStructure.GPIO_Pin = ER_SPI_CS_GPIO_PIN;
	GPIO_Init(ER_SPI_CS_GPIO, &GPIO_InitStructure);
	SPI_CS_HIGH(ER_SPI_CS_GPIO, ER_SPI_CS_GPIO_PIN);

	GPIO_InitStructure.GPIO_Pin = CNS_SPI_CS_GPIO_PIN;
	GPIO_Init(CNS_SPI_CS_GPIO, &GPIO_InitStructure);
	SPI_CS_HIGH(CNS_SPI_CS_GPIO, CNS_SPI_CS_GPIO_PIN);

	GPIO_InitStructure.GPIO_Pin = CWE_SPI_CS_GPIO_PIN;
	GPIO_Init(CWE_SPI_CS_GPIO, &GPIO_InitStructure);
	SPI_CS_HIGH(CWE_SPI_CS_GPIO, CWE_SPI_CS_GPIO_PIN);

	/* SPI3 configuration */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; /* TMC2660 max 4MHz, APB1=60MHz/16=3.75MHz */
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI3, &SPI_InitStructure);

	/* Enable SPI3 */
	SPI_Cmd(SPI3, ENABLE);
}

/**
* @ Function Name : TMC2660_SPI_SendData
* @ Desc :
* @ Param :
* @ Return :
*/
static void TMC2660_SPI_SendData(GPIO_TypeDef *pGpio, uint16_t gpio_pin, uint32_t spiData)
{
	uint32_t Received=0;
	uint32_t Received_byte=0;
	char string[64] = {0,};
	
	if(sysInfo.bShowMotorLog==SET && sysInfo.bShowLog == SET)
	{
		printUart(DBG_MSG_PC, "TMC2660_SPI_SendData : $%05x (%02x, %02x, %02x)", 
			spiData, (spiData >> 16) & 0xf, (spiData >> 8) & 0xff, (spiData & 0xff));
	}
	SPI_CS_LOW(pGpio, gpio_pin);
	/*

	Received_byte=SPI_SendByte((spiData >> 16) & 0xf);
	Received=Received_byte;
	Received_byte=SPI_SendByte((spiData >> 8) & 0xff);
	Received=((Received<<8) | Received_byte);
	Received_byte=SPI_SendByte(spiData & 0xff); 
	Received=((Received<<8) | Received_byte);

		*/
	Received |= SPI_SendByte((spiData >> 16) & 0xf);
	Received <<=8;
	Received |= SPI_SendByte((spiData >> 8) & 0xff);
	Received <<=8;
	Received |= SPI_SendByte(spiData & 0xff); 
	Received >>=4;


	SPI_CS_HIGH(pGpio, gpio_pin);

	if(sysInfo.bShowLog==SET && sysInfo.bShowMotorLog==SET )
	{
		if(pGpio==ROT_SPI_CS_GPIO)
			sprintf(string, "MOTOR_R Received Data :");
		else if(pGpio==PAN_SPI_CS_GPIO)
			sprintf(string, "MOTOR_V Received Data :");
		else if(pGpio==CS_SPI_CS_GPIO)
			sprintf(string, "MOTOR_C Received Data :");
		else if(pGpio==CC_SPI_CS_GPIO)
			sprintf(string, "MOTOR_S Received Data :");
		else if(pGpio==MS_SPI_CS_GPIO)
			sprintf(string, "MOTOR_A Received Data :");
		else if(pGpio==ER_SPI_CS_GPIO)
			sprintf(string, "MOTOR_T Received Data :");
		else if(pGpio==CNS_SPI_CS_GPIO)
			sprintf(string, "MOTOR_CNS Received Data :");
		else if(pGpio==CWE_SPI_CS_GPIO)
			sprintf(string, "MOTOR_CWE Received Data :");

		if(Received == 0xfffff)
		{
			printUart(DBG_MSG_PC, "%s $%05x Check the 24V Power",string,Received);
		}
		else
		{
			//Received_byte = Received & 0x00080;
			//Received_byte>>=7;
			//if(Received_byte == SET)
			//	printUart(DBG_MSG_PC, "%s $%05x Standstill : No Active Edge",string,Received);
			Received_byte = Received & 0x00040;
			Received_byte>>=6;
			if(Received_byte == SET)
				printUart(DBG_MSG_PC, "%s $%05x Open Load B",string,Received);
			Received_byte = Received & 0x00020;
			Received_byte>>=5;
			if(Received_byte == SET)
				printUart(DBG_MSG_PC, "%s $%05x Open Load A",string,Received);
			Received_byte = Received & 0x00010;
			Received_byte>>=4;
			if(Received_byte == SET)
				printUart(DBG_MSG_PC, "%s $%05x Short to GND detection B",string,Received);
			Received_byte = Received & 0x00008;
			Received_byte>>=3;
			if(Received_byte == SET)
				printUart(DBG_MSG_PC, "%s $%05x Short to GND detection A",string,Received);
			Received_byte = Received & 0x00004;	
			Received_byte>>=2;
			if(Received_byte == SET)
				printUart(DBG_MSG_PC, "%s $%05x OverTemperture Warning (100��C)",string,Received);
			Received_byte = Received & 0x00002;
			Received_byte>>=1;
			if(Received_byte == SET)
				printUart(DBG_MSG_PC, "%s $%05x OverTemperture ShotDown (150��C)",string,Received);
			Received_byte = Received & 0x00001;
			if(Received_byte == SET)
				printUart(DBG_MSG_PC, "%s $%05x StallGaurd 2",string,Received);
		}
	}
}

void TMC2660_SPI_ReadData(Motor_Typedef* Mot)
{

	if(Mot == &Motor_R)
	{
		TMC2660_SPI_SendData(ROT_SPI_CS_GPIO, ROT_SPI_CS_GPIO_PIN, 0x00000);
	}
	else if(Mot == &Motor_V)
	{
#if defined(USE_MOTOR_PANO_AXIS)
		TMC2660_SPI_SendData(PAN_SPI_CS_GPIO, PAN_SPI_CS_GPIO_PIN, 0x00000);
#endif /* USE_MOTOR_PANO_AXIS */		
	}
	else if(Mot == &Motor_C)
	{
		TMC2660_SPI_SendData(CS_SPI_CS_GPIO, CS_SPI_CS_GPIO_PIN, 0x00000); /* Fix: was ROT (Motor_R) CS pin */
	}
	else if(Mot == &Motor_S)
	{
		TMC2660_SPI_SendData(CC_SPI_CS_GPIO, CC_SPI_CS_GPIO_PIN, 0x00000);
	}
	else if(Mot == &Motor_T)
	{
#ifdef USE_MOTOR_CHINREST_TS
		TMC2660_SPI_SendData(ER_SPI_CS_GPIO, ER_SPI_CS_GPIO_PIN, 0x00000);
#endif /* USE_MOTOR_CHINREST_TS */
	}
	else if(Mot == &Motor_A)
	{
#ifdef USE_MOTOR_GANTRY_MS
		TMC2660_SPI_SendData(MS_SPI_CS_GPIO, MS_SPI_CS_GPIO_PIN, 0x00000);	
#endif /* USE_MOTOR_GANTRY_MS */
	}
	else if(Mot == &Motor_CNS)
	{
#ifdef USE_MOTOR_CHINREST_VER
		TMC2660_SPI_SendData(CNS_SPI_CS_GPIO, CNS_SPI_CS_GPIO_PIN, 0x00000);
#endif /* USE_MOTOR_CHINREST_VER */
	}
	else if(Mot == &Motor_CWE)
	{
#ifdef USE_MOTOR_CHINREST_HOR
		TMC2660_SPI_SendData(CWE_SPI_CS_GPIO, CWE_SPI_CS_GPIO_PIN, 0x00000);
#endif 
	}
}

void TMC2660_SPI_Data_Sending(Motor_Typedef* Mot,uint32_t spiData)
{
	if(Mot == &Motor_R)
	{
		TMC2660_SPI_SendData(ROT_SPI_CS_GPIO, ROT_SPI_CS_GPIO_PIN, spiData);
	}
	else if(Mot == &Motor_V)
	{
#if defined(USE_MOTOR_PANO_AXIS)
		TMC2660_SPI_SendData(PAN_SPI_CS_GPIO, PAN_SPI_CS_GPIO_PIN, spiData);
#endif /* USE_MOTOR_PANO_AXIS */		
	}
	else if(Mot == &Motor_C)
	{
		TMC2660_SPI_SendData(ROT_SPI_CS_GPIO, ROT_SPI_CS_GPIO_PIN, spiData);
	}
	else if(Mot == &Motor_S)
	{
		TMC2660_SPI_SendData(CC_SPI_CS_GPIO, CC_SPI_CS_GPIO_PIN, spiData);
	}
	else if(Mot == &Motor_T)
	{
#ifdef USE_MOTOR_CHINREST_TS
		TMC2660_SPI_SendData(ER_SPI_CS_GPIO, ER_SPI_CS_GPIO_PIN, spiData);
#endif /* USE_MOTOR_CHINREST_TS */
	}
	else if(Mot == &Motor_A)
	{
#ifdef USE_MOTOR_GANTRY_MS
		TMC2660_SPI_SendData(MS_SPI_CS_GPIO, MS_SPI_CS_GPIO_PIN, spiData);	
#endif /* USE_MOTOR_GANTRY_MS */
	}
	else if(Mot == &Motor_CNS)
	{
#ifdef USE_MOTOR_CHINREST_VER
		TMC2660_SPI_SendData(CNS_SPI_CS_GPIO, CNS_SPI_CS_GPIO_PIN, spiData);
#endif /* USE_MOTOR_CHINREST_VER */
	}
	else if(Mot == &Motor_CWE)
	{
#ifdef USE_MOTOR_CHINREST_HOR
		TMC2660_SPI_SendData(CWE_SPI_CS_GPIO, CWE_SPI_CS_GPIO_PIN, spiData);
#endif 
	}

}



/**
* @ Function Name : TMC2660_SPI_Init
* @ Desc :
* @ Param :
* @ Return :
*/

/*
Chopper Control Register (CHOPCONF)
0x81548, 
1000	0001	0101	0100	1000

Blanking time : Blanking time interval, in system clock periods : 16
CHM Chopper mode : this mode bit affects the interpretation of the HDEC,HEND,HSTRT parameters shown below. : Standard mode(spread Cycle)
Random TOFF time : Chopper off time is fixed as set by bits toff
system clock periods : hysteresis decrement period setting, in system clock periods : 48
Hysteresis is 7 
Hysteresis start value  offset from HEND : 5
slow decay time is a multiple of system clock periods :10

coolStep COntrol Register ( SMARTEN)
0xAA120
1010	1010	0001	0010	0000

Minimum coolStep current : 1 1/4 CS current Setting
Current decrement speed : 8 
Upper coolStep threshold as an offset from the lower threshold : 1
Current increment size : 2
SEMIN is 0 == coolStep is disabled

Driver COntrol Register(DRVCONF)
0xEF010
1110	1111	0000	0001	0000
TST : 0 test mode
Slope control high side : Maximum 
Slope control low side : Maximum
Short to GND protection is enabled
Short to GND detection timer : 3.2us
Enable STEP and DIR interface
Full-scale sense resistor voltage is 305mV
Select value for read out: stallGuard2 level read back


DRVCTRL Register in STEP/DIR Mode
0x00204
0000	0000	0010	0000	0100

INTPOL : Enable Step interpolation : Enable Step pulse multiplication by 16
Micrstep resolution for STEP/DIR mode : Microsteps per 90: 16

*/
void TMC2660_SPI_Resolution_Change(unsigned char resolution){
	if(resolution == 32){
		TMC2660_SPI_SendData(GPIOB, GPIO_Pin_1, 0x00203); //spi_init_data_res32[3]); // 32 resolution
	}
	else if(resolution == 16){
		TMC2660_SPI_SendData(GPIOB, GPIO_Pin_1, 0x00204); //spi_init_data_res32[3]); // 32 resolution
	}
}
static void TMC2660_SPI_Init(void)
{
#define TMC2660_SPI_INIT_DATA_SIZE 4
	uint32_t spi_init_data[TMC2660_SPI_INIT_DATA_SIZE] = { // INTPOL, 16 resolution
					0x81548, 0xAA120 /*0xAA444*/, 0xEF010, 0x00204 }; //0xEF050
	int i;
	if(sysInfo.bShowMotorLog==SET && sysInfo.bShowLog == SET)
	{
		printUart(DBG_MSG_PC, "TMC2660_SPI_Init :: Initial register setting");
	}

	for (i = 0; i < TMC2660_SPI_INIT_DATA_SIZE; i++) {
		TMC2660_SPI_SendData(ROT_SPI_CS_GPIO, ROT_SPI_CS_GPIO_PIN, spi_init_data[i]);
	}

#if defined(USE_MOTOR_PANO_AXIS)
	for (i = 0; i < TMC2660_SPI_INIT_DATA_SIZE; i++) {
		TMC2660_SPI_SendData(PAN_SPI_CS_GPIO, PAN_SPI_CS_GPIO_PIN, spi_init_data[i]);
	}
#endif /* USE_MOTOR_PANO_AXIS */

	if (sysInfo.model_id == MODEL_T2_CS) {
		for (i = 0; i < TMC2660_SPI_INIT_DATA_SIZE; i++) {
			TMC2660_SPI_SendData(CS_SPI_CS_GPIO, CS_SPI_CS_GPIO_PIN, spi_init_data[i]/*init_23hs0041[i]*/);
		}

		for (i = 0; i < TMC2660_SPI_INIT_DATA_SIZE; i++) {
			TMC2660_SPI_SendData(CC_SPI_CS_GPIO, CC_SPI_CS_GPIO_PIN, spi_init_data[i]/*init_23hs0041[i]*/);
		}
	}

#ifdef USE_MOTOR_CHINREST_TS
	for (i = 0; i < TMC2660_SPI_INIT_DATA_SIZE; i++) {
		TMC2660_SPI_SendData(ER_SPI_CS_GPIO, ER_SPI_CS_GPIO_PIN, spi_init_data[i]/*init_motor_er[i]*/);
	}
#endif /* USE_MOTOR_CHINREST_TS */

#ifdef USE_MOTOR_GANTRY_MS
	for (i = 0; i < TMC2660_SPI_INIT_DATA_SIZE; i++) {
		TMC2660_SPI_SendData(MS_SPI_CS_GPIO, MS_SPI_CS_GPIO_PIN, spi_init_data[i]/*init_motor_ms[i]*/); 	
	}
#endif /* USE_MOTOR_GANTRY_MS */

#ifdef USE_MOTOR_CHINREST_VER
	spi_init_data[1] = 0xAA120; //need check
	for (i = 0; i < TMC2660_SPI_INIT_DATA_SIZE; i++) {
		TMC2660_SPI_SendData(CNS_SPI_CS_GPIO, CNS_SPI_CS_GPIO_PIN, spi_init_data[i]/*init_14n2115s4[i]*/);
	}
#endif /* USE_MOTOR_CHINREST_VER */
#ifdef USE_MOTOR_CHINREST_HOR
	spi_init_data[1] = 0xAA120; //need check
	for (i = 0; i < TMC2660_SPI_INIT_DATA_SIZE; i++) {
		TMC2660_SPI_SendData(CWE_SPI_CS_GPIO, CWE_SPI_CS_GPIO_PIN, spi_init_data[i]/*init_14n2115s4[i]*/);
	}
#endif /* USE_MOTOR_CHINREST_HOR */
}


static void TMC2660_SPI_Specific_Init(Motor_Typedef* motor)
{
#define TMC2660_SPI_INIT_DATA_SIZE 4
	uint32_t spi_init_data[TMC2660_SPI_INIT_DATA_SIZE] = { // INTPOL, 16 resolution
					0x81548, 0xAA120 /*0xAA444*/, 0xEF010, 0x00204 }; //0xEF050
	int i;


	if(motor==&Motor_R)
	{
		for (i = 0; i < TMC2660_SPI_INIT_DATA_SIZE; i++) {
			TMC2660_SPI_SendData(ROT_SPI_CS_GPIO, ROT_SPI_CS_GPIO_PIN, spi_init_data[i]);
		}
	}
	else if(motor==&Motor_V)
	{
#if defined(USE_MOTOR_PANO_AXIS)
		for (i = 0; i < TMC2660_SPI_INIT_DATA_SIZE; i++) {
			TMC2660_SPI_SendData(PAN_SPI_CS_GPIO, PAN_SPI_CS_GPIO_PIN, spi_init_data[i]);
		}
#endif /* USE_MOTOR_PANO_AXIS */
	}
	else if(motor==&Motor_C)
	{
		if (sysInfo.model_id == MODEL_T2_CS) {
			for (i = 0; i < TMC2660_SPI_INIT_DATA_SIZE; i++) {
				TMC2660_SPI_SendData(CS_SPI_CS_GPIO, CS_SPI_CS_GPIO_PIN, spi_init_data[i]/*init_23hs0041[i]*/);
			}
		}
	}	
	else if(motor==&Motor_S)
	{	
		if (sysInfo.model_id == MODEL_T2_CS) {
			for (i = 0; i < TMC2660_SPI_INIT_DATA_SIZE; i++) {
				TMC2660_SPI_SendData(CC_SPI_CS_GPIO, CC_SPI_CS_GPIO_PIN, spi_init_data[i]/*init_23hs0041[i]*/);
			}
		}
	}
	else if(motor==&Motor_T)
	{
#ifdef USE_MOTOR_CHINREST_TS
		for (i = 0; i < TMC2660_SPI_INIT_DATA_SIZE; i++) {
			TMC2660_SPI_SendData(ER_SPI_CS_GPIO, ER_SPI_CS_GPIO_PIN, spi_init_data[i]/*init_motor_er[i]*/);
		}
#endif /* USE_MOTOR_CHINREST_TS */
	}
	else if(motor==&Motor_A)
	{
#ifdef USE_MOTOR_GANTRY_MS
		for (i = 0; i < TMC2660_SPI_INIT_DATA_SIZE; i++) {
			TMC2660_SPI_SendData(MS_SPI_CS_GPIO, MS_SPI_CS_GPIO_PIN, spi_init_data[i]/*init_motor_ms[i]*/); 	
		}
#endif /* USE_MOTOR_GANTRY_MS */	
	}
	else if(motor==&Motor_CNS)
	{
#ifdef USE_MOTOR_CHINREST_VER
		for (i = 0; i < TMC2660_SPI_INIT_DATA_SIZE; i++) {
			TMC2660_SPI_SendData(CNS_SPI_CS_GPIO, CNS_SPI_CS_GPIO_PIN, spi_init_data[i]/*init_14n2115s4[i]*/);
		}
#endif /* USE_MOTOR_CHINREST_VER */	
	}
	else if(motor==&Motor_CWE)
	{
#ifdef USE_MOTOR_CHINREST_HOR
		for (i = 0; i < TMC2660_SPI_INIT_DATA_SIZE; i++) {
			TMC2660_SPI_SendData(CWE_SPI_CS_GPIO, CWE_SPI_CS_GPIO_PIN, spi_init_data[i]/*init_14n2115s4[i]*/);
		}
#endif /* USE_MOTOR_CHINREST_HOR */		
	}
	
}


/**
* @ Function Name : Motor_PanoStart
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_PanoStart(void)
{
	TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
#if defined(USE_MOTOR_PANO_AXIS)
	TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
#endif /* USE_MOTOR_PANO_AXIS */

    Motor_MoveArchPosition(&Motor_R);
    Motor_MoveArchPosition(&Motor_V);
}

/**
* @ Function Name : Motor_PanoEnd
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_PanoEnd(void)
{
    Motor_Stop(&Motor_R);
    Motor_Stop(&Motor_V);

	TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StopCurrent);
#if defined(USE_MOTOR_PANO_AXIS)
	TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StopCurrent);
#endif /* USE_MOTOR_PANO_AXIS */
	Timer_Config_T(&Motor_R.Timer);
}

/**
* @ Function Name : Motor_CTEnd
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_CTEnd(void)
{
	//while(Motor_R.Update != RESET);
	Motor_R.Update=FALSE;
	Motor_Stop(&Motor_R);
	Timer_Config_T(&Motor_R.Timer);
	MotorR_InitMinFreq(Motor_R.MinFreq);
	TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StopCurrent);
	TMC2660_SetResolution(Motor_R.MotType, RES_X_16);
}


void Motor_Ceph_StartPosition(void)
{
	uint32_t nStartCnt = 0;
	double dPulse = MOTOR_C_PULLEY_DIAMETER * pi / MOTOR_C_MICROSTEP;
	uint32_t nEndCnt = 300 / dPulse; //CEPH_SCAN_TOTAL_DISTANCE=300
	double StepPerCm=(double)nEndCnt/(double)30;
	double n2nd_EndCnt=0,rate=0;
	int16_t n2nd_StartOffset=0;
	TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StartCurrent);
	TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);	

	if(CephParam.Time==5000)
		n2nd_StartOffset=CephParam.n2ndColFastStartOffset;
	else
		n2nd_StartOffset=CephParam.n2ndColOffset;
	
	n2nd_EndCnt=((CephParam.n2ndColEndOffset+(300 * (MOTOR_C_MICROSTEP / (MOTOR_C_PULLEY_DIAMETER * pi))-CEPH_2ND_COLI_ALIGN_POSITION))-(n2nd_StartOffset+CEPH_2ND_COLI_ALIGN_POSITION));
	
	rate = n2nd_EndCnt/nEndCnt;
	if(sysInfo.bShowLog==SET)
	{
		printUart(DBG_MSG_PC, "rate : %lf", rate);
	}
	/*if(CephParam.Mode==Ceph_Carpus)
	{
		nStartCnt=StepPerCm*4.5;
		nStartCnt-=15;
		Motor_MoveAbsolutePosition(&Motor_C, 2000, 0, CEPH_SCAN_ALIGN_POSITION * (MOTOR_C_MICROSTEP / (MOTOR_C_PULLEY_DIAMETER * pi))+nStartCnt, 100);		
		if(CephParam.bScanEar==RESET)
		{
			Motor_MoveAbsolutePosition(&Motor_S, 2000, 0, CEPH_2ND_COLI_ALIGN_POSITION + CephParam.n2ndColOffset+(nStartCnt*rate), 100);
		}
	}
	else */
	if(CephParam.Mode==Ceph_AP || CephParam.Mode==Ceph_PA || CephParam.Mode==Ceph_Carpus || CephParam.Mode==Ceph_SVM || CephParam.Mode==Ceph_Waters)
	{
		nStartCnt=StepPerCm*3;
		nStartCnt-=15;
		if(CephParam.bScanEar==RESET)
		Motor_MoveAbsolutePosition(&Motor_C, 2000, 0, CEPH_SCAN_ALIGN_POSITION * (MOTOR_C_MICROSTEP / (MOTOR_C_PULLEY_DIAMETER * pi))+nStartCnt, 100);		
		{
			Motor_MoveAbsolutePosition(&Motor_S, 2000, 0, CEPH_2ND_COLI_ALIGN_POSITION + n2nd_StartOffset+(nStartCnt*rate), 100);
		}
	}
	else if(CephParam.Mode==Ceph_EarLod)
	{	
		nStartCnt=StepPerCm*10;
		nStartCnt-=15;
		Motor_MoveAbsolutePosition(&Motor_C, 2000, 0, CEPH_SCAN_ALIGN_POSITION * (MOTOR_C_MICROSTEP / (MOTOR_C_PULLEY_DIAMETER * pi))+nStartCnt, 100);
		if(CephParam.bScanEar==RESET)
		{
			Motor_MoveAbsolutePosition(&Motor_S, 2000, 0, CEPH_2ND_COLI_ALIGN_POSITION + n2nd_StartOffset+(nStartCnt*rate), 100);
		}
	}
	else if(CephParam.Type==Ceph_Child && (CephParam.Mode==Ceph_Lateral||CephParam.Mode==Ceph_Full_Lateral))
	{	
		nStartCnt=StepPerCm*0.5;
		nStartCnt-=15;
		Motor_MoveAbsolutePosition(&Motor_C, 2000, 0, CEPH_SCAN_ALIGN_POSITION * (MOTOR_C_MICROSTEP / (MOTOR_C_PULLEY_DIAMETER * pi))+nStartCnt, 100);
		if(CephParam.bScanEar==RESET)
		{
			Motor_MoveAbsolutePosition(&Motor_S, 2000, 0, CEPH_2ND_COLI_ALIGN_POSITION + n2nd_StartOffset+(nStartCnt*rate), 100);
		}
	}
	else
	{		
		Motor_CephMoveAbsolutePosition(&Motor_C, 2000, 0, CEPH_SCAN_ALIGN_POSITION * (MOTOR_C_MICROSTEP / (MOTOR_C_PULLEY_DIAMETER * pi)), 100);
		if(CephParam.bScanEar==RESET)
		{
			Motor_CephMoveAbsolutePosition(&Motor_S, 2000, 0, CEPH_2ND_COLI_ALIGN_POSITION + n2nd_StartOffset/* + CephParam.n2ndColStartOffset*/, 100);
		}        
	}

	while(!Motor_GetStatus(&Motor_C, STATUS_STOP));
    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));

	TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StopCurrent);
	TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
}


/**
* @ Function Name : Motor_CephScanStart
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_CephScanStart(uint32_t nEndCount)
{
	double n2nd_speed=0;
	double RUNCCR = 0;
	int32_t nEndCount_fast_mode =  0;

	TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StartCurrent);
	TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);

	nEndCount_fast_mode =  nEndCount *2 - CEPH_2ND_COLI_ALIGN_POSITION *2 + 1000;
	// jehun - 20200720
	if(CephParam.Time==5000)
	{
		n2nd_speed= (((CephParam.n2ndColEndOffset+(300 * (MOTOR_C_MICROSTEP / (MOTOR_C_PULLEY_DIAMETER * pi))-CEPH_2ND_COLI_ALIGN_POSITION))-(CephParam.n2ndColFastStartOffset+CEPH_2ND_COLI_ALIGN_POSITION))/(CephParam.Time/1000));
		RUNCCR = TIM_COUNT_CLOCK / (TIM_TOGGLE * n2nd_speed/4) - CephParam.n2ndFastSpeedStepOffset;
	}
	else{
		n2nd_speed= (((CephParam.n2ndColEndOffset+(300 * (MOTOR_C_MICROSTEP / (MOTOR_C_PULLEY_DIAMETER * pi))-CEPH_2ND_COLI_ALIGN_POSITION))-(CephParam.n2ndColOffset+CEPH_2ND_COLI_ALIGN_POSITION))/(CephParam.Time/1000));
		RUNCCR = TIM_COUNT_CLOCK / (TIM_TOGGLE * n2nd_speed/4) - CephParam.n2ndSpeedStepOffset;
	}

	n2nd_speed = TIM_COUNT_CLOCK / RUNCCR / TIM_TOGGLE;

	if(sysInfo.bShowLog==SET)
	{
		if(CephParam.Time==5000)
		{
			printUart(DBG_MSG_PC, "n2nd_speed : %lf", n2nd_speed);
			printUart(DBG_MSG_PC, "Motor_S nDetector_speed : %lf", Motor_S.RunParam.RunFreq);
			printUart(DBG_MSG_PC, "Motor_C nDetector_speed : %lf", Motor_C.RunParam.RunFreq);
			printUart(DBG_MSG_PC, "n2nd_start_offset : %d", CephParam.n2ndColFastStartOffset);
			printUart(DBG_MSG_PC, "n2nd_end_offset : %d", CephParam.n2ndColEndOffset);
			printUart(DBG_MSG_PC, "n2ndFastSpeedStepOffset : %d", CephParam.n2ndFastSpeedStepOffset);
		}
		else
		{
			printUart(DBG_MSG_PC, "n2nd_speed : %lf", n2nd_speed);
			printUart(DBG_MSG_PC, "Motor_S nDetector_speed : %lf", Motor_S.RunParam.RunFreq);
			printUart(DBG_MSG_PC, "Motor_C nDetector_speed : %lf", Motor_C.RunParam.RunFreq);
			printUart(DBG_MSG_PC, "n2nd_start_offset : %d", CephParam.n2ndColOffset);
			printUart(DBG_MSG_PC, "n2nd_end_offset : %d", CephParam.n2ndColEndOffset);
			printUart(DBG_MSG_PC, "n2ndSpeedStepOffset : %d", CephParam.n2ndSpeedStepOffset);
		}
		printUart(DBG_MSG_PC, "Motor_S n2nd_current_step : %d", Motor_S.CurrentStep);
		printUart(DBG_MSG_PC, "Motor_C n2nd_current_step : %d", Motor_C.CurrentStep);
		printUart(DBG_MSG_PC, "Time : %d", CephParam.Time);
		printUart(DBG_MSG_PC, "RUNCCR : %lf", RUNCCR);
	}


//	if(CephParam.Time==5000){
//		//jehun - 20200722 debug
//		TMC2660_SetResolution(Motor_S.MotType, RES_X_32);  // �������� 2��
//		Timer_Config_CC(&Motor_S.Timer, 8);    // �������� 6��
//
//		Motor_MoveAbsolutePosition(&Motor_C, Motor_C.RunParam.RunFreq, 0, (nEndCount + 200), 100);
//
//		if(CephParam.bScanEar==RESET)
//		{
//
//			Motor_MoveAbsolutePosition(&Motor_S, n2nd_speed, 0, (nEndCount_fast_mode), 100);
//		}
//
//	}
//	else
//	{
//		TMC2660_SetResolution(Motor_S.MotType, RES_X_32);  // �������� 2��
//		Timer_Config_CC(&Motor_S.Timer, 8);    // �������� 6��
//
//		Motor_MoveAbsolutePosition(&Motor_C, Motor_C.RunParam.RunFreq, 0, (nEndCount + 200), 100);
//
//		if(CephParam.bScanEar==RESET)
//		{
//
////			Motor_MoveAbsolutePosition(&Motor_S, n2nd_speed, 0, (nEndCount +200 ), 100);
//			Motor_MoveAbsolutePosition(&Motor_S, n2nd_speed, 0, (nEndCount_fast_mode), 100);
//		}
//	}

	// jehun
	TMC2660_SetResolution(Motor_S.MotType, RES_X_32);
	Timer_Config_CC(&Motor_S.Timer, 8);

	Motor_MoveAbsolutePosition(&Motor_C, Motor_C.RunParam.RunFreq, 0, (nEndCount + 200), 100);
	if(CephParam.bScanEar==RESET && Tube.bXrayFrameOnOff==RESET)
	{
		Motor_MoveAbsolutePosition(&Motor_S, n2nd_speed, 0, (nEndCount_fast_mode), 100);
	}
	else if(Tube.bXrayFrameOnOff==SET)
	{
		Motor_S.CurrentStep*=2;
		Motor_MoveAbsolutePosition(&Motor_S, n2nd_speed, 0, 0.1 * (MOTOR_S_MICROSTEP / (MOTOR_S_PULLEY_DIAMETER * pi)), 100);
	}


}
/**
* @ Function Name : Motor_CephScanEnd
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_CephScanEnd(void)
{
	//while(!Motor_GetStatus(&Motor_C, STATUS_STOP));
	//while(!Motor_GetStatus(&Motor_S, STATUS_STOP));

    Motor_Stop(&Motor_C);
    Motor_Stop(&Motor_S);
	Motor_S.CurrentStep /=2;

	TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StopCurrent);
	TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
}

/**
* @ Function Name : Motor_SetCurrent
* @ Desc :
* @ Param :
* @ Return :
*/
static void Motor_SetCurrent(char bStart)
{
	if (bStart) 
	{
		if(sysInfo.bShowLog==SET && sysInfo.bShowMotorLog==SET)
		{
			printUart(DBG_MSG_PC, "Motor_SetCurrent to Start");
		}
		TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
#if defined(USE_MOTOR_PANO_AXIS)
		TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
#endif /* USE_MOTOR_PANO_AXIS */
		if ( sysInfo.model_id == MODEL_T2_CS) 
		{
			TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StartCurrent);
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
		}
#if 0//defined(USE_MOTOR_CHINREST_TS)	
		TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StartCurrent);
#endif /* USE_MOTOR_CHINREST_TS */
#ifdef USE_MOTOR_GANTRY_MS
	   TMC2660_SetCurrent(Motor_A.MotType, Motor_A.StartCurrent);
#endif /* USE_MOTOR_GANTRY_MS */
#ifdef USE_MOTOR_CHINREST_VER
		TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StartCurrent);
#endif /* USE_MOTOR_CHINREST_VER */
#ifdef USE_MOTOR_CHINREST_HOR
		TMC2660_SetCurrent(Motor_CWE.MotType, Motor_CWE.StartCurrent);
#endif /* USE_MOTOR_CHINREST_HOR */
	} 
	else 
	{
		if(sysInfo.bShowLog==SET && sysInfo.bShowMotorLog==SET )
		{
			printUart(DBG_MSG_PC, "Motor_SetCurrent to Stop");
		}
		TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StopCurrent);
#if defined(USE_MOTOR_PANO_AXIS)
		TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StopCurrent);
#endif /* USE_MOTOR_PANO_AXIS */
		if ( sysInfo.model_id == MODEL_T2_CS ) 
		{
			TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StopCurrent);
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
		}
#if 0//defined(USE_MOTOR_CHINREST_TS)	
		TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StopCurrent);
#endif /* USE_MOTOR_CHINREST_TS */
#ifdef USE_MOTOR_GANTRY_MS
	   TMC2660_SetCurrent(Motor_A.MotType, Motor_A.StopCurrent);
#endif /* USE_MOTOR_GANTRY_MS */
#ifdef USE_MOTOR_CHINREST_VER
		TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StopCurrent);
#endif /* USE_MOTOR_CHINREST_VER */
#ifdef USE_MOTOR_CHINREST_HOR
		TMC2660_SetCurrent(Motor_CWE.MotType, Motor_CWE.StopCurrent);
#endif /* USE_MOTOR_CHINREST_HOR */
	}
}

/**
* @ Function Name : TMC2660_SetResolution
* @ Desc :
* @ Param :
* @ Return :
*/
void TMC2660_SetResolution(MotorType motor, MotorResolution res)
{
	const uint32_t arrRes[7] = { 0x00200/*x256*/, 0x00201/*x128*/, 
						0x00202/*x64*/, 0x00203/*x32*/, 0x00204/*x16*/,
						0x00205/*x8*/, 0x00206/*x4*/};
	GPIO_TypeDef *pPort = ROT_SPI_CS_GPIO;
	uint16_t pin = ROT_SPI_CS_GPIO_PIN;
	char str[16] = {0,};

	switch(motor) {
	case TYPE_PAN:
		sprintf(str, "%s", "TYPE_PAN");
		pPort = PAN_SPI_CS_GPIO;
		pin = PAN_SPI_CS_GPIO_PIN;
		break;

	case TYPE_CC:
		sprintf(str, "%s", "TYPE_CC");
		pPort = CC_SPI_CS_GPIO;
		pin = CC_SPI_CS_GPIO_PIN;
		break;

	case TYPE_CS:
		sprintf(str, "%s", "TYPE_CS");
		pPort = CS_SPI_CS_GPIO;
		pin = CS_SPI_CS_GPIO_PIN;
		break;

	case TYPE_ER:
		sprintf(str, "%s", "TYPE_ER");
		pPort = ER_SPI_CS_GPIO;
		pin = ER_SPI_CS_GPIO_PIN;			
		break;

	case TYPE_MS:
		sprintf(str, "%s", "TYPE_MS");
		pPort = MS_SPI_CS_GPIO;
		pin = MS_SPI_CS_GPIO_PIN;			
		break;

	case TYPE_CNS:
		sprintf(str, "%s", "TYPE_CNS");
		pPort = CNS_SPI_CS_GPIO;
		pin = CNS_SPI_CS_GPIO_PIN;			
		break;

	case TYPE_CWE:
		sprintf(str, "%s", "TYPE_CWE");
		pPort = CWE_SPI_CS_GPIO;
		pin = CWE_SPI_CS_GPIO_PIN;			
		break;

	default:			
		sprintf(str, "%s", "TYPE_ROT");
		break;
	}
	if(sysInfo.bShowMotorLog==SET && sysInfo.bShowLog == SET)
	{
		printUart(DBG_MSG_PC, "%s :: arrRes(%d) = 0x%X", str, res, arrRes[res]);
	}
	TMC2660_SPI_SendData(pPort, pin, arrRes[res]);	
}

/**
* @ Function Name : TMC2660_SetCurrent
* @ Desc :
* @ Param :
* @ Return :
*/

/*

stallGuard2 Control Register ((SGCSCONF)
0xD4000-0xD4016
1101	0100	0000	XXXX	XXXX
StallGuard2 Filter Enable : Filtered Mode
stallGuard2 threshold value : 0

const uint32_t arrCurrent[12] = 
{ 0xD3600,//0.125A, 
  0xD3601,//0.25A
  0xD3602,//0.375A
  0xD3603,//0.5A
  0xD3604,//0.625A
  0xD3607,//1.0A
  0xD3609,//1.25A
  0xD360A,//1.375A
  0xD360D,//1.75A
  0xD360F,//2.0A
  0xD3612,//2.25A
  0xD3616//2.75A
  };
*/

/*

stallGuard2 Control Register ((SGCSCONF)
0xD0900-0xD0916
1100	0100	0000	XXXX	XXXX
StallGuard2 Filter Disable : Standard Mode, fastest response time.
stallGuard2 threshold value : 0

const uint32_t arrCurrent[12] = 
{ 0xC3600,//0.125A, 
  0xC3601,//0.25A
  0xC3602,//0.375A
  0xC3603,//0.5A
  0xC3604,//0.625A
  0xC3607,//1.0A
  0xC3609,//1.25A
  0xC360A,//1.375A
  0xC360D,//1.75A
  0xC360F,//2.0A
  0xC3612,//2.25A
  0xC3616//2.75A
  };
*/

/*
const uint32_t arrCurrent[12] = 
{ 0xC0900,//0.125A, 
  0xC0901,//0.25A
  0xC0902,//0.375A
  0xC0903,//0.5A
  0xC0904,//0.625A
  0xC0907,//1.0A
  0xC0909,//1.25A
  0xC090A,//1.375A
  0xC090D,//1.75A
  0xC090F,//2.0A
  0xC0912,//2.25A
  0xC0916//2.75A
  };

*/
/*	const uint32_t arrCurrent[12] = { 0xD0900, 0xD0901,0xD0902, 0xD0903, 0xD0904, 
							0xD0907, 0xD0909, 0xD090A,
							0xD090D, 0xD090F, 0xD0912,
							0xD0916};
*/

void TMC2660_SetCurrent(MotorType motor, MotorCurrent current)
{
	const uint32_t arrCurrent[12] = 
{ 0xD0900,//0.125A, 
  0xD0901,//0.25A
  0xD0902,//0.375A
  0xD0903,//0.5A
  0xD0904,//0.625A
  0xD0907,//1.0A
  0xD0909,//1.25A
  0xD090A,//1.375A
  0xD090D,//1.75A
  0xD090F,//2.0A
  0xD0912,//2.25A
  0xD0916//2.75A
  };
	
	GPIO_TypeDef *pPort = ROT_SPI_CS_GPIO;
	uint16_t pin = ROT_SPI_CS_GPIO_PIN;
	char str[16] = {0,};

	switch(motor) {
	case TYPE_PAN:
		sprintf(str, "%s", "TYPE_PAN");
		pPort = PAN_SPI_CS_GPIO;
		pin = PAN_SPI_CS_GPIO_PIN;
		break;

	case TYPE_CC:
		sprintf(str, "%s", "TYPE_CC");
		pPort = CC_SPI_CS_GPIO;
		pin = CC_SPI_CS_GPIO_PIN;
		break;

	case TYPE_CS:
		sprintf(str, "%s", "TYPE_CS");
		pPort = CS_SPI_CS_GPIO;
		pin = CS_SPI_CS_GPIO_PIN;
		break;

	case TYPE_ER:
		sprintf(str, "%s", "TYPE_ER");
		pPort = ER_SPI_CS_GPIO;
		pin = ER_SPI_CS_GPIO_PIN;			
		break;

	case TYPE_MS:
		sprintf(str, "%s", "TYPE_MS");
		pPort = MS_SPI_CS_GPIO;
		pin = MS_SPI_CS_GPIO_PIN;			
		break;

	case TYPE_CNS:
		sprintf(str, "%s", "TYPE_CNS");
		pPort = CNS_SPI_CS_GPIO;
		pin = CNS_SPI_CS_GPIO_PIN;			
		break;

	case TYPE_CWE:
		sprintf(str, "%s", "TYPE_CWE");
		pPort = CWE_SPI_CS_GPIO;
		pin = CWE_SPI_CS_GPIO_PIN;			
		break;

	default:			
		sprintf(str, "%s", "TYPE_ROT");
		break;
	}
	if(sysInfo.bShowMotorLog==SET && sysInfo.bShowLog == SET)
	{
		printUart(DBG_MSG_PC, "%s :: arrCurrent(%d) = 0x%X", str, current, arrCurrent[current]);
	}
	TMC2660_SPI_SendData(pPort, pin, arrCurrent[current]);	
}

#if defined(USE_CT_STITCH_MODE)
/**
* @ Function Name : Motor_ChangeMode
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_ChangeMode(char bPWM) 
{
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	TIM_OCInitTypeDef TIM_OCStruct;
	GPIO_InitTypeDef GPIO_InitStructure;

#if defined(USE_MOTOR_CHINREST_VER)	
	/* TIMx Deinitialization */
	TIM_DeInit(MOTOR_CNS);

	if (bPWM) {
		//GPIO_InitTypeDef GPIO_InitStruct;

		Timer_Config(&Motor_CNS.Timer);
		TIM_DeInit(MOTOR_CNS);

		if(sysInfo.bShowLog==SET)
		{
			printUart(DBG_MSG_PC, "Motor_PWM_Init");
		}

		/* Enable clock for TIMx */
		RCC_APB1PeriphClockCmd(MOTOR_CNS_CLOCK, ENABLE);

		TIM_BaseStruct.TIM_Prescaler = (uint16_t) ((SystemCoreClock / 2) / TIM_COUNT_CLOCK) - 1;

		/* Count up */
		TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_BaseStruct.TIM_Period = TIM3_PERIOD - 1;
		TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_BaseStruct.TIM_RepetitionCounter = 0;

		/* Initialize TIMx */
		TIM_TimeBaseInit(MOTOR_CNS, &TIM_BaseStruct);

		/* Start count on TIMx */
		//TIM_Cmd(MOTOR_CNS, ENABLE);

		/* PWM mode 2 = Clear on compare match */
		/* PWM mode 1 = Set on compare match */
		TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_High;//TIM_OCPolarity_Low;

		TIM_OCStruct.TIM_Pulse = TIM3_PERIOD /2;

		TIM_OC3Init(MOTOR_CNS, &TIM_OCStruct);
		TIM_OC3PreloadConfig(MOTOR_CNS, TIM_OCPreload_Enable);

		TIM_ARRPreloadConfig(MOTOR_CNS, ENABLE);
		//TIM_CtrlPWMOutputs(MOTOR_CNS, ENABLE);
	}
	else 
	{
		//cns���� �ʱ�ȭ������ �̵� ����
		//Motor_Config();
//		Timer_Config_T(&Motor_CNS.Timer);
//		TIM_DeInit(MOTOR_CNS);
		//Motor_InitParam();
		GPIO_InitStructure.GPIO_Pin = MOTOR_CNS_GPIO_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_Init(MOTOR_CNS_GPIO, &GPIO_InitStructure);

		Timer_Config_T(&Motor_CNS.Timer);

//		Motor_R.OrgSen.uStatus.bit.HistoryOn=1;
//		Motor_R.OrgSen.uStatus.bit.HistoryOff=1;
//		Motor_V.OrgSen.uStatus.bit.HistoryOn=1;
//		Motor_V.OrgSen.uStatus.bit.HistoryOff=1;
//		Motor_C.OrgSen.uStatus.bit.HistoryOn=1;
//		Motor_C.OrgSen.uStatus.bit.HistoryOff=1;
//		Motor_S.OrgSen.uStatus.bit.HistoryOn=1;
//		Motor_S.OrgSen.uStatus.bit.HistoryOff=1;
//		Motor_A.OrgSen.uStatus.bit.HistoryOn=1;
//		Motor_A.OrgSen.uStatus.bit.HistoryOff=1;
//		Motor_CNS.OrgSen.uStatus.bit.HistoryOn=1;
//		Motor_CNS.OrgSen.uStatus.bit.HistoryOff=1;
//		Motor_CWE.OrgSen.uStatus.bit.HistoryOn=1;
//		Motor_CWE.OrgSen.uStatus.bit.HistoryOff=1;
//		Motor_T.OrgSen.uStatus.bit.HistoryOn=1;
//		Motor_T.OrgSen.uStatus.bit.HistoryOff=1;


		//Timer_Config_T(&Motor_CNS.Timer);
		//Timer_Config(&Motor_CWE.Timer);
	}
#endif /* USE_MOTOR_CHINREST_VER */

}

/**
* @ Function Name : Motor_ControlPWM
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_ControlPWM(char bEnable, uint32_t nArr)
{
#if defined(USE_MOTOR_CHINREST_VER)		
	if (bEnable) {
		MOTOR_CNS->ARR = nArr - 1;
		MOTOR_CNS->CCR3 = nArr / 2;

		TIM_Cmd(MOTOR_CNS, ENABLE);
		//TIM_CtrlPWMOutputs(MOTOR_CNS, ENABLE);
	} else {
		//TIM_CtrlPWMOutputs(MOTOR_CNS, DISABLE);		
		TIM_Cmd(MOTOR_CNS, DISABLE);
	}
#endif /* USE_MOTOR_CHINREST_VER */
}

/**
* @ Function Name : Motor_MoveStitchPosition
* @ Desc :
* @ Param :
* @ Return :
*/
bool Motor_MoveStitchPosition(StitchDirection dir,uint32_t dir_count,uint32_t delay)
{
	bool Stitch_Success = SET;
	int8_t	Stitch_Status = 0;
	uint32_t OldnHallSensorCount =0;

#if defined(USE_MOTOR_CHINREST_VER)	
	if(sysInfo.bShowLog == SET)
	{
		printUart(DBG_MSG_PC, "Motor_MoveStitchPosition : Direction(%d)", dir);
	}
	Motor_ChangeMode(1);	

	TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StartCurrent);

	Column_EncControl(SET);
	bColumnStop = RESET;
	nHallSensorCount = 0;
	bTsMotorInterlock = SET;
	limit_count=dir_count;

	if (dir == BOTTOM_UP) 
	{
		Motor_SetOnlyDir(&Motor_CNS, DIR_ORG);		
		Column_Control(COLUMN_UP);

		IntTimer_Delay(500);	
		//Motor_ControlPWM(SET, MOTOR_CNS_INIT_ARR);
	} 
	else 
	{
		Motor_SetOnlyDir(&Motor_CNS, DIR_LIMIT);		
		Column_Control(COLUMN_DOWN);
		//190827 HWAN TEST
		IntTimer_Delay(500);		
	    //while(!IntTimer_GetStatus());
		//Motor_ControlPWM(SET, MOTOR_CNS_INIT_ARR);
	}

	while(!bColumnStop)
	{	
		if(IntTimer_GetStatus())
		{
			if(nHallSensorCount==0)
			{
				UART_SendMessage(DBG_MSG_PC, ERR_CODE_HALL_SENSOR_NOT_START);
				Stitch_Success=RESET;
				break;
			}
			Stitch_Status=1;
			IntTimer_Delay(100);
		}

		if(Stitch_Status==1&&IntTimer_GetStatus())
		{
			IntTimer_Stop();
			if(nHallSensorCount==OldnHallSensorCount)
			{
				UART_SendMessage(DBG_MSG_PC, ERR_CODE_HALL_SENSOR_STUCK);
				Stitch_Success=RESET;
				break;
			}
			OldnHallSensorCount=nHallSensorCount;
			IntTimer_Delay(100);

		}
		else if(nHallSensorCount>limit_count+20)
		{
			UART_SendMessage(DBG_MSG_PC, ERR_CODE_HALL_SENSOR_OVER);
			Stitch_Success=RESET;
			break;
		}
	}
	IntTimer_Stop();

	if(Stitch_Success==RESET)
	{
		Column_Control(COLUMN_STOP);
	}

	IntTimer_Stop();

	if (dir == BOTTOM_UP) 
	{

	}
	else
	{
		if(Motor_CheckOrgType(&Motor_CNS)&& nHallSensorCount!=0) 
		{
			UART_SendMessage(DBG_MSG_PC, ERR_CODE_CNS_NOT_MOVE);
			Stitch_Success=RESET;
		}
	}
	IntTimer_Delay(180);
	while(!IntTimer_GetStatus());

	if((nHallSensorCount>delay+5 || nHallSensorCount<delay-5) && Stitch_Success==SET)  // 64~66mm
	{
		UART_SendMessage(DBG_MSG_PC, ERR_CODE_HALL_SENSOR_NOT_END);
	}
	Motor_ControlPWM(0, 0);
	TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StopCurrent);

	Column_EncControl(RESET);
	if(sysInfo.bShowLog==SET)
	{
		printUart(DBG_MSG_PC, "Count(%d), ARR(%d), CCR3(%d)", nHallSensorCount, \
			MOTOR_CNS->ARR, MOTOR_CNS->CCR3);
	}	
	printUart(DBG_MSG_PC, "Stitch_Success (%d)" ,Stitch_Success);
	Motor_ChangeMode(0);
	CtParam.bStitchDone = SET;
	bTsMotorInterlock = RESET;

	return Stitch_Success;
#endif /* USE_MOTOR_CHINREST_VER */	


}
#endif /* USE_CT_STITCH_MODE && USE_MOTOR_CHINREST_VER*/


/**
* @ Function Name : `
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_Config(void)
{	
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Motor Parameter Init */
    Motor_InitParam();

	/* Enable GPIOx clock */
#if defined(USE_MOTOR_CHINREST_TS) || defined(USE_MOTOR_CHINREST_VER)
	RCC_AHB1PeriphClockCmd(MOTOR_DIR_GPIO_A_CLOCK, ENABLE);
#endif /* USE_MOTOR_CHINREST_TS ||USE_MOTOR_CHINREST_VER */
	RCC_AHB1PeriphClockCmd(MOTOR_DIR_GPIO_C_CLOCK, ENABLE);
	RCC_AHB1PeriphClockCmd(MOTOR_DIR_GPIO_D_CLOCK, ENABLE);
#ifdef USE_MOTOR_CHINREST_HOR
	RCC_AHB1PeriphClockCmd(MOTOR_CWE_ORG_GPIO_CLOCK, ENABLE);
#endif /* USE_MOTOR_CHINREST_HOR */

    /* Motor Dir Port Configuration */
	GPIO_InitStructure.GPIO_Pin = MOTOR_R_DIR_GPIO_PIN |MOTOR_V_DIR_GPIO_PIN;
	if (sysInfo.model_id == MODEL_T2_CS) {
		GPIO_InitStructure.GPIO_Pin |= MOTOR_S_DIR_GPIO_PIN;
	}
#ifdef USE_MOTOR_GANTRY_MS
	GPIO_InitStructure.GPIO_Pin |= MOTOR_A_DIR_GPIO_PIN;
#endif /* USE_MOTOR_GANTRY_MS */
#ifdef USE_MOTOR_CHINREST_HOR
	GPIO_InitStructure.GPIO_Pin |= MOTOR_CWE_DIR_GPIO_PIN;
#endif /* USE_MOTOR_CHINREST_HOR */
	GPIO_InitStructure.GPIO_Mode = MOTOR_DIR_GPIO_MODE;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(MOTOR_DIR_GPIO_D_PORT, &GPIO_InitStructure);

#ifdef USE_MOTOR_CHINREST_VER
	GPIO_InitStructure.GPIO_Pin = MOTOR_CNS_DIR_GPIO_PIN;
	GPIO_Init(MOTOR_DIR_GPIO_C_PORT, &GPIO_InitStructure);
#endif /* USE_MOTOR_CHINREST_VER */
	if (sysInfo.model_id == MODEL_T2_CS) {
		GPIO_InitStructure.GPIO_Pin = MOTOR_C_DIR_GPIO_PIN;
		GPIO_Init(MOTOR_DIR_GPIO_C_PORT, &GPIO_InitStructure);
	}

#ifdef USE_MOTOR_CHINREST_TS
	GPIO_InitStructure.GPIO_Pin = MOTOR_T_DIR_GPIO_PIN;
	GPIO_Init(MOTOR_DIR_GPIO_A_PORT, &GPIO_InitStructure);
#endif /* USE_MOTOR_CHINREST_TS */	

    /* Motor Org Port Configuration */
	GPIO_InitStructure.GPIO_Pin = MOTOR_R_ORG_GPIO_PIN |MOTOR_R_ORG2_GPIO_PIN |MOTOR_V_ORG_GPIO_PIN;
	if (sysInfo.model_id == MODEL_T2_CS) {
		GPIO_InitStructure.GPIO_Pin |= MOTOR_C_ORG_GPIO_PIN |MOTOR_S_ORG_GPIO_PIN;
	}
#ifdef USE_MOTOR_CHINREST_TS
	GPIO_InitStructure.GPIO_Pin |= MOTOR_T_ORG_GPIO_PIN;
#endif /* USE_MOTOR_CHINREST_TS */
#ifdef USE_MOTOR_GANTRY_MS
	GPIO_InitStructure.GPIO_Pin |= MOTOR_A_ORG_GPIO_PIN;
#endif /* USE_MOTOR_GANTRY_MS */
	GPIO_InitStructure.GPIO_Mode = MOTOR_ORG_GPIO_MODE;
	GPIO_Init(MOTOR_ORG_GPIO_D_PORT, &GPIO_InitStructure);

#ifdef USE_MOTOR_CHINREST_VER
	GPIO_InitStructure.GPIO_Pin = MOTOR_CNS_ORG_GPIO_PIN;
	GPIO_Init(MOTOR_DIR_GPIO_A_PORT, &GPIO_InitStructure);
#endif /* USE_MOTOR_CHINREST_VER */
#ifdef USE_MOTOR_CHINREST_HOR
	GPIO_InitStructure.GPIO_Pin = MOTOR_CWE_ORG_GPIO_PIN;
	GPIO_Init(MOTOR_CWE_ORG_GPIO, &GPIO_InitStructure);
#endif /* USE_MOTOR_CHINREST_HOR */

//jehun
//Motor Step Pulse GPIO

	GPIO_InitStructure.GPIO_Pin = MOTOR_R_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(MOTOR_R_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = MOTOR_V_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(MOTOR_V_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = MOTOR_C_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(MOTOR_C_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = MOTOR_S_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(MOTOR_S_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = MOTOR_T_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(MOTOR_T_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = MOTOR_A_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(MOTOR_A_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = MOTOR_CNS_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(MOTOR_CNS_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = MOTOR_CWE_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(MOTOR_CWE_GPIO, &GPIO_InitStructure);

	// MCU Run LED, ERR LED
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


    /* TIMx Configuration */
//    Timer_Config(&Motor_R.Timer);
		Timer_Config_T(&Motor_R.Timer);
//    Timer_Config(&Motor_V.Timer);
		Timer_Config_T(&Motor_V.Timer);
    if (sysInfo.model_id == MODEL_T2_CS) {
//        Timer_Config(&Motor_C.Timer);
			Timer_Config_T(&Motor_C.Timer);
//        Timer_Config(&Motor_S.Timer);
			Timer_Config_T(&Motor_S.Timer); // Second Collimator

    }
#ifdef USE_MOTOR_CHINREST_TS
    Timer_Config_T(&Motor_T.Timer);
#endif /* USE_MOTOR_CHINREST_TS */
#ifdef USE_MOTOR_GANTRY_MS
//    Timer_Config(&Motor_A.Timer);
		Timer_Config_T(&Motor_A.Timer);
#endif /* USE_MOTOR_GANTRY_MS */
#ifdef USE_MOTOR_CHINREST_VER
	//jehun
		Timer_Config_T(&Motor_CNS.Timer);

		//Timer_Config_CNS(&Motor_CNS.Timer);
#endif /* USE_MOTOR_CHINREST_VER */
#ifdef USE_MOTOR_CHINREST_HOR
//	Timer_Config(&Motor_CWE.Timer);
		Timer_Config_T(&Motor_CWE.Timer);
#endif /* USE_MOTOR_CHINREST_HOR */

	SPI_Initialize();
	TMC2660_SPI_Init();

	if(sysInfo.bShowLog==SET)
	{
    	printUart(DBG_MSG_PC, "Motor Config Done");
	}
}

/**
* @ Function Name : Motor_InitParam
* @ Desc :
* @ Param :
* @ Return :
*/
static void Motor_InitParam(void)
{
	/* Motor Timer Parameter Init */
	Motor_R.Timer.Periph				= MOTOR_R;
	Motor_R.Timer.Clock					= MOTOR_R_CLOCK;
	Motor_R.Timer.Gpio					= MOTOR_R_GPIO;
	Motor_R.Timer.GpioClock				= MOTOR_R_GPIO_CLOCK;
	Motor_R.Timer.GpioPin				= MOTOR_R_GPIO_PIN;
	Motor_R.Timer.GpioPinsource			= MOTOR_R_GPIO_PinSource;
	// Motor_R.Timer.GpioAf				= MOTOR_R_GPIO_AF;
	Motor_R.Timer.InitStatus			= MOTOR_R_INIT_STATUS;
	Motor_R.Timer.Interrupt				= MOTOR_R_INTERRUPT;
	Motor_R.Timer.Source				= MOTOR_R_SOURCE;
	// Motor_R.Timer.Channel				= MOTOR_R_CHANNEL;

	Motor_R.Org.Gpio					= MOTOR_R_ORG_GPIO;
	//Motor_R.Org.GpioClock				= MOTOR_R_ORG_GPIO_CLOCK;
	Motor_R.Org.GpioPin					= MOTOR_R_ORG_GPIO_PIN;
	//Motor_R.Org.GpioMode				= MOTOR_R_ORG_GPIO_MODE;

	Motor_R.Org2.Gpio					= MOTOR_R_ORG2_GPIO;
	//Motor_R.Org2.GpioClock			= MOTOR_R_ORG2_GPIO_CLOCK;
	Motor_R.Org2.GpioPin				= MOTOR_R_ORG2_GPIO_PIN;
	//Motor_R.Org2.GpioMode				= MOTOR_R_ORG_GPIO_MODE;

	Motor_R.Dir.Gpio					= MOTOR_R_DIR_GPIO;
	//Motor_R.Dir.GpioClock				= MOTOR_R_DIR_GPIO_CLOCK;
	Motor_R.Dir.GpioPin					= MOTOR_R_DIR_GPIO_PIN;
	//Motor_R.Dir.GpioMode				= MOTOR_R_DIR_GPIO_MODE;

	Motor_R.OrgType 					= MOTOR_R_ORG_TYPE;
	Motor_R.OrgDir						= MOTOR_R_TO_ORG_DIR;

	Motor_R.Status						= STATUS_STOP;
	Motor_R.Update						= RESET;

	Motor_R.MinFreq						= MOTOR_R_MIN_FREQ;
	Motor_R.DelayFreq					= MOTOR_R_DELAY_UNIT_FREQ;
	Motor_R.ToOrgParam.RunFreq			= MOTOR_R_TO_ORG_RUN_FREQ;
	Motor_R.ToOrgParam.RunStep			= MOTOR_R_TO_ORG_RUN_STEP;
	Motor_R.ToOrgParam.AccStep			= MOTOR_R_TO_ORG_ACC_STEP;
	Motor_R.ToOrgParam.DecStep			= MOTOR_R_TO_ORG_DEC_STEP;
	Motor_R.ToLimitParam.RunFreq		= MOTOR_R_TO_LIMIT_RUN_FREQ;
	Motor_R.ToLimitParam.RunStep		= MOTOR_R_TO_LIMIT_RUN_STEP;
	Motor_R.ToLimitParam.AccStep		= MOTOR_R_TO_LIMIT_ACC_STEP;
	Motor_R.ToLimitParam.DecStep		= MOTOR_R_TO_LIMIT_DEC_STEP;

	Motor_R.OrgSen.uStatus.bit.HistoryOn = 0;
	Motor_R.OrgSen.uStatus.bit.HistoryOff = 0;
	Motor_R.nInitRunStep = 0;
#if defined(USE_CT_XRAY_PULSED_MODE)
	Motor_R.StartCurrent = CR_1_8A;
#else /* not USE_CT_XRAY_PULSED_MODE */
	Motor_R.StartCurrent = CR_2_4A;
#endif /* USE_CT_XRAY_PULSED_MODE */
	Motor_R.StopCurrent = CR_1_0A;
	//Motor_R.StopCurrent = CR_0_0A;

	Motor_R.MotType = TYPE_ROT;
	Motor_R.Timer.Priority = Priority_0;
	/* Motor Org Parameter Init */
	Motor_InitOrgParam_R(&Motor_R);

	/* Motor Timer Parameter Initialization */
	Motor_V.Timer.Periph				= MOTOR_V;
	Motor_V.Timer.Clock					= MOTOR_V_CLOCK;
	Motor_V.Timer.Gpio					= MOTOR_V_GPIO;
	Motor_V.Timer.GpioClock				= MOTOR_V_GPIO_CLOCK;
	Motor_V.Timer.GpioPin				= MOTOR_V_GPIO_PIN;
	Motor_V.Timer.GpioPinsource			= MOTOR_V_GPIO_PinSource;
	// Motor_V.Timer.GpioAf				= MOTOR_V_GPIO_AF;
	Motor_V.Timer.InitStatus			= MOTOR_V_INIT_STATUS;
	Motor_V.Timer.Interrupt				= MOTOR_V_INTERRUPT;
	Motor_V.Timer.Source				= MOTOR_V_SOURCE;
	// Motor_V.Timer.Channel				= MOTOR_V_CHANNEL;

	Motor_V.Org.Gpio					= MOTOR_V_ORG_GPIO;
	//Motor_V.Org.GpioClock				= MOTOR_V_ORG_GPIO_CLOCK;
	Motor_V.Org.GpioPin					= MOTOR_V_ORG_GPIO_PIN;
	//Motor_V.Org.GpioMode				= MOTOR_V_ORG_GPIO_MODE;

	Motor_V.Dir.Gpio					= MOTOR_V_DIR_GPIO;
	//Motor_V.Dir.GpioClock				= MOTOR_V_DIR_GPIO_CLOCK;
	Motor_V.Dir.GpioPin					= MOTOR_V_DIR_GPIO_PIN;
	//Motor_V.Dir.GpioMode				= MOTOR_V_DIR_GPIO_MODE;

	Motor_V.OrgType 					= MOTOR_V_ORG_TYPE;
	Motor_V.OrgDir						= MOTOR_V_TO_ORG_DIR;

	Motor_V.Status						= STATUS_STOP;
	Motor_V.Update						= RESET;
	Motor_V.MinFreq						= MOTOR_V_MIN_FREQ;
	Motor_V.DelayFreq					= MOTOR_V_DELAY_UNIT_FREQ;
	Motor_V.ToOrgParam.RunFreq			= MOTOR_V_TO_ORG_RUN_FREQ;
	Motor_V.ToOrgParam.RunStep			= MOTOR_V_TO_ORG_RUN_STEP;
	Motor_V.ToOrgParam.AccStep			= MOTOR_V_TO_ORG_ACC_STEP;
	Motor_V.ToOrgParam.DecStep			= MOTOR_V_TO_ORG_DEC_STEP;
	Motor_V.ToLimitParam.RunFreq		= MOTOR_V_TO_LIMIT_RUN_FREQ;
	Motor_V.ToLimitParam.RunStep		= MOTOR_V_TO_LIMIT_RUN_STEP;
	Motor_V.ToLimitParam.AccStep		= MOTOR_V_TO_LIMIT_ACC_STEP;
	Motor_V.ToLimitParam.DecStep		= MOTOR_V_TO_LIMIT_DEC_STEP;

	Motor_V.OrgSen.uStatus.bit.HistoryOn = 0;
	Motor_V.OrgSen.uStatus.bit.HistoryOff = 0;
	Motor_V.nInitRunStep = 0;

	Motor_V.StartCurrent = CR_1_8A;//CR_2_9A;//CR_2_4A;
	//Motor_V.StopCurrent = CR_1_0A;//CR_2_4A;//CR_1_4A;
	Motor_V.StopCurrent = CR_0_5A;

	Motor_V.MotType = TYPE_PAN;
	Motor_V.Timer.Priority = Priority_1;
	/* Motor Org Parameter Init */
	Motor_InitOrgParam(&Motor_V);

    if (sysInfo.model_id == MODEL_T2_CS) {
        /* Motor Timer Parameter Init */
        Motor_C.Timer.Periph				= MOTOR_C;
        Motor_C.Timer.Clock					= MOTOR_C_CLOCK;
        Motor_C.Timer.Gpio					= MOTOR_C_GPIO;
        Motor_C.Timer.GpioClock				= MOTOR_C_GPIO_CLOCK;
        Motor_C.Timer.GpioPin				= MOTOR_C_GPIO_PIN;
        Motor_C.Timer.GpioPinsource			= MOTOR_C_GPIO_PinSource;
        // Motor_C.Timer.GpioAf				= MOTOR_C_GPIO_AF;
        Motor_C.Timer.InitStatus			= MOTOR_C_INIT_STATUS;
        Motor_C.Timer.Interrupt				= MOTOR_C_INTERRUPT;
        Motor_C.Timer.Source				= MOTOR_C_SOURCE;
        // Motor_C.Timer.Channel				= MOTOR_C_CHANNEL;

        Motor_C.Org.Gpio					= MOTOR_C_ORG_GPIO;
        //Motor_C.Org.GpioClock				= MOTOR_C_ORG_GPIO_CLOCK;
        Motor_C.Org.GpioPin					= MOTOR_C_ORG_GPIO_PIN;
        //Motor_C.Org.GpioMode				= MOTOR_C_ORG_GPIO_MODE;

        Motor_C.Dir.Gpio					= MOTOR_C_DIR_GPIO;
        //Motor_C.Dir.GpioClock				= MOTOR_C_DIR_GPIO_CLOCK;
        Motor_C.Dir.GpioPin					= MOTOR_C_DIR_GPIO_PIN;
        //Motor_C.Dir.GpioMode				= MOTOR_C_DIR_GPIO_MODE;

        Motor_C.OrgType 					= MOTOR_C_ORG_TYPE;
        Motor_C.OrgDir						= MOTOR_C_TO_ORG_DIR;

        Motor_C.Status						= STATUS_STOP;
        Motor_C.Update						= RESET;
        Motor_C.MinFreq						= MOTOR_C_MIN_FREQ;
        Motor_C.DelayFreq					= MOTOR_C_DELAY_UNIT_FREQ;
        Motor_C.ToOrgParam.RunFreq			= MOTOR_C_TO_ORG_RUN_FREQ;
        Motor_C.ToOrgParam.RunStep			= MOTOR_C_TO_ORG_RUN_STEP;
        Motor_C.ToOrgParam.AccStep			= MOTOR_C_TO_ORG_ACC_STEP;
        Motor_C.ToOrgParam.DecStep			= MOTOR_C_TO_ORG_DEC_STEP;
        Motor_C.ToLimitParam.RunFreq		= MOTOR_C_TO_LIMIT_RUN_FREQ;
        Motor_C.ToLimitParam.RunStep		= MOTOR_C_TO_LIMIT_RUN_STEP;
        Motor_C.ToLimitParam.AccStep		= MOTOR_C_TO_LIMIT_ACC_STEP;
        Motor_C.ToLimitParam.DecStep		= MOTOR_C_TO_LIMIT_DEC_STEP;

    	Motor_C.OrgSen.uStatus.bit.HistoryOn = 0;
    	Motor_C.OrgSen.uStatus.bit.HistoryOff = 0;
    	Motor_C.nInitRunStep = 0;

		Motor_C.StartCurrent = CR_1_4A;
		Motor_C.StopCurrent = CR_0_2A; //CR_1_0A;
		//Motor_C.StopCurrent = CR_0_5A; //CR_1_0A;

		Motor_C.MotType = TYPE_CS;
		Motor_C.Timer.Priority = Priority_1;
        /* Motor Org Parameter Init */
        Motor_InitOrgParam(&Motor_C);

        /* Motor Timer Parameter Init */
        Motor_S.Timer.Periph				= MOTOR_S;
        Motor_S.Timer.Clock					= MOTOR_S_CLOCK;
        Motor_S.Timer.Gpio					= MOTOR_S_GPIO;
        Motor_S.Timer.GpioClock				= MOTOR_S_GPIO_CLOCK;
        Motor_S.Timer.GpioPin				= MOTOR_S_GPIO_PIN;
        Motor_S.Timer.GpioPinsource			= MOTOR_S_GPIO_PinSource;
        // Motor_S.Timer.GpioAf				= MOTOR_S_GPIO_AF;
        Motor_S.Timer.InitStatus			= MOTOR_S_INIT_STATUS;
        Motor_S.Timer.Interrupt				= MOTOR_S_INTERRUPT;
        Motor_S.Timer.Source				= MOTOR_S_SOURCE;
        // Motor_S.Timer.Channel				= MOTOR_S_CHANNEL;

        Motor_S.Org.Gpio					= MOTOR_S_ORG_GPIO;
        //Motor_S.Org.GpioClock				= MOTOR_S_ORG_GPIO_CLOCK;
        Motor_S.Org.GpioPin					= MOTOR_S_ORG_GPIO_PIN;
        //Motor_S.Org.GpioMode				= MOTOR_S_ORG_GPIO_MODE;

        Motor_S.Dir.Gpio					= MOTOR_S_DIR_GPIO;
        //Motor_S.Dir.GpioClock				= MOTOR_S_DIR_GPIO_CLOCK;
        Motor_S.Dir.GpioPin					= MOTOR_S_DIR_GPIO_PIN;
        //Motor_S.Dir.GpioMode				= MOTOR_S_DIR_GPIO_MODE;

        Motor_S.OrgType 					= MOTOR_S_ORG_TYPE;
        Motor_S.OrgDir						= MOTOR_S_TO_ORG_DIR;

        Motor_S.Status						= STATUS_STOP;
        Motor_S.Update						= RESET;
        Motor_S.MinFreq						= MOTOR_S_MIN_FREQ;
        Motor_S.DelayFreq					= MOTOR_S_DELAY_UNIT_FREQ;
        Motor_S.ToOrgParam.RunFreq			= MOTOR_S_TO_ORG_RUN_FREQ;
        Motor_S.ToOrgParam.RunStep			= MOTOR_S_TO_ORG_RUN_STEP;
        Motor_S.ToOrgParam.AccStep			= MOTOR_S_TO_ORG_ACC_STEP;
        Motor_S.ToOrgParam.DecStep			= MOTOR_S_TO_ORG_DEC_STEP;
        Motor_S.ToLimitParam.RunFreq		= MOTOR_S_TO_LIMIT_RUN_FREQ;
        Motor_S.ToLimitParam.RunStep		= MOTOR_S_TO_LIMIT_RUN_STEP;
        Motor_S.ToLimitParam.AccStep		= MOTOR_S_TO_LIMIT_ACC_STEP;
        Motor_S.ToLimitParam.DecStep		= MOTOR_S_TO_LIMIT_DEC_STEP;

    	Motor_S.OrgSen.uStatus.bit.HistoryOn = 0;
    	Motor_S.OrgSen.uStatus.bit.HistoryOff = 0;
    	Motor_S.nInitRunStep = 0;

		Motor_S.StartCurrent = CR_1_4A;
		Motor_S.StopCurrent = CR_0_2A; //CR_1_0A;
		//Motor_S.StopCurrent = CR_0_5A; //CR_1_0A;

		Motor_S.MotType = TYPE_CC;
		Motor_S.Timer.Priority = Priority_1;
        /* Motor Org Parameter Init */
        Motor_InitOrgParam(&Motor_S);
    }

#ifdef USE_MOTOR_CHINREST_TS
	/* Motor Timer Parameter Init */
	Motor_T.Timer.Periph				= MOTOR_T;
	Motor_T.Timer.Clock					= MOTOR_T_CLOCK;
	Motor_T.Timer.Gpio					= MOTOR_T_GPIO;
	Motor_T.Timer.GpioClock				= MOTOR_T_GPIO_CLOCK;
	Motor_T.Timer.GpioPin				= MOTOR_T_GPIO_PIN;
	Motor_T.Timer.GpioPinsource			= MOTOR_T_GPIO_PinSource;
//	Motor_T.Timer.GpioAf				= MOTOR_T_GPIO_AF;
//	Motor_T.Timer.InitStatus			= MOTOR_T_INIT_STATUS;
	Motor_T.Timer.Interrupt				= MOTOR_T_INTERRUPT;
	Motor_T.Timer.Source				= MOTOR_T_SOURCE;
//	Motor_T.Timer.Channel				= MOTOR_T_CHANNEL;

	Motor_T.Org.Gpio					= MOTOR_T_ORG_GPIO;
	//Motor_T.Org.GpioClock				= MOTOR_T_ORG_GPIO_CLOCK;
	Motor_T.Org.GpioPin					= MOTOR_T_ORG_GPIO_PIN;
	//Motor_T.Org.GpioMode				= MOTOR_T_ORG_GPIO_MODE;

	Motor_T.Dir.Gpio					= MOTOR_T_DIR_GPIO;
	//Motor_T.Dir.GpioClock				= MOTOR_T_DIR_GPIO_CLOCK;
	Motor_T.Dir.GpioPin					= MOTOR_T_DIR_GPIO_PIN;
	//Motor_T.Dir.GpioMode				= MOTOR_T_DIR_GPIO_MODE;

	Motor_T.OrgType 					= MOTOR_T_ORG_TYPE;
	Motor_T.OrgDir						= MOTOR_T_TO_ORG_DIR;

	Motor_T.Status						= STATUS_STOP;
	Motor_T.Update						= RESET;
	Motor_T.MinFreq						= MOTOR_T_MIN_FREQ;
	Motor_T.DelayFreq					= MOTOR_T_DELAY_UNIT_FREQ;
	Motor_T.ToOrgParam.RunFreq			= MOTOR_T_TO_ORG_RUN_FREQ;
	Motor_T.ToOrgParam.RunStep			= MOTOR_T_TO_ORG_RUN_STEP;
	Motor_T.ToOrgParam.AccStep			= MOTOR_T_TO_ORG_ACC_STEP;
	Motor_T.ToOrgParam.DecStep			= MOTOR_T_TO_ORG_DEC_STEP;
	Motor_T.ToLimitParam.RunFreq		= MOTOR_T_TO_LIMIT_RUN_FREQ;
	Motor_T.ToLimitParam.RunStep		= MOTOR_T_TO_LIMIT_RUN_STEP;
	Motor_T.ToLimitParam.AccStep		= MOTOR_T_TO_LIMIT_ACC_STEP;
	Motor_T.ToLimitParam.DecStep		= MOTOR_T_TO_LIMIT_DEC_STEP;

	Motor_T.OrgSen.uStatus.bit.HistoryOn = 0;
	Motor_T.OrgSen.uStatus.bit.HistoryOff = 0;
	Motor_T.nInitRunStep = 0;

	Motor_T.StartCurrent = CR_0_7A;
	//Motor_T.StopCurrent = CR_0_5A;
	Motor_T.StopCurrent = CR_0_1A;

	Motor_T.MotType = TYPE_ER;
	Motor_T.Timer.Priority = Priority_1;
	/* Motor Org Parameter Init */
	Motor_InitOrgParam(&Motor_T);
#endif /* USE_MOTOR_CHINREST_TS */

#ifdef USE_MOTOR_GANTRY_MS
	/* Motor Timer Parameter Init */
	Motor_A.Timer.Periph				= MOTOR_A;
	Motor_A.Timer.Clock					= MOTOR_A_CLOCK;
	Motor_A.Timer.Gpio					= MOTOR_A_GPIO;
	Motor_A.Timer.GpioClock				= MOTOR_A_GPIO_CLOCK;
	Motor_A.Timer.GpioPin				= MOTOR_A_GPIO_PIN;
	Motor_A.Timer.GpioPinsource			= MOTOR_A_GPIO_PinSource;
	// Motor_A.Timer.GpioAf				= MOTOR_A_GPIO_AF;
	Motor_A.Timer.InitStatus			= MOTOR_A_INIT_STATUS;
	Motor_A.Timer.Interrupt				= MOTOR_A_INTERRUPT;
	Motor_A.Timer.Source				= MOTOR_A_SOURCE;
	// Motor_A.Timer.Channel				= MOTOR_A_CHANNEL;

	Motor_A.Org.Gpio					= MOTOR_A_ORG_GPIO;
	//Motor_A.Org.GpioClock				= MOTOR_A_ORG_GPIO_CLOCK;
	Motor_A.Org.GpioPin					= MOTOR_A_ORG_GPIO_PIN;
	//Motor_A.Org.GpioMode				= MOTOR_A_ORG_GPIO_MODE;

	Motor_A.Dir.Gpio					= MOTOR_A_DIR_GPIO;
	//Motor_A.Dir.GpioClock				= MOTOR_A_DIR_GPIO_CLOCK;
	Motor_A.Dir.GpioPin					= MOTOR_A_DIR_GPIO_PIN;
	//Motor_A.Dir.GpioMode				= MOTOR_A_DIR_GPIO_MODE;

	Motor_A.OrgType 					= MOTOR_A_ORG_TYPE;
	Motor_A.OrgDir						= MOTOR_A_TO_ORG_DIR;

	Motor_A.Status						= STATUS_STOP;
	Motor_A.Update						= RESET;
	Motor_A.MinFreq						= MOTOR_A_MIN_FREQ;
	Motor_A.DelayFreq					= MOTOR_A_DELAY_UNIT_FREQ;
	Motor_A.ToOrgParam.RunFreq			= MOTOR_A_TO_ORG_RUN_FREQ;
	Motor_A.ToOrgParam.RunStep			= MOTOR_A_TO_ORG_RUN_STEP;
	Motor_A.ToOrgParam.AccStep			= MOTOR_A_TO_ORG_ACC_STEP;
	Motor_A.ToOrgParam.DecStep			= MOTOR_A_TO_ORG_DEC_STEP;
	Motor_A.ToLimitParam.RunFreq		= MOTOR_A_TO_LIMIT_RUN_FREQ;
	Motor_A.ToLimitParam.RunStep		= MOTOR_A_TO_LIMIT_RUN_STEP;
	Motor_A.ToLimitParam.AccStep		= MOTOR_A_TO_LIMIT_ACC_STEP;
	Motor_A.ToLimitParam.DecStep		= MOTOR_A_TO_LIMIT_DEC_STEP;

	Motor_A.OrgSen.uStatus.bit.HistoryOn = 0;
	Motor_A.OrgSen.uStatus.bit.HistoryOff = 0;
	Motor_A.nInitRunStep = 0;

	Motor_A.StartCurrent = CR_1_3A;
	//Motor_A.StopCurrent = CR_1_0A;
	Motor_A.StopCurrent = CR_0_2A;

	Motor_A.MotType = TYPE_MS;
	Motor_A.Timer.Priority = Priority_1;
	/* Motor Org Parameter Init */
	Motor_InitOrgParam(&Motor_A);	
#endif /* USE_MOTOR_GANTRY_MS */    

#ifdef USE_MOTOR_CHINREST_VER
	Motor_CNS.Timer.Periph				= MOTOR_CNS;
	Motor_CNS.Timer.Clock				= MOTOR_CNS_CLOCK;
	Motor_CNS.Timer.Gpio				= MOTOR_CNS_GPIO;
	Motor_CNS.Timer.GpioClock			= MOTOR_CNS_GPIO_CLOCK;
	Motor_CNS.Timer.GpioPin				= MOTOR_CNS_GPIO_PIN;
	Motor_CNS.Timer.GpioPinsource		= MOTOR_CNS_GPIO_PinSource;
	Motor_CNS.Timer.GpioAf				= MOTOR_CNS_GPIO_AF;
	Motor_CNS.Timer.InitStatus			= MOTOR_CNS_INIT_STATUS;
	Motor_CNS.Timer.Interrupt			= MOTOR_CNS_INTERRUPT;
	Motor_CNS.Timer.Source				= MOTOR_CNS_SOURCE;
	Motor_CNS.Timer.Channel				= MOTOR_CNS_CHANNEL;

	Motor_CNS.Org.Gpio					= MOTOR_CNS_ORG_GPIO;
	//Motor_CNS.Org.GpioClock				= MOTOR_CNS_ORG_GPIO_CLOCK;
	Motor_CNS.Org.GpioPin				= MOTOR_CNS_ORG_GPIO_PIN;
	//Motor_CNS.Org.GpioMode				= MOTOR_CNS_ORG_GPIO_MODE;

	Motor_CNS.Dir.Gpio					= MOTOR_CNS_DIR_GPIO;
	//Motor_CNS.Dir.GpioClock				= MOTOR_CNS_DIR_GPIO_CLOCK;
	Motor_CNS.Dir.GpioPin				= MOTOR_CNS_DIR_GPIO_PIN;
	//Motor_CNS.Dir.GpioMode				= MOTOR_CNS_DIR_GPIO_MODE;

	Motor_CNS.OrgType 					= MOTOR_CNS_ORG_TYPE;
	Motor_CNS.OrgDir					= MOTOR_CNS_TO_ORG_DIR;

	Motor_CNS.Status					= STATUS_STOP;
	Motor_CNS.Update					= RESET;
	Motor_CNS.MinFreq					= MOTOR_CNS_MIN_FREQ;
	Motor_CNS.DelayFreq					= MOTOR_CNS_DELAY_UNIT_FREQ;
	Motor_CNS.ToOrgParam.RunFreq		= MOTOR_CNS_TO_ORG_RUN_FREQ;
	Motor_CNS.ToOrgParam.RunStep		= MOTOR_CNS_TO_ORG_RUN_STEP;
	Motor_CNS.ToOrgParam.AccStep		= MOTOR_CNS_TO_ORG_ACC_STEP;
	Motor_CNS.ToOrgParam.DecStep		= MOTOR_CNS_TO_ORG_DEC_STEP;
	Motor_CNS.ToLimitParam.RunFreq		= MOTOR_CNS_TO_LIMIT_RUN_FREQ;
	Motor_CNS.ToLimitParam.RunStep		= MOTOR_CNS_TO_LIMIT_RUN_STEP;
	Motor_CNS.ToLimitParam.AccStep		= MOTOR_CNS_TO_LIMIT_ACC_STEP;
	Motor_CNS.ToLimitParam.DecStep		= MOTOR_CNS_TO_LIMIT_DEC_STEP;

	Motor_CNS.OrgSen.uStatus.bit.HistoryOn = 0;
	Motor_CNS.OrgSen.uStatus.bit.HistoryOff = 0;
	Motor_CNS.nInitRunStep = 0;

#if defined(USE_MOTOR_CNS_SCREW_PITCH_6_35)
	Motor_CNS.StartCurrent = CR_1_8A;
	Motor_CNS.StopCurrent = CR_0_7A;
#else /* not USE_MOTOR_CNS_SCREW_PITCH_6_35 */
	//Motor_CNS.StartCurrent = CR_1_0A; //No 2 :: 14N2115D4-160SCSSSN-030
	Motor_CNS.StartCurrent = CR_1_3A; //No 1 :: 14N2110D4-160SCSN-030
	//Motor_CNS.StopCurrent = CR_0_5A;
	Motor_CNS.StopCurrent = CR_0_0A;
#endif /* USE_MOTOR_CNS_SCREW_PITCH_6_35 */

	Motor_CNS.MotType = TYPE_CNS;
	Motor_CNS.Timer.Priority = Priority_1;
	/* Motor Org Parameter Init */
	Motor_InitOrgParam(&Motor_CNS);	
#endif /* USE_MOTOR_CHINREST_VER */

#ifdef USE_MOTOR_CHINREST_HOR
	Motor_CWE.Timer.Periph				= MOTOR_CWE;
	Motor_CWE.Timer.Clock				= MOTOR_CWE_CLOCK;
	Motor_CWE.Timer.Gpio				= MOTOR_CWE_GPIO;
	Motor_CWE.Timer.GpioClock			= MOTOR_CWE_GPIO_CLOCK;
	Motor_CWE.Timer.GpioPin 			= MOTOR_CWE_GPIO_PIN;
	Motor_CWE.Timer.GpioPinsource		= MOTOR_CWE_GPIO_PinSource;
	// Motor_CWE.Timer.GpioAf				= MOTOR_CWE_GPIO_AF;
	Motor_CWE.Timer.InitStatus			= MOTOR_CWE_INIT_STATUS;
	Motor_CWE.Timer.Interrupt			= MOTOR_CWE_INTERRUPT;
	Motor_CWE.Timer.Source				= MOTOR_CWE_SOURCE;
	// Motor_CWE.Timer.Channel 			= MOTOR_CWE_CHANNEL;

	Motor_CWE.Org.Gpio					= MOTOR_CWE_ORG_GPIO;
	//Motor_CWE.Org.GpioClock 			= MOTOR_CWE_ORG_GPIO_CLOCK;
	Motor_CWE.Org.GpioPin				= MOTOR_CWE_ORG_GPIO_PIN;
	//Motor_CWE.Org.GpioMode				= MOTOR_CWE_ORG_GPIO_MODE;

	Motor_CWE.Dir.Gpio					= MOTOR_CWE_DIR_GPIO;
	//Motor_CWE.Dir.GpioClock 			= MOTOR_CWE_DIR_GPIO_CLOCK;
	Motor_CWE.Dir.GpioPin				= MOTOR_CWE_DIR_GPIO_PIN;
	//Motor_CWE.Dir.GpioMode				= MOTOR_CWE_DIR_GPIO_MODE;

	Motor_CWE.OrgType					= MOTOR_CWE_ORG_TYPE;
	Motor_CWE.OrgDir					= MOTOR_CWE_TO_ORG_DIR;

	Motor_CWE.Status					= STATUS_STOP;
	Motor_CWE.Update					= RESET;
	Motor_CWE.MinFreq					= MOTOR_CWE_MIN_FREQ;
	Motor_CWE.DelayFreq 				= MOTOR_CWE_DELAY_UNIT_FREQ;
	Motor_CWE.ToOrgParam.RunFreq		= MOTOR_CWE_TO_ORG_RUN_FREQ;
	Motor_CWE.ToOrgParam.RunStep		= MOTOR_CWE_TO_ORG_RUN_STEP;
	Motor_CWE.ToOrgParam.AccStep		= MOTOR_CWE_TO_ORG_ACC_STEP;
	Motor_CWE.ToOrgParam.DecStep		= MOTOR_CWE_TO_ORG_DEC_STEP;
	Motor_CWE.ToLimitParam.RunFreq		= MOTOR_CWE_TO_LIMIT_RUN_FREQ;
	Motor_CWE.ToLimitParam.RunStep		= MOTOR_CWE_TO_LIMIT_RUN_STEP;
	Motor_CWE.ToLimitParam.AccStep		= MOTOR_CWE_TO_LIMIT_ACC_STEP;
	Motor_CWE.ToLimitParam.DecStep		= MOTOR_CWE_TO_LIMIT_DEC_STEP;

	Motor_CWE.OrgSen.uStatus.bit.HistoryOn = 0;
	Motor_CWE.OrgSen.uStatus.bit.HistoryOff = 0;
	Motor_CWE.nInitRunStep = 0;

	Motor_CWE.StartCurrent = CR_1_0A;
	//Motor_CWE.StopCurrent = CR_0_5A;//CR_0_7A;
	Motor_CWE.StopCurrent = CR_0_2A;

	Motor_CWE.MotType = TYPE_CWE;
	Motor_CWE.Timer.Priority = Priority_1;
	/* Motor Org Parameter Init */
	Motor_InitOrgParam(&Motor_CWE); 	
#endif /* USE_MOTOR_CHINREST_HOR */
}

/**
* @ Function Name : Motor_InitOrgParam
* @ Desc :
* @ Param :
* @ Return :
*/
static void Motor_InitOrgParam(Motor_Typedef* motor)
{
    motor->MinFreqCCR = Hz_To_CCR(motor->MinFreq);
    motor->Timer.InitCCR = motor->MinFreqCCR;
    motor->DelayFreqCCR = Hz_To_CCR(motor->DelayFreq);

    Motor_InitRunParam(&motor->ToOrgParam, motor->MinFreqCCR);
    Motor_InitRunParam(&motor->ToLimitParam, motor->MinFreqCCR);
}

/**
* @ Function Name : Motor_InitRunParam
* @ Desc :
* @ Param :
* @ Return :
*/
static void Motor_InitOrgParam_R(Motor_Typedef* motor)
{
    motor->MinFreqCCR = (double) 2500000 / (TIM_TOGGLE * motor->MinFreq);
    motor->Timer.InitCCR = motor->MinFreqCCR;
    motor->DelayFreqCCR = (double) 2500000 / (TIM_TOGGLE * motor->DelayFreq);

    Motor_InitRunParam_R(&motor->ToOrgParam, motor->MinFreqCCR);
    Motor_InitRunParam_R(&motor->ToLimitParam, motor->MinFreqCCR);
}

/**
* @ Function Name : Motor_InitRunParam
* @ Desc :
* @ Param :
* @ Return :
*/
static void Motor_InitRunParam(RunParam_Typedef* param, double ccr)
{
    param->RunCCR = Hz_To_CCR(param->RunFreq);
    param->AccCCR = Motor_CalcCCRValue(ccr, param->RunCCR, param->AccStep);
    param->DecCCR = Motor_CalcCCRValue(ccr, param->RunCCR, param->DecStep);
}

static void Motor_InitRunParam_R(RunParam_Typedef* param, double ccr)
{
    param->RunCCR = (double) 2500000 / (TIM_TOGGLE * param->RunFreq);
    param->AccCCR = Motor_CalcCCRValue(ccr, param->RunCCR, param->AccStep);
    param->DecCCR = Motor_CalcCCRValue(ccr, param->RunCCR, param->DecStep);
}
/**
* @ Function Name : Motor_CalcCCRValue
* @ Desc :
* @ Param :
* @ Return :
*/
static double Motor_CalcCCRValue(double ccr1, double ccr2, uint32_t step)
{
	if(ccr1==ccr2) return 500/step;
	else return (ccr1 - ccr2) / step;
}

/**
* @ Function Name : Motor_ClearCurrentStep
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_ClearCurrentStep(Motor_Typedef *pMotor)
{
	pMotor->CurrentStep = 0;
}

/**
* @ Function Name : Motor_SetOnlyDir
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_SetOnlyDir(Motor_Typedef *pMot, MotorDir_Typedef dir)
{
	if(sysInfo.bShowLog==SET)
	{
		printUart(DBG_MSG_PC, "Motor_SetOnlyDir : dir(%d)", dir);
	}

	if (dir == DIR_LIMIT) {
		if (pMot->OrgDir == ORGDIR_LOW)
			GPIO_SetBits(pMot->Dir.Gpio, pMot->Dir.GpioPin);
		else
			GPIO_ResetBits(pMot->Dir.Gpio, pMot->Dir.GpioPin);			
	} else {
		if (pMot->OrgDir == ORGDIR_LOW)
			GPIO_ResetBits(pMot->Dir.Gpio, pMot->Dir.GpioPin);
		else
			GPIO_SetBits(pMot->Dir.Gpio, pMot->Dir.GpioPin);
	}
}

/**
* @ Function Name : Motor_Start
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_Start(Motor_Typedef* motor)
{
    TIM_SetCounter(motor->Timer.Periph, 0);

    if (Motor_GetStatus(motor, STATUS_ARCH))
        motor->NextCCR = motor->ArchParam.IndexCcr;
    else if(Motor_GetStatus(motor, STATUS_CT))
		motor->NextCCR = (motor->RunParam.RunCCR*motor->RunParam.AccStep*5)/(motor->RunParam.AccStep+4);
	else if(Motor_GetStatus(motor, STATUS_ORG))
	{		
		if(motor == &Motor_R)
		{
			motor->NextCCR = motor->ToOrgParam.RunCCR*((double)(motor->ToOrgParam.AccStep));
		}
		else
		{
			motor->NextCCR = motor->MinFreqCCR;
		}		
	}
	else if(Motor_GetStatus(motor, STATUS_LIMIT))
	{
		if(motor == &Motor_R)
		{
			motor->NextCCR = motor->ToLimitParam.RunCCR*((double)(motor->ToLimitParam.AccStep));			
		}
		else
		{
			motor->NextCCR = motor->MinFreqCCR;
		}
	}
	else if(Motor_GetStatus(motor, STATUS_REVERSE_ORG))
	{		
		if(motor == &Motor_R)
		{
			motor->NextCCR = motor->ToOrgParam.RunCCR*((double)(motor->ToOrgParam.AccStep));
		}
		else
		{
			motor->NextCCR = motor->MinFreqCCR;
		}		
	}
	else if(Motor_GetStatus(motor, STATUS_REVERSE_LIMIT))
	{
		if(motor == &Motor_R)
		{
			motor->NextCCR = motor->ToLimitParam.RunCCR*((double)(motor->ToLimitParam.AccStep));			
		}
		else
		{
			motor->NextCCR = motor->MinFreqCCR;
		}
	}
	else 
	{
		if(motor == &Motor_R)
		{
			motor->NextCCR = (motor->RunParam.RunCCR*motor->RunParam.AccStep*5)/(motor->RunParam.AccStep+4);//motor->NextCCR = motor->RunParam.RunCCR*((double)(motor->RunParam.AccStep));//
		}
		else
		{
			motor->NextCCR = motor->MinFreqCCR;
			//motor->NextCCR = (motor->RunParam.RunCCR*motor->RunParam.AccStep*3)/(motor->RunParam.AccStep+2);
		}		
	}

    //jehun - 20200720
    // CNS Stitching ������ ���ܵ�, ������ ���� I/O�����?�ʿ� ����
    if(motor == &Motor_CNS){
    	switch(motor->Timer.Channel) {
    	case TIM_Channel_1:
    		TIM_SetCompare1(motor->Timer.Periph, motor->NextCCR + 0.5);
    		break;
    	case TIM_Channel_2:
    		TIM_SetCompare2(motor->Timer.Periph, motor->NextCCR + 0.5);
    		break;
    	case TIM_Channel_3:
    		TIM_SetCompare3(motor->Timer.Periph, motor->NextCCR + 0.5);
    		break;
    	case TIM_Channel_4:
    		TIM_SetCompare4(motor->Timer.Periph, motor->NextCCR + 0.5);
    		break;
    	default: // error
    		break;
    	}
    }
    TIM_Cmd(motor->Timer.Periph, ENABLE);
}

/**
* @ Function Name : Motor_Stop
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_Stop(Motor_Typedef* motor)
{		
	// jehun - 20200723
	// Ceph 2�� Collimaotor �ӵ� ������
	if(motor == &Motor_S)
	{
		TMC2660_SetResolution(Motor_S.MotType, RES_X_16);
		Timer_Config_CC(&Motor_S.Timer, 1);
	}
	TIM_Cmd(motor->Timer.Periph, DISABLE);
	//TMC2660_SetCurrent(motor->Timer.Periph, 0);
	Motor_SetStatus(motor, STATUS_STOP);
}

/**
* @ Function Name : Motor_GetStatus
* @ Desc :
* @ Param :
* @ Return :
*/
bool Motor_GetStatus(Motor_Typedef* motor, MotorStatus_Typedef status)
{
	return (motor->Status  == status) ? SET : RESET;
}

/**
* @ Function Name : Motor_SetStatus
* @ Desc :
* @ Param :
* @ Return :
*/
static void Motor_SetStatus(Motor_Typedef* motor, MotorStatus_Typedef status)
{
	motor->Status = status;
}

/**
* @ Function Name : MotorR_CheckOrgSequence
* @ Desc :
* @ Param :
* @ Return :
*/
bool MotorR_CheckOrgSequence(void)
{
	double R_ORG_SEQ2_MIN_STEP =(10.0 / ( 360.0 /MOTOR_R_MICROSTEP__ / MOTOR_R_PULLEY_RATIO));
	double R_ORG_SEQ2_MAX_STEP=(350.0 / ( 360.0 /MOTOR_R_MICROSTEP__ / MOTOR_R_PULLEY_RATIO));
	double R_ORG_SEQ2_STICH_MIN_STEP=(85.0 / ( 360.0 /MOTOR_R_MICROSTEP__ / MOTOR_R_PULLEY_RATIO));

	if(CtParam.bStitchMode==SET && CtParam.bStitchORGCheck==SET)
	{
		if ( Motor_R.CurrentStep < R_ORG_SEQ2_STICH_MIN_STEP)
		{
			bIsOrgSeq2 = SET;
			return SET;
		}
	}
	else if(CtParam.bReverse_Status==SET)
	{
		if ( Motor_R.CurrentStep < R_ORG_SEQ2_STICH_MIN_STEP)
		{
			bIsOrgSeq2 = SET;
			return SET;
		}
	}
	else
	{
		if ( (Motor_R.CurrentStep > R_ORG_SEQ2_MIN_STEP) && (GPIO_ReadInputDataBit(Motor_R.Org.Gpio, Motor_R.Org.GpioPin) || (Motor_R.CurrentStep > R_ORG_SEQ2_MAX_STEP)) )
		{
			bIsOrgSeq2 = SET;
			return SET;
		}
	}

    return RESET;
}

/**
* @ Function Name : Motor_CheckOrgType
* @ Desc :
* @ Param :
* @ Return :
*/
static bool Motor_CheckOrgType(Motor_Typedef* motor)
{
	bool ret = RESET;

	if (motor->OrgType == ORG1_LOW) 
	{
		ret = (!GPIO_ReadInputDataBit(motor->Org.Gpio, motor->Org.GpioPin)) ? SET : RESET;
	} 
	else if (motor->OrgType == ORG1_HIGH) 
	{
		ret = (GPIO_ReadInputDataBit(motor->Org.Gpio, motor->Org.GpioPin)) ? SET : RESET;
	} 
	else if (motor->OrgType == ORG1LOW_ORG2LOW) 
	{
		ret = (!GPIO_ReadInputDataBit(motor->Org.Gpio, motor->Org.GpioPin)) &&\
				(!GPIO_ReadInputDataBit(motor->Org2.Gpio, motor->Org2.GpioPin)) ? SET : RESET;
	} 
	else if (motor->OrgType == ORG1LOW_ORG2HIGH) 
	{
		ret = (!GPIO_ReadInputDataBit(motor->Org.Gpio, motor->Org.GpioPin)) &&\
				(GPIO_ReadInputDataBit(motor->Org2.Gpio, motor->Org2.GpioPin)) ? SET : RESET;
	} 
	else if (motor->OrgType == ORG1HIGH_ORG2LOW) 
	{
		if (/*(motor->Timer.Periph == MOTOR_R) &&*/(bIsOrgSeq2 == SET))
		{
			static bool bSkiped = RESET;
			if (bSkiped == RESET)
			{
				if (!GPIO_ReadInputDataBit(motor->Org.Gpio, motor->Org.GpioPin))
					return RESET;

				bSkiped = SET;
			} 
			else 
			{
				if (GPIO_ReadInputDataBit(motor->Org.Gpio, motor->Org.GpioPin))
					return RESET;

				bIsOrgSeq2 = RESET;
				bSkiped = RESET;
			}
		} 
		else 
		{
			ret = (GPIO_ReadInputDataBit(motor->Org.Gpio, motor->Org.GpioPin)) && \
				(!GPIO_ReadInputDataBit(motor->Org2.Gpio, motor->Org2.GpioPin)) ? SET : RESET;
		}
	}
	else if (motor->OrgType  == ORG1HIGH_ORG2HIGH) 
	{

		ret = (GPIO_ReadInputDataBit(motor->Org.Gpio, motor->Org.GpioPin)) &&\
				(GPIO_ReadInputDataBit(motor->Org2.Gpio, motor->Org2.GpioPin)) ? SET : RESET;
	}

	if (ret == RESET)
	{
		motor->OrgSen.uStatus.bit.HistoryOff = 1;
	}
	else 
	{
		motor->OrgSen.uStatus.bit.HistoryOn = 1;
	}

	return ret;	
}

/**
* @ Function Name : Motor_CheckOrgType
* @ Desc :
* @ Param :
* @ Return :
*/
static bool Motor_Reverse_CheckOrgType(Motor_Typedef* motor)
{
	bool ret = RESET;
	
	
	if (motor->Status== STATUS_REVERSE_ORG) 
	{
		ret = (GPIO_ReadInputDataBit(motor->Org.Gpio, motor->Org.GpioPin)) && \
				(GPIO_ReadInputDataBit(motor->Org2.Gpio, motor->Org2.GpioPin)) ? SET : RESET;
	}
	else if (motor->Status== STATUS_REVERSE_FIN) 
	{
		ret = (!GPIO_ReadInputDataBit(motor->Org.Gpio, motor->Org.GpioPin)) && \
				(GPIO_ReadInputDataBit(motor->Org2.Gpio, motor->Org2.GpioPin)) ? SET : RESET;
	}
	else//if (motor->Status== STATUS_REVERSE_LIMIT) 
	{
		ret = (!GPIO_ReadInputDataBit(motor->Org2.Gpio, motor->Org2.GpioPin)) ? SET : RESET;
	}
	
	if (ret == RESET)
	{
		motor->OrgSen.uStatus.bit.HistoryOff = 1;
	}
	else 
	{
		motor->OrgSen.uStatus.bit.HistoryOn = 1;
	}

	return ret;	
}



/**
* @ Function Name : Motor_SetDir
* @ Desc :
* @ Param :
* @ Return :
*/
static void Motor_SetDir(Motor_Typedef* motor, MotorDir_Typedef direction)
{
	motor->MotorDir = direction;

	if (direction == DIR_LIMIT) {
		if (motor->OrgDir == ORGDIR_LOW)
			GPIO_SetBits(motor->Dir.Gpio, motor->Dir.GpioPin);
		else
			GPIO_ResetBits(motor->Dir.Gpio, motor->Dir.GpioPin);			
	} else {
		if (motor->OrgDir == ORGDIR_LOW)
			GPIO_ResetBits(motor->Dir.Gpio, motor->Dir.GpioPin);
		else
			GPIO_SetBits(motor->Dir.Gpio, motor->Dir.GpioPin);
	}
}

/**
* @ Function Name : Motor_SetArchParam
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_SetArchParam(Motor_Typedef* motor)
{
    if (motor->ArchParam.StepArray[motor->ArchParam.ArrayIndex] != 0) 
	{
        if (TIM_OCInitStructure.TIM_OutputState == TIM_OutputState_Disable) 
		{
            TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

            if (motor->Timer.Source == TIM_IT_CC1)
                TIM_OC1Init(motor->Timer.Periph, &TIM_OCInitStructure);
            else if (motor->Timer.Source == TIM_IT_CC2)
                TIM_OC2Init(motor->Timer.Periph, &TIM_OCInitStructure);
            else if (motor->Timer.Source == TIM_IT_CC3)
                TIM_OC3Init(motor->Timer.Periph, &TIM_OCInitStructure);
            else if (motor->Timer.Source == TIM_IT_CC4)
                TIM_OC4Init(motor->Timer.Periph, &TIM_OCInitStructure);
        }

        if (motor->ArchParam.StepArray[motor->ArchParam.ArrayIndex] > 0) 
		{
            Motor_SetDir(motor, DIR_LIMIT);
            motor->ArchParam.IndexStep = motor->ArchParam.StepArray[motor->ArchParam.ArrayIndex];
        } 
		else 
		{
            Motor_SetDir(motor, DIR_ORG);
            motor->ArchParam.IndexStep = (-1) * motor->ArchParam.StepArray[motor->ArchParam.ArrayIndex];
        }
    }
	else 
	{
    
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;

        if (motor->Timer.Source == TIM_IT_CC1)
            TIM_OC1Init(motor->Timer.Periph, &TIM_OCInitStructure);
        else if(motor->Timer.Source == TIM_IT_CC2)
            TIM_OC2Init(motor->Timer.Periph, &TIM_OCInitStructure);
        else if(motor->Timer.Source == TIM_IT_CC3)
            TIM_OC3Init(motor->Timer.Periph, &TIM_OCInitStructure);
        else if(motor->Timer.Source == TIM_IT_CC4)
            TIM_OC4Init(motor->Timer.Periph, &TIM_OCInitStructure);

        motor->ArchParam.IndexStep = 1;
    }

    //jehun - ����, �ĳ� �Կ� Ȯ�� �ʿ�
    motor->ArchParam.IndexCcr = motor->ArchParam.CcrArray[motor->ArchParam.ArrayIndex] -1;
}

/**
* @ Function Name : Motor_ProcPanoMode
* @ Desc :
* @ Param :
* @ Return :
*/
static void Motor_ProcPanoMode(void)
{
    switch(PanoParam.Mode) {
    case PANO_MODE_NORMAL:
    case PANO_MODE_SINUS:
	case PANO_MODE_3D:
        if (Motor_R.ArchParam.ArrayIndex == Motor_R.ArchParam.AccSize -1) 
		{
            Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
        }		
		else if(Tube.bXrayFrameOnOff==SET && (Motor_R.ArchParam.ArrayIndex == (Motor_R.ArchParam.AccSize + 10)))
		{
			Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
		}
		else if(Tube.bXrayFrameOnOff==SET && (Motor_R.ArchParam.ArrayIndex == (Motor_R.ArchParam.nExpOffIndex - 10)))
		{
			Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
			Tube.bXrayFrameOnOff=RESET;
		}
		else if (Motor_R.ArchParam.ArrayIndex == Motor_R.ArchParam.nExpOffIndex) 
        {
            Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
            Tube_CtrlReady(TUBE_READY_DISABLE);
            Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
        }
        break;
	case PANO_MODE_ALIGN:
		PanoCapture_ProcAlignMode();
		break;
    case PANO_MODE_LEFT:
        PanoCapture_ProcLeftMode();
        break;

    case PANO_MODE_FRONT:
        PanoCapture_ProcFrontMode();
        break;

    case PANO_MODE_RIGHT:
        PanoCapture_ProcRightMode();
        break;

    case PANO_MODE_TMJ:
        PanoCapture_ProcTmjMode();
        break;

    case PANO_MODE_NONE:
    default:
        UART_SendMessage(DBG_MSG_PC, "Motor_ProcPanoMode check failed");
        break;
    }
}

/**
* @ Function Name : Motor_GetNextOrgCCR
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_GetNextOrgCCR(Motor_Typedef* motor)
{
	if (motor->Status == STATUS_LIMIT) 
	{
		if (Motor_CheckOrgType(motor)) 
		{
			motor->ToOrgStep++;

			if (motor->ToOrgStep <= motor->ToLimitParam.AccStep)
			{				
				//motor->NextCCR = (motor->ToLimitParam.RunCCR*motor->ToLimitParam.AccStep*3)/(motor->ToLimitParam.AccStep+2*motor->ToOrgStep);
				motor->NextCCR -= motor->ToLimitParam.AccCCR;
				motor->ToLimitParam.CurrCCR=motor->NextCCR;
			}
			else
				motor->NextCCR = motor->ToLimitParam.RunCCR;				
		} 
		else 
		{
			motor->OrgOverStep++;

			if (motor->OrgOverStep == motor->ToLimitParam.RunStep) 
			{
				motor->CurrentStep = motor->ToLimitParam.RunStep;
				Motor_Stop(motor);
				return;
			} 
			else if (motor->OrgOverStep > motor->ToLimitParam.RunStep) 
			{
				if(motor == &Motor_V)
					UART_SendMessage(DBG_MSG_PC, "Motor_V ORG Limit Step Error...!!!");
				else if(motor == &Motor_A)
					UART_SendMessage(DBG_MSG_PC, "Motor_A ORG Limit Step Error...!!!");
				else if(motor == &Motor_C)
					UART_SendMessage(DBG_MSG_PC, "Motor_C ORG Limit Step Error...!!!");
				else if(motor == &Motor_S)
					UART_SendMessage(DBG_MSG_PC, "Motor_S ORG Limit Step Error...!!!");
				else if(motor == &Motor_T)
					UART_SendMessage(DBG_MSG_PC, "Motor_T ORG Limit Step Error...!!!");
				else if(motor == &Motor_CNS)
					UART_SendMessage(DBG_MSG_PC, "Motor_CNS ORG Limit Step Error...!!!");
				else if(motor == &Motor_CWE)
		        	UART_SendMessage(DBG_MSG_PC, "Motor_CWE ORG Limit Step Error...!!!");
			}

			if (motor->OrgOverStep <= motor->ToLimitParam.RunStep - motor->ToLimitParam.DecStep)
			{
				if(motor->ToOrgStep >= motor->ToLimitParam.AccStep)
					motor->NextCCR = motor->ToLimitParam.RunCCR;
				else
					motor->NextCCR = motor->ToLimitParam.CurrCCR;
			}
			else
			{	
				//motor->NextCCR = (motor->ToLimitParam.RunCCR*motor->ToLimitParam.DecStep*3)/(motor->ToLimitParam.DecStep+2*(motor->ToLimitParam.RunStep-motor->ToOrgStep));
				motor->NextCCR += motor->ToLimitParam.DecCCR;
			}

		}		
	} 
	else if (motor->Status == STATUS_ORG) 
	{
		if (!Motor_CheckOrgType(motor))
		{
			motor->ToOrgStep++;
			if (motor->ToOrgStep <= motor->ToOrgParam.AccStep)
			{
				//motor->NextCCR = (motor->ToOrgParam.RunCCR*motor->ToOrgParam.AccStep*3)/(motor->ToOrgParam.AccStep+2*motor->ToOrgStep);
				motor->NextCCR -= motor->ToOrgParam.AccCCR;
				motor->ToOrgParam.CurrCCR=motor->NextCCR;
			}
			else
				motor->NextCCR = motor->ToOrgParam.RunCCR;
		} 
		else 
		{
			motor->OrgOverStep++;

			if (motor->OrgOverStep == motor->ToOrgParam.RunStep) 
			{
				Motor_SetStatus(motor, STATUS_DELAY);

				TIM_CCxCmd(motor->Timer.Periph, motor->Timer.Channel, TIM_CCx_Disable);
				motor->NextCCR = motor->DelayFreqCCR;

				motor->ToOrgStep = 0;
				motor->OrgOverStep = 0;				

				return;

			} 
			else if (motor->OrgOverStep > motor->ToOrgParam.RunStep) 
			{
				if(motor == &Motor_V)
					UART_SendMessage(DBG_MSG_PC, "Motor_V ORG ToORG Step Error...!!!");
				else if(motor == &Motor_A)
					UART_SendMessage(DBG_MSG_PC, "Motor_A ORG ToORG Step Error...!!!");
				else if(motor == &Motor_C)
					UART_SendMessage(DBG_MSG_PC, "Motor_C ORG ToORG Step Error...!!!");
				else if(motor == &Motor_S)
					UART_SendMessage(DBG_MSG_PC, "Motor_S ORG ToORG Step Error...!!!");
				else if(motor == &Motor_T)
					UART_SendMessage(DBG_MSG_PC, "Motor_T ORG ToORG Step Error...!!!");
				else if(motor == &Motor_CNS)
					UART_SendMessage(DBG_MSG_PC, "Motor_CNS ORG ToORG Step Error...!!!");
				else if(motor == &Motor_CWE)
		        	UART_SendMessage(DBG_MSG_PC, "Motor_CWE ORG ToORG Step Error...!!!");
		        //UART_SendMessage(DBG_MSG_PC, "Motor Error...!!!"); // ���� ����
			}

			if (motor->OrgOverStep <= motor->ToOrgParam.RunStep - motor->ToOrgParam.DecStep)
			{
				if(motor->ToOrgStep >= motor->ToOrgParam.AccStep)
					motor->NextCCR = motor->ToOrgParam.RunCCR;
				else
					motor->NextCCR = motor->ToOrgParam.CurrCCR;
			}
			else
			{				
				//motor->NextCCR = (motor->ToOrgParam.RunCCR*motor->ToOrgParam.DecStep*3)/(motor->ToOrgParam.DecStep+2*(motor->ToOrgParam.RunStep - motor->ToOrgStep));
				motor->NextCCR += motor->ToOrgParam.DecCCR;
			}

		}
	} 
	else if (motor->Status == STATUS_DELAY) 
	{
		motor->DelayCnt++;

		if (motor->DelayCnt == MOTOR_DELAY_TIME) 
		{
			Motor_SetStatus(motor, STATUS_LIMIT);
			//motor->NextCCR = (motor->ToLimitParam.RunCCR*motor->ToLimitParam.AccStep*3)/(motor->ToLimitParam.AccStep+2);
			motor->NextCCR = motor->MinFreqCCR;
			Motor_SetDir(motor, DIR_LIMIT);

			TIM_CCxCmd(motor->Timer.Periph, motor->Timer.Channel, TIM_CCx_Enable);

			return;
		} 
		else if(motor->DelayCnt > MOTOR_DELAY_TIME) 
		{
			//
		}
		//motor->NextCCR = motor->DelayFreqCCR;
	}
}

/**
* @ Function Name : Motor_GetNextOrgCCR
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_GetNextOrgCCR_R(Motor_Typedef* motor)
{
	if (motor->Status == STATUS_LIMIT) 
	{
		if (Motor_CheckOrgType(motor)) 
		{
			motor->ToOrgStep++;

			if (motor->ToOrgStep <= motor->ToLimitParam.AccStep)
			{				
				//motor->NextCCR = motor->ToLimitParam.RunCCR*((double)(motor->ToLimitParam.AccStep/motor->ToOrgStep));
				motor->NextCCR = (motor->ToLimitParam.RunCCR*motor->ToLimitParam.AccStep*5)/(motor->ToLimitParam.AccStep+motor->ToOrgStep*4);
				motor->ToLimitParam.CurrCCR = motor->NextCCR;
			}
			else
				motor->NextCCR = motor->ToLimitParam.RunCCR;				
		} 
		else 
		{
			motor->OrgOverStep++;

			if (motor->OrgOverStep == motor->ToLimitParam.RunStep) 
			{
				motor->CurrentStep = motor->ToLimitParam.RunStep;
				Motor_Stop(motor);

				return;
			} 
			else if (motor->OrgOverStep > motor->ToLimitParam.RunStep) 
			{
		        UART_SendMessage(DBG_MSG_PC, "Motor_R ORG Limit Step Error...!!!");
			}

			if ((motor->ToOrgStep + motor->OrgOverStep) < motor->ToLimitParam.AccStep)
			{
				motor->NextCCR = (motor->ToLimitParam.RunCCR*motor->ToLimitParam.AccStep*5)/(motor->ToLimitParam.AccStep+(motor->ToOrgStep +motor->OrgOverStep)*4);
				motor->ToLimitParam.CurrCCR = motor->NextCCR;
			}
			else if (motor->OrgOverStep <= motor->ToLimitParam.RunStep - motor->ToLimitParam.DecStep)
			{
				motor->NextCCR = motor->ToLimitParam.RunCCR;
			}
			else
			{
				motor->NextCCR = (motor->ToLimitParam.RunCCR*motor->ToLimitParam.DecStep*5)/(motor->ToLimitParam.DecStep+(motor->ToLimitParam.RunStep-motor->OrgOverStep)*4);
				//motor->NextCCR = motor->ToLimitParam.RunCCR*sqrt((double)(motor->ToLimitParam.DecStep/(motor->ToLimitParam.RunStep-motor->OrgOverStep)));
			}

		}		
	} 
	else if (motor->Status == STATUS_ORG) 
	{
		if (!Motor_CheckOrgType(motor))
		{
			motor->ToOrgStep++;
			if (motor->ToOrgStep <= motor->ToOrgParam.AccStep)
			{
				//motor->NextCCR = motor->ToOrgParam.RunCCR*((double)(motor->ToOrgParam.AccStep/motor->ToOrgStep));
				motor->NextCCR = (motor->ToOrgParam.RunCCR*motor->ToOrgParam.AccStep*5)/(motor->ToOrgParam.AccStep+motor->ToOrgStep*4);
				motor->ToOrgParam.CurrCCR = motor->NextCCR;
			}
			else
				motor->NextCCR = motor->ToOrgParam.RunCCR;
		} 
		else 
		{
			motor->OrgOverStep++;

			if (motor->OrgOverStep == motor->ToOrgParam.RunStep) 
			{
				Motor_SetStatus(motor, STATUS_DELAY);

				TIM_CCxCmd(motor->Timer.Periph, motor->Timer.Channel, TIM_CCx_Disable);
				motor->NextCCR = motor->DelayFreqCCR;

				motor->ToOrgStep = 0;
				motor->OrgOverStep = 0;				

				return;

			} 
			else if (motor->OrgOverStep > motor->ToOrgParam.RunStep) 
			{
		        UART_SendMessage(DBG_MSG_PC, "Motor_R ORG ToOrg Step Error...!!!"); // ���� ����
			}

			if (motor->OrgOverStep <= motor->ToOrgParam.RunStep - motor->ToOrgParam.DecStep)
			{
				if (motor->ToOrgStep >= motor->ToOrgParam.AccStep)
					motor->NextCCR = motor->ToOrgParam.RunCCR;
				else
					motor->NextCCR = motor->ToOrgParam.CurrCCR;
			}
			else
			{				
				if (motor->ToOrgStep >= motor->ToOrgParam.AccStep)
				{
					//motor->NextCCR = motor->ToOrgParam.RunCCR*sqrt((double)(motor->ToOrgParam.DecStep/(motor->ToOrgParam.RunStep-motor->OrgOverStep)));
					motor->NextCCR = (motor->ToOrgParam.RunCCR*motor->ToOrgParam.DecStep*5)/(motor->ToOrgParam.DecStep+(motor->ToOrgParam.RunStep-motor->OrgOverStep)*4);
				}
				else
				{
					motor->NextCCR = (motor->ToOrgParam.CurrCCR*motor->ToOrgParam.DecStep*5)/(motor->ToOrgParam.DecStep+(motor->ToOrgParam.RunStep-motor->OrgOverStep)*4);
				}
			}


		}
	} 
	else if (motor->Status == STATUS_DELAY) 
	{
		motor->DelayCnt++;

		if (motor->DelayCnt == 1500)
		{
			Motor_SetStatus(motor, STATUS_LIMIT);
			//motor->NextCCR = motor->ToLimitParam.RunCCR*((double)(motor->ToLimitParam.AccStep/motor->ToOrgStep));
			motor->NextCCR = (motor->ToLimitParam.RunCCR*motor->ToLimitParam.AccStep*5)/(motor->ToLimitParam.AccStep+4);
			Motor_SetDir(motor, DIR_LIMIT);

			TIM_CCxCmd(motor->Timer.Periph, motor->Timer.Channel, TIM_CCx_Enable);

			return;
		} 
		else if(motor->DelayCnt > MOTOR_DELAY_TIME) 
		{
			//
		}
		//motor->NextCCR = motor->DelayFreqCCR;
	}
}


/**
* @ Function Name : Motor_GetNextOrgCCR
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_GetNextReverse_OrgCCR_R(Motor_Typedef* motor)
{
	if (motor->Status == STATUS_REVERSE_FIN) 
	{
		if (!Motor_Reverse_CheckOrgType(motor)) 
		{
			motor->ToOrgStep++;

			if (motor->ToOrgStep <= motor->ToLimitParam.AccStep)
			{				
				//motor->NextCCR = motor->ToLimitParam.RunCCR*((double)(motor->ToLimitParam.AccStep/motor->ToOrgStep));
				motor->NextCCR = (motor->ToLimitParam.RunCCR*motor->ToLimitParam.AccStep*5)/(motor->ToLimitParam.AccStep+motor->ToOrgStep*4);
			}
			else
				motor->NextCCR = motor->ToLimitParam.RunCCR;				
		} 
		else 
		{
			motor->OrgOverStep++;

			if (motor->OrgOverStep == motor->ToLimitParam.RunStep) 
			{
				motor->CurrentStep = motor->ToLimitParam.RunStep;
				Motor_Stop(motor);

				return;
			} 
			else if (motor->OrgOverStep > motor->ToLimitParam.RunStep) 
			{
		        UART_SendMessage(DBG_MSG_PC, "Motor_R ORG Limit Step Error...!!!");
			}

			if ((motor->ToOrgStep + motor->OrgOverStep) < motor->ToLimitParam.AccStep)
			{
				motor->NextCCR = (motor->ToLimitParam.RunCCR*motor->ToLimitParam.AccStep*5)/(motor->ToLimitParam.AccStep+(motor->ToOrgStep +motor->OrgOverStep)*4);
			}
			else if (motor->OrgOverStep <= motor->ToLimitParam.RunStep - motor->ToLimitParam.DecStep)
			{
				motor->NextCCR = motor->ToLimitParam.RunCCR;
			}
			else
			{	
				//motor->NextCCR = motor->ToLimitParam.RunCCR*sqrt((double)(motor->ToLimitParam.DecStep/(motor->ToLimitParam.RunStep-motor->OrgOverStep)));
				motor->NextCCR = (motor->ToLimitParam.RunCCR*motor->ToLimitParam.DecStep*5)/(motor->ToLimitParam.DecStep+(motor->ToLimitParam.RunStep-motor->OrgOverStep)*4);
			}

		}		
	} 
	else if (motor->Status == STATUS_REVERSE_ORG) 
	{
		if (!Motor_Reverse_CheckOrgType(motor))
		{
			motor->ToOrgStep++;
			if (motor->ToOrgStep <= motor->ToOrgParam.AccStep)
			{
				//motor->NextCCR = motor->ToOrgParam.RunCCR*((double)(motor->ToOrgParam.AccStep/motor->ToOrgStep));
				motor->NextCCR = (motor->ToOrgParam.RunCCR*motor->ToOrgParam.AccStep*5)/(motor->ToOrgParam.AccStep+motor->ToOrgStep*4);
				motor->ToOrgParam.CurrCCR = motor->NextCCR;
			}
			else
				motor->NextCCR = motor->ToOrgParam.RunCCR;
		} 
		else 
		{
			motor->OrgOverStep++;

			if (motor->OrgOverStep == motor->ToOrgParam.RunStep) 
			{
				Motor_SetStatus(motor, STATUS_DELAY_FIN);

				TIM_CCxCmd(motor->Timer.Periph, motor->Timer.Channel, TIM_CCx_Disable);
				motor->NextCCR = motor->DelayFreqCCR;

				motor->ToOrgStep = 0;
				motor->OrgOverStep = 0;				
				motor->DelayCnt=0;
				return;

			} 
			else if (motor->OrgOverStep > motor->ToOrgParam.RunStep) 
			{
		        UART_SendMessage(DBG_MSG_PC, "Motor_R ORG ToOrg Step Error...!!!"); // ���� ����
			}

			if (motor->OrgOverStep <= motor->ToOrgParam.RunStep - motor->ToOrgParam.DecStep)
			{
				motor->NextCCR = motor->ToOrgParam.RunCCR;
			}
			else
			{				
				if (motor->ToOrgStep >= motor->ToOrgParam.DecStep){
					//motor->NextCCR = motor->ToOrgParam.RunCCR*sqrt((double)(motor->ToOrgParam.DecStep/(motor->ToOrgParam.RunStep-motor->OrgOverStep)));
					motor->NextCCR = (motor->ToOrgParam.RunCCR*motor->ToOrgParam.DecStep*5)/(motor->ToOrgParam.DecStep+(motor->ToOrgParam.RunStep-motor->OrgOverStep)*4);
				}
				else
				{
					motor->NextCCR = (motor->ToOrgParam.CurrCCR*motor->ToOrgParam.DecStep*5)/(motor->ToOrgParam.DecStep+(motor->ToOrgParam.RunStep-motor->OrgOverStep)*4);
				}
			}


		}
	} 
	else if (motor->Status == STATUS_REVERSE_LIMIT) 
	{
		if (Motor_Reverse_CheckOrgType(motor))
		{
			motor->ToOrgStep++;
			if (motor->ToOrgStep <= motor->ToLimitParam.AccStep)
			{
				//motor->NextCCR = motor->ToOrgParam.RunCCR*((double)(motor->ToOrgParam.AccStep/motor->ToOrgStep));
				motor->NextCCR = (motor->ToLimitParam.RunCCR*motor->ToLimitParam.AccStep*5)/(motor->ToLimitParam.AccStep+motor->ToOrgStep*4);
				motor->ToLimitParam.CurrCCR = motor->NextCCR;
			}
			else
				motor->NextCCR = motor->ToLimitParam.RunCCR;
		} 
		else 
		{
			motor->OrgOverStep++;

			if (motor->OrgOverStep == motor->ToLimitParam.RunStep) 
			{
				Motor_SetStatus(motor, STATUS_DELAY_ORG);

				TIM_CCxCmd(motor->Timer.Periph, motor->Timer.Channel, TIM_CCx_Disable);
				motor->NextCCR = motor->DelayFreqCCR;

				motor->ToOrgStep = 0;
				motor->OrgOverStep = 0;		
				motor->DelayCnt=0;
				return;

			} 
			else if (motor->OrgOverStep > motor->ToLimitParam.RunStep) 
			{
		        UART_SendMessage(DBG_MSG_PC, "Motor_R ORG ToOrg Step Error...!!!"); // ���� ����
			}

			if (motor->OrgOverStep <= motor->ToLimitParam.RunStep - motor->ToLimitParam.DecStep)
			{
				motor->NextCCR = motor->ToLimitParam.RunCCR;
			}
			else
			{				
				if (motor->ToOrgStep >= motor->ToLimitParam.DecStep){
					//motor->NextCCR = motor->ToOrgParam.RunCCR*sqrt((double)(motor->ToOrgParam.DecStep/(motor->ToOrgParam.RunStep-motor->OrgOverStep)));
					motor->NextCCR = (motor->ToLimitParam.RunCCR*motor->ToLimitParam.DecStep*5)/(motor->ToLimitParam.DecStep+(motor->ToLimitParam.RunStep-motor->OrgOverStep)*4);
				}
				else
				{
					motor->NextCCR = (motor->ToLimitParam.CurrCCR*motor->ToLimitParam.DecStep*5)/(motor->ToLimitParam.DecStep+(motor->ToLimitParam.RunStep-motor->OrgOverStep)*4);
				}
			}


		}
	}
	else if (motor->Status == STATUS_DELAY_ORG) 
	{
		motor->DelayCnt++;

		if (motor->DelayCnt == 2500)
		{
			Motor_SetStatus(motor, STATUS_REVERSE_ORG);
			motor->NextCCR = (motor->ToOrgParam.RunCCR*motor->ToOrgParam.AccStep*5)/(motor->ToOrgParam.AccStep+4);
			Motor_SetDir(motor, DIR_ORG);
			motor->DelayCnt=0;

			TIM_CCxCmd(motor->Timer.Periph, motor->Timer.Channel, TIM_CCx_Enable);

			return;
		} 
		else if(motor->DelayCnt > MOTOR_DELAY_TIME) 
		{
			//
		}
		//motor->NextCCR = motor->DelayFreqCCR;
	}
	else if (motor->Status == STATUS_DELAY_FIN) 
	{
		motor->DelayCnt++;

		if (motor->DelayCnt == 2500)
		{
			Motor_SetStatus(motor, STATUS_REVERSE_FIN);
			motor->NextCCR = (motor->ToLimitParam.RunCCR*motor->ToLimitParam.AccStep*5)/(motor->ToLimitParam.AccStep+4);
			Motor_SetDir(motor, DIR_LIMIT);
			motor->DelayCnt=0;

			TIM_CCxCmd(motor->Timer.Periph, motor->Timer.Channel, TIM_CCx_Enable);

			return;
		} 
		else if(motor->DelayCnt > MOTOR_DELAY_TIME) 
		{
			//
		}
		//motor->NextCCR = motor->DelayFreqCCR;
	}
}

/**
* @ Function Name : Motor_GetNextRunCCR
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_GetNextRunCCR(Motor_Typedef* motor)
{
	motor->RunStep++;

	if (motor->MotorDir == DIR_LIMIT)
		motor->CurrentStep++;
	else
		motor->CurrentStep--;

	if(motor->RunStep == motor->RunParam.RunStep) 
	{
		Motor_Stop(motor);
		return;
	} 
	else if(motor->RunStep > motor->RunParam.RunStep) 
	{
		if(motor == &Motor_R)
			UART_SendMessage(DBG_MSG_PC, "Motor_R Run Step Error...!!!");
		else if(motor == &Motor_V)
			UART_SendMessage(DBG_MSG_PC, "Motor_V Run Step Error...!!!");
		else if(motor == &Motor_A)
			UART_SendMessage(DBG_MSG_PC, "Motor_A Run Step Error...!!!");
		else if(motor == &Motor_C)
			UART_SendMessage(DBG_MSG_PC, "Motor_C Run Step Error...!!!");
		else if(motor == &Motor_S)
			UART_SendMessage(DBG_MSG_PC, "Motor_S Run Step Error...!!!");
		else if(motor == &Motor_T)
			UART_SendMessage(DBG_MSG_PC, "Motor_T Run Step Error...!!!");
		else if(motor == &Motor_CNS)
			UART_SendMessage(DBG_MSG_PC, "Motor_CNS Run Step Error...!!!");
		else if(motor == &Motor_CWE)
        	UART_SendMessage(DBG_MSG_PC, "Motor_CWE Run Step Error...!!!");
        //UART_SendMessage(DBG_MSG_PC, "Motor Error...!!!");
	}

	if (motor->RunStep <= motor->RunParam.AccStep)
		motor->NextCCR = motor->NextCCR - motor->RunParam.AccCCR;
		//motor->NextCCR = motor->RunParam.RunCCR*((double)(motor->RunParam.AccStep/motor->RunStep));
	else if(motor->RunStep <= motor->RunParam.RunStep - motor->RunParam.DecStep)
		motor->NextCCR = motor->RunParam.RunCCR;
	else
		motor->NextCCR = motor->NextCCR + motor->RunParam.DecCCR;
		//motor->NextCCR = motor->RunParam.RunCCR*sqrt((double)(motor->RunParam.DecStep/(motor->RunParam.RunStep-motor->RunStep)));

}

/**
* @ Function Name : Motor_GetNextRunCCR
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_GetNextRunCCR_R(Motor_Typedef* motor)
{
	motor->RunStep++;

	if (motor->MotorDir == DIR_LIMIT)
		motor->CurrentStep++;
	else
		motor->CurrentStep--;

	if(motor->RunStep == motor->RunParam.RunStep) 
	{
		Motor_Stop(motor);
		return;
	} 
	else if(motor->RunStep > motor->RunParam.RunStep) 
	{
        UART_SendMessage(DBG_MSG_PC, "Motor_R Run Step Error...!!!");
	}

	if (motor->RunStep <= motor->RunParam.AccStep)
		motor->NextCCR = (motor->RunParam.RunCCR*motor->RunParam.AccStep*5)/(motor->RunParam.AccStep+motor->RunStep*4);
		//motor->NextCCR = motor->RunParam.RunCCR*((double)(motor->RunParam.AccStep/motor->RunStep));
	else if(motor->RunStep <= motor->RunParam.RunStep - motor->RunParam.DecStep)
		motor->NextCCR = motor->RunParam.RunCCR;
	else
		motor->NextCCR = (motor->RunParam.RunCCR*motor->RunParam.DecStep*5)/(motor->RunParam.DecStep+(motor->RunParam.RunStep-motor->RunStep)*4);
		//motor->NextCCR = motor->RunParam.RunCCR*sqrt((double)(motor->RunParam.DecStep/(motor->RunParam.RunStep-motor->RunStep)));

}


/**
* @ Function Name : Motor_GetNextArchCCR
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_GetNextArchCCR(Motor_Typedef* motor)
{
    motor->RunStep++;

    if (motor->MotorDir == DIR_LIMIT)
        motor->CurrentStep++;
    else
        motor->CurrentStep--;

    if (motor->RunStep == motor->ArchParam.IndexStep) 
	{
        motor->RunStep = 0;
        motor->ArchParam.ArrayIndex++;

        //if (motor->Timer.Periph == MOTOR_R)
            Motor_ProcPanoMode();

        if (motor->ArchParam.ArrayIndex == motor->ArchParam.TotalSize) 
		{
            Motor_Stop(motor);
    		//PanoSensor_Stop();
            return;
        }
		else 
		{
            Motor_SetArchParam(motor);
            motor->NextCCR = motor->ArchParam.IndexCcr;
        }
    }
}

/**
* @ Function Name : Motor_Get_NextCT_CCR
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_Get_NextCT_CCR(Motor_Typedef* motor)
{
	motor->RunStep++;

	if (motor->MotorDir == DIR_LIMIT)
		motor->CurrentStep++;
	else
		motor->CurrentStep--;

	if (motor->RunStep == motor->RunParam.RunStep) {
		Motor_Stop(motor);
		return;
	}

	if (motor->RunStep <= motor->RunParam.AccStep) 
	{
		//smotor->NextCCR = motor->NextCCR - motor->RunParam.AccCCR;		
		//motor->NextCCR = motor->RunParam.RunCCR*(motor->RunParam.AccStep/motor->RunStep);
		motor->NextCCR = (motor->RunParam.RunCCR*motor->RunParam.AccStep*5)/(motor->RunParam.AccStep+motor->RunStep*4);
	}
	else if (motor->RunStep <= CtParam.nRunStep)
	{
		motor->NextCCR = motor->RunParam.RunCCR;
	}
	else
	{
		//motor->NextCCR = motor->NextCCR + motor->RunParam.DecCCR;
		motor->NextCCR = (motor->RunParam.RunCCR*motor->RunParam.DecStep*5)/(motor->RunParam.DecStep+(CtParam.nEndStep+1000-motor->RunStep)*4);
		//motor->NextCCR = motor->RunParam.RunCCR*sqrt(motor->RunParam.DecStep/(motor->RunParam.RunStep-motor->RunStep));
	}

    if (motor->CurrentStep == CtParam.nXrayOnStep)
	{
        Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
#if defined(USE_CT_XRAY_PULSED_MODE) 
#if defined(USE_TUBE_PPS_TYPE_IO)
		Sensor_C.bSetFrameCount = SET;
#else /* not USE_TUBE_PPS_TYPE_IO */
		if (CtParam.TubeMode == CT_TUBE_PULSED)
		{
			Tube_Start();
		}
#endif /* USE_TUBE_PPS_TYPE_IO */
#endif /* USE_CT_XRAY_PULSED_MODE */
#ifdef BUILD_TYPE_DEBUG
		typElapsedTimer.nExposeTime = vSetCurrentMilliSec();
#endif /* BUILD_TYPE_DEBUG */
    }
	else if (motor->CurrentStep == CtParam.nXrayOffStep) 
	{
#if defined(USE_CT_XRAY_PULSED_MODE) 
#if defined(USE_TUBE_PPS_TYPE_IO)
		Sensor_C.bSetFrameCount = RESET;
#else /* not USE_TUBE_PPS_TYPE_IO */
		Tube_Stop();
#endif /* USE_TUBE_PPS_TYPE_IO */
#endif /* USE_CT_XRAY_PULSED_MODE && !USE_TUBE_PPS_TYPE_IO */
        Tube_CtrlReady(TUBE_READY_DISABLE);
        Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE); // 1.0.0e
#ifdef BUILD_TYPE_DEBUG
		typElapsedTimer.nExposeTime = nElapsedMilliSec(typElapsedTimer.nExposeTime);
#endif /* BUILD_TYPE_DEBUG */ 
		CtParam.bCaptEnd = SET;
    }/*
	else if (motor->CurrentStep == CtParam.nEndStep) 
	{
        CtParam.bCaptEnd = SET;
    }*/
}

/**
* @ Function Name : Motor_RunOrgCheck
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_RunOrgCheck(Motor_Typedef* motor)
{
	if (!Motor_GetStatus(motor, STATUS_STOP))
		Motor_Stop(motor);

	if (Motor_CheckOrgType(motor)) {
		Motor_SetStatus(motor, STATUS_LIMIT);
		Motor_SetDir(motor, DIR_LIMIT);
	} else {
		Motor_SetStatus(motor, STATUS_ORG);
		Motor_SetDir(motor, DIR_ORG);
	}

	motor->ToOrgStep = 0;
	motor->OrgOverStep = 0;
	motor->DelayCnt = 0;

	Motor_Start(motor);
}

/**
* @ Function Name : Motor_RunOrgCheck
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_Reverse_OrgCheck(Motor_Typedef* motor)
{
	if (!Motor_GetStatus(motor, STATUS_STOP))
		Motor_Stop(motor);

	if (Motor_Reverse_CheckOrgType(motor)) 
	{
		Motor_SetStatus(motor, STATUS_REVERSE_LIMIT);
		Motor_SetDir(motor, DIR_LIMIT);
	} 
	else 
	{
		Motor_SetStatus(motor, STATUS_REVERSE_ORG);
		Motor_SetDir(motor, DIR_ORG);
	}

	motor->ToOrgStep = 0;
	motor->OrgOverStep = 0;
	motor->DelayCnt = 0;

	Motor_Start(motor);
}


/**
* @ Function Name : Motor_CephMoveAbsolutePosition
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_CephMoveAbsolutePosition(Motor_Typedef* motor, double freq, uint32_t acc, int32_t run, uint32_t dec)
{	
	Motor_MoveAbsoluteDistance(motor, freq, acc, run - motor->CurrentStep, dec);
}

/**
* @ Function Name : Motor_MoveAbsolutePosition
* @ Desc :
* @ Param :
* @ Return :
*/

void Motor_MoveAbsolutePosition(Motor_Typedef* motor, double freq, uint32_t acc, int32_t run, uint32_t dec)
{
	Motor_MoveAbsoluteDistanceAccDec(motor, freq, acc, run - motor->CurrentStep, dec);
}

/**
* @ Function Name : Motor_MoveAbsolutePosition
* @ Desc :
* @ Param :
* @ Return :
*/
void MotorR_MoveAbsolutePosition_CT(Motor_Typedef* motor, double freq, uint32_t acc, int32_t run, uint32_t dec)
{	
	if(CtParam.TotalFrame==400)
		MotorR_MoveAbsoluteDistanceAccDec_CT(motor, freq, acc, run - motor->CurrentStep, dec,2500000.0);
	else
		MotorR_MoveAbsoluteDistanceAccDec_CT(motor, freq, acc, run - motor->CurrentStep, dec,5000000.0);
}


#ifdef USE_CT_STITCH_MODE
/**
* @ Function Name : Motor_SetDirMovePosistion
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_SetDirMovePosistion(Motor_Typedef *pMot, MotorDir_Typedef dir, double freq, int32_t run)
{	
	double TimClk;
	if(CtParam.TotalFrame==400)
		TimClk=2500000.0;
	else
		TimClk=5000000.0;

	pMot->MotorDir = dir;
	pMot->RunParam.RunStep = run;

	pMot->RunParam.RunFreq = freq;
    pMot->RunParam.AccStep = 2000;
    pMot->RunParam.DecStep = 2000;

	//Motor_InitRunParam(&pMot->RunParam, pMot->MinFreqCCR);
	pMot->RunParam.RunCCR = (double) TimClk / (2 * pMot->RunParam.RunFreq);
    pMot->RunParam.AccCCR = Motor_CalcCCRValue(pMot->MinFreqCCR, pMot->RunParam.RunCCR, pMot->RunParam.AccStep);
    pMot->RunParam.DecCCR = Motor_CalcCCRValue(pMot->MinFreqCCR, pMot->RunParam.RunCCR, pMot->RunParam.DecStep);

	Motor_SetStatus(pMot, STATUS_RUN);
	Motor_SetDir(pMot, pMot->MotorDir);

	pMot->RunStep = 0;

	Motor_Start(pMot);	
}
#endif /* USE_CT_STITCH_MODE */

void Motor_TestSetCurrent(Motor_Typedef *motor, bool Start)
{
	printUart(DBG_MSG_PC, "Motor_TestSetCurrent :: %d", Start);

	TMC2660_SetCurrent(motor->MotType, Start ? motor->StartCurrent : motor->StopCurrent);
}

void Motor_TestSetMinFreq(Motor_Typedef *motor, uint32_t min)
{
	printUart(DBG_MSG_PC, "Motor_TestSetMinFreq :: %d", min);

	motor->MinFreqCCR = Hz_To_CCR(min);
}

void Motor_TestAbsoluteMove(Motor_Typedef *motor, 
					double time, uint32_t acc, int32_t run, uint32_t dec)
{
	const double dTimCntClk = 1250000.0;

	if (!Motor_GetStatus(motor, STATUS_STOP))
	{
		UART_SendMessage(DBG_MSG_PC, "Motor Error...!!!");
		return;
	}

	//TMC2660_SetCurrent(motor->MotType, motor->StartCurrent);

	if (run > 0) {
		motor->MotorDir = DIR_LIMIT;
		motor->RunParam.RunStep = run;
	} else if (run < 0) {
		motor->MotorDir = DIR_ORG;
		motor->RunParam.RunStep = (-run);
	} else {
		Motor_Stop(motor);
		return;
	}

	motor->RunParam.AccStep = acc;
	motor->RunParam.DecStep = dec;

	//motor->MinFreqCCR = Hz_To_CCR(min);
	motor->RunParam.RunFreq = (360 / (360.0 / MOTOR_R_MICROSTEP__ / MOTOR_R_PULLEY_RATIO)) / time;
	motor->RunParam.RunCCR = dTimCntClk / (2 * motor->RunParam.RunFreq);
	motor->RunParam.AccCCR = Motor_CalcCCRValue(motor->MinFreqCCR, motor->RunParam.RunCCR, motor->RunParam.AccStep);
	motor->RunParam.DecCCR = Motor_CalcCCRValue(motor->MinFreqCCR, motor->RunParam.RunCCR, motor->RunParam.DecStep);	

	Motor_SetStatus(motor, STATUS_RUN);
	Motor_SetDir(motor, motor->MotorDir);
	motor->RunStep = 0;

	Motor_Start(motor);
}


/**
* @ Function Name : MotorR_MoveAbsoluteDistance
* @ Desc :
* @ Param :
* @ Return :
*/
#ifdef USE_CT_STITCH_MODE
void MotorR_MoveAbsoluteDistance(Motor_Typedef* motor, double time, uint32_t acc, int32_t run, uint32_t dec, char bReverse)
#else /* not USE_CT_STITCH_MODE */
void MotorR_MoveAbsoluteDistance(Motor_Typedef* motor, double time, uint32_t acc, int32_t run, uint32_t dec)
#endif /* USE_CT_STITCH_MODE */
{
	double dTimCntClk;

    if (!Motor_GetStatus(motor, STATUS_STOP))
    {
        UART_SendMessage(DBG_MSG_PC, "Motor R Absolute Distance Error...!!!"); // ���� ����
        return;
    }

	if( CtParam.TotalFrame==400)
	{
		dTimCntClk = 2489904.0;
		TMC2660_SetResolution(Motor_R.MotType, RES_X_16);
		motor->MinFreqCCR = Hz_To_CCR(2000);
	}
	else
	{
		dTimCntClk = 4990900.0;
		TMC2660_SetResolution(Motor_R.MotType, RES_X_16);
		motor->MinFreqCCR = Hz_To_CCR(2000);
	}


    if (run > 0)
	{
        motor->MotorDir = DIR_LIMIT;
        motor->RunParam.RunStep = run;
    }
	else if (run < 0) 
	{
        motor->MotorDir = DIR_ORG;
        motor->RunParam.RunStep = (-run);
    }
	else 
	{
        Motor_Stop(motor);
        return;
    }

	Motor_SetStatus(motor, STATUS_CT);
    Motor_SetDir(motor, motor->MotorDir);
#ifdef USE_CT_STITCH_MODE
	if (bReverse)
	{
		motor->RunParam.AccStep = dec;
		motor->RunParam.DecStep = acc;
		Motor_SetOnlyDir(motor, DIR_ORG);
		CtParam.nXrayOnStep = CtParam.nXrayOnStep - 5.0/(360.0 / MOTOR_R_MICROSTEP__ / MOTOR_R_PULLEY_RATIO);
		CtParam.nXrayOffStep = CtParam.nXrayOffStep - 5.0/(360.0 / MOTOR_R_MICROSTEP__ / MOTOR_R_PULLEY_RATIO);//+350;// - 350;
	}
	else
#endif /* USE_CT_STITCH_MODE */
	{
		motor->RunParam.AccStep = acc;
		motor->RunParam.DecStep = dec;
	}

    motor->RunParam.RunFreq = (360 / (360.0 / MOTOR_R_MICROSTEP__ / MOTOR_R_PULLEY_RATIO)) / time;
    motor->RunParam.RunCCR = dTimCntClk / (2 * motor->RunParam.RunFreq);
    motor->RunParam.AccCCR = Motor_CalcCCRValue(motor->MinFreqCCR, motor->RunParam.RunCCR, motor->RunParam.AccStep);
	motor->RunParam.DecCCR = Motor_CalcCCRValue(motor->MinFreqCCR, motor->RunParam.RunCCR, motor->RunParam.DecStep);

    CtParam.nRunStep = motor->RunParam.RunStep - motor->RunParam.DecStep;

	if(sysInfo.bShowLog==SET)
	{
		printUart(DBG_MSG_PC, "motorR->MinFreqCCR :: %f", motor->MinFreqCCR);
		printUart(DBG_MSG_PC, "motorR->RunParam.RunFreq :: %f", motor->RunParam.RunFreq);
		printUart(DBG_MSG_PC, "motorR->RunParam.RunCCR :: %f", motor->RunParam.RunCCR);
		printUart(DBG_MSG_PC, "motorR->RunParam.AccCCR :: %f", motor->RunParam.AccCCR);
		printUart(DBG_MSG_PC, "motorR->RunParam.DecCCR :: %f", motor->RunParam.DecCCR);
	}


    motor->RunStep = 0;

    Motor_Start(motor);
}

/**
* @ Function Name : Motor_MoveAbsoluteDistanceAccDec
* @ Desc :
* @ Param :
* @ Return :
*/
inline static void Motor_MoveAbsoluteDistanceAccDec(Motor_Typedef* motor, double freq, uint32_t acc, int32_t run, uint32_t dec)
{	
	if (!Motor_GetStatus(motor, STATUS_STOP)) 
	{

        UART_SendMessage(DBG_MSG_PC, "Motor Error...!!!");
		return;
	}

	if (run > 0) {
		motor->MotorDir = DIR_LIMIT;
		motor->RunParam.RunStep = run;
	} else if (run < 0) {
		motor->MotorDir = DIR_ORG;
		motor->RunParam.RunStep = (-run);
	} else {
		Motor_Stop(motor);
		return;
	}

	motor->RunParam.RunFreq = freq;
    motor->RunParam.AccStep = acc;
    motor->RunParam.DecStep = dec;

	Motor_InitRunParam(&motor->RunParam, motor->MinFreqCCR);

	Motor_SetStatus(motor, STATUS_RUN);
	Motor_SetDir(motor, motor->MotorDir);

	motor->RunStep = 0;

	Motor_Start(motor);	
}

/**
* @ Function Name : Motor_MoveAbsoluteDistanceAccDec
* @ Desc :
* @ Param :
* @ Return :
*/
static void MotorR_MoveAbsoluteDistanceAccDec_CT(Motor_Typedef* motor, double freq, uint32_t acc, int32_t run, uint32_t dec, double TimClk)
{	
	if (!Motor_GetStatus(motor, STATUS_STOP)) {
        UART_SendMessage(DBG_MSG_PC, "Motor_R CT Absolute Distance Error...!!!");
		return;
	}

	if (run > 0) 
	{
		motor->MotorDir = DIR_LIMIT;
		motor->RunParam.RunStep = run;
	} 
	else if (run < 0) 
	{
		motor->MotorDir = DIR_ORG;
		motor->RunParam.RunStep = (-run);
	} 
	else 
	{
		Motor_Stop(motor);
		return;
	}

	motor->RunParam.RunFreq = freq;
    motor->RunParam.AccStep = acc;
    motor->RunParam.DecStep = dec;

	//Motor_InitRunParam(&motor->RunParam, motor->MinFreqCCR);
	motor->RunParam.RunCCR = (double) TimClk / (2 * motor->RunParam.RunFreq);
    motor->RunParam.AccCCR = Motor_CalcCCRValue(motor->MinFreqCCR, motor->RunParam.RunCCR, motor->RunParam.AccStep);
    motor->RunParam.DecCCR = Motor_CalcCCRValue(motor->MinFreqCCR, motor->RunParam.RunCCR, motor->RunParam.DecStep);

	Motor_SetStatus(motor, STATUS_RUN);
	Motor_SetDir(motor, motor->MotorDir);

	motor->RunStep = 0;

	Motor_Start(motor);	
}


/**
* @ Function Name : Motor_MoveAbsoluteDistance
* @ Desc :
* @ Param :
* @ Return :
*/
static void Motor_MoveAbsoluteDistance(Motor_Typedef* motor, double freq, uint32_t acc, int32_t run, uint32_t dec)
{
	if (!Motor_GetStatus(motor, STATUS_STOP)) {
        UART_SendMessage(DBG_MSG_PC, "Motor Error...!!!");
		return;
	}

	if (run > 0) {
		motor->MotorDir = DIR_LIMIT;
		motor->RunParam.RunStep = run;
	} else if(run < 0) {
		motor->MotorDir = DIR_ORG;
		motor->RunParam.RunStep = (-run);
	} else {
		Motor_Stop(motor);
		return;
	}

	motor->RunParam.RunFreq = freq;

	Motor_InitRunParam(&motor->RunParam, motor->MinFreqCCR);

	Motor_SetStatus(motor, STATUS_RUN);
	Motor_SetDir(motor, motor->MotorDir);

	motor->RunStep = 0;

	Motor_Start(motor);
}

/**
* @ Function Name : Motor_MoveArchPosition
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_MoveArchPosition(Motor_Typedef* motor)
{
	if (!Motor_GetStatus(motor, STATUS_STOP)) 
	{
		if(motor==&Motor_R)
			UART_SendMessage(DBG_MSG_PC, "Motor_R Arch Position Error...!!!");
		else if(motor==&Motor_V)
        	UART_SendMessage(DBG_MSG_PC, "Motor_V Arch Position Error...!!!");
		return;
	}

	motor->ArchParam.TotalSize = motor->ArchParam.AccSize * 2 + motor->ArchParam.CaptureSize;
	motor->ArchParam.ArrayIndex = 0;
	motor->RunStep = 0;

	Motor_SetStatus(motor, STATUS_ARCH);	
	Motor_SetArchParam(motor);	

	Motor_Start(motor);
}


/**
* @ Function Name : Motor_MoveALL_ORG
* @ Desc :
* @ Param :
* @ Return :
*/
bool Motor_MoveALL_ORG(void)
{
    Lamp_Control(LAMP_RED, LAMP_DISABLE);
    Lamp_Control(LAMP_GREEN, LAMP_DISABLE);
    Lamp_Control(LAMP_BLUE, LAMP_DISABLE);

#ifdef USE_MOTOR_CHINREST_TS
    //Motor_MoveTempleSupport(RESET);
    //Motor_MoveTempleSupport(SET);
    TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StartCurrent);
    Motor_RunOrgCheck(&Motor_T);
	Temple_Status=RELEASED;
#endif /* USE_MOTOR_CHINREST_TS */

	Motor_SetCurrent(1);

	Motor_RunOrgCheck(&Motor_R);
#if defined(USE_MOTOR_PANO_AXIS)
    Motor_RunOrgCheck(&Motor_V);
#endif /* USE_MOTOR_PANO_AXIS */	
	if ( (sysInfo.model_id == MODEL_T2_CS) && (CephParam.bInitPosFlag == RESET) )
	{
		Motor_RunOrgCheck(&Motor_C);
		Motor_RunOrgCheck(&Motor_S);
	}
#ifdef USE_MOTOR_GANTRY_MS
	Motor_RunOrgCheck(&Motor_A);
#endif /* USE_MOTOR_GANTRY_MS */
#ifdef USE_MOTOR_CHINREST_VER
//jehun
	Motor_RunOrgCheck(&Motor_CNS);
	//Motor_RunOrgCheck_CNS(&Motor_CNS);
#endif /* USE_MOTOR_CHINREST_VER */
#ifdef USE_MOTOR_CHINREST_HOR
	Motor_RunOrgCheck(&Motor_CWE);
#endif /* USE_MOTOR_CHINREST_HOR */

    while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
#if defined(USE_MOTOR_PANO_AXIS)
    while(!Motor_GetStatus(&Motor_V, STATUS_STOP));
#endif /* USE_MOTOR_PANO_AXIS */	
    if (sysInfo.model_id == MODEL_T2_CS) {
        while(!Motor_GetStatus(&Motor_C, STATUS_STOP));
        while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
    }
#ifdef USE_MOTOR_GANTRY_MS
    while(!Motor_GetStatus(&Motor_A, STATUS_STOP));
#endif /* USE_MOTOR_GANTRY_MS */
#ifdef USE_MOTOR_CHINREST_VER
	while(!Motor_GetStatus(&Motor_CNS, STATUS_STOP));
#endif /* USE_MOTOR_CHINREST_VER */	
#ifdef USE_MOTOR_CHINREST_HOR
	while(!Motor_GetStatus(&Motor_CWE, STATUS_STOP));
#endif /* USE_MOTOR_CHINREST_HOR */	
	while(!Motor_GetStatus(&Motor_T, STATUS_STOP));

	Motor_SetCurrent(0);
	TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StopCurrent);

    return Motor_CheckRetryErrorStatus();
}



/**
* @ Function Name : Motor_MoveInitPosition
* @ Desc :
* @ Param :
* @ Return :
*/
bool Motor_MoveInitPosition(void)
{
#ifdef USE_MOTOR_CHINREST_VER
//190729 HWAN 7mm down CT INIT VER //51 //190729 PANORAMA_INIT_VER
#define nCT_VerStep (MOTOR_CNS_CT_INIT_POSITION/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP
#define nPano_VER_STEP (MOTOR_CNS_PANO_INIT_POSITION/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP
#endif /* USE_MOTOR_CHINREST_VER */
#ifdef USE_MOTOR_CHINREST_HOR
#define nCT_HorStep MOTOR_CWE_INIT_POSITION / (MOTOR_CWE_PULLEY_PITCH / MOTOR_CWE_MICROSTEP)
#define nPano_HOR_STEP MOTOR_CWE_INIT_POSITION / (MOTOR_CWE_PULLEY_PITCH / MOTOR_CWE_MICROSTEP)
#endif /* USE_MOTOR_CHINREST_HOR */

    Lamp_Control(LAMP_RED, LAMP_DISABLE);
    Lamp_Control(LAMP_GREEN, LAMP_DISABLE);
    Lamp_Control(LAMP_BLUE, LAMP_DISABLE);

#ifdef USE_MOTOR_CHINREST_TS
	if(CurCaptureMode== CAPTURE_PANO || CurCaptureMode == CAPTURE_CT)
	{
		if (Temple_Status != RELEASED) 
		{
			TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StartCurrent);
			Motor_RunOrgCheck(&Motor_T);
			while(!Motor_GetStatus(&Motor_T, STATUS_STOP));
			TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StopCurrent);
			Temple_Status = RELEASED;
		}
	}
#endif /* USE_MOTOR_CHINREST_TS */

	Motor_SetCurrent(1);

	while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
#if defined(USE_MOTOR_PANO_AXIS)
    while(!Motor_GetStatus(&Motor_V, STATUS_STOP));
#endif //USE_MOTOR_PANO_AXIS	
    if (sysInfo.model_id == MODEL_T2_CS) 
	{
        while(!Motor_GetStatus(&Motor_C, STATUS_STOP));
        while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
    }
#ifdef USE_MOTOR_GANTRY_MS
    while(!Motor_GetStatus(&Motor_A, STATUS_STOP));
#endif //USE_MOTOR_GANTRY_MS
#ifdef USE_MOTOR_CHINREST_VER
	while(!Motor_GetStatus(&Motor_CNS, STATUS_STOP));
#endif // USE_MOTOR_CHINREST_VER 
#ifdef USE_MOTOR_CHINREST_HOR
	while(!Motor_GetStatus(&Motor_CWE, STATUS_STOP));
#endif // USE_MOTOR_CHINREST_HOR 

/*
#ifdef USE_MOTOR_CHINREST_VER
	if(CtParam.bStitchDone== TRUE)
	{
		Motor_RunOrgCheck(&Motor_CNS);
		CtParam.bStitchDone = FALSE;
	}
#endif  //USE_MOTOR_CHINREST_VER 
*/		
	
    switch(CurCaptureMode) 
	{
    case CAPTURE_CANCEL:
        Motor_RunOrgCheck(&Motor_R);
#if defined(USE_MOTOR_PANO_AXIS)
        Motor_RunOrgCheck(&Motor_V);	
#endif /* USE_MOTOR_PANO_AXIS */
		if ( (sysInfo.model_id == MODEL_T2_CS) && (CephParam.bInitPosFlag == RESET) )
		{
            Motor_RunOrgCheck(&Motor_C);
            Motor_RunOrgCheck(&Motor_S);
        }
#ifdef USE_MOTOR_GANTRY_MS
       Motor_RunOrgCheck(&Motor_A);
#endif /* USE_MOTOR_GANTRY_MS */
#ifdef USE_MOTOR_CHINREST_VER
		Motor_RunOrgCheck(&Motor_CNS);
#endif /* USE_MOTOR_CHINREST_VER */
#ifdef USE_MOTOR_CHINREST_HOR
		Motor_RunOrgCheck(&Motor_CWE);
#endif /* USE_MOTOR_CHINREST_HOR */
        break;

    case CAPTURE_PANO:		
#ifdef USE_MOTOR_CHINREST_VER
		if(PanoParam.Init_Mode==PANO_NORMAL)
			Motor_RunOrgCheck(&Motor_CNS);
#endif /* USE_MOTOR_CHINREST_VER */

        Motor_RunOrgCheck(&Motor_R);
        Motor_RunOrgCheck(&Motor_V);
#ifdef USE_MOTOR_GANTRY_MS
		Motor_RunOrgCheck(&Motor_A);
#endif /* USE_MOTOR_GANTRY_MS */

#ifdef USE_MOTOR_CHINREST_HOR
		if(PanoParam.Init_Mode==PANO_NORMAL)
			Motor_RunOrgCheck(&Motor_CWE);
#endif /* USE_MOTOR_CHINREST_HOR */
        break;

    case CAPTURE_CT:
#ifdef USE_MOTOR_CHINREST_VER
		Motor_RunOrgCheck(&Motor_CNS);
#endif /* USE_MOTOR_CHINREST_VER */

		if(CtParam.bReverse_Status==SET)
		{
			Motor_Reverse_OrgCheck(&Motor_R);
		}
		else
			Motor_RunOrgCheck(&Motor_R);
        Motor_RunOrgCheck(&Motor_V);
#ifdef USE_MOTOR_GANTRY_MS
		Motor_RunOrgCheck(&Motor_A);
#endif /* USE_MOTOR_GANTRY_MS */
#ifdef USE_MOTOR_CHINREST_HOR
		Motor_RunOrgCheck(&Motor_CWE);
#endif /* USE_MOTOR_CHINREST_HOR */

        break;

    case CAPTURE_SCAN:
        if (sysInfo.model_id == MODEL_T2_CS)
		{
            Motor_RunOrgCheck(&Motor_R);
            Motor_RunOrgCheck(&Motor_V);
            Motor_RunOrgCheck(&Motor_C);
            Motor_RunOrgCheck(&Motor_S);
#ifdef USE_MOTOR_GANTRY_MS
            Motor_RunOrgCheck(&Motor_A);
#endif /* USE_MOTOR_GANTRY_MS */                    
			if(sysInfo.initDir_MotCC == INIT_DIR_REVERSE) CephParam.bInitPosFlag = RESET;
        }
        break;

    case CALIBRATION_MODE:
#ifdef USE_MOTOR_CHINREST_VER
		Motor_RunOrgCheck(&Motor_CNS);
#endif /* USE_MOTOR_CHINREST_VER */

        Motor_RunOrgCheck(&Motor_R);
        Motor_RunOrgCheck(&Motor_V);
        if (sysInfo.model_id == MODEL_T2_CS)
		{
            Motor_RunOrgCheck(&Motor_C);
            Motor_RunOrgCheck(&Motor_S);
			if(sysInfo.initDir_MotCC == INIT_DIR_REVERSE) CephParam.bInitPosFlag = RESET;
        }
#ifdef USE_MOTOR_GANTRY_MS
        Motor_RunOrgCheck(&Motor_A);
#endif /* USE_MOTOR_GANTRY_MS */

#ifdef USE_MOTOR_CHINREST_HOR
		Motor_RunOrgCheck(&Motor_CWE);
#endif /* USE_MOTOR_CHINREST_HOR */
        break;

    case GEOMETRY_ALIGN_MODE:
#ifdef USE_MOTOR_CHINREST_VER
		Motor_RunOrgCheck(&Motor_CNS);
#endif /* USE_MOTOR_CHINREST_VER */

        Motor_RunOrgCheck(&Motor_R);
        Motor_RunOrgCheck(&Motor_V);
        // 촬영 종료 ??org�??�동 추�?
        // Ceph 촬영 종료 ??current step??2배로 ?�어 ?�음
        // ceph 촬영 ??2�?colli??resolution??2배로 ?�정??
        if (sysInfo.model_id == MODEL_T2_CS)
		{
            Motor_RunOrgCheck(&Motor_C);
            Motor_RunOrgCheck(&Motor_S);
			if(sysInfo.initDir_MotCC == INIT_DIR_REVERSE) CephParam.bInitPosFlag = RESET;
        }
#ifdef USE_MOTOR_GANTRY_MS
        Motor_RunOrgCheck(&Motor_A);
#endif /* USE_MOTOR_GANTRY_MS */

#ifdef USE_MOTOR_CHINREST_HOR
		Motor_RunOrgCheck(&Motor_CWE);
#endif /* USE_MOTOR_CHINREST_HOR */

        break;
	case RESET_MODE:
		Motor_RunOrgCheck(&Motor_R);
        Motor_RunOrgCheck(&Motor_V);
#ifdef USE_MOTOR_GANTRY_MS
       Motor_RunOrgCheck(&Motor_A);
#endif /* USE_MOTOR_GANTRY_MS */
		if (sysInfo.model_id == MODEL_T2_CS && (sysInfo.initDir_MotCC == INIT_DIR_REVERSE) && (CephParam.bInitPosFlag == RESET)) 
		{
            Motor_RunOrgCheck(&Motor_C);
            Motor_RunOrgCheck(&Motor_S);
        }
        break;
    }


/*
#ifdef USE_MOTOR_CHINREST_VER
	if(CtParam.bStitchDone== SET)
	{
		while(!Motor_GetStatus(&Motor_CNS, STATUS_STOP));
		Motor_RunOrgCheck(&Motor_CNS);
		CtParam.bStitchDone = RESET;
	}
#endif // USE_MOTOR_CHINREST_VER 
*/		

	switch(CurCaptureMode) 
	{
		case CAPTURE_CANCEL:
		case CAPTURE_PANO:
#ifdef USE_MOTOR_CHINREST_VER
			if(PanoParam.Init_Mode==PANO_NORMAL)
			{
				while(!Motor_GetStatus(&Motor_CNS, STATUS_STOP));
				Motor_MoveAbsolutePosition(&Motor_CNS, 30000, 1000, nPano_VER_STEP+(PanoParam.nCNSaxisOffset/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP, 1000);
			}
#endif /* USE_MOTOR_CHINREST_VER */

            if ( (sysInfo.model_id == MODEL_T2_CS) && (sysInfo.initDir_MotCC == INIT_DIR_REVERSE) && (CephParam.bInitPosFlag == RESET) )
			{
				while(!Motor_GetStatus(&Motor_C, STATUS_STOP));
        		while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
                Motor_CephMoveAbsolutePosition(&Motor_C, 2500, 1000, CEPH_REVERSE_INIT_POS, 1000);
                Motor_CephMoveAbsolutePosition(&Motor_S, 2500, 1000, CEPH_REVERSE_INIT_POS, 1000);

                CephParam.bInitPosFlag = SET;
            }
			while(!Motor_GetStatus(&Motor_A, STATUS_STOP));
			Motor_MoveAbsolutePosition(&Motor_A, 12000, 1000, PANO_AUTO_ANGLE / (360.0 / MOTOR_A_MICROSTEP / MOTOR_A_PULLEY_RATIO), 1000);
#ifdef USE_MOTOR_CHINREST_HOR
			if(PanoParam.Init_Mode==PANO_NORMAL)
			{				
				while(!Motor_GetStatus(&Motor_CWE, STATUS_STOP));
				Motor_MoveAbsolutePosition(&Motor_CWE, 20000, 2000, nPano_HOR_STEP+ ((double)PanoParam.nCWEaxisOffset/10) / (MOTOR_CWE_PULLEY_PITCH / MOTOR_CWE_MICROSTEP), 2000);
			}
			#endif /* USE_MOTOR_CHINREST_HOR */
#if defined(USE_MOTOR_PANO_AXIS)
			while(!Motor_GetStatus(&Motor_V, STATUS_STOP));
			Motor_MoveAbsolutePosition(&Motor_V, 15000, 1000, PanoParam.nCanineOffset, 1000);
#endif /* USE_MOTOR_PANO_AXIS */

			break;

		case CAPTURE_CT:
#ifdef USE_MOTOR_CHINREST_VER
			while(!Motor_GetStatus(&Motor_CNS, STATUS_STOP));
			if(CtParam.bCWEaxisSET==RESET)
			{
				if(CtParam.Mode_15by9==CT_FULL_ARCH)
					Motor_MoveAbsolutePosition(&Motor_CNS, 30000, 3000, nCT_VerStep+((CtParam.nCNSaxisOffset+10)/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP, 3000);
				else if(CtParam.Mode_15by9==CT_SINUS)
					Motor_MoveAbsolutePosition(&Motor_CNS, 30000, 3000, nCT_VerStep+((CtParam.nCNSaxisOffset-20)/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP, 3000);
				else
					Motor_MoveAbsolutePosition(&Motor_CNS, 30000, 3000, nCT_VerStep+(CtParam.nCNSaxisOffset/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP, 3000);
			}
			else
			{
				Motor_MoveAbsolutePosition(&Motor_CNS, 30000, 3000, CtParam.nCNSaxisFovStep, 3000);
			}			
#endif /* USE_MOTOR_CHINREST_VER */	

#ifdef USE_MOTOR_CHINREST_HOR
			if(CtParam.bCWEaxisSET==RESET)
			{				
				Motor_MoveAbsolutePosition(&Motor_CWE, 20000, 2000, nCT_HorStep+((double)CtParam.nCWEaxisOffset/10) / (MOTOR_CWE_PULLEY_PITCH / MOTOR_CWE_MICROSTEP), 2000);
			}
			else
			{
				Motor_MoveAbsolutePosition(&Motor_CWE, 20000, 2000, CtParam.nCWEaxisFovStep, 2000);
			}
			while(!Motor_GetStatus(&Motor_CWE, STATUS_STOP));
#endif /* USE_MOTOR_CHINREST_HOR */			
#ifdef USE_MOTOR_GANTRY_MS
			while(!Motor_GetStatus(&Motor_A, STATUS_STOP));
			Motor_MoveAbsolutePosition(&Motor_A, 12000, 1000, CT_AUTO_ANGLE / (360.0 / MOTOR_A_MICROSTEP / MOTOR_A_PULLEY_RATIO), 1000);
#endif /* USE_MOTOR_GANTRY_MS */   

            if ( (sysInfo.model_id == MODEL_T2_CS) && (sysInfo.initDir_MotCC == INIT_DIR_REVERSE) && (CephParam.bInitPosFlag == RESET) )
			{
				while(!Motor_GetStatus(&Motor_C, STATUS_STOP));
        		while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
                Motor_CephMoveAbsolutePosition(&Motor_C, 2500, 1000, CEPH_REVERSE_INIT_POS, 1000);
                Motor_CephMoveAbsolutePosition(&Motor_S, 2500, 1000, CEPH_REVERSE_INIT_POS, 1000);

                CephParam.bInitPosFlag = SET;
            }
			while(!Motor_GetStatus(&Motor_V, STATUS_STOP));
			if(CtParam.bPaxisSET==RESET)
			{
				Motor_MoveAbsolutePosition(&Motor_V, 15000, 1000, 30 * (MOTOR_V_MICROSTEP / MOTOR_V_PULLEY_PITCH)+CtParam.nPaxisPatientOffset, 1000);				
			}
			else
			{
				Motor_MoveAbsolutePosition(&Motor_V, 15000, 1000, CtParam.nPaxisFovStep, 1000);	
			}
			while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
			Motor_MoveAbsolutePosition(&Motor_R, 1000, 500, CT_ALIGN_POS_RAXIS_OFFSET	+ CtParam.nRaxisPatientOffset/*+ CtParam.nAlignRaxisOffset*/, 500);
			break;

		case CAPTURE_SCAN:
            if (sysInfo.model_id == MODEL_T2_CS)
			{
				while(!Motor_GetStatus(&Motor_C, STATUS_STOP));
                Motor_CephMoveAbsolutePosition(&Motor_C, 3000, 1000, CEPH_SCAN_ALIGN_POSITION * (MOTOR_C_MICROSTEP / (MOTOR_C_PULLEY_DIAMETER * pi)), 1000);
				while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
                Motor_CephMoveAbsolutePosition(&Motor_S, 3000, 1000, CEPH_2ND_COLI_ALIGN_POSITION + CephParam.n2ndColOffset/* + CephParam.n2ndColStartOffset*/, 1000);
#ifdef USE_MOTOR_GANTRY_MS
				while(!Motor_GetStatus(&Motor_A, STATUS_STOP));
    			Motor_MoveAbsolutePosition(&Motor_A, 12000, 1000, CEPH_AUTO_ANGLE / (360.0 / MOTOR_A_MICROSTEP / MOTOR_A_PULLEY_RATIO), 1000);
#endif /* USE_MOTOR_GANTRY_MS */
				while(!Motor_GetStatus(&Motor_V, STATUS_STOP));
				Motor_MoveAbsolutePosition(&Motor_V, 15000, 2000, CEPH_VERTICAL_POSITION * (MOTOR_V_MICROSTEP / MOTOR_V_PULLEY_PITCH), 2000);
				while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
    			Motor_MoveAbsolutePosition(&Motor_R, 3000, 2000, (CEPH_ROTATE_ANGLE / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO)) + CephParam.nRAxisOffset , 2000);
            }
			break;

		case CALIBRATION_MODE:
#ifdef USE_MOTOR_CHINREST_VER
			while(!Motor_GetStatus(&Motor_CNS, STATUS_STOP));
			Motor_MoveAbsolutePosition(&Motor_CNS, 30000, 3000, nPano_VER_STEP+(PanoParam.nCNSaxisOffset/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP, 3000);
#endif /* USE_MOTOR_CHINREST_VER */

            if (sysInfo.model_id == MODEL_T2_CS)
			{
				while(!Motor_GetStatus(&Motor_C, STATUS_STOP));
        		while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
                Motor_CephMoveAbsolutePosition(&Motor_C, 3000, 1000, CEPH_SCAN_CALIBRATION_POSITION * (MOTOR_C_MICROSTEP / (MOTOR_C_PULLEY_DIAMETER * pi)), 1000);
                Motor_CephMoveAbsolutePosition(&Motor_S, 3000, 1000, 159.0 * (MOTOR_S_MICROSTEP / (MOTOR_S_PULLEY_DIAMETER * pi)), 1000);
				if (sysInfo.initDir_MotCC == INIT_DIR_REVERSE)
			    {
			        CephParam.bInitPosFlag = FALSE;
			    }
            }
#ifdef USE_MOTOR_GANTRY_MS
			while(!Motor_GetStatus(&Motor_A, STATUS_STOP));
			Motor_MoveAbsolutePosition(&Motor_A, 12000, 1000, CEPH_AUTO_ANGLE / (360.0 / MOTOR_A_MICROSTEP / MOTOR_A_PULLEY_RATIO), 1000);
#endif /* USE_MOTOR_GANTRY_MS */
			while(!Motor_GetStatus(&Motor_V, STATUS_STOP));
			Motor_MoveAbsolutePosition(&Motor_V, 15000, 2000, CEPH_VERTICAL_POSITION * (MOTOR_V_MICROSTEP / MOTOR_V_PULLEY_PITCH), 2000);
			while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
			Motor_MoveAbsolutePosition(&Motor_R, 3000, 2000, (CEPH_ROTATE_ANGLE / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO)) + CephParam.nRAxisOffset , 2000);
			break;
		case GEOMETRY_ALIGN_MODE:			
			while(!Motor_GetStatus(&Motor_CNS, STATUS_STOP));
			#ifdef USE_MOTOR_CHINREST_VER
#if defined(USE_MOTOR_CNS_SCREW_PITCH_6_35)			
			Motor_MoveAbsolutePosition(&Motor_CNS, 4000, 2000, nPano_VER_STEP+(PanoParam.nCNSaxisOffset/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP, 2000); 
#else /* not USE_MOTOR_CNS_SCREW_PITCH_6_35 */
			Motor_MoveAbsolutePosition(&Motor_CNS, 30000, 3000, nPano_VER_STEP+(PanoParam.nCNSaxisOffset/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP, 3000);
#endif /* USE_MOTOR_CNS_SCREW_PITCH_6_35 */
#endif /* USE_MOTOR_CHINREST_VER */
			while(!Motor_GetStatus(&Motor_V, STATUS_STOP));
			Motor_MoveAbsolutePosition(&Motor_V, 15000, 1000, GEOMETRY_ALIGN_VAXIS_START_ANGLE, 1000);
			
			while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
		    Motor_MoveAbsolutePosition(&Motor_R, 3000, 1500, GEOMETRY_ALIGN_RAXIS_START_ANGLE, 1500);
#ifdef USE_MOTOR_CHINREST_HOR
			while(!Motor_GetStatus(&Motor_CWE, STATUS_STOP));
			Motor_MoveAbsolutePosition(&Motor_CWE, 4000, 2000, nPano_HOR_STEP+ ((double)PanoParam.nCWEaxisOffset/10) / (MOTOR_CWE_PULLEY_PITCH / MOTOR_CWE_MICROSTEP), 2000);
#endif /* USE_MOTOR_CHINREST_HOR */
			break;
		case RESET_MODE:
#if defined(USE_MOTOR_PANO_AXIS)
			while(!Motor_GetStatus(&Motor_V, STATUS_STOP));
			Motor_MoveAbsolutePosition(&Motor_V, 15000, 1000, PanoParam.nCanineOffset, 1000);
#endif /* USE_MOTOR_PANO_AXIS */

            if ( (sysInfo.model_id == MODEL_T2_CS) && (sysInfo.initDir_MotCC == INIT_DIR_REVERSE) && (CephParam.bInitPosFlag == RESET) )
			{
				while(!Motor_GetStatus(&Motor_C, STATUS_STOP));
        		while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
                Motor_CephMoveAbsolutePosition(&Motor_C, 2500, 1000, CEPH_REVERSE_INIT_POS, 1000);
                Motor_CephMoveAbsolutePosition(&Motor_S, 2500, 1000, CEPH_REVERSE_INIT_POS, 1000);

                CephParam.bInitPosFlag = SET;
            }
			break;
		default:
			break;
	}


    while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
#if defined(USE_MOTOR_PANO_AXIS)
    while(!Motor_GetStatus(&Motor_V, STATUS_STOP));
#endif /* USE_MOTOR_PANO_AXIS */	
    if (sysInfo.model_id == MODEL_T2_CS) 
	{
        while(!Motor_GetStatus(&Motor_C, STATUS_STOP));
        while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
    }
#ifdef USE_MOTOR_CHINREST_TS
	//while(!Motor_GetStatus(&Motor_T, STATUS_STOP));	
	//TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StopCurrent);
#endif /* USE_MOTOR_CHINREST_TS */
#ifdef USE_MOTOR_GANTRY_MS
    while(!Motor_GetStatus(&Motor_A, STATUS_STOP));
#endif /* USE_MOTOR_GANTRY_MS */
#ifdef USE_MOTOR_CHINREST_VER
	while(!Motor_GetStatus(&Motor_CNS, STATUS_STOP));
#endif /* USE_MOTOR_CHINREST_VER */	
#ifdef USE_MOTOR_CHINREST_HOR
	while(!Motor_GetStatus(&Motor_CWE, STATUS_STOP));
#endif /* USE_MOTOR_CHINREST_HOR */
	Motor_SetCurrent(0);

    return Motor_CheckErrorStatus();
}


void Motor_Ceph_AlignPosition(void)
{
	Motor_SetCurrent(1);

	Motor_MoveAbsolutePosition(&Motor_R, 3000, 2000, (CEPH_ROTATE_ANGLE / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO)) + CephParam.nRAxisOffset , 2000);
	Motor_MoveAbsolutePosition(&Motor_V, 15000, 2000, CEPH_VERTICAL_POSITION * (MOTOR_V_MICROSTEP / MOTOR_V_PULLEY_PITCH), 2000);
    Motor_CephMoveAbsolutePosition(&Motor_C, 3000, 1000, CEPH_SCAN_ALIGN_POSITION * (MOTOR_C_MICROSTEP / (MOTOR_C_PULLEY_DIAMETER * pi)), 1000);
    Motor_CephMoveAbsolutePosition(&Motor_S, 3000, 1000, CEPH_2ND_COLI_ALIGN_POSITION + CephParam.n2ndColOffset/* + CephParam.n2ndColStartOffset*/, 1000);
#ifdef USE_MOTOR_GANTRY_MS
	Motor_MoveAbsolutePosition(&Motor_A, 12000, 1000, CEPH_AUTO_ANGLE / (360.0 / MOTOR_A_MICROSTEP / MOTOR_A_PULLEY_RATIO), 1000);
#endif /* USE_MOTOR_GANTRY_MS */  
	while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
	while(!Motor_GetStatus(&Motor_V, STATUS_STOP));
    if (sysInfo.model_id == MODEL_T2_CS)
    {
    	while(!Motor_GetStatus(&Motor_C, STATUS_STOP));
    	while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
    }
	while(!Motor_GetStatus(&Motor_A, STATUS_STOP));

	Motor_SetCurrent(0);
}



/**
* @ Function Name : Motor_MoveAlignPosition
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_MoveAlignPosition(void)
{
#ifdef USE_MOTOR_CHINREST_VER
//190729 HWAN 7mm down CT INIT VER //51 //190729 PANORAMA_INIT_VER
#define nCT_VerStep (MOTOR_CNS_CT_INIT_POSITION/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP
#define nPano_VER_STEP (MOTOR_CNS_PANO_INIT_POSITION/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP
#endif /* USE_MOTOR_CHINREST_VER */
#ifdef USE_MOTOR_CHINREST_HOR
#define nCT_HorStep MOTOR_CWE_INIT_POSITION / (MOTOR_CWE_PULLEY_PITCH / MOTOR_CWE_MICROSTEP)
#define nPano_HOR_STEP MOTOR_CWE_INIT_POSITION / (MOTOR_CWE_PULLEY_PITCH / MOTOR_CWE_MICROSTEP)
#endif /* USE_MOTOR_CHINREST_HOR */

	Motor_SetCurrent(1);

	switch(CurCaptureMode) {
		case CAPTURE_CANCEL:
		case CAPTURE_PANO:
			//Motor_MoveAbsolutePosition(&Motor_R, 3000, 1000, PANO_ROTATE_ALIGN_ANGLE / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO)+ PanoParam.nRAxisOffset, 1000);
			//200312 HWAN TEST
			Motor_MoveAbsolutePosition(&Motor_R, 3000, 1000, PANO_ROTATE_ALIGN_ANGLE / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO), 1000);
#if defined(USE_MOTOR_PANO_AXIS)
            //Motor_MoveAbsolutePosition(&Motor_V, 6400, 1000, PanoParam.nCanineOffset+PanoParam.nVAxisOffset, 1000);
            //200312 HWAN TEST
			Motor_MoveAbsolutePosition(&Motor_V, 15000, 1000, PanoParam.nCanineOffset, 1000);
#endif /* USE_MOTOR_PANO_AXIS */

            if ( (sysInfo.model_id == MODEL_T2_CS) && (sysInfo.initDir_MotCC == INIT_DIR_REVERSE) && (CephParam.bInitPosFlag == RESET) )
			{
                Motor_CephMoveAbsolutePosition(&Motor_C, 2500, 1000, CEPH_REVERSE_INIT_POS, 1000);
                Motor_CephMoveAbsolutePosition(&Motor_S, 2500, 1000, CEPH_REVERSE_INIT_POS, 1000);

                CephParam.bInitPosFlag = SET;
            }
#ifdef USE_MOTOR_CHINREST_VER
			//			Motor_MoveAbsolutePosition(&Motor_CNS, 14000, 3000, nPano_VER_STEP+(PanoParam.nCNSaxisOffset/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP, 3000);
			//Motor_MoveAbsolutePosition(&Motor_CNS, 40000, 3000, nPano_VER_STEP+(PanoParam.nCNSaxisOffset/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP, 3000);
#if defined(USE_MOTOR_CNS_SCREW_PITCH_6_35)			
			Motor_MoveAbsolutePosition(&Motor_CNS, 4000, 2000, nPano_VER_STEP+(PanoParam.nCNSaxisOffset/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP, 2000); 
#else /* not USE_MOTOR_CNS_SCREW_PITCH_6_35 */
			Motor_MoveAbsolutePosition(&Motor_CNS, 20000, 3000, nPano_VER_STEP+(PanoParam.nCNSaxisOffset/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP, 3000);
#endif /* USE_MOTOR_CNS_SCREW_PITCH_6_35 */
#endif /* USE_MOTOR_CHINREST_VER */
#ifdef USE_MOTOR_CHINREST_HOR

			//Motor_MoveAbsolutePosition(&Motor_CWE, 4000, 2000, ALIGN_HOR_STEP, 2000);
			Motor_MoveAbsolutePosition(&Motor_CWE, 20000, 2000, nPano_HOR_STEP+ ((double)PanoParam.nCWEaxisOffset/10) / (MOTOR_CWE_PULLEY_PITCH / MOTOR_CWE_MICROSTEP), 2000);
#endif /* USE_MOTOR_CHINREST_HOR */
			break;

		case CAPTURE_CT:
			Motor_MoveAbsolutePosition(&Motor_V, 15000, 1000, CtParam.nPaxisPatientOffset, 1000);
			Motor_MoveAbsolutePosition(&Motor_R, 3000, 1000, CT_ALIGN_POS_RAXIS_OFFSET  + CtParam.nRaxisPatientOffset/*+ CtParam.nAlignRaxisOffset*/, 1000);
#ifdef USE_MOTOR_GANTRY_MS
			Motor_MoveAbsolutePosition(&Motor_A, 12000, 1000, CT_AUTO_ANGLE / (360.0 / MOTOR_A_MICROSTEP / MOTOR_A_PULLEY_RATIO), 1000);
#endif /* USE_MOTOR_GANTRY_MS */   

            if ( (sysInfo.model_id == MODEL_T2_CS) && (sysInfo.initDir_MotCC == INIT_DIR_REVERSE) && (CephParam.bInitPosFlag == RESET) )
			{
                Motor_CephMoveAbsolutePosition(&Motor_C, 2500, 1000, CEPH_REVERSE_INIT_POS, 1000);
                Motor_CephMoveAbsolutePosition(&Motor_S, 2500, 1000, CEPH_REVERSE_INIT_POS, 1000);

                CephParam.bInitPosFlag = SET;
            }
#ifdef USE_MOTOR_CHINREST_VER
#if defined(USE_MOTOR_CNS_SCREW_PITCH_6_35)
			Motor_MoveAbsolutePosition(&Motor_CNS, 4000, 2000, nCT_VerStep+(CtParam.nCNSaxisOffset/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP, 2000); 
#else /* not USE_MOTOR_CNS_SCREW_PITCH_6_35 */
	if(CtParam.Mode_15by9==CT_FULL_ARCH)
				//jehun
				Motor_MoveAbsolutePosition(&Motor_CNS, 30000, 3000, nCT_VerStep+((CtParam.nCNSaxisOffset+10)/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP, 3000);
//			Motor_MoveAbsolutePosition(&Motor_CNS, 14000, 3000, nCT_VerStep+((CtParam.nCNSaxisOffset+10)/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP, 3000); 
			else if(CtParam.Mode_15by9==CT_SINUS)
				//jehun
				Motor_MoveAbsolutePosition(&Motor_CNS, 30000, 3000, nCT_VerStep+((CtParam.nCNSaxisOffset-20)/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP, 3000);
//			Motor_MoveAbsolutePosition(&Motor_CNS, 14000, 3000, nCT_VerStep+((CtParam.nCNSaxisOffset-20)/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP, 3000);
			else
				//jehun
				Motor_MoveAbsolutePosition(&Motor_CNS, 30000, 3000, nCT_VerStep+(CtParam.nCNSaxisOffset/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP, 3000);
//			Motor_MoveAbsolutePosition(&Motor_CNS, 14000, 3000, nCT_VerStep+(CtParam.nCNSaxisOffset/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP, 3000);
#endif /* USE_MOTOR_CNS_SCREW_PITCH_6_35 */
#endif /* USE_MOTOR_CHINREST_VER */				
#ifdef USE_MOTOR_CHINREST_HOR
			Motor_MoveAbsolutePosition(&Motor_CWE, 20000, 2000, nCT_HorStep+CtParam.nCWEaxisOffset / (MOTOR_CWE_PULLEY_PITCH / MOTOR_CWE_MICROSTEP), 2000);
#endif /* USE_MOTOR_CHINREST_HOR */
			break;

		case CAPTURE_SCAN:
            if (sysInfo.model_id == MODEL_T2_CS)
			{
    			Motor_MoveAbsolutePosition(&Motor_R, 3000, 2000, (CEPH_ROTATE_ANGLE / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO)) + CephParam.nRAxisOffset , 2000);
    			Motor_MoveAbsolutePosition(&Motor_V, 15000, 2000, CEPH_VERTICAL_POSITION * (MOTOR_V_MICROSTEP / MOTOR_V_PULLEY_PITCH), 2000);
                Motor_CephMoveAbsolutePosition(&Motor_C, 3000, 1000, CEPH_SCAN_ALIGN_POSITION * (MOTOR_C_MICROSTEP / (MOTOR_C_PULLEY_DIAMETER * pi)), 1000);
                Motor_CephMoveAbsolutePosition(&Motor_S, 3000, 1000, CEPH_2ND_COLI_ALIGN_POSITION + CephParam.n2ndColOffset/* + CephParam.n2ndColStartOffset*/, 1000);
#ifdef USE_MOTOR_GANTRY_MS
    			Motor_MoveAbsolutePosition(&Motor_A, 12000, 1000, CEPH_AUTO_ANGLE / (360.0 / MOTOR_A_MICROSTEP / MOTOR_A_PULLEY_RATIO), 1000);
#endif /* USE_MOTOR_GANTRY_MS */
            }
			break;

		case CALIBRATION_MODE:
			Motor_MoveAbsolutePosition(&Motor_R, 3000, 2000, (CEPH_ROTATE_ANGLE / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO)) + CephParam.nRAxisOffset , 2000);
			Motor_MoveAbsolutePosition(&Motor_V, 15000, 2000, CEPH_VERTICAL_POSITION * (MOTOR_V_MICROSTEP / MOTOR_V_PULLEY_PITCH), 2000);
            if (sysInfo.model_id == MODEL_T2_CS)
			{
                Motor_CephMoveAbsolutePosition(&Motor_C, 3000, 1000, CEPH_SCAN_CALIBRATION_POSITION * (MOTOR_C_MICROSTEP / (MOTOR_C_PULLEY_DIAMETER * pi)), 1000);
                Motor_CephMoveAbsolutePosition(&Motor_S, 3000, 1000, 159.0 * (MOTOR_S_MICROSTEP / (MOTOR_S_PULLEY_DIAMETER * pi)), 1000);
            }
#ifdef USE_MOTOR_GANTRY_MS
			Motor_MoveAbsolutePosition(&Motor_A, 12000, 1000, CEPH_AUTO_ANGLE / (360.0 / MOTOR_A_MICROSTEP / MOTOR_A_PULLEY_RATIO), 1000);
#endif /* USE_MOTOR_GANTRY_MS */
			break;
		case RESET_MODE:
			Motor_MoveAbsolutePosition(&Motor_R, 3000, 1000, PANO_ROTATE_ALIGN_ANGLE / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO), 1000);
#if defined(USE_MOTOR_PANO_AXIS)
			Motor_MoveAbsolutePosition(&Motor_V, 15000, 1000, PanoParam.nCanineOffset, 1000);
#endif /* USE_MOTOR_PANO_AXIS */

            if ( (sysInfo.model_id == MODEL_T2_CS) && (sysInfo.initDir_MotCC == INIT_DIR_REVERSE) && (CephParam.bInitPosFlag == RESET) )
			{
                Motor_CephMoveAbsolutePosition(&Motor_C, 2500, 1000, CEPH_REVERSE_INIT_POS, 1000);
                Motor_CephMoveAbsolutePosition(&Motor_S, 2500, 1000, CEPH_REVERSE_INIT_POS, 1000);

                CephParam.bInitPosFlag = SET;
            }
			break;
		default:
			break;
	}

	while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
#if defined(USE_MOTOR_PANO_AXIS)
	while(!Motor_GetStatus(&Motor_V, STATUS_STOP));
#endif /* USE_MOTOR_PANO_AXIS */
    if (sysInfo.model_id == MODEL_T2_CS) {
    	while(!Motor_GetStatus(&Motor_C, STATUS_STOP));
    	while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
    }
#ifdef USE_MOTOR_GANTRY_MS
	while(!Motor_GetStatus(&Motor_A, STATUS_STOP));
#endif /* USE_MOTOR_GANTRY_MS */
#ifdef USE_MOTOR_CHINREST_VER
	while(!Motor_GetStatus(&Motor_CNS, STATUS_STOP));
#endif /* USE_MOTOR_CHINREST_VER */
#ifdef USE_MOTOR_CHINREST_HOR
	while(!Motor_GetStatus(&Motor_CWE, STATUS_STOP));
#endif /* USE_MOTOR_CHINREST_HOR */	

	Motor_SetCurrent(0);
}	

/**
* @ Function Name : Motor_MoveStartPosition
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_MoveStartPosition(void)
{
	Motor_SetCurrent(1);

    switch(CurCaptureMode)
	{
    case CAPTURE_PANO:
        //Motor_MoveAbsolutePosition(&Motor_V, 4000, 2000, Motor_V.ArchParam.OffsetStep + PanoParam.ModeOffset, 2000);
		//200312 HWAN 
		Motor_MoveAbsolutePosition(&Motor_V, 15000, 2000, Motor_V.ArchParam.OffsetStep + PanoParam.ModeOffset + PanoParam.nVAxisOffset, 2000);
        Motor_MoveAbsolutePosition(&Motor_R, 2000, 1000, (PANO_ROTATE_START_ANGLE / (360.0 / MOTOR_R_MICROSTEP /MOTOR_R_PULLEY_RATIO)) - Motor_R.ArchParam.OffsetStep + PanoParam.nRAxisOffset, 1000);
#ifdef USE_MOTOR_GANTRY_MS
		Motor_MoveAbsolutePosition(&Motor_A, 12000, 1000, PANO_AUTO_ANGLE / (360.0 / MOTOR_A_MICROSTEP / MOTOR_A_PULLEY_RATIO), 1000);
#endif /* USE_MOTOR_GANTRY_MS */
        break;

    case CAPTURE_CT:
		if(CtParam.bReverse_Status==RESET)
		{
			if(CtParam.TotalFrame==400)
				Motor_MoveAbsolutePosition(&Motor_R, 2000, 1000, CtParam.nFastRaxisOffset, 1000);
			else
				Motor_MoveAbsolutePosition(&Motor_R, 2000, 1000, CtParam.nRaxisOffset, 1000);
		}
		else
		{
			if(CtParam.TotalFrame==400)
				Motor_MoveAbsolutePosition(&Motor_R, 2000, 1000, CtParam.nFastRaxisOffset+CtParam.nReverseTotalStep-(360 / (360.0 / MOTOR_R_MICROSTEP /MOTOR_R_PULLEY_RATIO)), 1000);
			else
				Motor_MoveAbsolutePosition(&Motor_R, 2000, 1000, CtParam.nRaxisOffset+CtParam.nReverseTotalStep-(360 / (360.0 / MOTOR_R_MICROSTEP /MOTOR_R_PULLEY_RATIO)), 1000);
		}
		Motor_MoveAbsolutePosition(&Motor_V, 15000, 2000, Motor_V.CurrentStep-CtParam.nPaxisPatientOffset+CtParam.nPaxisOffset, 2000); 
        break;

    case CAPTURE_SCAN:
        break;
    }

    while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
    while(!Motor_GetStatus(&Motor_V, STATUS_STOP));
    if (sysInfo.model_id == MODEL_T2_CS)
	{
        while(!Motor_GetStatus(&Motor_C, STATUS_STOP));
        while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
    }
#ifdef USE_MOTOR_GANTRY_MS
    while(!Motor_GetStatus(&Motor_A, STATUS_STOP));
#endif /* USE_MOTOR_GANTRY_MS */

	Motor_SetCurrent(0);
}


/**
* @ Function Name : Motor_MoveEndPosition
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_MoveEndPosition(void)
{
	if (CurCaptureMode != CAPTURE_SCAN) 
	{
		if (CurCaptureMode == CAPTURE_PANO) 
		{
			Motor_MoveAbsolutePosition(&Motor_R, 3000, 1500, (210.0 /(360.0 / MOTOR_R_MICROSTEP /MOTOR_R_PULLEY_RATIO)), 2000); 			
		} 
		else if (CurCaptureMode == CAPTURE_CT) 
		{
			if(CtParam.TotalFrame==400)
				MotorR_MoveAbsolutePosition_CT(&Motor_R, 6000, 3000, (390.0 /(360.0 / MOTOR_R_MICROSTEP__ /MOTOR_R_PULLEY_RATIO))-CtParam.nFastRaxisOffset, 3000); 	
			else
				MotorR_MoveAbsolutePosition_CT(&Motor_R, 6000, 3000, (390.0 /(360.0 / MOTOR_R_MICROSTEP__ /MOTOR_R_PULLEY_RATIO))-CtParam.nRaxisOffset, 3000); 	
		}

		while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
	}
}


#if defined(USE_MOTOR_CHINREST_HOR) ||defined(USE_MOTOR_CHINREST_VER)
/**
* @ Function Name : Motor_ControlChinrest
* @ Desc :
* @ Param :
* @ Return :
*/
bool Motor_ControlChinrest(bool bVertical, double nDistance)
{
	// 200203 HWAN Vertical 68 ~ -16		// CT U17~D67		//PANO U10~D74
	// 200203 HWAN Horizontal 64 ~ -15		// CT L41~R38		//PANO L41~R38
	if (bVertical==RESET && (nDistance < -15 || nDistance > 64))
	{
		printUart(DBG_MSG_PC, "Unsupported range!");
		return RESET;
	}
	else if(bVertical==SET && (nDistance < -16 || nDistance > 68))
	{
		printUart(DBG_MSG_PC, "Unsupported range!");
		return RESET;
	}
	else
	{
		double nRunStep;
		//uint32_t nAccDecStep;

		if (bVertical == RESET)
		{
			nRunStep = nDistance /(MOTOR_CWE_PULLEY_PITCH / MOTOR_CWE_MICROSTEP);

#ifdef USE_MOTOR_CHINREST_HOR
			TMC2660_SetCurrent(Motor_CWE.MotType, Motor_CWE.StartCurrent);

			Motor_MoveAbsolutePosition(&Motor_CWE, 20000, 2000, (int32_t)nRunStep, 2000); 
			while(!Motor_GetStatus(&Motor_CWE, STATUS_STOP));

			TMC2660_SetCurrent(Motor_CWE.MotType, Motor_CWE.StopCurrent);
//190918 HWAN TEST
			if(sysInfo.bShowLog==SET)
			{
				printUart(DBG_MSG_PC, "Horizontal : CurrentStep(%d)", Motor_CWE.CurrentStep);
			}
#endif /* USE_MOTOR_CHINREST_HOR */
		}
		else
		{
			nRunStep = (nDistance / MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP;

#ifdef USE_MOTOR_CHINREST_VER
			TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StartCurrent);

			Motor_MoveAbsolutePosition(&Motor_CNS, 30000, 2000, (int32_t)nRunStep, 2000);
			while(!Motor_GetStatus(&Motor_CNS, STATUS_STOP));

			TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StopCurrent);
			//190918 HWAN TEST
			if(sysInfo.bShowLog==SET)
			{
				printUart(DBG_MSG_PC, "Vertical : CurrentStep(%d)", Motor_CNS.CurrentStep);
			}

#endif /* USE_MOTOR_CHINREST_VER */
		}

		return SET;
	}
}

#endif /* USE_MOTOR_CHINREST_HOR) ||USE_MOTOR_CHINREST_VER */

#ifdef USE_MOTOR_CHINREST_TS
/**
* @ Function Name : Motor_TempleSupportOrgCheck
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_TempleSupportOrgCheck(void)
{
    Motor_RunOrgCheck(&Motor_T);
	while(!Motor_GetStatus(&Motor_T, STATUS_STOP));
}

/**
* @ Function Name : Motor_MoveTempleSupport
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_MoveTempleSupport(bool status)
{
	TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StartCurrent);

    if (status)
	{
		if(Temple_Status == PUSHED)
		{
			TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StopCurrent);
			return;
		}
        Motor_MoveAbsolutePosition(&Motor_T, 3000, 2000, 8000+PanoParam.nTAxisOffset, 2000);
		//Motor_MoveAbsolutePosition(&Motor_T, 2000, 2000, 8000, 2000);  
		Temple_Status = PUSHED;
    }
	else 
	{
		if(Temple_Status == RELEASED)
		{
			TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StopCurrent);
			return;
		}

        Motor_RunOrgCheck(&Motor_T);
		Temple_Status=RELEASED;
	}
    while(!Motor_GetStatus(&Motor_T, STATUS_STOP));

	TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StopCurrent);
}

void Motor_MoveTempleSupport_Child(bool status)
{
	TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StartCurrent);

    if (status)
	{
		if(Temple_Status == PUSHED)
		{
			TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StopCurrent);
			return;
		}
        Motor_MoveAbsolutePosition(&Motor_T, 3000, 2000, 8000+PanoParam.nTAxisChildOffset, 2000);
		//Motor_MoveAbsolutePosition(&Motor_T, 2000, 2000, 8000, 2000);  
		Temple_Status = PUSHED;
    }
	else 
	{
		if(Temple_Status == RELEASED)
		{
			TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StopCurrent);
			return;
		}

        Motor_RunOrgCheck(&Motor_T);
		Temple_Status=RELEASED;
	}
    while(!Motor_GetStatus(&Motor_T, STATUS_STOP));

	TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StopCurrent);
}


#endif /* USE_MOTOR_CHINREST_TS */

/**
* @ Function Name : Motor_MoveGeoAlignPosition
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_MoveGeoAlignPosition(void)
{
#ifdef USE_MOTOR_CHINREST_VER
	//190729 HWAN 7mm down CT INIT VER //51 //190729 PANORAMA_INIT_VER
#define nPano_VER_STEP (MOTOR_CNS_PANO_INIT_POSITION/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP
#endif /* USE_MOTOR_CHINREST_VER */
#ifdef USE_MOTOR_CHINREST_HOR
#define nPano_HOR_STEP MOTOR_CWE_INIT_POSITION / (MOTOR_CWE_PULLEY_PITCH / MOTOR_CWE_MICROSTEP)
#endif /* USE_MOTOR_CHINREST_HOR */

	Motor_SetCurrent(1);

    Motor_MoveAbsolutePosition(&Motor_R, 3000, 1500, GEOMETRY_ALIGN_RAXIS_START_ANGLE, 1500);
    Motor_MoveAbsolutePosition(&Motor_V, 15000, 500, GEOMETRY_ALIGN_VAXIS_START_ANGLE, 500);
#ifdef USE_MOTOR_CHINREST_VER
#if defined(USE_MOTOR_CNS_SCREW_PITCH_6_35)			
	Motor_MoveAbsolutePosition(&Motor_CNS, 4000, 2000, nPano_VER_STEP+(PanoParam.nCNSaxisOffset/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP, 2000); 
#else /* not USE_MOTOR_CNS_SCREW_PITCH_6_35 */
	Motor_MoveAbsolutePosition(&Motor_CNS, 30000, 3000, nPano_VER_STEP+(PanoParam.nCNSaxisOffset/MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP, 3000);
#endif /* USE_MOTOR_CNS_SCREW_PITCH_6_35 */
#endif /* USE_MOTOR_CHINREST_VER */
#ifdef USE_MOTOR_CHINREST_HOR
	Motor_MoveAbsolutePosition(&Motor_CWE, 4000, 2000, nPano_HOR_STEP+ ((double)PanoParam.nCWEaxisOffset/10) / (MOTOR_CWE_PULLEY_PITCH / MOTOR_CWE_MICROSTEP), 2000);
#endif /* USE_MOTOR_CHINREST_HOR */
	while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
	while(!Motor_GetStatus(&Motor_V, STATUS_STOP));
	while(!Motor_GetStatus(&Motor_CNS, STATUS_STOP));
	while(!Motor_GetStatus(&Motor_CWE, STATUS_STOP));

	Motor_SetCurrent(0);
}

/**
* @ Function Name : Motor_CheckOrgHistory
* @ Desc :
* @ Param :
* @ Return :
*/
static bool Motor_CheckOrgHistory(Motor_Typedef *pMotor)
{
    if ( (pMotor->OrgSen.uStatus.bit.HistoryOn == 0) ||
        (pMotor->OrgSen.uStatus.bit.HistoryOff == 0) ) {
        //pMotor->OrgSen.uStatus.bit.HistoryOn = 0;
        //pMotor->OrgSen.uStatus.bit.HistoryOff = 0;

        return RESET;
    }

    return SET;
}

/**
* @ Function Name : Motor_CheckErrorStatus
* @ Desc :
* @ Param :
* @ Return :
*/
void Motor_Lamp_ErrorStatus(void)
{    
	if (!Motor_CheckOrgHistory(&Motor_R)) 
		LampErrorTimer_Enable(3,10);
	else
		LampErrorTimer_Enable(0,10);

	while(!LampTimer_GetStatus());
	LampErrorTimer_Enable(5,10);
	while(!LampTimer_GetStatus());
#if defined(USE_MOTOR_PANO_AXIS)
    if (!Motor_CheckOrgHistory(&Motor_V)) 
		LampErrorTimer_Enable(3,10);
	else
		LampErrorTimer_Enable(0,10);

	while(!LampTimer_GetStatus());
	LampErrorTimer_Enable(5,10);
	while(!LampTimer_GetStatus());
#endif /* USE_MOTOR_PANO_AXIS */	

    if (sysInfo.model_id == MODEL_T2_CS) 
	{
        if (!Motor_CheckOrgHistory(&Motor_C)) 
			LampErrorTimer_Enable(3,10);
		else
			LampErrorTimer_Enable(0,10);

		while(!LampTimer_GetStatus());
		LampErrorTimer_Enable(5,10);
		while(!LampTimer_GetStatus());
		
        if (!Motor_CheckOrgHistory(&Motor_S)) 
			LampErrorTimer_Enable(3,10);
		else
			LampErrorTimer_Enable(0,10);

		while(!LampTimer_GetStatus());
		LampErrorTimer_Enable(5,10);
		while(!LampTimer_GetStatus());
		 
    }

#ifdef USE_MOTOR_CHINREST_TS
    if (!Motor_CheckOrgHistory(&Motor_T)) 
		LampErrorTimer_Enable(3,10);
	else
		LampErrorTimer_Enable(0,10);

	while(!LampTimer_GetStatus());
	LampErrorTimer_Enable(5,10);
	while(!LampTimer_GetStatus());
#endif /* USE_MOTOR_CHINREST_TS */

#ifdef USE_MOTOR_GANTRY_MS
    if (!Motor_CheckOrgHistory(&Motor_A)) 
		LampErrorTimer_Enable(3,10);
	else
		LampErrorTimer_Enable(0,10);

	while(!LampTimer_GetStatus()); 
	LampErrorTimer_Enable(5,10);
	while(!LampTimer_GetStatus());
#endif /* USE_MOTOR_GANTRY_MS */

#ifdef USE_MOTOR_CHINREST_VER
    if (!Motor_CheckOrgHistory(&Motor_CNS)) 
		LampErrorTimer_Enable(3,10);
	else
		LampErrorTimer_Enable(0,10);

	while(!LampTimer_GetStatus()); 
	LampErrorTimer_Enable(5,10);
	while(!LampTimer_GetStatus());
#endif /* USE_MOTOR_CHINREST_VER */

#ifdef USE_MOTOR_CHINREST_HOR
    if (!Motor_CheckOrgHistory(&Motor_CWE)) 
		LampErrorTimer_Enable(3,10);
	else
		LampErrorTimer_Enable(0,10);

	while(!LampTimer_GetStatus());
	LampErrorTimer_Enable(5,2);
	while(!LampTimer_GetStatus());
#endif /* USE_MOTOR_CHINREST_HOR */

}


/**
* @ Function Name : Motor_CheckErrorStatus
* @ Desc :
* @ Param :
* @ Return :
*/
bool Motor_CheckErrorStatus(void)
{    
    bool ret = SET;

	if(sysInfo.bShowLog==SET)
	{
    	printUart(DBG_MSG_PC, "====Motor_CheckErrorStatus start====");
	}

	if (!Motor_CheckOrgHistory(&Motor_R)) 
	{
		UART_SendMessage(DBG_MSG_PC, ERR_CODE_MOTOR_R_AXIS);
        ret = RESET;
	}

#if defined(USE_MOTOR_PANO_AXIS)
    if (!Motor_CheckOrgHistory(&Motor_V))
	{
		UART_SendMessage(DBG_MSG_PC, ERR_CODE_MOTOR_P_AXIS);
        ret = RESET;
    }
#endif /* USE_MOTOR_PANO_AXIS */	

    if (sysInfo.model_id == MODEL_T2_CS)
	{
        if (!Motor_CheckOrgHistory(&Motor_C))
		{
    		UART_SendMessage(DBG_MSG_PC, ERR_CODE_MOTOR_C_AXIS);

            if ( (CurCaptureMode != CAPTURE_PANO) && (CurCaptureMode != CAPTURE_CT) )
                ret = RESET;
        }
        if (!Motor_CheckOrgHistory(&Motor_S))
		{
    		UART_SendMessage(DBG_MSG_PC, ERR_CODE_MOTOR_S_AXIS);

            if ( (CurCaptureMode != CAPTURE_PANO) && (CurCaptureMode != CAPTURE_CT) )
                ret = RESET;
    	}
    }

#ifdef USE_MOTOR_CHINREST_TS
    if (!Motor_CheckOrgHistory(&Motor_T))
	{
		UART_SendMessage(DBG_MSG_PC, ERR_CODE_MOTOR_TS_AXIS);
        ret = RESET;
	}
#endif /* USE_MOTOR_CHINREST_TS */

#ifdef USE_MOTOR_GANTRY_MS
    if (!Motor_CheckOrgHistory(&Motor_A))
	{
		UART_SendMessage(DBG_MSG_PC, ERR_CODE_MOTOR_MS_AXIS);
        ret = RESET;
	} 
#endif /* USE_MOTOR_GANTRY_MS */

#ifdef USE_MOTOR_CHINREST_VER
    if (!Motor_CheckOrgHistory(&Motor_CNS))
	{
		UART_SendMessage(DBG_MSG_PC, ERR_CODE_MOTOR_CNS_AXIS);
		if ( (CurCaptureMode != CAPTURE_SCAN))
                ret = RESET;
	} 
#endif /* USE_MOTOR_CHINREST_VER */

#ifdef USE_MOTOR_CHINREST_HOR
    if (!Motor_CheckOrgHistory(&Motor_CWE))
	{
		UART_SendMessage(DBG_MSG_PC, ERR_CODE_MOTOR_CWE_AXIS);
		if ( (CurCaptureMode != CAPTURE_SCAN) )
        	ret = RESET;
	} 
#endif /* USE_MOTOR_CHINREST_HOR */



	printUart(DBG_MSG_PC, "====Motor_CheckErrorStatus finish==== %d",ret);

    return ret;
}

/**
* @ Function Name : Motor_CheckErrorStatus
* @ Desc :
* @ Param :
* @ Return :
*/
bool Motor_CheckRetryErrorStatus(void)
{    
    bool ret = SET;

	if(sysInfo.bShowLog==SET)
	{
    	printUart(DBG_MSG_PC, "====Motor_CheckErrorStatus start====");
	}

	if (!Motor_CheckOrgHistory(&Motor_R)) 
	{		
		printUart(DBG_MSG_PC, "====Motor_R Retry ORG====");
		Motor_Stop(&Motor_R);
		Motor_R.nInitRunStep = 0;		
		TMC2660_SPI_Specific_Init(&Motor_R);
		TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
		Motor_RunOrgCheck(&Motor_R);
		while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
		TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StopCurrent);
		if (!Motor_CheckOrgHistory(&Motor_R)) 
		{
			UART_SendMessage(DBG_MSG_PC, ERR_CODE_MOTOR_R_AXIS);			
        	ret = RESET;
		}
	}

#if defined(USE_MOTOR_PANO_AXIS)
    if (!Motor_CheckOrgHistory(&Motor_V))
	{
		printUart(DBG_MSG_PC, "====Motor_V Retry ORG====");
		Motor_Stop(&Motor_V);
		Motor_V.nInitRunStep = 0;	
		TMC2660_SPI_Specific_Init(&Motor_V);
		TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
		Motor_RunOrgCheck(&Motor_V);
		while(!Motor_GetStatus(&Motor_V, STATUS_STOP));
		TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StopCurrent);
		if (!Motor_CheckOrgHistory(&Motor_V)) 
		{
			UART_SendMessage(DBG_MSG_PC, ERR_CODE_MOTOR_P_AXIS);
        	ret = RESET;
		}
    }
#endif /* USE_MOTOR_PANO_AXIS */	

    if (sysInfo.model_id == MODEL_T2_CS)
	{
        if (!Motor_CheckOrgHistory(&Motor_C))
		{
			printUart(DBG_MSG_PC, "====Motor_C Retry ORG====");
			Motor_Stop(&Motor_C);
			Motor_C.nInitRunStep = 0;			
			TMC2660_SPI_Specific_Init(&Motor_C);
			TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StartCurrent);
			Motor_RunOrgCheck(&Motor_C);
			while(!Motor_GetStatus(&Motor_C, STATUS_STOP));
			TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StopCurrent);
			if (!Motor_CheckOrgHistory(&Motor_C)) 
			{
				UART_SendMessage(DBG_MSG_PC, ERR_CODE_MOTOR_C_AXIS);
	        	ret = RESET;
			} 
        }
		
        if (!Motor_CheckOrgHistory(&Motor_S))
		{
			printUart(DBG_MSG_PC, "====Motor_S Retry ORG====");
			Motor_Stop(&Motor_S);
			Motor_S.nInitRunStep = 0;			
			TMC2660_SPI_Specific_Init(&Motor_S);
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
			Motor_RunOrgCheck(&Motor_S);
			while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
			if (!Motor_CheckOrgHistory(&Motor_S)) 
			{
				UART_SendMessage(DBG_MSG_PC, ERR_CODE_MOTOR_S_AXIS);
	        	ret = RESET;
			}  
    	}
    }

#ifdef USE_MOTOR_CHINREST_TS
    if (!Motor_CheckOrgHistory(&Motor_T))
	{
		printUart(DBG_MSG_PC, "====Motor_T Retry ORG====");
		Motor_Stop(&Motor_T);
		Motor_T.nInitRunStep = 0;		
		TMC2660_SPI_Specific_Init(&Motor_T);
		TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StartCurrent);
		Motor_RunOrgCheck(&Motor_T);
		while(!Motor_GetStatus(&Motor_T, STATUS_STOP));
		TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StopCurrent);
		if (!Motor_CheckOrgHistory(&Motor_T)) 
		{
			UART_SendMessage(DBG_MSG_PC, ERR_CODE_MOTOR_TS_AXIS);
        	ret = RESET;
		}  
	}
#endif /* USE_MOTOR_CHINREST_TS */

#ifdef USE_MOTOR_GANTRY_MS
    if (!Motor_CheckOrgHistory(&Motor_A))
	{
		printUart(DBG_MSG_PC, "====Motor_A Retry ORG====");
		Motor_Stop(&Motor_A);
		Motor_A.nInitRunStep = 0;		
		TMC2660_SPI_Specific_Init(&Motor_A);
		TMC2660_SetCurrent(Motor_A.MotType, Motor_A.StartCurrent);
		Motor_RunOrgCheck(&Motor_A);
		while(!Motor_GetStatus(&Motor_A, STATUS_STOP));
		TMC2660_SetCurrent(Motor_A.MotType, Motor_A.StopCurrent);
		if (!Motor_CheckOrgHistory(&Motor_A)) 
		{
			UART_SendMessage(DBG_MSG_PC, ERR_CODE_MOTOR_MS_AXIS);
        	ret = RESET;
		} 		 
	} 
#endif /* USE_MOTOR_GANTRY_MS */

#ifdef USE_MOTOR_CHINREST_VER
    if (!Motor_CheckOrgHistory(&Motor_CNS))
	{
		printUart(DBG_MSG_PC, "====Motor_CNS Retry ORG====");
		Motor_Stop(&Motor_CNS);
		Motor_CNS.nInitRunStep = 0;		
		TMC2660_SPI_Specific_Init(&Motor_CNS);
		TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StartCurrent);
		Motor_RunOrgCheck(&Motor_CNS);
		while(!Motor_GetStatus(&Motor_CNS, STATUS_STOP));
		TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StopCurrent);
		if (!Motor_CheckOrgHistory(&Motor_CNS)) 
		{
			UART_SendMessage(DBG_MSG_PC, ERR_CODE_MOTOR_CNS_AXIS);
        	ret = RESET;
		}  
	} 
#endif /* USE_MOTOR_CHINREST_VER */

#ifdef USE_MOTOR_CHINREST_HOR
    if (!Motor_CheckOrgHistory(&Motor_CWE))
	{
		printUart(DBG_MSG_PC, "====Motor_CWE Retry ORG====");
		Motor_Stop(&Motor_CWE);
		Motor_CWE.nInitRunStep = 0;		
		TMC2660_SPI_Specific_Init(&Motor_CWE);
		TMC2660_SetCurrent(Motor_CWE.MotType, Motor_CWE.StartCurrent);
		Motor_RunOrgCheck(&Motor_CWE);
		while(!Motor_GetStatus(&Motor_CWE, STATUS_STOP));
		TMC2660_SetCurrent(Motor_CWE.MotType, Motor_CWE.StopCurrent);
		if (!Motor_CheckOrgHistory(&Motor_CWE)) 
		{
			UART_SendMessage(DBG_MSG_PC, ERR_CODE_MOTOR_CWE_AXIS);
        	ret = RESET;
		}  
	} 
#endif /* USE_MOTOR_CHINREST_HOR */



	printUart(DBG_MSG_PC, "====Motor_CheckErrorStatus finish==== %d",ret);

    return ret;
}



/**
* @ Function Name : Motor_Status_Check
* @ Desc :
* @ Param :
* @ Return :
*/
bool Motor_Init_Check(void)
{
    bool ret;

#ifdef BUILD_TYPE_DEBUG
    ElapseTimerOnOff(SET);
    gInitTimeOut = vSetCurrentMilliSec();
#endif /* BUILD_TYPE_DEBUG */

    printUart(DBG_MSG_PC, "Motor Org check is started");

	ret = Motor_InitBootPosition();

#ifdef BUILD_TYPE_DEBUG
    printUart(DBG_MSG_PC, "initTime : %d", nElapsedMilliSec(gInitTimeOut));
    ElapseTimerOnOff(RESET);
#endif /* BUILD_TYPE_DEBUG */

    printUart(DBG_MSG_PC, "Motor Org check is completed");

    return ret;
}

/**
* @ Function Name : Motor_InitBootPosition
* @ Desc :
* @ Param :
* @ Return :
*/
static bool Motor_InitBootPosition(void)
{
	if(sysInfo.bShowLog==SET)
	{
    	printUart(DBG_MSG_PC, "Motor_InitBootPosition Start : %d", nElapsedMilliSec(gInitTimeOut));
	}

    if (Motor_MoveALL_ORG() != SET)
	{
        printUart(DBG_MSG_PC, "Motor_MoveInitPosition failed");
        return RESET;
    }

	TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);

    
	while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
	//201103 HWAN ADD 1.0.1c
	if ( (sysInfo.model_id == MODEL_T2_CS) && (sysInfo.initDir_MotCC == INIT_DIR_REVERSE))
	{
		while(!Motor_GetStatus(&Motor_C, STATUS_STOP));
		while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
	}
	
	Motor_MoveAbsolutePosition(&Motor_R, 2000, 1000, BOOT_ROT_CHECK_STEP, 1000);
	//201103 HWAN ADD 1.0.1c
	if ( (sysInfo.model_id == MODEL_T2_CS) && (sysInfo.initDir_MotCC == INIT_DIR_REVERSE) && (CephParam.bInitPosFlag == RESET) )
	{
		TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StartCurrent);
		TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
        Motor_MoveAbsolutePosition(&Motor_C, 5000, 1000, CEPH_REVERSE_INIT_POS, 1000);
        Motor_MoveAbsolutePosition(&Motor_S, 5000, 1000, CEPH_REVERSE_INIT_POS, 1000);

        CephParam.bInitPosFlag = SET;
    }
    while(!Motor_GetStatus(&Motor_R, STATUS_STOP));

    if (GPIO_ReadInputDataBit(Motor_R.Org2.Gpio, Motor_R.Org2.GpioPin) == SET)
	{
        Motor_RunOrgCheck(&Motor_R);
        while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
    }
	else
	{
		Motor_MoveAbsolutePosition(&Motor_R, 2000, 1000, ROTATOR_ALIGN_STEP, 1000);
    	while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
	}
	//201103 HWAN ADD 1.0.1c
	if ( (sysInfo.model_id == MODEL_T2_CS) && (sysInfo.initDir_MotCC == INIT_DIR_REVERSE))
	{
		while(!Motor_GetStatus(&Motor_C, STATUS_STOP));
		while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
		TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StopCurrent);
		TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
	}

	TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StopCurrent);

	if(sysInfo.bShowLog==SET)
	{
    	printUart(DBG_MSG_PC, "Motor_InitBootPosition End : %d", nElapsedMilliSec(gInitTimeOut));
	}

    return SET;
}


/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/

