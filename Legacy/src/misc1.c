/*
*******************************************************************************
* column.c :
*******************************************************************************
* Copyright (C) 2014-2016 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date : 
* @ Brief :
*
* @ Revision History :
*       1) initial creation. ------------------------------------- 2014-11-16
*       2) V1.1.0  ------------------------------------------- 2015-03-20
*       3) V1.2.0  ------------------------------------------- 2015-09-18
*       4) V1.3.0  ------------------------------------------- 2015-10-28
*******************************************************************************
*/

/* Include files ------------------------------------------------------ */
#include "stm32f2xx_syscfg.h"
#include "extern.h"
#include "misc1.h"
#include "system.h"
#include "serial.h"
#include "can.h"
#include "motor.h"


/* Private typedef --------------------------------------------------- */
/* Private define ---------------------------------------------------- */
#define COLUMN_GPIO 						GPIOC
#define COLUMN_GPIO_CLOCK 					RCC_AHB1Periph_GPIOC
#define COLUMN_GPIO_MODE 					GPIO_Mode_OUT
#define COLUMN_GPIO_ENABLE 					Bit_RESET
#define COLUMN_GPIO_DISABLE 				Bit_SET
#define COLUMN_UP_GPIO_PIN 					GPIO_Pin_0
#define COLUMN_DOWN_GPIO_PIN 				GPIO_Pin_1

#define	UP_IN_GPIO_CLOCK					RCC_AHB1Periph_GPIOE
#define	UP_IN_GPIO_PORT						GPIOE
#define	UP_IN_GPIO_PIN						GPIO_Pin_0
#define	UP_IN_GPIO_MODE						GPIO_Mode_IN
#define	UP_IN_GPIO_PUPD               		GPIO_PuPd_NOPULL
#define	UP_IN_GPIO_SPEED              		GPIO_Speed_50MHz
#define	UP_IN_GPIO_PORT_SOURCE      		EXTI_PortSourceGPIOE
#define	UP_IN_GPIO_PIN_SOURCE     		   	EXTI_PinSource0
#define	UP_IN_GPIO_EXTI_LINE           		EXTI_Line0
#define	UP_IN_GPIO_EXTI_MODE          		EXTI_Mode_Interrupt
#define	UP_IN_GPIO_EXTI_TRIGGER    	    	EXTI_Trigger_Rising_Falling

#define	DOWN_IN_GPIO_CLOCK					RCC_AHB1Periph_GPIOE
#define	DOWN_IN_GPIO_PORT					GPIOE
#define	DOWN_IN_GPIO_PIN					GPIO_Pin_11
#define	DOWN_IN_GPIO_MODE					GPIO_Mode_IN
#define	DOWN_IN_GPIO_PUPD 					GPIO_PuPd_NOPULL
#define	DOWN_IN_GPIO_SPEED 					GPIO_Speed_50MHz
#define	DOWN_IN_GPIO_PORT_SOURCE		 	EXTI_PortSourceGPIOE
#define	DOWN_IN_GPIO_PIN_SOURCE 			EXTI_PinSource11
#define	DOWN_IN_GPIO_EXTI_LINE 				EXTI_Line11
#define	DOWN_IN_GPIO_EXTI_MODE 				EXTI_Mode_Interrupt
#define	DOWN_IN_GPIO_EXTI_TRIGGER 			EXTI_Trigger_Rising_Falling

#ifdef USE_CT_STITCH_MODE
#define HS_DISTANCE_PER_REV					0.2
#define HS_ACC_COUNT 						10
#define HS_STITCH_COUNT 					312 // x0.2 = 62.4mm

//20190618_myshin : SCB Rev 2.0
#if 1
#define ENCODER_HS1_TIM							TIM4
#define ENCODER_HS1_TIM_CLOCK					RCC_APB1Periph_TIM4
#define ENCODER_HS1_TIM_POLARITY				TIM_ICPolarity_Rising
#define	ENCODER_HS1_TIM_INTERRUPT				TIM4_IRQn
#define ENCODER_HS1_TIM_ISR						TIM4_IRQHandler
#define	ENCODER_HS1_TIM_SOURCE					TIM_IT_CC1
#define	ENCODER_HS1_TIM_CHANNEL					TIM_Channel_1
#define ENCODER_HS1_TIM_SELECTION				TIM_ICSelection_DirectTI
#define ENCODER_HS1_TIM_PRESCALER				TIM_ICPSC_DIV1

#define	ENCODER_HS1_GetCapture					TIM_GetCapture1
#define	ENCODER_HS1_SetCompare					TIM_SetCompare1

#define	ENCODER_HS1_GPIO_PORT					GPIOB
#define	ENCODER_HS1_GPIO_CLOCK					RCC_AHB1Periph_GPIOB
#define	ENCODER_HS1_GPIO_PIN					GPIO_Pin_6
#define	ENCODER_HS1_GPIO_PinSource				GPIO_PinSource6
#define	ENCODER_HS1_GPIO_SPEED					GPIO_Speed_50MHz
#define	ENCODER_HS1_GPIO_AF						GPIO_AF_TIM4
#else
#define ENCODER_HS1_TIM							TIM2
#define ENCODER_HS1_TIM_CLOCK					RCC_APB1Periph_TIM2
#define ENCODER_HS1_TIM_POLARITY				TIM_ICPolarity_Rising
#define	ENCODER_HS1_TIM_INTERRUPT				TIM2_IRQn
#define ENCODER_HS1_TIM_ISR						TIM2_IRQHandler
#define	ENCODER_HS1_TIM_SOURCE					TIM_IT_CC2
#define	ENCODER_HS1_TIM_CHANNEL					TIM_Channel_2
#define ENCODER_HS1_TIM_SELECTION				TIM_ICSelection_DirectTI
#define ENCODER_HS1_TIM_PRESCALER				TIM_ICPSC_DIV1

#define	ENCODER_HS1_GetCapture					TIM_GetCapture2
#define	ENCODER_HS1_SetCompare					TIM_SetCompare2

#define	ENCODER_HS1_GPIO_PORT					GPIOA
#define	ENCODER_HS1_GPIO_CLOCK					RCC_AHB1Periph_GPIOA
#define	ENCODER_HS1_GPIO_PIN					GPIO_Pin_1
#define	ENCODER_HS1_GPIO_PinSource				GPIO_PinSource1
#define	ENCODER_HS1_GPIO_SPEED					GPIO_Speed_50MHz
#define	ENCODER_HS1_GPIO_AF						GPIO_AF_TIM2
#endif


#if 0
#define ENCODER_HS2_TIM							TIM2
#define ENCODER_HS2_TIM_CLOCK					RCC_APB1Periph_TIM2

#define ENCODER_HS2_TIM_POLARITY				TIM_ICPolarity_Rising
#define ENCODER_HS2_TIM_INTERRUPT				TIM2_IRQn
//#define ENCODER_HS2_TIM_ISR					TIM2_IRQHandler
#define ENCODER_HS2_TIM_SOURCE					TIM_IT_CC3
#define ENCODER_HS2_TIM_CHANNEL					TIM_Channel_3
#define ENCODER_HS2_TIM_SELECTION				TIM_ICSelection_DirectTI
#define ENCODER_HS2_TIM_PRESCALER				TIM_ICPSC_DIV1


#define	ENCODER_HS2_GPIO_PORT					GPIOA
#define	ENCODER_HS2_GPIO_CLOCK					RCC_AHB1Periph_GPIOA
#define	ENCODER_HS2_GPIO_PIN					GPIO_Pin_2
#define	ENCODER_HS2_GPIO_PinSource				GPIO_PinSource2
//#define	ENCODER_HS2_GPIO_MODE				GPIO_Mode_IN_FLOATING
#define	ENCODER_HS2_GPIO_SPEED					GPIO_Speed_50MHz
#define	ENCODER_HS2_GPIO_AF						GPIO_AF_TIM2
#endif
#endif /* USE_CT_STITCH_MODE */


/* EXPOSURE SWITCH */
#define	EXPOSURE_LED_GPIO						GPIOC
#define	EXPOSURE_LED_GPIO_CLOCK					RCC_AHB1Periph_GPIOC
#define	EXPOSURE_LED_GPIO_PIN					GPIO_Pin_9
#define	EXPOSURE_LED_GPIO_MODE					GPIO_Mode_OUT

#define	EXPOSURE_SWITCH_GPIO					GPIOD
#define	EXPOSURE_SWITCH_GPIO_PIN				GPIO_Pin_7
#define	EXPOSURE_SWITCH_GPIO_MODE				GPIO_Mode_IN

#define SWITCH_RELEASE  		Bit_SET
#define SWITCH_PUSH     		Bit_RESET


/* INDICATOR */
#define INDICATOR_GPIO_PORT 					GPIOE
#define INDICATOR_GPIO_PORT_CLOCK 				RCC_AHB1Periph_GPIOE
#define INDICATOR_GPIO_PORT_PIN 				GPIO_Pin_15
#define INDICATOR_GPIO_PORT_MODE		 		GPIO_Mode_OUT

/* LED CHECK */
#define LED_DIP_GPIO_PORT 						GPIOB
#define LED_DIP_GPIO_PORT_CLOCK 				RCC_AHB1Periph_GPIOB
#define LED_DIP_GPIO_PORT_MODE 					GPIO_Mode_OUT
#define LED_DIP_GPIO_PORT_PIN 					GPIO_Pin_5


/* LED LAMP */
#define LAMP_GPIO_PORT 							GPIOE
#define LAMP_GPIO_PORT_CLOCK 					RCC_AHB1Periph_GPIOE
#define LAMP_GPIO_PORT_MODE 					GPIO_Mode_OUT
#define LAMP_R_GPIO_PORT_PIN 					GPIO_Pin_2
#define LAMP_G_GPIO_PORT_PIN 					GPIO_Pin_3
#define LAMP_B_GPIO_PORT_PIN 					GPIO_Pin_4

#define	LAMP_MOOD_GPIO_PORT 					GPIOC
#define	LAMP_MOOD_GPIO_PORT_CLOCK 				RCC_AHB1Periph_GPIOC
#define	LAMP_MOOD_GPIO_PORT_MODE 				GPIO_Mode_OUT
#define	LAMP_MOOD_1_GPIO_PORT_PIN 				GPIO_Pin_13
#define	LAMP_MOOD_2_GPIO_PORT_PIN 				GPIO_Pin_14


/* LASER BEAM */
#define LASER_GPIO_PORT 						GPIOC
#define LASER_GPIO_PORT_CLOCK			  		RCC_AHB1Periph_GPIOC
#define LASER_GPIO_PORT_MODE		 			GPIO_Mode_OUT
#define LASER_V_GPIO_PORT_PIN 					GPIO_Pin_7

#define LASER_F_GPIO_PORT 						GPIOE
#define LASER_F_GPIO_PORT_CLOCK			  		RCC_AHB1Periph_GPIOE
#define LASER_F_GPIO_PORT_MODE		 			GPIO_Mode_OUT
#define LASER_F_GPIO_PORT_PIN 					GPIO_Pin_14

#define CMD_LASER_POSITION_PANO					0x1450
#define CMD_LASER_POSITION_CEPH					0x1460

#define BEAM_ON 	Bit_SET
#define BEAM_OFF	Bit_RESET

// 200408  HWAN TEST STITCH SENSOR
#define	STICH_UNDER_SENSOR_GPIO					GPIOE
#define	STICH_UNDER_SENSOR_GPIO_PIN				GPIO_Pin_7
#define STICH_UNDER_SENSOR_GPIO_MODE			GPIO_Mode_IN
#define STICH_UNDER_SENSOR_GPIO_CLOCK			RCC_AHB1Periph_GPIOE

#define	STICH_UP_SWITCH_GPIO					GPIOE
#define	STICH_UP_SWITCH_GPIO_PIN				GPIO_Pin_9
#define STICH_UP_SWITCH_GPIO_MODE				GPIO_Mode_IN
#define STICH_UP_SWITCH_GPIO_CLOCK				RCC_AHB1Periph_GPIOE



#define MOTOR_CNS_INIT_ARR				100
#define MOTOR_CNS_ACC_COUNT				2


/* Private macro ---------------------------------------------------- */
/* Private variables -------------------------------------------------- */
bool g_bColumnPressed;

#ifdef USE_CT_STITCH_MODE
uint32_t nHallSensorCount = 0,limit_count=314;
char bColumnStop = FALSE, bTsMotorInterlock = FALSE,bColumnUp = FALSE;
__IO uint16_t IC1Value = 0;
uint16_t Before_IC1Value = 0,IC1_VALUE_Index=0,IC1_SAME_VALUE=1;
const float nMotorPulse = ((HS_DISTANCE_PER_REV /MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP);
#endif /* USE_CT_STITCH_MODE */


/* Private function prototypes ------------------------------------------*/
static void Column_Delay(unsigned int mills);
#ifdef USE_CT_STITCH_MODE
static void Column_EncConfig(void);
#endif /* USE_CT_STITCH_MODE */
static void Exposure_DelayMs(uint32_t millis);
static void LaserSetVertical(BitAction bit);
static void LaserSetFoot(BitAction bit);


/* Private functions -------------------------------------------------- */

/**
* @ Function Name : Column_Config
* @ Desc : 
* @ Param : 
* @ Return :
*/
void Column_Config(void)
{	
    GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

	/* Enable GPIOx clock */
	
	RCC_AHB1PeriphClockCmd(COLUMN_GPIO_CLOCK, ENABLE);
	
	/* GPIOx Configuration */
	GPIO_InitStructure.GPIO_Pin = COLUMN_UP_GPIO_PIN |COLUMN_DOWN_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = COLUMN_GPIO_MODE;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(COLUMN_GPIO, &GPIO_InitStructure);    

    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(UP_IN_GPIO_CLOCK, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_InitStructure.GPIO_Mode = UP_IN_GPIO_MODE;
    GPIO_InitStructure.GPIO_PuPd = UP_IN_GPIO_PUPD;
    GPIO_InitStructure.GPIO_Speed = UP_IN_GPIO_SPEED;
    GPIO_InitStructure.GPIO_Pin = UP_IN_GPIO_PIN;
    GPIO_Init(UP_IN_GPIO_PORT, &GPIO_InitStructure);
   
    /* Connect Button EXTI Line to Button GPIO Pin */
    SYSCFG_EXTILineConfig(UP_IN_GPIO_PORT_SOURCE, UP_IN_GPIO_PIN_SOURCE);

    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = UP_IN_GPIO_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = UP_IN_GPIO_EXTI_MODE;
    EXTI_InitStructure.EXTI_Trigger = UP_IN_GPIO_EXTI_TRIGGER;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 

    GPIO_InitStructure.GPIO_Mode = DOWN_IN_GPIO_MODE;
    GPIO_InitStructure.GPIO_PuPd = DOWN_IN_GPIO_PUPD;
    GPIO_InitStructure.GPIO_Speed = DOWN_IN_GPIO_SPEED;
    GPIO_InitStructure.GPIO_Pin = DOWN_IN_GPIO_PIN;
    GPIO_Init(DOWN_IN_GPIO_PORT, &GPIO_InitStructure);

    /* Connect Button EXTI Line to Button GPIO Pin */
    SYSCFG_EXTILineConfig(DOWN_IN_GPIO_PORT_SOURCE, DOWN_IN_GPIO_PIN_SOURCE);

    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = DOWN_IN_GPIO_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = DOWN_IN_GPIO_EXTI_MODE;
    EXTI_InitStructure.EXTI_Trigger = DOWN_IN_GPIO_EXTI_TRIGGER;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);     

    Column_Control(COLUMN_STOP);
	g_bColumnPressed = FALSE;

#ifdef USE_CT_STITCH_MODE
	Column_EncConfig();
#endif /* USE_CT_STITCH_MODE */
}

/**
* @ Function Name : Column_Control
* @ Desc : 
* @ Param : 
* @ Return :
*/
void Column_Control(uint8_t status)
{
    switch(status) 
	{
    case COLUMN_STOP:
        GPIO_WriteBit(COLUMN_GPIO, COLUMN_UP_GPIO_PIN, COLUMN_GPIO_DISABLE);
        GPIO_WriteBit(COLUMN_GPIO, COLUMN_DOWN_GPIO_PIN, COLUMN_GPIO_DISABLE);
		break;

    case COLUMN_UP:
        GPIO_WriteBit(COLUMN_GPIO, COLUMN_UP_GPIO_PIN, COLUMN_GPIO_ENABLE);
        GPIO_WriteBit(COLUMN_GPIO, COLUMN_DOWN_GPIO_PIN, COLUMN_GPIO_DISABLE);
        break;

    case COLUMN_DOWN:
        GPIO_WriteBit(COLUMN_GPIO, COLUMN_UP_GPIO_PIN, COLUMN_GPIO_DISABLE);
        GPIO_WriteBit(COLUMN_GPIO, COLUMN_DOWN_GPIO_PIN, COLUMN_GPIO_ENABLE);
        break;

    default:
        break;
    }
}

/**
* @ Function Name : DelayCount
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void Column_Delay(unsigned int mills)
{
    /* While loop takes 4 cycles */
    /* For 1 us delay, we need to divide with 4M */
    uint32_t nMultiplier = SystemCoreClock / 4000000;

    /* Multiply millis with multipler */
    /* Substract 10 */
    mills = 1000 * mills * nMultiplier - 10;
    /* 4 cycles for one loop */
    while(mills--);
}

#ifdef USE_CT_STITCH_MODE

void Column_Enable(void)
{
/*
	//TEST1
	RCC_AHB1PeriphClockCmd(COLUMN_GPIO_CLOCK, ENABLE);
*/
	/*

	//TEST2
	RCC_AHB1PeriphClockCmd(UP_IN_GPIO_CLOCK, ENABLE);
	//TEST2-1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
*/

	//TEST3
	EXTI_InitTypeDef EXTI_InitStructure;
	
	
	// Configure Button EXTI line 
    EXTI_InitStructure.EXTI_Line = UP_IN_GPIO_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = UP_IN_GPIO_EXTI_MODE;
    EXTI_InitStructure.EXTI_Trigger = UP_IN_GPIO_EXTI_TRIGGER;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

	// Configure Button EXTI line 
    EXTI_InitStructure.EXTI_Line = DOWN_IN_GPIO_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = DOWN_IN_GPIO_EXTI_MODE;
    EXTI_InitStructure.EXTI_Trigger = DOWN_IN_GPIO_EXTI_TRIGGER;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
/**/

/*
	//TEST4
	NVIC_InitTypeDef NVIC_InitStructure;
	// Enable and set Button EXTI Interrupt to the lowest priority 
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 
	
	// Enable and set Button EXTI Interrupt to the lowest priority 
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 
*/	
		
}


void Column_Disable(void)
{
/*
	//TEST1
	RCC_AHB1PeriphClockCmd(COLUMN_GPIO_CLOCK, DISABLE);
*/
/*

	//TEST2
	RCC_AHB1PeriphClockCmd(UP_IN_GPIO_CLOCK, DISABLE);
	//TEST2-1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, DISABLE);
*/

	//TEST3
	EXTI_InitTypeDef EXTI_InitStructure;	
	// Configure Button EXTI line 
    EXTI_InitStructure.EXTI_Line = UP_IN_GPIO_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = UP_IN_GPIO_EXTI_MODE;
    EXTI_InitStructure.EXTI_Trigger = UP_IN_GPIO_EXTI_TRIGGER;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);

	// Configure Button EXTI line 
    EXTI_InitStructure.EXTI_Line = DOWN_IN_GPIO_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = DOWN_IN_GPIO_EXTI_MODE;
    EXTI_InitStructure.EXTI_Trigger = DOWN_IN_GPIO_EXTI_TRIGGER;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);
/**/

/*
	//TEST4
	NVIC_InitTypeDef NVIC_InitStructure;
	// Enable and set Button EXTI Interrupt to the lowest priority 
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure); 
	
	// Enable and set Button EXTI Interrupt to the lowest priority 
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&NVIC_InitStructure); 
*/
}


/**
* @ Function Name : Column_EncConfig
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void Column_EncConfig(void)
{
	TIM_ICInitTypeDef TIM_ICInitStruct;
	Timer_Typedef	ColumnEncTimer;

#if 0
	ColumnEncTimer.Periph = ENCODER_HS2_TIM;
	ColumnEncTimer.Clock = ENCODER_HS2_TIM_CLOCK;
	ColumnEncTimer.Gpio = ENCODER_HS2_GPIO_PORT;
	ColumnEncTimer.GpioClock = ENCODER_HS2_GPIO_CLOCK;
	ColumnEncTimer.GpioPin = ENCODER_HS2_GPIO_PIN;
	ColumnEncTimer.GpioPinsource = ENCODER_HS2_GPIO_PinSource;
	ColumnEncTimer.GpioAf = ENCODER_HS2_GPIO_AF;
	ColumnEncTimer.InitStatus = ENCODER_HS2_TIM_POLARITY;
	ColumnEncTimer.Interrupt = ENCODER_HS2_TIM_INTERRUPT;
	ColumnEncTimer.Source = ENCODER_HS2_TIM_SOURCE;
	ColumnEncTimer.Channel = ENCODER_HS2_TIM_CHANNEL;
	Timer_Config(&tColumnEncTimer);
#endif

	ColumnEncTimer.Periph = ENCODER_HS1_TIM;
	ColumnEncTimer.Clock = ENCODER_HS1_TIM_CLOCK;
	ColumnEncTimer.Gpio = ENCODER_HS1_GPIO_PORT;
	ColumnEncTimer.GpioClock = ENCODER_HS1_GPIO_CLOCK;
	ColumnEncTimer.GpioPin = ENCODER_HS1_GPIO_PIN;
	ColumnEncTimer.GpioPinsource = ENCODER_HS1_GPIO_PinSource;
	ColumnEncTimer.GpioAf = ENCODER_HS1_GPIO_AF;
	ColumnEncTimer.InitStatus = ENCODER_HS1_TIM_POLARITY;
	ColumnEncTimer.Interrupt = ENCODER_HS1_TIM_INTERRUPT;
	ColumnEncTimer.Source = ENCODER_HS1_TIM_SOURCE;
	ColumnEncTimer.Channel = ENCODER_HS1_TIM_CHANNEL;
	
	Timer_Config(&ColumnEncTimer);

#if 0
	/* TIM3 Input Configuration */
	TIM_ICInitStruct.TIM_Channel = ENCODER_HS2_TIM_CHANNEL;
	TIM_ICInitStruct.TIM_ICPolarity = ENCODER_HS2_TIM_POLARITY;
	TIM_ICInitStruct.TIM_ICSelection = ENCODER_HS2_TIM_SELECTION;
	TIM_ICInitStruct.TIM_ICPrescaler = ENCODER_HS2_TIM_PRESCALER;
	TIM_ICInitStruct.TIM_ICFilter = 0x0;
	TIM_ICInit(ENCODER_HS2_TIM, &TIM_ICInitStruct);

	/* TIM enable counter */
	TIM_Cmd(ENCODER_HS2_TIM, ENABLE);

	/* Enable the CC3 Interrupt Request */
	TIM_ITConfig(ENCODER_HS2_TIM, ENCODER_HS2_TIM_SOURCE, ENABLE);
#endif

	TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStruct.TIM_ICFilter = 0x0;

	TIM_PWMIConfig(ENCODER_HS1_TIM, &TIM_ICInitStruct);

	/* Select the TIM2 Input Trigger: TI2FP2 */
	TIM_SelectInputTrigger(ENCODER_HS1_TIM, TIM_TS_TI1FP1);

	/* Select the slave Mode: Reset Mode */
	TIM_SelectSlaveMode(ENCODER_HS1_TIM, TIM_SlaveMode_Reset);

	/* Enable the Master/Slave Mode */
	TIM_SelectMasterSlaveMode(ENCODER_HS1_TIM, TIM_MasterSlaveMode_Enable);

	/* TIM enable counter */
	TIM_Cmd(ENCODER_HS1_TIM, ENABLE);
	
	/* Enable the CC2 Interrupt Request */
	TIM_ITConfig(ENCODER_HS1_TIM, TIM_IT_CC1, ENABLE);

	//Column_EncControl(FALSE);

#if 0
	/* TIM enable counter */
	TIM_Cmd(ENCODER_HS1_TIM, ENABLE);

	/* Enable the CC3 Interrupt Request */
	TIM_ITConfig(ENCODER_HS1_TIM, ENCODER_HS1_TIM_SOURCE, ENABLE);

	//TIM_SelectHallSensor(ENCODER_HS1_TIM, ENABLE);
#endif	

	if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "Column Encoder Setup Completed");	
	}

}

/**
* @ Function Name : set_polarity
* @ Desc : 
* @ Param : 
* @ Return :
*/
__inline void set_polarity(TIM_TypeDef *pTim, uint16_t ch, uint16_t polarity)
{
	uint16_t c = TIM_CCER_CC1P << ((ch-1)*4);

	if (polarity == 0)
		pTim->CCER &= ~c;
	else
		pTim->CCER |= c;
}

/**
* @ Function Name : Column_EncControl
* @ Desc : 
* @ Param : 
* @ Return :
*/
void Column_EncControl(bool bEnable)
{
	if (bEnable) 
	{
		TIM_Cmd(ENCODER_HS1_TIM, ENABLE); // TIM enable counter
		TIM_ITConfig(ENCODER_HS1_TIM, ENCODER_HS1_TIM_SOURCE, ENABLE); //Enable the CC3 Interrupt Request
	} 
	else 
	{
		TIM_ITConfig(ENCODER_HS1_TIM, ENCODER_HS1_TIM_SOURCE, DISABLE);
		TIM_Cmd(ENCODER_HS1_TIM, DISABLE);
	}
}
#endif /* USE_CT_STITCH_MODE */

/**
* @ Function Name : Exposure_Config
* @ Desc : 
* @ Param : 
* @ Return :
*/
void Exposure_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIOx clock */
	RCC_AHB1PeriphClockCmd(EXPOSURE_LED_GPIO_CLOCK, ENABLE);
	
	/* GPIOx Configuration */
	GPIO_InitStructure.GPIO_Pin = EXPOSURE_LED_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = EXPOSURE_LED_GPIO_MODE;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(EXPOSURE_LED_GPIO, &GPIO_InitStructure);

	/* Exposure Port Init */
	Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);		

	GPIO_InitStructure.GPIO_Pin = EXPOSURE_SWITCH_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = EXPOSURE_SWITCH_GPIO_MODE;
	GPIO_Init(EXPOSURE_SWITCH_GPIO, &GPIO_InitStructure);	
}

/**
* @ Function Name : Exposure_CtrlLed
* @ Desc : 
* @ Param : 
* @ Return :
*/
void Exposure_CtrlLed(BitAction bit)
{
	GPIO_WriteBit(EXPOSURE_LED_GPIO, EXPOSURE_LED_GPIO_PIN, bit);	
}

/**
* @ Function Name : Exposure_DelayMs
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void Exposure_DelayMs(uint32_t millis)
{
    /* While loop takes 4 cycles */
    /* For 1 us delay, we need to divide with 4M */
    uint32_t nMultiplier = SystemCoreClock / 4000000;

    /* Multiply millis with multipler */
    /* Substract 10 */
    millis = 1000 * millis * nMultiplier - 10;
    /* 4 cycles for one loop */
    while(millis--);
}


/**
* @ Function Name : Exposure_CheckSwitch
* @ Desc : 
* @ Param : 
* @ Return :
*/
bool Exposure_CheckSwitch(void)
{
#if !defined(USE_AGING_MODE)
    BitAction check = (BitAction)GPIO_ReadInputDataBit(EXPOSURE_SWITCH_GPIO, EXPOSURE_SWITCH_GPIO_PIN);	

    if (check == SWITCH_RELEASE)   
    {
        Exposure_DelayMs(500); //Switch Debounce

        check = (BitAction)GPIO_ReadInputDataBit(EXPOSURE_SWITCH_GPIO, EXPOSURE_SWITCH_GPIO_PIN);	
        
        if (check == SWITCH_RELEASE)
        {
            return TRUE;
        }
    }
#endif /* !USE_AGING_MODE */

    // pressed
    return FALSE;
}

/**
* @ Function Name : bIsExposureSwitchReleased
* @ Desc : check exposure switch status
* @ Param : millis = check time of chattering...
*                    more better way is update senser value in timer interrupt and use other functions.
* @ Return : TRUE = released
*            FALSE = pressed
*/
bool bIsExposureSwitchReleased (uint32_t millis)
{
    BitAction bPressed = (BitAction)GPIO_ReadInputDataBit(EXPOSURE_SWITCH_GPIO, EXPOSURE_SWITCH_GPIO_PIN);	

    if (bPressed == Bit_SET) 
	{   // If the switch is releassed
        Exposure_DelayMs(millis);

        bPressed = (BitAction)GPIO_ReadInputDataBit(EXPOSURE_SWITCH_GPIO, EXPOSURE_SWITCH_GPIO_PIN);	

        if (bPressed == Bit_SET) 
		{ // If the switch is still releassed
            return TRUE;
        }
    }

    return FALSE;
}

/**
* @ Function Name : Indicator_Config
* @ Desc : 
* @ Param : 
* @ Return :
*/
void Indicator_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIOx clock */
	RCC_AHB1PeriphClockCmd(INDICATOR_GPIO_PORT_CLOCK, ENABLE);
	
	/* GPIOx Configuration */
	GPIO_InitStructure.GPIO_Pin = INDICATOR_GPIO_PORT_PIN;
	GPIO_InitStructure.GPIO_Mode = INDICATOR_GPIO_PORT_MODE;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(INDICATOR_GPIO_PORT, &GPIO_InitStructure);
	
    Indicator_Control(INDICATOR_DISABLE);
}

/**
* @ Function Name : Indicator_Control
* @ Desc : 
* @ Param : 
* @ Return :
*/
void Indicator_Control(BitAction bit)
{
    GPIO_WriteBit(INDICATOR_GPIO_PORT, INDICATOR_GPIO_PORT_PIN, bit);
}

/**
* @ Function Name : Lamp_Config
* @ Desc : 
* @ Param : 
* @ Return :
*/
void Lamp_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIOx clock */
	RCC_AHB1PeriphClockCmd(LAMP_GPIO_PORT_CLOCK, ENABLE);
	RCC_AHB1PeriphClockCmd(LAMP_MOOD_GPIO_PORT_CLOCK, ENABLE);	
	
	/* GPIOx Configuration */
	GPIO_InitStructure.GPIO_Pin = LAMP_R_GPIO_PORT_PIN |LAMP_G_GPIO_PORT_PIN |LAMP_B_GPIO_PORT_PIN;
	GPIO_InitStructure.GPIO_Mode = LAMP_GPIO_PORT_MODE;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(LAMP_GPIO_PORT, &GPIO_InitStructure);
	
    Lamp_Control(LAMP_RED, LAMP_DISABLE);
    Lamp_Control(LAMP_GREEN, LAMP_DISABLE);
    Lamp_Control(LAMP_BLUE, LAMP_DISABLE);

	GPIO_InitStructure.GPIO_Pin = LAMP_MOOD_1_GPIO_PORT_PIN |LAMP_MOOD_2_GPIO_PORT_PIN;
	GPIO_InitStructure.GPIO_Mode = LAMP_MOOD_GPIO_PORT_MODE;
	GPIO_Init(LAMP_MOOD_GPIO_PORT, &GPIO_InitStructure);

    Lamp_Control(LAMP_MOOD_1, LAMP_ENABLE);
    Lamp_Control(LAMP_MOOD_2, LAMP_ENABLE);

	if(sysInfo.bShowLog==TRUE)
	{
    	printUart(DBG_MSG_PC, "Lamp Config Done");
	}
}

void Led_dip_Config(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIOx clock */
	RCC_AHB1PeriphClockCmd(LED_DIP_GPIO_PORT_CLOCK, ENABLE);
	
	/* GPIOx Configuration */
	GPIO_InitStructure.GPIO_Pin = LED_DIP_GPIO_PORT_PIN;
	GPIO_InitStructure.GPIO_Mode = LED_DIP_GPIO_PORT_MODE;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(LED_DIP_GPIO_PORT, &GPIO_InitStructure);

	if(sysInfo.bShowLog==TRUE)
	{
	  	printUart(DBG_MSG_PC, "Led Dip Config Done");
	}
}

void Led_dip_Control(BitAction bit)
{
	GPIO_WriteBit(LED_DIP_GPIO_PORT, LED_DIP_GPIO_PORT_PIN, bit);
}

/**
* @ Function Name : Lamp_Control
* @ Desc : 
* @ Param : 
* @ Return :
*/
void Lamp_Control(uint8_t lamp, BitAction bit)
{
    switch(lamp)
    {
    case LAMP_RED:
        GPIO_WriteBit(LAMP_GPIO_PORT, LAMP_R_GPIO_PORT_PIN, bit);
        break;
        
    case LAMP_GREEN:
        GPIO_WriteBit(LAMP_GPIO_PORT, LAMP_G_GPIO_PORT_PIN, bit);
        break;
        
    case LAMP_BLUE:
        GPIO_WriteBit(LAMP_GPIO_PORT, LAMP_B_GPIO_PORT_PIN, bit);
        break;
        
    case LAMP_MOOD_1:
        GPIO_WriteBit(LAMP_MOOD_GPIO_PORT, LAMP_MOOD_1_GPIO_PORT_PIN, bit);        
        break;
        
    case LAMP_MOOD_2:
        GPIO_WriteBit(LAMP_MOOD_GPIO_PORT, LAMP_MOOD_2_GPIO_PORT_PIN, bit);
        break;
        
    default:
        break;
    }
}

void Lamp_Color_Control(Indicator_Typedef color)
{
	switch(color)
	{
		case LED_BLACK:
			Lamp_Control(LAMP_RED, LAMP_DISABLE);
		    Lamp_Control(LAMP_GREEN, LAMP_DISABLE);
		    Lamp_Control(LAMP_BLUE, LAMP_DISABLE);
			break;
		case LED_WHITE:
			Lamp_Control(LAMP_RED, LAMP_ENABLE);
		    Lamp_Control(LAMP_GREEN, LAMP_ENABLE);
		    Lamp_Control(LAMP_BLUE, LAMP_ENABLE);
			break;
		case LED_GREEN:
			Lamp_Control(LAMP_RED, LAMP_DISABLE);
		    Lamp_Control(LAMP_GREEN, LAMP_ENABLE);
		    Lamp_Control(LAMP_BLUE, LAMP_DISABLE);
			break;
		case LED_YELLOW:
			Lamp_Control(LAMP_RED, LAMP_ENABLE);
		    Lamp_Control(LAMP_GREEN, LAMP_ENABLE);
		    Lamp_Control(LAMP_BLUE, LAMP_DISABLE);
			break;
		case LED_RED:
			Lamp_Control(LAMP_RED, LAMP_ENABLE);
		    Lamp_Control(LAMP_GREEN, LAMP_DISABLE);
		    Lamp_Control(LAMP_BLUE, LAMP_DISABLE);
			break;
		case LED_PURPLE:
			Lamp_Control(LAMP_RED, LAMP_ENABLE);
		    Lamp_Control(LAMP_GREEN, LAMP_DISABLE);
		    Lamp_Control(LAMP_BLUE, LAMP_ENABLE);
			break;
		case LED_BLUE:
			Lamp_Control(LAMP_RED, LAMP_DISABLE);
		    Lamp_Control(LAMP_GREEN, LAMP_DISABLE);
		    Lamp_Control(LAMP_BLUE, LAMP_ENABLE);
			break;
		case LED_CYAN:
			Lamp_Control(LAMP_RED, LAMP_DISABLE);
		    Lamp_Control(LAMP_GREEN, LAMP_ENABLE);
		    Lamp_Control(LAMP_BLUE, LAMP_ENABLE);
			break;
		default:
			Lamp_Control(LAMP_RED, LAMP_DISABLE);
		    Lamp_Control(LAMP_GREEN, LAMP_DISABLE);
		    Lamp_Control(LAMP_BLUE, LAMP_DISABLE);
			break;
	}
}

/**
* @ Function Name : LaserConfig
* @ Desc : 
* @ Param : 
* @ Return :
*/
void LaserConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIOx clock */
	RCC_AHB1PeriphClockCmd(LASER_GPIO_PORT_CLOCK, ENABLE);
	RCC_AHB1PeriphClockCmd(LASER_F_GPIO_PORT_CLOCK, ENABLE);
	
	/* GPIOx Configuration */
	GPIO_InitStructure.GPIO_Pin = LASER_V_GPIO_PORT_PIN;
	GPIO_InitStructure.GPIO_Mode = LASER_GPIO_PORT_MODE;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(LASER_GPIO_PORT, &GPIO_InitStructure);

	LaserSetVertical(BEAM_OFF);

	GPIO_InitStructure.GPIO_Pin = LASER_F_GPIO_PORT_PIN;
	GPIO_Init(LASER_F_GPIO_PORT, &GPIO_InitStructure);

	LaserSetFoot(BEAM_OFF);
}

/**
* @ Function Name : LaserControl
* @ Desc : 
* @ Param : 
* @ Return :
*/
void LaserControl(BeamType_t beamType, bool isEnable)
{	
	if (isEnable == TRUE)
	{
		if (beamType == TYPE_FOOT)
		{
			LaserSetFoot(BEAM_ON);
		}
		else // beamType == TYPE_HEAD_
		{
			LaserSetVertical(BEAM_ON);

            if (beamType == TYPE_HEAD_PANO)
            {
    			CAN_Collimator_SendMessage(CMD_LASER_CANINE_ON, 0, 0, 2000, 2);
    			CAN_Collimator_SendMessage(CMD_LASER_HORI_TOP_ON, 0, 0, 2000, 2);
            }
            else if (beamType == TYPE_HEAD_CT)
            {
    			CAN_Collimator_SendMessage(CMD_LASER_CANINE_ON, 0, 0, 2000, 2);
            }
            else // beamType == TYPE_HEAD_CEPH
            {
				CAN_Collimator_SendMessage(CMD_LASER_HORI_BOT_ON, 0, 0, 2000, 2);
            }
		}
	}
	else // disable
	{
		if (beamType == TYPE_FOOT)
		{
			LaserSetFoot(BEAM_OFF);
		}
		else // beamType == TYPE_HEAD_
		{
			LaserSetVertical(BEAM_OFF);

            if (beamType == TYPE_HEAD_PANO)
            {
    			CAN_Collimator_SendMessage(CMD_LASER_CANINE_OFF, 0, 0, 2000, 2);
    			CAN_Collimator_SendMessage(CMD_LASER_HORI_TOP_OFF, 0, 0, 2000, 2);
            }
            else if (beamType == TYPE_HEAD_CT)
            {
    			CAN_Collimator_SendMessage(CMD_LASER_CANINE_OFF, 0, 0, 2000, 2);
            }
            else // beamType == TYPE_HEAD_CEPH
            {
				CAN_Collimator_SendMessage(CMD_LASER_HORI_BOT_OFF, 0, 0, 2000, 2);
            }
		}
	}
}

/**
* @ Function Name : LaserSetVertical
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void LaserSetVertical(BitAction bit)
{
	GPIO_WriteBit(LASER_GPIO_PORT, LASER_V_GPIO_PORT_PIN, bit);
}

/**
* @ Function Name : LaserSetFoot
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void LaserSetFoot(BitAction bit)
{
	GPIO_WriteBit(LASER_F_GPIO_PORT, LASER_F_GPIO_PORT_PIN, bit);
}

/**
* @ Function Name : LaserSetPosition
* @ Desc : 
* @ Param : 
* @ Return :
*/
void LaserSetPosition(void)
{
	if ((CurCaptureMode == CAPTURE_SCAN) || (CurCaptureMode == CALIBRATION_MODE))
		CAN_Collimator_SendMessage(CMD_LASER_POSITION_CEPH, 0, 0, 2000, 2);
	else
		CAN_Collimator_SendMessage(CMD_LASER_POSITION_PANO, 0, 0, 2000, 2);
}



// 200408  HWAN TEST STITCH SENSOR

void StichSensor_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(STICH_UNDER_SENSOR_GPIO_CLOCK, ENABLE);	
	GPIO_InitStructure.GPIO_Pin = STICH_UNDER_SENSOR_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = STICH_UNDER_SENSOR_GPIO_MODE;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(STICH_UNDER_SENSOR_GPIO, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(STICH_UP_SWITCH_GPIO_CLOCK, ENABLE);	
	GPIO_InitStructure.GPIO_Pin = STICH_UP_SWITCH_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = STICH_UP_SWITCH_GPIO_MODE;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(STICH_UP_SWITCH_GPIO, &GPIO_InitStructure);
}


bool StitchUnderSensor_Check(void)
{	
	bool ret = FALSE;

	ret = (GPIO_ReadInputDataBit(STICH_UNDER_SENSOR_GPIO, STICH_UNDER_SENSOR_GPIO_PIN)) ? TRUE : FALSE;

	return ret;
}

bool StitchUpSwitch_Check(void)
{	
	bool ret = FALSE;

	ret = (!GPIO_ReadInputDataBit(STICH_UP_SWITCH_GPIO, STICH_UP_SWITCH_GPIO_PIN)) ? TRUE : FALSE;

	return ret;
}

/**/

/**
  * @brief  This function handles External lines 0 interrupt request.
  * @param  None
  * @retval : None
  */
void EXTI0_IRQHandler(void)
{
    if (EXTI_GetITStatus(UP_IN_GPIO_EXTI_LINE) != RESET) 
	{
        if (!GPIO_ReadInputDataBit(UP_IN_GPIO_PORT, UP_IN_GPIO_PIN)) 
		{
            //Column_Delay(500);
            if (!GPIO_ReadInputDataBit(UP_IN_GPIO_PORT, UP_IN_GPIO_PIN))
                Column_Control(COLUMN_UP);
        } 
		else 
		{
            Column_Control(COLUMN_STOP);
        }
        
        /* Clear the Emergency Switch EXTI line pending bit */
        EXTI_ClearITPendingBit(UP_IN_GPIO_EXTI_LINE);
    }
}

/**
  * @brief  This function handles External lines 11 interrupt request.
  * @param  None
  * @retval : None
  */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(DOWN_IN_GPIO_EXTI_LINE) != RESET) 
	{
        if (!GPIO_ReadInputDataBit(DOWN_IN_GPIO_PORT, DOWN_IN_GPIO_PIN)) 
		{
            //Column_Delay(500);
            if (!GPIO_ReadInputDataBit(DOWN_IN_GPIO_PORT, DOWN_IN_GPIO_PIN))
                Column_Control(COLUMN_DOWN);
        } 
		else 
		{
            Column_Control(COLUMN_STOP);
        }

        /* Clear the Emergency Switch EXTI line pending bit */
        EXTI_ClearITPendingBit(DOWN_IN_GPIO_EXTI_LINE);
    }
}

#ifdef USE_CT_STITCH_MODE
/**
  * @brief  This function handles Timer2 interrupt request.
  * @param  None
  * @retval : None
  */
void ENCODER_HS1_TIM_ISR(void)
{
	/* Clear TIM4 Capture compare interrupt pending bit */
	TIM_ClearITPendingBit(ENCODER_HS1_TIM, TIM_IT_CC1);

	/* Get the Input Capture value */
	IC1Value = TIM_GetCapture1(ENCODER_HS1_TIM);
	if (IC1Value != 0) 
	{
		if (bTsMotorInterlock == TRUE) 
		{
			if(nHallSensorCount==0)
			{
				IC1_VALUE_Index=0;				
			}
			
			if (nHallSensorCount == 1) 
			{
				Motor_ControlPWM(TRUE, MOTOR_CNS_INIT_ARR);
			} 
			else if(nHallSensorCount<limit_count)
			{
				float nArr = (float)IC1Value/nMotorPulse;
			
				if (nHallSensorCount <= 4) 
				{
					nArr = MOTOR_CNS_INIT_ARR;
				}
				
				MOTOR_CNS->ARR = nArr-0.5;
				MOTOR_CNS->CCR3 = nArr / 2;
				TIM_SetAutoreload(Motor_C.Timer.Periph, nArr/2); 	// ARR
				//TIM_ITConfig(MOTOR_CNS, MOTOR_CNS_SOURCE, ENABLE);
				//TIM_ARRPreloadConfig(MOTOR_CNS, ENABLE);
				
			}		
			else if(nHallSensorCount !=0)
			{
				float nArr = (float)IC1Value/nMotorPulse;
				
				MOTOR_CNS->ARR = nArr + (nHallSensorCount - limit_count)*MOTOR_CNS_ACC_COUNT;
				MOTOR_CNS->CCR3 = nArr / 2;
				
				Column_Control(COLUMN_STOP);
				bColumnStop = TRUE;
			} 
			nHallSensorCount++;

			//200217 HWAN TEST
			/*
			if(IC1Value!=Before_IC1Value)
			{		
				float nArr = (float)IC1Value/nMotorPulse;
				Before_IC1Value=IC1Value;
				CtParam.IC1VALUE_TABLE[IC1_VALUE_Index]=IC1Value;
				CtParam.IC1ARR[IC1_VALUE_Index]=nArr;
				if(IC1_VALUE_Index!=0) CtParam.IC1VALUE_SAME[IC1_VALUE_Index-1]=IC1_SAME_VALUE;
				if(IC1_VALUE_Index<350) IC1_VALUE_Index++;
				IC1_SAME_VALUE=1;
			}
			else
			{
				IC1_SAME_VALUE++;
			}*/
		}
	}

	
	
	/*
	//200212 HWAN

	if (IC1Value != 0) 
	{
		if (bTsMotorInterlock == TRUE) 
		{
			
			if (nHallSensorCount == 0) 
			{
				Motor_ControlPWM(TRUE, MOTOR_CNS_INIT_ARR);
			} 
			else if(nHallSensorCount<limit_count)
			{
				uint32_t nArr = IC1Value/nMotorPulse;
			
				if (nHallSensorCount <= HS_ACC_COUNT) 
				{
					nArr = MOTOR_CNS_INIT_ARR - nHallSensorCount + 7;
				}
				
				if(nHallSensorCount <= (limit_count)/2)
				{
					MOTOR_CNS->ARR = nArr;
					MOTOR_CNS->CCR3 = nArr / 2;
				}
				else
				{
					MOTOR_CNS->ARR = nArr-1;
					MOTOR_CNS->CCR3 = nArr / 2;
				}

				
				
			}		
			else
			{
				uint32_t nArr = IC1Value/nMotorPulse;
				
				MOTOR_CNS->ARR = nArr - 1 + nHallSensorCount - limit_count;
				MOTOR_CNS->CCR3 = nArr / 2;
				
				Column_Control(COLUMN_STOP);
				bColumnStop = TRUE;
			} 
			nHallSensorCount++;
		}
	}

	
	//MOTOR_CNS_INIT_ARR = 85
	if (IC1Value != 0) 
	{
		if (bTsMotorInterlock == TRUE) 
		{
			
			if (nHallSensorCount == 0) 
			{
				Motor_ControlPWM(TRUE, MOTOR_CNS_INIT_ARR);
			} 
			else
			{
				uint32_t nArr = IC1Value/nMotorPulse;
			
				if (nHallSensorCount <= 6) 
				{
					nArr = MOTOR_CNS_INIT_ARR - nHallSensorCount;
				}
				else if (nHallSensorCount <= 8) 
				{
					nArr = MOTOR_CNS_INIT_ARR - nHallSensorCount - 5;
				}
				else if(nHallSensorCount<= HS_ACC_COUNT)
				{
					nArr = MOTOR_CNS_INIT_ARR - ((nHallSensorCount-1) * MOTOR_CNS_ACC_COUNT);
				}
				
				if(nHallSensorCount <= (limit_count*2)/5)
				{
					MOTOR_CNS->ARR = nArr-1;
					MOTOR_CNS->CCR3 = nArr / 2;
				}
				else
				{
					MOTOR_CNS->ARR = nArr;
					MOTOR_CNS->CCR3 = nArr / 2;
				}

				
				
			}		

			if (nHallSensorCount >= limit_count) 
			{
				uint32_t nArr = IC1Value/nMotorPulse;
				
				MOTOR_CNS->ARR = nArr - (4+limit_count-nHallSensorCount);
				MOTOR_CNS->CCR3 = nArr / 2;
				
				Column_Control(COLUMN_STOP);
				bColumnStop = TRUE;
			} 
			nHallSensorCount++;
		}
	}
	
	*/
}
#endif /* USE_CT_STITCH_MODE */

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/

