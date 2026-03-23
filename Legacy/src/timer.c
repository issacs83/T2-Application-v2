/*
*******************************************************************************
* timer.c :
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
#include "extern.h"
#include "system.h"
#include "serial.h"
#include "timer.h"
#include "misc1.h"


/* Private typedef --------------------------------------------------- */
/* Private define ---------------------------------------------------- */
#define	INT_TIMER						TIM6
#define	INT_TIMER_CLOCK					RCC_APB1Periph_TIM6
#define	INT_TIMER_PERIOD				1250 // 1 msec
#define	INT_TIMER_INTERRUPT				TIM6_DAC_IRQn
//#define	INT_TIMER_INTERRUPT			TIM7_IRQn

#define LAMP_TIMER_PERIOD				10000

// Config-Bit to start or stop the SysTick Timer
#define SYSTICK_ENABLE 					0
// Config-Bit to enable or disable the SysTick interrupt
#define SYSTICK_TICKINT 				1
// Config-Bit to enable or disable the SysTick interrupt
#define SYSTICK_CLKSOURCE 				2

#define	ELAPSED_TIM 					TIM7
#define	ELAPSED_TIM_CLOCK 				RCC_APB1Periph_TIM7
#define	ELAPSED_TIM_PERIOD 				1250 /* 1 msec */
#define	ELAPSED_TIM_INTERRUPT 			TIM7_IRQn
#define ERROR_NONE						0
#define ERROR_COLLIMATOR				1
#define ERROR_GENERATOR					2
#define ERROR_MOTOR						3
#define ERROR_STATUS_START				4
#define ERROR_LAMP_OFF					5


/* Private macro ---------------------------------------------------- */
/* Private variables -------------------------------------------------- */
IntTimer_Typedef IntTimer;
ElapsedTimer_Typedef typElapsedTimer;
static volatile unsigned int nSysTicks;


/* Private function prototypes ------------------------------------------*/
static void Timer_Init(Timer_Typedef* timer);
static void Timer_Init_CT400(Timer_Typedef* timer);
static void Timer_Init_CT600(Timer_Typedef* timer);
static void IntTimer_ParamInit(void);
static void IntTimer_Init(void);
static void vElapsedTimerParamInit(void);
static void vElapsedTimerInit(void);
//jehun
void Timer_Init_T(Timer_Typedef * timer);
void Timer_Init_T_CT400(Timer_Typedef * timer);
void Timer_Init_T_CT600(Timer_Typedef * timer);

/* Private functions -------------------------------------------------- */
	//jehun
void Timer_Init_T_CT600(Timer_Typedef * timer)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	uint16_t PrescalerValue = 0;

	// Compute the prescaler value 

	if((timer->Periph == TIM1) || (timer->Periph == TIM8) ||(timer->Periph == TIM9) ||\
		(timer->Periph == TIM10) || (timer->Periph == TIM11)) 
		PrescalerValue = (uint16_t) (SystemCoreClock / 5000000.0) - 1;
	else
		PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 5000000.0) - 1;
	
	// Time base configuration 
	TIM_TimeBaseStructure.TIM_Period		= 65535;
	TIM_TimeBaseStructure.TIM_Prescaler		= PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision	= 0;
	TIM_TimeBaseStructure.TIM_CounterMode	= TIM_CounterMode_Up;
	TIM_TimeBaseInit(timer->Periph, &TIM_TimeBaseStructure);

	// TIMx IT enable 
	TIM_ITConfig(timer->Periph, timer->Source, ENABLE);

	if((timer->Periph == TIM1) || (timer->Periph == TIM8))
		TIM_CtrlPWMOutputs(timer->Periph, ENABLE);
	// Enable the TIMx Interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			= 0;//timer->Priority;
	NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel						= timer->Interrupt;
	NVIC_Init(&NVIC_InitStructure);
	
}
void Timer_Init_T_CT400(Timer_Typedef * timer)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	uint16_t PrescalerValue = 0;

	// Compute the prescaler value

	if((timer->Periph == TIM1) || (timer->Periph == TIM8) ||(timer->Periph == TIM9) ||\
		(timer->Periph == TIM10) || (timer->Periph == TIM11))
		PrescalerValue = (uint16_t) (SystemCoreClock / 2500000.0) - 1;
	else
		PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 5000000.0) - 1;

	// Time base configuration
	TIM_TimeBaseStructure.TIM_Period		= 65535;
	TIM_TimeBaseStructure.TIM_Prescaler		= PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision	= 0;
	TIM_TimeBaseStructure.TIM_CounterMode	= TIM_CounterMode_Up;
	TIM_TimeBaseInit(timer->Periph, &TIM_TimeBaseStructure);

	// TIMx IT enable
	TIM_ITConfig(timer->Periph, timer->Source, ENABLE);

	if((timer->Periph == TIM1) || (timer->Periph == TIM8))
		TIM_CtrlPWMOutputs(timer->Periph, ENABLE);
	// Enable the TIMx Interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			= 0;//timer->Priority;
	NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel						= timer->Interrupt;
	NVIC_Init(&NVIC_InitStructure);

}

void Timer_Init_T(Timer_Typedef * timer)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	uint16_t PrescalerValue = 0;

	// Compute the prescaler value

	if((timer->Periph == TIM1) || (timer->Periph == TIM8) ||(timer->Periph == TIM9) ||\
		(timer->Periph == TIM10) || (timer->Periph == TIM11))
		PrescalerValue = (uint16_t) (SystemCoreClock / TIM_COUNT_CLOCK) - 1;
	else
		PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / TIM_COUNT_CLOCK) - 1;

	// Time base configuration
	TIM_TimeBaseStructure.TIM_Period		= 65535;
	TIM_TimeBaseStructure.TIM_Prescaler		= PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision	= 0;
	TIM_TimeBaseStructure.TIM_CounterMode	= TIM_CounterMode_Up;
	TIM_TimeBaseInit(timer->Periph, &TIM_TimeBaseStructure);

	// TIMx IT enable
	TIM_ITConfig(timer->Periph, timer->Source, ENABLE);

	if((timer->Periph == TIM1) || (timer->Periph == TIM8))
		TIM_CtrlPWMOutputs(timer->Periph, ENABLE);
	// Enable the TIMx Interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			= 0;//timer->Priority;
	NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel						= timer->Interrupt;
	NVIC_Init(&NVIC_InitStructure);

}
//jehun
void Timer_Config_T(Timer_Typedef* timer)
{
//    GPIO_InitTypeDef GPIO_InitStructure;
	
	// TIMx Deinitialization
	//TIM_DeInit(timer->Periph);

	/* Enable TIM clock */
	if((timer->Periph == TIM1) || (timer->Periph == TIM8) || (timer->Periph == TIM9) ||\
		(timer->Periph == TIM10) || (timer->Periph == TIM11))
		RCC_APB2PeriphClockCmd(timer->Clock, ENABLE);
	else
		RCC_APB1PeriphClockCmd(timer->Clock, ENABLE);

	// Enable GPIO clock
	RCC_AHB1PeriphClockCmd(timer->GpioClock, ENABLE);

	if(timer->Periph == TIM8)
	{
		if(CurCaptureMode==CAPTURE_CT && CtParam.TotalFrame==600)
			Timer_Init_T_CT600(timer);
		else if(CurCaptureMode==CAPTURE_PANO)
			Timer_Init_T(timer);
		else
			Timer_Init_T_CT400(timer);
	}
	else
		Timer_Init_T(timer);
}   

// jehun - 20200722
// 2차 Collimator resolution 수정
// 진행중
void Timer_Config_CC(Timer_Typedef* timer, unsigned char prescaler)
{

	RCC_APB2PeriphClockCmd(timer->Clock, DISABLE);
	// Enable GPIO clock
	RCC_AHB1PeriphClockCmd(timer->GpioClock, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;

	uint16_t PrescalerValue = 0;

	//    GPIO_InitTypeDef GPIO_InitStructure;

	// TIMx Deinitialization
	TIM_DeInit(timer->Periph);


	/* Enable TIM clock */
	if((timer->Periph == TIM1) || (timer->Periph == TIM8) || (timer->Periph == TIM9) ||\
		(timer->Periph == TIM10) || (timer->Periph == TIM11))
		RCC_APB2PeriphClockCmd(timer->Clock, ENABLE);
	else
		RCC_APB1PeriphClockCmd(timer->Clock, ENABLE);


	// Compute the prescaler value
	// SystemCoreClock / TIM_COUNT_CLOCK = 96
	if(prescaler == 2){
		PrescalerValue = (uint16_t) (SystemCoreClock / TIM_COUNT_CLOCK /2) - 1; // 48 --> 2.5*2Mhz
	}
	else if(prescaler == 4){
		PrescalerValue = (uint16_t) (SystemCoreClock / TIM_COUNT_CLOCK /4) - 1; // 24 --> 5*2MHz
	}
	else if(prescaler == 8){
		PrescalerValue = (uint16_t) (SystemCoreClock / TIM_COUNT_CLOCK /8) - 1; // 16 --> 7.5*2Mhz
	}
	else if(prescaler == 16){
		PrescalerValue = (uint16_t) (SystemCoreClock / TIM_COUNT_CLOCK /16) - 1; // 12 --> 10*2MHz
	}
	else{
		PrescalerValue = (uint16_t) (SystemCoreClock / TIM_COUNT_CLOCK ) - 1; // 96 --> 1.25*2MHz
	}

	// Time base configuration
	TIM_TimeBaseStructure.TIM_Period		= 65535;
	TIM_TimeBaseStructure.TIM_Prescaler		= PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision	= 0;
	TIM_TimeBaseStructure.TIM_CounterMode	= TIM_CounterMode_Up;
	TIM_TimeBaseInit(timer->Periph, &TIM_TimeBaseStructure);

	// TIMx IT enable
	TIM_ITConfig(timer->Periph, timer->Source, ENABLE);

	// Enable the TIMx Interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			= 0;//timer->Priority;
	NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel						= timer->Interrupt;
	NVIC_Init(&NVIC_InitStructure);

}

/**
* @ Function Name : Timer_Config
* @ Desc : 
* @ Param : 
* @ Return :
*/
void Timer_Config(Timer_Typedef* timer)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	
	/* TIMx Deinitialization */
	//TIM_DeInit(timer->Periph);

	/* Enable TIM clock */
	if((timer->Periph == TIM1) || (timer->Periph == TIM8) || (timer->Periph == TIM9) ||\
		(timer->Periph == TIM10) || (timer->Periph == TIM11))
		RCC_APB2PeriphClockCmd(timer->Clock, ENABLE);
	else
		RCC_APB1PeriphClockCmd(timer->Clock, ENABLE);

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(timer->GpioClock, ENABLE);

	/* GPIOx Configuration */
	GPIO_InitStructure.GPIO_Pin		= timer->GpioPin;
	GPIO_InitStructure.GPIO_Mode	= GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed	= GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd	= GPIO_PuPd_UP;
	GPIO_Init(timer->Gpio, &GPIO_InitStructure);

	/* Connect TIMx Channels to AFx */
	GPIO_PinAFConfig(timer->Gpio, timer->GpioPinsource, timer->GpioAf);

	/* TIMx configuration */
	/*if(CurCaptureMode==CAPTURE_CT && timer->Periph == TIM8)
	{
		if(CtParam.TotalFrame==400)
			Timer_Init_CT400(timer);
		else
			Timer_Init_CT600(timer);
	}*/
	if(timer->Periph == TIM8)
	{
		if(CurCaptureMode==CAPTURE_CT && CtParam.TotalFrame==600)
			Timer_Init_CT600(timer);
		else if(CurCaptureMode==CAPTURE_PANO)
			Timer_Init(timer);
		else
			Timer_Init_CT400(timer);
	}
	else
		Timer_Init(timer);
}

/**
* @ Function Name : Timer_Init
* @ Desc : 
* @ Param : 
* @ Return :
*/
void Timer_Init(Timer_Typedef * timer)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* ---------------------------------------------------------------------------
		TIM9 Configuration: Output Compare Toggle Mode:

		In this example TIM9 input clock (TIM9CLK) is set to 2 * APB2 clock (PCLK2),
		since APB2 prescaler is different from 1.   
			TIM9CLK = 2 * PCLK2  
			PCLK2 = HCLK / 2 
			=> TIM9CLK = HCLK = SystemCoreClock
		  
		To get TIM9 counter clock at 15 MHz, the prescaler is computed as follows:
			Prescaler = (TIM9CLK / TIM9 counter clock) - 1
			Prescaler = (SystemCoreClock /15 MHz) - 1
											  
		CC1 update rate = TIM9 counter clock / CCR1_Val = 366.2 Hz
			==> So the TIM9 Channel 1 generates a periodic signal with a frequency equal to 183.1 Hz

		CC2 update rate = TIM9 counter clock / CCR2_Val = 732.4 Hz
			==> So the TIM9 channel 2 generates a periodic signal with a frequency equal to 366.3 Hz

		Note: 
			SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f2xx.c file.
			Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
			function to update SystemCoreClock variable value. Otherwise, any configuration
			based on this variable will be incorrect.    
	--------------------------------------------------------------------------- */   

	uint16_t PrescalerValue = 0;

	/* Compute the prescaler value */
#if defined(USE_CT_XRAY_PULSED_MODE)
	if((timer->Periph == TIM1) || (timer->Periph == TIM8) ||(timer->Periph == TIM9) ||\
		(timer->Periph == TIM10) || (timer->Periph == TIM11) ||(timer->Periph == TIM12)) 
		PrescalerValue = (uint16_t) (SystemCoreClock / TIM_COUNT_CLOCK) - 1;	
	else
		PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / TIM_COUNT_CLOCK) - 1;
#else /* not USE_CT_XRAY_PULSED_MODE */
	if((timer->Periph == TIM1) || (timer->Periph == TIM8) ||(timer->Periph == TIM9) ||\
		(timer->Periph == TIM10) || (timer->Periph == TIM11)) 
		PrescalerValue = (uint16_t) (SystemCoreClock / TIM_COUNT_CLOCK) - 1;	
	else
		PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / TIM_COUNT_CLOCK) - 1;

#endif /* USE_CT_XRAY_PULSED_MODE */

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period		= 65535;
	TIM_TimeBaseStructure.TIM_Prescaler		= PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision	= 0;
	TIM_TimeBaseStructure.TIM_CounterMode	= TIM_CounterMode_Up;
	TIM_TimeBaseInit(timer->Periph, &TIM_TimeBaseStructure);

	/* Output Compare Toggle Mode configuration: Channelx */
	TIM_OCInitStructure.TIM_OCMode				= TIM_OCMode_Toggle;
	TIM_OCInitStructure.TIM_OCPolarity			= timer->InitStatus;
	TIM_OCInitStructure.TIM_OutputState			= TIM_OutputState_Enable;
//	TIM_OCInitStructure.TIM_Pulse				= timer->InitCCR;
	
	switch(timer->Channel)
	{
	case TIM_Channel_1:
		TIM_OC1Init(timer->Periph, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(timer->Periph, TIM_OCPreload_Disable);
		
		/* TIMx's pending flags clear */
		TIM_ClearFlag(timer->Periph, TIM_FLAG_CC1);
		break;
        
	case TIM_Channel_2:
		TIM_OC2Init(timer->Periph, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(timer->Periph, TIM_OCPreload_Disable);
		
		/* TIMx's pending flags clear */
		TIM_ClearFlag(timer->Periph, TIM_FLAG_CC2);
		break;
        
	case TIM_Channel_3:
		TIM_OC3Init(timer->Periph, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(timer->Periph, TIM_OCPreload_Disable);
		
		/* TIMx's pending flags clear */
		TIM_ClearFlag(timer->Periph, TIM_FLAG_CC3);
		break;
        
	case TIM_Channel_4:
		TIM_OC4Init(timer->Periph, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(timer->Periph, TIM_OCPreload_Disable);
		
		/* TIMx's pending flags clear */
		TIM_ClearFlag(timer->Periph, TIM_FLAG_CC4);
		break;
        
	default:
		// error
		break;
	}
	
	/* TIMx IT enable */
	TIM_ITConfig(timer->Periph, timer->Source, ENABLE);

	if((timer->Periph == TIM1) || (timer->Periph == TIM8))
		TIM_CtrlPWMOutputs(timer->Periph, ENABLE);
		
	/* Enable the TIMx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			= 0;//timer->Priority;
	NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel						= timer->Interrupt;
	NVIC_Init(&NVIC_InitStructure);
	
}

/**
* @ Function Name : Timer_Init_CT
* @ Desc : 
* @ Param : 
* @ Return :
*/

void Timer_Init_CT400(Timer_Typedef * timer)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* ---------------------------------------------------------------------------
		TIM9 Configuration: Output Compare Toggle Mode:

		In this example TIM9 input clock (TIM9CLK) is set to 2 * APB2 clock (PCLK2),
		since APB2 prescaler is different from 1.   
			TIM9CLK = 2 * PCLK2  
			PCLK2 = HCLK / 2 
			=> TIM9CLK = HCLK = SystemCoreClock
		  
		To get TIM9 counter clock at 15 MHz, the prescaler is computed as follows:
			Prescaler = (TIM9CLK / TIM9 counter clock) - 1
			Prescaler = (SystemCoreClock /15 MHz) - 1
											  
		CC1 update rate = TIM9 counter clock / CCR1_Val = 366.2 Hz
			==> So the TIM9 Channel 1 generates a periodic signal with a frequency equal to 183.1 Hz

		CC2 update rate = TIM9 counter clock / CCR2_Val = 732.4 Hz
			==> So the TIM9 channel 2 generates a periodic signal with a frequency equal to 366.3 Hz

		Note: 
			SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f2xx.c file.
			Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
			function to update SystemCoreClock variable value. Otherwise, any configuration
			based on this variable will be incorrect.    
	--------------------------------------------------------------------------- */   

	uint16_t PrescalerValue = 0;

	/* Compute the prescaler value */
#if defined(USE_CT_XRAY_PULSED_MODE)
	if((timer->Periph == TIM1) || (timer->Periph == TIM8) ||(timer->Periph == TIM9) ||\
		(timer->Periph == TIM10) || (timer->Periph == TIM11) ||(timer->Periph == TIM12)) 
		PrescalerValue = (uint16_t) (SystemCoreClock / 2500000.0) - 1;	
	else
		PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 2500000.0) - 1;
#else /* not USE_CT_XRAY_PULSED_MODE */
	if((timer->Periph == TIM1) || (timer->Periph == TIM8) ||(timer->Periph == TIM9) ||\
		(timer->Periph == TIM10) || (timer->Periph == TIM11)) 
		PrescalerValue = (uint16_t) (SystemCoreClock / 2500000.0) - 1;	
	else
		PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 5000000.0) - 1;

#endif /* USE_CT_XRAY_PULSED_MODE */

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period		= 65535;
	TIM_TimeBaseStructure.TIM_Prescaler		= PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision	= 0;
	TIM_TimeBaseStructure.TIM_CounterMode	= TIM_CounterMode_Up;
	TIM_TimeBaseInit(timer->Periph, &TIM_TimeBaseStructure);

	/* Output Compare Toggle Mode configuration: Channelx */
	TIM_OCInitStructure.TIM_OCMode				= TIM_OCMode_Toggle;
	TIM_OCInitStructure.TIM_OCPolarity			= timer->InitStatus;
	TIM_OCInitStructure.TIM_OutputState			= TIM_OutputState_Enable;
//	TIM_OCInitStructure.TIM_Pulse				= timer->InitCCR;
	
	switch(timer->Channel)
	{
	case TIM_Channel_1:
		TIM_OC1Init(timer->Periph, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(timer->Periph, TIM_OCPreload_Disable);
		
		/* TIMx's pending flags clear */
		TIM_ClearFlag(timer->Periph, TIM_FLAG_CC1);
		break;
        
	case TIM_Channel_2:
		TIM_OC2Init(timer->Periph, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(timer->Periph, TIM_OCPreload_Disable);
		
		/* TIMx's pending flags clear */
		TIM_ClearFlag(timer->Periph, TIM_FLAG_CC2);
		break;
        
	case TIM_Channel_3:
		TIM_OC3Init(timer->Periph, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(timer->Periph, TIM_OCPreload_Disable);
		
		/* TIMx's pending flags clear */
		TIM_ClearFlag(timer->Periph, TIM_FLAG_CC3);
		break;
        
	case TIM_Channel_4:
		TIM_OC4Init(timer->Periph, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(timer->Periph, TIM_OCPreload_Disable);
		
		/* TIMx's pending flags clear */
		TIM_ClearFlag(timer->Periph, TIM_FLAG_CC4);
		break;
        
	default:
		// error
		break;
	}
	
	/* TIMx IT enable */
	TIM_ITConfig(timer->Periph, timer->Source, ENABLE);

	if((timer->Periph == TIM1) || (timer->Periph == TIM8))
		TIM_CtrlPWMOutputs(timer->Periph, ENABLE);
		
	/* Enable the TIMx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			= 0;//timer->Priority;
	NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel						= timer->Interrupt;
	NVIC_Init(&NVIC_InitStructure);
	
}

/**
* @ Function Name : Timer_Init_CT
* @ Desc : 
* @ Param : 
* @ Return :
*/

void Timer_Init_CT600(Timer_Typedef * timer)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* ---------------------------------------------------------------------------
		TIM9 Configuration: Output Compare Toggle Mode:

		In this example TIM9 input clock (TIM9CLK) is set to 2 * APB2 clock (PCLK2),
		since APB2 prescaler is different from 1.   
			TIM9CLK = 2 * PCLK2  
			PCLK2 = HCLK / 2 
			=> TIM9CLK = HCLK = SystemCoreClock
		  
		To get TIM9 counter clock at 15 MHz, the prescaler is computed as follows:
			Prescaler = (TIM9CLK / TIM9 counter clock) - 1
			Prescaler = (SystemCoreClock /15 MHz) - 1
											  
		CC1 update rate = TIM9 counter clock / CCR1_Val = 366.2 Hz
			==> So the TIM9 Channel 1 generates a periodic signal with a frequency equal to 183.1 Hz

		CC2 update rate = TIM9 counter clock / CCR2_Val = 732.4 Hz
			==> So the TIM9 channel 2 generates a periodic signal with a frequency equal to 366.3 Hz

		Note: 
			SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f2xx.c file.
			Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
			function to update SystemCoreClock variable value. Otherwise, any configuration
			based on this variable will be incorrect.    
	--------------------------------------------------------------------------- */   

	uint16_t PrescalerValue = 0;

	/* Compute the prescaler value */
#if defined(USE_CT_XRAY_PULSED_MODE)
	if((timer->Periph == TIM1) || (timer->Periph == TIM8) ||(timer->Periph == TIM9) ||\
		(timer->Periph == TIM10) || (timer->Periph == TIM11) ||(timer->Periph == TIM12)) 
		PrescalerValue = (uint16_t) (SystemCoreClock / 5000000.0) - 1;	
	else
		PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 5000000.0) - 1;
#else /* not USE_CT_XRAY_PULSED_MODE */
	if((timer->Periph == TIM1) || (timer->Periph == TIM8) ||(timer->Periph == TIM9) ||\
		(timer->Periph == TIM10) || (timer->Periph == TIM11)) 
		PrescalerValue = (uint16_t) (SystemCoreClock / 5000000.0) - 1;	
	else
		PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 5000000.0) - 1;

#endif /* USE_CT_XRAY_PULSED_MODE */

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period		= 65535;
	TIM_TimeBaseStructure.TIM_Prescaler		= PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision	= 0;
	TIM_TimeBaseStructure.TIM_CounterMode	= TIM_CounterMode_Up;
	TIM_TimeBaseInit(timer->Periph, &TIM_TimeBaseStructure);

	/* Output Compare Toggle Mode configuration: Channelx */
	TIM_OCInitStructure.TIM_OCMode				= TIM_OCMode_Toggle;
	TIM_OCInitStructure.TIM_OCPolarity			= timer->InitStatus;
	TIM_OCInitStructure.TIM_OutputState			= TIM_OutputState_Enable;
//	TIM_OCInitStructure.TIM_Pulse				= timer->InitCCR;
	
	switch(timer->Channel)
	{
	case TIM_Channel_1:
		TIM_OC1Init(timer->Periph, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(timer->Periph, TIM_OCPreload_Disable);
		
		/* TIMx's pending flags clear */
		TIM_ClearFlag(timer->Periph, TIM_FLAG_CC1);
		break;
        
	case TIM_Channel_2:
		TIM_OC2Init(timer->Periph, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(timer->Periph, TIM_OCPreload_Disable);
		
		/* TIMx's pending flags clear */
		TIM_ClearFlag(timer->Periph, TIM_FLAG_CC2);
		break;
        
	case TIM_Channel_3:
		TIM_OC3Init(timer->Periph, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(timer->Periph, TIM_OCPreload_Disable);
		
		/* TIMx's pending flags clear */
		TIM_ClearFlag(timer->Periph, TIM_FLAG_CC3);
		break;
        
	case TIM_Channel_4:
		TIM_OC4Init(timer->Periph, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(timer->Periph, TIM_OCPreload_Disable);
		
		/* TIMx's pending flags clear */
		TIM_ClearFlag(timer->Periph, TIM_FLAG_CC4);
		break;
        
	default:
		// error
		break;
	}
	
	/* TIMx IT enable */
	TIM_ITConfig(timer->Periph, timer->Source, ENABLE);

	if((timer->Periph == TIM1) || (timer->Periph == TIM8))
		TIM_CtrlPWMOutputs(timer->Periph, ENABLE);
		
	/* Enable the TIMx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority			= 0;//timer->Priority;
	NVIC_InitStructure.NVIC_IRQChannelCmd					= ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel						= timer->Interrupt;
	NVIC_Init(&NVIC_InitStructure);
	
}



/**
* @ Function Name : IntTimer_Config
* @ Desc : 
* @ Param : 
* @ Return :
*/
void IntTimer_Config(void)
{
	IntTimer_ParamInit();
	IntTimer_Init();
}

/**
* @ Function Name : IntTimer_ParamInit
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void IntTimer_ParamInit(void)
{
	IntTimer.Timer.Periph = INT_TIMER;
	IntTimer.Timer.Clock = INT_TIMER_CLOCK;
	IntTimer.Timer.Interrupt = INT_TIMER_INTERRUPT;
	IntTimer.Period = INT_TIMER_PERIOD;	
    IntTimer.bLampTimer = FALSE;
    IntTimer.bLampEnable = FALSE;
}

/**
* @ Function Name : IntTimer_Init
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void IntTimer_Init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;	
	uint16_t PrescalerValue = 0;
	
	/* TIMx Deinitialization */
	TIM_DeInit(IntTimer.Timer.Periph);

	/* Enable TIM clock */
	RCC_APB1PeriphClockCmd(IntTimer.Timer.Clock, ENABLE);
	
	/* Compute the prescaler value */
	PrescalerValue = (uint16_t)((SystemCoreClock / 2) / TIM_COUNT_CLOCK) - 1;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = IntTimer.Period;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
	TIM_TimeBaseInit(IntTimer.Timer.Periph, &TIM_TimeBaseStructure);
	
	/* TIMx's pending flags clear */
	TIM_ClearFlag(IntTimer.Timer.Periph, TIM_IT_Update);

	/* TIMx IT enable */
	TIM_ITConfig(IntTimer.Timer.Periph, TIM_IT_Update, ENABLE);

	/* Enable the TIMx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = IntTimer.Timer.Interrupt;
    
	NVIC_Init(&NVIC_InitStructure);
}

/**
* @ Function Name : IntTimer_Delay
* @ Desc : 
* @ Param : 
* @ Return :
*/
void IntTimer_Delay(uint32_t delay)
{
	IntTimer.Delay = delay;
	
	TIM_SetCounter(IntTimer.Timer.Periph, 0);
	TIM_Cmd(IntTimer.Timer.Periph, ENABLE);
}

/**
* @ Function Name : IntTimer_GetStatus
* @ Desc : 
* @ Param : 
* @ Return :
*/
bool IntTimer_GetStatus(void)
{
    bool ret = FALSE;

    if (IntTimer.Delay <= IntTimer.Count)
    {
        TIM_Cmd(IntTimer.Timer.Periph, DISABLE);

        IntTimer.Count = 0;
        ret = TRUE;
    }

    return ret;
}

void IntTimer_Stop(void)
{
	TIM_Cmd(IntTimer.Timer.Periph, DISABLE);
    IntTimer.Count = 0;
}



/**
* @ Function Name : LampTimer_Enable
* @ Desc : 
* @ Param : 
* @ Return :
*/
void LampTimer_Enable(void)
{
    if (CurCaptureMode == CAPTURE_CANCEL)
    {
        Lamp_Control(LAMP_RED, LAMP_DISABLE);
        Lamp_Control(LAMP_GREEN, LAMP_DISABLE);
        Lamp_Control(LAMP_BLUE, LAMP_ENABLE);
    
        IntTimer.bLampTimer = TRUE;
        IntTimer.bLampEnable = TRUE;
        IntTimer_Delay(LAMP_TIMER_PERIOD);
    }
}


/**
* @ Function Name : LampErrorTimer_Enable
* @ Desc : 
* @ Param : 
* @ Return :
*/
void LampErrorTimer_Enable(int8_t Error_Case , int8_t Timer_D)
{
	switch(Error_Case)
	{
		case ERROR_NONE:
			Lamp_Control(LAMP_RED, LAMP_ENABLE);
		    Lamp_Control(LAMP_GREEN, LAMP_ENABLE);
		    Lamp_Control(LAMP_BLUE, LAMP_ENABLE);
			
			IntTimer.bLampTimer = TRUE;
	        IntTimer_Delay(LAMP_TIMER_PERIOD/Timer_D);
			break;
		case ERROR_COLLIMATOR:
			Lamp_Control(LAMP_RED, LAMP_ENABLE);
	        Lamp_Control(LAMP_GREEN, LAMP_ENABLE);
	        Lamp_Control(LAMP_BLUE, LAMP_DISABLE);
	    
	        IntTimer.bLampTimer = TRUE;
	        IntTimer_Delay(LAMP_TIMER_PERIOD/Timer_D);
			break;
		case ERROR_GENERATOR:			
			Lamp_Control(LAMP_RED, LAMP_DISABLE);
			Lamp_Control(LAMP_GREEN, LAMP_ENABLE);
			Lamp_Control(LAMP_BLUE, LAMP_ENABLE);
		
			IntTimer.bLampTimer = TRUE;
			IntTimer_Delay(LAMP_TIMER_PERIOD/Timer_D);
			break;
		case ERROR_MOTOR:			
			Lamp_Control(LAMP_RED, LAMP_ENABLE);
			Lamp_Control(LAMP_GREEN, LAMP_DISABLE);
			Lamp_Control(LAMP_BLUE, LAMP_ENABLE);
		
			IntTimer.bLampTimer = TRUE;
			IntTimer_Delay(LAMP_TIMER_PERIOD/Timer_D);
			break;
		case ERROR_STATUS_START:
			Lamp_Control(LAMP_RED, LAMP_ENABLE);
		    Lamp_Control(LAMP_GREEN, LAMP_DISABLE);
		    Lamp_Control(LAMP_BLUE, LAMP_DISABLE);
			
			IntTimer.bLampTimer = TRUE;
	        IntTimer_Delay(LAMP_TIMER_PERIOD/Timer_D);
			break;
		case ERROR_LAMP_OFF:
			Lamp_Control(LAMP_RED, LAMP_DISABLE);
		    Lamp_Control(LAMP_GREEN, LAMP_DISABLE);
		    Lamp_Control(LAMP_BLUE, LAMP_DISABLE);
			
			IntTimer.bLampTimer = TRUE;
	        IntTimer_Delay(LAMP_TIMER_PERIOD/Timer_D);
			break;
			
	}
    
        
}



/**
* @ Function Name : LampTimer_GetStatus
* @ Desc : 
* @ Param : 
* @ Return :
*/
bool LampTimer_GetStatus(void)
{
	if(CurCaptureMode!=CAPTURE_CANCEL)
	{
		IntTimer.bLampTimer = FALSE;
		IntTimer.Count = 0;
	}
	
    if ((IntTimer.bLampTimer == FALSE) || (IntTimer.Delay <= IntTimer.Count))
    {
        TIM_Cmd(IntTimer.Timer.Periph, DISABLE);

		if(sysInfo.bShowLog==TRUE)
		{
        	//printUart(DBG_MSG_PC, "Delay(%d):Count(%d)", IntTimer.Delay, IntTimer.Count);
		}

        Lamp_Control(LAMP_RED, LAMP_DISABLE);
        Lamp_Control(LAMP_GREEN, LAMP_DISABLE);
        Lamp_Control(LAMP_BLUE, LAMP_DISABLE);

        IntTimer.Count = 0;
        IntTimer.bLampTimer = FALSE;
        IntTimer.bLampEnable = FALSE;

        return TRUE;
    }

    return FALSE;
}

#ifdef USE_TUBE_PPS_TYPE_IO
void IntTimer_Disable(void)
{
    TIM_Cmd(IntTimer.Timer.Periph, DISABLE);
}
#endif /* USE_TUBE_PPS_TYPE_IO */            

/**
* @ Function Name : _delay_tick
* @ Desc : it takes 3 cycles
* @ Param : 
* @ Return :
*/
//__asm void _delay_tick(unsigned long ulCount)
//__asm void _delay_tick(unsigned long ulCount)
//{
//	subs r0, #1
//	bne.n _delay_tick //recursive
//	bx lr
//}
// jehun
static void _delay_tick(unsigned long ulCount)
{
    __asm("    subs    r0, #1\n"
          "    bne.n   _delay_tick\n"
          "    bx      lr");
}
/**
* @ Function Name : delay_us
* @ Desc : delay for microsecond
* @ Param : 
* @ Return :
*/
void delay_us(unsigned long us) 
{ 
	_delay_tick(us * (SystemCoreClock / 3 / 1000000)); 
} 

/**
* @ Function Name : delay_ms
* @ Desc : delay for milisecond
* @ Param : 
* @ Return :
*/
void delay_ms(unsigned long ms) 
{
	delay_us(ms * 1000); 
} 

/**
* @ Function Name : SysTick_Handler
* @ Desc : 
* @ Param : 
* @ Return :
*/
void SysTick_Handler(void)
{
	nSysTicks++;

/*
	if (nSysTicks%300 == 0)
	{
		printUart(DBG_MSG_PC, "nSysTicks:%d", nSysTicks);
	}
*/	
#if 0	
	if (nSysTicks++ > 200)
	{
		SysTick->CTRL = (0 << SYSTICK_CLKSOURCE) | (0 << SYSTICK_ENABLE) | (0 << SYSTICK_TICKINT);
		printUart(DBG_MSG_PC, "nSysTickCnt = %d", nSysTicks);
	}
#endif	
	//printUart(DBG_MSG_PC, "gggg");
}

char Systick_Configuration(void)
{
	uint32_t returnCode;

	returnCode = SysTick_Config(SystemCoreClock / 1000);
	
	if (returnCode != 0) {
		// Error Handling ?
		printUart(DBG_MSG_PC, "Systick_Configuration failed");

		return 1;
	}

	NVIC_SetPriority(SysTick_IRQn, 0);

	return 0;
}

/**
* @ Function Name : SysTick_DelayMs
* @ Desc : 
* @ Param : 
* @ Return :
*/
/*__inline*/ void SysTick_DelayMs(unsigned int ms)
{
	unsigned int nCurTicks = nSysTicks;

	//Setup SysTick Timer for 1 msec interrupts
	if (SysTick_Config(SystemCoreClock / 1000)) 
	{
#if 1//defined(BUILD_TYPE_DEBUG)
		printUart(DBG_MSG_PC, "Software Timer");
#endif /*  BUILD_TYPE_DEBUG */
		
		delay_ms(ms);

		return;
	}
	
	NVIC_SetPriority(SysTick_IRQn, 0);

	if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "delay ms -Start : nCurTicks = %d, nSysTicks = %d", nCurTicks, nSysTicks);
	}

	while( (nSysTicks - nCurTicks) < ms);
	SysTick->CTRL = (0 << SYSTICK_CLKSOURCE) | (0 << SYSTICK_ENABLE) | (0 << SYSTICK_TICKINT);

	if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "delay ms -End : nCurTicks = %d, nSysTicks = %d", nCurTicks, nSysTicks);
	}
}

/**
* @ Function Name : SysTick_DelayUs
* @ Desc : 
* @ Param : 
* @ Return :
*/
/*__inline*/ void SysTick_DelayUs(unsigned int us)
{
	unsigned int nCurTicks = nSysTicks;

	//Setup SysTick Timer for 1 usec interrupts
	if (SysTick_Config(SystemCoreClock / 1000000)) 
	{
#if 1//defined(BUILD_TYPE_DEBUG)
		printUart(DBG_MSG_PC, "Software Timer");
#endif /*  BUILD_TYPE_DEBUG */
		
		delay_us(us);

		return;
	}
	
	NVIC_SetPriority(SysTick_IRQn, 0);

	if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "delay us -Start : nCurTicks = %d, nSysTicks = %d", nCurTicks, nSysTicks);
	}

	while( (nSysTicks - nCurTicks) < us);
	SysTick->CTRL = (0 << SYSTICK_CLKSOURCE) | (0 << SYSTICK_ENABLE) | (0 << SYSTICK_TICKINT);

	if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "delay us -End : nCurTicks = %d, nSysTicks = %d", nCurTicks, nSysTicks);
	}
}

/**
* @ Function Name : vElapsedTimer_Config
* @ Desc : 
* @ Param : 
* @ Return :
*/
void vElapsedTimerConfig(void)
{
	vElapsedTimerParamInit();
	vElapsedTimerInit();
}

/**
* @ Function Name : vElapsedTimerParamInit
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void vElapsedTimerParamInit(void)
{
	typElapsedTimer.Timer.Periph = ELAPSED_TIM;
	typElapsedTimer.Timer.Clock = ELAPSED_TIM_CLOCK;
	typElapsedTimer.Timer.Interrupt = ELAPSED_TIM_INTERRUPT;
	typElapsedTimer.Period = ELAPSED_TIM_PERIOD;	
}

/**
* @ Function Name : ElapsedTimer_Init
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void vElapsedTimerInit(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;	
	uint16_t PrescalerValue = 0;
	
	/* TIMx Deinitialization */
	TIM_DeInit(typElapsedTimer.Timer.Periph);

	/* Enable TIM clock */
	RCC_APB1PeriphClockCmd(typElapsedTimer.Timer.Clock, ENABLE);
	
	/* Compute the prescaler value */
	PrescalerValue = (uint16_t)((SystemCoreClock / 2) / TIM_COUNT_CLOCK) - 1;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = typElapsedTimer.Period;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
	TIM_TimeBaseInit(typElapsedTimer.Timer.Periph, &TIM_TimeBaseStructure);
	
	/* TIMx's pending flags clear */
	TIM_ClearFlag(typElapsedTimer.Timer.Periph, TIM_IT_Update);

	/* TIMx IT enable */
	TIM_ITConfig(typElapsedTimer.Timer.Periph, TIM_IT_Update, ENABLE);

	/* Enable the TIMx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	= 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = typElapsedTimer.Timer.Interrupt;
    
	NVIC_Init(&NVIC_InitStructure);
}

/**
* @ Function Name : nGetCurrentMilliSec
* @ Desc :
* @ Param : 
* @ Return : millisec for paramer of nElapsedMilliSec()
*/
uint32_t vSetCurrentMilliSec(void)
{
	return typElapsedTimer.startMs = typElapsedTimer.endMs;
}

/**
* @ Function Name : nElapsedMilliSec
* @ Desc : return elapsed time from parameter time 
*          unsigned long : 0~4,294,967,295mSec = about 49.7day
* @ Param : 
* @ Return :
*/
uint32_t nElapsedMilliSec(uint32_t ulStartTime)
{
	uint32_t ulEndTime = typElapsedTimer.endMs;
	if(ulStartTime <= ulEndTime) {
		return ( ulEndTime - ulStartTime );
	} else {
		return ( (kMax_uint32_t - ulStartTime) + ulEndTime );
	}
}

/**
* @ Function Name : ElapseTimerOnOff
* @ Desc : 
* @ Param : 
* @ Return :
*/
void ElapseTimerOnOff(bool bEnable)
{
#ifndef USE_AGING_MODE
    if (bEnable == TRUE) {
       TIM_SetCounter(typElapsedTimer.Timer.Periph, 0);
       TIM_Cmd(typElapsedTimer.Timer.Periph, ENABLE);
    } else {
       TIM_Cmd(typElapsedTimer.Timer.Periph, DISABLE);
	   
	    typElapsedTimer.startMs = 0;
        typElapsedTimer.endMs = 0;
    }
#endif /* not USE_AGING_MODE */
}

#ifdef USE_AGING_MODE
static uint32_t SysGetTime(void);
static uint32_t g_nTimeStart, g_nTimeEnd;

/**
* @ Function Name : SysSetTime
* @ Desc : 
* @ Param : 
* @ Return :
*/
void SysSetTime(_Bool bEnable)
{
    if (bEnable == TRUE) 
	{
		typElapsedTimer.startMs = 0;
		typElapsedTimer.endMs = 0;

       TIM_SetCounter(typElapsedTimer.Timer.Periph, 0);
       TIM_Cmd(typElapsedTimer.Timer.Periph, ENABLE);
    } 
	else 
	{
       TIM_Cmd(typElapsedTimer.Timer.Periph, DISABLE);	   
    }
}

/**
* @ Function Name : SysGetTime
* @ Desc : 
* @ Param : 
* @ Return :
*/
static uint32_t SysGetTime(void)
{
	return typElapsedTimer.endMs;
}

/**
* @ Function Name : SysTime_MesureStart
* @ Desc : 
* @ Param : 
* @ Return :
*/
void SysTime_MesureStart(const char *pFuncName, int nLine)
{
	g_nTimeStart = SysGetTime();

	printUart(DBG_MSG_PC, "[%s (%d) Time Mesure Start ==================]", pFuncName, nLine);
}

/**
* @ Function Name : SysTime_MesureEnd
* @ Desc : 
* @ Param : 
* @ Return :
*/
void SysTime_MesureEnd(const char *pFuncName, int nLine)
{
	uint32_t nTimeDiff;
	
	g_nTimeEnd = SysGetTime();
	nTimeDiff = g_nTimeEnd - g_nTimeStart;

	printUart(DBG_MSG_PC, "[%s (%d) Time Mesure End ::%d msec ========]", pFuncName, nLine, nTimeDiff);
}
#endif /* USE_AGING_MODE */

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/

