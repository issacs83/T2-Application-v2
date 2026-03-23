/*
*******************************************************************************
* tube.c :
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
#include "tube.h"
#include "timer.h"
#include "misc1.h"
#include "system.h"
#include "serial.h"
#ifdef BUILD_TYPE_DEBUG
#include "serial.h"
#endif /* BUILD_TYPE_DEBUG */


/* Private typedef --------------------------------------------------- */
/* Private define ---------------------------------------------------- */
#define	TUBE									TIM14 // Tube PPS
#define	TUBE_CLOCK								RCC_APB1Periph_TIM14
#define	TUBE_GPIO								GPIOA
#define	TUBE_GPIO_CLOCK							RCC_AHB1Periph_GPIOA
#define	TUBE_GPIO_PIN							GPIO_Pin_7
#define	TUBE_GPIO_PinSource						GPIO_PinSource7
#define	TUBE_GPIO_AF							GPIO_AF_TIM14
#define	TUBE_INIT_STATUS						TIM_OCPolarity_High	// OC Port initial state is Low
#define	TUBE_INTERRUPT							TIM8_TRG_COM_TIM14_IRQn
#define	TUBE_CHANNEL							TIM_Channel_1

#define	TUBE_IO_PORT_GPIO_C_CLOCK				RCC_AHB1Periph_GPIOC
#define	TUBE_IO_PORT_GPIO_C						GPIOC
#define	TUBE_IO_PORT_GPIO_MODE					GPIO_Mode_OUT

//#define	TUBE_READY_GPIO							GPIOC
//#define	TUBE_READY_GPIO_CLOCK					RCC_AHB1Periph_GPIOC
#define	TUBE_READY_GPIO_PIN						GPIO_Pin_4
//#define	TUBE_READY_GPIO_MODE					GPIO_Mode_OUT

//#define	TUBE_EXPOSURE_GPIO						GPIOC
//#define	TUBE_EXPOSURE_GPIO_CLOCK				RCC_AHB1Periph_GPIOC
#define	TUBE_EXPOSURE_GPIO_PIN					GPIO_Pin_5
//#define	TUBE_EXPOSURE_GPIO_MODE					GPIO_Mode_OUT

#ifdef USE_TUBE_PPS_TYPE_IO
#define	TUBE_PPS_GPIO							GPIOA
#define	TUBE_PPS_GPIO_CLOCK						RCC_AHB1Periph_GPIOA
#define	TUBE_PPS_GPIO_PIN						GPIO_Pin_7
#define	TUBE_PPS_GPIO_MODE						GPIO_Mode_OUT
#endif /* USE_TUBE_PPS_TYPE_IO */

#define	SENSOR_READOUT_MS				35.0
#define	SENSOR_DELAY_MS					1

#if !defined(USE_CT_XRAY_PULSED_MODE)
#define	TUBE_XRAY_ON_ms					15
#endif /* not USE_CT_XRAY_PULSED_MODE */
#define	TUBE_XRAY_OFF_ms				SENSOR_READOUT_MS + (2 * SENSOR_DELAY_MS)
#define	TUBE_XRAY_DELAY_ms				SENSOR_READOUT_MS + SENSOR_DELAY_MS


/* Private macro ---------------------------------------------------- */
/* Private variables -------------------------------------------------- */
Tube_Typedef Tube;


/* Private function prototypes ------------------------------------------*/
static void Tube_ParamInit(void);


/* Private functions -------------------------------------------------- */

/**
* @ Function Name : Tube_Config
* @ Desc : 
* @ Param : 
* @ Return :
*/
void Tube_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	/* Tube Parameter Init */
	Tube_ParamInit();	

	/* Enable GPIOx clock */
	RCC_AHB1PeriphClockCmd(TUBE_IO_PORT_GPIO_C_CLOCK, ENABLE);
#ifdef USE_TUBE_PPS_TYPE_IO
	RCC_AHB1PeriphClockCmd(TUBE_PPS_GPIO_CLOCK, ENABLE);
#endif /* USE_TUBE_PPS_TYPE_IO */
	
	/* Tube Port Configuration */
	GPIO_InitStructure.GPIO_Pin = TUBE_READY_GPIO_PIN |TUBE_EXPOSURE_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = TUBE_IO_PORT_GPIO_MODE;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(TUBE_IO_PORT_GPIO_C, &GPIO_InitStructure);

	Tube_CtrlReady(TUBE_READY_DISABLE);
	Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);

#ifdef USE_TUBE_PPS_TYPE_IO
	GPIO_InitStructure.GPIO_Pin = TUBE_PPS_GPIO_PIN;
	GPIO_Init(TUBE_PPS_GPIO, &GPIO_InitStructure);
#endif /* USE_TUBE_PPS_TYPE_IO */
	
#ifndef USE_TUBE_PPS_TYPE_IO
	/* TIMx Configuration */	
	Timer_Config(&Tube.Timer);
#endif /* not USE_TUBE_PPS_TYPE_IO */	
	if(sysInfo.bShowLog==TRUE)
	{
    	printUart(DBG_MSG_PC, "Tube Config Done");
	}
}

/**
* @ Function Name : Tube_ParamInit
* @ Desc : 
* @ Param : 
* @ Return :
*/
void Tube_ParamInit(void)
{
#ifndef USE_TUBE_PPS_TYPE_IO
	/* Tube Timer Parameter Init */                               
	Tube.Timer.Periph			= TUBE;      
	Tube.Timer.Clock			= TUBE_CLOCK;    
	Tube.Timer.Gpio				= TUBE_GPIO; 
	Tube.Timer.GpioClock		= TUBE_GPIO_CLOCK;
	Tube.Timer.GpioPin			= TUBE_GPIO_PIN; 
	Tube.Timer.GpioPinsource	= TUBE_GPIO_PinSource;
	Tube.Timer.GpioAf			= TUBE_GPIO_AF;
    Tube.Timer.InitStatus		= TIM_OCPolarity_Low; // Tube X-Ray Off : Low (Active)
	//Tube.Timer.InitStatus		= TUBE_INIT_STATUS;
	Tube.Timer.Interrupt		= TUBE_INTERRUPT;
	Tube.Timer.Source			= TUBE_SOURCE;
	Tube.Timer.Channel			= TUBE_CHANNEL;	
#endif /* not USE_TUBE_PPS_TYPE_IO */    
	
	Tube.Param.XonTime			= TUBE_XRAY_ON_ms;
	Tube.Param.XoffTime			= TUBE_XRAY_OFF_ms;
	Tube.Param.DelayTime		= TUBE_XRAY_DELAY_ms;

//�씠�긽�빐?? �궡�렣蹂� 寃�..
#ifndef USE_TUBE_PPS_TYPE_IO
	Tube.Param.XonTimeCcr		= ms_To_CCR(Tube.Param.XonTime);
	Tube.Param.XoffTimeCcr		= ms_To_CCR(Tube.Param.XoffTime);
	Tube.Param.DelayTimeCcr		= ms_To_CCR(Tube.Param.DelayTime);
	Tube.Timer.InitCCR			= Tube.Param.DelayTimeCcr;
	Tube.Update					= TRUE;
#endif /* not USE_TUBE_PPS_TYPE_IO */    

    Tube.bXrayOnOff = TRUE;
	Tube.bXrayFrameOnOff = RESET;
}

#ifdef USE_TUBE_PPS_TYPE_IO
/**
* @ Function Name : Tube_CtrlPps
* @ Desc : 
* @ Param : Bit_SET(OFF), Bit_RESET(ON)
* @ Return :
*/
void Tube_CtrlPps(BitAction bit)
{
    GPIO_WriteBit(TUBE_PPS_GPIO, TUBE_PPS_GPIO_PIN, bit);    
}
#else /* not USE_TUBE_PPS_TYPE_IO */
/**
* @ Function Name : Tube_Start
* @ Desc : 
* @ Param : 
* @ Return :
*/
void Tube_Start(void)
{
	TIM_SetCounter(Tube.Timer.Periph, 0);
	
	Tube.Timer.InitCCR = ms_To_CCR(Tube.Param.DelayTime);
	
	switch(Tube.Timer.Channel)
	{
	case TIM_Channel_1:
		TIM_SetCompare1(Tube.Timer.Periph, Tube.Timer.InitCCR + 0.5);
		break;
        
	case TIM_Channel_2:
		TIM_SetCompare2(Tube.Timer.Periph, Tube.Timer.InitCCR + 0.5);
		break;
        
	case TIM_Channel_3:
		TIM_SetCompare3(Tube.Timer.Periph, Tube.Timer.InitCCR + 0.5);
		break;
        
	case TIM_Channel_4:
		TIM_SetCompare4(Tube.Timer.Periph, Tube.Timer.InitCCR + 0.5);
		break;
        
	default:
		// error
		break;
	}
		
	TIM_Cmd(Tube.Timer.Periph, ENABLE);
}

/**
* @ Function Name : Tube_Stop
* @ Desc : 
* @ Param : 
* @ Return :
*/
void Tube_Stop(void)
{
	TIM_Cmd(Tube.Timer.Periph, DISABLE);
}
#endif /* USE_TUBE_PPS_TYPE_IO */

/**
* @ Function Name : Tube_CtrlReady
* @ Desc : 
* @ Param : 
* @ Return :
*/
void Tube_CtrlReady(BitAction bit)
{
    if (Tube.bXrayOnOff) {
    	GPIO_WriteBit(TUBE_IO_PORT_GPIO_C, TUBE_READY_GPIO_PIN, bit);
    }
}

/**
* @ Function Name : Tube_CtrlExposure
* @ Desc : 
* @ Param : 
* @ Return :
*/
void Tube_CtrlExposure(BitAction bit)
{
    if (bit == TUBE_EXPOSURE_ENABLE) {
		if(Tube.bXrayOnOff)
		{
			Lamp_Control(LAMP_RED, LAMP_ENABLE);
	        Lamp_Control(LAMP_GREEN, LAMP_ENABLE);
	        Lamp_Control(LAMP_BLUE, LAMP_DISABLE);
		}    
    } else {
        Lamp_Control(LAMP_RED, LAMP_DISABLE);
        Lamp_Control(LAMP_GREEN, LAMP_DISABLE);
        Lamp_Control(LAMP_BLUE, LAMP_DISABLE);    
    }

    if (Tube.bXrayOnOff) {
    	GPIO_WriteBit(TUBE_IO_PORT_GPIO_C, TUBE_EXPOSURE_GPIO_PIN, bit);
    }
}


/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/

