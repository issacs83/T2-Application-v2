/*
*******************************************************************************
* sensor.c :
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
#include "sensor.h"
#include "timer.h"
#include "serial.h"
#if defined(USE_CT_XRAY_PULSED_MODE)
#include "tube.h"
#endif /* USE_CT_XRAY_PULSED_MODE */


/* Private typedef --------------------------------------------------- */
/* Private define ---------------------------------------------------- */
#define	SENSOR_P								TIM12
#define	SENSOR_P_CLOCK							RCC_APB1Periph_TIM12
#define	SENSOR_P_GPIO							GPIOB
#define	SENSOR_P_GPIO_CLOCK						RCC_AHB1Periph_GPIOB
#define	SENSOR_P_GPIO_PIN						GPIO_Pin_14
#define	SENSOR_P_GPIO_PinSource					GPIO_PinSource14
#define	SENSOR_P_GPIO_AF						GPIO_AF_TIM12
#define	SENSOR_P_INIT_STATUS					TIM_OCPolarity_High
#define	SENSOR_P_INTERRUPT						TIM8_BRK_TIM12_IRQn
#define	SENSOR_P_CHANNEL						TIM_Channel_1

#define	SENSOR_P_INPUT_GPIO						GPIOB
#define	SENSOR_P_INPUT_GPIO_CLOCK				RCC_AHB1Periph_GPIOB
#define	SENSOR_P_INPUT_GPIO_PIN					GPIO_Pin_15
#define	SENSOR_P_INPUT_GPIO_MODE				GPIO_Mode_IN
#define	SENSOR_P_INPUT_ON						Bit_SET
#define	SENSOR_P_INPUT_OFF						Bit_RESET

#define	VEN_GPIO_PORT							GPIOE
#define	VEN_GPIO_CLOCK			        		RCC_AHB1Periph_GPIOE
#define	VEN_GPIO_PIN				    		GPIO_Pin_6
#define	VEN_GPIO_MODE			       			GPIO_Mode_OUT
#define	VEN_GPIO_ENABLE			        		Bit_RESET
#define	VEN_GPIO_DISABLE			    		Bit_SET

#define	SENSOR_P_USERSYNC_HIGH_ms			 	1.0
#define	SENSOR_P_READOUT_ms 					3.8
#define	SENSOR_P_USERSYNC_LOW_ms	 			SENSOR_P_READOUT_ms


#define	SENSOR_S_OUTPUT_GPIO					GPIOE
#define	SENSOR_S_OUTPUT_GPIO_CLOCK				RCC_AHB1Periph_GPIOE
#define	SENSOR_S_OUTPUT_GPIO_PIN				GPIO_Pin_8
#define	SENSOR_S_OUTPUT_GPIO_MODE				GPIO_Mode_OUT


#define	SENSOR_S_POWER_GPIO_PORT				GPIOA
#define	SENSOR_S_POWER_GPIO_CLOCK			    RCC_AHB1Periph_GPIOA
#define	SENSOR_S_POWER_GPIO_PIN				    GPIO_Pin_4
#define	SENSOR_S_POWER_GPIO_MODE			    GPIO_Mode_OUT
#define	SENSOR_S_POWER_GPIO_ENABLE			    Bit_RESET
#define	SENSOR_S_POWER_GPIO_DISABLE			  	Bit_SET

#if defined(USE_CT_XRAY_PULSED_MODE)
#define	SENSOR_C								TIM12
#define	SENSOR_C_CLOCK							RCC_APB1Periph_TIM12
#define	SENSOR_C_GPIO							GPIOB
#define	SENSOR_C_GPIO_CLOCK						RCC_AHB1Periph_GPIOB
#define	SENSOR_C_GPIO_PIN						GPIO_Pin_14
#define	SENSOR_C_GPIO_PinSource					GPIO_PinSource14
#define	SENSOR_C_GPIO_AF						GPIO_AF_TIM12
#define	SENSOR_C_INIT_STATUS					TIM_OCPolarity_High
#define	SENSOR_C_INTERRUPT						TIM8_BRK_TIM12_IRQn
//#define	SENSOR_C_ISR							TIM8_BRK_TIM12_IRQHandler
#define	SENSOR_C_SOURCE							TIM_IT_CC1
#define	SENSOR_C_CHANNEL						TIM_Channel_1
//#define	SENSOR_C_GetCapture						TIM_GetCapture1
//#define	SENSOR_C_SetCompare						TIM_SetCompare1

#define	SENSOR_C_USERSYNC_HIGH_ms			4.5//6.0
#define	SENSOR_C_READOUT_ms					35.0
#define	SENSOR_C_DELAY_ms					1
#define	SENSOR_C_USERSYNC_LOW_ms			SENSOR_C_READOUT_ms - SENSOR_C_USERSYNC_HIGH_ms + TUBE_XRAY_ON_ms + (2 * SENSOR_C_DELAY_ms)
#endif /* USE_CT_XRAY_PULSED_MODE */

#define	SENSOR_P_3D_USERSYNC_HIGH_MS			 	4.0


/* Private macro ---------------------------------------------------- */
/* Private variables -------------------------------------------------- */
PanoSensor_Typedef Sensor_P;
#if defined(USE_CT_XRAY_PULSED_MODE)
CtSensor_Typedef Sensor_C;
#endif /* USE_CT_XRAY_PULSED_MODE */


/* Private function prototypes ------------------------------------------*/
static void PanoSensor_InitParam(void);
#if defined(USE_CT_XRAY_PULSED_MODE)
static void CtSensor_InitParam(void);
#endif /* USE_CT_XRAY_PULSED_MODE */


/* Private functions -------------------------------------------------- */
/**
* @ Function Name : PanoSensor_PowerConfig
* @ Desc : 
* @ Param : 
* @ Return :
*/
void PanoSensor_PowerConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIOx clock */
	RCC_AHB1PeriphClockCmd(VEN_GPIO_CLOCK, ENABLE);
	
	/* GPIOx Configuration */
	GPIO_InitStructure.GPIO_Pin = VEN_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = VEN_GPIO_MODE;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(VEN_GPIO_PORT, &GPIO_InitStructure);

	// 평상시에는 센서 전원 끈다 => 일단 보류
    //PanoSensor_PowerControl(FALSE);	
    PanoSensor_PowerControl(TRUE);	

	if(sysInfo.bShowLog==TRUE)
	{
    	printUart(DBG_MSG_PC, "PanoSensor_PowerConfig Done");
	}
}

/**
* @ Function Name : PanoSensor_PowerControl
* @ Desc : 
* @ Param : 
* @ Return :
*/
void PanoSensor_PowerControl(char bEnable)
{
	if(sysInfo.bShowLog==TRUE)
	{
    	printUart(DBG_MSG_PC, "PanoSensor_PowerControl : %d", bEnable);
	}

    if (bEnable)
    	GPIO_WriteBit(VEN_GPIO_PORT, VEN_GPIO_PIN, VEN_GPIO_ENABLE);
    else
    	GPIO_WriteBit(VEN_GPIO_PORT, VEN_GPIO_PIN, VEN_GPIO_DISABLE);

    Sensor_P.PowerStatus = bEnable;
}

/**
* @ Function Name : PanoSensor_Config
* @ Desc : 
* @ Param : 
* @ Return :
*/
void PanoSensor_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Sensor Parameter Init */
    PanoSensor_InitParam();

	/* Enable GPIOx clock */
	RCC_AHB1PeriphClockCmd(SENSOR_P_INPUT_GPIO_CLOCK, ENABLE);
	
	/* GPIOx Configuration */
	GPIO_InitStructure.GPIO_Pin = SENSOR_P_INPUT_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = SENSOR_P_INPUT_GPIO_MODE;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SENSOR_P_INPUT_GPIO, &GPIO_InitStructure);

    /* TIMx Configuration */	
    Timer_Config(&Sensor_P.Timer);
}

/**
* @ Function Name : PanoSensor_InitParam
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void PanoSensor_InitParam(void)
{
    Sensor_P.Timer.Periph 		= SENSOR_P;      
    Sensor_P.Timer.Clock 		= SENSOR_P_CLOCK;
    Sensor_P.Timer.Gpio 		= SENSOR_P_GPIO; 
    Sensor_P.Timer.GpioClock 	= SENSOR_P_GPIO_CLOCK;
    Sensor_P.Timer.GpioPin 		= SENSOR_P_GPIO_PIN; 
    Sensor_P.Timer.GpioPinsource = SENSOR_P_GPIO_PinSource;
    Sensor_P.Timer.GpioAf 		= SENSOR_P_GPIO_AF;
    Sensor_P.Timer.InitStatus 	= SENSOR_P_INIT_STATUS;
    Sensor_P.Timer.Interrupt 	= SENSOR_P_INTERRUPT;
    Sensor_P.Timer.Source 		= SENSOR_P_SOURCE;
    Sensor_P.Timer.Channel 		= SENSOR_P_CHANNEL;

    Sensor_P.Trigger.SyncHighTime	= SENSOR_P_USERSYNC_HIGH_ms;
    Sensor_P.Trigger.SyncLowTime 	= SENSOR_P_USERSYNC_LOW_ms;

    Sensor_P.Trigger.SyncHighTimeCcr 	= ms_To_CCR(Sensor_P.Trigger.SyncHighTime);
    Sensor_P.Trigger.VariableLowTimeCcr	= Sensor_P.CaptureCcr[Sensor_P.Index];
    
    Sensor_P.Timer.InitCCR = Sensor_P.Trigger.VariableLowTimeCcr;

    Sensor_P.Update = FALSE;
}

/**
* @ Function Name : PanoSensor_Start
* @ Desc : 
* @ Param : 
* @ Return :
*/
void PanoSensor_Start(void)
{
    TIM_SetCounter(Sensor_P.Timer.Periph, 0);

    Sensor_P.Trigger.VariableLowTimeCcr = Sensor_P.CaptureCcr[Sensor_P.Index];
    Sensor_P.Timer.InitCCR = Sensor_P.Trigger.VariableLowTimeCcr;

    switch (Sensor_P.Timer.Channel) 
	{
    case TIM_Channel_1:
        TIM_SetCompare1(Sensor_P.Timer.Periph, Sensor_P.Timer.InitCCR * 2 + 0.5);
        break;
    case TIM_Channel_2:
        TIM_SetCompare2(Sensor_P.Timer.Periph, Sensor_P.Timer.InitCCR * 2 + 0.5);
        break;
    case TIM_Channel_3:
        TIM_SetCompare3(Sensor_P.Timer.Periph, Sensor_P.Timer.InitCCR * 2 + 0.5);
        break;
    case TIM_Channel_4:
        TIM_SetCompare4(Sensor_P.Timer.Periph, Sensor_P.Timer.InitCCR * 2 + 0.5);
        break;
    default:
        // error
        break;
    }

    TIM_Cmd(Sensor_P.Timer.Periph, ENABLE);
}

/**
* @ Function Name : PanoSensor_Stop
* @ Desc : 
* @ Param : 
* @ Return :
*/
void PanoSensor_Stop(void)
{
    TIM_Cmd(Sensor_P.Timer.Periph, DISABLE);

    Sensor_P.Start = FALSE;
    Sensor_P.Index = 0;
    Sensor_P.Count = 0;
    Sensor_P.Update = FALSE;
}

/**
* @ Function Name : PanoSensor_CheckInput
* @ Desc : 
* @ Param : 
* @ Return :
*/
BitAction PanoSensor_CheckInput(void)
{
	return (BitAction)GPIO_ReadInputDataBit(SENSOR_P_INPUT_GPIO, SENSOR_P_INPUT_GPIO_PIN);
}

/**
* @ Function Name : PanoSensor_SetUserSync
* @ Desc : 
* @ Param : 
* @ Return :
*/
void PanoSensor_SetUserSync(void)
{
	if (PanoParam.Mode == PANO_MODE_3D) {
		if(sysInfo.bShowLog==TRUE)
		{
			printUart(DBG_MSG_PC, "UserSync Setup :: ""3D Panorama");
		}
		Sensor_P.Trigger.SyncHighTime = SENSOR_P_3D_USERSYNC_HIGH_MS;	
	} else {
		if(sysInfo.bShowLog==TRUE)
		{
			printUart(DBG_MSG_PC, "UserSync Setup :: Normal Panorama");
		}
		Sensor_P.Trigger.SyncHighTime = SENSOR_P_USERSYNC_HIGH_ms;	
	}
	Sensor_P.Trigger.SyncHighTimeCcr = ms_To_CCR(Sensor_P.Trigger.SyncHighTime);
}

/**
* @ Function Name : ScanSensor_PowerConfig
* @ Desc : 
* @ Param : 
* @ Return :
*/
void ScanSensor_PowerConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	
	

	/* Enable GPIOx clock */
	RCC_AHB1PeriphClockCmd(SENSOR_S_POWER_GPIO_CLOCK, ENABLE);
	
	/* GPIOx Configuration */
	GPIO_InitStructure.GPIO_Pin = SENSOR_S_POWER_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = SENSOR_S_POWER_GPIO_MODE;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SENSOR_S_POWER_GPIO_PORT, &GPIO_InitStructure);

	// 평상시에는 센서 전원 끈다 => 일단 보류
    //ScanSensor_PowerControl(FALSE);	
    ScanSensor_PowerControl(TRUE);

	if(sysInfo.bShowLog==TRUE)
	{
    	printUart(DBG_MSG_PC, "ScanSensor_PowerConfig Done");
	}	
}

/**
* @ Function Name : ScanSensor_PowerControl
* @ Desc : 
* @ Param : 
* @ Return :
*/
void ScanSensor_PowerControl(char bEnable)
{
	if(sysInfo.bShowLog==TRUE)
	{
    	printUart(DBG_MSG_PC, "ScanSensor_PowerControl : %d", bEnable);
	}

    if (bEnable)
    	GPIO_WriteBit(SENSOR_S_POWER_GPIO_PORT, SENSOR_S_POWER_GPIO_PIN, SENSOR_S_POWER_GPIO_ENABLE);
    else
    	GPIO_WriteBit(SENSOR_S_POWER_GPIO_PORT, SENSOR_S_POWER_GPIO_PIN, SENSOR_S_POWER_GPIO_DISABLE);

    //Sensor_P.PowerStatus = bEnable;
}


/**
* @ Function Name : ScanSensor_Config
* @ Desc : 
* @ Param : 
* @ Return :
*/
void ScanSensor_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIOx clock */
	RCC_AHB1PeriphClockCmd(SENSOR_S_OUTPUT_GPIO_CLOCK, ENABLE);
	
	/* GPIOx Configuration */
	GPIO_InitStructure.GPIO_Pin = SENSOR_S_OUTPUT_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = SENSOR_S_OUTPUT_GPIO_MODE;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(SENSOR_S_OUTPUT_GPIO, &GPIO_InitStructure);
	
	/* Sensor Port Init */
	ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE);   

	if(sysInfo.bShowLog==TRUE)
	{
    	printUart(DBG_MSG_PC, "ScanSensor_Config Done");
	}
}

/**
* @ Function Name : ScanSensor_CtrlOutput
* @ Desc : 
* @ Param : 
* @ Return :
*/
void ScanSensor_CtrlOutput(BitAction bit)
{
	GPIO_WriteBit(SENSOR_S_OUTPUT_GPIO, SENSOR_S_OUTPUT_GPIO_PIN, bit);
}

#if defined(USE_CT_XRAY_PULSED_MODE)
/**
* @ Function Name : CtSensor_Config
* @ Desc : 
* @ Param : 
* @ Return :
*/
void CtSensor_Config(void)
{
	/* Sensor Parameter Init */
	CtSensor_InitParam();

	/* TIMx Configuration */	
	Timer_Config(&Sensor_C.Timer);
}

/**
* @ Function Name : CtSensor_InitParam
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void CtSensor_InitParam(void)
{
	/* CT Varian Sensor Timer Parameter Init */
	Sensor_C.Timer.Periph	 	= SENSOR_C;      
	Sensor_C.Timer.Clock 		= SENSOR_C_CLOCK;
	Sensor_C.Timer.Gpio			= SENSOR_C_GPIO; 
	Sensor_C.Timer.GpioClock 	= SENSOR_C_GPIO_CLOCK;
	Sensor_C.Timer.GpioPin 		= SENSOR_C_GPIO_PIN; 
	Sensor_C.Timer.GpioPinsource = SENSOR_C_GPIO_PinSource;
	Sensor_C.Timer.GpioAf 		= SENSOR_C_GPIO_AF;
	Sensor_C.Timer.InitStatus 	= SENSOR_C_INIT_STATUS;
	Sensor_C.Timer.Interrupt 	= SENSOR_C_INTERRUPT;
	Sensor_C.Timer.Source	 	= SENSOR_C_SOURCE;
	Sensor_C.Timer.Channel 		= SENSOR_C_CHANNEL;

	Sensor_C.Param.SyncHighTime	= SENSOR_C_USERSYNC_HIGH_ms;
	Sensor_C.Param.SyncLowTime	= SENSOR_C_USERSYNC_LOW_ms;

	Sensor_C.Param.SyncHighTimeCcr	= ms_To_CCR(Sensor_C.Param.SyncHighTime);
	Sensor_C.Param.SyncLowTimeCcr	= ms_To_CCR(Sensor_C.Param.SyncLowTime);
	
	Sensor_C.Timer.InitCCR = Sensor_C.Param.SyncLowTimeCcr;
	
	Sensor_C.Update = FALSE;

    Sensor_C.bSetFrameCount = FALSE;
    Sensor_C.nFrameCount = 0;
}

/**
* @ Function Name : CtSensor_Start
* @ Desc : 
* @ Param : 
* @ Return :
*/
void CtSensor_Start(void)
{
	TIM_SetCounter(Sensor_C.Timer.Periph, 0);
	
	Sensor_C.Timer.InitCCR = ms_To_CCR(Sensor_C.Param.SyncLowTimeCcr);
	
	switch(Sensor_C.Timer.Channel)
	{
	case TIM_Channel_1:
		TIM_SetCompare1(Sensor_C.Timer.Periph, Sensor_C.Timer.InitCCR + 0.5);
		break;
        
	case TIM_Channel_2:
		TIM_SetCompare2(Sensor_C.Timer.Periph, Sensor_C.Timer.InitCCR + 0.5);
		break;
        
	case TIM_Channel_3:
		TIM_SetCompare3(Sensor_C.Timer.Periph, Sensor_C.Timer.InitCCR + 0.5);
		break;
        
	case TIM_Channel_4:
		TIM_SetCompare4(Sensor_C.Timer.Periph, Sensor_C.Timer.InitCCR + 0.5);
		break;
        
	default:
		// error
		break;
	}
		
	TIM_Cmd(Sensor_C.Timer.Periph, ENABLE);
}

/**
* @ Function Name : CtSensor_Stop
* @ Desc : 
* @ Param : 
* @ Return :
*/
void CtSensor_Stop(void)
{
	Sensor_C.Update = FALSE;
    Sensor_C.bSetFrameCount = FALSE;
    Sensor_C.nFrameCount = 0;

	TIM_Cmd(Sensor_C.Timer.Periph, DISABLE);
}
#endif /* USE_CT_XRAY_PULSED_MODE */

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/

