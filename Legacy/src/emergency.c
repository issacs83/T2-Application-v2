/*
*******************************************************************************
* emergency.c :
*******************************************************************************
* Copyright (C) 2014-2016 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date : 
* @ Brief :
*
* @ Revision History :
*       1) initial creation. ------------------------------------- 2016-06-14
*******************************************************************************
*/

/* Include files ------------------------------------------------------ */
#include "stm32f2xx_syscfg.h"
#include "extern.h"
#include "emergency.h"
#include "system.h"
#include "tube.h"
#include "serial.h"
#include "misc1.h"
#include "Error_code.h"


/* Private typedef --------------------------------------------------- */
/* Private define ---------------------------------------------------- */


/* Private macro ---------------------------------------------------- */
/* Private variables -------------------------------------------------- */


/* Private function prototypes ------------------------------------------*/


/* Private functions -------------------------------------------------- */

/**
* @ Function Name : EMG_Config
* @ Desc : 
* @ Param : 
* @ Return :
*/
void EMG_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(EMG_SW_GPIO_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_InitStructure.GPIO_Mode = EMG_SW_GPIO_MODE;
    GPIO_InitStructure.GPIO_PuPd = EMG_SW_GPIO_PUPD;
    GPIO_InitStructure.GPIO_Speed = EMG_SW_GPIO_SPEED;
    GPIO_InitStructure.GPIO_Pin = EMG_SW_GPIO_PIN;
    GPIO_Init(EMG_SW_GPIO_PORT, &GPIO_InitStructure);
    
    /* Connect Button EXTI Line to Button GPIO Pin */
    SYSCFG_EXTILineConfig(EMG_SW_GPIO_PORT_SOURCE, EMG_SW_GPIO_PIN_SOURCE);

    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = EMG_SW_GPIO_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EMG_SW_GPIO_EXTI_MODE;
    //EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
    EXTI_InitStructure.EXTI_Trigger = EMG_SW_GPIO_EXTI_TRIGGER;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure); 

	if(sysInfo.bShowLog==TRUE)
	{
   		printUart(DBG_MSG_PC, "EMG_Config Done");
	} 
}

/**
* @ Function Name : EMG_ProcessSwitch
* @ Desc : 
* @ Param : 
* @ Return :
*/
char EMG_ProcessSwitch(void)
{
//    unsigned char released = (char)GPIO_ReadInputDataBit(EMG_SW_GPIO_PORT, EMG_SW_GPIO_PIN);
	unsigned char released = GPIO_ReadInputDataBit(EMG_SW_GPIO_PORT, EMG_SW_GPIO_PIN);
//    char release = HAL_GPIO_ReadPin(EMG_SW_GPIO_PORT, EMG_SW_GPIO_PIN);

    // 20200729
    //
//    delay_ms(300);
//    released = (char)GPIO_ReadInputDataBit(EMG_SW_GPIO_PORT, EMG_SW_GPIO_PIN);

	static char old_released = 0xff;
	
    if (!released) 
	{ // Pressed
		
		if(old_released!=released)
		{
			printUart(DBG_MSG_PC, "EMG Switch Pressed!!");
			old_released=released;
			UART_SendMessage(DBG_MSG_PC, EMERGENCY_SWITCH_ON);
#ifdef USE_TABLET_PC
			UART_SendMessage(DBG_MSG_TABLET, EMERGENCY_SWITCH_ON);
#endif /* USE_TABLET_PC */
		}  
		
    	Tube_CtrlReady(TUBE_READY_DISABLE);
        Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);

        Lamp_Control(LAMP_RED, LAMP_ENABLE);
        Lamp_Control(LAMP_GREEN, LAMP_DISABLE);
        Lamp_Control(LAMP_BLUE, LAMP_DISABLE);    
    } 
	else 
	{ // Released
		if(old_released!=released)
		{
			printUart(DBG_MSG_PC, "EMG Switch Released!!");
			old_released=released;
			UART_SendMessage(DBG_MSG_PC, EMERGENCY_SWITCH_OFF);
#ifdef USE_TABLET_PC
			UART_SendMessage(DBG_MSG_TABLET, EMERGENCY_SWITCH_OFF);
#endif /* USE_TABLET_PC */
		}    

        Lamp_Control(LAMP_RED, LAMP_DISABLE);
        Lamp_Control(LAMP_GREEN, LAMP_DISABLE);
        Lamp_Control(LAMP_BLUE, LAMP_DISABLE);    
    }

    return released;
}

/**
  * @brief  This function handles External lines 9 to 5 interrupt request.
  * @param  None
  * @retval : None
  */
void EXTI9_5_IRQHandler(void)
{
   
    //static char bPressed = FALSE;
    
    if (EXTI_GetITStatus(EMG_SW_GPIO_EXTI_LINE) != RESET)
    {
        /* Clear the Emergency Switch EXTI line pending bit */
        EXTI_ClearITPendingBit(EMG_SW_GPIO_EXTI_LINE);

        if (!EMG_ProcessSwitch())
		{
            //bPressed = TRUE;
        }
		else
		{
            printUart(DBG_MSG_PC, "\r\n\r\n\r\nSystem will be reset....!\r\n\r\n");

            RestartSystem();
        }
    }
}


/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/


