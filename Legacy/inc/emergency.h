/*
*******************************************************************************
* emergency.h :
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

/* Define to prevent recursive inclusion ---------------------------------- */
#ifndef __EMERGENCY_H__
#define __EMERGENCY_H__


/* Include files ------------------------------------------------------ */


/* Exported typedef -------------------------------------------------- */
/* Exported define --------------------------------------------------- */

#if 0 // test for exposure => good
#define EMG_SW_GPIO_CLK                 RCC_AHB1Periph_GPIOD
#define EMG_SW_GPIO_PORT               GPIOD
#define EMG_SW_GPIO_PIN                 GPIO_Pin_7
#define EMG_SW_GPIO_MODE              GPIO_Mode_IN
#define EMG_SW_GPIO_PUPD               GPIO_PuPd_NOPULL
#define EMG_SW_GPIO_SPEED              GPIO_Speed_50MHz
#define EMG_SW_GPIO_PORT_SOURCE      EXTI_PortSourceGPIOD
#define EMG_SW_GPIO_PIN_SOURCE        EXTI_PinSource7
#define EMG_SW_GPIO_EXTI_LINE           EXTI_Line7
#define EMG_SW_GPIO_EXTI_MODE          EXTI_Mode_Interrupt
#define EMG_SW_GPIO_EXTI_TRIGGER        EXTI_Trigger_Rising_Falling
#else
#define EMG_SW_GPIO_CLK                 RCC_AHB1Periph_GPIOA
#define EMG_SW_GPIO_PORT               GPIOA
#define EMG_SW_GPIO_PIN                 GPIO_Pin_5
#define EMG_SW_GPIO_MODE              GPIO_Mode_IN
#define EMG_SW_GPIO_PUPD               GPIO_PuPd_NOPULL
#define EMG_SW_GPIO_SPEED              GPIO_Speed_50MHz
#define EMG_SW_GPIO_PORT_SOURCE      EXTI_PortSourceGPIOA
#define EMG_SW_GPIO_PIN_SOURCE        EXTI_PinSource5
#define EMG_SW_GPIO_EXTI_LINE           EXTI_Line5
#define EMG_SW_GPIO_EXTI_MODE          EXTI_Mode_Interrupt
#define EMG_SW_GPIO_EXTI_TRIGGER        EXTI_Trigger_Rising_Falling
#endif


/* Exported macro --------------------------------------------------- */
/* Exported variables ------------------------------------------------- */


/* Exported functions ------------------------------------------------- */
void EMG_Config(void);
char EMG_ProcessSwitch(void);


#endif /* __EMERGENCY_H__ */

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/
