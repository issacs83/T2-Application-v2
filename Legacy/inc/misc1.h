/*
*********************************************************************************************
* misc.h :
*********************************************************************************************
* Copyright (C) 2016-2018 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date : 
* @ Brief :
*
* @ Revision History :
*       1) initial creation. -------------------------------------- 2018-11-12
*********************************************************************************************
*/

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __MISC_H__
#define __MISC_H__

#include <stdbool.h>
#include <stm32f2xx_gpio.h>
#include <misc.h>
/* Exported define ----------------------------------------------------- */
#define	EXPOSURE_LED_GPIO_ENABLE			Bit_RESET
#define	EXPOSURE_LED_GPIO_DISABLE			Bit_SET

#define INDICATOR_ENABLE 					Bit_RESET
#define INDICATOR_DISABLE 					Bit_SET

#define LAMP_ENABLE 						Bit_RESET
#define LAMP_DISABLE 						Bit_SET


/* Exported types ------------------------------------------------------*/
typedef enum
{
    LAMP_RED,
    LAMP_GREEN,
    LAMP_BLUE,
    LAMP_MOOD_1,
    LAMP_MOOD_2,
} Lamp_Typedef;

typedef enum
{
	LED_BLACK,
	LED_WHITE,
	LED_GREEN,
	LED_YELLOW,
	LED_RED,
	LED_PURPLE,
	LED_BLUE,
	LED_CYAN
} Indicator_Typedef;

typedef enum
{
	COLUMN_STOP,
	COLUMN_UP,
	COLUMN_DOWN
} Column_Typedef;

typedef enum BeamType {
	TYPE_HEAD_CT,
	TYPE_HEAD_PANO,
	TYPE_HEAD_CEPH,
	TYPE_FOOT,
	//TYPE_ALIGN_CEPH,
} BeamType_t;


/* Exported macro ---------------------------------------------------- */
/* Exported variables -------------------------------------------------- */
extern bool g_bColumnPressed;
#ifdef USE_CT_STITCH_MODE
extern uint32_t nHallSensorCount,limit_count;
extern char bColumnStop;
extern char bTsMotorInterlock;
#endif /* USE_CT_STITCH_MODE */


/* Exported functions -------------------------------------------------- */
void Exposure_Config(void);
void Exposure_CtrlLed(BitAction bit);
bool Exposure_CheckSwitch(void);
bool bIsExposureSwitchReleased(uint32_t millis);
void LaserConfig(void);
void LaserControl(BeamType_t beamType, bool isEnable);
void LaserSetPosition(void);
void Column_Config(void);
void Column_Control(uint8_t status);
#ifdef USE_CT_STITCH_MODE
void Column_EncControl(bool bEnable);
void Column_Enable(void);
void Column_Disable(void);
#endif /* USE_CT_STITCH_MODE */
void Lamp_Config(void);
void Lamp_Control(uint8_t lamp, BitAction bit);
void Lamp_Color_Control(Indicator_Typedef color);
void Led_dip_Config(void);
void Led_dip_Control(BitAction bit);
void Indicator_Config(void);
void Indicator_Control(BitAction bit);
// 200408  HWAN TEST STITCH SENSOR
bool StitchUnderSensor_Check(void);
bool StitchUpSwitch_Check(void);
void StichSensor_Config(void);



//


#endif /* __MISC_H__ */
