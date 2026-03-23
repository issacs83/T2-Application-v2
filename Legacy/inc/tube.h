/*
*******************************************************************************************
* tube.h :
*******************************************************************************************
* Copyright (C) 2016-2018 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date : 
* @ Brief :
*
* @ Revision History :
*       1) initial creation. ------------------------------------ 2018-11-12
*******************************************************************************************
*/

/* Define to prevent recursive inclusion ---------------------------------- */
#ifndef __TUBE_H__
#define __TUBE_H__


/* Exported define --------------------------------------------------- */
#define	MAX_TUBE_KV 			100 // 100 KV
#define	MIN_TUBE_KV 			50 // 50 KV
#define	MAX_TUBE_mA 			160 //16.0 mA
#define	MIN_TUBE_mA 			40 // 4.0 mA
#define	MAX_TANK_TEMP 			65 // 60'
#define	MIN_TANK_TEMP 			40 // 40'

#define	TUBE_SOURCE								TIM_IT_CC1

#if 1 // 신호 반전 
#define	TUBE_READY_ENABLE						Bit_RESET
#define	TUBE_READY_DISABLE						Bit_SET

#define	TUBE_EXPOSURE_ENABLE					Bit_RESET
#define	TUBE_EXPOSURE_DISABLE					Bit_SET
#else
#define	TUBE_READY_ENABLE						Bit_SET
#define	TUBE_READY_DISABLE						Bit_RESET

#define	TUBE_EXPOSURE_ENABLE					Bit_SET
#define	TUBE_EXPOSURE_DISABLE					Bit_RESET
#endif

#if defined(USE_CT_XRAY_PULSED_MODE)
#define	TUBE_XRAY_ON_ms					15
#endif /* USE_CT_XRAY_PULSED_MODE */


/* Exported types ----------------------------------------------------*/
typedef struct
{
	double		XonTime;
	double		XoffTime;
	double		DelayTime;
	double		XonTimeCcr;
	double		XoffTimeCcr;
	double		DelayTimeCcr;
} TubePulseParam_Typdef;

typedef struct
{
#ifndef USE_TUBE_PPS_TYPE_IO
	Timer_Typedef	Timer;
#endif /* not USE_TUBE_PPS_TYPE_IO */
	TubePulseParam_Typdef	Param;
	bool	Update;	
    char    bXrayOnOff;
	char	bXrayFrameOnOff;
} Tube_Typedef;


/* Exported macro ---------------------------------------------------*/
/* Exported variables -------------------------------------------------*/
extern Tube_Typedef Tube;


/* Exported functions ------------------------------------------------ */
void Tube_Config(void);
#ifdef USE_TUBE_PPS_TYPE_IO
void Tube_CtrlPps(BitAction bit);
#else /* not USE_TUBE_PPS_TYPE_IO */
void Tube_Start(void);
void Tube_Stop(void);
#endif /* USE_TUBE_PPS_TYPE_IO */
void Tube_CtrlReady(BitAction bit);
void Tube_CtrlExposure(BitAction bit);


#endif /* __TUBE_H__ */
