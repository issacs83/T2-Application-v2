/**
  ******************************************************************************
  * @file			extern.h
  * @author  	Kim Sewoon
  * @version 	V1.0.0
  * @date			23-September-2014
  * @brief			
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EXTERN_H
#define __EXTERN_H

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f2xx.h"

/******************************************************************************/
/*            System                                               */
/******************************************************************************/

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/




//#define	true					1
//#define	false					0


#ifndef __cplusplus
#ifndef bool 
typedef enum {FALSE = 0, TRUE = !FALSE} bool;
#endif /* not bool */
#endif /* not __cplusplus */


/* Exported macro -----------------------------------------------------------*/
/* Exported variables ---------------------------------------------------------*/

//extern GPIO_InitTypeDef GPIO_InitStructure;
extern TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
extern TIM_OCInitTypeDef TIM_OCInitStructure;
//extern NVIC_InitTypeDef NVIC_InitStructure;


/* g_bTiltStatus and CurCaptureMode now defined in system.c,
 * declared extern volatile in shared_vars.h */
#include "shared_vars.h"


/******************************************************************************/
/*            GPIO                                                 */
/******************************************************************************/

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct
{
	GPIO_TypeDef*	Gpio;
	uint16_t		GpioPin;
} GpioParam;


/******************************************************************************/
/*            TIM                                                   */
/******************************************************************************/

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef struct
{
	TIM_TypeDef*	Periph;
	GPIO_TypeDef*	Gpio;
	uint32_t		Clock;
	uint32_t		GpioClock;
	uint16_t		GpioPin;
	uint8_t			GpioPinsource;
	uint8_t			GpioAf;
	uint16_t		InitStatus;
	uint32_t		InitCCR;
	int32_t			Interrupt;
	uint16_t		Source;
	uint16_t		Channel;
	uint8_t			Priority;
} Timer_Typedef;

/* Exported macro -----------------------------------------------------------*/
/* Exported variables ---------------------------------------------------------*/
/* Exported functions ---------------------------------------------------------*/
void Timer_Config(Timer_Typedef* timer);
//jehun
void Timer_Config_T(Timer_Typedef* timer);
void Timer_Config_CC(Timer_Typedef* timer, unsigned char prescaler);
/******************************************************************************/
/*            Motor                                                 */
/******************************************************************************/

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/



/* Exported functions ---------------------------------------------------------*/


/******************************************************************************/
/*            Pano Sensor                                      */
/******************************************************************************/

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/




/* Exported macro -----------------------------------------------------------*/
/* Exported variables ---------------------------------------------------------*/



/* Exported functions ---------------------------------------------------------*/




/******************************************************************************/
/*            Tube			                                          */
/******************************************************************************/

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/


/* Exported macro -----------------------------------------------------------*/
/* Exported variables ---------------------------------------------------------*/

/* Exported functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Internal Timer                                      */
/******************************************************************************/

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/* Exported macro -----------------------------------------------------------*/
/* Exported variables ---------------------------------------------------------*/

/* Exported functions ---------------------------------------------------------*/




/******************************************************************************/
/*            Panorama                                          */
/******************************************************************************/

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#if 0
typedef enum
{
	PANO_SENSOR_I3SYSTEM,
	PANO_SENSOR_AJAT,
	PANO_SENSOR_VARIAN
} PanoSensor;
#endif

typedef enum
{
	PANO_MODE_NONE,
	PANO_MODE_NORMAL,
	PANO_MODE_LEFT,
	PANO_MODE_FRONT,
	PANO_MODE_RIGHT,
	PANO_MODE_SINUS,
	PANO_MODE_TMJ,
	PANO_MODE_3D,
	PANO_MODE_ALIGN
} PanoMode;

typedef enum
{
	PANO_NORMAL,	
	PANO_TMJ
} PanoInitMode_Typedef;


typedef enum
{
	PANO_ARCH_NONE,
	PANO_ARCH_ADULT,
	PANO_ARCH_CHILD,
	PANO_ARCH_SINUS,
	PANO_ARCH_TMJ,
} PanoArch;

typedef enum
{
	PANO_SCAN_NONE,	
	PANO_SCAN_ND,
	PANO_SCAN_HD,
} PanoScan;

typedef struct
{
	//PanoSensor	Sensor;	
	uint32_t 		KV;
	uint32_t		mA;
	PanoMode		Mode;
	PanoArch		Arch;
	PanoScan		Scan;
	int32_t			ModeOffset;
	int16_t			nRAxisOffset;
	int16_t			nVAxisOffset;
	int16_t			nCanineOffset;
	int16_t			nCNSaxisOffset;
	int16_t 		nCWEaxisOffset;
	bool			bColliOpen;
	PanoInitMode_Typedef Init_Mode;
	int16_t			nTAxisOffset;
	int16_t			nTAxisChildOffset;
	bool			bFirstPano;
#ifdef VARIABLE_EXPOSURE
    unsigned int    nExpStartTime;
    unsigned int    nExpEndTime;
    unsigned int    nExpKV;
    unsigned int    nExpMA;    
#endif /* VARIABLE_EXPOSURE */

} PanoParam_Typedef;
typedef enum
{
	PUSHED,
	RELEASED
} TempleSupporStatus;

/* Exported macro -----------------------------------------------------------*/
/* Exported variables ---------------------------------------------------------*/
extern	PanoParam_Typedef PanoParam;
extern TempleSupporStatus Temple_Status;
/* Exported functions ---------------------------------------------------------*/
void PanoCapture(void);
void PanoCapture_SetParam(void);
void PanoCapture_ProcNormalMode(void);
void PanoCapture_ProcLeftMode(void);
void PanoCapture_ProcTmjMode(void);
void PanoCapture_ProcLeftMode(void);
void PanoCapture_ProcRightMode(void);
void PanoCapture_ProcFrontMode(void);
void PanoCapture_ProcAlignMode(void);


void Arch_GetTable(PanoArch arch, PanoScan scan);
//void Led_dip_Config(void);
//void Led_dip_Control(BitAction bit);



/******************************************************************************/
/*            CT                                                       */
/******************************************************************************/

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

#if 0
typedef enum
{
	CT_SENSOR_VARIAN,
	CT_SENSOR_DALSA,
	CT_SENSOR_ZETTA
} CtSensor;

typedef enum
{
	CT_RESOLUTION_NONE,
	CT_RESOLUTION_100um,
	CT_RESOLUTION_200um,
	CT_RESOLUTION_300um
} Resolution_Typedef;
#endif

typedef enum
{
	CT_FOV_NONE = 0,
	CT_FOV_5X5,
	CT_FOV_8X8,
	CT_FOV_8X9,
	CT_FOV_8X10,
	CT_FOV_10X9,
	CT_FOV_10X10,
	CT_FOV_12X9,
	CT_FOV_12X10,
	CT_FOV_15X9,
	CT_FOV_15X10,	
	CT_FOV_15X15,	
} Fov_Typedef;

typedef enum
{
	CT_BINNING_NONE,
	CT_BINNING_1X1,
	CT_BINNING_2X2
} Binning_Typedef;

typedef enum
{
	CT_OCCLUSION,
	CT_FULL_ARCH,	
	CT_SINUS
} e15by9_Typedef;


#if defined(USE_CT_XRAY_PULSED_MODE)
typedef enum
{
	CT_TUBE_NONE,
	CT_TUBE_CONTINUOUS,
	CT_TUBE_PULSED
} TubeMode_Typedef;
#endif /* USE_CT_XRAY_PULSED_MODE */

typedef struct
{
	//CtSensor				Sensor;
	uint32_t 				KV;
	uint32_t				mA;
	//Resolution_Typedef	Resolution;
	Fov_Typedef			Fov;
	Binning_Typedef		Binning;
	e15by9_Typedef		Mode_15by9;
	//Mar_Typedef			Mar;
#if defined(USE_CT_XRAY_PULSED_MODE)    
	TubeMode_Typedef	TubeMode;	
#endif /* USE_CT_XRAY_PULSED_MODE */	
	uint32_t				XonTime;
	uint32_t				XoffTime;
	uint32_t				DelayTime;
	uint32_t				TotalFrame;
	uint32_t				nMotorVStartPos;
	uint32_t				nExpStartPos;
	uint32_t				nRunStep;
	uint32_t				nXrayOnStep;
	uint32_t				nXrayOffStep;
	uint32_t				nEndStep;
    bool                   bCaptEnd;
    int16_t nRaxisOffset;
    int16_t nPaxisOffset;
	int16_t	nFastRaxisOffset;
	int16_t	nCNSaxisOffset;
	int16_t nCWEaxisOffset;
	int32_t nPaxisFovStep;
	int32_t nCNSaxisFovStep;
	double nCWEaxisFovStep;
#ifdef USE_CT_STITCH_MODE
	bool bStitchMode;
	bool bStitchDone;
	bool bStitchBack;
	bool bStitchORGCheck;
#endif /* USE_CT_STITCH_MODE */
	int32_t nAlignRaxisOffset;
	int32_t nAlignPaxisOffset;
	int16_t	nRaxisPatientOffset;
	int16_t	nPaxisPatientOffset;
	bool bStitch_Status;
	bool bReverse_Status;
	uint32_t nReverseTotalStep;
	bool bFirstCT;
	bool bCNSaxisSET;
	bool bCWEaxisSET;
	bool bPaxisSET;
	/*
	//200217 HWAN TEST
	uint16_t	IC1VALUE_TABLE[350];
	uint16_t	IC1VALUE_SAME[350];
	float    IC1ARR[350];
	*/
} CtParam_Typedef;

/* Exported macro -----------------------------------------------------------*/
/* Exported variables ---------------------------------------------------------*/
extern double	MOTOR_R_MICROSTEP__;
extern CtParam_Typedef CtParam;

/* Exported functions ---------------------------------------------------------*/
void CtCapture(void);


/******************************************************************************/
/*            Ceph                                                  */
/******************************************************************************/

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/


typedef enum
{
	Ceph_Full_Lateral,
	Ceph_Lateral,
	Ceph_Carpus,
	Ceph_PA,
	Ceph_AP,
	Ceph_Waters,
	Ceph_SVM,
	Ceph_EarLod,
} Ceph_Mode_Typedef;

typedef enum
{
	Ceph_Adult,
	Ceph_Child,
} Ceph_Person_Typedef;


typedef struct {
    uint32_t KV;
    uint32_t mA;
    uint32_t Time;
    uint32_t Size;
	Ceph_Mode_Typedef Mode;
	Ceph_Person_Typedef Type;
    int16_t n2ndColOffset;
	int16_t n2ndColFastStartOffset;
	int16_t n2ndColEndOffset;
	int32_t n2ndSpeedStepOffset;
	int32_t n2ndFastSpeedStepOffset;
	int16_t nRAxisOffset;
	bool	bScanAlign;
    bool bInitPosFlag;
	bool bScanEar;
} CephParam_Typedef;

/* Exported macro -----------------------------------------------------------*/
/* Exported variables ---------------------------------------------------------*/
extern CephParam_Typedef CephParam;

/* Exported functions ---------------------------------------------------------*/
void ScanCapture(void);

#endif /* __EXTERN_H */

/***************************** END OF FILE *****************************/
