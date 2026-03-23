/*
*******************************************************************************************
* sensor.h :
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
#ifndef __SENSOR_H__
#define __SENSOR_H__


/* Exported define --------------------------------------------------- */
#define	SENSOR_P_SOURCE							TIM_IT_CC1

#define	SENSOR_S_OUTPUT_ENABLE					Bit_RESET
#define	SENSOR_S_OUTPUT_DISABLE					Bit_SET


/* Exported types ----------------------------------------------------*/
typedef struct 
{
    double 	SyncHighTime;
    double 	SyncLowTime;

    double 	SyncHighTimeCcr;
    double 	SyncLowTimeCcr;
    float 	VariableLowTimeCcr;
} VarianSync_Typdef;

typedef struct
{
	Timer_Typedef		Timer;
    VarianSync_Typdef Trigger;
	//GpioParam		Input;
	//GpioParam		Power;
    char        PowerStatus;
	uint32_t	CaptureSize;
	uint32_t	CaptureStep;
	float*		CaptureCcr;
	
	float		NextCcr;
	uint32_t	Index;
	bool		Start;
	bool		Update;
	uint32_t	Count;
} PanoSensor_Typedef;

#if defined(USE_CT_XRAY_PULSED_MODE)
typedef struct
{
	double		SyncHighTime;
	double		SyncLowTime;

	double		SyncHighTimeCcr;
	double		SyncLowTimeCcr;
} SensorSync_Typdef;

typedef struct
{
    Timer_Typedef Timer;
    SensorSync_Typdef Param;
    bool Update;

    char bSetFrameCount;
    unsigned int nFrameCount;
} CtSensor_Typedef;
#endif /* USE_CT_XRAY_PULSED_MODE */


/* Exported macro ---------------------------------------------------*/
/* Exported variables -------------------------------------------------*/
extern PanoSensor_Typedef Sensor_P;
#if defined(USE_CT_XRAY_PULSED_MODE)
extern CtSensor_Typedef Sensor_C;
#endif /* USE_CT_XRAY_PULSED_MODE */


/* Exported functions ------------------------------------------------ */
void PanoSensor_Config(void);
void PanoSensor_Start(void);
void PanoSensor_Stop(void);
BitAction PanoSensor_CheckInput(void);
void PanoSensor_PowerConfig(void);
void PanoSensor_PowerControl(char bEnable);
void PanoSensor_SetUserSync(void);

void ScanSensor_Config(void);
void ScanSensor_CtrlOutput(BitAction bit);
void ScanSensor_PowerConfig(void);
void ScanSensor_PowerControl(char bEnable);

#if defined(USE_CT_XRAY_PULSED_MODE)
void CtSensor_Config(void);
void CtSensor_Start(void);
void CtSensor_Stop(void);
#endif /* USE_CT_XRAY_PULSED_MODE */


#endif /* __SENSOR_H__ */
