/*
*******************************************************************************************
* arch.c :
*******************************************************************************************
* Copyright (C) 2014-2016 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date : 
* @ Brief :
*
* @ Revision History :
*       1) initial creation. ----------------------------------- 2014-11-16
*       2) V1.1.0  ---------------------------------------- 2015-03-20
*       3) V1.2.0  ---------------------------------------- 2015-09-18
*       4) V1.3.0  ---------------------------------------- 2015-10-28
*******************************************************************************************
*/

/* Include files ------------------------------------------------------ */
#include "extern.h"
#include "arch_standard.h"
#include "arch_child.h"
#include "arch_sinus.h"
#include "arch_tmj.h"
//#include "arch_3d.h"
#include "motor.h"
#include "sensor.h"
#include "timer.h"


/* Private typedef --------------------------------------------------- */
/* Private define ---------------------------------------------------- */
#define	PANO_ARCH_ACC_SIZE			50
#define	PANO_ARCH_MAX_SIZE			152


/* Private macro ---------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static int32_t Motor_R_Step[(PANO_ARCH_ACC_SIZE + PANO_ARCH_MAX_SIZE) * 2];
static float Motor_R_Ccr[(PANO_ARCH_ACC_SIZE + PANO_ARCH_MAX_SIZE) * 2];
static int32_t Motor_V_Step[(PANO_ARCH_ACC_SIZE + PANO_ARCH_MAX_SIZE) * 2];
static float Motor_V_Ccr[(PANO_ARCH_ACC_SIZE + PANO_ARCH_MAX_SIZE) * 2];
static float Sensor_P_Ccr[(PANO_ARCH_ACC_SIZE + PANO_ARCH_MAX_SIZE) * 2];


/* Private function prototypes ------------------------------------------*/
static void Arch_CopyFreqCcr(uint32_t size, float* copy, const float* ccr);
static void Arch_CopySensorFreqCcr(uint32_t size, float* copy, const float* freq);
static void Arch_CopyRstep(uint32_t size, int32_t* copy, const int32_t* step);
static void Arch_CopyVstep(uint32_t size, int32_t* copy, const int32_t* step);


/* Private functions -------------------------------------------------- */
/**
* @ Function Name : Arch_GetTable
* @ Desc : 
* @ Param : 
* @ Return :
*/
void Arch_GetTable(PanoArch arch, PanoScan scan)
{
	Motor_R.ArchParam.StepArray = Motor_R_Step;
	Motor_R.ArchParam.CcrArray = Motor_R_Ccr;
	Motor_V.ArchParam.StepArray = Motor_V_Step;
	Motor_V.ArchParam.CcrArray = Motor_V_Ccr;
	Sensor_P.CaptureCcr = Sensor_P_Ccr;

	memset(Motor_R.ArchParam.StepArray, 0, sizeof(int32_t) * (PANO_ARCH_ACC_SIZE + PANO_ARCH_MAX_SIZE) * 2);
	memset(Motor_R.ArchParam.CcrArray, 0, sizeof(float) * (PANO_ARCH_ACC_SIZE + PANO_ARCH_MAX_SIZE) * 2);
	memset(Motor_V.ArchParam.StepArray, 0, sizeof(int32_t) * (PANO_ARCH_ACC_SIZE + PANO_ARCH_MAX_SIZE) * 2);
	memset(Motor_V.ArchParam.CcrArray, 0, sizeof(float) * (PANO_ARCH_ACC_SIZE + PANO_ARCH_MAX_SIZE) * 2);
	memset(Sensor_P.CaptureCcr, 0, sizeof(float) * (PANO_ARCH_ACC_SIZE + PANO_ARCH_MAX_SIZE) * 2);

	/*if (PanoParam.Mode == PANO_MODE_3D)
	{
		if (arch == PANO_ARCH_CHILD)
		{
			Arch_CopyFreqCcr(n3dAccelSize + n3dCaptureSize, Motor_R_Ccr, arr3dChildRtableRate);
			Arch_CopyRstep(n3dAccelSize + n3dCaptureSize, Motor_R_Step, arr3dChildRtableStep);
			Arch_CopyFreqCcr(n3dAccelSize + n3dCaptureSize, Motor_V_Ccr, arr3dChildVtableRate);
			Arch_CopyVstep(n3dAccelSize + n3dCaptureSize, Motor_V_Step, arr3dChildVtableStep);
			Arch_CopySensorFreqCcr(n3dAccelSize + n3dCaptureSize, Sensor_P_Ccr, arr3dChildStableRate);
		
			Motor_R.ArchParam.OffsetStep = n3dChildStartPosRoffset;
			Motor_V.ArchParam.OffsetStep = n3dChildStartPosVoffset;
		}
		else
		{
			Arch_CopyFreqCcr(n3dAccelSize + n3dCaptureSize, Motor_R_Ccr, arr3dAdultRtableRate);
			Arch_CopyRstep(n3dAccelSize + n3dCaptureSize, Motor_R_Step, arr3dAdultRtableStep);
			Arch_CopyFreqCcr(n3dAccelSize + n3dCaptureSize, Motor_V_Ccr, arr3dAdultVtableRate);
			Arch_CopyVstep(n3dAccelSize + n3dCaptureSize, Motor_V_Step, arr3dAdultVtableStep);
			Arch_CopySensorFreqCcr(n3dAccelSize + n3dCaptureSize, Sensor_P_Ccr, arr3dAdultStableRate);
		
			Motor_R.ArchParam.OffsetStep = n3dAdultStartPosRoffset;
			Motor_V.ArchParam.OffsetStep = n3dAdultStartPosVoffset;
		}
		
		//Motor_R, Motor_V, Sensor_P 중복
		Motor_R.ArchParam.AccSize = n3dAccelSize;
		Motor_R.ArchParam.CaptureSize = (n3dCaptureSize * 2) + 1;

		Motor_V.ArchParam.AccSize = n3dAccelSize;
		Motor_V.ArchParam.CaptureSize = (n3dCaptureSize * 2) + 1;

		Sensor_P.CaptureStep = n3dCaptureStep;
		Sensor_P.CaptureSize = ((n3dAccelSize + n3dCaptureSize) * 2) + 1;		
	}
	else*/
	{
		if (scan == PANO_SCAN_HD)
		{
			if (arch == PANO_ARCH_ADULT)
			{
				Arch_CopyFreqCcr(StandardAccelSize + StandardCaptureSize, Motor_R_Ccr, StandardRtableHighRate);
				Arch_CopyRstep(StandardAccelSize + StandardCaptureSize,	 Motor_R_Step, StandardRtableStep);
				Arch_CopyFreqCcr(StandardAccelSize + StandardCaptureSize, Motor_V_Ccr, StandardVtableHighRate);
				Arch_CopyVstep(StandardAccelSize + StandardCaptureSize, Motor_V_Step, StandardVtableStep);
	            Arch_CopySensorFreqCcr(StandardAccelSize + StandardCaptureSize, Sensor_P_Ccr, StandardStableHighRate);

				Motor_R.ArchParam.AccSize = StandardAccelSize;
				Motor_R.ArchParam.OffsetStep = StandardStartPosRoffset;
				Motor_R.ArchParam.CaptureSize = (StandardCaptureSize * 2) + 1;

				Motor_V.ArchParam.AccSize = StandardAccelSize;
				Motor_V.ArchParam.OffsetStep = StandardStartPosVoffset;
				Motor_V.ArchParam.CaptureSize = (StandardCaptureSize * 2) + 1;

				Sensor_P.CaptureStep = StandardCaptureStep;
				Sensor_P.CaptureSize = ((StandardAccelSize + StandardCaptureSize) * 2) + 1;
			}
			else if (arch == PANO_ARCH_CHILD)
			{
				Arch_CopyFreqCcr(ChildAccelSize + ChildCaptureSize, Motor_R_Ccr, ChildRtableHighRate);
				Arch_CopyRstep(ChildAccelSize + ChildCaptureSize, Motor_R_Step, ChildRtableStep);
				Arch_CopyFreqCcr(ChildAccelSize + ChildCaptureSize, Motor_V_Ccr, ChildVtableHighRate);
				Arch_CopyVstep(ChildAccelSize + ChildCaptureSize, Motor_V_Step, ChildVtableStep);
	            Arch_CopySensorFreqCcr(ChildAccelSize + ChildCaptureSize, Sensor_P_Ccr, ChildStableHighRate);
				
				Motor_R.ArchParam.AccSize = ChildAccelSize;
				Motor_R.ArchParam.OffsetStep = ChildStartPosRoffset;
				Motor_R.ArchParam.CaptureSize = (ChildCaptureSize * 2) + 1;

				Motor_V.ArchParam.AccSize = ChildAccelSize;
				Motor_V.ArchParam.OffsetStep = ChildStartPosVoffset;
				Motor_V.ArchParam.CaptureSize = (ChildCaptureSize * 2) + 1;

				Sensor_P.CaptureStep = ChildCaptureStep;
				Sensor_P.CaptureSize = ((ChildAccelSize + ChildCaptureSize) * 2) + 1;
			}
			else if (arch == PANO_ARCH_SINUS)
			{
				Arch_CopyFreqCcr(SinusAccelSize + SinusCaptureSize, Motor_R_Ccr, SinusRtableHighRate);
				Arch_CopyRstep(SinusAccelSize + SinusCaptureSize, Motor_R_Step, SinusRtableStep);
				Arch_CopyFreqCcr(SinusAccelSize + SinusCaptureSize, Motor_V_Ccr, SinusVtableHighRate);
				Arch_CopyVstep(SinusAccelSize + SinusCaptureSize, Motor_V_Step, SinusVtableStep);
	            Arch_CopySensorFreqCcr(SinusAccelSize + SinusCaptureSize, Sensor_P_Ccr, SinusStableHighRate);

				Motor_R.ArchParam.AccSize = SinusAccelSize;
				Motor_R.ArchParam.OffsetStep = SinusStartPosRoffset;
				Motor_R.ArchParam.CaptureSize = (SinusCaptureSize * 2) + 1;

				Motor_V.ArchParam.AccSize = SinusAccelSize;
				Motor_V.ArchParam.OffsetStep = SinusStartPosVoffset;
				Motor_V.ArchParam.CaptureSize = (SinusCaptureSize * 2) + 1;

				Sensor_P.CaptureStep = SinusCaptureStep;
				Sensor_P.CaptureSize = ((SinusAccelSize + SinusCaptureSize) * 2) + 1;
			}
			else if (arch == PANO_ARCH_TMJ)
			{
				Arch_CopyFreqCcr(TmjAccelSize + TmjCaptureSize, Motor_R_Ccr, TmjRtableHighRate);
				Arch_CopyRstep(TmjAccelSize + TmjCaptureSize, Motor_R_Step, TmjRtableStep);
				Arch_CopyFreqCcr(TmjAccelSize + TmjCaptureSize, Motor_V_Ccr, TmjVtableHighRate);
				Arch_CopyVstep(TmjAccelSize + TmjCaptureSize, Motor_V_Step,	 TmjVtableStep);
	            Arch_CopySensorFreqCcr(TmjAccelSize + TmjCaptureSize, Sensor_P_Ccr, TmjStableHighRate);

				Motor_R.ArchParam.AccSize = TmjAccelSize;
				Motor_R.ArchParam.OffsetStep = TmjStartPosRoffset;
				Motor_R.ArchParam.CaptureSize = (TmjCaptureSize * 2) + 1;

				Motor_V.ArchParam.AccSize = TmjAccelSize;
				Motor_V.ArchParam.OffsetStep = TmjStartPosVoffset;
				Motor_V.ArchParam.CaptureSize = (TmjCaptureSize * 2) + 1;

				Sensor_P.CaptureStep = TmjCaptureStep;
				Sensor_P.CaptureSize = ((TmjAccelSize + TmjCaptureSize) * 2) + 1;
			}
		}
		else //(scan == PANO_SCAN_ND)
		{
			if (arch == PANO_ARCH_ADULT)
			{
				Arch_CopyFreqCcr(StandardAccelSize + StandardCaptureSize, Motor_R_Ccr, StandardRtableNormalRate);
				Arch_CopyRstep(StandardAccelSize + StandardCaptureSize,	 Motor_R_Step, StandardRtableStep);
				Arch_CopyFreqCcr(StandardAccelSize + StandardCaptureSize, Motor_V_Ccr, StandardVtableNormalRate);
				Arch_CopyVstep(StandardAccelSize + StandardCaptureSize, Motor_V_Step, StandardVtableStep);
	            Arch_CopySensorFreqCcr(StandardAccelSize + StandardCaptureSize, Sensor_P_Ccr, StandardStableNormalRate);

				Motor_R.ArchParam.AccSize = StandardAccelSize;
				Motor_R.ArchParam.OffsetStep = StandardStartPosRoffset;
				Motor_R.ArchParam.CaptureSize = (StandardCaptureSize * 2) + 1;
				
				Motor_V.ArchParam.AccSize = StandardAccelSize;
				Motor_V.ArchParam.OffsetStep = StandardStartPosVoffset;
				Motor_V.ArchParam.CaptureSize = (StandardCaptureSize * 2) + 1;
				
				Sensor_P.CaptureStep = StandardCaptureStep;
				Sensor_P.CaptureSize = ((StandardAccelSize + StandardCaptureSize) * 2) + 1;
			}
			else if (arch == PANO_ARCH_CHILD)
			{
				Arch_CopyFreqCcr(ChildAccelSize + ChildCaptureSize, Motor_R_Ccr, ChildRtableNormalRate);
				Arch_CopyRstep(ChildAccelSize + ChildCaptureSize, Motor_R_Step, ChildRtableStep);
				Arch_CopyFreqCcr(ChildAccelSize + ChildCaptureSize, Motor_V_Ccr, ChildVtableNormalRate);
				Arch_CopyVstep(ChildAccelSize + ChildCaptureSize, Motor_V_Step, ChildVtableStep);
	            Arch_CopySensorFreqCcr(ChildAccelSize + ChildCaptureSize, Sensor_P_Ccr, ChildStableNormalRate);
	            
				Motor_R.ArchParam.AccSize = ChildAccelSize;
				Motor_R.ArchParam.OffsetStep = ChildStartPosRoffset;
				Motor_R.ArchParam.CaptureSize = (ChildCaptureSize * 2) + 1;
				
				Motor_V.ArchParam.AccSize = ChildAccelSize;
				Motor_V.ArchParam.OffsetStep = ChildStartPosVoffset;
				Motor_V.ArchParam.CaptureSize = (ChildCaptureSize * 2) + 1;

				Sensor_P.CaptureStep = ChildCaptureStep;
				Sensor_P.CaptureSize = ((ChildAccelSize + ChildCaptureSize) * 2) + 1;
			}
			else if (arch == PANO_ARCH_SINUS)
			{
				Arch_CopyFreqCcr(SinusAccelSize + SinusCaptureSize, Motor_R_Ccr, SinusRtableNormalRate);
				Arch_CopyRstep(SinusAccelSize + SinusCaptureSize, Motor_R_Step, SinusRtableStep);
				Arch_CopyFreqCcr(SinusAccelSize + SinusCaptureSize, Motor_V_Ccr, SinusVtableNormalRate);
				Arch_CopyVstep(SinusAccelSize + SinusCaptureSize, Motor_V_Step,	SinusVtableStep);
	            Arch_CopySensorFreqCcr(SinusAccelSize + SinusCaptureSize, Sensor_P_Ccr, SinusStableNormalRate);

				Motor_R.ArchParam.AccSize = SinusAccelSize;
				Motor_R.ArchParam.OffsetStep = SinusStartPosRoffset;
				Motor_R.ArchParam.CaptureSize = (SinusCaptureSize * 2) + 1;

				Motor_V.ArchParam.AccSize = SinusAccelSize;
				Motor_V.ArchParam.OffsetStep = SinusStartPosVoffset;
				Motor_V.ArchParam.CaptureSize = (SinusCaptureSize * 2) + 1;

				Sensor_P.CaptureStep = SinusCaptureStep;
				Sensor_P.CaptureSize = ((SinusAccelSize + SinusCaptureSize) * 2) + 1;
			}
			else if (arch == PANO_ARCH_TMJ)
			{
				Arch_CopyFreqCcr(TmjAccelSize + TmjCaptureSize, Motor_R_Ccr, TmjRtableNormalRate);
				Arch_CopyRstep(TmjAccelSize + TmjCaptureSize, Motor_R_Step, TmjRtableStep);
				Arch_CopyFreqCcr(TmjAccelSize + TmjCaptureSize, Motor_V_Ccr, TmjVtableNormalRate);
				Arch_CopyVstep(TmjAccelSize + TmjCaptureSize, Motor_V_Step, TmjVtableStep);
	            Arch_CopySensorFreqCcr(TmjAccelSize + TmjCaptureSize, Sensor_P_Ccr, TmjStableNormalRate);

				Motor_R.ArchParam.AccSize = TmjAccelSize;
				Motor_R.ArchParam.OffsetStep = TmjStartPosRoffset;
				Motor_R.ArchParam.CaptureSize = (TmjCaptureSize * 2) + 1;

				Motor_V.ArchParam.AccSize = TmjAccelSize;
				Motor_V.ArchParam.OffsetStep = TmjStartPosVoffset;
				Motor_V.ArchParam.CaptureSize = (TmjCaptureSize * 2) + 1;

				Sensor_P.CaptureStep = TmjCaptureStep;
				Sensor_P.CaptureSize = ((TmjAccelSize + TmjCaptureSize) * 2) + 1;
			}
		}
	}
	
    Motor_R.ArchParam.nExpOffIndex = (Motor_R.ArchParam.AccSize + Motor_R.ArchParam.CaptureSize);// - 1;
}

/**
* @ Function Name : Arch_CopyFreqCcr
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void Arch_CopyFreqCcr(uint32_t size, float* copy, const float* freq)
{
    int i;

    for (i = 0; i < size; i++)
    {
        copy[i] = Hz_To_CCR(freq[size - 1 - i]);
        copy[size + 1 + i] = Hz_To_CCR(freq[i]);
    }
    copy[size] = Hz_To_CCR(freq[0]);
}

/**
* @ Function Name : Arch_CopySensorFreqCcr
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void Arch_CopySensorFreqCcr(uint32_t size, float* copy, const float* freq)
{
    int i;

    for (i = 0; i < size ; i++)
    {
        copy[i] = LowSig_Hz_To_CCR(freq[size - 1 - i]);
        copy[size + 1 + i] = LowSig_Hz_To_CCR(freq[i]);
    }
    copy[size] = LowSig_Hz_To_CCR(freq[0]);
}

/**
* @ Function Name : Arch_CopyRstep
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void Arch_CopyRstep(uint32_t size, int32_t* copy, const int32_t* step)
{
    int i;

    for (i = 0; i < size; i++)
    {                       
        copy[i] = step[size - 1 - i];
        copy[size + 1 + i] = step[i];
    }
    copy[size] = step[0];
}

/**
* @ Function Name : Arch_CopyVstep
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void Arch_CopyVstep(uint32_t size, int32_t* copy, const int32_t* step)
{
    int i;

    for (i = 0; i < size; i++)	
    {                       
        copy[i] = -step[size - 1 - i];
        copy[size +1 + i] = step[i];
    }
    copy[size] = step[0];
}

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/

