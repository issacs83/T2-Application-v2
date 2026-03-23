 /*
*******************************************************************************************
* ct_capture.c :
*******************************************************************************************
* Copyright (C) 2014-2016 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date : 
* @ Brief :
*
* @ Revision History :
*       1) initial creation. ------------------------------------ 2014-11-16
*       2) V1.1.0  ---------------------------------------- 2015-03-20
*       3) V1.2.0  ---------------------------------------- 2015-09-18
*       4) V1.3.0  ---------------------------------------- 2015-10-28
*******************************************************************************************
*/

/* Include files ------------------------------------------------------ */
#include "extern.h"
#include "system.h"
#include "error_code.h"
#include "motor.h"
#include "can.h"
#include "tube.h"
#include "serial.h"
#include "misc1.h"
#include "timer.h"
#if defined(USE_CT_XRAY_PULSED_MODE)
#include "sensor.h"
#endif /* USE_CT_XRAY_PULSED_MODE */
#include "safety.h"
#include "motor_compat.h"


/* Private typedef --------------------------------------------------- */
/* Private define ---------------------------------------------------- */
#define CT_MOTORV_START_OFFSET_DEFAULT 				10.0
//#define EXPOSURE_START_POSITION 					25.0 // => Start Frame issue
#define EXPOSURE_START_POSITION 					30.0 // => Start Frame issue
#define EXPOSURE_FAST_START_POSITION 				30.0
#define EXPOSURE_STITCH_START_POSITION 				25.0


#define MOTOR_R_ACCEL_ANGLE					25.0
#ifdef USE_AGING_MODE
#define MOTOR_R_DECEL_ANGLE					20.0
#else /* not USE_AGING_MODE */
#define MOTOR_R_DECEL_ANGLE					20.0 // 200203 HWAN  20->25 stitching issue //38.0 // 22Sec������ �Ĺڴ���  Ȯ�� �ʿ�
#endif /* USE_AGING_MODE */

#define MOTOR_R_FAST_ACCEL_ANGLE			30.0
#define MOTOR_R_FAST_DECEL_ANGLE			38.0

#define END_STEP_MARGIN						1000

#define CT_EXPOSURE_TOTAL_ANGLE 			365.0 //375.0 //370.0//365.0

#define PAXIS_ORG_LIMIT		1 //ORGLIMT�� LIMIT �����ϰ� 1�� �������� ����
#define PAXIS_DISTANCE_PER_FOV_8X			35 //(150 - 80) / 2
#define PAXIS_DISTANCE_PER_FOV_10X			25 //(150 - 100) / 2
#define PAXIS_DISTANCE_PER_FOV_12X			15 //(150 - 120) / 2

#define RECON_22SEC_FRAME_NUMBER				600
#define RECON_14SEC_FRAME_NUMBER				400
#define RECON_07SEC_FRAME_NUMBER				400 // Binning(2x2)

#define CT_ROTATE_XON_ANGLE_DEFAULT 			30.0 // degree
#define CT_ROTATE_XOFF_ANGLE_DEFAULT 	395.0
#define CT_ROTATE_TOTAL_ANGLE_DEFAULT	415.0

#define	CT_ROTATE_ACCEL_ANGLE			25.0 / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO)	
#define	CT_ROTATE_TOTAL_ANGLE			415.0 / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO)
#define	CT_ROTATE_DECEL_ANGLE			20.0 / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO)	
#define	CT_ROTATE_XOFF_ANGLE			395.0 / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO)
#define	CT_ROTATE_XON_ANGLE				30.0 / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO)	



#if defined(USE_CT_XRAY_PULSED_MODE)
#define USE_SENSOR_USER_SYNC
#endif /* USE_CT_XRAY_PULSED_MODE */


/* Private macro ---------------------------------------------------- */
/* Private variables -------------------------------------------------- */
CtParam_Typedef CtParam;
#ifdef USE_CT_STITCH_MODE
typedef enum {
	STATUS_CAPTURE_DONE,
	STATUS_FIRST_CAPTURE_DONE,
	STATUS_CAPTURE_CANCEL,
	STATUS_CAPTURE_STOP,
	STATUS_CAPTURE_STITCH_FAIL,
} Capture_Status;

static double dFps, dTime;
static uint32_t nRotateTotalStep, nAccStep, nDecStep;
static char capture_count;
double MOTOR_R_DEGREE_STEP;
double	MOTOR_R_MICROSTEP__;
#endif /* USE_CT_STITCH_MODE */


/* Private function prototypes ------------------------------------------*/
static void CtCapture_ClearParam(void);
static void CtCapture_EndProcess(void);
static void CtCapture_StopProcess(bool status);
static void CtCapture_CancelProcess(void);
static void CtCapture_ClearProcess(void);
#ifdef USE_CT_STITCH_MODE
static Capture_Status CtCapture_StartProcess(void);
#endif /* USE_CT_STITCH_MODE */


/* Private functions -------------------------------------------------- */

/**
* @ Function Name : CtCapture
* @ Desc : 
* @ Param : 
* @ Return :
*/
void CtCapture(void)
{
#ifdef USE_CT_STITCH_MODE
	Capture_Status status;
#else /* not USE_CT_STITCH_MODE */
    bool bEnd = FALSE;
    double dFps, dTime;
	uint32_t nRotateTotalStep, nAccStep, nDecStep;
#endif /* USE_CT_STITCH_MODE */
    uint16_t nCollimatorCmd = CMD_COLLI_CT15X9;
#if defined(USE_CT_XRAY_PULSED_MODE)
	double readout = 0;
	double period = 0;
#endif /* USE_CT_XRAY_PULSED_MODE */

#ifdef USE_AGING_MODE
	printUart(DBG_MSG_PC, "CT Aging Count :: %d", g_nCtCnt);

	SysTime_MesureStart(__FUNCTION__, __LINE__);
#endif /* USE_AGING_MODE */

	if(gMachStat.bFirstEntry==RESET)
	{
		printUart(DBG_MSG_PC, "==First Entry after boot==");	
		gMachStat.bFirstEntry=SET;
	}

    if (checkPreCaptureStatus() != TRUE) 
	{		
    	printUart(DBG_MSG_PC, "checkPreCaptureStatus :: FALSE");	
		CtCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
		PrevCaptureMode = CAPTURE_CANCEL;
#endif /* USE_AGING_MODE */

        return;
    }

	//�Կ��� ���� ���� ������ Ų�� => �ϴ� ����
    //PanoSensor_PowerControl(TRUE);	

	if(CtParam.bReverse_Status==RESET)
    	UART_SendMessage(DBG_MSG_PC, "[SP_MODE_CT___]");
	else
		UART_SendMessage(DBG_MSG_PC, "[SP_MODE_CT_R_]");
#ifdef USE_TABLET_PC
	if(CtParam.bReverse_Status==RESET)
    	UART_SendMessage(DBG_MSG_TABLET, "[SP_MODE_CT___]");
	else
		UART_SendMessage(DBG_MSG_TABLET, "[SP_MODE_CT_R_]");
#endif /* USE_TABLET_PC */

	if (g_bColumnPressed == SET)
	{
		Column_Control(COLUMN_STOP);
		g_bColumnPressed = FALSE;
	}

#if defined(USE_CT_XRAY_PULSED_MODE) && defined(USE_SENSOR_USER_SYNC)
	//sunghwan - V1.0.0e
	if (CtParam.TubeMode == CT_TUBE_PULSED)
	{
		CtSensor_Config();
		CtSensor_Start();
	}
	// sunghwan - V1.0.0e
#endif /* USE_CT_XRAY_PULSED_MODE && USE_SENSOR_USER_SYNC */

    if (!CAN_Collimator_SendMessage(CMD_TILT_OFF, 0, 0, 5000, 2)) 
	{
		UART_SendMessage(DBG_MSG_PC, "Collimator Communication Fail.");
        CtCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL; 
#ifdef USE_AGING_MODE
				PrevCaptureMode = CAPTURE_CANCEL;
#endif /* USE_AGING_MODE */

        return;
    }
    g_bTiltStatus = FALSE;    

	if (CAN_SendMessage(CAN_TUBE_COM_CHECK, CAN_TUBE_GEN2_COM, 0, 3000, 2))
	{
		CAN_SendMessage(CAN_TUBE_TANK_TEMP_READ, 0, 0, 1000, 2);
	}
	else
	{
		UART_SendMessage(DBG_MSG_PC, "Tube Communication Fail.");
        CtCapture_CancelProcess();
//#ifdef USE_AGING_MODE
//				PrevCaptureMode = CAPTURE_CANCEL;
//#endif /* USE_AGING_MODE */

        CurCaptureMode = CAPTURE_CANCEL;
        
#ifdef USE_AGING_MODE
		PrevCaptureMode = CAPTURE_CANCEL;
#endif /* USE_AGING_MODE */	
        return;
    }

    if (!Motor_MoveInitPosition()) 
	{
		UART_SendMessage(DBG_MSG_PC, "Motor Init Position Fail.");
        CtCapture_CancelProcess();
#ifdef USE_AGING_MODE
				PrevCaptureMode = CAPTURE_CANCEL;
#endif /* USE_AGING_MODE */

        CurCaptureMode = CAPTURE_CANCEL;
        
        return;
    }
	CtCapture_ClearParam();
	//Motor_MoveAlignPosition();

    if (CurCaptureMode != CAPTURE_CT) {
        CtCapture_CancelProcess();
        return;
    }

#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
	ElapseTimerOnOff(SET);
#endif /* BUILD_TYPE_DEBUG && !USE_AGING_MODE */

#ifdef USE_MOTOR_CHINREST_TS
    //Motor_MoveTempleSupport(FALSE);
#endif /* USE_MOTOR_CHINREST_TS */

	LaserControl(TYPE_HEAD_CT, SET);
	LaserControl(TYPE_FOOT, SET);
	
    UART_SendMessage(DBG_MSG_PC, "[SP_CT___READY]");
#ifdef USE_TABLET_PC
    UART_SendMessage(DBG_MSG_TABLET, "[SP_CT___READY]");
#endif /* USE_TABLET_PC */

	if(sysInfo.bShowLog==SET)
	{		
		printUart(DBG_MSG_PC, "CT Mode = %d", CtParam.Mode_15by9);
		printUart(DBG_MSG_PC, "CT Init Motor R::CurrentStep = %d", Motor_R.CurrentStep);
		printUart(DBG_MSG_PC, "CT Init Motor V::CurrentStep = %d", Motor_V.CurrentStep);
		printUart(DBG_MSG_PC, "CT Init Motor A::CurrentStep = %d", Motor_A.CurrentStep);
		printUart(DBG_MSG_PC, "CT Init Motor CNS::CurrentStep = %d", Motor_CNS.CurrentStep);
		printUart(DBG_MSG_PC, "CT Init Motor CWE::CurrentStep = %d", Motor_CWE.CurrentStep);
	}
#ifdef USE_AGING_MODE
	if (g_bAgingMode != SET) {
		while(!UART_SetCtParam() && CurCaptureMode != CAPTURE_CANCEL);
	}
#else /* not USE_AGING_MODE */
    while(!UART_SetCtParam() && CurCaptureMode != CAPTURE_CANCEL);
#endif /* USE_AGING_MODE */
	/*
	if(CtParam.TotalFrame==400)
	{
		MOTOR_R_MICROSTEP__=16*200;
	}
	else
	{
		MOTOR_R_MICROSTEP__=16*200;
	}*/
	MOTOR_R_MICROSTEP__=16*200;
	MOTOR_R_DEGREE_STEP=(360.0 / MOTOR_R_MICROSTEP__ / MOTOR_R_PULLEY_RATIO);
	if (g_bColumnPressed == SET)
	{
		Column_Control(COLUMN_STOP);
		g_bColumnPressed = FALSE;
	}

	LaserControl(TYPE_HEAD_CT, FALSE);
	LaserControl(TYPE_FOOT, FALSE);

	if (!bIsExposureSwitchReleased(500)) 
	{
		UART_SendMessage(DBG_MSG_PC, ERR_CODE_EXP_SWITCH);
		CurCaptureMode = CAPTURE_CANCEL;
	}

    if (CurCaptureMode != CAPTURE_CT) 
	{
        CtCapture_CancelProcess();
        return;
    }
	
	CAN_SendMessage(CAN_TUBE_TANK_TEMP_READ, 0, 0, 1000, 2);

    Lamp_Control(LAMP_RED, LAMP_DISABLE);
    Lamp_Control(LAMP_GREEN, LAMP_ENABLE);
    Lamp_Control(LAMP_BLUE, LAMP_DISABLE);    

    switch(CtParam.Fov) 
	{
	case CT_FOV_NONE:
		nCollimatorCmd = CMD_COLLI_OPEN;
		break;		  
	case CT_FOV_5X5:
		nCollimatorCmd = CMD_COLLI_CT5X5;
		break;
	case CT_FOV_8X9:
		nCollimatorCmd = CMD_COLLI_CT8X9;
		//CtParam.nPaxisFovStep = PAXIS_DISTANCE_PER_FOV_8X / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP);		
		break;
	case CT_FOV_8X10:
		nCollimatorCmd = CMD_COLLI_CT8X10;
		break;
	case CT_FOV_10X9:
		nCollimatorCmd = CMD_COLLI_CT10X9;
		//CtParam.nPaxisFovStep = PAXIS_DISTANCE_PER_FOV_10X / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP);		
		break;
	
	case CT_FOV_12X9:
		nCollimatorCmd = CMD_COLLI_CT12X9;
		//CtParam.nPaxisFovStep = PAXIS_DISTANCE_PER_FOV_12X / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP);		
		break;
	
	case CT_FOV_15X9:
		nCollimatorCmd = CMD_COLLI_CT15X9;
		break;
	case CT_FOV_10X10:
		nCollimatorCmd = CMD_COLLI_CT10X10;
		//CtParam.nPaxisFovStep = PAXIS_DISTANCE_PER_FOV_10X / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP);		
		break;	
	case CT_FOV_12X10:
		nCollimatorCmd = CMD_COLLI_CT12X10;
		//CtParam.nPaxisFovStep = PAXIS_DISTANCE_PER_FOV_12X / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP);		
		break;	
	case CT_FOV_15X10:
		nCollimatorCmd = CMD_COLLI_CT15X10;
		break;
	default:
		nCollimatorCmd = CMD_COLLI_CT15X9;
		break;
    }

	if(sysInfo.bShowLog==SET)
	{
		printUart(DBG_MSG_PC, "FOV Set::CurrentStep = %d, nPaxisFovStep = %d", Motor_V.CurrentStep, CtParam.nPaxisFovStep);
	}

    if (!CAN_Collimator_SendMessage(nCollimatorCmd, 0, 0, 4000, 2)) 
	{
		UART_SendMessage(DBG_MSG_PC, "Collimator FOV Move Fail.");
        CtCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
        
        return;
    }

    if (!CAN_Collimator_SendMessage(CMD_FILTER_CT, 0, 0, 4000, 2)) 
	{	
		UART_SendMessage(DBG_MSG_PC, "Collimator Filter Move Fail.");
        CtCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
        
        return;
    }        

#if defined(USE_CT_XRAY_PULSED_MODE)
#define	n_ms_To_CCR(ms)				(double) (TIM_COUNT_CLOCK/2 * ms) / sec_To_msec

	if (CtParam.TubeMode == CT_TUBE_PULSED)
	{
#if defined(USE_SENSOR_USER_SYNC)
		while(Sensor_C.Update != FALSE);
		CtSensor_Stop();
#endif /* USE_SENSOR_USER_SYNC */

		if (CtParam.Binning == CT_BINNING_1X1)
			readout = 35.0;
		else
			readout = 17.3;

		period = (CtParam.XonTime + CtParam.DelayTime * 2 + readout) / 1000;
		dFps = 1 / period;
		dTime = CtParam.TotalFrame / dFps;

		Tube.Param.XonTime = CtParam.XonTime;
		printUart(DBG_MSG_PC, "XonTime = %.2f", Tube.Param.XonTime);
		
		Tube.Param.XoffTime = CtParam.XoffTime;		
		printUart(DBG_MSG_PC, "XoffTime = %.2f", Tube.Param.XoffTime);

		Tube.Param.DelayTime = CtParam.DelayTime;
		printUart(DBG_MSG_PC, "DelayTime = %.2f", Tube.Param.DelayTime);

		Tube.Param.XonTimeCcr = ms_To_CCR(Tube.Param.XonTime);
		Tube.Param.XoffTimeCcr = ms_To_CCR(Tube.Param.XoffTime);
		Tube.Param.DelayTimeCcr = ms_To_CCR(Tube.Param.DelayTime);

		Sensor_C.Param.SyncLowTime = CtParam.XonTime + CtParam.XoffTime;
		printUart(DBG_MSG_PC, "SyncLowTime = %.2f", Sensor_C.Param.SyncLowTime);

		Sensor_C.Param.SyncHighTime = 4.5;
		printUart(DBG_MSG_PC, "SyncHighTime = %.2f", Sensor_C.Param.SyncHighTime);

		Sensor_C.Param.SyncLowTimeCcr = n_ms_To_CCR(Sensor_C.Param.SyncLowTime);		
		printUart(DBG_MSG_PC, "SyncLowTimeCcr = %.2f", Sensor_C.Param.SyncLowTimeCcr);

		Sensor_C.Param.SyncHighTimeCcr = n_ms_To_CCR(Sensor_C.Param.SyncHighTime);
		printUart(DBG_MSG_PC, "SyncHighTimeCcr = %.2f", Sensor_C.Param.SyncHighTimeCcr);

#if defined(USE_SENSOR_USER_SYNC)
		CtSensor_Start();
#endif /* USE_SENSOR_USER_SYNC */		
	}
#endif /* USE_CT_XRAY_PULSED_MODE */

    if (CtParam.Binning == CT_BINNING_1X1) 
	{
        dFps = 28.0;
		nAccStep = MOTOR_R_ACCEL_ANGLE / MOTOR_R_DEGREE_STEP;
		nDecStep = MOTOR_R_DECEL_ANGLE / MOTOR_R_DEGREE_STEP;
    } 
	else 
	{ // CT_BINNING_2X2
        dFps = 56.0;
		if (CtParam.TotalFrame != RECON_22SEC_FRAME_NUMBER)
			CtParam.nExpStartPos = EXPOSURE_FAST_START_POSITION;

		nAccStep = MOTOR_R_FAST_ACCEL_ANGLE / MOTOR_R_DEGREE_STEP;
		nDecStep = MOTOR_R_FAST_DECEL_ANGLE / MOTOR_R_DEGREE_STEP;
	}

    dTime = CtParam.TotalFrame/ dFps;

    if (CtParam.nExpStartPos != 0) 
	{
	    CtParam.nXrayOnStep = (CtParam.nExpStartPos / MOTOR_R_DEGREE_STEP);
		CtParam.nXrayOffStep = (CtParam.nXrayOnStep + (CT_EXPOSURE_TOTAL_ANGLE / MOTOR_R_DEGREE_STEP));
		nRotateTotalStep = CtParam.nXrayOffStep + nDecStep + 5.0/MOTOR_R_DEGREE_STEP;
    } 
	else 
	{
	    CtParam.nXrayOnStep = (CT_ROTATE_XON_ANGLE_DEFAULT / MOTOR_R_DEGREE_STEP);
	    CtParam.nXrayOffStep = (CT_ROTATE_XOFF_ANGLE_DEFAULT / MOTOR_R_DEGREE_STEP);
		nRotateTotalStep = (CT_ROTATE_TOTAL_ANGLE_DEFAULT / MOTOR_R_DEGREE_STEP);
    }

	CtParam.nReverseTotalStep=nRotateTotalStep;
	
	if(sysInfo.bShowLog==SET)
	{
		printUart(DBG_MSG_PC, "CtParam.nXrayOnStep(%d), CtParam.nXrayOffStep(%d)", CtParam.nXrayOnStep, CtParam.nXrayOffStep);
		printUart(DBG_MSG_PC, "nAccStep(%d), nDecStep(%d), nRotateTotalStep(%d), CtParam.nEndStep(%d)", \
					nAccStep, nDecStep, nRotateTotalStep, CtParam.nEndStep);
	}
	
    CtParam.nEndStep = nRotateTotalStep - END_STEP_MARGIN;

#if defined(USE_CT_XRAY_PULSED_MODE)
{
	double dFrameRate = 1000.0 /(Sensor_C.Param.SyncHighTime+Sensor_C.Param.SyncLowTime);
	printUart(DBG_MSG_PC, "Frame Rate : %f, Total Frame : %f", dFrameRate, dFrameRate * dTime);
}
#endif /* USE_CT_XRAY_PULSED_MODE */

#ifdef USE_CT_STITCH_MODE
CtCaptureProcess:

	gMachStat.bTubeCountFlag=RESET;
	if (!CAN_SendMessage(CAN_TUBE_COUNT_CHECK, 0, 0, 1000, 2))
    {
    	UART_SendMessage(DBG_MSG_PC, "Tube Count Fail.");
        CtCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
        
        return;
    }

#if defined(USE_CT_XRAY_PULSED_MODE)
	if (CtParam.TubeMode == CT_TUBE_PULSED)
	{
		if (!CAN_SendMessage(CAN_TUBE_PULSE_MODE_SET, 0, 0, 1000, 2))
		{
			CtCapture_CancelProcess();
			CurCaptureMode = CAPTURE_CANCEL;
			Motor_CTEnd();
			return;
		}
	}
	else
	{
		if (!CAN_SendMessage(CAN_TUBE_CONTI_MODE_SET, 0, 0, 1000, 2))
		{
			CtCapture_CancelProcess();
			CurCaptureMode = CAPTURE_CANCEL;
			Motor_CTEnd();
			return;
		}
	}
#else /* not USE_CT_XRAY_PULSED_MODE */    
    if (!CAN_SendMessage(CAN_TUBE_CONTI_MODE_SET, 0, 0, 1000, 2)) 
	{
		UART_SendMessage(DBG_MSG_PC, "Tube Continuous mode Fail.");
        CtCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
        Motor_CTEnd();
        return;
    }	
#endif /* USE_CT_XRAY_PULSED_MODE */

    if (!CAN_SendMessage(CAN_TUBE_KV_SET, CtParam.KV, 0, 1000, 2)) 
	{
		UART_SendMessage(DBG_MSG_PC, "Tube KV Send Fail.");
        CtCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
        Motor_CTEnd();
        return;
    }

    if (!CAN_SendMessage(CAN_TUBE_mA_SET, CtParam.mA, 0, 1000, 2)) 
	{
		UART_SendMessage(DBG_MSG_PC, "Tube mA Send Fail.");
        CtCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
        Motor_CTEnd();
        return;
    }

	if (!capture_count)
    	Motor_MoveStartPosition();

	if(sysInfo.bShowLog==SET)
	{
		printUart(DBG_MSG_PC, "CT ::TotalFrame = %d, dTime = %f ", CtParam.TotalFrame,dTime);
		printUart(DBG_MSG_PC, "CT ::kV = %d , mA = %d", CtParam.KV,CtParam.mA);
		printUart(DBG_MSG_PC, "CT Start Motor R::CurrentStep = %d", Motor_R.CurrentStep);
		printUart(DBG_MSG_PC, "CT Start Motor V::CurrentStep = %d", Motor_V.CurrentStep);
	}
	status = CtCapture_StartProcess();
    if (status == STATUS_CAPTURE_DONE) 
	{		
        CtCapture_EndProcess();
    } 
	else if (status == STATUS_FIRST_CAPTURE_DONE) 
	{
		capture_count++;
		goto CtCaptureProcess;
	} 
	else if (status == STATUS_CAPTURE_STOP) 
	{
#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
		typElapsedTimer.nExposeTime = nElapsedMilliSec(typElapsedTimer.nExposeTime);
#endif /* BUILD_TYPE_DEBUG && !USE_AGING_MODE */
        CtCapture_StopProcess(FALSE);
    }
	else if(status == STATUS_CAPTURE_STITCH_FAIL)
	{
		CtCapture_StopProcess(SET);
	}
	else
	{ //status == STATUS_CAPTURE_CANCEL
		//CtCapture_CancelProcess();
	}
#else /* not USE_CT_STITCH_MODE */
    Motor_R.CurrentStep = 0;

    Exposure_CtrlLed(EXPOSURE_LED_GPIO_ENABLE);

    UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_CONFI]");
    UART_SendMessage(DBG_MSG_PC, "[SP_STBY_EXPSW]");

    Indicator_Control(INDICATOR_ENABLE);

#if 1
    while(Exposure_CheckSwitch()) {
        if (CurCaptureMode != CAPTURE_CT) {
            CtCapture_CancelProcess();
            return;
        }
    }
#endif
    /* Verify X-ray interlock is safe before enabling tube */
    if (!Safety_VerifyXrayInterlock()) {
        printUart(DBG_MSG_PC, "SAFETY: X-ray interlock fault, aborting CT");
        CtCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
        return;
    }

    Tube_CtrlReady(TUBE_READY_ENABLE);

    UART_SendMessage(DBG_MSG_PC, "Wait for Tube Heatting Time");

    IntTimer_Delay(2000);
    while(!IntTimer_GetStatus());

    UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_START]");
#ifdef USE_TABLET_PC
	UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_START]");
#endif /* USE_TABLET_PC */

    MotorR_MoveAbsoluteDistance(&Motor_R, dTime, nAccStep, nRotateTotalStep, nDecStep);

    while(!Exposure_CheckSwitch()) {
        if (CtParam.bCaptEnd == SET) {
            bEnd = SET;
            break;
        }
    }

    Indicator_Control(INDICATOR_DISABLE);
	
    if (bEnd == SET) {
        CtCapture_EndProcess();
    } else {
#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
		typElapsedTimer.nExposeTime = nElapsedMilliSec(typElapsedTimer.nExposeTime);
#endif /* BUILD_TYPE_DEBUG && !USE_AGING_MODE */
        CtCapture_StopProcess(FALSE);
    }
#endif /* USE_CT_STITCH_MODE */	
}

/**
* @ Function Name : CtCapture_ClearParam
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void CtCapture_ClearParam(void)
{
    //CtParam.Sensor 			= CT_SENSOR_VARIAN;
    CtParam.KV 				= 80;
    CtParam.mA 				= 80; // 8mA
    //CtParam.Resolution 		= CT_RESOLUTION_100um;
    CtParam.Fov 			= CT_FOV_15X9;//CT_FOV_10X9;
    CtParam.Binning 		= CT_BINNING_1X1;
    //CtParam.Mar 			= CT_MAR_ON;
#if defined(USE_CT_XRAY_PULSED_MODE)
	CtParam.TubeMode		= CT_TUBE_PULSED;
	//CtParam.TubeMode		= CT_TUBE_CONTINUOUS;
#endif /* USE_CT_XRAY_PULSED_MODE */    
    CtParam.XonTime 		= 15;
	CtParam.XoffTime		= 1;
    CtParam.DelayTime 		= 1;
    CtParam.TotalFrame 		= 600;
    CtParam.nMotorVStartPos = CT_MOTORV_START_OFFSET_DEFAULT;
    CtParam.nExpStartPos	= EXPOSURE_START_POSITION;
    CtParam.nRunStep 		= 0;
    CtParam.nXrayOnStep 	= 0;
    CtParam.nXrayOffStep 	= 0;
    CtParam.nEndStep 		= 0;
    CtParam.bCaptEnd 		= FALSE;
	CtParam.nPaxisFovStep 	= 0;
	CtParam.bPaxisSET=RESET;
	CtParam.bCNSaxisSET=RESET;
	CtParam.bCWEaxisSET=RESET;
#ifdef USE_CT_STITCH_MODE
	CtParam.bStitchMode = FALSE;
	CtParam.bStitchBack = FALSE;
	CtParam.bStitchORGCheck = FALSE;
	dFps = 0;
	dTime = 0;
	nRotateTotalStep = 0;
	nAccStep = 0;
	nDecStep = 0;
	capture_count = 0;
#endif /* USE_CT_STITCH_MODE */	
	CtParam.bStitch_Status = StitchUnderSensor_Check();
}

/**
* @ Function Name : CtCapture_EndProcess
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void CtCapture_EndProcess(void)
{
    while(!Motor_GetStatus(&Motor_R, STATUS_STOP));

	
	UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_END__]");
#ifdef USE_TABLET_PC
	UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_END__]");
#endif /* USE_TABLET_PC */

	// 210202 HWAN DEFECT Protocol add
	if(CtParam.bFirstCT==RESET)
	{
		CtParam.bFirstCT=SET;
		UART_SendMessage(DBG_MSG_PC, "[SP_CT___DEFEC]");
	}

#ifdef USE_CT_STITCH_MODE
	if (CtParam.bStitchMode == TRUE || CtParam.bReverse_Status == SET) 
	{
		//printUart(DBG_MSG_PC, "CT END Current Step : %d", Motor_R.CurrentStep);
		//Motor_MoveAbsolutePosition(&Motor_R,6000,2000,9600,2000);
		if(CtParam.TotalFrame==400)
			Motor_SetDirMovePosistion(&Motor_R, DIR_LIMIT, 3000, 9600-CtParam.nFastRaxisOffset);
		else
			Motor_SetDirMovePosistion(&Motor_R, DIR_LIMIT, 3000, 9600-CtParam.nRaxisOffset);
			//Motor_R.CurrentStep-(400.0/(360.0/MOTOR_R_MICROSTEP__/MOTOR_R_PULLEY_RATIO))
			//(400.0/(360.0/MOTOR_R_MICROSTEP__/MOTOR_R_PULLEY_RATIO)) - Motor_R.CurrentStep);
		while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
	} 
	else 
	{
		//MotorR_InitMinFreq(2000);
		Motor_MoveEndPosition();
	}
#else /* not USE_CT_STITCH_MODE */
	Motor_MoveEndPosition();
#endif /* USE_CT_STITCH_MODE */	
	CurCaptureMode = CAPTURE_CANCEL;
	Motor_CTEnd();
	//printUart(DBG_MSG_PC, " END Current Step : %d", Motor_R.CurrentStep);
	if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "CT END Motor R::CurrentStep = %d", Motor_R.CurrentStep);
	}
    
	

#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
	printUart(DBG_MSG_PC, "Exposed Time : %d(ms)", typElapsedTimer.nExposeTime);
    ElapseTimerOnOff(FALSE);
#endif /* BUILD_TYPE_DEBUG && !USE_AGING_MODE */
	if(CtParam.bStitchBack==TRUE)
	{
		Motor_MoveStitchPosition(BOTTOM_UP,317,325);
	}
	CtCapture_ClearProcess();
}

/**
* @ Function Name : CtCapture_StopProcess
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void CtCapture_StopProcess(bool Status)
{    
    Tube_CtrlReady(TUBE_READY_DISABLE);
    Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);

    Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);

	CurCaptureMode = CAPTURE_CANCEL;
	Motor_CTEnd();

    UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_STOP_]");
#ifdef USE_TABLET_PC
    UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_STOP_]");
#endif /* USE_TABLET_PC */

    

#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
	printUart(DBG_MSG_PC, "Exposed Time : %d(ms)", typElapsedTimer.nExposeTime);
    ElapseTimerOnOff(FALSE);
#endif /* BUILD_TYPE_DEBUG && !USE_AGING_MODE */

    if(Status==FALSE) UART_SendMessage(DBG_MSG_PC, ERR_CODE_CT_CAPT_STOP);

    if (MotorR_CheckOrgSequence() == SET) 
	{
		if(sysInfo.bShowLog==TRUE)
		{
        	printUart(DBG_MSG_PC, "R-Axis Org Sequence 2 : CurStep(%d)", Motor_R.CurrentStep);
		}
    }
    if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "CT Stop Motor R::CurrentStep = %d", Motor_R.CurrentStep);
	}

    CtCapture_CancelProcess();
}

/**
* @ Function Name : CtCapture_CancelProcess
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void CtCapture_CancelProcess(void)
{
    UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_CANCE]"); 
#ifdef USE_TABLET_PC
    UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_CANCE]");
#endif /* USE_TABLET_PC */

	CtCapture_ClearProcess();
}

/**
* @ Function Name : CtCapture_ClearProcess
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void CtCapture_ClearProcess(void)
{
	if(sysInfo.bShowLog==TRUE)
	{
		UART_SendMessage(DBG_MSG_PC, "Run CtCapture_ClearProcess");
	}
	//200121 HWAN TEST
	//Timer_Config(&Motor_R.Timer);
	
	//MotorR_InitMinFreq(Motor_R.MinFreq);

    CtCapture_ClearParam();

	if(sysInfo.bXrayNoSound==TRUE)
	{
		sysInfo.bXrayNoSound=FALSE;
		CAN_SendMessage(CAN_TUBE_SOUND_ON, 0, 0, 1000, 2);
	}
	
    Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
	
	if(gMachStat.bTubeCountCompareFlag==TRUE)
	{
		gMachStat.bTubeCountFlag=SET;
		if (!CAN_SendMessage(CAN_TUBE_COUNT_CHECK, 0, 0, 1000, 2))
	    {
	    	UART_SendMessage(DBG_MSG_PC, "Tube Count Fail.");
	        CtCapture_CancelProcess();
	        CurCaptureMode = CAPTURE_CANCEL;
	        
	        return;
	    }
		if(gMachStat.nTubeOldCount==gMachStat.nTubeCount)
		{
			UART_SendMessage(DBG_MSG_PC, "Warning : Generator Doesn't Exposed");
		}
	}
	gMachStat.bTubeCountCompareFlag=FALSE;

#ifdef USE_MOTOR_CHINREST_TS
    Motor_MoveTempleSupport(FALSE);
#endif /* USE_MOTOR_CHINREST_TS */

#if defined(USE_CT_XRAY_PULSED_MODE) && defined(USE_SENSOR_USER_SYNC)
	//if (CtParam.TubeMode == CT_TUBE_PULSED)
	{
		while(Sensor_C.Update != FALSE);
		CtSensor_Stop();
	}
#endif /* USE_CT_XRAY_PULSED_MODE && USE_SENSOR_USER_SYNC */

	//�Կ��� ����Ǹ� ���� ������ ���� => �ϴ� ����
    //PanoSensor_PowerControl(FALSE);	

#ifdef USE_AGING_MODE
	PrevCaptureMode = CAPTURE_CT;
	g_nCtCnt++;

	SysTime_MesureEnd(__FUNCTION__, __LINE__);	
#endif /* USE_AGING_MODE */

    LampTimer_Enable();
}

#ifdef USE_CT_STITCH_MODE
/**
* @ Function Name : CtCapture_StartProcess
* @ Desc : 
* @ Param : 
* @ Return :
*/
static Capture_Status CtCapture_StartProcess(void)
{
	BitAction check;
	Capture_Status ret = STATUS_CAPTURE_STOP;
	char bReverse = FALSE;
	bool Stitch_success = TRUE;
	
	if ( (CtParam.bStitchMode == TRUE) && (capture_count) ) 
	{
		//Stitch_success=Motor_MoveStitchPosition(TOP_DOWN,317,325);
		Stitch_success=Motor_MoveStitchPosition(TOP_DOWN,317,325);
		
		if(Stitch_success==FALSE)
		{
			return STATUS_CAPTURE_STITCH_FAIL;
		}
		bReverse = TRUE;
		
#define	UART_2ND_CAPTURE_CONFIRM			"[SM_CAPT_CONFI]"
		while (!UART_WaitProtocol(UART_2ND_CAPTURE_CONFIRM)) 
		{
			if (CurCaptureMode == CAPTURE_CANCEL)
			{
				//Ȯ�� �ʿ� - ��ġ �ٲ�
				//Motor_CTEnd();
				CtCapture_CancelProcess();				
				return STATUS_CAPTURE_CANCEL;
			}
		}
		CtParam.bStitchORGCheck=SET;
	}

	Motor_ClearCurrentStep(&Motor_R);
	
	if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "CurrentStep(%d), capture_count(%d)", Motor_R.CurrentStep, capture_count);
		printUart(DBG_MSG_PC, "nAccStep(%d), nDecStep(%d), nRotateTotalStep(%d), CtParam.nEndStep(%d)", \
					nAccStep, nDecStep, nRotateTotalStep, CtParam.nEndStep);
	}

	Exposure_CtrlLed(EXPOSURE_LED_GPIO_ENABLE);

	UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_CONFI]");
	UART_SendMessage(DBG_MSG_PC, "[SP_STBY_EXPSW]");

	Indicator_Control(INDICATOR_ENABLE);

	Timer_Config_T(&Motor_R.Timer);
#if 0
	while (!UART_WaitProtocol("[SP_CAPT_START]")) {
		if (CurCaptureMode == CAPTURE_CANCEL)
			return STATUS_CAPTURE_CANCEL;
	}
#else
	while(Exposure_CheckSwitch()&& UART_Exposure_SET()!=1)
	{
		if (CurCaptureMode != CAPTURE_CT) 
		{
			Motor_CTEnd();
			CtCapture_CancelProcess();
			return STATUS_CAPTURE_CANCEL;
		}
	}
#endif

	check = (BitAction)GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7);

	/* Verify X-ray interlock is safe before enabling tube (stitch path) */
	if (!Safety_VerifyXrayInterlock()) {
		printUart(DBG_MSG_PC, "SAFETY: X-ray interlock fault, aborting CT stitch");
		Motor_CTEnd();
		CtCapture_CancelProcess();
		return STATUS_CAPTURE_CANCEL;
	}

	Tube_CtrlReady(TUBE_READY_ENABLE);

	UART_SendMessage(DBG_MSG_PC, "Wait for Tube Heatting Time");	

	IntTimer_Delay(2000);
	while(!IntTimer_GetStatus());

	UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_START]");
#ifdef USE_TABLET_PC
	UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_START]");
#endif /* USE_TABLET_PC */

	if(CtParam.bReverse_Status==SET) bReverse=SET;
	gMachStat.bTubeCountCompareFlag=TRUE;
	MotorR_MoveAbsoluteDistance(&Motor_R, dTime, nAccStep, nRotateTotalStep, nDecStep, bReverse);

	if(check==Bit_SET)
	{
		while(UART_Exposure_SET()!=2)
	    {
#ifdef USE_TUBE_PPS_TYPE_IO
			if (Sensor_C.bSetFrameCount == TRUE)
			{
				if (PanoSensor_CheckInput()) 
				{
					Tube_CtrlPps(Bit_RESET); //X-Ray Off
				} 
				else 
				{
					Tube_CtrlPps(Bit_SET); // X-Ray On
				}
			}
#endif /* USE_TUBE_PPS_TYPE_IO */
		
			if (CtParam.bCaptEnd == TRUE)
				break;
	    }
	}
	else
	{
		while(!Exposure_CheckSwitch())
	    {
#ifdef USE_TUBE_PPS_TYPE_IO
			if (Sensor_C.bSetFrameCount == TRUE)
			{
				if (PanoSensor_CheckInput()) 
				{
					Tube_CtrlPps(Bit_RESET); //X-Ray Off
				} 
				else 
				{
					Tube_CtrlPps(Bit_SET); // X-Ray On
				}
			}
#endif /* USE_TUBE_PPS_TYPE_IO */
		
			if (CtParam.bCaptEnd == TRUE)
				break;
	    }
	}

	Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
	Indicator_Control(INDICATOR_DISABLE);
	if ( (CtParam.bStitchMode == TRUE) && (!capture_count) &&  CtParam.bCaptEnd==TRUE) 
	{		
		while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
		//190925 HWAN
		CurCaptureMode=CAPTURE_CANCEL;
		Motor_CTEnd();
		CurCaptureMode=CAPTURE_CT;
		CtParam.bCaptEnd = FALSE;				
		UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_END__]");
		//201120 HWAN First 15X15 end for touch pad
#ifdef USE_TABLET_PC
		UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_END_F]");
#endif /* USE_TABLET_PC */
		ret = STATUS_FIRST_CAPTURE_DONE;
	}
	else if(CtParam.bCaptEnd==TRUE)
	{
		ret = STATUS_CAPTURE_DONE;
	}
	else
	{
		CtCapture_StopProcess(FALSE);
		ret = STATUS_CAPTURE_CANCEL;
	}

	while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
	if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "Current Step : %d", Motor_R.CurrentStep);
	}

	return ret;
}
#endif /* USE_CT_STITCH_MODE */


/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/

