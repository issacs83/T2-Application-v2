/*
*********************************************************************************************
* scan_capture.c :
*********************************************************************************************
* Copyright (C) 2014-2016 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date : 
* @ Brief :
*
* @ Revision History :
*       1) initial creation. -------------------------------------- 2014-11-16
*       2) V1.1.0  ------------------------------------------- 2015-03-20
*       3) V1.2.0  ------------------------------------------- 2015-09-18
*       4) V1.3.0  ------------------------------------------- 2015-10-28
*********************************************************************************************
*/

/* Include files -------------------------------------------------------- */
#include "extern.h"
#include "sensor.h"
#include "system.h"
#include "timer.h"
#include "error_code.h"
#include "motor.h"
#include "can.h"
#include "tube.h"
#include "serial.h"
#include "misc1.h"
#include "safety.h"
#include "motor_compat.h"


/* Private typedef ----------------------------------------------------- */
/* Private define ------------------------------------------------------ */
#define UART_SCAN_WAIT_2SEC_CONFI		  		"[SM_2SEC_CONFI]"

#define	CEPH_SCAN_TOTAL_DISTANCE 	        	300 //mm
#define	CEPH_2SEC_SCAN_STOP_DISTANCE 	    	200 //mm

#define	CEPH_SCAN_TIME_10SEC 	    			10000 //ms
#define	CEPH_SCAN_TIME_5SEC 				    5000 //ms
#define	CEPH_SCAN_TIME_2SEC 				    2000 //ms


/* Private macro ------------------------------------------------------ */
/* Private variables ---------------------------------------------------- */
CephParam_Typedef CephParam;
//jehun
//bool g_bTiltStatus;


/* Private function prototypes --------------------------------------------*/
static void ScanCapture_ClearParam(void);
static void ScanCapture_EndProcess(void);
static void ScanCapture_StopProcess(void);
static void ScanCapture_CancelProcess(void);
static void ScanCapture_ClearProcess(void);



/* Private functions ---------------------------------------------------- */

/**
* @ Function Name : ScanCapture
* @ Desc : 
* @ Param : 
* @ Return :
*/
void ScanCapture(void)
{
    double dPulse = 0,StepPerCm=0;
    uint32_t nStartCnt = 0;
    uint32_t nEndCnt = 0;
    bool bStop = FALSE;
    uint32_t nTwoSecEndCnt = 0;
	uint32_t nDistance = 0;
	BitAction check;

#ifdef USE_AGING_MODE
	printUart(DBG_MSG_PC, "Cephalo Aging Count :: %d", g_nCephCnt);

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
		ScanCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
		PrevCaptureMode = CAPTURE_CANCEL;
#endif /* USE_AGING_MODE */		
        return;
    }

    //ScanSensor_PowerControl(TRUE);	

    UART_SendMessage(DBG_MSG_PC, "[SP_MODE_SCAN_]");
#ifdef USE_TABLET_PC
    UART_SendMessage(DBG_MSG_TABLET, "[SP_MODE_SCAN_]");
#endif /* USE_TABLET_PC */

	if (g_bColumnPressed == SET)
	{
		Column_Control(COLUMN_STOP);
		g_bColumnPressed = FALSE;
	}

	if (CAN_SendMessage(CAN_TUBE_COM_CHECK, CAN_TUBE_GEN2_COM, 0, 3000, 2))
	{
		CAN_SendMessage(CAN_TUBE_TANK_TEMP_READ, 0, 0, 1000, 2);
	}
	else
	{
		UART_SendMessage(DBG_MSG_PC, "Tube Communication Fail.");
		ScanCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
		PrevCaptureMode = CAPTURE_CANCEL;
#endif /* USE_AGING_MODE */		
        return;
	}

    if (!CAN_Collimator_SendMessage(CMD_TILT_OFF, 0, 0, 5000, 2))
    {
    	UART_SendMessage(DBG_MSG_PC, "Collimator Communication Fail.");
		ScanCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
		PrevCaptureMode = CAPTURE_CANCEL;
#endif /* USE_AGING_MODE */	

        return;
    }
    g_bTiltStatus = FALSE;

    ScanCapture_ClearParam();

    if (!Motor_MoveInitPosition())
    {
    	UART_SendMessage(DBG_MSG_PC, "Motor Init Position Fail.");
        ScanCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
		PrevCaptureMode = CAPTURE_CANCEL;
#endif /* USE_AGING_MODE */        
        return;
    }
	//Motor_MoveAlignPosition();

    if (CurCaptureMode != CAPTURE_SCAN)
    {
        ScanCapture_CancelProcess();
#ifdef USE_AGING_MODE
		PrevCaptureMode = CAPTURE_CANCEL;
#endif /* USE_AGING_MODE */

        return;
    }

#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
    ElapseTimerOnOff(TRUE);
#endif /* BUILD_TYPE_DEBUG && !USE_AGING_MODE */

    if (!CAN_Collimator_SendMessage(CMD_TILT_ON, 0, 0, 5000, 2))
    {
    	UART_SendMessage(DBG_MSG_PC, "Tilt Fail.");
		ScanCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
		PrevCaptureMode = CAPTURE_CANCEL;
#endif /* USE_AGING_MODE */

        return;
    }
    g_bTiltStatus = TRUE;

	LaserSetPosition();
	LaserControl(TYPE_HEAD_CEPH, TRUE);

	if (sysInfo.initDir_MotCC == INIT_DIR_REVERSE)
    {
        CephParam.bInitPosFlag = FALSE;
    }

    UART_SendMessage(DBG_MSG_PC, "[SP_SCAN_READY]");
#ifdef USE_TABLET_PC
    UART_SendMessage(DBG_MSG_TABLET, "[SP_SCAN_READY]");
#endif /* USE_TABLET_PC */
	if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "Cephalo Init Motor R::CurrentStep = %d", Motor_R.CurrentStep);
		printUart(DBG_MSG_PC, "Cephalo Init Motor V::CurrentStep = %d", Motor_V.CurrentStep);
		printUart(DBG_MSG_PC, "Cephalo Init Motor A::CurrentStep = %d", Motor_A.CurrentStep);
		printUart(DBG_MSG_PC, "Cephalo Init Motor CC::CurrentStep = %d", Motor_C.CurrentStep);
		printUart(DBG_MSG_PC, "Cephalo Init Motor CS::CurrentStep = %d", Motor_S.CurrentStep);
	}
#ifdef USE_AGING_MODE
	if (g_bAgingMode != TRUE) {
		while(!UART_SetCephParam() && CurCaptureMode != CAPTURE_CANCEL);
	}
#else /* not USE_AGING_MODE */
    while(!UART_SetCephParam() && CurCaptureMode != CAPTURE_CANCEL);
#endif /* USE_AGING_MODE */

	Motor_Ceph_StartPosition();

	if (g_bColumnPressed == TRUE)
	{
		Column_Control(COLUMN_STOP);
		g_bColumnPressed = FALSE;
	}
    
	LaserControl(TYPE_HEAD_CEPH, FALSE);

	if (!bIsExposureSwitchReleased(500)) 
	{
		UART_SendMessage(DBG_MSG_PC, ERR_CODE_EXP_SWITCH);
        CurCaptureMode = CAPTURE_CANCEL;
	}

    if (CurCaptureMode != CAPTURE_SCAN)
    {
        ScanCapture_CancelProcess();
        return;
    }

    Lamp_Control(LAMP_RED, LAMP_DISABLE);
    Lamp_Control(LAMP_GREEN, LAMP_ENABLE);
    Lamp_Control(LAMP_BLUE, LAMP_DISABLE);    
    
    if (!CAN_Collimator_SendMessage(CMD_COLLI_OPEN, 0, 0, 1000, 2))
    {
    	UART_SendMessage(DBG_MSG_PC, "Collimator Open Fail.");
        ScanCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
        
        return;
    } 

	if (CephParam.bScanAlign == FALSE)
	{
	    if (!CAN_Collimator_SendMessage(CMD_COLLI_CEPHSCAN_INIT, 0, 0, 1000, 2))
	    {
	    	UART_SendMessage(DBG_MSG_PC, "Collimator CEPHSCAN_INI Fail.");
	        ScanCapture_CancelProcess();
	        CurCaptureMode = CAPTURE_CANCEL;
	        
	        return;
	    }
	}

	
    if (!CAN_Collimator_SendMessage(CMD_FILTER_CENTER, 0, 0, 1000, 2))
    {
    	UART_SendMessage(DBG_MSG_PC, "Collimator filter move Fail.");
        ScanCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
        
        return;
    }  

    nDistance = CephParam.Size; // 300
    if (CephParam.Time == CEPH_SCAN_TIME_2SEC) // 2초는 삭제됨
	{
        nDistance = CEPH_2SEC_SCAN_STOP_DISTANCE;
	}

    Motor_C.RunParam.RunFreq = (nDistance * MOTOR_C_MICROSTEP) / (CephParam.Time * msec_To_sec * MOTOR_C_PULLEY_DIAMETER * pi);

    dPulse = MOTOR_C_PULLEY_DIAMETER * pi / MOTOR_C_MICROSTEP; //mm_per_pulse
    nStartCnt = 10; //15;
	nEndCnt = nDistance / dPulse;
	
    if (CephParam.Time == CEPH_SCAN_TIME_2SEC)
	{
		nTwoSecEndCnt = CEPH_2SEC_SCAN_STOP_DISTANCE / dPulse;
        nEndCnt = CEPH_SCAN_TOTAL_DISTANCE / dPulse;
	}
	//190830 HWAN Exposure length change
	StepPerCm=(double)nEndCnt/(double)30;
	
	switch(CephParam.Mode)
	{
		case Ceph_Full_Lateral:
			if(CephParam.Type==Ceph_Adult)
			{
				nEndCnt = nEndCnt-StepPerCm*2;
				//nEndCnt = nEndCnt-1862;
			}
			else
			{
				nStartCnt = StepPerCm*0.5;
				nEndCnt = nEndCnt-StepPerCm*3.5;
				if (!CAN_Collimator_SendMessage(CMD_COL_CEPHSCAN_CHILD, 0, 0, 1000, 2))
			    {
			    	UART_SendMessage(DBG_MSG_PC, "Collimator cephalo child move Fail.");
			        ScanCapture_CancelProcess();
			        CurCaptureMode = CAPTURE_CANCEL;
			        
			        return;
			    }
			}
			break;
		case Ceph_Lateral:
			if(CephParam.Type==Ceph_Adult)
			{
				nEndCnt = nEndCnt-StepPerCm*9;
				//nEndCnt = nEndCnt-3714;
			}
			else
			{
				nStartCnt = StepPerCm*0.5;
				nEndCnt = nEndCnt-StepPerCm*10;
				if (!CAN_Collimator_SendMessage(CMD_COL_CEPHSCAN_CHILD, 0, 0, 1000, 2))
			    {
			    	UART_SendMessage(DBG_MSG_PC, "Collimator cephalo child move Fail.");
			        ScanCapture_CancelProcess();
			        CurCaptureMode = CAPTURE_CANCEL;
			        
			        return;
			    }
			}
			break;		
		/*case Ceph_Carpus:
			nStartCnt = StepPerCm*4.5;
			nEndCnt = nEndCnt-StepPerCm*4.5;
			if (!CAN_Collimator_SendMessage(CMD_COLLI_CEPH_CAPUS, 0, 0, 1000, 2))
		    {
		        ScanCapture_CancelProcess();
		        CurCaptureMode = CAPTURE_CANCEL;
		        
		        return;
		    }
			//nEndCnt = nEndCnt-3251;
			break;*/
		case Ceph_Carpus:
		case Ceph_PA:
		case Ceph_AP:
		case Ceph_Waters:
		case Ceph_SVM:
			nStartCnt = StepPerCm*3;
			nEndCnt = nEndCnt-StepPerCm*3;
			//nEndCnt = nEndCnt-1862;
			if (!CAN_Collimator_SendMessage(CMD_COLLI_CEPH_AP_PA, 0, 0, 1000, 2))
		    {
		    	UART_SendMessage(DBG_MSG_PC, "Collimator cephalo AP PA move Fail.");
		        ScanCapture_CancelProcess();
		        CurCaptureMode = CAPTURE_CANCEL;
		        
		        return;
		    }
			break;
		case Ceph_EarLod:
			nStartCnt = StepPerCm*10;
			nEndCnt = nEndCnt - StepPerCm*10;
			break;
		default:
			break;
	}


	gMachStat.bTubeCountFlag=RESET;
	if (!CAN_SendMessage(CAN_TUBE_COUNT_CHECK, 0, 0, 1000, 2))
    {
    	UART_SendMessage(DBG_MSG_PC, "Tube Count Fail.");
        ScanCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
        
        return;
    }
	
    if (!CAN_SendMessage(CAN_TUBE_CONTI_MODE_SET, 0, 0, 1000, 2))
    {
    	UART_SendMessage(DBG_MSG_PC, "Tube Continuous mode Fail.");
        ScanCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
        
        return;
    }

    if (!CAN_SendMessage(CAN_TUBE_KV_SET, CephParam.KV, 0, 1000, 2))
    {
    	UART_SendMessage(DBG_MSG_PC, "Tube KV Send Fail.");
        ScanCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
        
        return;
    }

    if (!CAN_SendMessage(CAN_TUBE_mA_SET, CephParam.mA, 0, 1000, 2))
    {
    	UART_SendMessage(DBG_MSG_PC, "Tube mA Send Fail.");
        ScanCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
        
        return;
    }

    if (CephParam.Time == CEPH_SCAN_TIME_2SEC)
    {
        while (!UART_WaitProtocol(UART_SCAN_WAIT_2SEC_CONFI))
        {
            if (CurCaptureMode == CAPTURE_CANCEL)
            {
                ScanCapture_CancelProcess();
                return;
            }
        }
    }
	
	if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "Cephalo ::Mode = %d, Type = %d, Time=%d", CephParam.Mode,CephParam.Type,CephParam.Time);
		printUart(DBG_MSG_PC, "Cephalo ::kV = %d, mA = %d", CephParam.KV,CephParam.mA);
		printUart(DBG_MSG_PC, "Cephalo Start Motor CC::CurrentStep = %d", Motor_C.CurrentStep);
		printUart(DBG_MSG_PC, "Cephalo Start Motor CS::CurrentStep = %d", Motor_S.CurrentStep);
		printUart(DBG_MSG_PC, "Cephalo Start Cnt = %d, End Cnt = %d", nStartCnt,nEndCnt);
	}
	
    Exposure_CtrlLed(EXPOSURE_LED_GPIO_ENABLE);

    UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_CONFI]");
    UART_SendMessage(DBG_MSG_PC, "[SP_STBY_EXPSW]");

    Indicator_Control(INDICATOR_ENABLE);

    while(Exposure_CheckSwitch() &&UART_Exposure_SET()!=1)
    {
        if (CurCaptureMode != CAPTURE_SCAN)
        {
            ScanCapture_CancelProcess();
            return;
        }
    }

	check = (BitAction)GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7);	

    /* Verify X-ray interlock is safe before enabling tube */
    if (!Safety_VerifyXrayInterlock()) {
        printUart(DBG_MSG_PC, "SAFETY: X-ray interlock fault, aborting scan");
        ScanCapture_CancelProcess();
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
	gMachStat.bTubeCountCompareFlag=TRUE;

	if (CephParam.bScanAlign == FALSE)
	{
	    if (CephParam.Time == CEPH_SCAN_TIME_10SEC) // Normal Scan : 10 Sec
	    {
	        if (!CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB, CMD_COL_CEPHSCAN_START_NORMAL, 0x96, 0))
	        {
	            printUart(DBG_MSG_PC, "Collimator CEPHSCAN_START_NORMAL failed...!");
	        }   
	    }
	    else if ( (CephParam.Time == CEPH_SCAN_TIME_5SEC) /*|| (CephParam.Time == CEPH_SCAN_TIME_2SEC)*/ )
	    {
	        if (!CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB, CMD_COL_CEPHSCAN_START_FAST, 0x96, 0))
	        {
	            printUart(DBG_MSG_PC, "Collimator CEPHSCAN_START_FAST failed...!");
	        }   
	    }
	    else if (CephParam.Time == CEPH_SCAN_TIME_2SEC)
	    {
	        if (!CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB, CMD_COL_CEPHSCAN_START_2SEC, 0x32, 0))
	        {
	            UART_SendMessage(DBG_MSG_PC, "Collimator CEPHSCAN_START_FAST failed...!");
	        }   
	    }
	    else
	    {
	        printUart(DBG_MSG_PC, "Scan Time Error!");
	    }
	}

	Motor_CephScanStart(nEndCnt);

	if(check==Bit_SET)	//Switch release
	{
		while(Motor_C.CurrentStep < nEndCnt + 100)
		{
			if (UART_Exposure_SET()==2)
			{
				bStop = TRUE;
				break;
			}
			if (Motor_C.CurrentStep == nStartCnt)
			{
				Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
				typElapsedTimer.nExposeTime = vSetCurrentMilliSec();
#endif /* BUILD_TYPE_DEBUG && !USE_AGING_MODE */
			}
			else if (Motor_C.CurrentStep == nStartCnt + 5)
			{
				ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_ENABLE);
			}
			else if (Motor_C.CurrentStep >= nTwoSecEndCnt && CephParam.Time == 2000 && nDistance == 200)
			{
				ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE);
				Tube_CtrlReady(TUBE_READY_DISABLE);
				Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
				typElapsedTimer.nExposeTime = nElapsedMilliSec(typElapsedTimer.nExposeTime);
#endif /* BUILD_TYPE_DEBUG && !USE_AGING_MODE */
				break;
			}
			else if (Motor_C.CurrentStep >= nEndCnt && !(CephParam.Time == 2000 && nDistance == 200) ) // hsyou
			{
				ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE);
				Tube_CtrlReady(TUBE_READY_DISABLE);
				Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
				typElapsedTimer.nExposeTime = nElapsedMilliSec(typElapsedTimer.nExposeTime);
#endif /* BUILD_TYPE_DEBUG && !USE_AGING_MODE */
			}
		}

	}
	else
	{
		while(Motor_C.CurrentStep < nEndCnt + 100)
	    {
	        if ((Exposure_CheckSwitch()))
	        {
	            bStop = TRUE;
	            break;
	        }
	        if (Motor_C.CurrentStep == nStartCnt)
	        {
	            Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
				typElapsedTimer.nExposeTime = vSetCurrentMilliSec();
#endif /* BUILD_TYPE_DEBUG && !USE_AGING_MODE */
	        }
			else if (Motor_C.CurrentStep == nStartCnt + 5)
			{
	            ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_ENABLE);
			}
			else if (Motor_C.CurrentStep >= nTwoSecEndCnt && CephParam.Time == 2000 && nDistance == 200)
			{
				ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE);
	            Tube_CtrlReady(TUBE_READY_DISABLE);
	            Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
				typElapsedTimer.nExposeTime = nElapsedMilliSec(typElapsedTimer.nExposeTime);
#endif /* BUILD_TYPE_DEBUG && !USE_AGING_MODE */
				break;
			}
	        else if (Motor_C.CurrentStep >= nEndCnt && !(CephParam.Time == 2000 && nDistance == 200) ) // hsyou
	        {
		        ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE);
	            Tube_CtrlReady(TUBE_READY_DISABLE);
	            Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
	            typElapsedTimer.nExposeTime = nElapsedMilliSec(typElapsedTimer.nExposeTime);
#endif /* BUILD_TYPE_DEBUG && !USE_AGING_MODE */
	        }
	    }		
	}

	Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
    Indicator_Control(INDICATOR_DISABLE);



    if (!bStop)
    {
        ScanCapture_EndProcess();
    }
    else
    {
#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
		typElapsedTimer.nExposeTime = nElapsedMilliSec(typElapsedTimer.nExposeTime);
#endif /* BUILD_TYPE_DEBUG && !USE_AGING_MODE */
        ScanCapture_StopProcess();
    }

    //ScanSensor_PowerControl(FALSE);	
	
}    

/**
* @ Function Name : ScanCapture_ClearParam
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void ScanCapture_ClearParam(void)
{
#ifdef USE_AGING_MODE
    CephParam.KV = 80;
    CephParam.mA = 80;
#else /* USE_AGING_MODE */
    CephParam.KV = 0;
    CephParam.mA = 0;
#endif /* USE_AGING_MODE */
    CephParam.Time = CEPH_SCAN_TIME_10SEC;
    CephParam.Size = CEPH_SCAN_TOTAL_DISTANCE;
	CephParam.Mode = Ceph_Lateral;
	CephParam.Type= Ceph_Adult;
	CephParam.bScanAlign = FALSE;
	CephParam.bScanEar = FALSE;
}

/**
* @ Function Name : ScanCapture_EndProcess
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void ScanCapture_EndProcess(void)
{
    while(!Motor_GetStatus(&Motor_C, STATUS_STOP));	
    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
	
    UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_END__]");
#ifdef USE_TABLET_PC
    UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_END__]");
#endif /* USE_TABLET_PC */

	Motor_CephScanEnd();
	
	if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "Cephalo END Motor CC::CurrentStep = %d", Motor_C.CurrentStep);
		printUart(DBG_MSG_PC, "Cephalo END Motor CS::CurrentStep = %d", Motor_S.CurrentStep);
	}

    CurCaptureMode = CAPTURE_CANCEL;

#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
	printUart(DBG_MSG_PC, "Exposed Time : %d(ms)", typElapsedTimer.nExposeTime);
    ElapseTimerOnOff(FALSE);
#endif /* BUILD_TYPE_DEBUG && !USE_AGING_MODE */

    ScanCapture_ClearProcess();
}

/**
* @ Function Name : ScanCapture_StopProcess
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void ScanCapture_StopProcess(void)
{
    Tube_CtrlReady(TUBE_READY_DISABLE);
    Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);

    Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);

    ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE);

	Motor_CephScanEnd();

	if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "Cephalo Stop Motor CC::CurrentStep = %d", Motor_C.CurrentStep);
		printUart(DBG_MSG_PC, "Cephalo Stop Motor CS::CurrentStep = %d", Motor_S.CurrentStep);
	}
	
    UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_STOP_]");
#ifdef USE_TABLET_PC
    UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_STOP_]");	
#endif /* USE_TABLET_PC */

    CurCaptureMode = CAPTURE_CANCEL;

#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
	printUart(DBG_MSG_PC, "Exposed Time : %d(ms)", typElapsedTimer.nExposeTime);
	ElapseTimerOnOff(FALSE);
#endif /* BUILD_TYPE_DEBUG && !USE_AGING_MODE */

    UART_SendMessage(DBG_MSG_PC, ERR_CODE_SCAN_CAPT_STOP);

    if (sysInfo.initDir_MotCC == INIT_DIR_REVERSE)
    {
        CephParam.bInitPosFlag = FALSE;
    }

    ScanCapture_CancelProcess();
}

/**
* @ Function Name : ScanCapture_CancelProcess
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void ScanCapture_CancelProcess(void)
{
    //ScanSensor_PowerControl(FALSE);	
	
    UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_CANCE]");
#ifdef USE_TABLET_PC
    UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_CANCE]");
#endif /* USE_TABLET_PC */

	ScanCapture_ClearProcess();
}

/**
* @ Function Name : ScanCapture_ClearProcess
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void ScanCapture_ClearProcess(void)
{
	UART_SendMessage(DBG_MSG_PC, "Run ScanCapture_ClearProcess");

    ScanCapture_ClearParam();
	
	if(sysInfo.bXrayNoSound==TRUE)
	{
		sysInfo.bXrayNoSound=FALSE;
		CAN_SendMessage(CAN_TUBE_SOUND_ON, 0, 0, 1000, 2);
	}

	if(gMachStat.bTubeCountCompareFlag==TRUE)
	{
		gMachStat.bTubeCountFlag=SET;
		if (!CAN_SendMessage(CAN_TUBE_COUNT_CHECK, 0, 0, 1000, 2))
	    {
	    	UART_SendMessage(DBG_MSG_PC, "Tube Count Fail.");
	        ScanCapture_CancelProcess();
	        CurCaptureMode = CAPTURE_CANCEL;
	        
	        return;
	    }
		if(gMachStat.nTubeOldCount==gMachStat.nTubeCount)
		{
			UART_SendMessage(DBG_MSG_PC, "Warning : Generator Doesn't Exposed");
		}
	}
	gMachStat.bTubeCountCompareFlag=FALSE;
    //Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);

#ifdef USE_AGING_MODE
	PrevCaptureMode = CAPTURE_SCAN;
	g_nCephCnt++;

	SysTime_MesureEnd(__FUNCTION__, __LINE__);	
#endif /* USE_AGING_MODE */

    LampTimer_Enable();
}


/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/

