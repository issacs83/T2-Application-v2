/*
*******************************************************************************************
* pano_capture.c :
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
#include "system.h"
#include "error_code.h"
#include "motor.h"
#include "sensor.h"
#include "can.h"
#include "tube.h"
#include "serial.h"
#include "misc1.h"
#include "timer.h"
#include "safety.h"
#include "motor_compat.h"


/* Private typedef --------------------------------------------------- */
/* Private define ---------------------------------------------------- */
#define TMJ_CAPT_OFFSET 			85
#define LEFT_CAPT_OFFSET 			40
#define RIGHT_CAPT_OFFSET 			40
#define FRONT_CAPT_OFFSET 			90

#define	PANO_V_SINUS_OFFSET			22.0 / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP)
#define	PANO_V_TMJ_OFFSET			85.0 / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP)
#define	PANO_V_TMJ_POSITION			80.0 / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP)


/* Private macro ---------------------------------------------------- */
/* Private variables -------------------------------------------------- */
PanoParam_Typedef PanoParam;


/* Private function prototypes ------------------------------------------*/
static void PanoCapture_ClearParam(void);
static void PanoCapture_EndProcess(void);
static void PanoCapture_StopProcess(void);
static void PanoCapture_CancelProcess(void);
static void PanoCapture_ClearProcess(void);
static void PanoCapture_ModeOffset(void);


/* Private functions -------------------------------------------------- */

/**
* @ Function Name : PanoCapture
* @ Desc : 
* @ Param : 
* @ Return :
*/
void PanoCapture(void)
{
    bool stop = FALSE;
	BitAction check;
	
#ifdef USE_AGING_MODE
	printUart(DBG_MSG_PC, "Panorama Aging Count :: %d", g_nPanoCnt);

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
		PanoCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
				PrevCaptureMode = CAPTURE_CANCEL;
#endif /* USE_AGING_MODE */

        return;
    }

	//촬열할 때만 센서 전원을 킨다 => 일단 보류
    //PanoSensor_PowerControl(TRUE);	

	if(PanoParam.Init_Mode==PANO_NORMAL)
	{
	    UART_SendMessage(DBG_MSG_PC, "[SP_MODE_PANO_]");
#ifdef USE_TABLET_PC
	    UART_SendMessage(DBG_MSG_TABLET, "[SP_MODE_PANO_]");
#endif /* USE_TABLET_PC */
	}
	else if(PanoParam.Init_Mode==PANO_TMJ)
	{
		UART_SendMessage(DBG_MSG_PC, "[SP_MODE_PANOR]");
#ifdef USE_TABLET_PC
	    UART_SendMessage(DBG_MSG_TABLET, "[SP_MODE_PANOR]");
#endif /* USE_TABLET_PC */
	}

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
		PanoCapture_CancelProcess();
		CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
				PrevCaptureMode = CAPTURE_CANCEL;
#endif /* USE_AGING_MODE */

		return;
	}

    if (!CAN_Collimator_SendMessage(CMD_TILT_OFF, 0, 0, 5000, 2))
    {
    	UART_SendMessage(DBG_MSG_PC, "Collimator Communication Fail.");
        PanoCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
				PrevCaptureMode = CAPTURE_CANCEL;
#endif /* USE_AGING_MODE */

        return;
    }
    g_bTiltStatus = FALSE;

    PanoCapture_ClearParam();

    Arch_GetTable(PanoParam.Arch, PanoParam.Scan);

    PanoSensor_Config();
    PanoSensor_Start();

    if (!Motor_MoveInitPosition())
    {
    	UART_SendMessage(DBG_MSG_PC, "Motor Init Position Fail.");
        PanoCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
				PrevCaptureMode = CAPTURE_CANCEL;
#endif /* USE_AGING_MODE */

        
        return;
    }
	//Motor_MoveAlignPosition();

    if (CurCaptureMode != CAPTURE_PANO)
    {
        PanoCapture_CancelProcess();
        return;
    }

#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
	//if (PanoParam.Mode == PANO_MODE_NORMAL)
	{
		ElapseTimerOnOff(TRUE);
	}
#endif /* BUILD_TYPE_DEBUG && !USE_AGING_MODE */



#ifdef USE_MOTOR_CHINREST_TS
    Motor_MoveTempleSupport(FALSE);
#endif /* USE_MOTOR_CHINREST_TS */

	LaserSetPosition();

	LaserControl(TYPE_HEAD_PANO, TRUE);
	LaserControl(TYPE_FOOT, TRUE);

	if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "Panorama Init Motor R::CurrentStep = %d", Motor_R.CurrentStep);
		printUart(DBG_MSG_PC, "Panorama Init Motor V::CurrentStep = %d", Motor_V.CurrentStep);
		printUart(DBG_MSG_PC, "Panorama Init Motor CNS::CurrentStep = %d", Motor_CNS.CurrentStep);
		printUart(DBG_MSG_PC, "Panorama Init Motor CWE::CurrentStep = %d", Motor_CWE.CurrentStep);
	}

	UART_SendMessage(DBG_MSG_PC, "[SP_PANO_READY]");	
#ifdef USE_TABLET_PC
    UART_SendMessage(DBG_MSG_TABLET, "[SP_PANO_READY]");	
#endif /* USE_TABLET_PC */
#ifdef USE_AGING_MODE
	if (g_bAgingMode != TRUE) {
		while(!UART_SetPanoParam() && CurCaptureMode != CAPTURE_CANCEL);
	}
#else /* not USE_AGING_MODE */
    while(!UART_SetPanoParam() && CurCaptureMode != CAPTURE_CANCEL);
#endif /* USE_AGING_MODE */

	if (g_bColumnPressed == TRUE)
	{
		Column_Control(COLUMN_STOP);
		g_bColumnPressed = FALSE;
	}

	LaserControl(TYPE_HEAD_PANO, FALSE);
	LaserControl(TYPE_FOOT, FALSE);

	if (!bIsExposureSwitchReleased(500)) {
		UART_SendMessage(DBG_MSG_PC, ERR_CODE_EXP_SWITCH);
        CurCaptureMode = CAPTURE_CANCEL;
	}

    if (CurCaptureMode != CAPTURE_PANO)
    {
        PanoCapture_CancelProcess();
        return;
    }

    Lamp_Control(LAMP_RED, LAMP_DISABLE);
    Lamp_Control(LAMP_GREEN, LAMP_ENABLE);
    Lamp_Control(LAMP_BLUE, LAMP_DISABLE);
    
    Arch_GetTable(PanoParam.Arch, PanoParam.Scan);
    PanoCapture_ModeOffset();

    if (PanoParam.bColliOpen == TRUE)
    {
        if (!CAN_Collimator_SendMessage(CMD_COLLI_OPEN, 0, 0, 2000, 2))
        {
        	UART_SendMessage(DBG_MSG_PC, "Collimator Open Fail.");
            PanoCapture_CancelProcess();
            CurCaptureMode = CAPTURE_CANCEL;
            
            return;
        }
    }
    else
    {
		if (PanoParam.Mode == PANO_MODE_3D)
		{
			if (!CAN_Collimator_SendMessage(CMD_COLLI_PANO_180PX, 0, 0, 2000, 2))
			{
				UART_SendMessage(DBG_MSG_PC, "Collimator PANO_180PX Fail.");
				PanoCapture_CancelProcess();
				CurCaptureMode = CAPTURE_CANCEL;
				
				return;
			}
		}
		else
		{
		    if (PanoParam.Arch == PANO_ARCH_CHILD)
		    {
		        if (!CAN_Collimator_SendMessage(CMD_COLLI_PANOR_CHILD, 0, 0, 2000, 2))
		        {
		            if (!CAN_Collimator_SendMessage(CMD_COLLI_PANOR, 0, 0, 2000, 2))
		            {
		            	UART_SendMessage(DBG_MSG_PC, "Collimator Panorama Fail.");
		                PanoCapture_CancelProcess();
		                CurCaptureMode = CAPTURE_CANCEL;

		                return;
		            }
		        }
		    }
		    else
		    {
		        if (!CAN_Collimator_SendMessage(CMD_COLLI_PANOR, 0, 0, 2000, 2))
		        {
		        	UART_SendMessage(DBG_MSG_PC, "Collimator Panorama Fail.");
		            PanoCapture_CancelProcess();
		            CurCaptureMode = CAPTURE_CANCEL;
		            
		            return;
		        }
		    }
		}
    }
    if (!CAN_Collimator_SendMessage(CMD_FILTER_CENTER, 0, 0, 2000, 2))
    {
    	UART_SendMessage(DBG_MSG_PC, "Collimator Filter Move Fail.");
        PanoCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
        
        return;
    }
	if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "Panorama ::Arch = %d, Scan = %d, Mode = %d", PanoParam.Arch,PanoParam.Scan,PanoParam.Mode);
		printUart(DBG_MSG_PC, "Panorama ::kV = %d, mA = %d", PanoParam.KV,PanoParam.mA);
	}

	//jehun - 20200724 확인 필요
	Sensor_P.Update = FALSE;
    while(Sensor_P.Update != FALSE);

    PanoSensor_Stop();
	PanoSensor_SetUserSync();

	gMachStat.bTubeCountFlag=RESET;
	if (!CAN_SendMessage(CAN_TUBE_COUNT_CHECK, 0, 0, 1000, 2))
    {
    	UART_SendMessage(DBG_MSG_PC, "Tube Count Fail.");
        PanoCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
        
        return;
    }

    if (!CAN_SendMessage(CAN_TUBE_CONTI_MODE_SET, 0, 0, 1000, 2))
    {
    	UART_SendMessage(DBG_MSG_PC, "Tube Continuous mode Fail.");
        PanoCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
        
        return;
    }

    if (!CAN_SendMessage(CAN_TUBE_KV_SET, PanoParam.KV, 0, 1000, 2))
    {
    	UART_SendMessage(DBG_MSG_PC, "Tube KV Send Fail.");
        PanoCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
        
        return;
    }

    if (!CAN_SendMessage(CAN_TUBE_mA_SET, PanoParam.mA, 0, 1000, 2))
    {
    	UART_SendMessage(DBG_MSG_PC, "Tube mA Send Fail.");
        PanoCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
        
        return;
    }
#ifdef VARIABLE_EXPOSURE
#define CAN_TUBE_EXP_START_TIME_SET 0x2900
#define CAN_TUBE_EXP_END_TIME_SET 0x2901
#define CAN_TUBE_EXP_KV_SET 0x2902
#define CAN_TUBE_EXP_MA_SET 0x2903
    if (PanoParam.nExpStartTime != 0) 
    {
        if (!CAN_SendMessage(CAN_TUBE_EXP_START_TIME_SET, PanoParam.nExpStartTime, 0, 1000, 2))
        {
            printUart(DBG_MSG_PC, "CAN_TUBE_EXP_START_TIME_SET failed");
        }
    }
    if (PanoParam.nExpEndTime != 0) 
    {
        if (!CAN_SendMessage(CAN_TUBE_EXP_END_TIME_SET, PanoParam.nExpEndTime, 0, 1000, 2))
        {
            printUart(DBG_MSG_PC, "CAN_TUBE_EXP_END_TIME_SET failed");
        }
    }
    if (PanoParam.nExpKV != 0) 
    {
        if (!CAN_SendMessage(CAN_TUBE_EXP_KV_SET, PanoParam.nExpKV, 0, 1000, 2))
        {
            printUart(DBG_MSG_PC, "CAN_TUBE_EXP_KV_SET failed");
        }
    }
    if (PanoParam.nExpMA != 0) 
    {
        if (!CAN_SendMessage(CAN_TUBE_EXP_MA_SET, PanoParam.nExpMA, 0, 1000, 2))
        {
            printUart(DBG_MSG_PC, "CAN_TUBE_EXP_MA_SET failed");
        }
    }
#endif /* VARIABLE_EXPOSURE */



    PanoSensor_Start();
    
    Motor_MoveStartPosition();

	if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "Panorama Start Motor R::CurrentStep = %d", Motor_R.CurrentStep);
		printUart(DBG_MSG_PC, "Panorama Start Motor V::CurrentStep = %d", Motor_V.CurrentStep);
		printUart(DBG_MSG_PC, "Panorama Start Motor A::CurrentStep = %d", Motor_A.CurrentStep);
	}
    Exposure_CtrlLed(EXPOSURE_LED_GPIO_ENABLE);

    UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_CONFI]");
    UART_SendMessage(DBG_MSG_PC, "[SP_STBY_EXPSW]");

    Indicator_Control(INDICATOR_ENABLE);

    while((Exposure_CheckSwitch())&&UART_Exposure_SET()!=1)
    {
        if (CurCaptureMode != CAPTURE_PANO)
        {
            PanoCapture_CancelProcess();
            return;
        }
    }

	check = (BitAction)GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7);

	Timer_Config_T(&Motor_R.Timer);

    /* Verify X-ray interlock is safe before enabling tube */
    if (!Safety_VerifyXrayInterlock()) {
        printUart(DBG_MSG_PC, "SAFETY: X-ray interlock fault, aborting pano");
        PanoCapture_CancelProcess();
        CurCaptureMode = CAPTURE_CANCEL;
        return;
    }

    Tube_CtrlReady(TUBE_READY_ENABLE);

    printUart(DBG_MSG_PC, "Wait for Tube Heatting Time");	

    IntTimer_Delay(2000);
    while(!IntTimer_GetStatus());

    UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_START]");
#ifdef USE_TABLET_PC
	UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_START]");
#endif /* USE_TABLET_PC */
	gMachStat.bTubeCountCompareFlag=TRUE;
	Motor_PanoStart();
	Sensor_P.Count=0;
	if(check==Bit_SET)
	{		
	    while(1)
	    {
	        uint32_t nEndIndex = Motor_R.ArchParam.AccSize + Motor_R.ArchParam.CaptureSize;
	        
	        if (UART_Exposure_SET()==2) 
			{
	            stop = TRUE;
	            break;
	        }

	        if (Motor_R.ArchParam.ArrayIndex > nEndIndex) 
			{
	            break;
	        }
	    }
	}
	else
	{
		
	    while(1)
	    {
	        uint32_t nEndIndex = Motor_R.ArchParam.AccSize + Motor_R.ArchParam.CaptureSize;
	        
	        if ((Exposure_CheckSwitch())) 
			{
	            stop = TRUE;
	            break;
	        }

	        if (Motor_R.ArchParam.ArrayIndex > nEndIndex) 
			{
	            break;
	        }

			if(PanoParam.Mode==PANO_MODE_ALIGN)
			{
				if (Motor_R.ArchParam.ArrayIndex == Motor_R.ArchParam.AccSize+(Motor_R.ArchParam.CaptureSize/2)) 
				{
					if(Motor_R.RunStep == Motor_R.ArchParam.IndexStep/2)
					{
						stop = TRUE;
						Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
				        Tube_CtrlReady(TUBE_READY_DISABLE);
						Motor_PanoEnd();
			            break;
					}					
		        }
			}
	    }
	}

    Indicator_Control(INDICATOR_DISABLE);
	Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
	
    if (!stop)
    {
        PanoCapture_EndProcess();
    }
    else
    {
#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
        //if (PanoParam.Mode == PANO_MODE_NORMAL)
        {
            typElapsedTimer.nExposeTime = nElapsedMilliSec(typElapsedTimer.nExposeTime);
        }
#endif /* BUILD_TYPE_DEBUG && !USE_AGING_MODE */

        PanoCapture_StopProcess();
    }	
}

/**
* @ Function Name : PanoCapture_ClearParam
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void PanoCapture_ClearParam(void)
{
    //PanoParam.Sensor = PANO_SENSOR_VARIAN;
    PanoParam.KV = 80;
    PanoParam.mA = 80; // 8mA
    PanoParam.Mode = PANO_MODE_NORMAL;
    PanoParam.Arch = PANO_ARCH_ADULT;
    PanoParam.Scan = PANO_SCAN_HD;
    Sensor_P.Index = 0;    
    PanoParam.bColliOpen = FALSE;
    Motor_R.ArchParam.ArrayIndex = 0;
#ifdef VARIABLE_EXPOSURE
    PanoParam.nExpStartTime = 0;
    PanoParam.nExpEndTime = 0;
    PanoParam.nExpKV = 0;
    PanoParam.nExpMA = 0;
#endif /* VARIABLE_EXPOSURE */
}

/**
* @ Function Name : PanoCapture_EndProcess
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void PanoCapture_EndProcess(void)
{
    while(!Motor_GetStatus(&Motor_R, STATUS_STOP));	
    while(!Motor_GetStatus(&Motor_V, STATUS_STOP));

	UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_END__]");
#ifdef USE_TABLET_PC
	UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_END__]");
#endif /* USE_TABLET_PC */	
	// 210202 HWAN DEFECT Protocol add
	if(PanoParam.bFirstPano==RESET)
	{
		PanoParam.bFirstPano=SET;
		UART_SendMessage(DBG_MSG_PC, "[SP_PANO_DEFEC]");
	}
    Motor_MoveEndPosition();
	CurCaptureMode = CAPTURE_CANCEL;
	Motor_PanoEnd();

	if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "Panorama END Motor R::CurrentStep = %d", Motor_R.CurrentStep);
		printUart(DBG_MSG_PC, "Panorama END Motor V::CurrentStep = %d", Motor_V.CurrentStep);
		printUart(DBG_MSG_PC, "Panorama END Sensor Trigger Count = %d", Sensor_P.Count);
	}   
	//CurCaptureMode = CAPTURE_CANCEL;

#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
    //if (PanoParam.Mode == PANO_MODE_NORMAL)
    {
        printUart(DBG_MSG_PC, "Exposed Time : %d(ms)", typElapsedTimer.nExposeTime);
        ElapseTimerOnOff(FALSE);
    }
#endif /* BUILD_TYPE_DEBUG && !USE_AGING_MODE */

	PanoCapture_ClearProcess();
}

/**
* @ Function Name : PanoCapture_StopProcess
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void PanoCapture_StopProcess(void)
{
    Tube_CtrlReady(TUBE_READY_DISABLE);
    Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);   

    Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);    
	CurCaptureMode = CAPTURE_CANCEL;
	Motor_PanoEnd();
	if(sysInfo.bShowLog==TRUE)
	{
		printUart(DBG_MSG_PC, "Panorama Stop Motor R::CurrentStep = %d", Motor_R.CurrentStep);
		printUart(DBG_MSG_PC, "Panorama Stop Motor V::CurrentStep = %d", Motor_V.CurrentStep);
	}
    UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_STOP_]");
#ifdef USE_TABLET_PC
    UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_STOP_]");
#endif /* USE_TABLET_PC */

    //CurCaptureMode = CAPTURE_CANCEL;

#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
	//if (PanoParam.Mode == PANO_MODE_NORMAL)
	{
		printUart(DBG_MSG_PC, "Exposed Time : %d(ms)", typElapsedTimer.nExposeTime);
		ElapseTimerOnOff(FALSE);
	}
#endif /* BUILD_TYPE_DEBUG && !USE_AGING_MODE */

    UART_SendMessage(DBG_MSG_PC, ERR_CODE_PANO_CAPT_STOP);

    PanoCapture_CancelProcess();
}

/**
* @ Function Name : PanoCapture_CancelProcess
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void PanoCapture_CancelProcess(void)
{
    UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_CANCE]");
#ifdef USE_TABLET_PC
    UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_CANCE]");
#endif /* USE_TABLET_PC */

	PanoCapture_ClearProcess();
}

/**
* @ Function Name : PanoCapture_ClearProcess
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void PanoCapture_ClearProcess(void)
{    
	printUart(DBG_MSG_PC, "Run PanoCapture_ClearProcess");

	Sensor_P.Update = FALSE;
	while(Sensor_P.Update != FALSE);
	PanoSensor_Stop();

	PanoCapture_ClearParam();

	//Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
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
	        PanoCapture_CancelProcess();
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

	//珥ъ쁺�씠 醫낅즺�릺硫� �꽱�꽌 �쟾�썝�룄 �걟�떎 => �씪�떒 蹂대쪟
    //PanoSensor_PowerControl(FALSE);	

#ifdef USE_AGING_MODE
	PrevCaptureMode = CAPTURE_PANO;
	g_nPanoCnt++;

	SysTime_MesureEnd(__FUNCTION__, __LINE__);	
#endif /* USE_AGING_MODE */
	Tube.bXrayFrameOnOff = RESET;
    LampTimer_Enable();
}

/**
* @ Function Name : PanoCapture_ModeOffset
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void PanoCapture_ModeOffset(void)
{	
    switch(PanoParam.Mode)
    {
	case PANO_MODE_NORMAL:
	case PANO_MODE_LEFT:
	case PANO_MODE_FRONT:
	case PANO_MODE_RIGHT:
	case PANO_MODE_3D:
	case PANO_MODE_ALIGN:
        PanoParam.ModeOffset = Motor_V.CurrentStep;
        break;
	case PANO_MODE_SINUS:
        PanoParam.ModeOffset = Motor_V.CurrentStep;// - PANO_V_SINUS_OFFSET;
        break;
	case PANO_MODE_TMJ:
        PanoParam.ModeOffset = PANO_V_TMJ_POSITION + (Motor_V.CurrentStep - PANO_V_TMJ_OFFSET);        
        break;
    case PANO_MODE_NONE:
    default:
        printUart(DBG_MSG_PC, "PanoCapture_ModeOffset check failed");
        break;
    }
}

/**
* @ Function Name : PanoCapture_ProcNormalMode
* @ Desc : 
* @ Param : 
* @ Return :
*/
void PanoCapture_ProcNormalMode(void)
{
    if (Motor_R.ArchParam.ArrayIndex == Motor_R.ArchParam.AccSize)
    {
        Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
    }
    else if (Motor_R.ArchParam.ArrayIndex == (Motor_R.ArchParam.AccSize + Motor_R.ArchParam.CaptureSize))
    {
        Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
        Tube_CtrlReady(TUBE_READY_DISABLE);

        Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
    }
}

/**
* @ Function Name : PanoCapture_ProcTmjMode
* @ Desc : 
* @ Param : 
* @ Return :
*/
void PanoCapture_ProcTmjMode(void)
{
    uint32_t nXrayOffIndex = Motor_R.ArchParam.AccSize + TMJ_CAPT_OFFSET;
    uint32_t nXrayOnIndex = Motor_R.ArchParam.AccSize + Motor_R.ArchParam.CaptureSize - TMJ_CAPT_OFFSET;

	if(Tube.bXrayFrameOnOff==SET)
	{
		if (Motor_R.ArchParam.ArrayIndex == Motor_R.ArchParam.AccSize -1)
	    {
	        Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
	    }
		else if (Motor_R.ArchParam.ArrayIndex == Motor_R.ArchParam.AccSize + 10 )
	    {
	        Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
	        //Tube_CtrlReady(TUBE_READY_DISABLE);
	    }
		else if (Motor_R.ArchParam.ArrayIndex == (Motor_R.ArchParam.AccSize + Motor_R.ArchParam.CaptureSize - 10 ) )
	    {
	        //Tube_CtrlReady(TUBE_READY_ENABLE);
	        Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
	    }
	    else if (Motor_R.ArchParam.ArrayIndex == (Motor_R.ArchParam.AccSize + Motor_R.ArchParam.CaptureSize) )
	    {
	        Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
	        Tube_CtrlReady(TUBE_READY_DISABLE);

	        Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
			Tube.bXrayFrameOnOff=RESET;
	    }
	}
	else
	{
		if (Motor_R.ArchParam.ArrayIndex == Motor_R.ArchParam.AccSize -1)
	    {
	        Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
	    }
	    else if (Motor_R.ArchParam.ArrayIndex == nXrayOffIndex )
	    {
	        Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
	        //Tube_CtrlReady(TUBE_READY_DISABLE);
	    }
	    else if (Motor_R.ArchParam.ArrayIndex == nXrayOnIndex -1)
	    {
	        //Tube_CtrlReady(TUBE_READY_ENABLE);
	        Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
	    }
	    else if (Motor_R.ArchParam.ArrayIndex == (Motor_R.ArchParam.AccSize + Motor_R.ArchParam.CaptureSize) )
	    {
	        Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
	        Tube_CtrlReady(TUBE_READY_DISABLE);

	        Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
	    }
	}
    
}

/**
* @ Function Name : PanoCapture_ProcLeftMode
* @ Desc : 
* @ Param : 
* @ Return :
*/
void PanoCapture_ProcLeftMode(void)
{
    uint32_t nXrayOffIndex = Motor_R.ArchParam.TotalSize / 2;

    if (Motor_R.ArchParam.ArrayIndex == Motor_R.ArchParam.AccSize -1)
    {
        Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
    }
    else if (Motor_R.ArchParam.ArrayIndex == nXrayOffIndex)
    {
        Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
        Tube_CtrlReady(TUBE_READY_DISABLE);
    }
    else if (Motor_R.ArchParam.ArrayIndex == (Motor_R.ArchParam.AccSize + Motor_R.ArchParam.CaptureSize))
    {
        Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
    }
}

/**
* @ Function Name : PanoCapture_ProcRightMode
* @ Desc : 
* @ Param : 
* @ Return :
*/
void PanoCapture_ProcRightMode(void)
{
    uint32_t nXrayOnIndex = Motor_R.ArchParam.TotalSize / 2;

    if (Motor_R.ArchParam.ArrayIndex == nXrayOnIndex -1)
    {
        Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
    }
    else if (Motor_R.ArchParam.ArrayIndex == (Motor_R.ArchParam.AccSize + Motor_R.ArchParam.CaptureSize))
    {
        Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
        Tube_CtrlReady(TUBE_READY_DISABLE);

        Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
    }
}

/**
* @ Function Name : PanoCapture_ProcFrontMode
* @ Desc : 
* @ Param : 
* @ Return :
*/
void PanoCapture_ProcFrontMode(void)
{
    uint32_t nXrayOnIndex = Motor_R.ArchParam.AccSize + FRONT_CAPT_OFFSET;
    uint32_t nXrayOffIndex = Motor_R.ArchParam.AccSize + Motor_R.ArchParam.CaptureSize - FRONT_CAPT_OFFSET;
    
    if (Motor_R.ArchParam.ArrayIndex == nXrayOnIndex -1)
    {
        Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
    }
    else if (Motor_R.ArchParam.ArrayIndex == nXrayOffIndex)
    {
        Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
        Tube_CtrlReady(TUBE_READY_DISABLE);
    }
    else if (Motor_R.ArchParam.ArrayIndex == (Motor_R.ArchParam.AccSize + Motor_R.ArchParam.CaptureSize))
    {
        Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
    }
}

/**
* @ Function Name : PanoCapture_ProcFrontMode
* @ Desc : 
* @ Param : 
* @ Return :
*/
void PanoCapture_ProcAlignMode(void)
{
    uint32_t nXrayOnIndex = Motor_R.ArchParam.AccSize + ((Motor_R.ArchParam.CaptureSize-1)/2);
    uint32_t nXrayOffIndex = Motor_R.ArchParam.AccSize + ((Motor_R.ArchParam.CaptureSize+1)/2);    
    
	if (Motor_R.ArchParam.ArrayIndex == nXrayOnIndex) 
	{
        Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
    }	
	else if (Motor_R.ArchParam.ArrayIndex == nXrayOffIndex) 
    {
        Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
        Tube_CtrlReady(TUBE_READY_DISABLE);
        Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
    }
}



/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/

