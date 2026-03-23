/*
*******************************************************************************************
* calibration.c :
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
#include "calibration.h"
#include "system.h"
#include "can.h"
#include "serial.h"
#include "misc1.h"
#include "motor.h"


/* Private typedef --------------------------------------------------- */
/* Private define ---------------------------------------------------- */
#define UART_GEO_ALIGN_READY					"[SP_ALGN_READY]"


/* Private macro ---------------------------------------------------- */
/* Private variables -------------------------------------------------- */
CalibrationMode_Typedef CalibrationModeParam;
AlignMode_Typedef AlignModeParam;


/* Private function prototypes ------------------------------------------*/
static void CalibrationMode_ClearParam(void);


/* Private functions -------------------------------------------------- */

/**
* @ Function Name : CalibrationMode
* @ Desc : 
* @ Param : 
* @ Return :
*/
void CalibrationMode(void)
{
	checkPreCaptureStatus();
		
    UART_SendMessage(DBG_MSG_PC, "Calibration Mode is selected");
    UART_SendMessage(DBG_MSG_PC, "[SP_MODE_CALIB]");

	if (g_bColumnPressed == SET)
	{
		Column_Control(COLUMN_STOP);
		g_bColumnPressed = FALSE;
	}
	
	if(CalibrationModeParam.Cal_Mode == Cal_Move)
	{
	    if (!CAN_Collimator_SendMessage(CMD_TILT_OFF, 0, 0, 5000, 2)) 
		{
			// sunghwan - V1.0.0e
			UART_SendMessage(DBG_MSG_PC, "Tilt Off Fail.");
			// sunghwan - V1.0.0e
	        CurCaptureMode = CAPTURE_CANCEL;            
	        return;
	    }
	    g_bTiltStatus = RESET;

	    if (!Motor_MoveInitPosition()) 
		{
	        UART_SendMessage(DBG_MSG_PC, "[SP_CALI_STOP_]");
	        // sunghwan - V1.0.0e
			UART_SendMessage(DBG_MSG_PC, "Motor Init Position Fail.");
	        // sunghwan - V1.0.0e
	        CurCaptureMode = CAPTURE_CANCEL;

	        return;
	    }
		
		LaserSetPosition();
		LaserControl(TYPE_HEAD_CEPH, TRUE);
	}

	CalibrationMode_ClearParam();

    UART_SendMessage(DBG_MSG_PC, "[SP_CALI_READY]");	
#ifdef USE_TABLET_PC
    UART_SendMessage(DBG_MSG_TABLET, "[SP_CALI_READY]");	
#endif /* USE_TABLET_PC */

    while((!UART_SensorCalibration()) && (CurCaptureMode != CAPTURE_CANCEL));

	LaserControl(TYPE_HEAD_CEPH, RESET);
	LaserControl(TYPE_HEAD_PANO, RESET);
	LaserControl(TYPE_FOOT, RESET);

    UART_SendMessage(DBG_MSG_PC, "Calibration Mode exit");
}

/**
* @ Function Name : GeometryAlignMode
* @ Desc : 
* @ Param : 
* @ Return :
*/
void GeometryAlignMode(void)
{
	printUart(DBG_MSG_PC, "Image Alignment is selected");    
	UART_SendMessage(DBG_MSG_PC, "[SP_MODE_ALIGN]");

	checkPreCaptureStatus();

	if (g_bColumnPressed == SET)
	{
		Column_Control(COLUMN_STOP);
		g_bColumnPressed = FALSE;
	}
	
    if (!CAN_Collimator_SendMessage(CMD_TILT_OFF, 0, 0, 5000, 2)) 
	{
        CurCaptureMode = CAPTURE_CANCEL;            
        return;
    }
    g_bTiltStatus = RESET;

    Lamp_Control(LAMP_RED, LAMP_DISABLE);
    Lamp_Control(LAMP_GREEN, LAMP_DISABLE);
    Lamp_Control(LAMP_BLUE, LAMP_ENABLE);

    if (!Motor_MoveInitPosition())
    {
        UART_SendMessage(DBG_MSG_PC, "[SP_ALIG_STOP_]");
		UART_SendMessage(DBG_MSG_PC, "Motor Init Position Fail.");
        CurCaptureMode = CAPTURE_CANCEL;
        
        return;
    }
	//Motor_MoveGeoAlignPosition();

	UART_SendMessage(DBG_MSG_PC, UART_GEO_ALIGN_READY);

    while((!UART_SetGeoAlignParam()) && (CurCaptureMode != CAPTURE_CANCEL));

    Lamp_Control(LAMP_RED, LAMP_ENABLE);
    Lamp_Control(LAMP_GREEN, LAMP_ENABLE);
    Lamp_Control(LAMP_BLUE, LAMP_ENABLE);

    if (!CAN_Collimator_SendMessage(CMD_LASER_CANINE_OFF, 0, 0, 1000, 2))
    {
        CurCaptureMode = CAPTURE_CANCEL;        
        return;
    }

    if (!CAN_Collimator_SendMessage(CMD_LASER_HORI_TOP_OFF, 0, 0, 1000, 2))
    {
        CurCaptureMode = CAPTURE_CANCEL;        
        return;
    }

    if (!CAN_Collimator_SendMessage(CMD_LASER_HORI_BOT_OFF, 0, 0, 1000, 2))
    {
        CurCaptureMode = CAPTURE_CANCEL;        
        return;
    }    

	printUart(DBG_MSG_PC, "Image Alignment End");
}

/**
* @ Function Name : CalibrationMode_ClearParam
* @ Desc : 
* @ Param : 
* @ Return :
*/
static void CalibrationMode_ClearParam(void)
{
    CalibrationModeParam.KV 		= 0;
    CalibrationModeParam.mA 		= 0;
    CalibrationModeParam.mode 		= DET_TYPE_CT;
    CalibrationModeParam.nFovCmd 	= CMD_COLLI_CT15X9;
    CalibrationModeParam.AlignType 	= COL_TYPE_NONE;
	CalibrationModeParam.nScanTime	= 10;
}
	
/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/

