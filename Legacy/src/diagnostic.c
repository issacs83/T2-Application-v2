/*
*********************************************************************************************
* diagnostic.c :
*********************************************************************************************
* Copyright (C) 2014-2018 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date : 
* @ Brief :
*
* @ Revision History :
*       1) initial creation. ------------------------------------- 2018-06-26
*********************************************************************************************
*/

/* Include files -------------------------------------------------------- */
#include "extern.h"
#include "system.h"
#include "serial.h"
#include "diagnostic.h"
#include "can.h"


/* Private typedef ----------------------------------------------------- */
/* Private define ------------------------------------------------------ */
DiagnosticMode_Typedef DiagnosticModeParam;

 
/* Private macro ------------------------------------------------------ */
/* Private variables ---------------------------------------------------- */


/* Private function prototypes --------------------------------------------*/
/* Private functions ---------------------------------------------------- */
/**
* @ Function Name : DiagnosticMode
* @ Desc : 
* @ Param : 
* @ Return :
*/
void DiagnosticMode(void)
{
	checkPreCaptureStatus();
	
	printUart(DBG_MSG_PC, "DiagnosticMode enter");

	if (!CAN_Collimator_SendMessage(CMD_TILT_OFF, 0, 0, 5000, 2)) 
	{
		UART_SendMessage(DBG_MSG_PC, "Tilt Off Fail.");
        //return;
    }
		
    while( (CurCaptureMode == DIAGNOSTIC_MODE) && !UART_SetDiagParam() );

	//CurCaptureMode = CAPTURE_CANCEL;
	printUart(DBG_MSG_PC, "DiagnosticMode exit");
}

