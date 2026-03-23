/*
*********************************************************************************************
* diagnostic.h :
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

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __DIAGNOSTIC_H__
#define __DIAGNOSTIC_H__


/* Include files -------------------------------------------------------- */
#include "Motor.h"

/* Exported typedef ---------------------------------------------------- */
/* Exported define ----------------------------------------------------- */

typedef struct
{
	Motor_Typedef* Motor;
	long Data;
} DiagnosticMode_Typedef;


extern DiagnosticMode_Typedef DiagnosticModeParam;


/* Exported macro ---------------------------------------------------- */
/* Exported variables -------------------------------------------------- */


/* Exported functions -------------------------------------------------- */
void DiagnosticMode(void);


#endif /* __DIAGNOSTIC_H__ */

/*************************** (C) COPYRIGHT Osstem Implant ******END OF FILE*****/

