/*
*********************************************************************************************
* system.c :
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
#include "version.h"
#include "system.h"
#include "shared_vars.h"
#ifdef USE_I2C_EEPROM
#include "eeprom.h"
#endif /* USE_I2C_EEPROM */
#include "emergency.h"
#include "error_code.h"
#include "can.h"
#include "serial.h"
#include "sensor.h"
#include "misc1.h"
#include "motor.h"
#include "tube.h"
#include "timer.h"


/* Private typedef ----------------------------------------------------- */
/* Private define ------------------------------------------------------ */
#define AIRCR_VECTKEY_MASK    		((uint32_t)0x05FA0000)
#define SYS_CHK_RESP_TIME      		5000


/* Private macro ------------------------------------------------------ */
/* Private variables ---------------------------------------------------- */
systemInfo_t sysInfo;
machineStatus_t gMachStat;

/* Shared variable definitions (declared extern volatile in shared_vars.h).
 * These are the ONE TRUE definition for each ISR-shared variable. */
volatile bool g_bTiltStatus = FALSE;
volatile OpStatus_t CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
OpStatus_t PrevCaptureMode = CAPTURE_CANCEL;
bool g_bAgingMode = RESET;
unsigned int g_nPanoCnt, g_nCtCnt, g_nCephCnt;
#endif /* USE_AGING_MODE */


/* Private function prototypes --------------------------------------------*/


/* Private functions ---------------------------------------------------- */
/**
* @ Function Name : showBoardVersion
* @ Desc : 
* @ Param : 
* @ Return :
*/
void showBoardVersion(void)
{
    char str[8] = {0,};

    switch(sysInfo.model_id) {
    case MODEL_T2_CS:
		sprintf(str, "%s", "CS");
        break;
    case MODEL_T2_C:
		sprintf(str, "%s", "C");
        break;
		/*

    case MODEL_T1_CO:
		sprintf(str, "%s", "CO");
        break;

    case MODEL_T1_S:
		sprintf(str, "%s", "S");
        break;

    case MODEL_T1_CS_SEN1:
		sprintf(str, "%s", "CS(SEN1)");
        break;
        */
    default:
		sprintf(str, "%s", "CS");
        break;
    }   

    printUart(DBG_MSG_PC, " Model Name : T2_%s SCM", str);
    printUart(DBG_MSG_PC, " Board Version : %d.%d", (sysInfo.board_id / 100), \
		(sysInfo.board_id % 100));
}

/**
* @ Function Name : showLockedVersion
* @ Desc : 
* @ Param : 
* @ Return :
*/
void showLockedVersion(bool isProtocol)
{
	const char *pLockVers = LOCKED_VERSION;	

	if (isProtocol == SET) {
		char arrProtocol[UART_MSG_SIZE] = {0,};

		sprintf(arrProtocol, "[SP_VER__%s]", pLockVers);
		
		UART_SendMessage(DBG_MSG_PC, arrProtocol);
#ifdef USE_TABLET_PC
		UART_SendMessage(DBG_MSG_TABLET, arrProtocol);
#endif /* USE_TABLET_PC */
	} else {
		char arrString[32] = {0,};

		sprintf(arrString, " F/W Version : %c.%c.%c", pLockVers[0], pLockVers[2], pLockVers[4]);

		printUart(DBG_MSG_PC, arrString);
	}
}

/**
* @ Function Name : showReleasedVersion
* @ Desc : 
* @ Param : 
* @ Return :
*/
void showReleasedVersion(bool isProtocol)
{
	const char *pReleaseVers = RELEASED_VERSION;

	if (isProtocol == SET) {
		char arrProtocol[UART_MSG_SIZE] = {0,};

		sprintf(arrProtocol, "[SP_BLD__%s]", RELEASED_VERSION);
		
		UART_SendMessage(DBG_MSG_PC, arrProtocol);
#ifdef USE_TABLET_PC
		UART_SendMessage(DBG_MSG_TABLET, arrProtocol);
#endif /* USE_TABLET_PC */
	} else {
		char arrTimeStamp[] = __DATE__ __TIME__;
		char arrString[64] = {0,};

		sprintf(arrString, "SCB F/W Build : %c.%c%c.%c%c%c  %s", pReleaseVers[0], \
			pReleaseVers[1], pReleaseVers[2], pReleaseVers[3], pReleaseVers[4],pReleaseVers[5], arrTimeStamp);

		printUart(DBG_MSG_PC, arrString);
	}
}

/**
* @ Function Name : showSystemInfo
* @ Desc : 
* @ Param : 
* @ Return :
*/
void showSystemInfo(void)
{
#ifdef USE_I2C_EEPROM
    EEPRom_LoadSysInfo();
#endif /* USE_I2C_EEPROM */

    printUart(DBG_MSG_PC, "\r\n");
    printUart(DBG_MSG_PC, "=========== Firmware Information ===========");

    showBoardVersion();
    showLockedVersion(RESET);

	if(sysInfo.bShowLog==SET)
	{
		showReleasedVersion(RESET);
	}

    printUart(DBG_MSG_PC, "Usage : Product status - PP");
    printUart(DBG_MSG_PC, "======================================");
}

/**
* @ Function Name : RestartSystem
* @ Desc : 
* @ Param : 
* @ Return :
*/
void RestartSystem(void)
{
    // Generate system reset (software reset)
    SCB->AIRCR = AIRCR_VECTKEY_MASK | (uint32_t)0x04;
}

/**
* @ Function Name : SysInitGenerator
* @ Desc : 
* @ Param : 
* @ Return :
*/
bool SysInitGenerator(void)
{
    if (!CAN_SendMessage(CAN_TUBE_COM_CHECK, CAN_TUBE_GEN2_COM, 0, SYS_CHK_RESP_TIME, 2))
        return RESET;
    
	if (!CAN_SendMessage(CAN_TUBE_VER_CHECK, 0, 0, 5000, 2))
		return RESET;
	/*if (!CAN_SendMessage(CAN_Sub_Version_Check, 0, 0, 5000, 2))
	{
		printUart(DBG_MSG_PC, "CURRENT GENERATOR FW DOES NOT SUPPORT SUB VERSION");
	}*/
    
    if (!CAN_SendMessage(CAN_TUBE_TANK_TEMP_READ, 0, 0, SYS_CHK_RESP_TIME, 2))
        return RESET;
    
    return SET;
}

/**
* @ Function Name : SysInitCollimator
* @ Desc : 
* @ Param : 
* @ Return :
*/
bool SysInitCollimator(void)
{
    if (!CAN_Collimator_SendMessage(CMD_COMM_CHECK, 0, 0, SYS_CHK_RESP_TIME, 2))
        return RESET;
    
	if (!CAN_Collimator_SendMessage(CMD_BUILD_CHECK, 0, 0, 5000, 2))
		return RESET;
	/*if (!CAN_Collimator_SendMessage(CMD_BUILD_SUB_CHECK, 0, 0, 5000, 2))
	{
		printUart(DBG_MSG_PC, "CURRENT COLLIMATOR FW DOES NOT SUPPORT SUB VERSION");
	}*/
    
    if (!CAN_Collimator_SendMessage(CMD_TILT_OFF, 0, 0, SYS_CHK_RESP_TIME, 2))
        return RESET;
    



    return SET;
}

/**
* @ Function Name : SysCheckErrorStatus
* @ Desc : 
* @ Param : 
* @ Return :
*/
bool SysCheckErrorStatus(void)
{
    bool Tube_ret = SET,Colli_ret = SET,ret=SET;

	if(sysInfo.bShowLog==SET)
		printUart(DBG_MSG_PC, "====System Checking Start====");
	
    if (gMachStat.bBootErr)
        printUart(DBG_MSG_PC, "Error Detected on Booting");
    
    if (gMachStat.bTubeCommRxErr) 
	{
        UART_SendMessage(DBG_MSG_PC, ERR_CODE_TUBE_RX_MSG_ERROR);
        //gMachStat.bTubeCommRxErr = RESET;
        ret= RESET;
    }

    if ( (gMachStat.bTubeErr) && (gMachStat.pTubeErrCode != NULL) ) 
	{
        printUart(DBG_MSG_PC, "%s", gMachStat.pTubeErrCode->strCode);

        if (gMachStat.pTubeErrCode->level > ERROR_LEVEL_1)
            Tube_ret = RESET;
    }
   
    if (gMachStat.bColliCommRxErr) 
	{
        UART_SendMessage(DBG_MSG_PC, ERR_CODE_COL_RX_MSG_ERROR);
        //gMachStat.bColliCommRxErr = RESET;
        ret= RESET;
    }

    Colli_ret = CAN_Collimator_SendMessage(CMD_STATUS_CHECK, 0, 0, SYS_CHK_RESP_TIME, 0);

    if ( ((!Colli_ret) && (gMachStat.ErrorLevel > ERROR_LEVEL_1))  || !Tube_ret) 
	{
		printUart(DBG_MSG_PC, "Detect Error : more than ERROR_LEVEL_1");
        ret= RESET;
    }

	printUart(DBG_MSG_PC, "====System Checking Complete==== %d",ret);

    return ret;
}

/**
* @ Function Name : checkPreCaptureStatus
* @ Desc : 
* @ Param : 
* @ Return :
*/
bool checkPreCaptureStatus(void)
{
    return (bool)(SysCheckErrorStatus() && Motor_CheckErrorStatus());
}

/**
* @ Function Name : System_Init
* @ Desc : 
* @ Param : 
* @ Return :
*/
void System_Init(void)
{
	Led_dip_Config();
	Led_dip_Control(Bit_SET);
    UART_Config();

#ifdef USE_I2C_EEPROM
    EEPRom_Init();
#endif /* USE_I2C_EEPROM */
    showSystemInfo();

    Lamp_Config();
	//Led_dip_Config();
    EMG_Config();
    //while(!EMG_ProcessSwitch());
	CAN_Config();

    IntTimer_Config();

	vElapsedTimerConfig();
#ifdef USE_AGING_MODE
	SysSetTime(SET);
#endif /* USE_AGING_MODE */


	//jehun - 20200731
	// EMG 스위치 복귀 시, 간헐적으로 리셋 안먹히는 현상 개선 문구 추가
	IntTimer_Delay(100);
	while(!IntTimer_GetStatus());

    Motor_Config();

    Tube_Config();

    if (sysInfo.model_id == MODEL_T2_CS)
	{
		ScanSensor_PowerConfig();
        ScanSensor_Config();
    }

    PanoSensor_PowerConfig();

    Exposure_Config();

    LaserConfig();

    Column_Config();

    Indicator_Config();

	StichSensor_Config();
}


/***************************** (C) COPYRIGHT Osstem Implant *****END OF FILE****/

