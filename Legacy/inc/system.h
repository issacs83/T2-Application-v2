/*
*******************************************************************************
* system.h :
*******************************************************************************
* Copyright (C) 2014-2016 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date : 
* @ Brief :
*
* @ Revision History :
*       1) initial creation. ------------------------------------- 2014-11-16
*       2) V1.1.0  ------------------------------------------- 2015-03-20
*       3) V1.2.0  ------------------------------------------- 2015-09-18
*       4) V1.3.0  ------------------------------------------- 2015-10-28
*******************************************************************************
*/

/* Define to prevent recursive inclusion ---------------------------------- */
#ifndef __SYSTEM_H__
#define __SYSTEM_H__


/* Include files ------------------------------------------------------ */
#include <stdbool.h>

/* Exported typedef -------------------------------------------------- */

typedef enum {
	CAPTURE_PANO, //					0x01
	CAPTURE_CT, //						0x02
	CAPTURE_SCAN, //					0x08
	CALIBRATION_MODE, //				0x20
	GEOMETRY_ALIGN_MODE, // 			0xB0
	EEPROM_MODE, // 					0xF0
	DIAGNOSTIC_MODE, //	 		0xF4
	RESET_MODE, // 						0xC0
	CAPTURE_CANCEL, //					0x00
} OpStatus_t;

typedef enum {
    MODEL_T2_CS, // CT + Pano + Scan Ceph
    MODEL_T2_C,	// CT + Pano
} modelNameId_t;

typedef enum {
    INIT_DIR_FORWARD,
    INIT_DIR_REVERSE,
} motorInitDir_t;

typedef struct systemInfo {
    unsigned short board_id;
    modelNameId_t model_id;
    motorInitDir_t initDir_MotCC;
	bool bShowLog;
	bool bShowMotorLog;
	bool bShowQueueLog;
	bool bXrayNoSound;
	int16_t nFW_Com_Value;
	bool bIf0xffff;
	uint8_t	nMembrane_Sound_Value;
	uint8_t nMembrane_Sound_Select;
	uint16_t nMembrane_Button_Delay;
	uint16_t nMembrane_Cancel_Delay;
} systemInfo_t;

typedef enum {
    ERROR_LEVEL_1,
    ERROR_LEVEL_2,
    ERROR_LEVEL_3,
    ERROR_LEVEL_4,
    ERROR_LEVEL_5,
} errorLevel_e;

typedef struct tubeError {
    unsigned short cmd;
    const char *strCode;
    errorLevel_e level;
} tubeError_t;


typedef struct _MACHINESTATUS {
    bool bBootErr;
	bool bBootDone;
	bool bFirstEntry;
    
	bool bTubeCommRxErr;
	bool bTubeErr;
	const tubeError_t *pTubeErrCode;
	bool bTubeCountFlag;
	bool bTubeCountCompareFlag;
	uint16_t nTubeOldCount;
	uint16_t nTubeCount;

	bool bColliCommRxErr;
	bool bColliErr;

	unsigned char nMotorErrorCode;

    errorLevel_e ErrorLevel;
	uint8_t uiColliMatchCnt;
} machineStatus_t;


/* Exported define --------------------------------------------------- */
#define BOARD_REVISION_2_0          100


#define	pi						3.1415926535897932


/* Exported macro --------------------------------------------------- */
/* Exported variables ------------------------------------------------- */
extern systemInfo_t sysInfo;
extern machineStatus_t gMachStat;
/* CurCaptureMode, PrevCaptureMode, g_bAgingMode now declared in shared_vars.h
 * and defined in system.c. Keep these for backward compatibility. */
extern volatile OpStatus_t CurCaptureMode;
#ifdef USE_AGING_MODE
extern OpStatus_t PrevCaptureMode;
extern bool g_bAgingMode;
extern unsigned int g_nPanoCnt, g_nCtCnt, g_nCephCnt;
#endif /* USE_AGING_MODE */


/* Exported functions ------------------------------------------------- */
void showLockedVersion(bool isProtocol);
void showReleasedVersion(bool isProtocol);
void showSystemInfo(void);
void showBoardVersion(void);
void System_Init(void);
void RestartSystem(void);
bool SysInitGenerator(void);
bool SysInitCollimator(void);
bool SysCheckErrorStatus(void);
bool checkPreCaptureStatus(void);

#endif /* __SYSTEM_H__ */

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/

