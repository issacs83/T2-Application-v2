/*
*******************************************************************************************
* calibration.h :
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
#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__


/* Exported define --------------------------------------------------- */


/* Exported types ----------------------------------------------------*/
typedef enum {
    DET_TYPE_CT,
    DET_TYPE_PANORAMA,
    DET_TYPE_CEPH_SCAN,
    DET_TYPE_CEPH_ONESHOT,
} DetectorType;

typedef enum {
    COL_TYPE_NONE,
    COL_TYPE_CT,
    COL_TYPE_PANO,
    COL_TYPE_CEPH_SCAN,
    COL_TYPE_CEPH_ONESHOT,
} AlignColliType;

typedef enum {
    Cal_Move,
    Cal_Not_Move,
} Calibration_Mode;


typedef struct
{
	uint32_t			KV;
	uint32_t			mA;
    DetectorType 		mode;
    AlignColliType 		AlignType;
    uint16_t 			ColCmd;
    uint32_t    		nExpTime;
    uint16_t    		nFovCmd;
	bool				bMotor_R;
	bool				bMotor_V;
	bool				bMotor_A;
	bool				bMotor_C;
	bool				bMotor_S;
	bool				bMotor_CNS;
	bool				bMotor_CWE;
	uint16_t			nScanTime;
	Calibration_Mode	Cal_Mode;
	uint16_t			nColli_Distance;
} CalibrationMode_Typedef;

typedef struct
{
	uint32_t			KV;
	uint32_t			mA;
} AlignMode_Typedef;


/* Exported macro ---------------------------------------------------*/
/* Exported variables -------------------------------------------------*/
extern CalibrationMode_Typedef CalibrationModeParam;
extern AlignMode_Typedef AlignModeParam;

/* Exported functions ------------------------------------------------ */
void CalibrationMode(void);
void GeometryAlignMode(void);


#endif /* __CALIBRATION_H__ */
