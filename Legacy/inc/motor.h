/*
*******************************************************************************************
* motor.h :
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
#ifndef __MOTOR_H__
#define __MOTOR_H__


/* Exported define --------------------------------------------------- */
#define	MOTOR_R_MICROSTEP							(16 * 200)
#define	MOTOR_R_PULLEY_RATIO						36.0	


#define	MOTOR_V_MICROSTEP							(16 * 200)
#define	MOTOR_V_PULLEY_PITCH						5.0

#define	MOTOR_V_TO_LIMIT_RUN_STEP					1000


#define	MOTOR_C_MICROSTEP							(16 * 200)
#define	MOTOR_C_PULLEY_DIAMETER						11.0


#define	MOTOR_S_MICROSTEP							(16 * 200)
#define	MOTOR_S_PULLEY_DIAMETER						11.0


#define	MOTOR_A_MICROSTEP							(16 * 200)
#define	MOTOR_A_PULLEY_RATIO						34.0

#define	CT_AUTO_ANGLE 								160.0 // degree

#define	CEPH_SCAN_ALIGN_POSITION    				0.1 // mm
#define	CEPH_SCAN_CALIBRATION_POSITION				150.1 // mm

//						(32 * 200)
//#define	MOTOR_R_MICROSTEP__							(32 * 200)


#ifdef USE_MOTOR_CHINREST_VER

//jehun
#define	MOTOR_CNS									TIM3
#define MOTOR_CNS_PANO_INIT_POSITION				61.0		// 200204 HWAN  58.0 
#define MOTOR_CNS_CT_INIT_POSITION					54.0		// 200204 HWAN  51.0 

#if defined(USE_MOTOR_CNS_SCREW_PITCH_6_35)
#define MOTOR_CNS_SCREW_PITCH						6.35
#else /* not USE_MOTOR_CNS_SCREW_PITCH_6_35 */
#define MOTOR_CNS_SCREW_PITCH						1.27
#endif /* USE_MOTOR_CNS_SCREW_PITCH_6_35 */
#define MOTOR_CNS_MICROSTEP							(16 * 200)
#endif /* USE_MOTOR_CHINREST_VER */
#ifdef USE_MOTOR_CHINREST_VER
#define MOTOR_CNS_ALIGN_POSION				68.0
#endif /* USE_MOTOR_CHINREST_VER */
#ifdef USE_MOTOR_CHINREST_HOR
#define MOTOR_CWE_MICROSTEP					(16 * 200)
#define	MOTOR_CWE_PULLEY_PITCH				5.0
#endif /* USE_MOTOR_CHINREST_HOR */

#ifdef USE_MOTOR_CHINREST_HOR
#define MOTOR_CWE_INIT_POSITION						23.0
#endif

/* Exported types ----------------------------------------------------*/
typedef enum
{
	ORG1_LOW,
	ORG1_HIGH,
	ORG1LOW_ORG2LOW,
	ORG1LOW_ORG2HIGH,
	ORG1HIGH_ORG2LOW,
	ORG1HIGH_ORG2HIGH
} OrgType_Typedef;

typedef enum
{
	ORGDIR_LOW,
	ORGDIR_HIGH
} OrgDir_Typedef;

typedef enum
{
	DIR_ORG,
	DIR_LIMIT
} MotorDir_Typedef;

typedef enum
{
	STATUS_STOP,
	STATUS_ORG,
	STATUS_LIMIT,
	STATUS_RUN,
	STATUS_ARCH,
	STATUS_DELAY,
	STATUS_DELAY_ORG,
	STATUS_DELAY_FIN,
    STATUS_CT,
    STATUS_REVERSE_ORG,
    STATUS_REVERSE_LIMIT,
    STATUS_REVERSE_FIN,
	STATUS_TS_ENC,
} MotorStatus_Typedef;

/*
typedef enum
{
	PUSHED,
	RELEASED
} TempleSupporStatus;
*/

typedef enum {
	CR_0_0A,
	CR_0_1A,
	CR_0_2A,
	CR_0_5A,
	CR_0_7A,
	CR_1_0A,
	CR_1_3A,
	CR_1_4A,
	CR_1_8A,
	CR_2_0A,
	CR_2_4A,
	CR_2_9A,
} MotorCurrent;

typedef enum {
	RES_X_256,
	RES_X_128,
	RES_X_64,
	RES_X_32,
	RES_X_16,
	RES_X_8,
	RES_X_4,
} MotorResolution;

typedef enum {
	TYPE_ROT,
	TYPE_PAN,
	TYPE_CC,
	TYPE_CS,
	TYPE_ER,
	TYPE_MS,
	TYPE_CNS,
	TYPE_CWE,
} MotorType;

typedef struct
{
	double		RunFreq;
	uint32_t	AccStep;
	uint32_t	RunStep;
	uint32_t	DecStep;
	double		AccCCR;
	double		RunCCR;
	double		DecCCR;
	double		CurrCCR; // ORG 이동 시, 가속 운동이 완료 되지 않고 ORG 센서를 체크하였을 경우 현재 속도로 감속하기위해 변수 선언 20200729 - jehun
} RunParam_Typedef;

typedef struct
{
	uint32_t	AccSize;
	uint32_t	CaptureSize;
	int32_t		OffsetStep;
	int32_t*	StepArray;
	float*		CcrArray;
	
	uint32_t	TotalSize;
	
	uint32_t	ArrayIndex;
	uint32_t	IndexStep;
	float		IndexCcr;
    uint32_t    nExpOffIndex;
} ArchParam_Typedef;

typedef struct {
	union {
		struct {
			uint8_t HistoryOff : 1;
			uint8_t HistoryOn : 1;
			uint8_t reservedBits : 6;
		} bit;
		uint8_t byte; // 8bits
	} uStatus;
} OrgSensorStatus_Typedef;

typedef struct
{
	Timer_Typedef Timer;
	
	GpioParam Org;
	GpioParam Org2;
	GpioParam Dir;
	
	OrgType_Typedef OrgType;
	OrgDir_Typedef OrgDir;

	MotorStatus_Typedef	Status;
	MotorDir_Typedef MotorDir;
	
	bool		Update;
	double		NextCCR;
	
	uint32_t	ToOrgStep;
	uint32_t	OrgOverStep;

    int32_t     nInitRunStep;
    int32_t		CurrentStep;
	uint32_t	RunStep;
	
	uint32_t	MinFreq;
	double		MinFreqCCR;
		
	uint32_t	DelayFreq;
	double		DelayFreqCCR;
	uint32_t	DelayCnt;

	uint8_t 	Priority;

    OrgSensorStatus_Typedef OrgSen;

	RunParam_Typedef	 ToOrgParam;
	RunParam_Typedef	 ToLimitParam;
	
	RunParam_Typedef	 RunParam;
	ArchParam_Typedef	ArchParam;

	MotorCurrent StartCurrent;
	MotorCurrent StopCurrent;

	MotorType MotType;
} Motor_Typedef;

#ifdef USE_CT_STITCH_MODE
typedef enum {
	TOP_DOWN,
	BOTTOM_UP
} StitchDirection;
#endif /* USE_CT_STITCH_MODE */


/* Exported macro ---------------------------------------------------*/
/* Exported variables -------------------------------------------------*/
extern Motor_Typedef Motor_R;
extern Motor_Typedef Motor_V;
extern Motor_Typedef Motor_C;
extern Motor_Typedef Motor_S;
#ifdef USE_MOTOR_CHINREST_TS
extern Motor_Typedef Motor_T;
#endif /* USE_MOTOR_CHINREST_TS */
extern Motor_Typedef Motor_A;
#ifdef USE_MOTOR_CHINREST_VER
extern Motor_Typedef Motor_CNS;
#endif /* USE_MOTOR_CHINREST_VER */
#ifdef USE_MOTOR_CHINREST_HOR
extern Motor_Typedef Motor_CWE;
#endif /* USE_MOTOR_CHINREST_HOR */


/* Exported functions ------------------------------------------------ */
void MotorR_InitMinFreq(uint32_t freq);
void Motor_Config(void);
void Motor_Start(Motor_Typedef* motor);
void Motor_Stop(Motor_Typedef* motor);
bool Motor_GetStatus(Motor_Typedef* motor, MotorStatus_Typedef status);
bool MotorR_CheckOrgSequence(void);
void Motor_SetArchParam(Motor_Typedef* motor);
void Motor_GetNextOrgCCR(Motor_Typedef* motor);
void Motor_GetNextOrgCCR_R(Motor_Typedef* motor);
void Motor_GetNextReverse_OrgCCR_R(Motor_Typedef* motor);
void Motor_GetNextRunCCR(Motor_Typedef* motor);
void Motor_GetNextRunCCR_R(Motor_Typedef* motor);
void Motor_GetNextArchCCR(Motor_Typedef* motor);
void Motor_Get_NextCT_CCR(Motor_Typedef* motor);
void Motor_RunOrgCheck(Motor_Typedef* motor);
void Motor_Reverse_OrgCheck(Motor_Typedef* motor);
void Motor_MoveAbsolutePosition(Motor_Typedef* motor, double freq, uint32_t acc, int32_t run, uint32_t dec);
void MotorR_MoveAbsolutePosition_CT(Motor_Typedef* motor, double freq, uint32_t acc, int32_t run, uint32_t dec);
void Motor_MoveArchPosition(Motor_Typedef* motor);
bool Motor_MoveALL_ORG(void);
bool Motor_MoveInitPosition(void);
void Motor_MoveAlignPosition(void);
void Motor_MoveStartPosition(void);
void Motor_Ceph_AlignPosition(void);
void Motor_MoveEndPosition(void);
void Motor_CephMoveAbsolutePosition(Motor_Typedef* motor, double freq, uint32_t acc, int32_t run, uint32_t dec);
#ifdef USE_MOTOR_CHINREST_TS
void Motor_TempleSupportOrgCheck(void);
void Motor_MoveTempleSupport(bool status);
void Motor_MoveTempleSupport_Child(bool status);
#endif /* USE_MOTOR_CHINREST_TS */
#ifdef USE_CT_STITCH_MODE
void MotorR_MoveAbsoluteDistance(Motor_Typedef* motor, double time, uint32_t acc, int32_t run, uint32_t dec, char bReverse);
#else /* not USE_CT_STITCH_MODE */
void MotorR_MoveAbsoluteDistance(Motor_Typedef* motor, double time, uint32_t acc, int32_t run, uint32_t dec);
#endif /* USE_CT_STITCH_MODE */
void Motor_MoveGeoAlignPosition(void);
void Motor_Lamp_ErrorStatus(void);
bool Motor_CheckErrorStatus(void);
bool Motor_CheckRetryErrorStatus(void);
bool Motor_Init_Check(void);
#ifdef USE_TS_4AXIS_MOTOR
void Motor_TSMoveAbsoluteDistance(Motor_Typedef* motor, double freq, uint32_t acc, int32_t run, uint32_t dec);
void Motor_TsMoveAbsPos(Motor_Typedef* motor, double freq, uint32_t acc, int32_t run, uint32_t dec);
#endif /* USE_TS_4AXIS_MOTOR */
#ifdef USE_CT_STITCH_MODE
void Motor_ChangeMode(char bPWM);
void Motor_ControlPWM(char bEnable, uint32_t nArr);
void Motor_SetDirMovePosistion(Motor_Typedef *pMot, MotorDir_Typedef dir, double freq, int32_t run);
bool Motor_MoveStitchPosition(StitchDirection dir,uint32_t dir_count,uint32_t delay);
#endif /* USE_CT_STITCH_MODE */
void Motor_PanoStart(void);
void Motor_PanoEnd(void);
void Motor_CTEnd(void);
void Motor_Ceph_StartPosition(void);
void Motor_CephScanStart(uint32_t nEndCount);
void Motor_CephScanEnd(void);
void TMC2660_SetResolution(MotorType motor, MotorResolution res);
void TMC2660_SetCurrent(MotorType motor, MotorCurrent Current);
void TMC2660_SPI_Data_Sending(Motor_Typedef* Mot,uint32_t spiData);
void Motor_ClearCurrentStep(Motor_Typedef *pMotor);
void Motor_SetOnlyDir(Motor_Typedef *pMot, MotorDir_Typedef dir);
bool Motor_ControlChinrest(bool bVertical, double nDistance);
void Motor_TestSetCurrent(Motor_Typedef *motor, bool Start);
void Motor_TestSetMinFreq(Motor_Typedef *motor, uint32_t min);
void Motor_TestAbsoluteMove(Motor_Typedef *motor, double time, uint32_t acc, int32_t run, uint32_t dec);


#endif /* __MOTOR_H__ */
