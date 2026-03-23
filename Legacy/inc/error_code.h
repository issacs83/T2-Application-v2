/*
*********************************************************************************************
* error_code.h :
*********************************************************************************************
* Copyright (C) 2016-2018 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date : 
* @ Brief :
*
* @ Revision History :
*       1) initial creation. -------------------------------------- 2018-11-12
*********************************************************************************************
*/

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __ERROR_CODE_H__
#define __ERROR_CODE_H__


/* Exported define ----------------------------------------------------- */

/* System Error */
#define	ERR_CODE_PANO_CAPT_STOP						"[SP_ERRS_00001]"
#define	ERR_CODE_CT_CAPT_STOP						"[SP_ERRS_00002]"
#define	ERR_CODE_SCAN_CAPT_STOP						"[SP_ERRS_00003]"

#define	ERR_CODE_CAN_QUEUE_ERROR 					"[SP_ERRS_00200]"

#define	ERR_CODE_UART_QUEUE_OVERFLOW				"[SP_ERRS_00101]"
#define	ERR_CODE_UART_MSG_ERROR						"[SP_ERRS_00102]"

#define	ERR_CODE_UART_CALIB_ERROR					"[SP_ERRS_00010]"

#define	ERR_CODE_EXP_SWITCH							"[SP_ERRI_00001]"

#define	UART_CALIBRATION_MSG_ERROR					"[SP_ERRS_00010]"
#define	UART_CALIBRATION_MOTOR_UNDER_ERROR			"[SP_ERRS_00011]"
#define	UART_CALIBRATION_MOTOR_OVER_ERROR			"[SP_ERRS_00012]"

#define	UART_ALIGN_MSG_ERROR						"[SP_ERRS_00020]"
#define	UART_ALIGN_MOTOR_UNDER_ERROR				"[SP_ERRS_00021]"
#define	UART_ALIGN_MOTOR_OVER_ERROR					"[SP_ERRS_00022]"


/* Collimator Error */
#define	ERR_CODE_COL_QUEUE_OVERFLOW					"[SP_ERRC_00001]"
#define	ERR_CODE_COL_TX_MSG_ERROR					"[SP_ERRC_00002]"
#define	ERR_CODE_COL_RX_MSG_ERROR					"[SP_ERRC_00003]"
#define	ERR_CODE_COL_RESP_ERROR						"[SP_ERRC_00004]"
#define ERR_CODE_COLLI_RETRY_MSG_ERROR				"[SP_ERRC_00005]"
#define ERR_CODE_COL_NOT_MATCH_ERROR				"<SP_ERRC_00006>"
#define ERR_CODE_COL_NOT_MATCH_MAX_ERROR			"[SP_ERRC_00007]"

#define ERR_CODE_COL_OVER_VALUE_ERROR				"[SP_ERRC_00010]"



#define	ERR_CODE_COL_TILT_TIMEOUT 		    	    "[SP_ERRC_00201]"
#define	ERR_CODE_COL_TILT_MOTOR		 				"[SP_ERRC_00202]"
#define	ERR_CODE_COL_TILT_NO_SENSOR		     		"[SP_ERRC_00203]"

#define	ERR_CODE_COL_T_AXIS_ERROR		            "[SP_ERRC_00301]"
#define	ERR_CODE_COL_B_AXIS_ERROR		            "[SP_ERRC_00302]"
#define	ERR_CODE_COL_L_AXIS_ERROR		            "[SP_ERRC_00303]"
#define	ERR_CODE_COL_R_AXIS_ERROR		            "[SP_ERRC_00304]"
#define	ERR_CODE_COL_F_AXIS_ERROR		            "[SP_ERRC_00305]"
#define	ERR_CODE_COL_H_AXIS_ERROR		            "[SP_ERRC_00306]"

/* Generator Error */
#define	ERR_CODE_TUBE_QUEUE_OVERFLOW				"[SP_ERRT_00001]"
#define	ERR_CODE_TUBE_TX_MSG_ERROR					"[SP_ERRT_00002]"
#define	ERR_CODE_TUBE_RX_MSG_ERROR					"[SP_ERRT_00003]"
#define	ERR_CODE_TUBE_RESP_ERROR					"[SP_ERRT_00004]"
#define	ERR_CODE_TUBE_RETRY_MSG_ERROR				"[SP_ERRT_00005]"

#define	ERR_CODE_KV_STB_ERROR						"[SP_ERRT_00101]"
#define	ERR_CODE_MA_STB_ERROR						"[SP_ERRT_00201]"
#define	ERR_CODE_FIL_STB_ERROR						"[SP_ERRT_00301]"
#define	ERR_CODE_READY_FIL_ERROR 					"[SP_ERRT_00302]"
#define	ERR_CODE_KV_HIGH_OUTPUT						"[SP_ERRT_00102]"
#define	ERR_CODE_KV_LOW_OUTPUT						"[SP_ERRT_00103]"
#define	ERR_CODE_MA_HIGH_OUTPUT						"[SP_ERRT_00202]"
#define	ERR_CODE_MA_LOW_OUTPUT						"[SP_ERRT_00203]"
#define	ERR_CODE_KV_REF_ERROR						"[SP_ERRT_00104]"
#define	ERR_CODE_MA_REF_ERROR						"[SP_ERRT_00204]"
#define	ERR_CODE_TANK_TEMP_OVER 					"[SP_ERRT_00303]"
#define	ERR_CODE_TANK_TEMP_OK_SIG 					"[SP_ERRT_00304]"
#define	ERR_CODE_SEL_KV_MA_ERROR 					"[SP_ERRT_00305]"
#define	ERR_CODE_KV_REF_OUT_HI_OVER					"[SP_ERRT_00105]"
#define	ERR_CODE_KV_REF_OUT_LOW_OVER				"[SP_ERRT_00106]"
#define	ERR_CODE_MA_REF_OUT_HI_OVER					"[SP_ERRT_00205]"
#define	ERR_CODE_MA_REF_OUT_LOW_OVER				"[SP_ERRT_00206]"
#define	ERR_CODE_LIMIT_ERROR						"[SP_ERRT_00306]"
#define	ERR_CODE_SEL_KV_DAC_ERROR 					"[SP_ERRT_00107]"
#define	ERR_CODE_SEL_MA_DAC_ERROR 					"[SP_ERRT_00207]"
#define	ERR_CODE_TANK_CON_ERROR						"[SP_ERRT_00307]"
#define	ERR_CODE_TANK_TEMP_SET_ERROR 				"[SP_ERRT_00308]"
#define	ERR_CODE_CMD_ERROR 							"[SP_ERRT_00309]"
#define	ERR_CODE_TANK_FAN_TEMP_SET_ERROR 			"[SP_ERRT_00312]"


/* Motor Error */
#define	ERR_CODE_MOTOR_R_AXIS						"[SP_ERRM_00001]"
#define	ERR_CODE_MOTOR_P_AXIS						"[SP_ERRM_00002]"
#define	ERR_CODE_MOTOR_C_AXIS						"[SP_ERRM_00003]"		//Ceph Detector Sensor
#define	ERR_CODE_MOTOR_S_AXIS						"[SP_ERRM_00004]"		//Ceph Second Collimator
#define	ERR_CODE_MOTOR_TS_AXIS						"[SP_ERRM_00005]"
#define	ERR_CODE_MOTOR_MS_AXIS						"[SP_ERRM_00006]"
#define	ERR_CODE_MOTOR_CNS_AXIS						"[SP_ERRM_00007]"
#define	ERR_CODE_MOTOR_CWE_AXIS						"[SP_ERRM_00008]"

#define	ERR_CODE_HALL_SENSOR_NOT_START				"[SP_ERRH_00001]"
#define	ERR_CODE_HALL_SENSOR_OVER					"[SP_ERRH_00002]"
#define	ERR_CODE_HALL_SENSOR_STUCK					"[SP_ERRH_00003]"
#define	ERR_CODE_HALL_SENSOR_NOT_END				"[SP_ERRH_00004]"
#define	ERR_CODE_CNS_NOT_MOVE						"[SP_ERRH_00005]"
#define	ERR_CODE_COLUMN_POSITION_NOT_ENOUGH			"[SP_ERRH_00006]"

#define	CT_15X15_OK_MESSAGE							"[SP_CT_15X15OK]"
#define	CT_15X15_NG_MESSAGE							"[SP_CT_15X15NG]"

#define	CONFIRM_NG_MESSAGE							"[SP_CONFIRM_NG]"

#define EMERGENCY_SWITCH_ON							"[SP_EMER_ON___]"
#define EMERGENCY_SWITCH_OFF						"[SP_EMER_OFF__]"




#endif /* __ERROR_CODE_H__ */
