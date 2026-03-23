/*
*******************************************************************************
* cmd_geoalign_cal_priv.h : Private defines shared by geoalign/calibration modules
*******************************************************************************
* Copyright (C) 2014-2018 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Brief   : Command string macros and constants originally in cmd_geoalign_cal.c.
*             Split into a shared private header so that cmd_geoalign.c,
*             cmd_geoalign_axes.c, and cmd_calibration.c can all access them.
*
* @ Revision History :
*       1) Extracted from cmd_geoalign_cal.c for modular build --- 2026-03-24
*******************************************************************************
*/

#ifndef __CMD_GEOALIGN_CAL_PRIV_H__
#define __CMD_GEOALIGN_CAL_PRIV_H__

/* Includes ---------------------------------------------------------------- */
#include <string.h>
#include "cmd_dispatch.h"
#include "error_code.h"
#include "timer.h"
#include "motor.h"
#include "can.h"
#include "tube.h"
#include "sensor.h"
#include "eeprom.h"
#include "calibration.h"
#include "misc1.h"

/* ---------------------------------------------------------------------------
 * Alignment command strings
 * -------------------------------------------------------------------------*/
#define UART_ALIGN_AUTO_COLLI_ALL_CT					"[SM_COLACM"
#define UART_ALIGN_AUTO_COLLI_ALL_PANO					"[SM_COLAPM"
#define UART_ALIGN_AUTO_COLLI_ALL_SCAN					"[SM_COLASM"
#define UART_ALIGN_AUTO_COLLI_ALL						"[SM_COLA_M"
#define UART_ALIGN_AUTO_COLLI_ALL_STOP					"[SM_COLA_STOP_]"
#define UART_ALIGN_AUTO_COLLI_BOT						"[SM_COLB_M"
#define UART_ALIGN_AUTO_COLLI_CT_BOT_STOP				"[SM_COLB_CSTOP]"
#define UART_ALIGN_AUTO_COLLI_CT_LEFT_STOP				"[SM_COLL_CSTOP]"
#define UART_ALIGN_AUTO_COLLI_CT_RIGHT_STOP				"[SM_COLR_CSTOP]"
#define UART_ALIGN_AUTO_COLLI_CT_TOP_STOP				"[SM_COLT_CSTOP]"
#define UART_ALIGN_AUTO_COLLI_LEFT						"[SM_COLL_M"
#define UART_ALIGN_AUTO_COLLIMATOR						"[SM_COLI_AUTO_]"
#define UART_ALIGN_AUTO_COLLI_PANO_BOT_STOP				"[SM_COLB_PSTOP]"
#define UART_ALIGN_AUTO_COLLI_PANO_LEFT_STOP			"[SM_COLL_PSTOP]"
#define UART_ALIGN_AUTO_COLLI_PANO_RIGHT_STOP			"[SM_COLR_PSTOP]"
#define UART_ALIGN_AUTO_COLLI_PANO_TOP_STOP				"[SM_COLT_PSTOP]"
#define UART_ALIGN_AUTO_COLLI_RIGHT						"[SM_COLR_M"
#define UART_ALIGN_AUTO_COLLI_SCAN_BOT_STOP				"[SM_COLB_SSTOP]"
#define UART_ALIGN_AUTO_COLLI_SCAN_LEFT_STOP			"[SM_COLL_SSTOP]"
#define UART_ALIGN_AUTO_COLLI_SCAN_RIGHT_STOP			"[SM_COLR_SSTOP]"
#define UART_ALIGN_AUTO_COLLI_SCAN_TOP_STOP				"[SM_COLT_SSTOP]"
#define UART_ALIGN_AUTO_COLLI_TOP						"[SM_COLT_M"
#define UART_ALIGN_BEAM_COLI_REQ			"[SM_ALG_REQ_BM]"
#define UART_ALIGN_BEAM_COLI				"[SM_ALG_SCAN_H]"
#define UART_ALIGN_BEAM_SET 				"[SM_ALG_SET_BM]"
#define UART_ALIGN_CT_COLI_BOTTOM 						"[SM_ALG_CT_B__]"
#define UART_ALIGN_CT_COLI_LEFT 						"[SM_ALG_CT_L__]"
#define	UART_ALIGN_CT_COLI_REQ_BOTTOM	 		       	"[SM_ALG_REQ_CB]"
#define	UART_ALIGN_CT_COLI_REQ_LEFT			        	"[SM_ALG_REQ_CL]"
#define	UART_ALIGN_CT_COLI_REQ_RIGHT		        	"[SM_ALG_REQ_CR]"
#define	UART_ALIGN_CT_COLI_REQ_TOP			        	"[SM_ALG_REQ_CT]"
#define UART_ALIGN_CT_COLI_RIGHT 						"[SM_ALG_CT_R__]"
#define UART_ALIGN_CT_COLI_TOP 							"[SM_ALG_CT_T__]"
#define	UART_ALIGN_CT_MODE			  		    	  	"[SM_ALG___CT__]"
#define	UART_ALIGN_ONESHOT_COLI_BOTTOM				    "[SM_ALG_ONES_B]"
#define	UART_ALIGN_ONESHOT_COLI_LEFT		     		"[SM_ALG_ONES_L]"
#define	UART_ALIGN_ONESHOT_COLI_REQ_BOTTOM				"[SM_ALG_REQ_OB]"
#define	UART_ALIGN_ONESHOT_COLI_REQ_LEFT				"[SM_ALG_REQ_OL]"
#define	UART_ALIGN_ONESHOT_COLI_REQ_RIGHT				"[SM_ALG_REQ_OR]"
#define	UART_ALIGN_ONESHOT_COLI_REQ_TOP		    		"[SM_ALG_REQ_OT]"
#define	UART_ALIGN_ONESHOT_COLI_RIGHT		        	"[SM_ALG_ONES_R]"
#define	UART_ALIGN_ONESHOT_COLI_TOP		        		"[SM_ALG_ONES_T]"
#define UART_ALIGN_PANO_1COL_START_POS					"[SM_MOV_1CLP_S]"
#define UART_ALIGN_PANO_COLLIMATOR_BOTTOM				"[SM_ALG_PAN_B_]"
#define UART_ALIGN_PANO_COLLIMATOR_LEFT 				"[SM_ALG_PAN_L_]"
#define UART_ALIGN_PANO_COLLIMATOR_REQ_BOTTOM 			"[SM_ALG_REQ_B_]"
#define UART_ALIGN_PANO_COLLIMATOR_REQ_LEFT 			"[SM_ALG_REQ_L_]"
#define UART_ALIGN_PANO_COLLIMATOR_REQ_RIGHT 			"[SM_ALG_REQ_R_]"
#define UART_ALIGN_PANO_COLLIMATOR_REQ_TOP 				"[SM_ALG_REQ_T_]"
#define UART_ALIGN_PANO_COLLIMATOR_RIGHT 				"[SM_ALG_PAN_R_]"
#define UART_ALIGN_PANO_COLLIMATOR_STEP_DEC 			"[SM_SDEC_"
#define UART_ALIGN_PANO_COLLIMATOR_STEP_INC 			"[SM_SINC_"
#define UART_ALIGN_PANO_COLLIMATOR_TOP	 				"[SM_ALG_PAN_T_]"
#define	UART_ALIGN_PANO_MODE					   		"[SM_ALG__PANO_]"
#define	UART_ALIGN_SCAN_1COL_CENTER			 			"[SM_MOV_1CL__C]"
#define	UART_ALIGN_SCAN_1COL_START_POS					"[SM_MOV_1CL__S]"
#define	UART_ALIGN_SCAN_2COL_SPEED_DN					"[SM_MOV_2CL_DN]"
#define UART_ALIGN_SCAN_2COL_SPEED_READ					"[SM_MOV_2CLREA]"
#define	UART_ALIGN_SCAN_2COL_SPEED_RESET				"[SM_MOV_2CLRST]"
#define	UART_ALIGN_SCAN_2COL_SPEED_SET					"[SM_MOV_2CLSET]"
#define	UART_ALIGN_SCAN_2COL_SPEED_UP					"[SM_MOV_2CL_UP]"
#define	UART_ALIGN_SCAN_2COL_SPEED_VALUE_DN				"[SM_MOV_2D"
#define	UART_ALIGN_SCAN_2COL_SPEED_VALUE_UP				"[SM_MOV_2U"
#define	UART_ALIGN_SCAN_COLI_BOTTOM			  			"[SM_ALG_SCAN_B]"
#define	UART_ALIGN_SCAN_COLI_LEFT			        	"[SM_ALG_SCAN_L]"
#define	UART_ALIGN_SCAN_COLI_REQ_BOTTOM		    		"[SM_ALG_REQ_SB]"
#define	UART_ALIGN_SCAN_COLI_REQ_LEFT			    	"[SM_ALG_REQ_SL]"
#define	UART_ALIGN_SCAN_COLI_REQ_RIGHT					"[SM_ALG_REQ_SR]"
#define	UART_ALIGN_SCAN_COLI_REQ_TOP			    	"[SM_ALG_REQ_ST]"
#define	UART_ALIGN_SCAN_COLI_RIGHT			        	"[SM_ALG_SCAN_R]"
#define	UART_ALIGN_SCAN_COLI_TOP			        	"[SM_ALG_SCAN_T]"
#define	UART_ALIGN_SCAN_MODE					    	"[SM_ALG__SCAN_]"
#define UART_CALIBRATION_CEPH_ONESHOT 			"[SM_CALI_ONES_]"
#define UART_CALIBRATION_CEPH_SCAN 				"[SM_CALI_SCAN_]"
#define UART_CALIBRATION_CT 					"[SM_CALI__CT__]"
#define	UART_CALIBRATION_EXIT					"[SM_CALI_EXIT_]"
#define UART_CALIBRATION_PANORAMA 				"[SM_CALI_PANO_]"
#define	UART_CANINE_ALIGN_MOVE_VAXIS_INIT		"[SM_CVXI_INIT_]"
#define	UART_CANINE_ALIGN_VAXIS_RESET     		"[SM_CVXI_RESET]"
#define	UART_CANINE_ALIGN_VAXIS_SET        		"[SM_CVXI_SET__]"
#define	UART_CANINE_ALIGN_VAXIS_STEP_DOWN		"[SM_CVXI_DOWN_]"
#define	UART_CANINE_ALIGN_VAXIS_STEP_UP			"[SM_CVXI__UP__]"
#define	UART_CANINE_ALIGN_VAXIS_STOP        	"[SM_CVXI_STOP_]"
#define UART_CAN_TEST_COLI_COMM_CHECK		"[SM_CANC_COMC_]"
#define UART_CAN_TEST_TUBE_COM_CHECK 		"[SM_CANT_COMC_]"
#define UART_CEPH_2ND_AROUND_SHOT						"[SM_CSHOT_"
#define	UART_CEPH_2ND_COLI_1ST_ALIGN_POSITION 			"[SM_MOV_2CL__L]"
#define UART_CEPH_2ND_COLI_ALIGN_DEC_OFFSET				"[SM_2CLD_"
#define UART_CEPH_2ND_COLI_ALIGN_INC_OFFSET				"[SM_2CLI_"
#define	UART_CEPH_2ND_COLI_CENTER 						"[SM_MOV_2CL__C]"
#define	UART_CEPH_2ND_COLI_END_STEP_RESET	        "[SM_RST_2CL_CE]"
#define	UART_CEPH_2ND_COLI_END_STEP_SET	            "[SM_SET_2CL_CE]"
#define	UART_CEPH_2ND_COLI_MOVE_ABSOLUTE_STEP_LEFT 	"[SM_M2CL_R"
#define	UART_CEPH_2ND_COLI_MOVE_ABSOLUTE_STEP_RIGHT 	"[SM_M2CL_L"
#define	UART_CEPH_2ND_COLI_MOVE_DETAIL_STEP_LEFT 		"[SM_MDS_2CL__L]"
#define	UART_CEPH_2ND_COLI_MOVE_DETAIL_STEP_RIGHT 		"[SM_MDS_2CL__R]"
#define	UART_CEPH_2ND_COLI_MOVE_STEP_LEFT	    		"[SM_MNS_2CL__L]"
#define	UART_CEPH_2ND_COLI_MOVE_STEP_RIGHT	    		"[SM_MNS_2CL__R]"
#define	UART_CEPH_2ND_COLI_STEP_RESET	            "[SM_RST_2CL_CS]"
#define	UART_CEPH_2ND_COLI_STEP_SET	    			"[SM_SET_2CL_CS]"
#define UART_CEPH_ALIGN_SCAN_TEST				"[SM_SCAN_TEST_]"
#define UART_CEPH_ALIGN_SCAN_TEST_TIME			"[SM_SCAN_TIM"
#define	UART_CEPH_ALIGN_XRAY_SHOT_OFF		    "[SM_ALG_XRS_OF]"
#define	UART_CEPH_ALIGN_XRAY_SHOT_ON		    "[SM_ALG_XRS_ON]"
#define UART_CEPH_DET_END_POS							"[SM_MOV_DET__E]"
#define UART_CEPH_DET_START_POS							"[SM_MOV_DET__S]"
#define	UART_CEPH_FAST_2ND_COLI_STEP_RESET	        "[SM_RST_2CL_FS]"
#define	UART_CEPH_FAST_2ND_COLI_STEP_SET	    	"[SM_SET_2CL_FS]"
#define UART_CEPH_FAST_SPEED_STEP_OFFSET			"[SM_CEPF_"
#define UART_CEPH_FAST_SPEED_STEP_RESET				"[SM_CEPF_RESET]"
#define UART_CEPH_OFFSET    					"[SM_CEPH_OFFS_]"
#define UART_CEPH_RAXIS_ALIGN_AROUND_SHOT	"[SM_SRAX_"
#define UART_CEPH_RAXIS_ALIGN_SCAN			"[SM_SCAN_ALIGN]"
#define UART_CEPH_SPEED_STEP_OFFSET				"[SM_CEPS_"
#define UART_CEPH_SPEED_STEP_RESET				"[SM_CEPS_RESET]"
#define	UART_CHINREST_DOWN				"[SM_CHNR_DOWN_]"
#define	UART_CHINREST_UP				"[SM_CHNR__UP__]"
#define UART_COLI_VERSION_CHECK 		"[SM_CANC_VERC_]"
#define UART_COLI_BUILD_CHECK        	"[SM_CANC_BUILD]"
#define UART_COLLIMATOR_FILTER_CENTER  				"[SM_FILT_CENTE]"
#define UART_COLLIMATOR_FILTER_LEFT    				"[SM_FILT_LEFT_]"
#define UART_COLLIMATOR_FILTER_RIGHT   				"[SM_FILT_RIGHT]"
#define UART_COLLI_READ_ALL				"[SM_COLI_ALL__]"
#define UART_COLLI_STATUS_ALL_MOVE					"[SM_COLA_"
#define UART_COLLI_STATUS_ALL_STOP					"[SM_COLI_ASTOP]"
#define UART_COLLI_STATUS_BOTTOM_MOVE				"[SM_COLB_"
#define UART_COLLI_STATUS_BOTTOM_STOP				"[SM_COLI_BSTOP]"
#define UART_COLLI_STATUS_FILTER_MOVE				"[SM_COLF_"
#define UART_COLLI_STATUS_FILTER_STOP				"[SM_COLI_FSTOP]"
#define UART_COLLI_STATUS_LASER_MOVE				"[SM_COLH_"
#define UART_COLLI_STATUS_LASER_STOP				"[SM_COLI_HSTOP]"
#define UART_COLLI_STATUS_LEFT_MOVE					"[SM_COLL_"
#define UART_COLLI_STATUS_LEFT_STOP					"[SM_COLI_LSTOP]"
#define UART_COLLI_STATUS_RIGHT_MOVE				"[SM_COLR_"
#define UART_COLLI_STATUS_RIGHT_STOP				"[SM_COLI_RSTOP]"
#define UART_COLLI_STATUS_TOP_MOVE					"[SM_COLT_"
#define UART_COLLI_STATUS_TOP_STOP					"[SM_COLI_TSTOP]"
#define UART_CT_ALIGN_FAST_RAXIS_GET_OFFSET	"[SM_CR_F_OFFSE]"
#define UART_CT_ALIGN_FAST_RAXIS_RESET 		"[SM_CR_F_RESET]"
#define UART_CT_ALIGN_FAST_RAXIS_SET_DOWN	"[SM_CRFDN__"
#define UART_CT_ALIGN_FAST_RAXIS_SET_UP 	"[SM_CRFUP__"
#define	UART_CT_ALIGN_MOVE_CT_POS		    "[SM_MOVE_CT___]"
#define UART_CT_ALIGN_PAXIS_GET_OFFSET		"[SM_CP_GET_OFS]"
#define UART_CT_ALIGN_PAXIS_RESET 			"[SM_CP_RESET__]"
#define UART_CT_ALIGN_PAXIS_SET_DOWN		"[SM_CP_DN__"
#define UART_CT_ALIGN_PAXIS_SET 			"[SM_CT_P_SET__]"
#define UART_CT_ALIGN_PAXIS_SET_UP 			"[SM_CP_UP__"
#define UART_CT_ALIGN_PAXIS_STEP_DOWN 		"[SM_CT_P_DOWN_]"
#define UART_CT_ALIGN_PAXIS_STEP_STOP 		"[SM_CT_P_STOP_]"
#define	UART_CT_ALIGN_PAXIS_STEP_UP	        "[SM_CT_P_UP___]"
#define UART_CT_ALIGN_RAXIS_GET_OFFSET		"[SM_CR_GET_OFS]"
#define UART_CT_ALIGN_RAXIS_RESET 			"[SM_CR_RESET__]"
#define UART_CT_ALIGN_RAXIS_SET_DOWN		"[SM_CR_DN__"
#define UART_CT_ALIGN_RAXIS_SET_UP 			"[SM_CR_UP__"
#define UART_CT_PATIENT_PAXIS_SET_DOWN		"[SM_CPP_DN_"
#define UART_CT_PATIENT_PAXIS_SET_UP		"[SM_CPP_UP_"
#define UART_CT_PATIENT_PAXIS_RESET			"[SM_CPP_RESET]"
#define UART_CT_PATIENT_PAXIS_GET_OFFSET	"[SM_CPP_OFFSE]"
#define UART_CT_PATIENT_RAXIS_SET_DOWN		"[SM_CRP_DN_"
#define UART_CT_PATIENT_RAXIS_SET_UP		"[SM_CRP_UP_"
#define UART_CT_PATIENT_RAXIS_RESET			"[SM_CRP_RESET]"
#define UART_CT_PATIENT_RAXIS_GET_OFFSET	"[SM_CRP_OFFSE]"
#define UART_GEN1_EXPOSE_DISABLE 			"[SM_GEN1_EXPDS]"
#define UART_GEN1_EXPOSE_ENABLE 			"[SM_GEN1_EXPEN]"
#define UART_GEN1_READY_DISABLE 			"[SM_GEN1_RDYDS]"
#define UART_GEN1_READY_ENABLE 				"[SM_GEN1_RDYEN]"
#define UART_GEN1_TEST		 				"[SM_GEN1_TEST_]"
#define UART_GEO_ALIGN_CT_CNSAXIS_DEC			"[SM_CCNS_DEC"
#define UART_GEO_ALIGN_CT_CNSAXIS_INC			"[SM_CCNS_INC"
#define UART_GEO_ALIGN_CT_CNSAXIS_RESET 		"[SM_CCNS_RESET]"
#define UART_GEO_ALIGN_CT_CNSAXIS_SET 			"[SM_CCNS_SET__]"
#define UART_GEO_ALIGN_CT_CNSAXIS_STEP_DOWN 	"[SM_CCNS_DOWN_]"
#define UART_GEO_ALIGN_CT_CNSAXIS_STEP_UP 		"[SM_CCNS_UP___]"
#define UART_GEO_ALIGN_CT_CWEAXIS_DEC			"[SM_CCWE_DE"
#define UART_GEO_ALIGN_CT_CWEAXIS_INC			"[SM_CCWE_IN"
#define UART_GEO_ALIGN_CT_CWEAXIS_RESET 		"[SM_CCWE_RESET]"
#define UART_GEO_ALIGN_CT_CWEAXIS_SET 			"[SM_CCWE_SET__]"
#define UART_GEO_ALIGN_CT_CWEAXIS_STEP_DOWN 	"[SM_CCWE_DOWN_]"
#define UART_GEO_ALIGN_CT_CWEAXIS_STEP_UP 		"[SM_CCWE_UP___]"
#define UART_GEO_ALIGN_GET_CT_CNSAXIS_OFFSET 	"[SM_CCNS_OFFSE]"
#define UART_GEO_ALIGN_GET_CT_CWEAXIS_OFFSET 	"[SM_CCWE_OFFSE]"
#define UART_GEO_ALIGN_GET_PANO_CNSAXIS_OFFSET 	"[SM_PCNS_OFFSE]"
#define UART_GEO_ALIGN_GET_PANO_CWEAXIS_OFFSET 	"[SM_PCWE_OFFSE]"
#define UART_GEO_ALIGN_GET_RAXIS_OFFSET				"[SM_RAXI_OFFSE]"
#define UART_GEO_ALIGN_GET_VAXIS_OFFSET				"[SM_VAXI_OFFSE]"
#define UART_GEO_ALIGN_PANO_CNSAXIS_DEC		"[SM_PCNS_DEC"
#define UART_GEO_ALIGN_PANO_CNSAXIS_INC		"[SM_PCNS_INC"
#define UART_GEO_ALIGN_PANO_CNSAXIS_RESET 		"[SM_PCNS_RESET]"
#define UART_GEO_ALIGN_PANO_CNSAXIS_SET 		"[SM_PCNS_SET__]"
#define UART_GEO_ALIGN_PANO_CNSAXIS_STEP_DOWN 	"[SM_PCNS_DOWN_]"
#define UART_GEO_ALIGN_PANO_CNSAXIS_STEP_UP 	"[SM_PCNS_UP___]"
#define UART_GEO_ALIGN_PANO_CWEAXIS_DEC		"[SM_PCWE_DE"
#define UART_GEO_ALIGN_PANO_CWEAXIS_INC		"[SM_PCWE_IN"
#define UART_GEO_ALIGN_PANO_CWEAXIS_RESET 		"[SM_PCWE_RESET]"
#define UART_GEO_ALIGN_PANO_CWEAXIS_SET 		"[SM_PCWE_SET__]"
#define UART_GEO_ALIGN_PANO_CWEAXIS_STEP_DOWN 	"[SM_PCWE_DOWN_]"
#define UART_GEO_ALIGN_PANO_CWEAXIS_STEP_UP 	"[SM_PCWE_UP___]"
#define UART_GEO_ALIGN_RAXIS_DEC_OFFSET				"[SM_RDEC_"
#define UART_GEO_ALIGN_RAXIS_INC_OFFSET				"[SM_RINC_"
#define UART_GEO_ALIGN_RAXIS_RESET 				"[SM_RAXI_RESET]"
#define UART_GEO_ALIGN_RAXIS_ROT00 			"[SM_RAXI_ROT00]"
#define UART_GEO_ALIGN_RAXIS_ROT90 			"[SM_RAXI_ROT90]"
#define UART_GEO_ALIGN_RAXIS_SET 				"[SM_RAXI_SET__]"
#define UART_GEO_ALIGN_RAXIS_STEP_DOWN	 		"[SM_RAXI_DOWN_]"
#define UART_GEO_ALIGN_RAXIS_STEP_STOP 		"[SM_RAXI_STOP_]"
#define UART_GEO_ALIGN_RAXIS_STEP_UP 			"[SM_RAXI_UP___]"
#define UART_GEO_ALIGN_TAXIS_CHILD_OFFSET 		"[SM_TAXICOFFSE]"
#define UART_GEO_ALIGN_TAXIS_CHILD_RESET 		"[SM_TAXICRESET]"
#define UART_GEO_ALIGN_TAXIS_CHILD_SET 		"[SM_TAXICSET__]"
#define UART_GEO_ALIGN_TAXIS_MOVE_STEP			"[SM_TAXI_M"
#define UART_GEO_ALIGN_TAXIS_OFFSET 			"[SM_TAXI_OFFSE]"
#define UART_GEO_ALIGN_TAXIS_RESET 			"[SM_TAXI_RESET]"
#define UART_GEO_ALIGN_TAXIS_SET 				"[SM_TAXI_SET__]"
#define UART_GEO_ALIGN_TAXIS_STEP_DOWN	 		"[SM_TAXI_DOWN_]"
#define UART_GEO_ALIGN_TAXIS_STEP_STOP 		"[SM_TAXI_STOP_]"
#define UART_GEO_ALIGN_TAXIS_STEP_UP 			"[SM_TAXI_UP___]"
#define UART_GEO_ALIGN_VAXIS_DEC_OFFSET				"[SM_VDEC_"
#define UART_GEO_ALIGN_VAXIS_INC_OFFSET				"[SM_VINC_"
#define UART_GEO_ALIGN_VAXIS_RESET 			"[SM_VAXI_RESET]"
#define UART_GEO_ALIGN_VAXIS_SET 				"[SM_VAXI_SET__]"
#define UART_GEO_ALIGN_VAXIS_STEP_DOWN 		"[SM_VAXI_DOWN_]"
#define UART_GEO_ALIGN_VAXIS_STEP_STOP 		"[SM_VAXI_STOP_]"
#define UART_GEO_ALIGN_VAXIS_STEP_UP 			"[SM_VAXI_UP___]"
#define UART_GEO_CEPH_ALIGN_RAXIS_MANUAL_DOWN 		"[SM_RCPD_"
#define UART_GEO_CEPH_ALIGN_RAXIS_MANUAL_UP 		"[SM_RCPU_"
#define UART_GEO_CEPH_ALIGN_RAXIS_OFFSET_INIT   	"[SM_RCEP_INIT_]"
#define UART_GEO_CEPH_ALIGN_RAXIS_OFFSET_READ 		"[SM_RCEP_READ_]"
#define UART_GEO_CEPH_ALIGN_RAXIS_RESET 			"[SM_RCEP_RESET]"
#define UART_GEO_CEPH_ALIGN_RAXIS_SET 				"[SM_RCEP_SET__]"
#define UART_GEO_CEPH_ALIGN_RAXIS_STEP_DOWN 		"[SM_RCEP_DOWN_]"
#define UART_GEO_CEPH_ALIGN_RAXIS_STEP_STOP 		"[SM_RCEP_STOP_]"
#define UART_GEO_CEPH_ALIGN_RAXIS_STEP_UP 			"[SM_RCEP_UP___]"
#define UART_GEO_CEPH_POSITION_MOVE					"[SM_RCEP_POSIT]"
#define	UART_MANUAL_XRAY_SHOT					"[SM_XRS__"
#define UART_MOVE_CEPH_BEAM_POS			"[SM_BM_MOV_POS]"
#define UART_RAXIS_ROOTIN						"[SM_RAXI_RTN"
#define	UART_SHOT_READY							"[SM_SHOT_READY]"
#define	UART_TUBE_COUNT_CHECK					"[SM_TUBE_COUNT]"
#define	UART_TUBE_COUNT_RESET					"[SM_TUBE_RESET]"
#define UART_TUBE_EEPROM_CHECK					"[SM_TUBE_EEPRO]"
#define UART_TUBE_TILT_OFF						"[SM_TUBE_T_OFF]"
#define UART_TUBE_TILT_ON						"[SM_TUBE_T_ON_]"
#define UART_TUBE_VERSION_CHECK 		"[SM_CANT_VERC_]"
#define UART_GET_CEPH_2ND_COLI_ALIGN_OFFSET			"[SM_2CL_OFFSET]"
#define UART_GET_CEPH_FAST_2ND_COLI_ALIGN_OFFSET	"[SM_2CLF_OFFSE]"

/* FOV commands (also defined in cmd_ct.c -- local redefinition for calibration) */
#define UART_FOV_5X5            "[SM_FOV__5X5__]"
#define UART_FOV_8X9            "[SM_FOV__8X9__]"
#define UART_FOV_8X10           "[SM_FOV__8X10_]"
#define UART_FOV_10X9           "[SM_FOV__10X9_]"
#define UART_FOV_12X9           "[SM_FOV__12X9_]"
#define UART_FOV_15X9           "[SM_FOV__15X9_]"
#define UART_FOV_10X10          "[SM_FOV__10X10]"
#define UART_FOV_12X10          "[SM_FOV__12X10]"
#define UART_FOV_15X10          "[SM_FOV__15X10]"

/* Chinrest commands (also defined in cmd_ct.c -- local redefinition) */
#define UART_CMD_CHINREST_HOR_MOVE  "[SM_HAXS_MV"
#define UART_CMD_CHINREST_VER_MOVE  "[SM_VAXS_MV"

/* Ceph 2nd collimator start/end/fast position (used in both geoalign and calibration) */
#define UART_CEPH_2COL_START_POS	"[SM_2COL_STRT_]"
#define UART_CEPH_2COL_END_POS		"[SM_2COL_END__]"
#define UART_CEPH_2COL_FAST_START_POS	"[SM_2COL_FSTRT]"

/* ---------------------------------------------------------------------------
 * Calculation constants
 * -------------------------------------------------------------------------*/
#define ALIGN_2ND_COLI_MOVE_STEP            	(1.0 * (MOTOR_S_MICROSTEP / (MOTOR_S_PULLEY_DIAMETER * pi)))
#define ALIGN_2ND_COLI_MOVE_DETAIL_STEP 		(0.1 * (MOTOR_S_MICROSTEP / (MOTOR_S_PULLEY_DIAMETER * pi)))
#define ALIGN_2ND_COLI_CENTER_POSISTION     	(150.1 * (MOTOR_S_MICROSTEP / (MOTOR_S_PULLEY_DIAMETER * pi)))
#define	CEPH_ROTATE_ANGLE           		90.0 /* degree */
#define CANINE_ALIGN_VAXIS_STEP 		(10.0 / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP))
#define CEPH_2ND_COLI_ALIGN_POS 		(40.31 * (MOTOR_S_MICROSTEP / (MOTOR_S_PULLEY_DIAMETER * pi)))
#define MAX_XRAY_SHOT_TIME               	23000 /* ~23 Sec */
#define CT_RAXIS_DEGREE         30.0
#define CT_RAXIS_OFFSET        (CT_RAXIS_DEGREE / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))
#define CT_PAXIS_DIAMETER_MIN     -20.0
#define CT_PAXIS_DIAMETER_MAX     20.0
#define CT_RAXIS_DEGREE_MIN     -10.0
#define CT_RAXIS_DEGREE_MAX     10.0
#define CT_PAXIS_DIAMETER        45.0
#define	CT_PAXIS_OFFSET	      (CT_PAXIS_DIAMETER / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP))
#define CT_RAXIS_START_DEGREE         0.0
#define CT_RAXIS_START_OFFSET         (CT_RAXIS_START_DEGREE / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))
#define CEPH_2ND_COLIRATIO (double)0.734
#define CEPH_2ND_COLI_START_POS (40.31 * (MOTOR_S_MICROSTEP / (MOTOR_S_PULLEY_DIAMETER * pi)))
#define	PANO_V_STANDARD_OFFSET			(10.0 / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP))
#define GEOMETRY_ALIGN_RAXIS_ROTATE_90D 		(120.0 / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))
#define GEOMETRY_ALIGN_RAXIS_ROTATE_00D 		(30.0 / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))
#define GEOMETRY_ALIGN_RAXIS_STEP_ONE 			(20.0 / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))
#define GEOMETRY_ALIGN_VAXIS_STEP_ONE 			(50.0 / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP))
#define GEOMETRY_ALIGN_TAXIS_MIN			-4500
#define GEOMETRY_ALIGN_TAXIS_MAX			9500
#define PAXIS_DISTANCE_PER_FOV_5X			50 /* (150mm - 50mm) /2 */

/* ---------------------------------------------------------------------------
 * Shared module-level variables (extern declarations)
 * -------------------------------------------------------------------------*/
#ifdef CEPH_SCAN_LOOPBACK
extern bool g_bCephLoopBack;
extern double g_dDetectorStartPos;
extern double g_dDetecotrRatio;
extern double g_dCollimatorStartPos;
extern double g_dCollimatorRatio;
extern uint32_t g_nCollimatorDistance;
#endif /* CEPH_SCAN_LOOPBACK */

extern bool bClose, bOpen;
extern bool b2ndColStart;

/* ---------------------------------------------------------------------------
 * Sub-handler prototypes (called from cmd_geoalign.c dispatcher)
 * -------------------------------------------------------------------------*/
bool GeoAlign_HandleRAxis(UART_MsgTypedef *message);
bool GeoAlign_HandleVAxis(UART_MsgTypedef *message);
bool GeoAlign_HandleCephRAxis(UART_MsgTypedef *message);
bool GeoAlign_HandleCTAxes(UART_MsgTypedef *message);
bool GeoAlign_HandleCNSCWEAxes(UART_MsgTypedef *message, bool *bChin_res);
bool GeoAlign_HandleTAxis(UART_MsgTypedef *message);
bool GeoAlign_HandleCommon(UART_MsgTypedef *message, bool *bChin_res);

#endif /* __CMD_GEOALIGN_CAL_PRIV_H__ */
