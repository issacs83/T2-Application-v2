/*
*******************************************************************************
* cmd_geoalign_cal.c : Geometry alignment and sensor calibration handlers
*******************************************************************************
* Copyright (C) 2014-2018 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Brief   : UART_SetGeoAlignParam and UART_SensorCalibration -- processes
*             queued commands during geometry alignment and calibration modes.
*
* @ Revision History :
*       1) Extracted from serial.c for modular build --------- 2026-03-24
*******************************************************************************
*/

/* Include files ----------------------------------------------------------- */
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

/* Private defines --------------------------------------------------------- */

#define UART_ALIGN_AUTO_COLLI_ALL_CT						"[SM_COLACM"
#define UART_ALIGN_AUTO_COLLI_ALL_PANO						"[SM_COLAPM"
#define UART_ALIGN_AUTO_COLLI_ALL_SCAN						"[SM_COLASM"
#define UART_ALIGN_AUTO_COLLI_ALL							"[SM_COLA_M"
#define UART_ALIGN_AUTO_COLLI_ALL_STOP						"[SM_COLA_STOP_]"
#define UART_ALIGN_AUTO_COLLI_BOT							"[SM_COLB_M"
#define UART_ALIGN_AUTO_COLLI_CT_BOT_STOP					"[SM_COLB_CSTOP]"
#define UART_ALIGN_AUTO_COLLI_CT_LEFT_STOP					"[SM_COLL_CSTOP]"
#define UART_ALIGN_AUTO_COLLI_CT_RIGHT_STOP					"[SM_COLR_CSTOP]"
#define UART_ALIGN_AUTO_COLLI_CT_TOP_STOP					"[SM_COLT_CSTOP]"
#define UART_ALIGN_AUTO_COLLI_LEFT							"[SM_COLL_M"
#define UART_ALIGN_AUTO_COLLIMATOR							"[SM_COLI_AUTO_]"
#define UART_ALIGN_AUTO_COLLI_PANO_BOT_STOP					"[SM_COLB_PSTOP]"
#define UART_ALIGN_AUTO_COLLI_PANO_LEFT_STOP				"[SM_COLL_PSTOP]"
#define UART_ALIGN_AUTO_COLLI_PANO_RIGHT_STOP				"[SM_COLR_PSTOP]"
#define UART_ALIGN_AUTO_COLLI_PANO_TOP_STOP					"[SM_COLT_PSTOP]"
#define UART_ALIGN_AUTO_COLLI_RIGHT							"[SM_COLR_M"
#define UART_ALIGN_AUTO_COLLI_SCAN_BOT_STOP					"[SM_COLB_SSTOP]"
#define UART_ALIGN_AUTO_COLLI_SCAN_LEFT_STOP				"[SM_COLL_SSTOP]"
#define UART_ALIGN_AUTO_COLLI_SCAN_RIGHT_STOP				"[SM_COLR_SSTOP]"
#define UART_ALIGN_AUTO_COLLI_SCAN_TOP_STOP					"[SM_COLT_SSTOP]"
#define UART_ALIGN_AUTO_COLLI_TOP							"[SM_COLT_M"
#define UART_ALIGN_BEAM_COLI_REQ			"[SM_ALG_REQ_BM]"
#define UART_ALIGN_BEAM_COLI				"[SM_ALG_SCAN_H]"
#define UART_ALIGN_BEAM_SET 				"[SM_ALG_SET_BM]"
#define UART_ALIGN_CT_COLI_BOTTOM 							"[SM_ALG_CT_B__]"
#define UART_ALIGN_CT_COLI_LEFT 							"[SM_ALG_CT_L__]"
#define	UART_ALIGN_CT_COLI_REQ_BOTTOM	 		       		"[SM_ALG_REQ_CB]"
#define	UART_ALIGN_CT_COLI_REQ_LEFT			        		"[SM_ALG_REQ_CL]"
#define	UART_ALIGN_CT_COLI_REQ_RIGHT		        		"[SM_ALG_REQ_CR]"
#define	UART_ALIGN_CT_COLI_REQ_TOP			        		"[SM_ALG_REQ_CT]"
#define UART_ALIGN_CT_COLI_RIGHT 							"[SM_ALG_CT_R__]"
#define UART_ALIGN_CT_COLI_TOP 								"[SM_ALG_CT_T__]"
#define	UART_ALIGN_CT_MODE			  		    	  		"[SM_ALG___CT__]"
#define	UART_ALIGN_ONESHOT_COLI_BOTTOM					    "[SM_ALG_ONES_B]"
#define	UART_ALIGN_ONESHOT_COLI_LEFT		     		 	"[SM_ALG_ONES_L]"
#define	UART_ALIGN_ONESHOT_COLI_REQ_BOTTOM					"[SM_ALG_REQ_OB]"
#define	UART_ALIGN_ONESHOT_COLI_REQ_LEFT				    "[SM_ALG_REQ_OL]"
#define	UART_ALIGN_ONESHOT_COLI_REQ_RIGHT					"[SM_ALG_REQ_OR]"
#define	UART_ALIGN_ONESHOT_COLI_REQ_TOP		    			"[SM_ALG_REQ_OT]"
#define	UART_ALIGN_ONESHOT_COLI_RIGHT		        		"[SM_ALG_ONES_R]"
#define	UART_ALIGN_ONESHOT_COLI_TOP		        			"[SM_ALG_ONES_T]"
#define UART_ALIGN_PANO_1COL_START_POS						"[SM_MOV_1CLP_S]"
#define UART_ALIGN_PANO_COLLIMATOR_BOTTOM					"[SM_ALG_PAN_B_]"
#define UART_ALIGN_PANO_COLLIMATOR_LEFT 					"[SM_ALG_PAN_L_]"
#define UART_ALIGN_PANO_COLLIMATOR_REQ_BOTTOM 				"[SM_ALG_REQ_B_]"
#define UART_ALIGN_PANO_COLLIMATOR_REQ_LEFT 				"[SM_ALG_REQ_L_]"
#define UART_ALIGN_PANO_COLLIMATOR_REQ_RIGHT 				"[SM_ALG_REQ_R_]"
#define UART_ALIGN_PANO_COLLIMATOR_REQ_TOP 					"[SM_ALG_REQ_T_]"
#define UART_ALIGN_PANO_COLLIMATOR_RIGHT 					"[SM_ALG_PAN_R_]"
#define UART_ALIGN_PANO_COLLIMATOR_STEP_DEC 				"[SM_SDEC_"
#define UART_ALIGN_PANO_COLLIMATOR_STEP_INC 				"[SM_SINC_"
#define UART_ALIGN_PANO_COLLIMATOR_TOP	 					"[SM_ALG_PAN_T_]"
#define	UART_ALIGN_PANO_MODE					   			"[SM_ALG__PANO_]"
#define	UART_ALIGN_SCAN_1COL_CENTER			 				"[SM_MOV_1CL__C]"
#define	UART_ALIGN_SCAN_1COL_START_POS						"[SM_MOV_1CL__S]"
#define	UART_ALIGN_SCAN_2COL_SPEED_DN						"[SM_MOV_2CL_DN]"
#define UART_ALIGN_SCAN_2COL_SPEED_READ						"[SM_MOV_2CLREA]"
#define	UART_ALIGN_SCAN_2COL_SPEED_RESET					"[SM_MOV_2CLRST]"
#define	UART_ALIGN_SCAN_2COL_SPEED_SET						"[SM_MOV_2CLSET]"
#define	UART_ALIGN_SCAN_2COL_SPEED_UP						"[SM_MOV_2CL_UP]"
#define	UART_ALIGN_SCAN_2COL_SPEED_VALUE_DN					"[SM_MOV_2D"
#define	UART_ALIGN_SCAN_2COL_SPEED_VALUE_UP					"[SM_MOV_2U"
#define	UART_ALIGN_SCAN_COLI_BOTTOM			  				"[SM_ALG_SCAN_B]"
#define	UART_ALIGN_SCAN_COLI_LEFT			        		"[SM_ALG_SCAN_L]"
#define	UART_ALIGN_SCAN_COLI_REQ_BOTTOM		    			"[SM_ALG_REQ_SB]"
#define	UART_ALIGN_SCAN_COLI_REQ_LEFT			    		"[SM_ALG_REQ_SL]"
#define	UART_ALIGN_SCAN_COLI_REQ_RIGHT					    "[SM_ALG_REQ_SR]"
#define	UART_ALIGN_SCAN_COLI_REQ_TOP			    		"[SM_ALG_REQ_ST]"
#define	UART_ALIGN_SCAN_COLI_RIGHT			        		"[SM_ALG_SCAN_R]"
#define	UART_ALIGN_SCAN_COLI_TOP			        		"[SM_ALG_SCAN_T]"
#define	UART_ALIGN_SCAN_MODE					    		"[SM_ALG__SCAN_]"
#define UART_CALIBRATION_CEPH_ONESHOT 				"[SM_CALI_ONES_]"
#define UART_CALIBRATION_CEPH_SCAN 					"[SM_CALI_SCAN_]"
#define UART_CALIBRATION_CT 						"[SM_CALI__CT__]"
#define	UART_CALIBRATION_EXIT						"[SM_CALI_EXIT_]"
#define UART_CALIBRATION_PANORAMA 					"[SM_CALI_PANO_]"
#define	UART_CANINE_ALIGN_MOVE_VAXIS_INIT			  	"[SM_CVXI_INIT_]"
#define	UART_CANINE_ALIGN_VAXIS_RESET     			   	"[SM_CVXI_RESET]"
#define	UART_CANINE_ALIGN_VAXIS_SET        		   	   	"[SM_CVXI_SET__]"
#define	UART_CANINE_ALIGN_VAXIS_STEP_DOWN		      	"[SM_CVXI_DOWN_]"
#define	UART_CANINE_ALIGN_VAXIS_STEP_UP			      	"[SM_CVXI__UP__]"
#define	UART_CANINE_ALIGN_VAXIS_STOP        		  	"[SM_CVXI_STOP_]"
#define UART_CAN_TEST_COLI_COMM_CHECK			"[SM_CANC_COMC_]"
#define UART_CAN_TEST_TUBE_COM_CHECK 			"[SM_CANT_COMC_]"
#define UART_CEPH_2ND_AROUND_SHOT							"[SM_CSHOT_"
#define	UART_CEPH_2ND_COLI_1ST_ALIGN_POSITION 				"[SM_MOV_2CL__L]"
#define UART_CEPH_2ND_COLI_ALIGN_DEC_OFFSET					"[SM_2CLD_"
#define UART_CEPH_2ND_COLI_ALIGN_INC_OFFSET					"[SM_2CLI_"
#define	UART_CEPH_2ND_COLI_CENTER 							"[SM_MOV_2CL__C]"
#define	UART_CEPH_2ND_COLI_END_STEP_RESET	        "[SM_RST_2CL_CE]"
#define	UART_CEPH_2ND_COLI_END_STEP_SET	            "[SM_SET_2CL_CE]"
#define	UART_CEPH_2ND_COLI_MOVE_ABSOLUTE_STEP_LEFT 				"[SM_M2CL_R"
#define	UART_CEPH_2ND_COLI_MOVE_ABSOLUTE_STEP_RIGHT 			"[SM_M2CL_L"
#define	UART_CEPH_2ND_COLI_MOVE_DETAIL_STEP_LEFT 				"[SM_MDS_2CL__L]"
#define	UART_CEPH_2ND_COLI_MOVE_DETAIL_STEP_RIGHT 				"[SM_MDS_2CL__R]"
#define	UART_CEPH_2ND_COLI_MOVE_STEP_LEFT	    			"[SM_MNS_2CL__L]"
#define	UART_CEPH_2ND_COLI_MOVE_STEP_RIGHT	    			"[SM_MNS_2CL__R]"
#define	UART_CEPH_2ND_COLI_STEP_RESET	            "[SM_RST_2CL_CS]"
#define	UART_CEPH_2ND_COLI_STEP_SET	    			"[SM_SET_2CL_CS]"
#define UART_CEPH_ALIGN_SCAN_TEST					"[SM_SCAN_TEST_]"
#define UART_CEPH_ALIGN_SCAN_TEST_TIME				"[SM_SCAN_TIM"
#define	UART_CEPH_ALIGN_XRAY_SHOT_OFF		        		"[SM_ALG_XRS_OF]"
#define	UART_CEPH_ALIGN_XRAY_SHOT_ON		    		    "[SM_ALG_XRS_ON]"
#define UART_CEPH_DET_END_POS								"[SM_MOV_DET__E]"
#define UART_CEPH_DET_START_POS								"[SM_MOV_DET__S]"
#define	UART_CEPH_FAST_2ND_COLI_STEP_RESET	        "[SM_RST_2CL_FS]"
#define	UART_CEPH_FAST_2ND_COLI_STEP_SET	    	"[SM_SET_2CL_FS]"
#define UART_CEPH_FAST_SPEED_STEP_OFFSET				"[SM_CEPF_"
#define UART_CEPH_FAST_SPEED_STEP_RESET					"[SM_CEPF_RESET]"
#define UART_CEPH_OFFSET    						"[SM_CEPH_OFFS_]"
#define UART_CEPH_RAXIS_ALIGN_AROUND_SHOT		"[SM_SRAX_"
#define UART_CEPH_RAXIS_ALIGN_SCAN					"[SM_SCAN_ALIGN]"
#define UART_CEPH_SPEED_STEP_OFFSET					"[SM_CEPS_"
#define UART_CEPH_SPEED_STEP_RESET					"[SM_CEPS_RESET]"
#define	UART_CHINREST_DOWN					"[SM_CHNR_DOWN_]"
#define	UART_CHINREST_UP					"[SM_CHNR__UP__]"
#define UART_COLI_VERSION_CHECK 			"[SM_CANC_VERC_]"
#define UART_COLLIMATOR_FILTER_CENTER  					"[SM_FILT_CENTE]"
#define UART_COLLIMATOR_FILTER_LEFT    					"[SM_FILT_LEFT_]"
#define UART_COLLIMATOR_FILTER_RIGHT   					"[SM_FILT_RIGHT]"
#define UART_COLLI_READ_ALL					"[SM_COLI_ALL__]"
#define UART_COLLI_STATUS_ALL_MOVE							"[SM_COLA_"
#define UART_COLLI_STATUS_ALL_STOP							"[SM_COLI_ASTOP]"
#define UART_COLLI_STATUS_BOTTOM_MOVE						"[SM_COLB_"
#define UART_COLLI_STATUS_BOTTOM_STOP						"[SM_COLI_BSTOP]"
#define UART_COLLI_STATUS_FILTER_MOVE						"[SM_COLF_"
#define UART_COLLI_STATUS_FILTER_STOP						"[SM_COLI_FSTOP]"
#define UART_COLLI_STATUS_LASER_MOVE						"[SM_COLH_"
#define UART_COLLI_STATUS_LASER_STOP						"[SM_COLI_HSTOP]"
#define UART_COLLI_STATUS_LEFT_MOVE							"[SM_COLL_"
#define UART_COLLI_STATUS_LEFT_STOP							"[SM_COLI_LSTOP]"
#define UART_COLLI_STATUS_RIGHT_MOVE						"[SM_COLR_"
#define UART_COLLI_STATUS_RIGHT_STOP						"[SM_COLI_RSTOP]"
#define UART_COLLI_STATUS_TOP_MOVE							"[SM_COLT_"
#define UART_COLLI_STATUS_TOP_STOP							"[SM_COLI_TSTOP]"
#define UART_CT_ALIGN_FAST_RAXIS_GET_OFFSET		"[SM_CR_F_OFFSE]"
#define UART_CT_ALIGN_FAST_RAXIS_RESET 			"[SM_CR_F_RESET]"
#define UART_CT_ALIGN_FAST_RAXIS_SET_DOWN		"[SM_CRFDN__"
#define UART_CT_ALIGN_FAST_RAXIS_SET_UP 		"[SM_CRFUP__"
#define	UART_CT_ALIGN_MOVE_CT_POS		        "[SM_MOVE_CT___]"
#define UART_CT_ALIGN_PAXIS_GET_OFFSET			"[SM_CP_GET_OFS]"
#define UART_CT_ALIGN_PAXIS_RESET 				"[SM_CP_RESET__]"
#define UART_CT_ALIGN_PAXIS_SET_DOWN			"[SM_CP_DN__"
#define UART_CT_ALIGN_PAXIS_SET 				"[SM_CT_P_SET__]"
#define UART_CT_ALIGN_PAXIS_SET_UP 				"[SM_CP_UP__"
#define UART_CT_ALIGN_PAXIS_STEP_DOWN 			"[SM_CT_P_DOWN_]"
#define UART_CT_ALIGN_PAXIS_STEP_STOP 			"[SM_CT_P_STOP_]"
#define	UART_CT_ALIGN_PAXIS_STEP_UP	            "[SM_CT_P_UP___]"
#define UART_CT_ALIGN_RAXIS_GET_OFFSET			"[SM_CR_GET_OFS]"
#define UART_CT_ALIGN_RAXIS_RESET 				"[SM_CR_RESET__]"
#define UART_CT_ALIGN_RAXIS_SET_DOWN			"[SM_CR_DN__"
#define UART_CT_ALIGN_RAXIS_SET_UP 				"[SM_CR_UP__"
#define UART_GEN1_EXPOSE_DISABLE 				"[SM_GEN1_EXPDS]"
#define UART_GEN1_EXPOSE_ENABLE 				"[SM_GEN1_EXPEN]"
#define UART_GEN1_READY_DISABLE 				"[SM_GEN1_RDYDS]"
#define UART_GEN1_READY_ENABLE 					"[SM_GEN1_RDYEN]"
#define UART_GEN1_TEST		 					"[SM_GEN1_TEST_]"
#define UART_GEO_ALIGN_CT_CNSAXIS_DEC				"[SM_CCNS_DEC"
#define UART_GEO_ALIGN_CT_CNSAXIS_INC				"[SM_CCNS_INC"
#define UART_GEO_ALIGN_CT_CNSAXIS_RESET 			"[SM_CCNS_RESET]"
#define UART_GEO_ALIGN_CT_CNSAXIS_SET 				"[SM_CCNS_SET__]"
#define UART_GEO_ALIGN_CT_CNSAXIS_STEP_DOWN 		"[SM_CCNS_DOWN_]"
#define UART_GEO_ALIGN_CT_CNSAXIS_STEP_UP 			"[SM_CCNS_UP___]"
#define UART_GEO_ALIGN_CT_CWEAXIS_DEC				"[SM_CCWE_DE"
#define UART_GEO_ALIGN_CT_CWEAXIS_INC				"[SM_CCWE_IN"
#define UART_GEO_ALIGN_CT_CWEAXIS_RESET 			"[SM_CCWE_RESET]"
#define UART_GEO_ALIGN_CT_CWEAXIS_SET 				"[SM_CCWE_SET__]"
#define UART_GEO_ALIGN_CT_CWEAXIS_STEP_DOWN 		"[SM_CCWE_DOWN_]"
#define UART_GEO_ALIGN_CT_CWEAXIS_STEP_UP 			"[SM_CCWE_UP___]"
#define UART_GEO_ALIGN_GET_CT_CNSAXIS_OFFSET 		"[SM_CCNS_OFFSE]"
#define UART_GEO_ALIGN_GET_CT_CWEAXIS_OFFSET 		"[SM_CCWE_OFFSE]"
#define UART_GEO_ALIGN_GET_PANO_CNSAXIS_OFFSET 		"[SM_PCNS_OFFSE]"
#define UART_GEO_ALIGN_GET_PANO_CWEAXIS_OFFSET 		"[SM_PCWE_OFFSE]"
#define UART_GEO_ALIGN_GET_RAXIS_OFFSET						"[SM_RAXI_OFFSE]"
#define UART_GEO_ALIGN_GET_VAXIS_OFFSET						"[SM_VAXI_OFFSE]"
#define UART_GEO_ALIGN_PANO_CNSAXIS_DEC				"[SM_PCNS_DEC"
#define UART_GEO_ALIGN_PANO_CNSAXIS_INC				"[SM_PCNS_INC"
#define UART_GEO_ALIGN_PANO_CNSAXIS_RESET 			"[SM_PCNS_RESET]"
#define UART_GEO_ALIGN_PANO_CNSAXIS_SET 			"[SM_PCNS_SET__]"
#define UART_GEO_ALIGN_PANO_CNSAXIS_STEP_DOWN 		"[SM_PCNS_DOWN_]"
#define UART_GEO_ALIGN_PANO_CNSAXIS_STEP_UP 		"[SM_PCNS_UP___]"
#define UART_GEO_ALIGN_PANO_CWEAXIS_DEC				"[SM_PCWE_DE"
#define UART_GEO_ALIGN_PANO_CWEAXIS_INC				"[SM_PCWE_IN"
#define UART_GEO_ALIGN_PANO_CWEAXIS_RESET 			"[SM_PCWE_RESET]"
#define UART_GEO_ALIGN_PANO_CWEAXIS_SET 			"[SM_PCWE_SET__]"
#define UART_GEO_ALIGN_PANO_CWEAXIS_STEP_DOWN 		"[SM_PCWE_DOWN_]"
#define UART_GEO_ALIGN_PANO_CWEAXIS_STEP_UP 		"[SM_PCWE_UP___]"
#define UART_GEO_ALIGN_RAXIS_DEC_OFFSET						"[SM_RDEC_"
#define UART_GEO_ALIGN_RAXIS_INC_OFFSET						"[SM_RINC_"
#define UART_GEO_ALIGN_RAXIS_RESET 					"[SM_RAXI_RESET]"
#define UART_GEO_ALIGN_RAXIS_ROT00 					"[SM_RAXI_ROT00]"
#define UART_GEO_ALIGN_RAXIS_ROT90 					"[SM_RAXI_ROT90]"
#define UART_GEO_ALIGN_RAXIS_SET 					"[SM_RAXI_SET__]"
#define UART_GEO_ALIGN_RAXIS_STEP_DOWN	 			"[SM_RAXI_DOWN_]"
#define UART_GEO_ALIGN_RAXIS_STEP_STOP 				"[SM_RAXI_STOP_]"
#define UART_GEO_ALIGN_RAXIS_STEP_UP 				"[SM_RAXI_UP___]"
#define UART_GEO_ALIGN_TAXIS_CHILD_OFFSET 			"[SM_TAXICOFFSE]"
#define UART_GEO_ALIGN_TAXIS_CHILD_RESET 			"[SM_TAXICRESET]"
#define UART_GEO_ALIGN_TAXIS_CHILD_SET 				"[SM_TAXICSET__]"
#define UART_GEO_ALIGN_TAXIS_MOVE_STEP				"[SM_TAXI_M"
#define UART_GEO_ALIGN_TAXIS_OFFSET 				"[SM_TAXI_OFFSE]"
#define UART_GEO_ALIGN_TAXIS_RESET 					"[SM_TAXI_RESET]"
#define UART_GEO_ALIGN_TAXIS_SET 					"[SM_TAXI_SET__]"
#define UART_GEO_ALIGN_TAXIS_STEP_DOWN	 			"[SM_TAXI_DOWN_]"
#define UART_GEO_ALIGN_TAXIS_STEP_STOP 				"[SM_TAXI_STOP_]"
#define UART_GEO_ALIGN_TAXIS_STEP_UP 				"[SM_TAXI_UP___]"
#define UART_GEO_ALIGN_VAXIS_DEC_OFFSET						"[SM_VDEC_"
#define UART_GEO_ALIGN_VAXIS_INC_OFFSET						"[SM_VINC_"
#define UART_GEO_ALIGN_VAXIS_RESET 					"[SM_VAXI_RESET]"
#define UART_GEO_ALIGN_VAXIS_SET 					"[SM_VAXI_SET__]"
#define UART_GEO_ALIGN_VAXIS_STEP_DOWN 				"[SM_VAXI_DOWN_]"
#define UART_GEO_ALIGN_VAXIS_STEP_STOP 				"[SM_VAXI_STOP_]"
#define UART_GEO_ALIGN_VAXIS_STEP_UP 				"[SM_VAXI_UP___]"
#define UART_GEO_CEPH_ALIGN_RAXIS_MANUAL_DOWN 				"[SM_RCPD_"
#define UART_GEO_CEPH_ALIGN_RAXIS_MANUAL_UP 				"[SM_RCPU_"
#define UART_GEO_CEPH_ALIGN_RAXIS_OFFSET_INIT   			"[SM_RCEP_INIT_]"
#define UART_GEO_CEPH_ALIGN_RAXIS_OFFSET_READ 				"[SM_RCEP_READ_]"
#define UART_GEO_CEPH_ALIGN_RAXIS_RESET 					"[SM_RCEP_RESET]"
#define UART_GEO_CEPH_ALIGN_RAXIS_SET 						"[SM_RCEP_SET__]"
#define UART_GEO_CEPH_ALIGN_RAXIS_STEP_DOWN 				"[SM_RCEP_DOWN_]"
#define UART_GEO_CEPH_ALIGN_RAXIS_STEP_STOP 				"[SM_RCEP_STOP_]"
#define UART_GEO_CEPH_ALIGN_RAXIS_STEP_UP 					"[SM_RCEP_UP___]"
#define UART_GEO_CEPH_POSITION_MOVE							"[SM_RCEP_POSIT]"
#define	UART_MANUAL_XRAY_SHOT						"[SM_XRS__"
#define UART_MOVE_CEPH_BEAM_POS				"[SM_BM_MOV_POS]"
#define UART_RAXIS_ROOTIN							"[SM_RAXI_RTN"
#define	UART_SHOT_READY								"[SM_SHOT_READY]"
#define	UART_TUBE_COUNT_CHECK						"[SM_TUBE_COUNT]"
#define	UART_TUBE_COUNT_RESET						"[SM_TUBE_RESET]"
#define UART_TUBE_EEPROM_CHECK						"[SM_TUBE_EEPRO]"
#define UART_TUBE_TILT_OFF							"[SM_TUBE_T_OFF]"
#define UART_TUBE_TILT_ON							"[SM_TUBE_T_ON_]"
#define UART_TUBE_VERSION_CHECK 			"[SM_CANT_VERC_]"
#define ALIGN_2ND_COLI_MOVE_STEP            		1.0 * (MOTOR_S_MICROSTEP / (MOTOR_S_PULLEY_DIAMETER * pi))
#define ALIGN_2ND_COLI_MOVE_DETAIL_STEP 			0.1 * (MOTOR_S_MICROSTEP / (MOTOR_S_PULLEY_DIAMETER * pi))
#define ALIGN_2ND_COLI_CENTER_POSISTION     		150.1 * (MOTOR_S_MICROSTEP / (MOTOR_S_PULLEY_DIAMETER * pi))
#define	CEPH_ROTATE_ANGLE           		90.0 // degree
#define CANINE_ALIGN_VAXIS_STEP 		10.0 / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP)
#define	UART_CANINE_ALIGN_MOVE_VAXIS_INIT			  	"[SM_CVXI_INIT_]"
#define	UART_CANINE_ALIGN_VAXIS_STEP_UP			      	"[SM_CVXI__UP__]"
#define	UART_CANINE_ALIGN_VAXIS_STEP_DOWN		      	"[SM_CVXI_DOWN_]"
#define	UART_CANINE_ALIGN_VAXIS_STOP        		  	"[SM_CVXI_STOP_]"
#define	UART_CANINE_ALIGN_VAXIS_SET        		   	   	"[SM_CVXI_SET__]"
#define	UART_CANINE_ALIGN_VAXIS_RESET     			   	"[SM_CVXI_RESET]"
#define CEPH_2ND_COLI_ALIGN_POS 		40.31 * (MOTOR_S_MICROSTEP / (MOTOR_S_PULLEY_DIAMETER * pi)) // default
#define MAX_XRAY_SHOT_TIME               			23000 // ~23 Sec
#define	UART_CEPH_2ND_COLI_1ST_ALIGN_POSITION 				"[SM_MOV_2CL__L]"
#define	UART_CEPH_2ND_COLI_CENTER 							"[SM_MOV_2CL__C]"
#define	UART_CEPH_2ND_COLI_MOVE_STEP_LEFT	    			"[SM_MNS_2CL__L]"
#define	UART_CEPH_2ND_COLI_MOVE_STEP_RIGHT	    			"[SM_MNS_2CL__R]"
#define	UART_CEPH_2ND_COLI_MOVE_DETAIL_STEP_LEFT 				"[SM_MDS_2CL__L]"
#define	UART_CEPH_2ND_COLI_MOVE_DETAIL_STEP_RIGHT 				"[SM_MDS_2CL__R]"
#define	UART_CEPH_2ND_COLI_MOVE_ABSOLUTE_STEP_LEFT 				"[SM_M2CL_R"
#define	UART_CEPH_2ND_COLI_MOVE_ABSOLUTE_STEP_RIGHT 			"[SM_M2CL_L"
#define	UART_CEPH_2ND_COLI_STEP_SET	    			"[SM_SET_2CL_CS]"
#define	UART_CEPH_2ND_COLI_STEP_RESET	            "[SM_RST_2CL_CS]"
#define	UART_CEPH_2ND_COLI_END_STEP_SET	            "[SM_SET_2CL_CE]"
#define	UART_CEPH_2ND_COLI_END_STEP_RESET	        "[SM_RST_2CL_CE]"
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
#define UART_CEPH_2ND_COLI_ALIGN_DEC_OFFSET					"[SM_2CLD_"
#define UART_CEPH_2ND_COLI_ALIGN_INC_OFFSET					"[SM_2CLI_"
#define UART_GET_CEPH_2ND_COLI_ALIGN_OFFSET					"[SM_2CL_OFFSET]"
#define CEPH_2ND_COLIRATIO (double)0.734
#define CEPH_2ND_COLI_START_POS 40.31 * (MOTOR_S_MICROSTEP / (MOTOR_S_PULLEY_DIAMETER * pi)) // default
#define	PANO_V_STANDARD_OFFSET			10.0 / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP)
#define GEOMETRY_ALIGN_RAXIS_ROTATE_90D 			(120.0 / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))
#define GEOMETRY_ALIGN_RAXIS_ROTATE_00D 			(30.0 / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))
#define GEOMETRY_ALIGN_RAXIS_STEP_ONE 				(20.0 / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO)) // ���� ����Ʈ!!
#define GEOMETRY_ALIGN_VAXIS_STEP_ONE 				(50.0 / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP))
#define GEOMETRY_ALIGN_TAXIS_MIN			-4500
#define GEOMETRY_ALIGN_TAXIS_MAX			9500
#define PAXIS_DISTANCE_PER_FOV_5X			50 // (150mm - 50mm) /2

#ifdef CEPH_SCAN_LOOPBACK
bool g_bCephLoopBack;
double g_dDetectorStartPos;
double g_dDetecotrRatio;
double g_dCollimatorStartPos;
double g_dCollimatorRatio;
uint32_t g_nCollimatorDistance;
#endif /* CEPH_SCAN_LOOPBACK */

static bool bClose = RESET, bOpen = RESET;
static bool b2ndColStart = RESET;


/* GeoAlign handler: extracted from serial.c lines 3090-4981 */
bool UART_SetGeoAlignParam(void)
{
	UART_MsgTypedef message;
	bool bRet = RESET, bChin_res = RESET;;

	while(MSG_QueueCnt(&UART_Queue))
    {
        message = MSG_Dequeue(&UART_Queue);
		if (strstr(message.Data, UART_MODE_EXIT))
		{
    		CurCaptureMode = CAPTURE_PANO;
			bRet = SET;
		}
        else if (strstr(message.Data, UART_GEO_ALIGN_RAXIS_ROT00))
        {
        	TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
            Motor_MoveAbsolutePosition(&Motor_R, 3000, 1500, GEOMETRY_ALIGN_RAXIS_ROTATE_00D, 1500);
        }
        else if (strstr(message.Data, UART_GEO_ALIGN_RAXIS_ROT90))
        {
        	TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
            Motor_MoveAbsolutePosition(&Motor_R, 3000, 1500, GEOMETRY_ALIGN_RAXIS_ROTATE_90D, 1500);
        }
        else if (strstr(message.Data, UART_GEO_ALIGN_RAXIS_STEP_UP))
        {
        	TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
            Motor_MoveAbsolutePosition(&Motor_R, 50, 100, Motor_R.CurrentStep + GEOMETRY_ALIGN_RAXIS_STEP_ONE, 100);
        }
        else if (strstr(message.Data, UART_GEO_ALIGN_RAXIS_STEP_DOWN))
        {
        	TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
            Motor_MoveAbsolutePosition(&Motor_R, 50, 100, Motor_R.CurrentStep - GEOMETRY_ALIGN_RAXIS_STEP_ONE, 100);
        }
        else if (strstr(message.Data, UART_GEO_ALIGN_RAXIS_STEP_STOP))
        {
            //while(Motor_R.Update != RESET);
            Motor_R.Update=RESET;
            Motor_Stop(&Motor_R);				
			TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StopCurrent);
            printUart(DBG_MSG_PC, "Motor_R.CurrentStep(%d)", Motor_R.CurrentStep);
        }
        else if (strstr(message.Data, UART_GEO_ALIGN_RAXIS_SET))
        {
            int16_t nOffset = 0;

			nOffset=EEPRom_I2C_Read_Word(ROM_RAXIS_ALG_ADDR);
			if(nOffset==0xffff)
			{
				nOffset=0;
			}
			else if(nOffset<0) nOffset++;

            if (Motor_R.CurrentStep < 75.0 / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO)) // RAxis deg 0
            {
#ifdef USE_I2C_EEPROM
                nOffset = nOffset + (Motor_R.CurrentStep - GEOMETRY_ALIGN_RAXIS_ROTATE_00D);
#else /* not USE_I2C_EEPROM */
                nOffset = Motor_R.CurrentStep - GEOMETRY_ALIGN_RAXIS_ROTATE_00D;
#endif /* USE_I2C_EEPROM */
            }
            else // RAxis deg 90
            {
#ifdef USE_I2C_EEPROM
                nOffset = nOffset + (Motor_R.CurrentStep - GEOMETRY_ALIGN_RAXIS_ROTATE_90D);
#else /* not USE_I2C_EEPROM */
                nOffset = Motor_R.CurrentStep - GEOMETRY_ALIGN_RAXIS_ROTATE_90D;
#endif /* USE_I2C_EEPROM */
            }
            printUart(DBG_MSG_PC, "Current Step is %d.", nOffset);

#ifdef USE_I2C_EEPROM
            EEPRom_I2C_Write_Word(ROM_RAXIS_ALG_ADDR, (uint16_t)nOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "R_AXIS_ADDR(0x%02x) : 0x%04x", ROM_RAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_RAXIS_ALG_ADDR));
#endif /* USE_I2C_EEPROM */

            PanoParam.nRAxisOffset = nOffset;
            
            printUart(DBG_MSG_PC, "nRAxisOffset = %d", PanoParam.nRAxisOffset);
        }
        else if (strstr(message.Data, UART_GEO_ALIGN_RAXIS_RESET))
        {
            int16_t nOffset = 0;
            
#ifdef USE_I2C_EEPROM
            EEPRom_EraseAddrWord(ROM_RAXIS_ALG_ADDR);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "R_AXIS_ADDR(0x%02x) : 0x%04x", ROM_RAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_RAXIS_ALG_ADDR));
#endif /* USE_I2C_EEPROM */

            PanoParam.nRAxisOffset = nOffset;

            printUart(DBG_MSG_PC, "nRAxisOffset = %d", PanoParam.nRAxisOffset);
        }
		else if (strstr(message.Data, UART_GEO_ALIGN_RAXIS_DEC_OFFSET))
		{
            int16_t nOffset = 0,ReadOffset=0;

			nOffset -= atoi(&(message.Data[9]));
            printUart(DBG_MSG_PC, "nOffset = %d", nOffset);

#ifdef USE_I2C_EEPROM
            ReadOffset = (int16_t)EEPRom_I2C_Read_Word(ROM_RAXIS_ALG_ADDR);
            if(ReadOffset==0xffff)
			{
				ReadOffset=0;
			}
			else if(ReadOffset<0) ReadOffset++;

			nOffset=ReadOffset+nOffset;
			
            EEPRom_I2C_Write_Word(ROM_RAXIS_ALG_ADDR, (uint16_t)nOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "R_AXIS_ADDR(0x%02x) : 0x%04x", ROM_RAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_RAXIS_ALG_ADDR));
#endif /* USE_I2C_EEPROM */

            PanoParam.nRAxisOffset = nOffset;
            printUart(DBG_MSG_PC, "nRAxisOffset = %d", PanoParam.nRAxisOffset);
		}
		else if (strstr(message.Data, UART_GEO_ALIGN_RAXIS_INC_OFFSET))
		{
            int16_t nOffset = 0,ReadOffset=0;
			
			nOffset += atoi(&(message.Data[9]));
            printUart(DBG_MSG_PC, "nOffset = %d", nOffset);

#ifdef USE_I2C_EEPROM            
			ReadOffset = (int16_t)EEPRom_I2C_Read_Word(ROM_RAXIS_ALG_ADDR);
			if(ReadOffset==0xffff)
			{
				ReadOffset=0;
			}
			else if(ReadOffset<0) ReadOffset++;

			nOffset=ReadOffset+nOffset;
			
            EEPRom_I2C_Write_Word(ROM_RAXIS_ALG_ADDR, (uint16_t)nOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "R_AXIS_ADDR(0x%02x) : +0x%04x", ROM_RAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_RAXIS_ALG_ADDR));
#endif /* USE_I2C_EEPROM */

            PanoParam.nRAxisOffset = nOffset;
            printUart(DBG_MSG_PC, "nRAxisOffset = %d", PanoParam.nRAxisOffset);
		}
		else if (strstr(message.Data, UART_GEO_ALIGN_GET_RAXIS_OFFSET))
		{
			int16_t nOffset = 0;
			nOffset=EEPRom_I2C_Read_Word(ROM_RAXIS_ALG_ADDR);
			if(nOffset==0xffff)
			{
				nOffset=0;
			}
			else if(nOffset<0) nOffset++;

			PanoParam.nRAxisOffset=nOffset;
            printUart(DBG_MSG_PC, "Current nRAxisOffset = %d", PanoParam.nRAxisOffset);            
		}        
        else if (strstr(message.Data, UART_GEO_ALIGN_VAXIS_STEP_UP))
        {        	
			TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
            Motor_MoveAbsolutePosition(&Motor_V, 200, 100, Motor_V.CurrentStep + GEOMETRY_ALIGN_VAXIS_STEP_ONE, 100);
            //Motor_MoveAbsolutePosition(&Motor_V, 1000, 100, Motor_R.CurrentStep + GEOMETRY_ALIGN_VAXIS_UP_STEP, 100);
        }
        else if (strstr(message.Data, UART_GEO_ALIGN_VAXIS_STEP_DOWN))
        {
        	TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
            Motor_MoveAbsolutePosition(&Motor_V, 200, 100, Motor_V.CurrentStep - GEOMETRY_ALIGN_VAXIS_STEP_ONE, 100);            
            //Motor_MoveAbsolutePosition(&Motor_V, 1000, 100, GEOMETRY_ALIGN_VAXIS_DOWN_STEP, 100);
        }
		else if (strstr(message.Data, UART_GEO_ALIGN_VAXIS_DEC_OFFSET))
		{
            int16_t nOffset = 0,ReadOffset=0;

			nOffset -= atoi(&(message.Data[9]));
            printUart(DBG_MSG_PC, "nOffset = %d", nOffset);

#ifdef USE_I2C_EEPROM
            ReadOffset = (int16_t)EEPRom_I2C_Read_Word(ROM_PAXIS_ALG_ADDR);
			if(ReadOffset==0xffff)
			{
				ReadOffset=0;
			}
			else if(ReadOffset<0) ReadOffset++;

			nOffset=ReadOffset+nOffset;
            EEPRom_I2C_Write_Word(ROM_PAXIS_ALG_ADDR, (uint16_t)nOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "PAXIS_ALG_ADDR(0x%02x) : 0x%04x", ROM_PAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_PAXIS_ALG_ADDR));
#endif /* USE_I2C_EEPROM */

            PanoParam.nVAxisOffset = nOffset;
            printUart(DBG_MSG_PC, "nVAxisOffset = %d", PanoParam.nVAxisOffset);
		}
		else if (strstr(message.Data, UART_GEO_ALIGN_VAXIS_INC_OFFSET))
		{
            int16_t nOffset = 0,ReadOffset=0;

			nOffset += atoi(&(message.Data[9]));
            printUart(DBG_MSG_PC, "nOffset = +%d", nOffset);

#ifdef USE_I2C_EEPROM
            ReadOffset = (int16_t)EEPRom_I2C_Read_Word(ROM_PAXIS_ALG_ADDR);
			if(ReadOffset==0xffff)
			{
				ReadOffset=0;
			}
			else if(ReadOffset<0) ReadOffset++;

			nOffset=ReadOffset+nOffset;
            EEPRom_I2C_Write_Word(ROM_PAXIS_ALG_ADDR, (uint16_t)nOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "nVAxisOffset(0x%02x) : +0x%04x", ROM_PAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_PAXIS_ALG_ADDR));
#endif /* USE_I2C_EEPROM */

            PanoParam.nVAxisOffset = nOffset;
            printUart(DBG_MSG_PC, "nVAxisOffset = +%d", PanoParam.nVAxisOffset);
		}
		else if (strstr(message.Data, UART_GEO_ALIGN_GET_VAXIS_OFFSET))
		{
			int16_t nOffset = 0;
			nOffset=EEPRom_I2C_Read_Word(ROM_PAXIS_ALG_ADDR);
			if(nOffset==0xffff)
			{
				nOffset=0;
			}
			else if(nOffset<0) nOffset++;

			PanoParam.nVAxisOffset=nOffset;
            printUart(DBG_MSG_PC, "Current nVAxisOffset = %d", PanoParam.nVAxisOffset);
		}        
        else if (strstr(message.Data, UART_GEO_ALIGN_VAXIS_STEP_STOP))
        {
            //while(Motor_V.Update != FALSE);
            Motor_V.Update=FALSE;
            Motor_Stop(&Motor_V);
			TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StopCurrent);
            printUart(DBG_MSG_PC, "Motor_V.CurrentStep(%d)", Motor_V.CurrentStep);
        }
        else if (strstr(message.Data, UART_GEO_ALIGN_VAXIS_SET))
        {
            int16_t nOffset = 0;

#ifdef USE_I2C_EEPROM
            nOffset = (int16_t)EEPRom_I2C_Read_Word(ROM_PAXIS_ALG_ADDR);
            if(nOffset==0xffff)
			{
				nOffset=0;
			}
			else if(nOffset<0) nOffset++;

			nOffset= nOffset+ (Motor_V.CurrentStep - PANO_V_STANDARD_OFFSET);

            printUart(DBG_MSG_PC, "nOffset = %d", nOffset);
            EEPRom_I2C_Write_Word(ROM_PAXIS_ALG_ADDR, (uint16_t)nOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "VAXIS_ALG_ADDR(0x%02x) : 0x%04x", ROM_PAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_PAXIS_ALG_ADDR));
#endif /* USE_I2C_EEPROM */

            PanoParam.nVAxisOffset = nOffset;
            
            printUart(DBG_MSG_PC, "nVAxisOffset = %d", PanoParam.nVAxisOffset);
        }        
        else if (strstr(message.Data, UART_GEO_ALIGN_VAXIS_RESET))
        {
            int16_t nOffset = 0;
#ifdef USE_I2C_EEPROM
            EEPRom_EraseAddrWord(ROM_PAXIS_ALG_ADDR);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "VAXIS_ALG_ADDR(0x%02x) : 0x%04x", ROM_PAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_PAXIS_ALG_ADDR));
#endif /* USE_I2C_EEPROM */

            PanoParam.nVAxisOffset = nOffset;
            printUart(DBG_MSG_PC, "nVAxisOffset = %d", PanoParam.nVAxisOffset);
        }        
        else if (strstr(message.Data, UART_CANINE_ALIGN_MOVE_VAXIS_INIT))
        {
        	TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
			Motor_RunOrgCheck(&Motor_V);
        	while(!Motor_GetStatus(&Motor_V, STATUS_STOP));
            TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StopCurrent);
            PanoParam.nCanineOffset = Motor_V.CurrentStep;
            printUart(DBG_MSG_PC, "nCanineOffset = %d", PanoParam.nCanineOffset);
        }   
        else if (strstr(message.Data, UART_LASER_ON))
        {
			LaserControl(TYPE_HEAD_PANO, SET);
			LaserControl(TYPE_FOOT, SET);
        }
        else if (strstr(message.Data, UART_LASER_OFF))
        {
			LaserControl(TYPE_HEAD_PANO, RESET);
			LaserControl(TYPE_FOOT, RESET);
        }
		else if (strstr(message.Data, UART_HEAD_BEAM_CUSTOM_ON))
        {
        	if (strchr(&(message.Data[13]), 'P'))
			{
				LaserControl(TYPE_HEAD_PANO, SET);
			}
			else if(strchr(&(message.Data[13]), 'C'))
			{
				LaserControl(TYPE_HEAD_CT, SET);
			}
			else if(strchr(&(message.Data[13]), 'S'))
			{
				LaserControl(TYPE_HEAD_CEPH, SET);
			}			
        }
        else if (strstr(message.Data, UART_HEAD_BEAM_CUSTOM_OFF))
        {
        	if (strchr(&(message.Data[13]), 'P'))
			{
				LaserControl(TYPE_HEAD_PANO, RESET);
			}
			else if(strchr(&(message.Data[13]), 'C'))
			{
				LaserControl(TYPE_HEAD_CT, RESET);
			}
			else if(strchr(&(message.Data[13]), 'S'))
			{
				LaserControl(TYPE_HEAD_CEPH, RESET);
			}			
        }	
        else if (strstr(message.Data, UART_CANINE_ALIGN_VAXIS_STEP_UP))
        {
            printUart(DBG_MSG_PC, "CurStep = %d", Motor_V.CurrentStep);
			TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
            Motor_MoveAbsolutePosition(&Motor_V, 200, 10, Motor_V.CurrentStep + CANINE_ALIGN_VAXIS_STEP, 10);
        }
        else if (strstr(message.Data, UART_CANINE_ALIGN_VAXIS_STEP_DOWN))
        {
            printUart(DBG_MSG_PC, "CurStep = %d", Motor_V.CurrentStep);
            TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
            Motor_MoveAbsolutePosition(&Motor_V, 200, 10, Motor_V.CurrentStep - CANINE_ALIGN_VAXIS_STEP, 10);
        }
        else if (strstr(message.Data, UART_CANINE_ALIGN_VAXIS_STOP))
        {
            //while(Motor_V.Update != FALSE);
            Motor_V.Update=FALSE;
            Motor_Stop(&Motor_V);
			TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StopCurrent);
            printUart(DBG_MSG_PC, "CurStep = %d", Motor_V.CurrentStep);
        }
        else if (strstr(message.Data, UART_CANINE_ALIGN_VAXIS_SET))
        {
            int16_t nOffset = (int16_t)Motor_V.CurrentStep;
#ifdef USE_I2C_EEPROM
            EEPRom_I2C_Write_Word(ROM_CANINE_ALIGN_ADDR, (uint16_t)nOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "CANINE_ALIGN_ADDR(0x%02x) : 0x%04x", ROM_CANINE_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CANINE_ALIGN_ADDR));
#endif /* USE_I2C_EEPROM */

            PanoParam.nCanineOffset = nOffset;            
            printUart(DBG_MSG_PC, "nCanineOffset = %d", PanoParam.nCanineOffset);
        }        
        else if (strstr(message.Data, UART_CANINE_ALIGN_VAXIS_RESET))
        {
            int16_t nOffset = 0;
            
#ifdef USE_I2C_EEPROM
            EEPRom_EraseAddrWord(ROM_CANINE_ALIGN_ADDR);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "CANINE_ALIGN_ADDR(0x%02x) : 0x%04x", ROM_CANINE_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CANINE_ALIGN_ADDR));
#endif /* USE_I2C_EEPROM */

            PanoParam.nCanineOffset = nOffset;
            printUart(DBG_MSG_PC, "nCanineOffset = %d", PanoParam.nCanineOffset);
        }
        else if (strstr(message.Data, UART_GEO_CEPH_ALIGN_RAXIS_STEP_UP))
        {	
        	TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
            Motor_MoveAbsolutePosition(&Motor_R, 50, 100, Motor_R.CurrentStep + GEOMETRY_ALIGN_RAXIS_STEP_ONE, 100);
        }
        else if (strstr(message.Data, UART_GEO_CEPH_ALIGN_RAXIS_STEP_DOWN))
        {
        	TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
            Motor_MoveAbsolutePosition(&Motor_R, 50, 100, Motor_R.CurrentStep - GEOMETRY_ALIGN_RAXIS_STEP_ONE, 100);
        }
        else if (strstr(message.Data, UART_GEO_CEPH_ALIGN_RAXIS_MANUAL_UP))
        {
            int16_t nOffset = 0;
			
			nOffset += atoi(&(message.Data[9]));
			
#ifdef USE_I2C_EEPROM
            EEPRom_I2C_Write_Word(ROM_CEPH_RAXIS_ALIGN_ADDR, (uint16_t)nOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "CEPH_RAXIS_ALIGN_ADDR(0x%02x) : +0x%04x", ROM_CEPH_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_RAXIS_ALIGN_ADDR));
#endif /* USE_I2C_EEPROM */

            CephParam.nRAxisOffset = nOffset;

            printUart(DBG_MSG_PC, "Ceph nRAxisOffset = +%d", CephParam.nRAxisOffset);
			
        }
        else if (strstr(message.Data, UART_GEO_CEPH_ALIGN_RAXIS_MANUAL_DOWN))
        {
            int16_t nOffset = 0;

			nOffset -= atoi(&(message.Data[9]));
            
            printUart(DBG_MSG_PC, "Ceph nOffset = %d", nOffset);
            
#ifdef USE_I2C_EEPROM
            EEPRom_I2C_Write_Word(ROM_CEPH_RAXIS_ALIGN_ADDR, (uint16_t)nOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "CEPH_RAXIS_ALIGN_ADDR(0x%02x) : 0x%04x", ROM_CEPH_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_RAXIS_ALIGN_ADDR));
#endif /* USE_I2C_EEPROM */

            CephParam.nRAxisOffset = nOffset;
            
            printUart(DBG_MSG_PC, "Ceph nRAxisOffset = %d", CephParam.nRAxisOffset);
        }
        else if (strstr(message.Data, UART_GEO_CEPH_ALIGN_RAXIS_STEP_STOP))
        {
			char str[32] = {0,};
			
            //while(Motor_R.Update != FALSE);
            Motor_R.Update=FALSE;
            Motor_Stop(&Motor_R);
			TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StopCurrent);
            sprintf(str, "Ceph Motor_R.CurrentStep(%d)", Motor_R.CurrentStep);
            UART_SendMessage(DBG_MSG_PC, str);			
        }
        else if (strstr(message.Data, UART_GEO_CEPH_ALIGN_RAXIS_SET))
        {
            int16_t nOffset = 0;

            if (Motor_R.CurrentStep < 75.0 / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO)) // RAxis deg 0
            {
#ifdef USE_I2C_EEPROM
                nOffset = Motor_R.CurrentStep - GEOMETRY_ALIGN_RAXIS_ROTATE_00D;
#else /* not USE_I2C_EEPROM */
                nOffset = Motor_R.CurrentStep - GEOMETRY_ALIGN_RAXIS_ROTATE_00D;
#endif /* USE_I2C_EEPROM */
            }
            else // RAxis deg 90
            {
#if defined(USE_I2C_EEPROM)                
				nOffset = Motor_R.CurrentStep - (CEPH_ROTATE_ANGLE/ (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO));
#else /* not USE_I2C_EEPROM */
				nOffset = Motor_R.CurrentStep - (CEPH_ROTATE_ANGLE / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO));
#endif /* USE_I2C_EEPROM */

            }

            printUart(DBG_MSG_PC, "Ceph nOffset = %d", nOffset);

			if( nOffset == 0xffff )
			{
				nOffset = 0;
			}			
#ifdef USE_I2C_EEPROM
            EEPRom_I2C_Write_Word(ROM_CEPH_RAXIS_ALIGN_ADDR, (uint16_t)nOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "CEPH_RAXIS_ALIGN_ADDR(0x%02x) : 0x%04x", ROM_CEPH_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_RAXIS_ALIGN_ADDR));
#endif /* USE_I2C_EEPROM */

			CephParam.nRAxisOffset = nOffset;
            
            printUart(DBG_MSG_PC, "Ceph nRAxisOffset = %d", CephParam.nRAxisOffset);
        }
        else if (strstr(message.Data, UART_GEO_CEPH_ALIGN_RAXIS_RESET))
        {
            int16_t nOffset = 0;
            
#ifdef USE_I2C_EEPROM
            EEPRom_EraseAddrWord(ROM_CEPH_RAXIS_ALIGN_ADDR);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "CEPH_RAXIS_ALIGN_ADDR(0x%02x) : 0x%04x", ROM_CEPH_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_RAXIS_ALIGN_ADDR));
#endif /* USE_I2C_EEPROM */

            CephParam.nRAxisOffset = nOffset;

            printUart(DBG_MSG_PC, "Ceph nRAxisOffset = %d", CephParam.nRAxisOffset);
        }
        else if (strstr(message.Data, UART_GEO_CEPH_ALIGN_RAXIS_OFFSET_READ))
        {
            int16_t nOffset = 0;
            
#ifdef USE_I2C_EEPROM
            nOffset = EEPRom_I2C_Read_Word(ROM_CEPH_RAXIS_ALIGN_ADDR);
			if(nOffset==0xffff)
			{
				nOffset=0;
			}
			else if(nOffset<0) nOffset++;
			CephParam.nRAxisOffset = nOffset;
            printUart(DBG_MSG_PC, "R_AXIS_ADDR(0x%02x) : 0x%04x", ROM_CEPH_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_RAXIS_ALIGN_ADDR));
#endif /* USE_I2C_EEPROM */

            printUart(DBG_MSG_PC, "nRAxisOffset = %d", CephParam.nRAxisOffset);
        }
        else if (strstr(message.Data, UART_GEO_CEPH_ALIGN_RAXIS_OFFSET_INIT))
        {
#ifdef USE_I2C_EEPROM
            EEPRom_EraseAddrWord(ROM_CEPH_RAXIS_ALIGN_ADDR);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "CEPH_RAXIS_ALIGN_ADDR(0x%02x) : 0x%04x", ROM_CEPH_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_RAXIS_ALIGN_ADDR));
#endif /* USE_I2C_EEPROM */

            CephParam.nRAxisOffset = 0;

            printUart(DBG_MSG_PC, "Ceph nRAxisOffset = %d", CephParam.nRAxisOffset);
        }
		else if (strstr(message.Data, UART_GEO_CEPH_POSITION_MOVE))
		{
			Motor_Ceph_AlignPosition();
            if (!CAN_Collimator_SendMessage(CMD_TILT_ON, 0, 0, 5000, 2))
            {
                return RESET;
            }
			
			if (!CAN_Collimator_SendMessage(CMD_FILTER_CENTER, 0, 0, 1000, 2))
            {
                return RESET;
            }
		}
        else if (strstr(message.Data, UART_CT_ALIGN_MOVE_CT_POS))
        {
            printUart(DBG_MSG_PC, "wait until the movement of the motor is finished...");
			TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
			TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
			TMC2660_SetCurrent(Motor_A.MotType, Motor_A.StartCurrent);
			Motor_MoveAbsolutePosition(&Motor_V, 15000, 1000, (CT_PAXIS_DIAMETER / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP)) + CtParam.nPaxisPatientOffset, 2000);	
			Motor_MoveAbsolutePosition(&Motor_R, 2000, 1000, CT_RAXIS_OFFSET + CtParam.nRaxisPatientOffset, 1000);
#ifdef USE_MOTOR_GANTRY_MS          
			Motor_MoveAbsolutePosition(&Motor_A, 12000, 1000, CT_AUTO_ANGLE / (360.0 / MOTOR_A_MICROSTEP / MOTOR_A_PULLEY_RATIO), 1000);
#endif /* USE_MOTOR_GANTRY_MS */    

        	while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
        	while(!Motor_GetStatus(&Motor_V, STATUS_STOP));
#ifdef USE_MOTOR_GANTRY_MS          
        	while(!Motor_GetStatus(&Motor_A, STATUS_STOP));
#endif /* USE_MOTOR_GANTRY_MS */

			TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StopCurrent);
			TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StopCurrent);
			TMC2660_SetCurrent(Motor_A.MotType, Motor_A.StopCurrent);
            printUart(DBG_MSG_PC, "motor movement is completed");
        }
        else if (strstr(message.Data, UART_CT_ALIGN_PAXIS_SET_DOWN))
        {
			signed short nOffset = (signed short)atoi(&(message.Data[11]));
            printUart(DBG_MSG_PC, "input diameter : -%d(mm)", nOffset);

            nOffset = -(nOffset / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP));
            printUart(DBG_MSG_PC, "converted step value : %d", nOffset);

            nOffset = CtParam.nPaxisOffset + nOffset;
            if ( (nOffset >= (CT_PAXIS_DIAMETER_MIN / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP))) && \
                (nOffset <= (CT_PAXIS_DIAMETER_MAX / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP))) ) {

                CtParam.nPaxisOffset = nOffset;
                
				TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
    			Motor_MoveAbsolutePosition(&Motor_V, 2000, 1000, (CT_PAXIS_DIAMETER/ (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP)) + CtParam.nPaxisOffset, 2000);	
            	while(!Motor_GetStatus(&Motor_V, STATUS_STOP));
				TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StopCurrent);
                printUart(DBG_MSG_PC, "CurrentStep : %d, nPaxisOffset(%d)", Motor_V.CurrentStep, CtParam.nPaxisOffset);


#ifdef USE_I2C_EEPROM
                EEPRom_I2C_Write_Word(ROM_CT_PAXIS_ALIGN_ADDR, (unsigned short)CtParam.nPaxisOffset);
                IntTimer_Delay(1000);
                while(!IntTimer_GetStatus());

                printUart(DBG_MSG_PC, "CT_PAXIS_ALIGN_ADDR(0x%02x) : 0x%04x", ROM_CT_PAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CT_PAXIS_ALIGN_ADDR));
#endif /* USE_I2C_EEPROM */
            }
			else 
			{
                printUart(DBG_MSG_PC, "wrong step value : (%d)", CtParam.nPaxisOffset);
            }
        }            
        else if (strstr(message.Data, UART_CT_ALIGN_PAXIS_SET_UP))
        {
			signed short nOffset = (signed short)atoi(&(message.Data[11]));
            printUart(DBG_MSG_PC, "input diameter : +%d(mm)", nOffset);

            nOffset = +(nOffset / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP));
            printUart(DBG_MSG_PC, "converted step value : %d", nOffset);

            nOffset = CtParam.nPaxisOffset + nOffset;            
            if ( (nOffset >= (CT_PAXIS_DIAMETER_MIN / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP))) && \
                (nOffset <= (CT_PAXIS_DIAMETER_MAX / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP))) ) {

                CtParam.nPaxisOffset = nOffset;

                printUart(DBG_MSG_PC, "nPaxisOffset(%d)", CtParam.nPaxisOffset);

				TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
    			Motor_MoveAbsolutePosition(&Motor_V, 2000, 1000, (CT_PAXIS_DIAMETER/ (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP)) + CtParam.nPaxisOffset, 2000);	
            	while(!Motor_GetStatus(&Motor_V, STATUS_STOP));
				TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StopCurrent);
                printUart(DBG_MSG_PC, "CurrentStep : %d, nPaxisOffset(%d)", Motor_V.CurrentStep, CtParam.nPaxisOffset);


#ifdef USE_I2C_EEPROM
                EEPRom_I2C_Write_Word(ROM_CT_PAXIS_ALIGN_ADDR, (unsigned short)CtParam.nPaxisOffset);
                IntTimer_Delay(1000);
                while(!IntTimer_GetStatus());

                printUart(DBG_MSG_PC, "CT_PAXIS_ALIGN_ADDR(0x%02x) : 0x%04x", ROM_CT_PAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CT_PAXIS_ALIGN_ADDR));
#endif /* USE_I2C_EEPROM */
            } else {
                printUart(DBG_MSG_PC, "wrong step value : (%d)", CtParam.nPaxisOffset);
            }
        }
        else if (strstr(message.Data, UART_CT_ALIGN_PAXIS_RESET))
        {
#ifdef USE_I2C_EEPROM
            EEPRom_EraseAddrWord(ROM_CT_PAXIS_ALIGN_ADDR);

            CtParam.nPaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_PAXIS_ALIGN_ADDR);
            if (CtParam.nPaxisOffset == (signed short)0xffff)
            {
                CtParam.nPaxisOffset = 0;
            }
#endif /* USE_I2C_EEPROM */
        }
        else if (strstr(message.Data, UART_CT_ALIGN_PAXIS_GET_OFFSET))
        {
#ifdef USE_I2C_EEPROM
            CtParam.nPaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_PAXIS_ALIGN_ADDR);
            if (CtParam.nPaxisOffset == (signed short)0xffff)
            {
                CtParam.nPaxisOffset = 0;
            }
			else if(CtParam.nPaxisOffset<0) CtParam.nPaxisOffset++;
#endif /* USE_I2C_EEPROM */

            printUart(DBG_MSG_PC, "CT P-Axis Offset = %d", CtParam.nPaxisOffset);;
        }
        else if (strstr(message.Data, UART_CT_ALIGN_RAXIS_SET_DOWN))
        {
			signed short nOffset = (signed short)atoi(&(message.Data[11]));
            printUart(DBG_MSG_PC, "input degree : -%d(')", nOffset);

            nOffset = -(nOffset / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO));
            printUart(DBG_MSG_PC, "converted step value : %d", nOffset);

            nOffset = CtParam.nRaxisOffset + nOffset;
            if ( (nOffset >= (CT_RAXIS_DEGREE_MIN / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))) && \
                (nOffset <= (CT_RAXIS_DEGREE_MAX / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))) ) {

                CtParam.nRaxisOffset = nOffset;
                
                printUart(DBG_MSG_PC, "nRaxisOffset(%d)", CtParam.nRaxisOffset);
                
#ifdef USE_I2C_EEPROM
                EEPRom_I2C_Write_Word(ROM_CT_RAXIS_ALIGN_ADDR, (unsigned short)CtParam.nRaxisOffset);
                IntTimer_Delay(1000);
                while(!IntTimer_GetStatus());

                printUart(DBG_MSG_PC, "CT_RAXIS_ALIGN_ADDR(0x%02x) : 0x%04x", ROM_CT_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CT_RAXIS_ALIGN_ADDR));
#endif /* USE_I2C_EEPROM */
            } else {
                printUart(DBG_MSG_PC, "wrong step value : (%d)", CtParam.nRaxisOffset);
            }
        }    
        else if (strstr(message.Data, UART_CT_ALIGN_RAXIS_SET_UP))
        {
			signed short nOffset = (signed short)atoi(&(message.Data[11]));
            printUart(DBG_MSG_PC, "input degree : +%d(')", nOffset);

            nOffset = +(nOffset / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO));
            printUart(DBG_MSG_PC, "converted step value : %d", nOffset);

            nOffset = CtParam.nRaxisOffset + nOffset;
            if ( (nOffset >= (CT_RAXIS_DEGREE_MIN / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))) && \
                (nOffset <= (CT_RAXIS_DEGREE_MAX / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))) ) {

                CtParam.nRaxisOffset = nOffset;

                printUart(DBG_MSG_PC, "nRaxisOffset(%d)", CtParam.nRaxisOffset);
                
#ifdef USE_I2C_EEPROM
                EEPRom_I2C_Write_Word(ROM_CT_RAXIS_ALIGN_ADDR, (unsigned short)CtParam.nRaxisOffset);
                IntTimer_Delay(1000);
                while(!IntTimer_GetStatus());

                printUart(DBG_MSG_PC, "CT_RAXIS_ALIGN_ADDR(0x%02x) : 0x%04x", ROM_CT_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CT_RAXIS_ALIGN_ADDR));
#endif /* USE_I2C_EEPROM */
            } else {
                printUart(DBG_MSG_PC, "wrong step value : (%d)", CtParam.nRaxisOffset);
            }
        }  
		else if (strstr(message.Data, UART_CT_ALIGN_RAXIS_RESET))
        {
#ifdef USE_I2C_EEPROM
            EEPRom_EraseAddrWord(ROM_CT_RAXIS_ALIGN_ADDR);

            CtParam.nRaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_RAXIS_ALIGN_ADDR);
            if (CtParam.nRaxisOffset == (signed short)0xffff)
            {
                CtParam.nRaxisOffset = 0;
            }
#endif /* USE_I2C_EEPROM */
			printUart(DBG_MSG_PC, "CT R-Axis Offset = %d", CtParam.nRaxisOffset);;
        }
        else if (strstr(message.Data, UART_CT_ALIGN_RAXIS_GET_OFFSET))
        {
#ifdef USE_I2C_EEPROM
            CtParam.nRaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_RAXIS_ALIGN_ADDR);
            if (CtParam.nRaxisOffset == (signed short)0xffff)
            {
                CtParam.nRaxisOffset = 0;
            }
			else if(CtParam.nRaxisOffset<0) CtParam.nRaxisOffset++;
#endif /* USE_I2C_EEPROM */

            printUart(DBG_MSG_PC, "CT R-Axis Offset = %d", CtParam.nRaxisOffset);;
        }
// 201110 HWAN
		else if (strstr(message.Data, UART_CT_ALIGN_FAST_RAXIS_SET_DOWN))
        {
			signed short nOffset = (signed short)atoi(&(message.Data[11]));
            printUart(DBG_MSG_PC, "input degree : -%d(')", nOffset);

            nOffset = -(nOffset / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO));
            printUart(DBG_MSG_PC, "converted step value : %d", nOffset);

            nOffset = CtParam.nFastRaxisOffset + nOffset;
            if ( (nOffset >= (CT_RAXIS_DEGREE_MIN / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))) && \
                (nOffset <= (CT_RAXIS_DEGREE_MAX / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))) ) 
            {

                CtParam.nFastRaxisOffset = nOffset;
                
                printUart(DBG_MSG_PC, "nFastRaxisOffset(%d)", CtParam.nFastRaxisOffset);
                
#ifdef USE_I2C_EEPROM
                EEPRom_I2C_Write_Word(ROM_CT_FAST_RAXIS_ALIGN_ADDR, (unsigned short)CtParam.nFastRaxisOffset);
                IntTimer_Delay(1000);
                while(!IntTimer_GetStatus());

                printUart(DBG_MSG_PC, "ROM_CT_FAST_RAXIS_ALIGN_ADDR(0x%02x) : 0x%04x", ROM_CT_FAST_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CT_FAST_RAXIS_ALIGN_ADDR));
#endif /* USE_I2C_EEPROM */
            } 
			else 
			{
                printUart(DBG_MSG_PC, "wrong step value : (%d)", CtParam.nFastRaxisOffset);
            }
        }    
        else if (strstr(message.Data, UART_CT_ALIGN_FAST_RAXIS_SET_UP))
        {
			signed short nOffset = (signed short)atoi(&(message.Data[11]));
            printUart(DBG_MSG_PC, "input degree : +%d(')", nOffset);

            nOffset = +(nOffset / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO));
            printUart(DBG_MSG_PC, "converted step value : %d", nOffset);

            nOffset = CtParam.nFastRaxisOffset + nOffset;
            if ( (nOffset >= (CT_RAXIS_DEGREE_MIN / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))) && \
                (nOffset <= (CT_RAXIS_DEGREE_MAX / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))) ) 
            {

                CtParam.nFastRaxisOffset = nOffset;

                printUart(DBG_MSG_PC, "nFastRaxisOffset(%d)", CtParam.nFastRaxisOffset);
                
#ifdef USE_I2C_EEPROM
                EEPRom_I2C_Write_Word(ROM_CT_FAST_RAXIS_ALIGN_ADDR, (unsigned short)CtParam.nFastRaxisOffset);
                IntTimer_Delay(1000);
                while(!IntTimer_GetStatus());

                printUart(DBG_MSG_PC, "ROM_CT_FAST_RAXIS_ALIGN_ADDR(0x%02x) : 0x%04x", ROM_CT_FAST_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CT_FAST_RAXIS_ALIGN_ADDR));
#endif /* USE_I2C_EEPROM */
            } 
			else 
			{
                printUart(DBG_MSG_PC, "wrong step value : (%d)", CtParam.nFastRaxisOffset);
            }
        }  
		else if (strstr(message.Data, UART_CT_ALIGN_FAST_RAXIS_RESET))
		{
#ifdef USE_I2C_EEPROM
			EEPRom_EraseAddrWord(ROM_CT_FAST_RAXIS_ALIGN_ADDR);

			CtParam.nFastRaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_FAST_RAXIS_ALIGN_ADDR);
			if (CtParam.nFastRaxisOffset == (signed short)0xffff)
			{
				CtParam.nFastRaxisOffset = 0;
			}
#endif /* USE_I2C_EEPROM */
			printUart(DBG_MSG_PC, "CT Fast R-Axis Offset = %d", CtParam.nFastRaxisOffset);;
		}
		else if (strstr(message.Data, UART_CT_ALIGN_FAST_RAXIS_GET_OFFSET))
        {
#ifdef USE_I2C_EEPROM
            CtParam.nFastRaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_FAST_RAXIS_ALIGN_ADDR);
            if (CtParam.nFastRaxisOffset == (signed short)0xffff)
            {
                CtParam.nFastRaxisOffset = 0;
            }
			else if(CtParam.nFastRaxisOffset<0) CtParam.nFastRaxisOffset++;
#endif /* USE_I2C_EEPROM */

            printUart(DBG_MSG_PC, "CT Fast R-Axis Offset = %d", CtParam.nFastRaxisOffset);;
        }
//191223 HWAN
		else if (strstr(message.Data, UART_CT_PATIENT_PAXIS_SET_DOWN))
		{
			signed short nOffset = (signed short)atoi(&(message.Data[10]));
			printUart(DBG_MSG_PC, "input diameter : -%d", nOffset);

			nOffset = -(nOffset / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP));
			printUart(DBG_MSG_PC, "converted step value : %d", nOffset);

			nOffset = CtParam.nPaxisPatientOffset + nOffset;
			if ( (nOffset >= (CT_PAXIS_DIAMETER_MIN / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP))) && \
				(nOffset <= (CT_PAXIS_DIAMETER_MAX / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP))) ) 
			{

				CtParam.nPaxisPatientOffset = nOffset;
				
				printUart(DBG_MSG_PC, "nPaxisPatientOffset(%d)", CtParam.nPaxisPatientOffset);

				TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
				Motor_MoveAbsolutePosition(&Motor_V, 2000, 1000, (CT_PAXIS_DIAMETER / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP)) + CtParam.nPaxisPatientOffset, 2000); 
				while(!Motor_GetStatus(&Motor_V, STATUS_STOP));
				TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StopCurrent);
				printUart(DBG_MSG_PC, "CurrentStep : %d, nPaxisPatientOffset(%d)", Motor_V.CurrentStep, CtParam.nPaxisPatientOffset);


#if defined(USE_I2C_EEPROM)
				EEPRom_I2C_Write_Word(ROM_CT_PAXIS_PATIENT_ALG_ADDR, (unsigned short)CtParam.nPaxisPatientOffset);
				IntTimer_Delay(1000);
				while(!IntTimer_GetStatus());

				printUart(DBG_MSG_PC, "ROM_CT_PAXIS_PATIENT_ALG_ADDR(0x%02x) : 0x%04x", ROM_CT_PAXIS_PATIENT_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_CT_PAXIS_PATIENT_ALG_ADDR));
#endif /* USE_I2C_EEPROM */
			} 
			else 
			{			
				printUart(DBG_MSG_PC, "wrong step value : (%d)", CtParam.nPaxisPatientOffset);
			}
		}			 
		else if (strstr(message.Data, UART_CT_PATIENT_PAXIS_SET_UP))
		{
			signed short nOffset = (signed short)atoi(&(message.Data[10]));
			printUart(DBG_MSG_PC, "input diameter : +%d", nOffset);

			nOffset = +(nOffset / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP));
			printUart(DBG_MSG_PC, "converted step value : %d", nOffset);

			nOffset = CtParam.nPaxisPatientOffset + nOffset;			 
			if ( (nOffset >= (CT_PAXIS_DIAMETER_MIN / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP))) && \
				(nOffset <= (CT_PAXIS_DIAMETER_MAX / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP))) ) 
			{

				CtParam.nPaxisPatientOffset = nOffset;

				TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
				Motor_MoveAbsolutePosition(&Motor_V, 2000, 1000, (CT_PAXIS_DIAMETER / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP)) + CtParam.nPaxisPatientOffset, 2000); 
				while(!Motor_GetStatus(&Motor_V, STATUS_STOP));
				TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StopCurrent);
				printUart(DBG_MSG_PC, "CurrentStep : %d, nPaxisPatientOffset(%d)", Motor_V.CurrentStep, CtParam.nPaxisPatientOffset);


#if defined(USE_I2C_EEPROM)
				EEPRom_I2C_Write_Word(ROM_CT_PAXIS_PATIENT_ALG_ADDR, (unsigned short)CtParam.nPaxisPatientOffset);
				IntTimer_Delay(1000);
				while(!IntTimer_GetStatus());

				printUart(DBG_MSG_PC, "ROM_CT_PAXIS_PATIENT_ALG_ADDR(0x%02x) : 0x%04x", ROM_CT_PAXIS_PATIENT_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_CT_PAXIS_PATIENT_ALG_ADDR));
#endif /* USE_I2C_EEPROM */
			} 
			else 
			{
				printUart(DBG_MSG_PC, "wrong step value : (%d)", CtParam.nPaxisPatientOffset);
			}
		}
		else if (strstr(message.Data, UART_CT_PATIENT_PAXIS_RESET))
		{
#if defined(USE_I2C_EEPROM)
			EEPRom_EraseAddrWord(ROM_CT_PAXIS_PATIENT_ALG_ADDR);
#endif /* USE_I2C_EEPROM */
			CtParam.nPaxisPatientOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_PAXIS_PATIENT_ALG_ADDR);
			if (CtParam.nPaxisPatientOffset == (signed short)0xffff)
			{
				CtParam.nPaxisPatientOffset = 0;
			}
		}
		else if (strstr(message.Data, UART_CT_PATIENT_PAXIS_GET_OFFSET))
		{
			CtParam.nPaxisPatientOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_PAXIS_PATIENT_ALG_ADDR);
			if (CtParam.nPaxisPatientOffset == (signed short)0xffff)
			{
				CtParam.nPaxisPatientOffset = 0;
			}
			else if(CtParam.nPaxisPatientOffset<0) CtParam.nPaxisPatientOffset++;

			printUart(DBG_MSG_PC, "CT P-Axis Patient Offset = %d", CtParam.nPaxisPatientOffset);;
		}
		else if (strstr(message.Data, UART_CT_PATIENT_RAXIS_SET_DOWN))
		{
			signed short nOffset = (signed short)atoi(&(message.Data[10]));
			printUart(DBG_MSG_PC, "input degree : -%d(')", nOffset);

			nOffset = -(nOffset / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO));
			printUart(DBG_MSG_PC, "converted step value : %d", nOffset);

			nOffset = CtParam.nRaxisPatientOffset + nOffset;
			if ( (nOffset >= (CT_RAXIS_DEGREE_MIN / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))) && \
				(nOffset <= (CT_RAXIS_DEGREE_MAX / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))) ) {

				CtParam.nRaxisPatientOffset = nOffset;
				
				printUart(DBG_MSG_PC, "nRaxisPatientOffset(%d)", CtParam.nRaxisPatientOffset);
				TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
				Motor_MoveAbsolutePosition(&Motor_R, 2000, 1000, CT_RAXIS_OFFSET +CtParam.nRaxisPatientOffset, 1000);
				while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
				TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StopCurrent);
#if defined(USE_I2C_EEPROM)
				EEPRom_I2C_Write_Word(ROM_CT_RAXIS_PATIENT_ALG_ADDR, (unsigned short)CtParam.nRaxisPatientOffset);
				IntTimer_Delay(1000);
				while(!IntTimer_GetStatus());

				printUart(DBG_MSG_PC, "ROM_CT_RAXIS_PATIENT_ALG_ADDR(0x%02x) : 0x%04x", ROM_CT_RAXIS_PATIENT_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_CT_RAXIS_PATIENT_ALG_ADDR));
#endif /* USE_I2C_EEPROM */
			} else {
				printUart(DBG_MSG_PC, "wrong step value : (%d)", CtParam.nRaxisPatientOffset);
			}
		}	
		else if (strstr(message.Data, UART_CT_PATIENT_RAXIS_SET_UP))
		{
			signed short nOffset = (signed short)atoi(&(message.Data[10]));
			printUart(DBG_MSG_PC, "input degree : +%d(')", nOffset);

			nOffset = +(nOffset / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO));
			printUart(DBG_MSG_PC, "converted step value : %d", nOffset);

			nOffset = CtParam.nRaxisPatientOffset + nOffset;
			if ( (nOffset >= (CT_RAXIS_DEGREE_MIN / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))) && \
				(nOffset <= (CT_RAXIS_DEGREE_MAX / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))) ) {

				CtParam.nRaxisPatientOffset = nOffset;

				printUart(DBG_MSG_PC, "nRaxisPatientOffset(%d)", CtParam.nRaxisPatientOffset);
				TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
				Motor_MoveAbsolutePosition(&Motor_R, 2000, 1000, CT_RAXIS_OFFSET +CtParam.nRaxisPatientOffset, 1000);
				while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
				TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StopCurrent);
#if defined(USE_I2C_EEPROM)
				EEPRom_I2C_Write_Word(ROM_CT_RAXIS_PATIENT_ALG_ADDR, (unsigned short)CtParam.nRaxisPatientOffset);
				IntTimer_Delay(1000);
				while(!IntTimer_GetStatus());

				printUart(DBG_MSG_PC, "ROM_CT_RAXIS_PATIENT_ALG_ADDR(0x%02x) : 0x%04x", ROM_CT_RAXIS_PATIENT_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_CT_RAXIS_PATIENT_ALG_ADDR));
#endif /* USE_I2C_EEPROM */
			} else {
				printUart(DBG_MSG_PC, "wrong step value : (%d)", CtParam.nRaxisPatientOffset);
			}
		}		 
		else if (strstr(message.Data, UART_CT_PATIENT_RAXIS_RESET))
		{
			EEPRom_EraseAddrWord(ROM_CT_RAXIS_PATIENT_ALG_ADDR);			  
			CtParam.nRaxisPatientOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_RAXIS_PATIENT_ALG_ADDR);
			if (CtParam.nRaxisPatientOffset == (signed short)0xffff)
			{
				CtParam.nRaxisPatientOffset = 0;
			}
		}
		else if (strstr(message.Data, UART_CT_PATIENT_RAXIS_GET_OFFSET))
		{			 
			CtParam.nRaxisPatientOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_RAXIS_PATIENT_ALG_ADDR);
			if (CtParam.nRaxisPatientOffset == (signed short)0xffff)
			{
				CtParam.nRaxisPatientOffset = 0;
			}
			else if(CtParam.nRaxisPatientOffset<0) CtParam.nRaxisPatientOffset++;

			printUart(DBG_MSG_PC, "CT R-Axis Patient Offset = %d", CtParam.nRaxisPatientOffset);;			  
		}		
		else if (strstr(message.Data, UART_CEPH_RAXIS_ALIGN_AROUND_SHOT))
		{			 
			int16_t nOffset = 0;
			int32_t Total_Move = 0;
			char str[40] = {0,};
			
			nOffset = atoi(&(message.Data[9]));
            
            printUart(DBG_MSG_PC, "Ceph Around Offset = %d", nOffset);

			if (AlignModeParam.KV == 0 || AlignModeParam.mA == 0)
            {
                char string1[UART_MSG_SIZE * 2 + 3];
                
                memset(string1, NULL, sizeof(char) * (UART_MSG_SIZE * 2 + 3));
                sprintf(string1, "%s : %s", UART_ALIGN_MSG_ERROR, message.Data);
                
                UART_SendMessage(DBG_MSG_PC, string1);
            }
			else if(Motor_R.CurrentStep - nOffset<0)
			{
				sprintf(str, "%s", UART_ALIGN_MOTOR_UNDER_ERROR);
            	UART_SendMessage(DBG_MSG_PC, str);
			}
			else if(Motor_R.CurrentStep + nOffset > 180000)
			{
				sprintf(str, "%s", UART_ALIGN_MOTOR_OVER_ERROR);
            	UART_SendMessage(DBG_MSG_PC, str);
			}
            else
            {
            	if (!CAN_Collimator_SendMessage(CMD_FILTER_CENTER, 0, 0, 1000, 2))
                {
                    return RESET;
                }
				
            	TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
				Motor_MoveAbsolutePosition(&Motor_R, 1000, 100, Motor_R.CurrentStep - nOffset, 100);
	    	    while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
				TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StopCurrent);

				UART_SendMessage(DBG_MSG_PC, "Enter into Communication with Tube");

	            if (!CAN_SendMessage(CAN_TUBE_COM_CHECK, CAN_TUBE_GEN2_COM, 0, 3000, 2))
	            {
	                return RESET;
	            }
	            
	            UART_SendMessage(DBG_MSG_PC, "Tube's Communication is checked");				

	            if (!CAN_SendMessage(CAN_TUBE_CONTI_MODE_SET, 0, 0, 3000, 2))
	            {
	                return RESET;
	            }

	            if (!CAN_SendMessage(CAN_TUBE_KV_SET, AlignModeParam.KV, 0, 3000, 2))
	            {
	                return RESET;
	            }

	            if (!CAN_SendMessage(CAN_TUBE_mA_SET, AlignModeParam.mA, 0, 3000, 2))
	            {
	                return RESET;
	            }

	            Exposure_CtrlLed(EXPOSURE_LED_GPIO_ENABLE);

				UART_SendMessage(DBG_MSG_PC, "[SP_STBY_EXPSW]");
				
	            while((Exposure_CheckSwitch()))
	            {
	                if (CurCaptureMode == CAPTURE_CANCEL)
	                {
	                	Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
                		Tube_CtrlReady(TUBE_READY_DISABLE);

                		Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
						UART_SendMessage(DBG_MSG_PC, "SHOT READY CANCELED");
	                    return SET;
	                }

					if(UART_Exposure_SET()==2)
					{
						Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
	            		Tube_CtrlReady(TUBE_READY_DISABLE);

	            		Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
						UART_SendMessage(DBG_MSG_PC, "SHOT READY CANCELED");
						return RESET;
					}
	            }				

	            Tube_CtrlReady(TUBE_READY_ENABLE);

	            UART_SendMessage(DBG_MSG_PC, "Wait for Tube Heatting Time");	

	            IntTimer_Delay(2000);
	            while(!IntTimer_GetStatus());
				Total_Move=Motor_R.CurrentStep + nOffset*2;
				TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
				Motor_MoveAbsolutePosition(&Motor_R, 300, 100, Motor_R.CurrentStep + nOffset*2, 100);

	            UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_START]");

	            Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
	            
#if 1 //[[ myshin_20160310_BEGIN -- �������� Flat ���� �ʾƼ� ������ 
                IntTimer_Delay(100);
                while(!IntTimer_GetStatus());
#endif //]] myshin_20160310_END -- �������� Flat ���� �ʾƼ� ������ 
                UART_SendMessage(DBG_MSG_PC, "SENSOR_S_OUTPUT_ENABLE");
                ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_ENABLE);                
	               
	            while(!Exposure_CheckSwitch())
	        	{
	        		if(UART_Exposure_SET()==2)
					{						
						break;
					}

					if(Motor_GetStatus(&Motor_R, STATUS_STOP))
					{
						UART_SendMessage(DBG_MSG_PC, "Motor_S End Position");
						break;
					}
	        	}
				//while(Motor_R.Update != FALSE);
				Motor_R.Update=FALSE;
			    Motor_Stop(&Motor_R);
			    TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StopCurrent);
	            ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE);
	            
	            Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
	            Tube_CtrlReady(TUBE_READY_DISABLE);

				UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_STOP_]");
				
	            UART_SendMessage(DBG_MSG_PC, "SENSOR_S_OUTPUT_DISABLE");

	            Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);

				sprintf(str, "Current Offset = %d", Motor_R.CurrentStep);
            	UART_SendMessage(DBG_MSG_PC, str);

	            UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_END__]");	

				if(Total_Move>Motor_R.CurrentStep)
					UART_SendMessage(DBG_MSG_PC, "[SP_RCEP_DETEC]");
				
            }  
		}
		else if (strstr(message.Data, UART_TUBE_VOLTAGE))
        {
            AlignModeParam.KV = atoi(&(message.Data[9])) / 100;
        }
        else if (strstr(message.Data, UART_TUBE_CURRENT))
        {
            AlignModeParam.mA = atoi(&(message.Data[9])) / 10;
        }
		else if (strstr(message.Data, UART_COLLIMATOR_OPEN))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_OPEN, 0, 0, 2000, 2))
            {
                return RESET;
            }
        }
		else if (strstr(message.Data, UART_CEPH_2COL_START_POS))
        {
            UART_SendMessage(DBG_MSG_PC, "UART_CEPH_2COL_START_POS Start");
			b2ndColStart = SET;
			
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
			Motor_RunOrgCheck(&Motor_S);
    	    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));

            Motor_CephMoveAbsolutePosition(&Motor_S, 2000, 1000, CEPH_2ND_COLI_ALIGN_POS + CephParam.n2ndColOffset /*+ CephParam.n2ndColStartOffset*/, 1000);
    	    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
            UART_SendMessage(DBG_MSG_PC, "UART_CEPH_2COL_START_POS End");     
        }
		else if (strstr(message.Data, UART_CEPH_2COL_END_POS))
        {
            UART_SendMessage(DBG_MSG_PC, "UART_CEPH_2COL_END_POS Start");
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
			Motor_RunOrgCheck(&Motor_S);
    	    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));

            Motor_CephMoveAbsolutePosition(&Motor_S, 2000, 1000, 300 * (MOTOR_C_MICROSTEP / (MOTOR_C_PULLEY_DIAMETER * pi))-(CEPH_2ND_COLI_ALIGN_POS) + CephParam.n2ndColEndOffset /*+100 + CephParam.n2ndColStartOffset*/, 1000);
    	    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
            UART_SendMessage(DBG_MSG_PC, "UART_CEPH_2COL_END_POS End");     
        }
        else if (strstr(message.Data, UART_CEPH_DET_START_POS))
        {
            UART_SendMessage(DBG_MSG_PC, "UART_CEPH_DET_START_POS Start");
			TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StartCurrent);
			Motor_RunOrgCheck(&Motor_C);
    	    while(!Motor_GetStatus(&Motor_C, STATUS_STOP));

            Motor_CephMoveAbsolutePosition(&Motor_C, 2000, 1000, CEPH_SCAN_ALIGN_POSITION * (MOTOR_C_MICROSTEP / (MOTOR_C_PULLEY_DIAMETER * pi)), 1000);
    	    while(!Motor_GetStatus(&Motor_C, STATUS_STOP));
			TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StopCurrent);
            UART_SendMessage(DBG_MSG_PC, "UART_CEPH_DET_START_POS End");     
        }
		else if (strstr(message.Data, UART_CEPH_DET_END_POS))
        {
            UART_SendMessage(DBG_MSG_PC, "UART_CEPH_DET_END_POS Start");
			TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StartCurrent);
			Motor_RunOrgCheck(&Motor_C);
    	    while(!Motor_GetStatus(&Motor_C, STATUS_STOP));

            Motor_CephMoveAbsolutePosition(&Motor_C, 2000, 1000, 300.0 * (MOTOR_C_MICROSTEP / (MOTOR_C_PULLEY_DIAMETER * pi)), 1000);
    	    while(!Motor_GetStatus(&Motor_C, STATUS_STOP));
			TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StopCurrent);
            UART_SendMessage(DBG_MSG_PC, "UART_CEPH_DET_START_POS End");     
        }
//191014 HWAN 
// PANO CNS Offset
		else if (strstr(message.Data, UART_GEO_ALIGN_PANO_CNSAXIS_STEP_UP))
        {
        	int16_t nOffset = PanoParam.nCNSaxisOffset;
            printUart(DBG_MSG_PC, "nOffset = +1");
			nOffset++;

			bChin_res=Motor_ControlChinrest(SET, nOffset+MOTOR_CNS_PANO_INIT_POSITION);

			if(bChin_res==SET)
			{
				PanoParam.nCNSaxisOffset = nOffset;
            	printUart(DBG_MSG_PC, "Pano_nCNSAxisOffset = %d", PanoParam.nCNSaxisOffset);
			}            
        }
        else if (strstr(message.Data, UART_GEO_ALIGN_PANO_CNSAXIS_STEP_DOWN))
        {
            int16_t nOffset = PanoParam.nCNSaxisOffset;
            printUart(DBG_MSG_PC, "nOffset = -1");
			nOffset--;

			bChin_res=Motor_ControlChinrest(SET, nOffset+MOTOR_CNS_PANO_INIT_POSITION);

            if(bChin_res==SET)
			{
				PanoParam.nCNSaxisOffset = nOffset;
            	printUart(DBG_MSG_PC, "Pano_nCNSAxisOffset = %d", PanoParam.nCNSaxisOffset);
			}   
        }
        else if (strstr(message.Data, UART_GEO_ALIGN_PANO_CNSAXIS_SET))
        {
            int16_t nOffset = 0;
			
            nOffset = PanoParam.nCNSaxisOffset;
            printUart(DBG_MSG_PC, "Current Pano_nCNSAxisOffset = %d.", nOffset);

#ifdef USE_I2C_EEPROM
            EEPRom_I2C_Write_Word(ROM_PANO_CNSAXIS_ALG_ADDR, (uint16_t)nOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());		

            printUart(DBG_MSG_PC, "PANO_CNS_AXIS_ADDR(0x%02x) : 0x%04x", ROM_PANO_CNSAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_PANO_CNSAXIS_ALG_ADDR));
#endif /* USE_I2C_EEPROM */            
            printUart(DBG_MSG_PC, "Pano_nCNSAxisOffset = %d", PanoParam.nCNSaxisOffset);

			CtParam.nCNSaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_CNSAXIS_ALG_ADDR);
			if (CtParam.nCNSaxisOffset== (signed short)0xffff)
			{
				CtParam.nCNSaxisOffset = PanoParam.nCNSaxisOffset;
			}
			else if(CtParam.nCNSaxisOffset<0) CtParam.nCNSaxisOffset++;

        }
        else if (strstr(message.Data, UART_GEO_ALIGN_PANO_CNSAXIS_RESET))
        {
        	printUart(DBG_MSG_PC, "Pano_nCNSAxisOffset RESET");
#ifdef USE_I2C_EEPROM
            EEPRom_EraseAddrWord(ROM_PANO_CNSAXIS_ALG_ADDR);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "PANO_CNS_AXIS_ADDR(0x%02x) : 0x%04x", ROM_PANO_CNSAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_PANO_CNSAXIS_ALG_ADDR));
#endif /* USE_I2C_EEPROM */

            PanoParam.nCNSaxisOffset = 0;

            printUart(DBG_MSG_PC, "Pano_nCNSAxisOffset = %d", PanoParam.nCNSaxisOffset);

			CtParam.nCNSaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_CNSAXIS_ALG_ADDR);
			if (CtParam.nCNSaxisOffset== (signed short)0xffff)
			{
				CtParam.nCNSaxisOffset = PanoParam.nCNSaxisOffset;
			}
			else if(CtParam.nCNSaxisOffset<0) CtParam.nCNSaxisOffset++;
        }
		else if (strstr(message.Data, UART_GEO_ALIGN_PANO_CNSAXIS_DEC))
		{
            int16_t nOffset = PanoParam.nCNSaxisOffset;

			nOffset -= atoi(&(message.Data[12]));
            printUart(DBG_MSG_PC, "nOffset = %d", nOffset);

			bChin_res=Motor_ControlChinrest(SET, nOffset+MOTOR_CNS_PANO_INIT_POSITION);

			if(bChin_res==SET)
			{
				PanoParam.nCNSaxisOffset = nOffset;
            	printUart(DBG_MSG_PC, "Pano_nCNSAxisOffset = %d", PanoParam.nCNSaxisOffset);
			}   
		}
		else if (strstr(message.Data, UART_GEO_ALIGN_PANO_CNSAXIS_INC))
		{
            int16_t nOffset = PanoParam.nCNSaxisOffset;
			
			nOffset += atoi(&(message.Data[12]));
            printUart(DBG_MSG_PC, "nOffset = %d", nOffset);
			
			bChin_res=Motor_ControlChinrest(SET, nOffset+MOTOR_CNS_PANO_INIT_POSITION);
			
            if(bChin_res==SET)
			{
				PanoParam.nCNSaxisOffset = nOffset;
            	printUart(DBG_MSG_PC, "Pano_nCNSAxisOffset = %d", PanoParam.nCNSaxisOffset);
			}   
		}
		else if (strstr(message.Data, UART_GEO_ALIGN_GET_PANO_CNSAXIS_OFFSET))
		{
			PanoParam.nCNSaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_PANO_CNSAXIS_ALG_ADDR);
			if (PanoParam.nCNSaxisOffset== (signed short)0xffff)
			{
				PanoParam.nCNSaxisOffset=0;
			}
			else if(PanoParam.nCNSaxisOffset<0) PanoParam.nCNSaxisOffset++;
            printUart(DBG_MSG_PC, "Current Pano_nCNSAxisOffset = %d", PanoParam.nCNSaxisOffset);            
		}       
// PANO CWE Offset
		else if (strstr(message.Data, UART_GEO_ALIGN_PANO_CWEAXIS_STEP_UP))
        {
        	int16_t nOffset = PanoParam.nCWEaxisOffset;
            printUart(DBG_MSG_PC, "nOffset = +1");
			nOffset++;

			bChin_res=Motor_ControlChinrest(RESET, (double)nOffset/10+MOTOR_CWE_INIT_POSITION);

			if(bChin_res==SET)
			{
				PanoParam.nCWEaxisOffset = nOffset;
				if(PanoParam.nCWEaxisOffset>=0)
            		printUart(DBG_MSG_PC, "Pano_nCWEaxisOffset = %d.%d", PanoParam.nCWEaxisOffset/10,PanoParam.nCWEaxisOffset%10);
				else
					printUart(DBG_MSG_PC, "Pano_nCWEaxisOffset = %d.%d", PanoParam.nCWEaxisOffset/10,((PanoParam.nCWEaxisOffset%10)*-1));
			}  
        }
        else if (strstr(message.Data, UART_GEO_ALIGN_PANO_CWEAXIS_STEP_DOWN))
        {
            int16_t nOffset = PanoParam.nCWEaxisOffset;
            printUart(DBG_MSG_PC, "nOffset = -1");
			nOffset--;

			bChin_res=Motor_ControlChinrest(RESET, (double)nOffset/10+MOTOR_CWE_INIT_POSITION);

            if(bChin_res==SET)
			{
				PanoParam.nCWEaxisOffset = nOffset;
            	if(PanoParam.nCWEaxisOffset>=0)
            		printUart(DBG_MSG_PC, "Pano_nCWEaxisOffset = %d.%d", PanoParam.nCWEaxisOffset/10,PanoParam.nCWEaxisOffset%10);
				else
					printUart(DBG_MSG_PC, "Pano_nCWEaxisOffset = %d.%d", PanoParam.nCWEaxisOffset/10,((PanoParam.nCWEaxisOffset%10)*-1));
			}  
        }
        else if (strstr(message.Data, UART_GEO_ALIGN_PANO_CWEAXIS_SET))
        {
            int16_t nOffset = 0;
			
            nOffset = PanoParam.nCWEaxisOffset;
			if(nOffset>=0)
            	printUart(DBG_MSG_PC, "Current PANO_nCWEaxisOffset = %d.%d", nOffset/10,nOffset%10);
			else
				printUart(DBG_MSG_PC, "Current PANO_nCWEaxisOffset = %d.%d", nOffset/10,((nOffset%10)*-1));

#ifdef USE_I2C_EEPROM
            EEPRom_I2C_Write_Word(ROM_PANO_CWEAXIS_ALG_ADDR, (uint16_t)nOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "PANO_CWE_AXIS_ADDR(0x%02x) : 0x%04x", ROM_PANO_CWEAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_PANO_CWEAXIS_ALG_ADDR));
#endif /* USE_I2C_EEPROM */            
            if(PanoParam.nCWEaxisOffset>=0)
        		printUart(DBG_MSG_PC, "Pano_nCWEaxisOffset = %d.%d", PanoParam.nCWEaxisOffset/10,PanoParam.nCWEaxisOffset%10);
			else
				printUart(DBG_MSG_PC, "Pano_nCWEaxisOffset = %d.%d", PanoParam.nCWEaxisOffset/10,((PanoParam.nCWEaxisOffset%10)*-1));

			CtParam.nCWEaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_CWEAXIS_ALG_ADDR);
		    if (CtParam.nCWEaxisOffset == (signed short)0xffff)
		    {
		        CtParam.nCWEaxisOffset = PanoParam.nCWEaxisOffset;
		    }
			else if(CtParam.nCWEaxisOffset<0) CtParam.nCWEaxisOffset++;
        }
        else if (strstr(message.Data, UART_GEO_ALIGN_PANO_CWEAXIS_RESET))
        {
        	printUart(DBG_MSG_PC, "PANO_nCWEaxisOffset RESET");
#ifdef USE_I2C_EEPROM
            EEPRom_EraseAddrWord(ROM_PANO_CWEAXIS_ALG_ADDR);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "PANO_CWE_AXIS_ADDR(0x%02x) : 0x%04x", ROM_PANO_CWEAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_PANO_CWEAXIS_ALG_ADDR));
#endif /* USE_I2C_EEPROM */

            PanoParam.nCWEaxisOffset = 0;

            printUart(DBG_MSG_PC, "PANO_nCWEaxisOffset = %d", PanoParam.nCWEaxisOffset);

			CtParam.nCWEaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_CWEAXIS_ALG_ADDR);
		    if (CtParam.nCWEaxisOffset == (signed short)0xffff)
		    {
		        CtParam.nCWEaxisOffset = PanoParam.nCWEaxisOffset;
		    }
			else if(CtParam.nCWEaxisOffset<0) CtParam.nCWEaxisOffset++;
        }
		else if (strstr(message.Data, UART_GEO_ALIGN_PANO_CWEAXIS_DEC))
		{
            int16_t nOffset = PanoParam.nCWEaxisOffset;

			nOffset -= atoi(&(message.Data[12]));
            printUart(DBG_MSG_PC, "nOffset = %d", nOffset);

			bChin_res=Motor_ControlChinrest(RESET, (double)nOffset/10+MOTOR_CWE_INIT_POSITION);

            if(bChin_res==SET)
			{
				PanoParam.nCWEaxisOffset = nOffset;
            	if(PanoParam.nCWEaxisOffset>=0)
            		printUart(DBG_MSG_PC, "Pano_nCWEaxisOffset = %d.%d", PanoParam.nCWEaxisOffset/10,PanoParam.nCWEaxisOffset%10);
				else
					printUart(DBG_MSG_PC, "Pano_nCWEaxisOffset = %d.%d", PanoParam.nCWEaxisOffset/10,((PanoParam.nCWEaxisOffset%10)*-1));
			}  
		}
		else if (strstr(message.Data, UART_GEO_ALIGN_PANO_CWEAXIS_INC))
		{
            int16_t nOffset = PanoParam.nCWEaxisOffset;
			
			nOffset += atoi(&(message.Data[12]));
            printUart(DBG_MSG_PC, "nOffset = %d", nOffset);
			
			bChin_res=Motor_ControlChinrest(RESET, (double)nOffset/10+MOTOR_CWE_INIT_POSITION);
			
            if(bChin_res==SET)
			{
				PanoParam.nCWEaxisOffset = nOffset;
            	if(PanoParam.nCWEaxisOffset>=0)
            		printUart(DBG_MSG_PC, "Pano_nCWEaxisOffset = %d.%d", PanoParam.nCWEaxisOffset/10,PanoParam.nCWEaxisOffset%10);
				else
					printUart(DBG_MSG_PC, "Pano_nCWEaxisOffset = %d.%d", PanoParam.nCWEaxisOffset/10,((PanoParam.nCWEaxisOffset%10)*-1));
			}  
		}
		else if (strstr(message.Data, UART_GEO_ALIGN_GET_PANO_CWEAXIS_OFFSET))
		{
			PanoParam.nCWEaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_PANO_CWEAXIS_ALG_ADDR);
		    if (PanoParam.nCWEaxisOffset == (signed short)0xffff)
		    {
		        PanoParam.nCWEaxisOffset = 0;
		    }
			else if(PanoParam.nCWEaxisOffset<0) PanoParam.nCWEaxisOffset++;
		
            if(PanoParam.nCWEaxisOffset>=0)
        		printUart(DBG_MSG_PC, "Pano_nCWEaxisOffset = %d.%d", PanoParam.nCWEaxisOffset/10,PanoParam.nCWEaxisOffset%10);
			else
				printUart(DBG_MSG_PC, "Pano_nCWEaxisOffset = %d.%d", PanoParam.nCWEaxisOffset/10,((PanoParam.nCWEaxisOffset%10)*-1));           
		}      
//CT CNS Offset
		else if (strstr(message.Data, UART_GEO_ALIGN_CT_CNSAXIS_STEP_UP))
		{
			int16_t nOffset = CtParam.nCNSaxisOffset;
			printUart(DBG_MSG_PC, "nOffset = +1");
			nOffset++;

			bChin_res=Motor_ControlChinrest(SET, nOffset+MOTOR_CNS_CT_INIT_POSITION);

			if(bChin_res==SET)
			{
				CtParam.nCNSaxisOffset = nOffset;
				printUart(DBG_MSG_PC, "CT_nCNSAxisOffset = %d", CtParam.nCNSaxisOffset);
			}  
		}
		else if (strstr(message.Data, UART_GEO_ALIGN_CT_CNSAXIS_STEP_DOWN))
		{
			int16_t nOffset = CtParam.nCNSaxisOffset;
			printUart(DBG_MSG_PC, "nOffset = -1");
			nOffset--;

			bChin_res=Motor_ControlChinrest(SET, nOffset+MOTOR_CNS_CT_INIT_POSITION);

			if(bChin_res==SET)
			{
				CtParam.nCNSaxisOffset = nOffset;
				printUart(DBG_MSG_PC, "CT_nCNSAxisOffset = %d", CtParam.nCNSaxisOffset);
			}  
		}
		else if (strstr(message.Data, UART_GEO_ALIGN_CT_CNSAXIS_SET))
		{
			int16_t nOffset = 0;
			
			nOffset = CtParam.nCNSaxisOffset;
			printUart(DBG_MSG_PC, "Current CT_nCNSAxisOffset = %d.", nOffset);

#ifdef USE_I2C_EEPROM
			EEPRom_I2C_Write_Word(ROM_CT_CNSAXIS_ALG_ADDR, (uint16_t)nOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());

			printUart(DBG_MSG_PC, "CT_CNS_AXIS_ADDR(0x%02x) : 0x%04x", ROM_CT_CNSAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_CT_CNSAXIS_ALG_ADDR));
#endif /* USE_I2C_EEPROM */            
			printUart(DBG_MSG_PC, "CT_nCNSAxisOffset = %d", CtParam.nCNSaxisOffset);
		}
		else if (strstr(message.Data, UART_GEO_ALIGN_CT_CNSAXIS_RESET))
		{
			printUart(DBG_MSG_PC, "CT_nCNSAxisOffset RESET");
#ifdef USE_I2C_EEPROM
			EEPRom_EraseAddrWord(ROM_CT_CNSAXIS_ALG_ADDR);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());

			printUart(DBG_MSG_PC, "CT_CNS_AXIS_ADDR(0x%02x) : 0x%04x", ROM_CT_CNSAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_CT_CNSAXIS_ALG_ADDR));
#endif /* USE_I2C_EEPROM */

			CtParam.nCNSaxisOffset = 0;

			printUart(DBG_MSG_PC, "CT_nCNSAxisOffset = %d", CtParam.nCNSaxisOffset);
		}
		else if (strstr(message.Data, UART_GEO_ALIGN_CT_CNSAXIS_DEC))
		{
			int16_t nOffset = CtParam.nCNSaxisOffset;

			nOffset -= atoi(&(message.Data[11]));
			printUart(DBG_MSG_PC, "nOffset = %d", nOffset);

			bChin_res=Motor_ControlChinrest(SET, nOffset+MOTOR_CNS_CT_INIT_POSITION);

			if(bChin_res==SET)
			{
				CtParam.nCNSaxisOffset = nOffset;
				printUart(DBG_MSG_PC, "CT_nCNSAxisOffset = %d", CtParam.nCNSaxisOffset);
			} 
		}
		else if (strstr(message.Data, UART_GEO_ALIGN_CT_CNSAXIS_INC))
		{
			int16_t nOffset = CtParam.nCNSaxisOffset;
			
			nOffset += atoi(&(message.Data[11]));
			printUart(DBG_MSG_PC, "nOffset = %d", nOffset);
			
			bChin_res=Motor_ControlChinrest(SET, nOffset+MOTOR_CNS_CT_INIT_POSITION);
			
			if(bChin_res==SET)
			{
				CtParam.nCNSaxisOffset = nOffset;
				printUart(DBG_MSG_PC, "CT_nCNSAxisOffset = %d", CtParam.nCNSaxisOffset);
			}
		}
		else if (strstr(message.Data, UART_GEO_ALIGN_GET_CT_CNSAXIS_OFFSET))
		{
			CtParam.nCNSaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_CNSAXIS_ALG_ADDR);
		    if (CtParam.nCNSaxisOffset == (signed short)0xffff)
		    {
		        CtParam.nCNSaxisOffset=0;
		    }
			else if(CtParam.nCNSaxisOffset<0) CtParam.nCNSaxisOffset++;
			
			printUart(DBG_MSG_PC, "Current CT_nCNSAxisOffset = %d", CtParam.nCNSaxisOffset); 		   
		}		
// CT CWE Offset
		else if (strstr(message.Data, UART_GEO_ALIGN_CT_CWEAXIS_STEP_UP))
		{
			int16_t nOffset = CtParam.nCWEaxisOffset;
			printUart(DBG_MSG_PC, "nOffset = +1");
			nOffset++;

			bChin_res=Motor_ControlChinrest(FALSE, (double)nOffset/10+MOTOR_CWE_INIT_POSITION);

			if(bChin_res==SET)
			{				
				CtParam.nCWEaxisOffset = nOffset;
				if(CtParam.nCWEaxisOffset>=0)
					printUart(DBG_MSG_PC, "CT_nCWEaxisOffset = %d.%d", CtParam.nCWEaxisOffset/10,CtParam.nCWEaxisOffset%10);
				else
					printUart(DBG_MSG_PC, "CT_nCWEaxisOffset = %d.%d", CtParam.nCWEaxisOffset/10,((CtParam.nCWEaxisOffset%10)*-1));
			}			
		}
		else if (strstr(message.Data, UART_GEO_ALIGN_CT_CWEAXIS_STEP_DOWN))
		{
			int16_t nOffset = CtParam.nCWEaxisOffset;
			printUart(DBG_MSG_PC, "nOffset = -1");
			nOffset--;

			bChin_res=Motor_ControlChinrest(RESET, (double)nOffset/10+MOTOR_CWE_INIT_POSITION);

			if(bChin_res==SET)
			{				
				CtParam.nCWEaxisOffset = nOffset;
				if(CtParam.nCWEaxisOffset>=0)
					printUart(DBG_MSG_PC, "CT_nCWEaxisOffset = %d.%d", CtParam.nCWEaxisOffset/10,CtParam.nCWEaxisOffset%10);
				else
					printUart(DBG_MSG_PC, "CT_nCWEaxisOffset = %d.%d", CtParam.nCWEaxisOffset/10,((CtParam.nCWEaxisOffset%10)*-1));
			}
		}
		else if (strstr(message.Data, UART_GEO_ALIGN_CT_CWEAXIS_SET))
		{
			int16_t nOffset = 0;
			
			nOffset = CtParam.nCWEaxisOffset;
			if(nOffset>=0)
				printUart(DBG_MSG_PC, "Current CT_nCWEaxisOffset = %d.%d", nOffset/10,nOffset%10);
			else
				printUart(DBG_MSG_PC, "Current CT_nCWEaxisOffset = %d.%d", nOffset/10,((nOffset%10)*-1));

#ifdef USE_I2C_EEPROM
			EEPRom_I2C_Write_Word(ROM_CT_CWEAXIS_ALG_ADDR, (uint16_t)nOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());

			printUart(DBG_MSG_PC, "CT_CWE_AXIS_ADDR(0x%02x) : 0x%04x", ROM_CT_CWEAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_CT_CWEAXIS_ALG_ADDR));
#endif /* USE_I2C_EEPROM */            
			if(CtParam.nCWEaxisOffset>=0)
				printUart(DBG_MSG_PC, "CT_nCWEaxisOffset = %d.%d", CtParam.nCWEaxisOffset/10,CtParam.nCWEaxisOffset%10);
			else
				printUart(DBG_MSG_PC, "CT_nCWEaxisOffset = %d.%d", CtParam.nCWEaxisOffset/10,((CtParam.nCWEaxisOffset%10)*-1));
		}
		else if (strstr(message.Data, UART_GEO_ALIGN_CT_CWEAXIS_RESET))
		{
			printUart(DBG_MSG_PC, "CT_nCWEaxisOffset RESET");
#ifdef USE_I2C_EEPROM
			EEPRom_EraseAddrWord(ROM_CT_CWEAXIS_ALG_ADDR);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());

			printUart(DBG_MSG_PC, "CT_CWE_AXIS_ADDR(0x%02x) : 0x%04x", ROM_CT_CWEAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_CT_CWEAXIS_ALG_ADDR));
#endif /* USE_I2C_EEPROM */

			CtParam.nCWEaxisOffset = 0;

			printUart(DBG_MSG_PC, "CT_nCWEaxisOffset = %d", CtParam.nCWEaxisOffset);
		}
		else if (strstr(message.Data, UART_GEO_ALIGN_CT_CWEAXIS_DEC))
		{
			int16_t nOffset = CtParam.nCWEaxisOffset;

			nOffset -= atoi(&(message.Data[11]));
			printUart(DBG_MSG_PC, "nOffset = %d", nOffset);

			bChin_res=Motor_ControlChinrest(FALSE, (double)nOffset/10+MOTOR_CWE_INIT_POSITION);

			if(bChin_res==SET)
			{				
				CtParam.nCWEaxisOffset = nOffset;
				if(CtParam.nCWEaxisOffset>=0)
					printUart(DBG_MSG_PC, "CT_nCWEaxisOffset = %d.%d", CtParam.nCWEaxisOffset/10,CtParam.nCWEaxisOffset%10);
				else
					printUart(DBG_MSG_PC, "CT_nCWEaxisOffset = %d.%d", CtParam.nCWEaxisOffset/10,((CtParam.nCWEaxisOffset%10)*-1));
			}
		}
		else if (strstr(message.Data, UART_GEO_ALIGN_CT_CWEAXIS_INC))
		{
			int16_t nOffset = CtParam.nCWEaxisOffset;
			
			nOffset += atoi(&(message.Data[11]));
			printUart(DBG_MSG_PC, "nOffset = %d", nOffset);
			
			bChin_res=Motor_ControlChinrest(FALSE, (double)nOffset/10+MOTOR_CWE_INIT_POSITION);
			
			if(bChin_res==SET)
			{				
				CtParam.nCWEaxisOffset = nOffset;
				if(CtParam.nCWEaxisOffset>=0)
					printUart(DBG_MSG_PC, "CT_nCWEaxisOffset = %d.%d", CtParam.nCWEaxisOffset/10,CtParam.nCWEaxisOffset%10);
				else
					printUart(DBG_MSG_PC, "CT_nCWEaxisOffset = %d.%d", CtParam.nCWEaxisOffset/10,((CtParam.nCWEaxisOffset%10)*-1));
			}
		}
		else if (strstr(message.Data, UART_GEO_ALIGN_GET_CT_CWEAXIS_OFFSET))
		{
			CtParam.nCWEaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_CWEAXIS_ALG_ADDR);
		    if (CtParam.nCWEaxisOffset == (signed short)0xffff)
		    {
		        CtParam.nCWEaxisOffset=0;
		    }
			else if(CtParam.nCWEaxisOffset<0) CtParam.nCWEaxisOffset++;
			
			if(CtParam.nCWEaxisOffset>=0)
				printUart(DBG_MSG_PC, "CT_nCWEaxisOffset = %d.%d", CtParam.nCWEaxisOffset/10,CtParam.nCWEaxisOffset%10);
			else
				printUart(DBG_MSG_PC, "CT_nCWEaxisOffset = %d.%d", CtParam.nCWEaxisOffset/10,((CtParam.nCWEaxisOffset%10)*-1));	   
		} 
		else if (strstr(message.Data, UART_GET_TUBE_TEMP))
		{
			CAN_SendMessage(CAN_TUBE_TANK_TEMP_READ, 0, 0, 1000, 2);
		}	
// 201112 Temple Support offset
		else if (strstr(message.Data, UART_EAR_ROD_PUSH))
		{
			if(Temple_Status==PUSHED)
				Temple_Status=RELEASED;
			Motor_MoveTempleSupport(SET);
		}
		else if (strstr(message.Data, UART_EAR_ROD_CHILD_PUSH))
		{
			if(Temple_Status==PUSHED)
				Temple_Status=RELEASED;
			Motor_MoveTempleSupport_Child(SET);
		}
		else if (strstr(message.Data, UART_EAR_ROD_RELEASE))
		{
			if(Temple_Status==RELEASED)
				Temple_Status=PUSHED;
			Motor_MoveTempleSupport(RESET);
		}
		else if (strstr(message.Data, UART_GEO_ALIGN_TAXIS_STEP_UP))
        {
        	TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StartCurrent);
            Motor_MoveAbsolutePosition(&Motor_T, 1000, 500, GEOMETRY_ALIGN_TAXIS_MAX, 500);	//Max value check
        }
        else if (strstr(message.Data, UART_GEO_ALIGN_TAXIS_STEP_DOWN))
        {
        	TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StartCurrent);
            Motor_MoveAbsolutePosition(&Motor_T, 1000, 500, GEOMETRY_ALIGN_TAXIS_MIN, 500);   //Min value check
        }
        else if (strstr(message.Data, UART_GEO_ALIGN_TAXIS_STEP_STOP))
        {
            //while(Motor_R.Update != RESET);
            Motor_T.Update=RESET;
            Motor_Stop(&Motor_T);				
			TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StopCurrent);
            printUart(DBG_MSG_PC, "nTAxisCurrentSteps = %d", Motor_T.CurrentStep);
        }
		else if (strstr(message.Data, UART_GEO_ALIGN_TAXIS_MOVE_STEP))
        {
        	int16_t nOffset = 0;

			if (strchr(&(message.Data[10]), 'P'))
				nOffset += atoi(&(message.Data[11]));
			else if(strchr(&(message.Data[10]), 'R'))
				nOffset -= atoi(&(message.Data[11]));
			else
				return RESET;		

			printUart(DBG_MSG_PC, "Move %d mm", nOffset);
			
        	TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StartCurrent);
            Motor_MoveAbsolutePosition(&Motor_T, 3000, 1000, Motor_T.CurrentStep+(nOffset*100), 1000);
			while(!Motor_GetStatus(&Motor_T, STATUS_STOP));
            TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StopCurrent);

			printUart(DBG_MSG_PC, "nTAxisCurrentSteps = %d", Motor_T.CurrentStep);
        }
        else if (strstr(message.Data, UART_GEO_ALIGN_TAXIS_SET))
        {
            int16_t nOffset = 0;
            
            printUart(DBG_MSG_PC, "nTAxisCurrentSteps = %d", Motor_T.CurrentStep);
			nOffset=Motor_T.CurrentStep-8000;
#ifdef USE_I2C_EEPROM
            EEPRom_I2C_Write_Word(ROM_TEMPLE_SUPPORT_STEP_ADDR, (uint16_t)nOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "ROM_TEMPLE_SUPPORT_STEP_ADDR(0x%02x) : 0x%04x", ROM_TEMPLE_SUPPORT_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_TEMPLE_SUPPORT_STEP_ADDR));
#endif /* USE_I2C_EEPROM */

            PanoParam.nTAxisOffset = nOffset;
            
            printUart(DBG_MSG_PC, "nTAxisOffset = %d", PanoParam.nTAxisOffset);
        }
        else if (strstr(message.Data, UART_GEO_ALIGN_TAXIS_RESET))
        {
            int16_t nOffset = 0;
            
#ifdef USE_I2C_EEPROM
            EEPRom_EraseAddrWord(ROM_TEMPLE_SUPPORT_STEP_ADDR);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "ROM_TEMPLE_SUPPORT_STEP_ADDR(0x%02x) : 0x%04x", ROM_TEMPLE_SUPPORT_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_TEMPLE_SUPPORT_STEP_ADDR));
#endif /* USE_I2C_EEPROM */

            PanoParam.nTAxisOffset = nOffset;

            printUart(DBG_MSG_PC, "nTAxisOffset = %d", PanoParam.nTAxisOffset);
        }
		else if (strstr(message.Data, UART_GEO_ALIGN_TAXIS_OFFSET))
        {
            printUart(DBG_MSG_PC, "Current Temple Support Offset = %d", PanoParam.nTAxisOffset);
        }
        else if (strstr(message.Data, UART_GEO_ALIGN_TAXIS_CHILD_SET))
        {
            int16_t nOffset = 0;
            
            printUart(DBG_MSG_PC, "nTAxisChildCurrentSteps = %d", Motor_T.CurrentStep);
			nOffset=Motor_T.CurrentStep-8000;
#ifdef USE_I2C_EEPROM
            EEPRom_I2C_Write_Word(ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR, (uint16_t)nOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR(0x%02x) : 0x%04x", ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR));
#endif /* USE_I2C_EEPROM */

            PanoParam.nTAxisChildOffset = nOffset;
            
            printUart(DBG_MSG_PC, "nTAxisChildOffset = %d", PanoParam.nTAxisChildOffset);
        }
        else if (strstr(message.Data, UART_GEO_ALIGN_TAXIS_CHILD_RESET))
        {
            int16_t nOffset = 1000;
            
#ifdef USE_I2C_EEPROM
            EEPRom_EraseAddrWord(ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR(0x%02x) : 0x%04x", ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR));
#endif /* USE_I2C_EEPROM */

            PanoParam.nTAxisChildOffset = nOffset;

            printUart(DBG_MSG_PC, "nTAxisChildOffset = %d", PanoParam.nTAxisChildOffset);
        }
		else if (strstr(message.Data, UART_GEO_ALIGN_TAXIS_CHILD_OFFSET))
        {
        	PanoParam.nTAxisChildOffset = (signed short)EEPRom_I2C_Read_Word(ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR);
		    if (PanoParam.nTAxisChildOffset == (signed short)0xffff)
		    {
		        PanoParam.nTAxisChildOffset=0;
		    }
			else if(PanoParam.nTAxisChildOffset<0) PanoParam.nTAxisChildOffset++;
			
            printUart(DBG_MSG_PC, "Current Temple Support ChildOffset = %d", PanoParam.nTAxisChildOffset);
        }
#ifdef USE_MOTOR_CHINREST_HOR
		else if (strstr(message.Data, UART_CMD_CHINREST_HOR_MOVE))
		{
			int32_t nDistance = atoi(&(message.Data[12]));

			printUart(DBG_MSG_PC, "H-Axis Distance = %dmm", nDistance);

			if (nDistance >= 0)
			{
				//if (strchr(&(message.Data[11]), 'L'))
				if (strchr(&(message.Data[11]), 'R'))
					nDistance = ~nDistance + 1;

				//추후 23?�라??값을 Align Offset?�로 ?�정??�?
				Motor_ControlChinrest(RESET, MOTOR_CWE_INIT_POSITION + nDistance);
				//Motor_MoveChinrestHorizontal(nDistance);
			}
		}
#endif /* USE_MOTOR_CHINREST_HOR */
#ifdef USE_MOTOR_CHINREST_VER
		else if (strstr(message.Data, UART_CMD_CHINREST_VER_MOVE))
		{
			int32_t nDistance = atoi(&(message.Data[12]));

			printUart(DBG_MSG_PC, "V-Axis Distance = %dmm", nDistance);

			if (nDistance >= 0)
			{
				if (strchr(&(message.Data[11]), 'D'))
					nDistance = ~nDistance + 1;

				//추후 58?�라??값을 Align Offset?�로 ?�정??�?
				Motor_ControlChinrest(SET, MOTOR_CNS_CT_INIT_POSITION + nDistance);
				//Motor_MoveChinrestHorizontal(nDistance);
			}
		}
#endif /* USE_MOTOR_CHINREST_VER */		
        else
        {
            printUart(DBG_MSG_PC, "%s : %s", ERR_CODE_UART_MSG_ERROR, message.Data);
            return RESET;
        }

        message.Data[1] = 'E';
        printUart(DBG_MSG_PC, message.Data);
    }
	
	return bRet;
}

/**

/* Calibration handler: extracted from serial.c lines 4982-7363 */
* @ Function Name : UART_SensorCalibration
* @ Desc : 
* @ Param : 
* @ Return :
*/
bool UART_SensorCalibration(void)
{
    bool ret = RESET;
    UART_MsgTypedef message;

    while(MSG_QueueCnt(&UART_Queue))
    {
        message = MSG_Dequeue(&UART_Queue);
            
        if (strstr(message.Data, UART_CALIBRATION_EXIT))
        {
            ret = SET;
        }
		else if (strstr(message.Data, UART_CEPH_OFFSET))
		{  // jehun - 20200827
    		printUart(DBG_MSG_PC, "Ceph Normal Speed Offset : %d", CephParam.n2ndSpeedStepOffset);
    		printUart(DBG_MSG_PC, "Ceph Fast Speed Offset : %d", CephParam.n2ndFastSpeedStepOffset);
        }
        else if (strstr(message.Data, UART_TUBE_VOLTAGE))
        {
            CalibrationModeParam.KV = atoi(&(message.Data[9])) / 100;
        }
        else if (strstr(message.Data, UART_TUBE_CURRENT))
        {
            CalibrationModeParam.mA = atoi(&(message.Data[9])) / 10;
        }
        else if (strstr(message.Data, UART_CALIBRATION_CT))
        {
            CalibrationModeParam.mode = DET_TYPE_CT;
            CalibrationModeParam.AlignType = COL_TYPE_NONE;
        }
        else if (strstr(message.Data, UART_CALIBRATION_PANORAMA))
        {
            CalibrationModeParam.mode = DET_TYPE_PANORAMA;
            CalibrationModeParam.AlignType = COL_TYPE_NONE;
        }
        else if (strstr(message.Data, UART_CALIBRATION_CEPH_SCAN))
        {
            if (!CAN_Collimator_SendMessage(CMD_TILT_ON, 0, 0, 5000, 2))
            {
                return RESET;
            }
            g_bTiltStatus = SET;
            CalibrationModeParam.mode = DET_TYPE_CEPH_SCAN;
            CalibrationModeParam.AlignType = COL_TYPE_NONE;
        }
        else if (strstr(message.Data, UART_CALIBRATION_CEPH_ONESHOT))
        {
            CalibrationModeParam.mode = DET_TYPE_CEPH_ONESHOT;
        }
        else if (strstr(message.Data, UART_ALIGN_CT_MODE))
        {
            CalibrationModeParam.mode = DET_TYPE_CT;            
            CalibrationModeParam.AlignType = COL_TYPE_CT;
        }
        else if (strstr(message.Data, UART_ALIGN_PANO_MODE))
        {
            CalibrationModeParam.mode = DET_TYPE_PANORAMA;            
            CalibrationModeParam.AlignType = COL_TYPE_PANO;
        }
        else if (strstr(message.Data, UART_ALIGN_SCAN_MODE))
        {
            CalibrationModeParam.mode = DET_TYPE_CEPH_SCAN;            
            CalibrationModeParam.AlignType = COL_TYPE_CEPH_SCAN;
        }        
        else if (strstr(message.Data, UART_ALIGN_PANO_COLLIMATOR_TOP))
        {
            //CalibrationModeParam.AlignType = COL_TYPE_PANO;
            CalibrationModeParam.ColCmd = CMD_OFFSET_PANO_T;
        }    
        else if (strstr(message.Data, UART_ALIGN_PANO_COLLIMATOR_BOTTOM))
        {
            //CalibrationModeParam.AlignType = COL_TYPE_PANO;
            CalibrationModeParam.ColCmd = CMD_OFFSET_PANO_B;            
        }    
        else if (strstr(message.Data, UART_ALIGN_PANO_COLLIMATOR_LEFT))
        {
            //CalibrationModeParam.AlignType = COL_TYPE_PANO;
            CalibrationModeParam.ColCmd = CMD_OFFSET_PANO_L;            
        }    
        else if (strstr(message.Data, UART_ALIGN_PANO_COLLIMATOR_RIGHT))
        {
            //CalibrationModeParam.AlignType = COL_TYPE_PANO;
            CalibrationModeParam.ColCmd = CMD_OFFSET_PANO_R;            
        }    
        else if (strstr(message.Data, UART_ALIGN_PANO_COLLIMATOR_STEP_INC)) // Collimator Sensor Align 문서 ?�의 - 방향
        {
            if ((CalibrationModeParam.AlignType != COL_TYPE_NONE) && (CalibrationModeParam.ColCmd != 0))
            {
                uint32_t nStep = atoi(&(message.Data[9]));
                {
                    if (!CAN_Collimator_SendMessage(CalibrationModeParam.ColCmd, nStep, 1, 1000, 2))
                    {
                        return RESET;
                    }
                }
            }
        }    
        else if (strstr(message.Data, UART_ALIGN_PANO_COLLIMATOR_STEP_DEC)) // Collimator Sensor Align 문서 ?�의 + 방향
        {
            if ((CalibrationModeParam.AlignType != COL_TYPE_NONE) && (CalibrationModeParam.ColCmd != 0))
            {
                uint32_t nStep = atoi(&(message.Data[9]));
                {
                    if (!CAN_Collimator_SendMessage(CalibrationModeParam.ColCmd, nStep, 0, 1000, 2))
                    {
                        return RESET;
                    }
                }
            }
        }
        else if (strstr(message.Data, UART_ALIGN_PANO_COLLIMATOR_REQ_TOP))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_PANO_T, 0, 0, 0, 0))
            {
                return RESET;
            }
        }
        else if (strstr(message.Data, UART_ALIGN_PANO_COLLIMATOR_REQ_BOTTOM))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_PANO_B, 0, 0, 0, 0))
            {
                return RESET;
            }
        }
        else if (strstr(message.Data, UART_ALIGN_PANO_COLLIMATOR_REQ_LEFT))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_PANO_L, 0, 0, 0, 0))
            {
                return RESET;
            }
        }
        else if (strstr(message.Data, UART_ALIGN_PANO_COLLIMATOR_REQ_RIGHT))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_PANO_R, 0, 0, 0, 0))
            {
                return RESET;
            }
        }
		else if (strstr(message.Data, UART_ALIGN_PANO_1COL_START_POS))
        {                                         
            if (!CAN_Collimator_SendMessage(CMD_COLLI_PANOR, 0, 0, 2000, 2))
            {
                return RESET;
            }    
		}
        else if (strstr(message.Data, UART_COLLIMATOR_CLOSE))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CLOSE, 0, 0, 2000, 2))
            {
                return RESET;
            }    
            bClose = SET;
			bOpen = RESET;
        }
        else if (strstr(message.Data, UART_COLLIMATOR_OPEN))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_OPEN, 0, 0, 2000, 2))
            {
                return RESET;
            }
			bOpen = SET;
			bClose = RESET;
        }
		else if (strstr(message.Data, UART_FOV_5X5))
		{
			if (!CAN_Collimator_SendMessage(CMD_COLLI_CT5X5, 0, 0, 1000, 2))
            {
                return RESET;
            }    
			CalibrationModeParam.nFovCmd = CMD_COLLI_CT5X5;
		}
        else if (strstr(message.Data, UART_FOV_8X9))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CT8X9, 0, 0, 1000, 2))
            {
                return RESET;
            }    
            CalibrationModeParam.nFovCmd = CMD_COLLI_CT8X9;
        }	
        else if (strstr(message.Data, UART_FOV_10X9))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CT10X9, 0, 0, 1000, 2))
            {
                return RESET;
            }    
            CalibrationModeParam.nFovCmd = CMD_COLLI_CT10X9;
        }	
        else if (strstr(message.Data, UART_FOV_12X9))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CT12X9, 0, 0, 1000, 2))
            {
                return RESET;
            }    
            CalibrationModeParam.nFovCmd = CMD_COLLI_CT12X9;
        }	
        else if (strstr(message.Data, UART_FOV_15X9))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CT15X9, 0, 0, 1000, 2))
            {
                return RESET;
            }    
            CalibrationModeParam.nFovCmd = CMD_COLLI_CT15X9;
        }
        else if (strstr(message.Data, UART_FOV_10X10))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CT10X10, 0, 0, 1000, 2))
            {
                return RESET;
            }    
            CalibrationModeParam.nFovCmd = CMD_COLLI_CT10X10;
        }
        else if (strstr(message.Data, UART_FOV_12X10))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CT12X10, 0, 0, 1000, 2))
            {
                return RESET;
            }    
            CalibrationModeParam.nFovCmd = CMD_COLLI_CT12X10;
        }
        else if (strstr(message.Data, UART_FOV_15X10))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CT15X10, 0, 0, 1000, 2))
            {
                return RESET;
            }
            CalibrationModeParam.nFovCmd = CMD_COLLI_CT15X10;
        }
        else if (strstr(message.Data, UART_FOV_8X10))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CT8X10, 0, 0, 1000, 2))
            {
                return RESET;
            }
            CalibrationModeParam.nFovCmd = CMD_COLLI_CT8X10;
        }
        else if (strstr(message.Data, UART_ALIGN_CT_COLI_REQ_TOP))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CT_T, 0, 0, 5000, 0))
            {
                return RESET;
            }
        }
        else if (strstr(message.Data, UART_ALIGN_CT_COLI_REQ_BOTTOM))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CT_B, 0, 0, 5000, 0))
            {
                return RESET;
            }    
        }
        else if (strstr(message.Data, UART_ALIGN_CT_COLI_REQ_LEFT))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CT_L, 0, 0, 5000, 0))
            {
                return RESET;
            }
        }
        else if (strstr(message.Data, UART_ALIGN_CT_COLI_REQ_RIGHT))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CT_R, 0, 0, 5000, 0))
            {
                return RESET;
            }
        }
        else if (strstr(message.Data, UART_ALIGN_CT_COLI_TOP))
        {
            //CalibrationModeParam.AlignType = COL_TYPE_CEPH_SCAN;
            CalibrationModeParam.ColCmd = CMD_OFFSET_CT_T;
        }
        else if (strstr(message.Data, UART_ALIGN_CT_COLI_BOTTOM))
        {
            //CalibrationModeParam.AlignType = COL_TYPE_CEPH_SCAN;
            CalibrationModeParam.ColCmd = CMD_OFFSET_CT_B;
        }
        else if (strstr(message.Data, UART_ALIGN_CT_COLI_LEFT))
        {
            //CalibrationModeParam.AlignType = COL_TYPE_CEPH_SCAN;
            CalibrationModeParam.ColCmd = CMD_OFFSET_CT_L;
        }
        else if (strstr(message.Data, UART_ALIGN_CT_COLI_RIGHT))
        {
            //CalibrationModeParam.AlignType = COL_TYPE_CEPH_SCAN;
            CalibrationModeParam.ColCmd = CMD_OFFSET_CT_R;
        }
        else if (strstr(message.Data, UART_ALIGN_SCAN_1COL_CENTER))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLI_CEPHSCAN_CENTER, 0, 0, 3000, 2))
            {
                return RESET;
            }    
        }        
        else if (strstr(message.Data, UART_ALIGN_SCAN_1COL_START_POS))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CEPHSCAN_INIT, 0, 0, 2000, 2))
            {
                return RESET;
            }    
        }        
        else if (strstr(message.Data, UART_ALIGN_SCAN_COLI_REQ_TOP))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CEPHSCAN_T, 0, 0, 5000, 0))
            {
                return RESET;
            }    
        }
        else if (strstr(message.Data, UART_ALIGN_SCAN_COLI_REQ_BOTTOM))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CEPHSCAN_B, 0, 0, 5000, 0))
            {
                return RESET;
            }    
        }
        else if (strstr(message.Data, UART_ALIGN_SCAN_COLI_REQ_LEFT))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CEPHSCAN_L, 0, 0, 5000, 0))
            {
                return RESET;
            }  
        }
        else if (strstr(message.Data, UART_ALIGN_SCAN_COLI_REQ_RIGHT))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CEPHSCAN_R, 0, 0, 5000, 0))
            {
                return RESET;
            }    
        }
        else if (strstr(message.Data, UART_ALIGN_SCAN_COLI_TOP))
        {
            //CalibrationModeParam.AlignType = COL_TYPE_CEPH_SCAN;
            CalibrationModeParam.ColCmd = CMD_OFFSET_CEPHSCAN_T;
        }
        else if (strstr(message.Data, UART_ALIGN_SCAN_COLI_BOTTOM))
        {
            //CalibrationModeParam.AlignType = COL_TYPE_CEPH_SCAN;
            CalibrationModeParam.ColCmd = CMD_OFFSET_CEPHSCAN_B;
        }
        else if (strstr(message.Data, UART_ALIGN_SCAN_COLI_LEFT))
        {
            //CalibrationModeParam.AlignType = COL_TYPE_CEPH_SCAN;
            CalibrationModeParam.ColCmd = CMD_OFFSET_CEPHSCAN_L;
        }
        else if (strstr(message.Data, UART_ALIGN_SCAN_COLI_RIGHT))
        {
            //CalibrationModeParam.AlignType = COL_TYPE_CEPH_SCAN;
            CalibrationModeParam.ColCmd = CMD_OFFSET_CEPHSCAN_R;
        }
        else if (strstr(message.Data, UART_CEPH_2ND_COLI_1ST_ALIGN_POSITION))
        {
            UART_SendMessage(DBG_MSG_PC, "2ND_COLI_1ST_ALIGN_POSITION Start");
            TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
            //Motor_CephMoveAbsolutePosition(&Motor_S, 2000, 1000, 50.0 * (MOTOR_S_MICROSTEP / (MOTOR_S_PULLEY_DIAMETER * pi)), 1000);
            Motor_RunOrgCheck(&Motor_S);
    	    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
            UART_SendMessage(DBG_MSG_PC, "2ND_COLI_1ST_ALIGN_POSITION End");            
        }
        else if (strstr(message.Data, UART_CEPH_2ND_COLI_CENTER))
        {
            UART_SendMessage(DBG_MSG_PC, "UART_CEPH_2ND_COLI_CENTER Start");
			
    		b2ndColStart = RESET;
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
			Motor_RunOrgCheck(&Motor_S);
    	    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));

            Motor_CephMoveAbsolutePosition(&Motor_S, 2000, 1000, (CEPH_SCAN_CALIBRATION_POSITION * (MOTOR_S_MICROSTEP / (MOTOR_S_PULLEY_DIAMETER * pi))) + CephParam.n2ndColOffset, 1000);
    	    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
            UART_SendMessage(DBG_MSG_PC, "UART_CEPH_2ND_COLI_CENTER End");            
        }
        else if (strstr(message.Data, UART_CEPH_2COL_START_POS))
        {
            UART_SendMessage(DBG_MSG_PC, "UART_CEPH_2COL_START_POS Start");
			b2ndColStart = SET;
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
			Motor_RunOrgCheck(&Motor_S);
    	    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));

            Motor_CephMoveAbsolutePosition(&Motor_S, 2000, 1000, CEPH_2ND_COLI_ALIGN_POS + CephParam.n2ndColOffset /*+ CephParam.n2ndColStartOffset*/, 1000);
    	    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
            UART_SendMessage(DBG_MSG_PC, "UART_CEPH_2COL_START_POS End");     
        }
		else if (strstr(message.Data, UART_CEPH_2COL_FAST_START_POS))
        {
            UART_SendMessage(DBG_MSG_PC, "UART_CEPH_2COL_FAST_START_POS Start");
			b2ndColStart = SET;
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
			Motor_RunOrgCheck(&Motor_S);
    	    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));

            Motor_CephMoveAbsolutePosition(&Motor_S, 2000, 1000, CEPH_2ND_COLI_ALIGN_POS + CephParam.n2ndColFastStartOffset /*+ CephParam.n2ndColStartOffset*/, 1000);
    	    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
            UART_SendMessage(DBG_MSG_PC, "UART_CEPH_2COL_START_POS End");     
        }
		else if (strstr(message.Data, UART_CEPH_2COL_END_POS))
        {
            UART_SendMessage(DBG_MSG_PC, "UART_CEPH_2COL_END_POS Start");
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
			Motor_RunOrgCheck(&Motor_S);
    	    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));

            Motor_CephMoveAbsolutePosition(&Motor_S, 2000, 1000, 300 * (MOTOR_C_MICROSTEP / (MOTOR_C_PULLEY_DIAMETER * pi))-(CEPH_2ND_COLI_ALIGN_POS) + CephParam.n2ndColEndOffset /*+100 + CephParam.n2ndColStartOffset*/, 1000);
    	    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
            UART_SendMessage(DBG_MSG_PC, "UART_CEPH_2COL_END_POS End");     
        }
        else if (strstr(message.Data, UART_CEPH_DET_START_POS))
        {
            UART_SendMessage(DBG_MSG_PC, "UART_CEPH_DET_START_POS Start");
			TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StartCurrent);
			Motor_RunOrgCheck(&Motor_C);
    	    while(!Motor_GetStatus(&Motor_C, STATUS_STOP));

            Motor_CephMoveAbsolutePosition(&Motor_C, 2000, 1000, CEPH_SCAN_ALIGN_POSITION * (MOTOR_C_MICROSTEP / (MOTOR_C_PULLEY_DIAMETER * pi)), 1000);
    	    while(!Motor_GetStatus(&Motor_C, STATUS_STOP));
			TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StopCurrent);
            UART_SendMessage(DBG_MSG_PC, "UART_CEPH_DET_START_POS End");     
        }
		else if (strstr(message.Data, UART_CEPH_DET_END_POS))
        {
            UART_SendMessage(DBG_MSG_PC, "UART_CEPH_DET_END_POS Start");
			TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StartCurrent);
			Motor_RunOrgCheck(&Motor_C);
    	    while(!Motor_GetStatus(&Motor_C, STATUS_STOP));

            Motor_CephMoveAbsolutePosition(&Motor_C, 2000, 1000, 300.0 * (MOTOR_C_MICROSTEP / (MOTOR_C_PULLEY_DIAMETER * pi)), 1000);
    	    while(!Motor_GetStatus(&Motor_C, STATUS_STOP));
			TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StopCurrent);
            UART_SendMessage(DBG_MSG_PC, "UART_CEPH_DET_END_POS End");     
        }
#if 0 //190730 HWAN 		
		else if (strstr(message.Data, UART_ALIGN_SCAN_2COL_SPEED_VALUE_UP))
		{			
			char str[40] = {0,};			
			int16_t nOffset = 0;
			nOffset = atoi(&(message.Data[10]));
			CephParam.CEPH_2ND_Colli_Offset+=nOffset;
			
			sprintf(str, "n2ndCol_Speed_Offset = %d", CephParam.CEPH_2ND_Colli_Offset);
			UART_SendMessage(DBG_MSG_PC, str);			
		}
		else if (strstr(message.Data, UART_ALIGN_SCAN_2COL_SPEED_VALUE_DN))
		{
			char str[40] = {0,};			
			int16_t nOffset = 0;
			nOffset = atoi(&(message.Data[10]));
			CephParam.CEPH_2ND_Colli_Offset-=nOffset;
			
			sprintf(str, "n2ndCol_Speed_Offset = %d", CephParam.CEPH_2ND_Colli_Offset);
			UART_SendMessage(DBG_MSG_PC, str);			
		}
		else if (strstr(message.Data, UART_ALIGN_SCAN_2COL_SPEED_UP))
		{
			char str[40] = {0,};			
			CephParam.CEPH_2ND_Colli_Offset++;
			
			sprintf(str, "n2ndCol_Speed_Offset = %d", CephParam.CEPH_2ND_Colli_Offset);
			UART_SendMessage(DBG_MSG_PC, str);
		}
		else if (strstr(message.Data, UART_ALIGN_SCAN_2COL_SPEED_DN))
		{
			char str[40] = {0,};			
			CephParam.CEPH_2ND_Colli_Offset--;
			
			sprintf(str, "n2ndCol_Speed_Offset = %d", CephParam.CEPH_2ND_Colli_Offset);
			UART_SendMessage(DBG_MSG_PC, str);			
		}
		else if (strstr(message.Data, UART_ALIGN_SCAN_2COL_SPEED_SET))
		{
			char str[40] = {0,};			
			int16_t nOffset = 0;

			nOffset= CephParam.CEPH_2ND_Colli_Offset;
#if defined(USE_I2C_EEPROM)
			EEPRom_I2C_Write_Word(ROM_CEPH_2ND_COL_SPEED_ADDR, (uint16_t)nOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());

			sprintf(str, "CCAXIS_ALIGN_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_COL_SPEED_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_SPEED_ADDR));
			UART_SendMessage(DBG_MSG_PC, str);
#endif /* USE_I2C_EEPROM */
			
			sprintf(str, "n2ndCol_Speed_Offset = %d", CephParam.CEPH_2ND_Colli_Offset);
			UART_SendMessage(DBG_MSG_PC, str);
		}
		else if (strstr(message.Data, UART_ALIGN_SCAN_2COL_SPEED_RESET))
		{
			char str[40] = {0,};			
			int16_t nOffset = 0;
			CephParam.CEPH_2ND_Colli_Offset=nOffset;
#if defined(USE_I2C_EEPROM)
			EEPRom_I2C_Write_Word(ROM_CEPH_2ND_COL_SPEED_ADDR, (uint16_t)nOffset);
			IntTimer_Delay(1000);
			while(!IntTimer_GetStatus());

			sprintf(str, "CCAXIS_ALIGN_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_COL_SPEED_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_SPEED_ADDR));
			UART_SendMessage(DBG_MSG_PC, str);
#endif /* USE_I2C_EEPROM */
			
			sprintf(str, "n2ndCol_Speed_Offset = %d", CephParam.CEPH_2ND_Colli_Offset);
			UART_SendMessage(DBG_MSG_PC, str);
		}
		else if(strstr(message.Data, UART_ALIGN_SCAN_2COL_SPEED_READ))
		{
#if defined(USE_I2C_EEPROM)
			char str[40] = {0,};
			int16_t nOffset = 0;
			
			nOffset = EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_SPEED_ADDR);
			
			sprintf(str, nOffset ? "ROM_CEPH_2ND_COL_SPEED_ADDR(0x%02x) : [+0x%04x]": "ROM_CEPH_2ND_COL_SPEED_ADDR(0x%02x) : [0x%04x]", 
																		ROM_CEPH_2ND_COL_SPEED_ADDR, nOffset);
			UART_SendMessage(DBG_MSG_PC, str);
#endif /* USE_I2C_EEPROM */

			sprintf(str, nOffset ? "n2ndColOffset = %+d" : "n2ndColOffset = %d", nOffset);
			UART_SendMessage(DBG_MSG_PC, str);
		}
#endif

        else if (strstr(message.Data, UART_CEPH_ALIGN_XRAY_SHOT_ON))
        {
            UART_SendMessage(DBG_MSG_PC, "Enter into Communication with Tube");

            if (!CAN_SendMessage(CAN_TUBE_COM_CHECK, CAN_TUBE_GEN2_COM, 0, 1000, 2))
            {
                return RESET;
            }
            
            UART_SendMessage(DBG_MSG_PC, "Tube's Communication is checked");				

            if (!CAN_SendMessage(CAN_TUBE_CONTI_MODE_SET, 0, 0, 1000, 2))
            {
                return RESET;
            }

            if (!CAN_SendMessage(CAN_TUBE_KV_SET, CalibrationModeParam.KV, 0, 1000, 2))
            {
                return RESET;
            }

            if (!CAN_SendMessage(CAN_TUBE_mA_SET, CalibrationModeParam.mA, 0, 1000, 2))
            {
                return RESET;
            }
            
            Tube_CtrlReady(TUBE_READY_ENABLE);

            UART_SendMessage(DBG_MSG_PC, "Wait for Tube Heatting Time");	

            IntTimer_Delay(2000);
            while(!IntTimer_GetStatus());

            UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_START]");

            Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
            
            IntTimer_Delay(100);
            while(!IntTimer_GetStatus());

            ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_ENABLE);      
        }
        else if (strstr(message.Data, UART_CEPH_ALIGN_XRAY_SHOT_OFF))
        {
            ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE);

            Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
            Tube_CtrlReady(TUBE_READY_DISABLE);

            Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);

            UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_END__]");
        }
        else if (strstr(message.Data, UART_CEPH_2ND_COLI_MOVE_STEP_LEFT))
        {
        	TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
            Motor_CephMoveAbsolutePosition(&Motor_S, 100, 50, Motor_S.CurrentStep - ALIGN_2ND_COLI_MOVE_STEP, 50);
    	    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
        }
        else if (strstr(message.Data, UART_CEPH_2ND_COLI_MOVE_STEP_RIGHT))
        {
        	TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
            Motor_CephMoveAbsolutePosition(&Motor_S, 100, 50, Motor_S.CurrentStep + ALIGN_2ND_COLI_MOVE_STEP, 50);
    	    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
        }
        else if (strstr(message.Data, UART_CEPH_2ND_COLI_MOVE_DETAIL_STEP_LEFT))
        {
        	TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
            Motor_CephMoveAbsolutePosition(&Motor_S, 100, 50, Motor_S.CurrentStep - ALIGN_2ND_COLI_MOVE_DETAIL_STEP, 50);
    	    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
        }
        else if (strstr(message.Data, UART_CEPH_2ND_COLI_MOVE_DETAIL_STEP_RIGHT))
        {
        	TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
            Motor_CephMoveAbsolutePosition(&Motor_S, 100, 50, Motor_S.CurrentStep + ALIGN_2ND_COLI_MOVE_DETAIL_STEP, 50);
    	    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
        }
		else if (strstr(message.Data, UART_CEPH_2ND_COLI_MOVE_ABSOLUTE_STEP_LEFT))
        {
        	char str[40] = {0,};
            int16_t nOffset = 0;

			nOffset = atoi(&(message.Data[10]));

            sprintf(str, "Absolute Step = %d", nOffset);
            UART_SendMessage(DBG_MSG_PC, str);
        	TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
            Motor_CephMoveAbsolutePosition(&Motor_S, 100, 50, Motor_S.CurrentStep - nOffset, 50);
    	    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
        }
		else if (strstr(message.Data, UART_CEPH_2ND_COLI_MOVE_ABSOLUTE_STEP_RIGHT))
        {
        	char str[40] = {0,};
            int16_t nOffset = 0;

			nOffset = atoi(&(message.Data[10]));

            sprintf(str, "Absolute Step = %d", nOffset);
            UART_SendMessage(DBG_MSG_PC, str);
        	TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
            Motor_CephMoveAbsolutePosition(&Motor_S, 100, 50, Motor_S.CurrentStep + nOffset, 50);
    	    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
        }
        else if (strstr(message.Data, UART_CEPH_2ND_COLI_STEP_SET))
        {
            char str[40] = {0,};            
            int16_t nOffset = 0;

			if (b2ndColStart == SET)
			{
	            nOffset = Motor_S.CurrentStep - CEPH_2ND_COLI_ALIGN_POS;
			} 
			else 
			{
	            nOffset = Motor_S.CurrentStep - ALIGN_2ND_COLI_CENTER_POSISTION;
			}

            sprintf(str, "nOffset = %d", nOffset);
            UART_SendMessage(DBG_MSG_PC, str);

#ifdef USE_I2C_EEPROM
            EEPRom_I2C_Write_Word(ROM_CEPH_2ND_COL_START_ADDR, (uint16_t)nOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            //jehun - 20200724
//            sprintf(str, "ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_COL_START_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_START_ADDR));
//            UART_SendMessage(DBG_MSG_PC, str);
            printUart(DBG_MSG_PC, "ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_COL_START_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_START_ADDR));


#endif /* USE_I2C_EEPROM */

            CephParam.n2ndColOffset = nOffset;
            
            sprintf(str, "n2ndColOffset = %d", CephParam.n2ndColOffset);
            UART_SendMessage(DBG_MSG_PC, str);

			CephParam.n2ndColFastStartOffset= (int16_t)EEPRom_I2C_Read_Word(ROM_CEPH_FAST_2ND_COL_START_ADDR);
		    if (CephParam.n2ndColFastStartOffset == (int16_t)0xffff)
		    {
		        CephParam.n2ndColFastStartOffset = CephParam.n2ndColOffset;
		    }
			else if(CephParam.n2ndColFastStartOffset<0) CephParam.n2ndColFastStartOffset++;
        }        
        else if (strstr(message.Data, UART_CEPH_2ND_COLI_STEP_RESET))
        {
            char str[40] = {0,};
            int16_t nOffset = 0;
            
#ifdef USE_I2C_EEPROM
            EEPRom_EraseAddrWord(ROM_CEPH_2ND_COL_START_ADDR);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            //jehun - 20200724
//            sprintf(str, "ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_COL_START_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_START_ADDR));
//            UART_SendMessage(DBG_MSG_PC, str);
            printUart(DBG_MSG_PC, "ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_COL_START_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_START_ADDR));
#endif /* USE_I2C_EEPROM */

            CephParam.n2ndColOffset = nOffset;

            sprintf(str, "CephParam.n2ndColOffset = %d", CephParam.n2ndColOffset);
            UART_SendMessage(DBG_MSG_PC, str);
        }
		else if (strstr(message.Data, UART_CEPH_FAST_2ND_COLI_STEP_SET))
        {
            char str[40] = {0,};            
            int16_t nOffset = 0;

			if (b2ndColStart == SET)
			{
	            nOffset = Motor_S.CurrentStep - CEPH_2ND_COLI_ALIGN_POS;
			} 
			else 
			{
	            nOffset = Motor_S.CurrentStep - ALIGN_2ND_COLI_CENTER_POSISTION;
			}

            sprintf(str, "nOffset = %d", nOffset);
            UART_SendMessage(DBG_MSG_PC, str);

#ifdef USE_I2C_EEPROM
            EEPRom_I2C_Write_Word(ROM_CEPH_FAST_2ND_COL_START_ADDR, (uint16_t)nOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            //jehun - 20200724
//            sprintf(str, "ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_COL_START_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_START_ADDR));
//            UART_SendMessage(DBG_MSG_PC, str);
            printUart(DBG_MSG_PC, "ROM_CEPH_FAST_2ND_COL_START_ADDR(0x%02x) : 0x%04x", ROM_CEPH_FAST_2ND_COL_START_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_FAST_2ND_COL_START_ADDR));


#endif /* USE_I2C_EEPROM */

            CephParam.n2ndColFastStartOffset = nOffset;
            
            sprintf(str, "n2ndColOffset = %d", CephParam.n2ndColFastStartOffset);
            UART_SendMessage(DBG_MSG_PC, str);
        }        
        else if (strstr(message.Data, UART_CEPH_FAST_2ND_COLI_STEP_RESET))
        {
            char str[40] = {0,};
            int16_t nOffset = 0;
            
#ifdef USE_I2C_EEPROM
            EEPRom_EraseAddrWord(ROM_CEPH_FAST_2ND_COL_START_ADDR);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            //jehun - 20200724
//            sprintf(str, "ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_COL_START_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_START_ADDR));
//            UART_SendMessage(DBG_MSG_PC, str);
            printUart(DBG_MSG_PC, "ROM_CEPH_FAST_2ND_COL_START_ADDR(0x%02x) : 0x%04x", ROM_CEPH_FAST_2ND_COL_START_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_FAST_2ND_COL_START_ADDR));
#endif /* USE_I2C_EEPROM */

            CephParam.n2ndColFastStartOffset = CephParam.n2ndColOffset;

            sprintf(str, "CephParam.n2ndColFastStartOffset = %d", CephParam.n2ndColFastStartOffset);
            UART_SendMessage(DBG_MSG_PC, str);
        }
		else if (strstr(message.Data, UART_CEPH_2ND_COLI_ALIGN_DEC_OFFSET))
		{
            char str[40] = {0,};
            int16_t nOffset = 0;

			nOffset -= atoi(&(message.Data[9]));

            sprintf(str, "nOffset = %d", nOffset);
            UART_SendMessage(DBG_MSG_PC, str);

#ifdef USE_I2C_EEPROM
            EEPRom_I2C_Write_Word(ROM_CEPH_2ND_COL_START_ADDR, (uint16_t)nOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            //jehun - 20200724
//            sprintf(str, "ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_COL_START_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_START_ADDR));
//            UART_SendMessage(DBG_MSG_PC, str);
            printUart(DBG_MSG_PC,"ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_COL_START_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_START_ADDR));
#endif /* USE_I2C_EEPROM */

            CephParam.n2ndColOffset = nOffset;
            
            sprintf(str, "n2ndColOffset = %d", CephParam.n2ndColOffset);
            UART_SendMessage(DBG_MSG_PC, str);
		}
		else if (strstr(message.Data, UART_CEPH_2ND_COLI_ALIGN_INC_OFFSET))
		{
            char str[40] = {0,};
            int16_t nOffset = 0;

			nOffset += atoi(&(message.Data[9]));

            sprintf(str, "nOffset = +%d", nOffset);
            UART_SendMessage(DBG_MSG_PC, str);

#ifdef USE_I2C_EEPROM
            EEPRom_I2C_Write_Word(ROM_CEPH_2ND_COL_START_ADDR, (uint16_t)nOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            //jehun - 20200724
//            sprintf(str, "ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_COL_START_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_START_ADDR));
//            UART_SendMessage(DBG_MSG_PC, str);
            printUart(DBG_MSG_PC, "ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_COL_START_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_START_ADDR));
#endif /* USE_I2C_EEPROM */

            CephParam.n2ndColOffset = nOffset;
            
            sprintf(str, "n2ndColOffset = +%d", CephParam.n2ndColOffset);
            UART_SendMessage(DBG_MSG_PC, str);
		}
		else if (strstr(message.Data, UART_GET_CEPH_2ND_COLI_ALIGN_OFFSET))
		{
#ifdef USE_I2C_EEPROM
			char str[40] = {0,};
			int16_t nOffset = 0;
			
			nOffset = EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_START_ADDR);
			if(nOffset==0xffff)
			{
				nOffset=0;
			}
			else if(nOffset<0) nOffset++;
			//jehun - 20200724
//			sprintf(str, nOffset ? "ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : [+0x%04x]": "ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : [0x%04x]",
//																		ROM_CEPH_2ND_COL_START_ADDR, nOffset);
//			UART_SendMessage(DBG_MSG_PC, str);
			if(nOffset == 1){
				printUart(DBG_MSG_PC,"ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : [+0x%04x]", ROM_CEPH_2ND_COL_START_ADDR, nOffset);
			}
			else{
				printUart(DBG_MSG_PC,"ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : [0x%04x]", ROM_CEPH_2ND_COL_START_ADDR, nOffset);
			}


			sprintf(str, nOffset ? "n2ndColOffset = %+d" : "n2ndColOffset = %d", nOffset);
			UART_SendMessage(DBG_MSG_PC, str);
#endif /* USE_I2C_EEPROM */
		}
		else if (strstr(message.Data, UART_GET_CEPH_FAST_2ND_COLI_ALIGN_OFFSET))
		{
#ifdef USE_I2C_EEPROM
			char str[40] = {0,};
			int16_t nOffset = 0;
			
			nOffset = EEPRom_I2C_Read_Word(ROM_CEPH_FAST_2ND_COL_START_ADDR);
			if(nOffset==0xffff)
			{
				nOffset=0;
			}
			else if(nOffset<0) nOffset++;
			//jehun - 20200724
//			sprintf(str, nOffset ? "ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : [+0x%04x]": "ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : [0x%04x]",
//																		ROM_CEPH_2ND_COL_START_ADDR, nOffset);
//			UART_SendMessage(DBG_MSG_PC, str);
			if(nOffset == 1){
				printUart(DBG_MSG_PC,"ROM_CEPH_FAST_2ND_COL_START_ADDR(0x%02x) : [+0x%04x]", ROM_CEPH_FAST_2ND_COL_START_ADDR, nOffset);
			}
			else{
				printUart(DBG_MSG_PC,"ROM_CEPH_FAST_2ND_COL_START_ADDR(0x%02x) : [0x%04x]", ROM_CEPH_FAST_2ND_COL_START_ADDR, nOffset);
			}


			sprintf(str, nOffset ? "n2ndColOffset = %+d" : "n2ndColOffset = %d", nOffset);
			UART_SendMessage(DBG_MSG_PC, str);
#endif /* USE_I2C_EEPROM */
		}
		else if (strstr(message.Data, UART_CEPH_2ND_COLI_END_STEP_SET))
        {
            char str[40] = {0,};            
            int32_t nOffset = 0;

			nOffset = Motor_S.CurrentStep - (300 * (MOTOR_C_MICROSTEP / (MOTOR_C_PULLEY_DIAMETER * pi))-CEPH_2ND_COLI_ALIGN_POS);

            sprintf(str, "nOffset = %d", nOffset);
            UART_SendMessage(DBG_MSG_PC, str);

#if defined(USE_I2C_EEPROM)
            EEPRom_I2C_Write_Word(ROM_CEPH_2ND_COL_END_ADDR, (uint16_t)nOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            //jehun - 20200724
//            sprintf(str, "ROM_CEPH_2ND_COL_END_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_COL_END_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_END_ADDR));
//            UART_SendMessage(DBG_MSG_PC, str);
            printUart(DBG_MSG_PC,"ROM_CEPH_2ND_COL_END_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_COL_END_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_END_ADDR));

#endif /* USE_I2C_EEPROM */

            CephParam.n2ndColEndOffset = nOffset;
            
            sprintf(str, "n2ndColEndOffset = %d", CephParam.n2ndColEndOffset);
            UART_SendMessage(DBG_MSG_PC, str);
			
        }        
        else if (strstr(message.Data, UART_CEPH_2ND_COLI_END_STEP_RESET))
        {
            char str[40] = {0,};
            int16_t nOffset = 0;
       
#if defined(USE_I2C_EEPROM)
            EEPRom_EraseAddrWord(ROM_CEPH_2ND_COL_END_ADDR);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            //jehun - 20200724
//            sprintf(str, "ROM_CEPH_2ND_COL_END_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_COL_END_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_END_ADDR));
//            UART_SendMessage(DBG_MSG_PC, str);
            printUart(DBG_MSG_PC,"ROM_CEPH_2ND_COL_END_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_COL_END_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_END_ADDR));

#endif /* USE_I2C_EEPROM */

            CephParam.n2ndColEndOffset = nOffset;

            sprintf(str, "CephParam.n2ndColEndOffset = %d", CephParam.n2ndColEndOffset);
            UART_SendMessage(DBG_MSG_PC, str);
        }		
		else if(strstr(message.Data, UART_CEPH_SPEED_STEP_RESET))
		{
			char str[40] = {0,};
			int16_t nOffset = 0;
			CephParam.n2ndSpeedStepOffset= nOffset;
			
#if defined(USE_I2C_EEPROM)
            EEPRom_EraseAddrWord(ROM_CEPH_2ND_ADD_STEP_ADDR);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            //jehun - 20200724
//            sprintf(str, "ROM_CEPH_2ND_ADD_STEP_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_ADD_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_ADD_STEP_ADDR));
//            UART_SendMessage(DBG_MSG_PC, str);
            printUart(DBG_MSG_PC, "ROM_CEPH_2ND_ADD_STEP_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_ADD_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_ADD_STEP_ADDR));

#endif /* USE_I2C_EEPROM */

			sprintf(str, "RESET n2ndAdditionStepOffset = %d", CephParam.n2ndSpeedStepOffset);
            UART_SendMessage(DBG_MSG_PC, str);		
		}
		else if(strstr(message.Data, UART_CEPH_SPEED_STEP_OFFSET))
		{
			char str[40] = {0,};
            int16_t nOffset = 0;

			if (strchr(&(message.Data[9]), 'M'))
			{
				nOffset -= atoi(&(message.Data[10]));
			}
			else
			{
				nOffset += atoi(&(message.Data[10]));
			}
			
			
            sprintf(str, "Addition_Step_nOffset = %d", nOffset);
            UART_SendMessage(DBG_MSG_PC, str);

			CephParam.n2ndSpeedStepOffset+=nOffset;

#if defined(USE_I2C_EEPROM)
            EEPRom_I2C_Write_Word(ROM_CEPH_2ND_ADD_STEP_ADDR, (uint16_t)CephParam.n2ndSpeedStepOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            //jehun - 20200724
//            sprintf(str, "ROM_CEPH_2ND_ADD_STEP_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_ADD_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_ADD_STEP_ADDR));
//            UART_SendMessage(DBG_MSG_PC, str);
            printUart(DBG_MSG_PC, "ROM_CEPH_2ND_ADD_STEP_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_ADD_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_ADD_STEP_ADDR));


			CephParam.n2ndFastSpeedStepOffset= (int16_t)EEPRom_I2C_Read_Word(ROM_CEPH_2ND_FAST_ADD_STEP_ADDR);
	        if (CephParam.n2ndFastSpeedStepOffset == (int16_t)0xffff)
	        {
	            CephParam.n2ndFastSpeedStepOffset = CephParam.n2ndSpeedStepOffset;
	        }
			else if(CephParam.n2ndFastSpeedStepOffset<0) CephParam.n2ndFastSpeedStepOffset++;
#endif /* USE_I2C_EEPROM */
            
            sprintf(str, "n2ndAdditionStepOffset = %d", CephParam.n2ndSpeedStepOffset);
            UART_SendMessage(DBG_MSG_PC, str);			
		}	
		else if(strstr(message.Data, UART_CEPH_FAST_SPEED_STEP_RESET))
		{
			char str[40] = {0,};
			int16_t nOffset = 0;
			CephParam.n2ndFastSpeedStepOffset= nOffset;
			
#if defined(USE_I2C_EEPROM)
            EEPRom_EraseAddrWord(ROM_CEPH_2ND_FAST_ADD_STEP_ADDR);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            //jehun - 20200724
//            sprintf(str, "ROM_CEPH_2ND_FAST_ADD_STEP_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_FAST_ADD_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_FAST_ADD_STEP_ADDR));
//            UART_SendMessage(DBG_MSG_PC, str);
            printUart(DBG_MSG_PC, "ROM_CEPH_2ND_FAST_ADD_STEP_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_FAST_ADD_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_FAST_ADD_STEP_ADDR));
#endif /* USE_I2C_EEPROM */

			sprintf(str, "RESET n2ndFastAdditionStepOffset = %d", CephParam.n2ndFastSpeedStepOffset);
            UART_SendMessage(DBG_MSG_PC, str);		
		}
		else if(strstr(message.Data, UART_CEPH_FAST_SPEED_STEP_OFFSET))
		{
			char str[40] = {0,};
            int16_t nOffset = 0;

			if (strchr(&(message.Data[9]), 'M'))
			{
				nOffset -= atoi(&(message.Data[10]));
			}
			else
			{
				nOffset += atoi(&(message.Data[10]));
			}
			
			
            sprintf(str, "Fast_Addition_Step_nOffset = %d", nOffset);
            UART_SendMessage(DBG_MSG_PC, str);


            //jehun - debug
            // offset ���� ���� RUN CCR �� �˱� ���� �߰�
//            double n2nd_speed2= ((CephParam.n2ndColEndOffset+(300 * (3200 / (11 * pi))-3732.675))-(CephParam.n2ndColOffset+3732.675))/(CephParam.Time/1000) + CephParam.n2ndFastSpeedStepOffset*12;
            double n2nd_speed2= ((CephParam.n2ndColEndOffset+(300 * (3200 / (11 * pi))-3732.675))-(CephParam.n2ndColFastStartOffset+3732.675)+CephParam.n2ndFastSpeedStepOffset)/(CephParam.Time/1000);
            n2nd_speed2 = 1250000/(2*n2nd_speed2);
            printUart(DBG_MSG_PC, "RUNCCR = %lf", n2nd_speed2);

			CephParam.n2ndFastSpeedStepOffset+=nOffset;

#if defined(USE_I2C_EEPROM)
            EEPRom_I2C_Write_Word(ROM_CEPH_2ND_FAST_ADD_STEP_ADDR, (uint16_t)CephParam.n2ndFastSpeedStepOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            //jehun - 20200724
//            sprintf(str, "ROM_CEPH_2ND_FAST_ADD_STEP_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_FAST_ADD_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_FAST_ADD_STEP_ADDR));
//            UART_SendMessage(DBG_MSG_PC, str);
            printUart(DBG_MSG_PC, "ROM_CEPH_2ND_FAST_ADD_STEP_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_FAST_ADD_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_FAST_ADD_STEP_ADDR));
#endif /* USE_I2C_EEPROM */
            
            sprintf(str, "n2ndFastAdditionStepOffset = %d", CephParam.n2ndFastSpeedStepOffset);
            UART_SendMessage(DBG_MSG_PC, str);			
		}	
        else if (strstr(message.Data, UART_MANUAL_XRAY_SHOT))
        {
            CalibrationModeParam.nExpTime = atoi(&(message.Data[9]));

            if ((CalibrationModeParam.nExpTime > 0) && (CalibrationModeParam.nExpTime <= MAX_XRAY_SHOT_TIME))
            {
                UART_SendMessage(DBG_MSG_PC, "Enter into Communication with Tube");

                if (!CAN_SendMessage(CAN_TUBE_COM_CHECK, CAN_TUBE_GEN2_COM, 0, 1000, 2))
                {
                    return RESET;
                }
                
                UART_SendMessage(DBG_MSG_PC, "Tube's Communication is checked");				

                if (!CAN_SendMessage(CAN_TUBE_CONTI_MODE_SET, 0, 0, 1000, 2))
                {
                    return RESET;
                }

                if (!CAN_SendMessage(CAN_TUBE_KV_SET, CalibrationModeParam.KV, 0, 1000, 2))
                {
                    return RESET;
                }

                if (!CAN_SendMessage(CAN_TUBE_mA_SET, CalibrationModeParam.mA, 0, 1000, 2))
                {
                    return RESET;
                }

				//200303 ADD

				Exposure_CtrlLed(EXPOSURE_LED_GPIO_ENABLE);

				UART_SendMessage(DBG_MSG_PC, "[SP_STBY_EXPSW]");
				
				while((Exposure_CheckSwitch()))
	            {
	                if (CurCaptureMode == CAPTURE_CANCEL)
	                {
	                	Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
                		Tube_CtrlReady(TUBE_READY_DISABLE);

                		Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
						UART_SendMessage(DBG_MSG_PC, "SHOT READY CANCELED");
	                    return SET;
	                }

					if(UART_Exposure_SET()==2)
					{
						Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
	            		Tube_CtrlReady(TUBE_READY_DISABLE);

	            		Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
						UART_SendMessage(DBG_MSG_PC, "SHOT READY CANCELED");
						return RESET;
					}
	            }	
                UART_SendMessage(DBG_MSG_PC, "MANUAL_XRAY_SHOT Start");	
                
                Tube_CtrlReady(TUBE_READY_ENABLE);

                UART_SendMessage(DBG_MSG_PC, "Wait for Tube Heatting Time");	

                IntTimer_Delay(2000);
                while(!IntTimer_GetStatus());

                UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_START]");

                Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);

                IntTimer_Delay(CalibrationModeParam.nExpTime);
                //while(!IntTimer_GetStatus());

				//200303 ADD
				while(!IntTimer_GetStatus())
				{
					if(Exposure_CheckSwitch())
					{
						UART_SendMessage(DBG_MSG_PC, "SHOT STOPED");
						break;
					}
				}
				Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);

				Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
                Tube_CtrlReady(TUBE_READY_DISABLE);

                UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_END__]");
                
                UART_SendMessage(DBG_MSG_PC, "MANUAL_XRAY_SHOT End");
				if (!CAN_SendMessage(CAN_TUBE_COUNT_CHECK, 0, 0, 1000, 2))
				{
	    			return RESET;
				}
#if defined(GEN_TEMP_CHECK)
				CAN_SendMessage(CAN_TUBE_TANK_TEMP_READ, 0, 0, 1000, 2);
#endif /* GEN_TEMP_CHECK */
            }
            else
            {
                UART_SendMessage(DBG_MSG_PC, "[ERR]Invalid Shot Time");
            }
        }
		else if (strstr(message.Data, UART_TUBE_COUNT_RESET))
		{
			if (!CAN_SendMessage(CAN_TUBE_COUNT_RESET, 0, 0, 1000, 2))
			{
    			return RESET;
			}
		}
		else if (strstr(message.Data, UART_TUBE_COUNT_CHECK))
		{	
			if (!CAN_SendMessage(CAN_TUBE_COUNT_CHECK, 0, 0, 1000, 2))
			{
				return RESET;
			}
		}
		else if (strstr(message.Data, UART_CEPH_2ND_AROUND_SHOT))
		{
			char str[40] = {0,};
            int16_t nOffset = 0,nUnder_Offset=0,nUnder_Init=2000;
			bool bUnder_Offset=RESET;

			nOffset = atoi(&(message.Data[10]));

            sprintf(str, "Around Offset = %d", nOffset);
            UART_SendMessage(DBG_MSG_PC, str);

			if (CalibrationModeParam.KV == 0 || CalibrationModeParam.mA == 0)
            {
                char string1[UART_MSG_SIZE * 2 + 3];
                
                memset(string1, NULL, sizeof(char) * (UART_MSG_SIZE * 2 + 3));
                sprintf(string1, "%s : %s", UART_CALIBRATION_MSG_ERROR, message.Data);
                
                UART_SendMessage(DBG_MSG_PC, string1);
            }
			/*else if(Motor_S.CurrentStep - nOffset<0)
			{
				sprintf(str, "%s", UART_CALIBRATION_MOTOR_UNDER_ERROR);
            	UART_SendMessage(DBG_MSG_PC, str);
			}*/
			else if(Motor_S.CurrentStep + nOffset > (300 * (MOTOR_S_MICROSTEP / (MOTOR_S_PULLEY_DIAMETER * pi))))
			{
				sprintf(str, "%s", UART_CALIBRATION_MOTOR_OVER_ERROR);
            	UART_SendMessage(DBG_MSG_PC, str);
			}
            else
            {
            	if (!CAN_Collimator_SendMessage(CMD_FILTER_CENTER, 0, 0, 1000, 2))
                {
                    return RESET;
                }

				if(Motor_S.CurrentStep - nOffset<0)
				{
					sprintf(str, "%s", UART_CALIBRATION_MOTOR_UNDER_ERROR);
            		UART_SendMessage(DBG_MSG_PC, str);
					nUnder_Offset=nOffset;
					nOffset=Motor_S.CurrentStep-nUnder_Init;
					bUnder_Offset=SET;
				}
            	//RunFreq = (Sensor_S.Number * MOTOR_C_MICROSTEP) / (CephParam.Time * msec_To_sec * MOTOR_C_PULLEY_DIAMETER * pi);
            	//RunFreq=(nOffset*MOTOR_C_MICROSTEP)/(10* MOTOR_C_PULLEY_DIAMETER * pi);
            	TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
				Motor_CephMoveAbsolutePosition(&Motor_S, 2000, 1000, Motor_S.CurrentStep - nOffset, 1000);
	    	    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
				TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
				UART_SendMessage(DBG_MSG_PC, "Enter into Communication with Tube");

	            if (!CAN_SendMessage(CAN_TUBE_COM_CHECK, CAN_TUBE_GEN2_COM, 0, 3000, 2))
	            {
	                return RESET;
	            }
	            
	            UART_SendMessage(DBG_MSG_PC, "Tube's Communication is checked");				

	            if (!CAN_SendMessage(CAN_TUBE_CONTI_MODE_SET, 0, 0, 3000, 2))
	            {
	                return RESET;
	            }

	            if (!CAN_SendMessage(CAN_TUBE_KV_SET, CalibrationModeParam.KV, 0, 3000, 2))
	            {
	                return RESET;
	            }

	            if (!CAN_SendMessage(CAN_TUBE_mA_SET, CalibrationModeParam.mA, 0, 3000, 2))
	            {
	                return RESET;
	            }

	            Exposure_CtrlLed(EXPOSURE_LED_GPIO_ENABLE);

				UART_SendMessage(DBG_MSG_PC, "[SP_STBY_EXPSW]");
				
	            while((Exposure_CheckSwitch()))
	            {
	                if (CurCaptureMode == CAPTURE_CANCEL)
	                {
	                	Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
                		Tube_CtrlReady(TUBE_READY_DISABLE);

                		Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
						UART_SendMessage(DBG_MSG_PC, "SHOT READY CANCELED");
	                    return SET;
	                }

					if(UART_Exposure_SET()==2)
					{
						Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
	            		Tube_CtrlReady(TUBE_READY_DISABLE);

	            		Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
						UART_SendMessage(DBG_MSG_PC, "SHOT READY CANCELED");
						return RESET;
					}
	            }				

	            Tube_CtrlReady(TUBE_READY_ENABLE);

	            UART_SendMessage(DBG_MSG_PC, "Wait for Tube Heatting Time");	

	            IntTimer_Delay(2000);
	            while(!IntTimer_GetStatus());
				TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
				if(bUnder_Offset==SET)
					Motor_CephMoveAbsolutePosition(&Motor_S, 400, 50, Motor_S.CurrentStep + nUnder_Offset + nUnder_Init, 50);	//200204 HWAN 100->400
				else
					Motor_CephMoveAbsolutePosition(&Motor_S, 400, 50, Motor_S.CurrentStep + nOffset*2, 50); //200204 HWAN 100->400

				bUnder_Offset=RESET;
	            UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_START]");
	            //UART_SendMessage(DBG_MSG_PC, "[SP_SHOT_START]");

	            Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
	            
#if 1 //[[ myshin_20160310_BEGIN -- �������� Flat ���� �ʾƼ� ������ 
                IntTimer_Delay(100);
                while(!IntTimer_GetStatus());
#endif //]] myshin_20160310_END -- �������� Flat ���� �ʾƼ� ������ 
                UART_SendMessage(DBG_MSG_PC, "SENSOR_S_OUTPUT_ENABLE");
                ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_ENABLE);                
	               
	            while(!Exposure_CheckSwitch())
	        	{
	        		if(UART_Exposure_SET()==2)
					{						
						break;
					}

					if(Motor_GetStatus(&Motor_S, STATUS_STOP))
					{
						UART_SendMessage(DBG_MSG_PC, "Motor_S End Position");
						break;
					}
	        	}
				//while(Motor_S.Update != FALSE);
				Motor_S.Update=FALSE;
			    Motor_Stop(&Motor_S);

			    TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
	            ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE);
	            
	            Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
	            Tube_CtrlReady(TUBE_READY_DISABLE);

				UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_STOP_]");
				
	            UART_SendMessage(DBG_MSG_PC, "SENSOR_S_OUTPUT_DISABLE");

	            Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);

				sprintf(str, "Current Offset = %d", Motor_S.CurrentStep);
            	UART_SendMessage(DBG_MSG_PC, str);

	            UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_END__]");

//	            return;
				
            }
		}
        else if (strstr(message.Data, UART_SHOT_READY))
        {
            if (CalibrationModeParam.KV == 0 || CalibrationModeParam.mA == 0)
            {
				printUart(DBG_MSG_PC, "%s : %s", ERR_CODE_UART_CALIB_ERROR, message.Data);
            }
            else
            {
                if (CalibrationModeParam.mode == DET_TYPE_PANORAMA)
                {
                    UART_SendMessage(DBG_MSG_PC, "DET_TYPE_PANORAMA");
                }
                else if (CalibrationModeParam.mode == DET_TYPE_CEPH_SCAN)
                {
                    UART_SendMessage(DBG_MSG_PC, "DET_TYPE_CEPH_SCAN");

                    if (!CAN_Collimator_SendMessage(CMD_FILTER_CENTER, 0, 0, 1000, 2))
                    {
                        return RESET;
                    }
                }
                else if (CalibrationModeParam.mode == DET_TYPE_CEPH_ONESHOT)
                {
                    UART_SendMessage(DBG_MSG_PC, "DET_TYPE_CEPH_ONESHOT");
                }
                else // default mode : DET_TYPE_CT
                {
                    UART_SendMessage(DBG_MSG_PC, "DET_TYPE_CT");
                }            
                if (CalibrationModeParam.mode != DET_TYPE_CEPH_SCAN)
                {
                    if ((CalibrationModeParam.mode == DET_TYPE_PANORAMA) && (CalibrationModeParam.AlignType == COL_TYPE_PANO))
                    {
                        if (!CAN_Collimator_SendMessage(CMD_COLLI_PANOR, 0, 0, 2000, 2))
                        {
                            return RESET;
                        }
                    }
                    else if ((CalibrationModeParam.mode == DET_TYPE_CT) && (CalibrationModeParam.AlignType == COL_TYPE_CT))
                    {
                        if (!CAN_Collimator_SendMessage(CalibrationModeParam.nFovCmd, 0, 0, 2000, 2))
                        {
                            return RESET;
                        }
                    }
                    else
                    {
                        if (CalibrationModeParam.AlignType == COL_TYPE_NONE)
                        {
                            if (bClose != SET)
                            {
                                if (!CAN_Collimator_SendMessage(CMD_COLLI_OPEN, 0, 0, 2000, 2))
                                {
                                    return RESET;
                                }
                            }
                        }
                    }
                    IntTimer_Delay(200);
                    while(!IntTimer_GetStatus());

                    if ((CalibrationModeParam.mode == DET_TYPE_CT) && (CalibrationModeParam.AlignType == COL_TYPE_NONE))
                    {
                        if (!CAN_Collimator_SendMessage(CMD_FILTER_CT_CALIB, 0, 0, 1000, 2))
                        {
                            return RESET;
                        }
                    }
                    else
                    {
                        if (!CAN_Collimator_SendMessage(CMD_FILTER_CENTER, 0, 0, 1000, 2))
                        {
                            return RESET;
                        }
                    }
                    IntTimer_Delay(200);
                    while(!IntTimer_GetStatus());                
                }
                //UART_SendMessage(DBG_MSG_PC, "Enter into Communication with Tube");

                if (!CAN_SendMessage(CAN_TUBE_COM_CHECK, CAN_TUBE_GEN2_COM, 0, 3000, 2))
                {
                	UART_SendMessage(DBG_MSG_PC, "Tube's Communication is error");	
                    return RESET;
                }

				gMachStat.bTubeCountFlag=RESET;
				if (!CAN_SendMessage(CAN_TUBE_COUNT_CHECK, 0, 0, 1000, 2))
			    {
			    	UART_SendMessage(DBG_MSG_PC, "Tube Count Fail.");
			        CurCaptureMode = CAPTURE_CANCEL;
			        
			        return RESET;
			    }
                //UART_SendMessage(DBG_MSG_PC, "Tube's Communication is checked");				

                if (!CAN_SendMessage(CAN_TUBE_CONTI_MODE_SET, 0, 0, 3000, 2))
                {
                    return RESET;
                }

                if (!CAN_SendMessage(CAN_TUBE_KV_SET, CalibrationModeParam.KV, 0, 3000, 2))
                {
                    return RESET;
                }

                if (!CAN_SendMessage(CAN_TUBE_mA_SET, CalibrationModeParam.mA, 0, 3000, 2))
                {
                    return RESET;
                }

                if ((CalibrationModeParam.mode == DET_TYPE_CT) || (CalibrationModeParam.mode == DET_TYPE_PANORAMA))
                {
                    //while(PanoSensor_CheckInput())
                    {
                        if (CurCaptureMode == CAPTURE_CANCEL)
                        {
                            return SET;
                        }
                    }		
                }

                Exposure_CtrlLed(EXPOSURE_LED_GPIO_ENABLE);

				UART_SendMessage(DBG_MSG_PC, "[SP_STBY_EXPSW]");
				
                while((Exposure_CheckSwitch()))
                {
                    if (CurCaptureMode == CAPTURE_CANCEL)
                    {
                    	Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
                		Tube_CtrlReady(TUBE_READY_DISABLE);

                		Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
						UART_SendMessage(DBG_MSG_PC, "SHOT READY CANCELED");
                        return SET;
                    }

					if(UART_Exposure_SET()==2)
					{
						Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
                		Tube_CtrlReady(TUBE_READY_DISABLE);

                		Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
						UART_SendMessage(DBG_MSG_PC, "SHOT READY CANCELED");
						return RESET;
					}
                }				

                Tube_CtrlReady(TUBE_READY_ENABLE);

                UART_SendMessage(DBG_MSG_PC, "Wait for Tube Heatting Time");	

                IntTimer_Delay(2000);
                while(!IntTimer_GetStatus());

                UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_START]");

                Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);

                if (CalibrationModeParam.mode == DET_TYPE_CEPH_SCAN)
                {
                    IntTimer_Delay(100);
                    while(!IntTimer_GetStatus());

                    UART_SendMessage(DBG_MSG_PC, "SENSOR_S_OUTPUT_ENABLE");
                    ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_ENABLE);                
                    //ScanSensor_Start();
                }

                while(!Exposure_CheckSwitch())
				{
            		if(UART_Exposure_SET()==2)
					{
						break;
					}
            	}

                if (CalibrationModeParam.mode == DET_TYPE_CEPH_SCAN)
                {
                    //ScanSensor_Stop();

                    UART_SendMessage(DBG_MSG_PC, "SENSOR_S_OUTPUT_DISABLE");
                    ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE);
                }

                Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
                Tube_CtrlReady(TUBE_READY_DISABLE);

                Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);

				gMachStat.bTubeCountFlag=SET;
				if (!CAN_SendMessage(CAN_TUBE_COUNT_CHECK, 0, 0, 1000, 2))
				{
					UART_SendMessage(DBG_MSG_PC, "Tube Count Fail.");
					CurCaptureMode = CAPTURE_CANCEL;
					
					return;
				}
				if(gMachStat.nTubeOldCount==gMachStat.nTubeCount)
				{
					UART_SendMessage(DBG_MSG_PC, "Warning : Generator Doesn't Exposed");
				}
                UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_END__]");
                //UART_SendMessage(DBG_MSG_PC, "[SP_SHOT_END__]");
                bClose = RESET;
            }
        }
		else if (strstr(message.Data, UART_CEPH_ALIGN_SCAN_TEST))
		{
		    double dPulse = 0;
		    uint32_t nStartCnt = 0;
		    uint32_t nEndCnt = 0;
			BitAction check;
		    //bool bStop = RESET;
			
            UART_SendMessage(DBG_MSG_PC, "UART_CEPH_ALIGN_SCAN_TEST Start");
			TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StartCurrent);
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);

			Motor_RunOrgCheck(&Motor_C);
			Motor_RunOrgCheck(&Motor_S);
			
    	    while(!Motor_GetStatus(&Motor_C, STATUS_STOP));
    	    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
			
            Motor_CephMoveAbsolutePosition(&Motor_C, 2000, 1000, CEPH_SCAN_ALIGN_POSITION * (MOTOR_C_MICROSTEP / (MOTOR_C_PULLEY_DIAMETER * pi)), 1000);

			if(Tube.bXrayFrameOnOff==RESET)
			{
				if(CalibrationModeParam.nScanTime==5)
					Motor_CephMoveAbsolutePosition(&Motor_S, 2000, 1000, CEPH_2ND_COLI_START_POS + CephParam.n2ndColFastStartOffset, 1000);
				else
					Motor_CephMoveAbsolutePosition(&Motor_S, 2000, 1000, CEPH_2ND_COLI_START_POS + CephParam.n2ndColOffset, 1000);
			}
			else Motor_CephMoveAbsolutePosition(&Motor_S, 2000, 1000, 259.69 * (MOTOR_S_MICROSTEP / (MOTOR_S_PULLEY_DIAMETER * pi)), 1000);

			while(!Motor_GetStatus(&Motor_C, STATUS_STOP));
		    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));

			TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StopCurrent);
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);

			UART_SendMessage(DBG_MSG_PC, "CEPH_ALIGN_SCAN_TEST Position Ready");

			if(bOpen == RESET)
			{
				if (!CAN_Collimator_SendMessage(CMD_COLLI_CEPHSCAN_INIT, 0, 0, 1000, 2))
			    {
		            UART_SendMessage(DBG_MSG_PC, "CMD_COLLI_CEPHSCAN_INIT Error");				
		            return RESET;
			    }

			    if (!CAN_Collimator_SendMessage(CMD_FILTER_CENTER, 0, 0, 1000, 2))
			    {
		            UART_SendMessage(DBG_MSG_PC, "CMD_FILTER_CENTER Error");				
		            return RESET;
			    } 
				
			}
		    

		    //Sensor_S.TriggerFreq = 300 / (10000 * msec_To_sec);
		    Motor_C.RunParam.RunFreq = (300 * MOTOR_C_MICROSTEP) / (CalibrationModeParam.nScanTime * MOTOR_C_PULLEY_DIAMETER * pi);
			CephParam.Time = CalibrationModeParam.nScanTime*1000;
		    dPulse = MOTOR_C_PULLEY_DIAMETER * pi / MOTOR_C_MICROSTEP; //mm_per_pulse

		    nStartCnt = 10;
			nEndCnt = 300 / dPulse;

		    if (!CAN_SendMessage(CAN_TUBE_CONTI_MODE_SET, 0, 0, 1000, 2))
		    {
	            UART_SendMessage(DBG_MSG_PC, "CAN_TUBE_CONTI_MODE_SET Error");				
	            return RESET;
		    }

			if (CalibrationModeParam.KV == 0)
			{
				CalibrationModeParam.KV = 80;
			}

			if (CalibrationModeParam.mA == 0)
			{
				CalibrationModeParam.mA = 100;
			}
			
		    if (!CAN_SendMessage(CAN_TUBE_KV_SET, CalibrationModeParam.KV, 0, 1000, 2))
		    {
	            UART_SendMessage(DBG_MSG_PC, "Error");				
	            return RESET;
		    }

		    if (!CAN_SendMessage(CAN_TUBE_mA_SET, CalibrationModeParam.mA, 0, 1000, 2))
		    {
	            UART_SendMessage(DBG_MSG_PC, "Error");				
	            return RESET;
		    }

		    Exposure_CtrlLed(EXPOSURE_LED_GPIO_ENABLE);

		    Indicator_Control(INDICATOR_ENABLE);

			UART_SendMessage(DBG_MSG_PC, "[SP_STBY_EXPSW]");

		    while(Exposure_CheckSwitch())
		    {
		    	char cret=0;
		    	if (CurCaptureMode == CAPTURE_CANCEL)
                {
                	Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
            		Tube_CtrlReady(TUBE_READY_DISABLE);

            		Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
					UART_SendMessage(DBG_MSG_PC, "SHOT READY CANCELED");
                    return SET;
                }
				cret=UART_Exposure_SET();
				if(cret==2)
				{
					Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
            		Tube_CtrlReady(TUBE_READY_DISABLE);

            		Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
					UART_SendMessage(DBG_MSG_PC, "SHOT READY CANCELED");
					return RESET;
				}
				else if(cret==1)
					break;
		    }
			check = (BitAction)GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7);	

		    Tube_CtrlReady(TUBE_READY_ENABLE);

		    UART_SendMessage(DBG_MSG_PC, "Wait for Tube Heatting Time");	

		    IntTimer_Delay(2000);
		    while(!IntTimer_GetStatus());

		    UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_START]");

			if(bOpen == RESET)
			{
				if(CalibrationModeParam.nScanTime==5)
				{					
					if (!CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB, CMD_COL_CEPHSCAN_START_FAST, 0x96, 0))
					{
						printUart(DBG_MSG_PC, "Collimator CEPHSCAN_START_FAST failed...!");
					}	
				}
				else
				{
					if (!CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB, CMD_COL_CEPHSCAN_START_NORMAL, 0x96, 0))
			        {
			            UART_SendMessage(DBG_MSG_PC, "CMD_COL_CEPHSCAN_START_NORMAL failed...!");
			        }
				}
			}
			bOpen= RESET;

			Motor_CephScanStart(nEndCnt);

			if(check==Bit_SET)	//Switch release
			{
				while(Motor_C.CurrentStep < nEndCnt + 100)
				{
					if (UART_Exposure_SET()==2)
					{
						break;
					}
					if (Motor_C.CurrentStep == nStartCnt)
					{
						Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
					}
					else if (Motor_C.CurrentStep == nStartCnt + 5)
					{
						ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_ENABLE);
					}
					else if(Tube.bXrayFrameOnOff==SET && (Motor_C.CurrentStep == nEndCnt/4))
					{
						Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
					}
					else if(Tube.bXrayFrameOnOff==SET && (Motor_C.CurrentStep == (nEndCnt*3)/4))
					{
						Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
						Tube.bXrayFrameOnOff=RESET;
					}
					else if (Motor_C.CurrentStep >= nEndCnt)
					{
						ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE);
						Tube_CtrlReady(TUBE_READY_DISABLE);
						Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
					}
				}

			}
			else
			{
				while(Motor_C.CurrentStep < nEndCnt + 100)
			    {
			        if ((Exposure_CheckSwitch()))
			        {
			            break;
			        }
			        if (Motor_C.CurrentStep == nStartCnt)
			        {
			            Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
			        }
					else if (Motor_C.CurrentStep == nStartCnt + 5)
					{
			            ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_ENABLE);
					}
					else if(Tube.bXrayFrameOnOff==SET && (Motor_C.CurrentStep == nEndCnt/4))
					{
						Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
					}
					else if(Tube.bXrayFrameOnOff==SET && (Motor_C.CurrentStep == (nEndCnt*3)/4))
					{
						Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
						Tube.bXrayFrameOnOff=RESET;
					}
			        else if (Motor_C.CurrentStep >= nEndCnt) 
			        {
				        ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE);
			            Tube_CtrlReady(TUBE_READY_DISABLE);
			            Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
			        }
			    }		
			}
			Motor_CephScanEnd();
			Tube.bXrayFrameOnOff=RESET;
		    // jehun - 20200723
			//printUart(DBG_MSG_PC, "Ceph_Colli_end_current_step : %d", Motor_S.CurrentStep);
			//printUart(DBG_MSG_PC, "Ceph_Sensor_end_current_step : %d", Motor_C.CurrentStep);


			ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE);
		    Indicator_Control(INDICATOR_DISABLE);

		    UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_END__]");

		    Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);

		    Lamp_Control(LAMP_RED, LAMP_DISABLE);
		    Lamp_Control(LAMP_GREEN, LAMP_DISABLE);
		    Lamp_Control(LAMP_BLUE, LAMP_ENABLE);

		    Tube_CtrlReady(TUBE_READY_DISABLE);
		    Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);

            UART_SendMessage(DBG_MSG_PC, "UART_CEPH_ALIGN_SCAN_TEST End");
		}
		else if (strstr(message.Data, UART_CEPH_ALIGN_SCAN_TEST_TIME))
		{
			int16_t nTime = atoi(&(message.Data[12]));
			printUart(DBG_MSG_PC, "Scan Test Time = %d sec", nTime);
			CalibrationModeParam.nScanTime=nTime;			
		}
		else if (strstr(message.Data, UART_MOVE_CEPH_BEAM_POS))
		{		
			LaserSetPosition();			
		}
        else if (strstr(message.Data, UART_HEAD_BEAM_ON))
        {
			LaserControl(TYPE_HEAD_CEPH, SET);
        }
        else if (strstr(message.Data, UART_HEAD_BEAM_OFF))
        {
			LaserControl(TYPE_HEAD_CEPH, RESET);
        }		
		else if (strstr(message.Data, UART_CANINE_BEAM_ON))
        {			
			if (!CAN_Collimator_SendMessage(CMD_LASER_CANINE_ON, 0, 0, 5000, 0))
			{
				return RESET;
			}	 
        }
		else if (strstr(message.Data, UART_CANINE_BEAM_OFF))
        {			
			if (!CAN_Collimator_SendMessage(CMD_LASER_CANINE_OFF, 0, 0, 5000, 0))
			{
				return RESET;
			}	 
        }
		else if (strstr(message.Data, UART_ALIGN_BEAM_COLI_REQ))
		{
			if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_HBEAM_LASER, 0, 0, 5000, 0))
			{
				return RESET;
			}	 
		}
		else if (strstr(message.Data, UART_ALIGN_BEAM_SET))
		{
			if (!CAN_Collimator_SendMessage(CMD_OFFSET_HBEAM_LASER_SET, 0, 0, 5000, 0))
			{
				return RESET;
			}	 
		}
		else if (strstr(message.Data, UART_ALIGN_BEAM_COLI))
		{
			if (!CAN_Collimator_SendMessage(CMD_OFFSET_HBEAM_LASER, 0, 0, 5000, 0))
			{
				return RESET;
			}	 
		}		
		else if (strstr(message.Data, UART_BEAM_POS_UP))
		{
			if (!CAN_Collimator_SendMessage(CMD_HBEAM_LASER_UP, 0, 0, 1000, 0))
			{
				return RESET;
			}	 
		}		
		else if (strstr(message.Data, UART_BEAM_POS_DOWN))
		{
			if (!CAN_Collimator_SendMessage(CMD_HBEAM_LASER_DOWN, 0, 0, 1000, 0))
			{
				return RESET;
			}	 
		}		
        else if (strstr(message.Data, UART_COLI_VERSION_CHECK))
        {
            if (!CAN_Collimator_SendMessage(CMD_VERSION_CHECK, 0, 0, 3000, 0))
            {
                printUart(DBG_MSG_PC, "Version Check Error");
                //ret = RESET;
            }    
        }
        else if (strstr(message.Data, UART_COLI_BUILD_CHECK))
        {
            if (!CAN_Collimator_SendMessage(CMD_BUILD_CHECK, 0, 0, 3000, 0))
            {
                printUart(DBG_MSG_PC, "Build Check Error");
                //ret = RESET;
            }    
        }
		else if (strstr(message.Data, UART_TUBE_VERSION_CHECK))
        {
			if (!CAN_SendMessage(CAN_TUBE_VER_CHECK, 0, 0, 3000, 0))
            {
                printUart(DBG_MSG_PC, "Version Check Error");
			}
        }
		else if (strstr(message.Data, UART_GET_TUBE_TEMP))
		{
			CAN_SendMessage(CAN_TUBE_TANK_TEMP_READ, 0, 0, 1000, 2);
		}	
		else if (strstr(message.Data, UART_COLLIMATOR_FILTER_RIGHT))
		{
			if (!CAN_Collimator_SendMessage(CMD_FILTER_CT, 0, 0, 1000, 2))
            {
                return RESET;
            }
		}
		else if (strstr(message.Data, UART_COLLIMATOR_FILTER_CENTER))
		{
			if (!CAN_Collimator_SendMessage(CMD_FILTER_CENTER, 0, 0, 1000, 2))
            {
                return RESET;
            }
		}
		else if (strstr(message.Data, UART_COLLIMATOR_FILTER_LEFT))
		{
			if (!CAN_Collimator_SendMessage(CMD_FILTER_CT_CALIB, 0, 0, 1000, 2))
            {
                return RESET;
            }
		}
#ifdef USE_MOTOR_CHINREST_HOR
		else if (strstr(message.Data, UART_CMD_CHINREST_HOR_MOVE))
		{
			//if (CtParam.Fov == CT_FOV_5X5) 
			{
				int32_t nDistance = atoi(&(message.Data[12]));

				printUart(DBG_MSG_PC, "H-Axis Distance = %dmm", nDistance);

				if (nDistance >= 0)
				{
					//if (strchr(&(message.Data[11]), 'L'))
					if (strchr(&(message.Data[11]), 'R'))
						nDistance = ~nDistance + 1;

					//추후 23?�라??값을 Align Offset?�로 ?�정??�?
					Motor_ControlChinrest(RESET, MOTOR_CWE_INIT_POSITION + nDistance);
					//Motor_MoveChinrestHorizontal(nDistance);
				}
			}
		}
#endif /* USE_MOTOR_CHINREST_HOR */
#ifdef USE_MOTOR_CHINREST_VER
		else if (strstr(message.Data, UART_CMD_CHINREST_VER_MOVE))
		{
			int32_t nDistance = atoi(&(message.Data[12]));

			printUart(DBG_MSG_PC, "V-Axis Distance = %dmm", nDistance);

			if (nDistance >= 0)
			{
				if (strchr(&(message.Data[11]), 'D'))
					nDistance = ~nDistance + 1;

				//추후 58?�라??값을 Align Offset?�로 ?�정??�?
				Motor_ControlChinrest(SET, MOTOR_CNS_CT_INIT_POSITION + nDistance);
				//Motor_MoveChinrestHorizontal(nDistance);
			}
		}
#endif /* USE_MOTOR_CHINREST_VER */		
		else if(strstr(message.Data, UART_TUBE_TILT_OFF))
		{
			if (!CAN_Collimator_SendMessage(CMD_TILT_OFF, 0, 0, 5000, 2))
            {
                return RESET;
            }
		}
		else if(strstr(message.Data, UART_TUBE_TILT_ON))
		{
			if (!CAN_Collimator_SendMessage(CMD_TILT_ON, 0, 0, 5000, 2))
            {
                return RESET;
            }
		}		
		else if(strstr(message.Data, UART_RAXIS_ROOTIN))
		{
			int16_t nCnt = atoi(&(message.Data[12]));
			int16_t nCntDis=0;

			printUart(DBG_MSG_PC, "R-Axis Rootin Count :", nCnt);
			
			if (!CAN_Collimator_SendMessage(CMD_TILT_OFF, 0, 0, 5000, 2))
            {
                return RESET;
            }
			IntTimer_Stop();			
		    
			
			while(nCnt>0)
			{
				/*Motor_R.MinFreq=5400+nCntDis;
				Motor_R.ToOrgParam.RunFreq			= 5400+nCntDis;
				Motor_R.ToLimitParam.RunFreq		= 5400+nCntDis;

				Motor_R.MinFreqCCR = Hz_To_CCR(Motor_R.MinFreq);
			  	Motor_R.Timer.InitCCR = Motor_R.MinFreqCCR;
			    Motor_R.DelayFreqCCR = Hz_To_CCR(Motor_R.DelayFreq);

				Motor_R.ToOrgParam.RunCCR = Hz_To_CCR(Motor_R.ToOrgParam.RunFreq);
			    Motor_R.ToOrgParam.AccCCR = 0;
			    Motor_R.ToOrgParam.DecCCR = 0;

				Motor_R.ToLimitParam.RunCCR = Hz_To_CCR(Motor_R.ToLimitParam.RunFreq);
			    Motor_R.ToLimitParam.AccCCR = 0;
			    Motor_R.ToLimitParam.DecCCR = 0;*/
				nCnt--;
				nCntDis++;
				TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
				Motor_RunOrgCheck(&Motor_R);
				while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
				/*IntTimer_Delay(1000);		
	   			while(!IntTimer_GetStatus());
				Motor_MoveAbsolutePosition(&Motor_R, 3000, 2000, (90 / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO)) + CephParam.nRAxisOffset , 2000);
				while(!Motor_GetStatus(&Motor_R, STATUS_STOP));*/
				
				TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StopCurrent);
				IntTimer_Delay(1500);		
	   			while(!IntTimer_GetStatus());
				printUart(DBG_MSG_PC, "Rootin Current : %d", nCntDis);
			}

			
		}
		else if(strstr(message.Data, UART_ALIGN_AUTO_COLLIMATOR))
		{
			int16_t nCnt =0; // = atoi(&(message.Data[10]));
			uint8_t choose = 0;

			UART_SendMessage(DBG_MSG_PC, "COLLI AUTO ALIGN START");

			while(1)
			{
				choose =UART_Collimator_Auto_SET();
				if(choose !=0 )
					break;

				if (CurCaptureMode == CAPTURE_CANCEL)
                {
					UART_SendMessage(DBG_MSG_PC, "COLLI AUTO ALIGN CANCELED");
                    return SET;
                }
			}

			nCnt=CalibrationModeParam.nColli_Distance;

			printUart(DBG_MSG_PC, "Collimator Range : %d", nCnt);

			switch(choose)
			{
				case 1:
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_POSITION, nCnt, 0, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align Position failed...!");
		                return RESET;
		            }
					break;
				case 6:
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_POSITION, nCnt, 1, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align Position failed...!");
		                return RESET;
		            }
					break;
				case 7:
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_POSITION, nCnt, 2, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align Position failed...!");
		                return RESET;
		            }
					break;
				case 8:
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_POSITION, nCnt, 3, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align Position failed...!");
		                return RESET;
		            }
					break;
				case 2:
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_TOP_POSITION, nCnt, 0, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align Position failed...!");
		                return RESET;
		            }	
					break;
				case 3:
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_BOTTOM_POSITION, nCnt, 0, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align Position failed...!");
		                return RESET;
		            }
					break;
				case 4:
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_LEFT_POSITION, nCnt, 0, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align Position failed...!");
		                return RESET;
		            }
					break;
				case 5:
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_RIGHT_POSITION, nCnt, 0, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align Position failed...!");
		                return RESET;
		            }
					break;
				default:
					break;
			}
			

			if (CalibrationModeParam.KV == 0)
			{
				CalibrationModeParam.KV = 80;
			}

			if (CalibrationModeParam.mA == 0)
			{
				CalibrationModeParam.mA = 80;
			}
			
		    if (!CAN_SendMessage(CAN_TUBE_KV_SET, CalibrationModeParam.KV, 0, 1000, 2))
		    {
	            UART_SendMessage(DBG_MSG_PC, "KV_SET_Error");				
	            return RESET;
		    }

		    if (!CAN_SendMessage(CAN_TUBE_mA_SET, CalibrationModeParam.mA, 0, 1000, 2))
		    {
	            UART_SendMessage(DBG_MSG_PC, "mA_SET_Error");				
	            return RESET;
		    }
			

		    Exposure_CtrlLed(EXPOSURE_LED_GPIO_ENABLE);

		    Indicator_Control(INDICATOR_ENABLE);

			UART_SendMessage(DBG_MSG_PC, "[SP_STBY_EXPSW]");

		    while(Exposure_CheckSwitch())
		    {
		    	char cret=0;
				cret=UART_Exposure_SET();
				if(cret==2)
				{
					Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
            		Tube_CtrlReady(TUBE_READY_DISABLE);

            		Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
					UART_SendMessage(DBG_MSG_PC, "COLLI AUTO ALIGN CANCELED");
					return RESET;
				}
				else if(cret==1)
					break;
				
		    	if (CurCaptureMode == CAPTURE_CANCEL)
                {
                	Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
            		Tube_CtrlReady(TUBE_READY_DISABLE);

            		Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
					UART_SendMessage(DBG_MSG_PC, "COLLI AUTO ALIGN CANCELED");
                    return SET;
                }	
		    }

		    Tube_CtrlReady(TUBE_READY_ENABLE);

		    UART_SendMessage(DBG_MSG_PC, "Wait for Tube Heatting Time");			

		    IntTimer_Delay(2000);
		    while(!IntTimer_GetStatus());
			
			ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_ENABLE);
		    UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_START]");

			switch(choose)
			{
				case 1:
					if (!CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB, CMD_COLLI_CALIBRATION_MOVE, nCnt, 0))
					{
						printUart(DBG_MSG_PC, "Collimator Align MOVE failed...!");
					}	
					break;
				case 2:
					if (!CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB, CMD_COLLI_CALIBRATION_TOP_MOVE, nCnt, 0))
					{
						printUart(DBG_MSG_PC, "Collimator Align MOVE failed...!");
					}	
					break;
				case 3:
					if (!CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB, CMD_COLLI_CALIBRATION_BOTTOM_MOVE, nCnt, 0))
					{
						printUart(DBG_MSG_PC, "Collimator Align MOVE failed...!");
					}	
					break;
				case 4:
					if (!CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB, CMD_COLLI_CALIBRATION_LEFT_MOVE, nCnt, 0))
					{
						printUart(DBG_MSG_PC, "Collimator Align MOVE failed...!");
					}	
					break;
				case 5:
					if (!CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB, CMD_COLLI_CALIBRATION_RIGHT_MOVE, nCnt, 0))
					{
						printUart(DBG_MSG_PC, "Collimator Align MOVE failed...!");
					}	
				case 6:
					if (!CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB, CMD_COLLI_CALIBRATION_PANO_MOVE, nCnt, 0))
					{
						printUart(DBG_MSG_PC, "Collimator Align MOVE failed...!");
					}	
				case 7:
					if (!CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB, CMD_COLLI_CALIBRATION_CT_MOVE, nCnt, 0))
					{
						printUart(DBG_MSG_PC, "Collimator Align MOVE failed...!");
					}	
				case 8:
					if (!CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB, CMD_COLLI_CALIBRATION_SCAN_MOVE, nCnt, 0))
					{
						printUart(DBG_MSG_PC, "Collimator Align MOVE failed...!");
					}	
					break;
				default:
					break;
			}
			
			Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);

			IntTimer_Stop();
			IntTimer_Delay(20000);
			while(!Exposure_CheckSwitch())
		    {
		    	char cret=0;
		        cret=UART_Collimator_Auto_STOP();

				if(cret==1)
				{
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_STOP, 0, 0, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align Stop failed...!");						
		                break;
		            }
				}
				else if(cret==2)
				{
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_PANO_TOP_STOP, 0, 0, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align TOP Stop failed...!");						
		                break;
		            }
				}
				else if(cret==3)
				{
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_PANO_BOTTOM_STOP, 0, 0, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align BOTTOM Stop failed...!");						
		                break;
		            }
				}
				else if(cret==4)
				{
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_PANO_LEFT_STOP, 0, 0, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align LEFT Stop failed...!");						
		                break;
		            }
				}
				else if(cret==5)
				{
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_PANO_RIGHT_STOP, 0, 0, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align RIGHT Stop failed...!");						
		                break;
		            }
				}
				else if(cret==6)
				{
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_CT_TOP_STOP, 0, 0, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align TOP Stop failed...!");						
		                break;
		            }
				}
				else if(cret==7)
				{
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_CT_BOTTOM_STOP, 0, 0, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align BOTTOM Stop failed...!");						
		                break;
		            }
				}
				else if(cret==8)
				{
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_CT_LEFT_STOP, 0, 0, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align LEFT Stop failed...!");						
		                break;
		            }
				}
				else if(cret==9)
				{
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_CT_RIGHT_STOP, 0, 0, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align RIGHT Stop failed...!");						
		                break;
		            }
				}
				else if(cret==10)
				{
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_SCAN_TOP_STOP, 0, 0, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align TOP Stop failed...!");						
		                break;
		            }
				}
				else if(cret==11)
				{
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_SCAN_BOTTOM_STOP, 0, 0, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align BOTTOM Stop failed...!");						
		                break;
		            }
				}
				else if(cret==12)
				{
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_SCAN_LEFT_STOP, 0, 0, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align LEFT Stop failed...!");						
		                break;
		            }
				}
				else if(cret==13)
				{
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_SCAN_RIGHT_STOP, 0, 0, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align RIGHT Stop failed...!");						
		                break;
		            }
				}
				else if(cret==14)
				{
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_STOP, 0, 0, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align Stop failed...!");						
		                break;
		            }
					break;
				}

				if(IntTimer_GetStatus())
				{
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_STOP, 0, 0, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align Stop failed...!");						
		                break;
		            }
					break;
				}

				if (CurCaptureMode == CAPTURE_CANCEL)
                {
                	Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
            		Tube_CtrlReady(TUBE_READY_DISABLE);
					ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE);

            		Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
					Indicator_Control(INDICATOR_DISABLE);
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_STOP, 0, 0, 1000, 2))
		            {
		            	printUart(DBG_MSG_PC, "Collimator Align Stop failed...!");						
		                break;
		            }
					UART_SendMessage(DBG_MSG_PC, "COLLI AUTO ALIGN CANCELED");
                    break;
                }
				
		    }	
			IntTimer_Stop();
			Tube_CtrlReady(TUBE_READY_DISABLE);
			Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
		    Indicator_Control(INDICATOR_DISABLE);
			ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE);

		    UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_END__]");

		    Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);

		    Lamp_Control(LAMP_RED, LAMP_DISABLE);
		    Lamp_Control(LAMP_GREEN, LAMP_DISABLE);
		    Lamp_Control(LAMP_BLUE, LAMP_ENABLE);

		    Tube_CtrlReady(TUBE_READY_DISABLE);
		    Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);

            UART_SendMessage(DBG_MSG_PC, "UART_COLLI_AUTO_ALIGN End");
			
			
		}
        else																			
        {
            printUart(DBG_MSG_PC, "%s : %s", ERR_CODE_UART_MSG_ERROR, message.Data);
            return RESET;
        }

        message.Data[1] = 'E';
        printUart(DBG_MSG_PC, message.Data);
    }

    return ret;
}

#ifdef USE_I2C_EEPROM
/**

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/
