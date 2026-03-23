/*
*********************************************************************************************
* can.h :
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
#ifndef __CAN_H__
#define __CAN_H__


/* Exported define ----------------------------------------------------- */
#define	CAN			CAN2

#define	CAN_STD_ID								0x321
#define	CAN_EXT_ID_SUBB_TO_SYSTEM				0x12FF3301
#define	CAN_EXT_ID_TUBE_TO_SYSTEM				0x12FF3401
#define	CAN_EXT_ID_SYSTEM_TO_SUBB				0x12FF3302
#define	CAN_EXT_ID_SYSTEM_TO_TUBE				0x12FF3402

#define	CAN_FRAME_LENGTH_BYTE					8
#define	CAN_FRAME_CMD_BYTE						2
#define	CAN_FRAME_VALUE_BYTE					4
#define	CAN_FRAME_DEBUG_BYTE					2

#define CAN_TUBE_GEN2_COM						100

/* Command */
#define CMD_COLLI_CT15X9 							0x1224

#define CMD_COMM_CHECK 								0x1000
#define CMD_VERSION_CHECK 							0x1001
#define CMD_BUILD_CHECK								0x1002
#define CMD_BUILD_SUB_CHECK							0x1003

#define CMD_INIT_SYSTEM 							0x1100
#define	CMD_EEPROM_INIT					            0x1101
#define	CMD_STATUS_CHECK				            0x1110

#define CMD_COLLI_CLOSE 							0x1200
#define CMD_COLLI_OPEN 								0x1201

#define CMD_COLLI_PANOR 							0x1210
#define CMD_COLLI_PANOR_CHILD						0x1211

#define CMD_COLLI_CT5X5 							0x1220
#define CMD_COLLI_CT8X8 							0x1221
#define CMD_COLLI_CT10X9 							0x1222
#define CMD_COLLI_CT12X9 							0x1223
#define CMD_COLLI_CT15X9 							0x1224
#define CMD_COLLI_CT10X10 							0x1225
#define CMD_COLLI_CT12X10 							0x1226
#define CMD_COLLI_CT15X10 							0x1227
#define CMD_COLLI_CT8X9 							0x1228
#define CMD_COLLI_CT8X10 							0x1229

#define CMD_COLLI_CEPHONE 							0x1230
#define CMD_COLLI_CEPHSCAN_INIT 					0x1231

#define CMD_COL_CEPHSCAN_START_NORMAL 				0x1233
#define CMD_COL_CEPHSCAN_START_FAST 				0x1232
#define CMD_COL_CEPHSCAN_START_4SEC 				0x1235
#define CMD_COL_CEPHSCAN_START_3SEC 				0x1236
#define CMD_COL_CEPHSCAN_START_2SEC 				0x1237
#define	CMD_COL_CEPHSCAN_CHILD						0x1238


#define CMD_COLLI_PANO_CALIB 						0x1240
#define	CMD_COLLI_PANO_180PX						0x1241

#define	CMD_COLLI_CEPH_AP_PA						0x1245
#define	CMD_COLLI_CEPH_CAPUS						0x1246

#define	CMD_COLLI_CALIBRATION_POSITION				0x1250
#define CMD_COLLI_CALIBRATION_TOP_POSITION			0x1251
#define CMD_COLLI_CALIBRATION_BOTTOM_POSITION		0x1252
#define CMD_COLLI_CALIBRATION_LEFT_POSITION			0x1253
#define CMD_COLLI_CALIBRATION_RIGHT_POSITION		0x1254

#define	CMD_COLLI_CALIBRATION_MOVE					0x1255
#define	CMD_COLLI_CALIBRATION_TOP_MOVE				0x1256
#define	CMD_COLLI_CALIBRATION_BOTTOM_MOVE			0x1257
#define	CMD_COLLI_CALIBRATION_LEFT_MOVE				0x1258
#define	CMD_COLLI_CALIBRATION_RIGHT_MOVE			0x1259

#define	CMD_COLLI_CALIBRATION_STOP					0x1260
#define	CMD_COLLI_CALIBRATION_PANO_TOP_STOP			0x1261
#define	CMD_COLLI_CALIBRATION_PANO_BOTTOM_STOP		0x1262
#define	CMD_COLLI_CALIBRATION_PANO_LEFT_STOP		0x1263
#define	CMD_COLLI_CALIBRATION_PANO_RIGHT_STOP		0x1264
#define	CMD_COLLI_CALIBRATION_CT_TOP_STOP			0x1265
#define	CMD_COLLI_CALIBRATION_CT_BOTTOM_STOP		0x1266
#define	CMD_COLLI_CALIBRATION_CT_LEFT_STOP			0x1267
#define	CMD_COLLI_CALIBRATION_CT_RIGHT_STOP			0x1268
#define	CMD_COLLI_CALIBRATION_SCAN_TOP_STOP			0x1269
#define	CMD_COLLI_CALIBRATION_SCAN_BOTTOM_STOP		0x1270
#define	CMD_COLLI_CALIBRATION_SCAN_LEFT_STOP		0x1271
#define	CMD_COLLI_CALIBRATION_SCAN_RIGHT_STOP		0x1272

#define	CMD_COLLI_CALIBRATION_PANO_MOVE				0x1280
#define	CMD_COLLI_CALIBRATION_CT_MOVE				0x1281
#define	CMD_COLLI_CALIBRATION_SCAN_MOVE				0x1282


#define CMD_FILTER_CENTER 							0x1300
#define CMD_FILTER_CT_CALIB 						0x1310
#define CMD_FILTER_CT 								0x1320

#define CMD_LASER_UP 								0x1400
#define CMD_LASER_DOWN 								0x1410
#define CMD_LASER_STOP 								0x1420

#define CMD_TILT_STATUS 							0x1500
#define CMD_TILT_ON 								0x1510
#define CMD_TILT_OFF 								0x1520

#define CMD_LASER_HORI_TOP_ON 						0x1600
#define CMD_LASER_HORI_TOP_OFF 						0x1601

#define CMD_LASER_HORI_BOT_ON 						0x1610
#define CMD_LASER_HORI_BOT_OFF 						0x1611

#define CMD_LASER_CANINE_ON 						0x1620
#define CMD_LASER_CANINE_OFF 						0x1621

#define CMD_REQ_OFFSET_PANO_T 						0x1710
#define CMD_REQ_OFFSET_PANO_B 						0x1711
#define CMD_REQ_OFFSET_PANO_L 						0x1712
#define CMD_REQ_OFFSET_PANO_R				 		0x1713

#define CMD_OFFSET_PANO_T 							0x1760
#define CMD_OFFSET_PANO_B 							0x1761
#define CMD_OFFSET_PANO_L 							0x1762
#define CMD_OFFSET_PANO_R 							0x1763

#define	CMD_COLI_CEPHSCAN_CENTER					0x1234

#define	CMD_REQ_OFFSET_CT_T			    			0x1720
#define	CMD_REQ_OFFSET_CT_B			    			0x1721
#define	CMD_REQ_OFFSET_CT_L			  				0x1722
#define	CMD_REQ_OFFSET_CT_R			    			0x1723

#define	CMD_OFFSET_CT_T				    			0x1770
#define	CMD_OFFSET_CT_B				    			0x1771
#define	CMD_OFFSET_CT_L				    			0x1772
#define	CMD_OFFSET_CT_R				    			0x1773

#define	CMD_REQ_OFFSET_CEPHSCAN_T					0x1730
#define	CMD_REQ_OFFSET_CEPHSCAN_B					0x1731
#define	CMD_REQ_OFFSET_CEPHSCAN_L					0x1732
#define	CMD_REQ_OFFSET_CEPHSCAN_R					0x1733

#define	CMD_OFFSET_CEPHSCAN_T						0x1780
#define	CMD_OFFSET_CEPHSCAN_B						0x1781
#define	CMD_OFFSET_CEPHSCAN_L						0x1782
#define	CMD_OFFSET_CEPHSCAN_R						0x1783

#define	CMD_REQ_OFFSET_CEPHONE_T					0x1740
#define	CMD_REQ_OFFSET_CEPHONE_B					0x1741
#define	CMD_REQ_OFFSET_CEPHONE_L					0x1742
#define	CMD_REQ_OFFSET_CEPHONE_R					0x1743

#define	CMD_OFFSET_CEPHONE_T						0x1790
#define	CMD_OFFSET_CEPHONE_B						0x1791
#define	CMD_OFFSET_CEPHONE_L						0x1792
#define	CMD_OFFSET_CEPHONE_R						0x1793

#define CMD_REQ_OFFSET_HBEAM_LASER					0x1750
#define CMD_OFFSET_HBEAM_LASER_SET					0x17B2
#define CMD_OFFSET_HBEAM_LASER						0x17A0
//#define CMD_OFFSET_HBEAM_LASER			0x17A0
#define CMD_HBEAM_LASER_UP							0x17B0
#define CMD_HBEAM_LASER_DOWN        				0x17B1

#define CMD_COLLI_MOTOR_ALL_STATUS_STOP				0x1901
#define CMD_COLLI_MOTOR_ALL_RUN						0x1902

#define CMD_COLLI_MOTOR_T_STATUS_STOP				0x1911
#define CMD_COLLI_MOTOR_T_RUN						0x1912

#define CMD_COLLI_MOTOR_B_STATUS_STOP				0x1921
#define CMD_COLLI_MOTOR_B_RUN						0x1922

#define CMD_COLLI_MOTOR_L_STATUS_STOP				0x1931
#define CMD_COLLI_MOTOR_L_RUN						0x1932

#define CMD_COLLI_MOTOR_R_STATUS_STOP				0x1941
#define CMD_COLLI_MOTOR_R_RUN						0x1942

#define CMD_COLLI_MOTOR_F_STATUS_STOP				0x1951
#define CMD_COLLI_MOTOR_F_RUN						0x1952

#define CMD_COLLI_MOTOR_H_STATUS_STOP				0x1961
#define CMD_COLLI_MOTOR_H_RUN						0x1962


#define	CAN_TUBE_COM_CHECK 							0x2000
#define	CAN_TUBE_VER_CHECK 							0x2001
#define	CAN_TUBE_COUNT_CHECK 						0x2002
#define CAN_TUBE_COUNT_RESET						0x2003
#define CAN_Sub_Version_Check						0x2004

#define	CAN_TUBE_CONTI_MODE_SET 					0x2100
#define	CAN_TUBE_PULSE_MODE_SET 					0x2101
#define	CAN_TUBE_TANK_TEMP_SET 		   				0x2102
#define CAN_TUBE_TANK_TEMP_SET_READ				    0x2103

#define	CAN_TUBE_KV_SET 							0x2200
#define	CAN_TUBE_KV_REF_READ 						0x2201
#define	CAN_TUBE_KV_FB_READ 						0x2202

#define	CAN_TUBE_mA_SET 							0x2300
#define	CAN_TUBE_mA_REF_READ 						0x2301
#define	CAN_TUBE_mA_FB_READ 						0x2302

#define	CAN_TUBE_READY_ON 							0x2400
#define	CAN_TUBE_XRAY_ON 							0x2401
#define	CAN_TUBE_XRAY_OFF 							0x2402

#define	CAN_TUBE_TANK_TEMP_READ 					0x2501

#define	CAN_TUBE_SOUND_OFF							0x2600
#define CAN_TUBE_SOUND_ON							0x2601


#define	CAN_TUBE_RESPONSE 				        	0x2600

#define CAN_TUBE_KV_STB_ERROR			        	0x2710
#define CAN_TUBE_mA_STB_ERROR 			        	0x2711
#define CAN_TUBE_FIL_STB_ERROR 			        	0x2712
#define CAN_TUBE_READY_FIL_ERROR 		        	0x2713
#define CAN_TUBE_KV_HIGH_OUTPUT 		        	0x2714
#define CAN_TUBE_KV_LOW_OUTPUT 		       	 		0x2715
#define CAN_TUBE_mA_HIGH_OUTPUT 		        	0x2716
#define CAN_TUBE_mA_LOW_OUTPUT 		        		0x2717
#define CAN_TUBE_KV_REF_ERROR 			        	0x2719
#define CAN_TUBE_mA_REF_ERROR 			        	0x2720
#define CAN_TUBE_TANK_TEMP_ERROR 		        	0x2721
#define CAN_TUBE_TANK_TEMP_OK_SIGNAL 	    		0x2722
#define CAN_TUBE_SEL_KV_mA_ERROR 		        	0x2723
#define CAN_TUBE_KV_REF_OUT_HI_OVER_ERROR 			0x2724
#define CAN_TUBE_KV_REF_OUT_LOW_OVER_ERROR 			0x2725
#define CAN_TUBE_mA_REF_OUT_HI_OVER_ERROR 			0x2726
#define CAN_TUBE_mA_REF_OUT_LOW_OVER_ERROR 			0x2727
#define CAN_TUBE_LIMIT_ERROR 			        	0x2728
#define CAN_TUBE_SEL_KV_DAC_ERROR 	        		0x2729
#define CAN_TUBE_SEL_mA_DAC_ERROR 	        		0x2730
#define CAN_TUBE_TANK_CON_ERROR 		    	    0x2731
#define CAN_TUBE_TANK_TEMP_SET_ERROR	        	0x2732
#define CAN_TUBE_CMD_ERROR          	   		    0x2733
#define CAN_Tank_FAN_Temp_Setting_ERROR				0x2736				//SP_ERRT_00312

//200324 HWAN
#define CAN_TUBE_kV_HIGH_Output_WARNING   			0x2750				//WARNING_102
#define CAN_TUBE_kV_LOW_Output_WARNING   			0x2751				//WARNING_103
#define CAN_TUBE_kV_Ref_Out_HI_OVER_WARNING   		0x2752				//WARNING_105
#define CAN_TUBE_kV_Ref_Out_LOW_OVER_WARNING   		0x2753				//WARNING_106
#define CAN_TUBE_mA_HIGH_Output_WARNING   			0x2754				//WARNING_202
#define CAN_TUBE_mA_LOW_Output_WARNING   			0x2755				//WARNING_203
#define CAN_TUBE_mA_Ref_Out_HI_OVER_WARNING   		0x2756				//WARNING_205
#define CAN_TUBE_mA_Ref_Out_LOW_OVER_WARNING   		0x2757				//WARNING_206 

#define CAN_TUBE_FAN_ACTIVE_FLAG					0x2780
#define CAN_TUBE_FAN_ACTIVE_READ_FLAG				0x2781
#define CAN_TUBE_FAN_ACTIVE_TEMP_SET				0x2782
#define CAN_TUBE_FAN_ACTIVE_TEMP_READ				0x2783


//201211 HWAN
#define CAN_TUBE_KV_ADC_VALUE_CHECK					0x2800
#define CAN_TUBE_MA_ADC_VALUE_CHECK					0x2801
//#define CAN_TUBE_KV_ERROR_CHECK_PERCENT			0x2802
//#define CAN_TUBE_MA_ERROR_CHECK_PERCENT			0x2803
//#define CAN_TUBE_KV_HIGH_ERROR_COUNT				0x2804
//#define CAN_TUBE_KV_LOW_ERROR_COUNT				0x2805
//#define CAN_TUBE_MA_HIGH_ERROR_COUNT				0x2806
//#define CAN_TUBE_MA_LOW_ERROR_COUNT				0x2807
#define CAN_TUBE_KV_ADD_VALUE_PERCENT				0x2808
#define CAN_TUBE_KV_ADD_VALUE_PERCENT_MINUS			0x2809
#define CAN_TUBE_MA_ADD_VALUE_PERCENT				0x2810
#define CAN_TUBE_MA_ADD_VALUE_PERCENT_MINUS			0x2811
#define CAN_TUBE_KV_MA_ERROR_DISABLE				0x2812
#define CAN_TUBE_ERROR_VALUE_CHECK					0x2813
#define CAN_TUBE_RESET_ERROR_TABLE					0x2814
#define CAN_TUBE_RESET_INIRIAL_VALUE				0x2815
//#define CAN_TUBE_TEST_VERSION_CHECK				0x2816
//#define CAN_TUBE_KV_MA_WRITE						0x2817
#define CAN_TUBE_TOTAL_ERROR_CHECK					0x2818
#define CAN_TUBE_KV_ADD_VALUE_CHECK					0x2819
#define CAN_TUBE_MA_ADD_VALUE_CHECK					0x2820
//#define CAN_TUBE_kV_MA_CHECK_COUNT				0x2821
//#define CAN_TUBE_kV_REAL_TIME						0x2822
//#define CAN_TUBE_KV_ADC_VALUE_REAL_TIME_CHECK		0x2823
//#define CAN_TUBE_MA_ADC_VALUE_REAL_TIME_CHECK		0x2824
//#define CAN_TUBE_mA_REAL_TIME						0x2825
//#define CAN_TUBE_kV_mA_DESTINATION_TIME			0x2826

#define CAN_TUBE_READY_TIME_CHECK					0x2850
#define CAN_TUBE_EXP_TIME_CHECK						0x2851

#define CAN_TUBE_kV_MIN_VALUE_CHECK					0x2860
#define CAN_TUBE_kV_MAX_VALUE_CHECK					0x2861
#define CAN_TUBE_mA_MIN_VALUE_CHECK					0x2862
#define CAN_TUBE_mA_MAX_VALUE_CHECK					0x2863
#define CAN_TUBE_KV_STANDBY_ADC_VALUE_CHECK			0x2864
#define CAN_TUBE_MA_STANDBY_ADC_VALUE_CHECK			0x2865

#define CAN_TUBE_EEPROM_VALUE_CHECK					0x2900 
#define CAN_TUBE_EEPROM_ONLY_ONE_CHK				0x2901
#define CAN_TUBE_EEPROM_JUMP_STATUS					0x2902
#define CAN_TUBE_EEPROM_CHK_EXIST_APP				0x2903
#define CAN_TUBE_EEPROM_CONNECT_STATUS				0x2904
#define CAN_TUBE_EEPROM_CHK_APP_SECTION				0x2905
#define CAN_TUBE_EEPROM_APP_FIRST_SECTION			0x2906

#define CAN_TUBE_EEPROM_FIRSTBOOT_ADDRESS			0x2907
#define CAN_TUBE_EEPROM_BUZZER_ADDRESS				0x2908
#define CAN_TUBE_EEPROM_TEMPERATURE_ADDRESS			0x2909
#define CAN_TUBE_EEPROM_XRAY_CNT_ADDRESS			0x2910


/* Exported macro ---------------------------------------------------- */
/* Exported variables -------------------------------------------------- */
extern CanRxMsg CAN_RxMessage;


/* Exported functions -------------------------------------------------- */
void CAN_Config(void);
//bool CAN_SendMessage(uint16_t cmd, uint32_t value, uint16_t debug, uint32_t timeOut, uint8_t retryCnt);
//bool CAN_SendMessage_NoResponse(uint32_t extid, uint16_t cmd, uint32_t value, uint16_t debug);
//bool CAN_Collimator_SendMessage(uint16_t cmd,
//            uint32_t value, uint16_t debug, uint32_t timeOut, uint8_t retryCnt);
void CAN_ParseMessage(void);


#endif /* __CAN_H__ */
