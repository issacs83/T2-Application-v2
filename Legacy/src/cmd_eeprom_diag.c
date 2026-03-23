/*
*******************************************************************************
* cmd_eeprom_diag.c : EEPROM and Diagnostic mode command handlers
*******************************************************************************
* Copyright (C) 2014-2018 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Brief   : UART_SetRomParam and UART_SetDiagParam -- processes queued
*             commands during EEPROM and diagnostic modes.
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
#include "Diagnostic.h"

/* Private defines --------------------------------------------------------- */
#define UART_COLLI_READ_ALL					"[SM_COLI_ALL__]"
#define UART_MODE_EEPROM 					"[SM_MODE_EEPROM]"
#define UART_MOTOR_A_END	 						"[SM_MOTA___END]"
#define UART_MOTOR_ALL_ORG_SET						"[SM_MOTALL_ORG"
#define UART_MOTOR_A_ORG	 						"[SM_MOTA___ORG]"
#define UART_MOTOR_A_STEP	 						"[SM_MOTA_"
#define UART_MOTOR_A_STOP	 						"[SM_MOTA__STOP]"
#define UART_MOTOR_A_TEST								"[SM_MOTA_VOL"
#define UART_MOTOR_C_END	 						"[SM_MOTC___END]"
#define UART_MOTOR_CNS_END	 						"[SM_MOTCN__END]"
#define UART_MOTOR_CNS_ORG	 						"[SM_MOTCN__ORG]"
#define UART_MOTOR_CNS_STEP	 						"[SM_MOTCN"
#define UART_MOTOR_CNS_STOP	 						"[SM_MOTCN_STOP]"
#define UART_MOTOR_CNS_TEST								"[SM_MOTN_VOL"
#define UART_MOTOR_C_ORG	 						"[SM_MOTC___ORG]"
#define UART_MOTOR_C_STEP	 						"[SM_MOTC_"
#define UART_MOTOR_C_STOP	 						"[SM_MOTC__STOP]"
#define UART_MOTOR_C_TEST								"[SM_MOTC_VOL"
#define UART_MOTOR_CWE_END	 						"[SM_MOTCW__END]"
#define UART_MOTOR_CWE_ORG	 						"[SM_MOTCW__ORG]"
#define UART_MOTOR_CWE_STEP	 						"[SM_MOTCW"
#define UART_MOTOR_CWE_STOP	 						"[SM_MOTCW_STOP]"
#define UART_MOTOR_CWE_TEST								"[SM_MOTW_VOL"
#define UART_MOTOR_ORG_TEST							"[SM_MOT__ORG_"
#define UART_MOTOR_READ_DATA						"[SM_READ_MOTOR]"
#define UART_MOTOR_R_END	 						"[SM_MOTR___END]"
#define UART_MOTOR_R_ORG	 						"[SM_MOTR___ORG]"
#define UART_MOTOR_R_STEP	 						"[SM_MOTR_"
#define UART_MOTOR_R_STOP	 						"[SM_MOTR__STOP]"
#define UART_MOTOR_R_TEST								"[SM_MOTR_VOL"
#define UART_MOTOR_SELECTION						"[SM_MOT_SEL_"
#define UART_MOTOR_S_END	 						"[SM_MOTS___END]"
#define UART_MOTOR_SEND_SPI_DATA					"[SM_SEND_MOTOR]"
#define UART_MOTOR_S_ORG	 						"[SM_MOTS___ORG]"
#define UART_MOTOR_SPI_DATA							"[SM_SPI0x"
#define UART_MOTOR_S_STEP	 						"[SM_MOTS_"
#define UART_MOTOR_S_STOP	 						"[SM_MOTS__STOP]"
#define UART_MOTOR_S_TEST								"[SM_MOTS_VOL"
#define UART_MOTOR_T_CHILD_END	 					"[SM_MOTT_CHEND]"
#define UART_MOTOR_T_END	 						"[SM_MOTT___END]"
#define UART_MOTOR_T_ORG	 						"[SM_MOTT___ORG]"
#define UART_MOTOR_T_STEP	 						"[SM_MOTT_"
#define UART_MOTOR_T_STOP	 						"[SM_MOTT__STOP]"
#define UART_MOTOR_T_TEST								"[SM_MOTT_VOL"
#define UART_MOTOR_V_END	 						"[SM_MOTV___END]"
#define UART_MOTOR_V_MOVE_TEST							"[SM_MOTVM"
#define UART_MOTOR_V_ORG	 						"[SM_MOTV___ORG]"
#define UART_MOTOR_V_STEP	 						"[SM_MOTV_"
#define UART_MOTOR_V_STOP	 						"[SM_MOTV__STOP]"
#define UART_MOTOR_V_TEST								"[SM_MOTV_VOL"
#define UART_QUEUE_TEST_PROTOCOL					"[SM_TEST_NUM"
#define UART_ROM_APPLY						"[SM_ROM__APPLY]"
#define UART_ROM_ERASE_ADDR 	  	    	"[SM_ROME__0X"
#define UART_ROM_ERASE_ALL       			"[SM_ROME_ALL__]"
#define UART_ROM_ERASE_BOARD 			    "[SM_ROME_BOARD]"
#define UART_ROM_ERASE_INITDIR_MOT_CC	    		"[SM_ROME_CCDIR]"
#define UART_ROM_ERASE_MODEL	  			"[SM_ROME_MODEL]"
#define UART_ROM_MODE_CONSOLE 				"[SM_ROMM_CONSO]"
#define UART_ROM_MODE_PROTOCOL 				"[SM_ROMM_PROTO]"
#define UART_ROM_READ_ADDR     		  	    "[SM_ROMR__0X"
#define UART_ROM_READ_ALL       			"[SM_ROMR_ALL__]"
#define UART_ROM_READ_BOARD 		 	   	"[SM_ROMR_BOARD]"
#define UART_ROM_READ_COLLIMATOR_OFFSET		"[SM_ROMR_COLOF]"
#define UART_ROM_READ_INITDIR_MOT_CC	    		"[SM_ROMR_CCDIR]"
#define UART_ROM_READ_MAP  			    	"[SM_ROMR_MAP__]"
#define UART_ROM_READ_MODEL  		    	"[SM_ROMR_MODEL]"
#define UART_ROM_READ_OFFSET 				"[SM_ROMR_OFFSE]"
#define UART_ROM_WRITE_ADDR    		     	"[SM_ADDR__0X"
#define UART_ROM_WRITE_BOARD 			    "[SM_ROMW_B" //[SM_ROMW_B0103]
#define UART_ROM_WRITE_DATA     	    	"[SM_DATA_"
#define UART_ROM_WRITE_INITDIR_MOT_CC	  			"[SM_ROMW_CCD_"
#define UART_ROM_WRITE_MODEL 		    	"[SM_ROMW_M" //[SM_ROMW_MT1CS] or [SM_ROMW_MT1C_]
#define UART_ROM_WRITE_SET      	   	    "[SM_ROMW__SET_]"
#define UART_STITCH_CHECK_TEST 						"[SM_SWIT_CHECK]"
#define UART_TUBE_CHECK_ON_OFF						"[SM_TANK_CHKO"
#define UART_TUBE_ERROR_ADC_RESET					"[SM_TANK_RESET]"
#define UART_TUBE_ERROR_ON_OFF						"[SM_TANK_ERRO"
#define UART_TUBE_ERROR_VALUE_CHECK					"[SM_TANK_VALUE]"
#define UART_TUBE_EXPOSURE_FEEDBACK_MAX_MIN_VALUE_CHECK		"[SM_TANK_EMXMN]"
#define UART_TUBE_EXPOSURE_TIME_CHECK				"[SM_TANK_ETIME]"
#define UART_TUBE_FAN_ON_OFF						"[SM_TANK_FANO"
#define UART_TUBE_FAN_READ_ON_OFF_FLAG				"[SM_TANK_RDFAN]"
#define UART_TUBE_FAN_TEMP_SETTING_READ				"[SM_TANK_RDFTP]"
#define UART_TUBE_FAN_TEMP_SETTING					"[SM_TANK_FTP"
#define UART_TUBE_INITIAL_VALUE						"[SM_TANK_INIT_]"
#define UART_TUBE_KV_ADC_VALUE_CHECK				"[SM_TKVE_CHK"
#define UART_TUBE_KV_ADC_VALUE_NUM_CHECK			"[SM_TKVE_NUM"
#define UART_TUBE_KV_STANDBY_VALUE_CHECK			"[SM_TKVS_CHK"
#define UART_TUBE_MA_ADC_VALUE_CHECK				"[SM_TMAE_CHK"
#define UART_TUBE_MA_ADC_VALUE_NUM_CHECK			"[SM_TMAE_NUM"
#define UART_TUBE_MA_STANDBY_VALUE_CHECK			"[SM_TMAS_CHK"
#define UART_TUBE_READY_TIME_CHECK					"[SM_TANK_RTIME]"
#define UART_TUBE_TEMP_SETTING_READ					"[SM_TANK_RDTMP]"
#define UART_TUBE_TEMP_SETTING						"[SM_TANK_TMP"
#define UART_TUBE_TOTAL_ERROR_ON_OFF				"[SM_TANK_TERO"
#define UART_CAN_TEST_COLI_COMM_CHECK			"[SM_CANC_COMC_]"
#define UART_CAN_TEST_TUBE_COM_CHECK 			"[SM_CANT_COMC_]"
#define UART_COLI_BUILD_CHECK        		"[SM_CANC_BUILD]"
#define UART_COLI_VERSION_CHECK 			"[SM_CANC_VERC_]"
#define UART_GEN1_EXPOSE_DISABLE 				"[SM_GEN1_EXPDS]"
#define UART_GEN1_EXPOSE_ENABLE 				"[SM_GEN1_EXPEN]"
#define UART_GEN1_READY_DISABLE 				"[SM_GEN1_RDYDS]"
#define UART_GEN1_READY_ENABLE 					"[SM_GEN1_RDYEN]"
#define UART_GEN1_TEST		 					"[SM_GEN1_TEST_]"
#define UART_GET_ALL_LOCKED_VERSION             "[SM_GETA_VER__]"
#define UART_GET_ALL_RELEASED_VERSION           "[SM_GETA_BUILD]"
#define UART_MEMB_BUTTON_DELAY_VALUE		"[SM_SWDLY_"
#define UART_MEMB_CANCEL_DELAY_VALUE		"[SM_CCDLY_"
#define UART_MEMB_MELODY_SELECT				"[SM_MELO_SEL"
#define UART_MEMB_READ_BUTTON_DELAY_VALUE_RETURN	"[MP_SWDLY_"
#define UART_MEMB_READ_BUTTON_DELAY_VALUE	"[SM_READ_SWDLY]"
#define UART_MEMB_READ_CANCEL_DELAY_VALUE_RETURN	"[MP_CCDLY_"
#define UART_MEMB_READ_CANCEL_DELAY_VALUE	"[SM_READ_CCDLY]"
#define UART_MEMB_READ_MELODY_SELECT_RETURN "[MP_MELO_SEL"
#define UART_MEMB_READ_MELODY_SELECT		"[SM_MELO_RDSEL]"
#define UART_MEMB_READ_VOLUME_VALUE_RETURN	"[MP_VOLUME_"
#define UART_MEMB_READ_VOLUME_VALUE			"[SM_RD_VOLUME_]"
#define UART_MEMB_VOLUME_DOWN				"[SM_VOLU_DOWN_]"
#define UART_MEMB_VOLUME_OFF				"[SM_VOLU_OFF__]"
#define UART_MEMB_VOLUME_ON					"[SM_VOLU_ON___]"
#define UART_MEMB_VOLUME_UP					"[SM_VOLU_UP___]"
#define UART_MEMB_VOLUME_VALUE				"[SM_VOLUME_"
#define UART_MOTOR_A_END	 						"[SM_MOTA___END]"
#define UART_MOTOR_ALL_ORG_SET						"[SM_MOTALL_ORG"
#define UART_MOTOR_A_ORG	 						"[SM_MOTA___ORG]"
#define UART_MOTOR_A_STEP	 						"[SM_MOTA_"
#define UART_MOTOR_A_STOP	 						"[SM_MOTA__STOP]"
#define UART_MOTOR_A_TEST								"[SM_MOTA_VOL"
#define UART_MOTOR_C_END	 						"[SM_MOTC___END]"
#define UART_MOTOR_CNS_END	 						"[SM_MOTCN__END]"
#define UART_MOTOR_CNS_ORG	 						"[SM_MOTCN__ORG]"
#define UART_MOTOR_CNS_STEP	 						"[SM_MOTCN"
#define UART_MOTOR_CNS_STOP	 						"[SM_MOTCN_STOP]"
#define UART_MOTOR_CNS_TEST								"[SM_MOTN_VOL"
#define UART_MOTOR_C_ORG	 						"[SM_MOTC___ORG]"
#define UART_MOTOR_C_STEP	 						"[SM_MOTC_"
#define UART_MOTOR_C_STOP	 						"[SM_MOTC__STOP]"
#define UART_MOTOR_C_TEST								"[SM_MOTC_VOL"
#define UART_MOTOR_CWE_END	 						"[SM_MOTCW__END]"
#define UART_MOTOR_CWE_ORG	 						"[SM_MOTCW__ORG]"
#define UART_MOTOR_CWE_STEP	 						"[SM_MOTCW"
#define UART_MOTOR_CWE_STOP	 						"[SM_MOTCW_STOP]"
#define UART_MOTOR_CWE_TEST								"[SM_MOTW_VOL"
#define UART_MOTOR_ORG_TEST							"[SM_MOT__ORG_"
#define UART_MOTOR_READ_DATA						"[SM_READ_MOTOR]"
#define UART_MOTOR_R_END	 						"[SM_MOTR___END]"
#define UART_MOTOR_R_ORG	 						"[SM_MOTR___ORG]"
#define UART_MOTOR_R_STEP	 						"[SM_MOTR_"
#define UART_MOTOR_R_STOP	 						"[SM_MOTR__STOP]"
#define UART_MOTOR_R_TEST								"[SM_MOTR_VOL"
#define UART_MOTOR_SELECTION						"[SM_MOT_SEL_"
#define UART_MOTOR_S_END	 						"[SM_MOTS___END]"
#define UART_MOTOR_SEND_SPI_DATA					"[SM_SEND_MOTOR]"
#define UART_MOTOR_S_ORG	 						"[SM_MOTS___ORG]"
#define UART_MOTOR_SPI_DATA							"[SM_SPI0x"
#define UART_MOTOR_S_STEP	 						"[SM_MOTS_"
#define UART_MOTOR_S_STOP	 						"[SM_MOTS__STOP]"
#define UART_MOTOR_S_TEST								"[SM_MOTS_VOL"
#define UART_MOTOR_T_CHILD_END	 					"[SM_MOTT_CHEND]"
#define UART_MOTOR_T_END	 						"[SM_MOTT___END]"
#define UART_MOTOR_T_ORG	 						"[SM_MOTT___ORG]"
#define UART_MOTOR_T_STEP	 						"[SM_MOTT_"
#define UART_MOTOR_T_STOP	 						"[SM_MOTT__STOP]"
#define UART_MOTOR_T_TEST								"[SM_MOTT_VOL"
#define UART_MOTOR_V_END	 						"[SM_MOTV___END]"
#define UART_MOTOR_V_MOVE_TEST							"[SM_MOTVM"
#define UART_MOTOR_V_ORG	 						"[SM_MOTV___ORG]"
#define UART_MOTOR_V_STEP	 						"[SM_MOTV_"
#define UART_MOTOR_V_STOP	 						"[SM_MOTV__STOP]"
#define UART_MOTOR_V_TEST								"[SM_MOTV_VOL"
#define UART_ROM_MODE_CONSOLE 				"[SM_ROMM_CONSO]"
#define UART_ROM_MODE_PROTOCOL 				"[SM_ROMM_PROTO]"
#define UART_TUBE_VERSION_CHECK 			"[SM_CANT_VERC_]"

/* EEPROM handler: extracted from serial.c lines 7363-8095 */
#ifdef USE_I2C_EEPROM
char UART_SetRomParam(void)
{
    UART_MsgTypedef msg;
    char ret = RESET;

	while(MSG_QueueCnt(&UART_Queue))
    {
        msg = MSG_Dequeue(&UART_Queue);

        if (strstr(msg.Data, UART_ROM_ERASE_ALL))
        {
            EEPRom_EraseAll();
        }
        else if (strstr(msg.Data, UART_ROM_ERASE_ADDR))
        {
            long ltemp = strtol(&(msg.Data[12]), NULL, 16);
            
            if ( (ltemp >= kEpStartAddr) && (ltemp <= kEpEndAddr) ) {
                EEPRom_EraseAddrWord((char)ltemp); // 2byte erase
            } else {
                printUart(DBG_MSG_PC, "ERROR : Invalid address(%d)", ltemp);
            }
        }		
        else if (strstr(msg.Data, UART_ROM_APPLY))
        {
            EEPRom_LoadSysInfo();
			EEPRom_Load_Align_Offset();
        }
        else if (strstr(msg.Data, UART_ROM_READ_ALL))
        {
            EEPRom_ReadAll();
        }
        else if (strstr(msg.Data, UART_ROM_READ_ADDR))
        {
            long ltemp = strtol(&(msg.Data[12]), NULL, 16);

            if ( (ltemp >= kEpStartAddr) && (ltemp <= kEpEndAddr) ) {
                printUart(DBG_MSG_PC, "Addr(0x%02x) = 0x%04x", ltemp, EEPRom_I2C_Read_Word((char)ltemp)); // 2byte erase
            } else {
                printUart(DBG_MSG_PC, "ERROR : Invalid address(%d)", ltemp);
            }
        }
        else if (strstr(msg.Data, UART_ROM_WRITE_ADDR))
        {
            long ltemp = strtol(&(msg.Data[12]), NULL, 16);

            if ( (ltemp >= kEpStartAddr) && (ltemp <= kEpEndAddr) ) {
                EEPRom_SetAddress((char)ltemp);
            } else {
                printUart(DBG_MSG_PC, "ERROR : Invalid address(%d)", ltemp);
            }
        }
        else if (strstr(msg.Data, UART_ROM_WRITE_DATA))
        {
            unsigned int ntemp = atoi(&(msg.Data[9]));

            if (ntemp <= SHORT_TYPE_MAX_HEX) {
                EEPRom_SetWordData((unsigned short)ntemp);
            } else {
                printUart(DBG_MSG_PC, "ERROR : Invalid data(%d)", ntemp);
            }
        }
        else if (strstr(msg.Data, UART_ROM_WRITE_SET))
        {
            EEPRom_StartWriteCmd();
        }        
        else if (strstr(msg.Data, UART_ROM_READ_BOARD))
        {
            sysInfo.board_id = EEPRom_I2C_Read_Word(ROM_BOARD_ID_ADDR);
            if (sysInfo.board_id == 0xffff) {
                sysInfo.board_id = BOARD_REVISION_2_0;
            }
            
            printUart(DBG_MSG_PC, "Board : %d.%d", (sysInfo.board_id / 100), (sysInfo.board_id % 100));
        }
        else if (strstr(msg.Data, UART_ROM_WRITE_BOARD))
        {
            unsigned short nVers = atoi(&(msg.Data[10]));

            EEPRom_I2C_Write_Word(ROM_BOARD_ID_ADDR, nVers);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());

            printUart(DBG_MSG_PC, "Addr(0x%02x) = 0x%04x", 
                    ROM_BOARD_ID_ADDR, EEPRom_I2C_Read_Word(ROM_BOARD_ID_ADDR));    
        }
        else if (strstr(msg.Data, UART_ROM_ERASE_BOARD))
        {
            EEPRom_EraseAddrWord(ROM_BOARD_ID_ADDR);
        }
        else if (strstr(msg.Data, UART_ROM_READ_MODEL))
        {
            char str[8] = {0,};
            sysInfo.model_id = (modelNameId_t)EEPRom_I2C_Read_Byte(ROM_MODEL_ID_ADDR);

            if ( (sysInfo.model_id == 0xff) || (sysInfo.model_id == MODEL_T2_CS) )
            {
        		sprintf(str, "%s", "CS");
            }
			else if (sysInfo.model_id == MODEL_T2_C)
            {
        		sprintf(str, "%s", "C");
            }/*
            else if (sysInfo.model_id == MODEL_T1_CO)
            {
        		sprintf(str, "%s", "CO");
            }
            else if (sysInfo.model_id == MODEL_T1_S)
            {
        		sprintf(str, "%s", "S");
            }
            else if (sysInfo.model_id == MODEL_T1_CS_SEN1)
            {
        		sprintf(str, "%s", "CS(SEN1)");
            } */
            else
            {
                printUart(DBG_MSG_PC, "Undefined model_id : 0x%02x", sysInfo.model_id);
                return RESET;
            }

            printUart(DBG_MSG_PC, "Model Name : T2-%s SCM", str);
        }
        else if (strstr(msg.Data, UART_ROM_WRITE_MODEL))
        {
            char str[8] = {0,};                
            modelNameId_t model;

            if (strstr(&(msg.Data[10]), "T2CS"))
            {
        		sprintf(str, "%s", "CS");
                model = MODEL_T2_CS;
            }                
            else if (strstr(&(msg.Data[10]), "T2C_"))
            {
        		sprintf(str, "%s", "C");
                model = MODEL_T2_C;
            }/*
            else if (strstr(&(msg.Data[10]), "T2CO"))
            {
        		sprintf(str, "%s", "CO");
                model = MODEL_T1_CO;
            }
            else if (strstr(&(msg.Data[10]), "T2S_"))
            {
        		sprintf(str, "%s", "S");
                model = MODEL_T1_S;
            }
            else if (strstr(&(msg.Data[10]), "SEN1"))
            {
        		sprintf(str, "%s", "CS(SEN1)");
                model = MODEL_T1_CS_SEN1;
            }  */              
            else
            {
                printUart(DBG_MSG_PC, "Undefined model name : %s", msg.Data);
                return RESET;
            }

            printUart(DBG_MSG_PC, "model name set to 'T2-%s'", str);

            EEPRom_I2C_Write_Byte(ROM_MODEL_ID_ADDR, (char)model);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());            
        }
        else if (strstr(msg.Data, UART_ROM_ERASE_MODEL))
        {
            EEPRom_EraseAddrByte(ROM_MODEL_ID_ADDR);
        }
        else if (strstr(msg.Data, UART_ROM_READ_INITDIR_MOT_CC))
        {
            char str[8] = {0,};
            sysInfo.initDir_MotCC = (motorInitDir_t)EEPRom_I2C_Read_Byte(ROM_CCAXIS_INIT_DIR_ADDR);

            if (sysInfo.initDir_MotCC == INIT_DIR_REVERSE) {
        		sprintf(str, "%s", "Reverse");
            } else {
        		sprintf(str, "%s", "Forward");
            }

            printUart(DBG_MSG_PC, "Init_Dir for Motor_CC : %s", str);
        }
        else if (strstr(msg.Data, UART_ROM_WRITE_INITDIR_MOT_CC))
        {
            char str[8] = {0,};
            motorInitDir_t initDir;
            
            if (strstr(&(msg.Data[13]), "R")) {
        		sprintf(str, "%s", "Reverse");
                initDir = INIT_DIR_REVERSE;
            } else {
        		sprintf(str, "%s", "Forward");
                initDir = INIT_DIR_FORWARD;
            }

    		printUart(DBG_MSG_PC, "Init_Dir for Motor_CC set to '%s'", str);

            EEPRom_I2C_Write_Byte(ROM_CCAXIS_INIT_DIR_ADDR, (char)initDir);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());   

            sysInfo.initDir_MotCC = initDir;
        }
        else if (strstr(msg.Data, UART_ROM_ERASE_INITDIR_MOT_CC))
        {
            EEPRom_EraseAddrByte(ROM_CCAXIS_INIT_DIR_ADDR);
        }
        else if (strstr(msg.Data, UART_COLI_VERSION_CHECK))
        {
            if (!CAN_Collimator_SendMessage(CMD_VERSION_CHECK, 0, 0, 3000, 0))
            {
                printUart(DBG_MSG_PC, "Version Check Error");
            }
        }
        else if (strstr(msg.Data, UART_COLI_BUILD_CHECK))
        {
            if (!CAN_Collimator_SendMessage(CMD_BUILD_CHECK, 0, 0, 3000, 0))
            {
                printUart(DBG_MSG_PC, "Build Check Error");
            }
        }
        else if (strstr(msg.Data, UART_TUBE_VERSION_CHECK))
        {
			if (!CAN_SendMessage(CAN_TUBE_VER_CHECK, 0, 0, 3000, 0))
            {
                printUart(DBG_MSG_PC, "Version Check Error");
			}
        }
        else if (strstr(msg.Data, UART_ROM_READ_MAP))
        {
            EEPRom_ShowMap();
        }
        else if (strstr(msg.Data, UART_CMD_SYSTEM_SW_RESET))
        {
            printUart(DBG_MSG_PC, "\r\n\r\n\r\nNow will be reset...!!!\r\n\r\n");
            RestartSystem();
        }
		else if(strstr(msg.Data,UART_ROM_READ_OFFSET))	// Read all offset value
		{
			EEPRom_ShowOffset();
		}
		else if(strstr(msg.Data, UART_ROM_READ_COLLIMATOR_OFFSET))	// Read all Collimator offset value
		{
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_PANO_T, 0, 0, 0, 0))
            {
                return RESET;
            }			
		    IntTimer_Delay(200);
		    while(!IntTimer_GetStatus());
			if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_PANO_B, 0, 0, 0, 0))
            {
                return RESET;
            }
			IntTimer_Delay(200);
		    while(!IntTimer_GetStatus());
			if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_PANO_L, 0, 0, 0, 0))
            {
                return RESET;
            }
			IntTimer_Delay(200);
		    while(!IntTimer_GetStatus());
			if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_PANO_R, 0, 0, 0, 0))
            {
                return RESET;
            }
			IntTimer_Delay(200);
		    while(!IntTimer_GetStatus());
			if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CT_T, 0, 0, 0, 0))
            {
                return RESET;
            }
			IntTimer_Delay(200);
		    while(!IntTimer_GetStatus());
			if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CT_B, 0, 0, 0, 0))
            {
                return RESET;
            }
			IntTimer_Delay(200);
		    while(!IntTimer_GetStatus());
			if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CT_L, 0, 0, 0, 0))
            {
                return RESET;
            }
			IntTimer_Delay(200);
		    while(!IntTimer_GetStatus());
			if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CT_R, 0, 0, 0, 0))
            {
                return RESET;
            }
			IntTimer_Delay(200);
		    while(!IntTimer_GetStatus());
			if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CEPHSCAN_T, 0, 0, 0, 0))
            {
                return RESET;
            }
			IntTimer_Delay(200);
		    while(!IntTimer_GetStatus());
			if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CEPHSCAN_B, 0, 0, 0, 0))
            {
                return RESET;
            }
			IntTimer_Delay(200);
		    while(!IntTimer_GetStatus());
			if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CEPHSCAN_L, 0, 0, 0, 0))
            {
                return RESET;
            }
			IntTimer_Delay(200);
		    while(!IntTimer_GetStatus());
			if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CEPHSCAN_R, 0, 0, 0, 0))
            {
                return RESET;
            }
			IntTimer_Delay(200);
		    while(!IntTimer_GetStatus());
			if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_HBEAM_LASER, 0, 0, 0, 0))
			{
				return RESET;
			}	
		}
		else if (strstr(msg.Data, UART_ALIGN_PANO_COLLIMATOR_TOP))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_PANO_T;
        }    
        else if (strstr(msg.Data, UART_ALIGN_PANO_COLLIMATOR_BOTTOM))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_PANO_B;            
        }    
        else if (strstr(msg.Data, UART_ALIGN_PANO_COLLIMATOR_LEFT))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_PANO_L;            
        }    
        else if (strstr(msg.Data, UART_ALIGN_PANO_COLLIMATOR_RIGHT))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_PANO_R;            
        }    
		else if (strstr(msg.Data, UART_ALIGN_CT_COLI_TOP))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_CT_T;
        }
        else if (strstr(msg.Data, UART_ALIGN_CT_COLI_BOTTOM))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_CT_B;
        }
        else if (strstr(msg.Data, UART_ALIGN_CT_COLI_LEFT))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_CT_L;
        }
        else if (strstr(msg.Data, UART_ALIGN_CT_COLI_RIGHT))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_CT_R;
        }		
        else if (strstr(msg.Data, UART_ALIGN_SCAN_COLI_TOP))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_CEPHSCAN_T;
        }
        else if (strstr(msg.Data, UART_ALIGN_SCAN_COLI_BOTTOM))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_CEPHSCAN_B;
        }
        else if (strstr(msg.Data, UART_ALIGN_SCAN_COLI_LEFT))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_CEPHSCAN_L;
        }
        else if (strstr(msg.Data, UART_ALIGN_SCAN_COLI_RIGHT))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_CEPHSCAN_R;
        }
		else if (strstr(msg.Data, UART_ALIGN_BEAM_COLI))
		{
			CalibrationModeParam.ColCmd = CMD_OFFSET_HBEAM_LASER;       
		}
        else if (strstr(msg.Data, UART_ALIGN_PANO_COLLIMATOR_STEP_INC)) // Collimator Sensor Align 문서 ?�의 - 방향
        {
            if (CalibrationModeParam.ColCmd != 0)
            {
                uint32_t nStep = atoi(&(msg.Data[9]));

// Step ???�동거리 = (Step) x 100 /768 * 2.44
// example : 100 step ( = 100 / 768 * 2.44mm = 0.3177)
                if (!CAN_Collimator_SendMessage(CalibrationModeParam.ColCmd, nStep, 1, 1000, 2))
                {
                    return RESET;
                }
                //IntTimer_Delay(500);
                //while(!IntTimer_GetStatus());
            }
        }    
        else if (strstr(msg.Data, UART_ALIGN_PANO_COLLIMATOR_STEP_DEC)) // Collimator Sensor Align 문서 ?�의 + 방향
        {
            if (CalibrationModeParam.ColCmd != 0)
            {
                uint32_t nStep = atoi(&(msg.Data[9]));
                if (!CAN_Collimator_SendMessage(CalibrationModeParam.ColCmd, nStep, 0, 1000, 2))
                {
                    return RESET;
                }
                //ntTimer_Delay(500);
                //while(!IntTimer_GetStatus());    
            }
        }
		else if (strstr(msg.Data, UART_TUBE_ERROR_ON_OFF))
		{
			char str[40] = {0,};
			if (strchr(&(msg.Data[13]), 'N'))
			{
				sprintf(str, "KV_MA_ERROR_ENABLE");
				UART_SendMessage(DBG_MSG_PC, str);
				CAN_SendMessage(CAN_TUBE_KV_MA_ERROR_DISABLE, 1, 0, 1000, 2);
			}
			else
			{
				sprintf(str, "KV_MA_ERROR_DISABLE");
				UART_SendMessage(DBG_MSG_PC, str);
				CAN_SendMessage(CAN_TUBE_KV_MA_ERROR_DISABLE, 0, 0, 1000, 2);
			}
		}
		else if (strstr(msg.Data, UART_TUBE_TOTAL_ERROR_ON_OFF))
		{
			char str[40] = {0,};
			if (strchr(&(msg.Data[13]), 'N'))
			{
				sprintf(str, "TOTAL_ERROR_ENABLE");
				UART_SendMessage(DBG_MSG_PC, str);
				CAN_SendMessage(CAN_TUBE_TOTAL_ERROR_CHECK, 1, 0, 1000, 2);
			}
			else
			{
				sprintf(str, "TOTAL_ERROR_DISABLE");
				UART_SendMessage(DBG_MSG_PC, str);
				CAN_SendMessage(CAN_TUBE_TOTAL_ERROR_CHECK, 0, 0, 1000, 2);
			}
		}// 200806 HWAN ADD  
		else if (strstr(msg.Data, UART_TUBE_ERROR_VALUE_CHECK))
		{
			char str[40] = {0,};
			uint8_t i=0;
			
			sprintf(str, "TANK VALUE CHECK");
			UART_SendMessage(DBG_MSG_PC, str);

			i=6;
			
			for(;i<10;i++)
			{
				CAN_SendMessage(CAN_TUBE_ERROR_VALUE_CHECK, i, 0, 1000, 2);
				IntTimer_Delay(100);
	   			while(!IntTimer_GetStatus());
			}	
		}
		else if (strstr(msg.Data, UART_TUBE_TEMP_SETTING))
		{			
			char str[40] = {0,};
			uint8_t nOffset = 0;
			nOffset = atoi(&(msg.Data[12]));
			
			sprintf(str, "Temperature set %d", nOffset);
			UART_SendMessage(DBG_MSG_PC, str);
			
			CAN_SendMessage(CAN_TUBE_TANK_TEMP_SET, nOffset, 0, 1000, 2);
		}
		else if (strstr(msg.Data, UART_TUBE_TEMP_SETTING_READ))
		{
			char str[40] = {0,};

			sprintf(str, "Temperature set Read");
			UART_SendMessage(DBG_MSG_PC, str);

			CAN_SendMessage(CAN_TUBE_TANK_TEMP_SET_READ, 0, 0, 1000, 2);
		}
		else if(strstr(msg.Data, UART_TUBE_FAN_READ_ON_OFF_FLAG))
		{
			UART_SendMessage(DBG_MSG_PC, "FAN FLAG READ");
			CAN_SendMessage(CAN_TUBE_FAN_ACTIVE_READ_FLAG, 1, 0, 1000, 2);			
		}
		else if (strstr(msg.Data, UART_TUBE_FAN_ON_OFF))
		{
			char str[40] = {0,};
			if (strchr(&(msg.Data[13]), 'N'))
			{
				sprintf(str, "FAN ON ACTIVE");
				UART_SendMessage(DBG_MSG_PC, str);
				CAN_SendMessage(CAN_TUBE_FAN_ACTIVE_FLAG, 1, 0, 1000, 2);
			}
			else
			{
				sprintf(str, "FAN OFF DEACTIVE");
				UART_SendMessage(DBG_MSG_PC, str);
				CAN_SendMessage(CAN_TUBE_FAN_ACTIVE_FLAG, 0, 0, 1000, 2);
			}
		}
		else if (strstr(msg.Data, UART_TUBE_FAN_TEMP_SETTING))
		{			
			char str[40] = {0,};
			uint8_t nOffset = 0;
			nOffset = atoi(&(msg.Data[12]));
			
			sprintf(str, "FAN Temperature set %d", nOffset);
			UART_SendMessage(DBG_MSG_PC, str);
			
			CAN_SendMessage(CAN_TUBE_FAN_ACTIVE_TEMP_SET, nOffset, 0, 1000, 2);
		}
		else if (strstr(msg.Data, UART_TUBE_FAN_TEMP_SETTING_READ))
		{
			char str[40] = {0,};

			sprintf(str, "FAN Temperature set Read");
			UART_SendMessage(DBG_MSG_PC, str);

			CAN_SendMessage(CAN_TUBE_FAN_ACTIVE_TEMP_READ, 0, 0, 1000, 2);
		}	
		else if (strstr(msg.Data, UART_GET_TUBE_TEMP))
		{
			CAN_SendMessage(CAN_TUBE_TANK_TEMP_READ, 0, 0, 1000, 2);
		}// 201117 HWAN Membrane Add		
		else if(strstr(msg.Data, UART_MEMB_VOLUME_ON))
		{
#ifdef USE_TABLET_PC
			UART_SendMessage(DBG_MSG_TABLET, "[SP_VOLU_ON___]");
#endif // USE_TABLET_PC
			IntTimer_Delay(200);		
			while(!IntTimer_GetStatus());
		}
		else if(strstr(msg.Data, UART_MEMB_VOLUME_OFF))
		{
#ifdef USE_TABLET_PC
			UART_SendMessage(DBG_MSG_TABLET, "[SP_VOLU_OFF__]");
#endif // USE_TABLET_PC
			IntTimer_Delay(200);		
			while(!IntTimer_GetStatus());
		}
		else if(strstr(msg.Data, UART_MEMB_VOLUME_UP))
		{
#ifdef USE_TABLET_PC
			UART_SendMessage(DBG_MSG_TABLET, "[SP_VOLU_UP___]");
#endif // USE_TABLET_PC
			IntTimer_Delay(200);		
			while(!IntTimer_GetStatus());
		}
		else if(strstr(msg.Data, UART_MEMB_VOLUME_DOWN))
		{
#ifdef USE_TABLET_PC
			UART_SendMessage(DBG_MSG_TABLET, "[SP_VOLU_DOWN_]");
#endif // USE_TABLET_PC
			IntTimer_Delay(200);		
			while(!IntTimer_GetStatus());
		}
		else if(strstr(msg.Data, UART_MEMB_READ_VOLUME_VALUE))
		{
			printUart(DBG_MSG_PC, "Current Volume Value = 0x%04x", 
						EEPRom_I2C_Read_Word(ROM_MEMBRANE_SOUND_VALUE_ADDR));
#ifdef USE_TABLET_PC
			printUart(DBG_MSG_TABLET, UART_MEMB_READ_VOLUME_VALUE);
#endif // USE_TABLET_PC
			
		}		
		else if(strstr(msg.Data, UART_MEMB_READ_VOLUME_VALUE_RETURN))
		{
			msg.Data[1]='S';
			printUart(DBG_MSG_PC, msg.Data);
		}
		else if(strstr(msg.Data, UART_MEMB_READ_MELODY_SELECT))
		{
			printUart(DBG_MSG_PC, "Current Melody Selected = 0x%04x", EEPRom_I2C_Read_Word(ROM_MEMBRANE_SELECT_VALUE_ADDR));
#ifdef USE_TABLET_PC
			printUart(DBG_MSG_TABLET, UART_MEMB_READ_MELODY_SELECT);
#endif // USE_TABLET_PC

		}
		else if(strstr(msg.Data, UART_MEMB_READ_MELODY_SELECT_RETURN))
		{
			msg.Data[1]='S';
			printUart(DBG_MSG_PC, msg.Data);
		}		
		else if(strstr(msg.Data, UART_MEMB_VOLUME_VALUE))
		{
			uint16_t Number = atoi(&(msg.Data[11]));
			char string[64] = {0,};

			if(Number >=0 || Number <255)
			{
				sprintf(string, "[SP_VOLUME_");
#ifdef USE_TABLET_PC
				printUart(DBG_MSG_TABLET, "%s%03d]",string,Number);
#endif // USE_TABLET_PC
				printUart(DBG_MSG_PC, "%s%03d]",string,Number);
				EEPRom_I2C_Write_Word(ROM_MEMBRANE_SOUND_VALUE_ADDR, Number);
				IntTimer_Delay(1000);
				while(!IntTimer_GetStatus());

				printUart(DBG_MSG_PC, "Addr(0x%02x) = 0x%04x", 
						ROM_MEMBRANE_SOUND_VALUE_ADDR, EEPRom_I2C_Read_Word(ROM_MEMBRANE_SOUND_VALUE_ADDR));	
			}
			else
			{
				printUart(DBG_MSG_PC, "Invalid Value");
			}
		}
		else if(strstr(msg.Data, UART_MEMB_MELODY_SELECT))
		{
			uint16_t Number = atoi(&(msg.Data[12]));
			char string[64] = {0,};

			if(Number > 0 || Number <100)
			{
				sprintf(string, "[SP_MELO_SEL");
#ifdef USE_TABLET_PC
				printUart(DBG_MSG_TABLET, "%s%02d]",string,Number);
#endif // USE_TABLET_PC
				printUart(DBG_MSG_PC, "%s%02d]",string,Number);

				EEPRom_I2C_Write_Word(ROM_MEMBRANE_SELECT_VALUE_ADDR, Number);
				IntTimer_Delay(1000);
				while(!IntTimer_GetStatus());

				printUart(DBG_MSG_PC, "Addr(0x%02x) = 0x%04x", 
						ROM_MEMBRANE_SELECT_VALUE_ADDR, EEPRom_I2C_Read_Word(ROM_MEMBRANE_SELECT_VALUE_ADDR));	
			}
			else
			{
				printUart(DBG_MSG_PC, "Invalid Value");
			}
		}
		else if (strstr(msg.Data, UART_MEMBRANE_SOUND_TEST))
		{	
#ifdef USE_TABLET_PC
			UART_SendMessage(DBG_MSG_TABLET, UART_MEMBRANE_SOUND_TEST);
#endif /* USE_TABLET_PC */
			UART_SendMessage(DBG_MSG_PC, UART_MEMBRANE_SOUND_TEST);
		}
		else if(strstr(msg.Data, UART_MEMB_READ_CANCEL_DELAY_VALUE))
		{
			printUart(DBG_MSG_PC, "Saved Cancel Delay Value = 0x%04x", 
						EEPRom_I2C_Read_Word(ROM_MEMBRANE_CANCEL_DELAY_ADDR));
			printUart(DBG_MSG_PC, "Current Cancel Delay Value = %d", 
						sysInfo.nMembrane_Cancel_Delay);
#ifdef USE_TABLET_PC
			printUart(DBG_MSG_TABLET, UART_MEMB_READ_CANCEL_DELAY_VALUE);
#endif // USE_TABLET_PC
			
		}		
		else if(strstr(msg.Data, UART_MEMB_READ_CANCEL_DELAY_VALUE_RETURN))
		{
			msg.Data[1]='S';
			printUart(DBG_MSG_PC, msg.Data);
		}
		else if(strstr(msg.Data, UART_MEMB_CANCEL_DELAY_VALUE))
		{
			uint16_t Number = atoi(&(msg.Data[10]));
			char string[64] = {0,};

			if(Number >=0 || Number <10000)
			{
				sprintf(string, "[SP_CCDLY_");
#ifdef USE_TABLET_PC
				printUart(DBG_MSG_TABLET, "%s%04d]",string,Number);
#endif // USE_TABLET_PC
				printUart(DBG_MSG_PC, "Current Cancel Delay Value = %d", Number);
				EEPRom_I2C_Write_Word(ROM_MEMBRANE_CANCEL_DELAY_ADDR, Number);
				IntTimer_Delay(1000);
				while(!IntTimer_GetStatus());

				printUart(DBG_MSG_PC, "Addr(0x%02x) = 0x%04x", 
						ROM_MEMBRANE_CANCEL_DELAY_ADDR, EEPRom_I2C_Read_Word(ROM_MEMBRANE_CANCEL_DELAY_ADDR));	
#ifdef USE_TABLET_PC
				printUart(DBG_MSG_TABLET, UART_MEMB_READ_CANCEL_DELAY_VALUE);
#endif // USE_TABLET_PC
			}
			else
			{
				printUart(DBG_MSG_PC, "Invalid Value");
			}
		}	
		else if(strstr(msg.Data, UART_MEMB_READ_BUTTON_DELAY_VALUE))
		{
			printUart(DBG_MSG_PC, "Saved Button Delay Value = 0x%04x", 
						EEPRom_I2C_Read_Word(ROM_MEMBRANE_BUTTON_DELAY_ADDR));
			printUart(DBG_MSG_PC, "Current Button Delay Value = %d", 
						sysInfo.nMembrane_Button_Delay);
#ifdef USE_TABLET_PC
			printUart(DBG_MSG_TABLET, UART_MEMB_READ_BUTTON_DELAY_VALUE);
#endif // USE_TABLET_PC
			
		}		
		else if(strstr(msg.Data, UART_MEMB_READ_BUTTON_DELAY_VALUE_RETURN))
		{
			msg.Data[1]='S';
			printUart(DBG_MSG_PC, msg.Data);
		}
		else if(strstr(msg.Data, UART_MEMB_BUTTON_DELAY_VALUE))
		{
			uint16_t Number = atoi(&(msg.Data[10]));
			char string[64] = {0,};

			if(Number >=0 || Number <10000)
			{
				sprintf(string, "[SP_SWDLY_");
#ifdef USE_TABLET_PC
				printUart(DBG_MSG_TABLET, "%s%04d]",string,Number);
#endif // USE_TABLET_PC
				printUart(DBG_MSG_PC, "Current Button Delay Value = %d", Number);
				EEPRom_I2C_Write_Word(ROM_MEMBRANE_BUTTON_DELAY_ADDR, Number);
				IntTimer_Delay(1000);
				while(!IntTimer_GetStatus());

				printUart(DBG_MSG_PC, "Addr(0x%02x) = 0x%04x", 
						ROM_MEMBRANE_BUTTON_DELAY_ADDR, EEPRom_I2C_Read_Word(ROM_MEMBRANE_BUTTON_DELAY_ADDR));	
#ifdef USE_TABLET_PC
				printUart(DBG_MSG_TABLET, UART_MEMB_READ_BUTTON_DELAY_VALUE);
#endif // USE_TABLET_PC
			}
			else
			{
				printUart(DBG_MSG_PC, "Invalid Value");
			}
		}
		else
        {
            printUart(DBG_MSG_PC, "%s : %s", ERR_CODE_UART_MSG_ERROR, msg.Data);
            return RESET;
        }
    }

    return ret;
}
#endif /* USE_I2C_EEPROM */

/**
#endif /* USE_I2C_EEPROM */

/* Diagnostic handler: extracted from serial.c lines 8096-9506 */
char UART_SetDiagParam(void)
{
	UART_MsgTypedef msg;
	char ret = RESET;

	while(MSG_QueueCnt(&UART_Queue))
	{
		msg = MSG_Dequeue(&UART_Queue);
		if (strstr(msg.Data, UART_GET_TUBE_TEMP))
		{
			CAN_SendMessage(CAN_TUBE_TANK_TEMP_READ, 0, 0, 1000, 2);
		}
		else if (strstr(msg.Data, UART_MODE_EXIT))
		{
			CurCaptureMode = CAPTURE_CANCEL;
			ret = SET;
		}			
		else if(strstr(msg.Data, UART_MOTOR_ALL_ORG_SET))
		{
			Motor_MoveALL_ORG();
		}		
		else if (strstr(msg.Data,UART_GET_ALL_LOCKED_VERSION))
		{
			printUart(DBG_MSG_PC, "\r\n");
		    printUart(DBG_MSG_PC, "=========== Firmware Information ===========");

		    showBoardVersion();
			showLockedVersion(RESET);
			if (!CAN_SendMessage(CAN_TUBE_VER_CHECK, 0, 0, 5000, 1))
        		return RESET;
			
			if (!CAN_Collimator_SendMessage(CMD_BUILD_CHECK, 0, 0, 5000, 1))
        		return RESET;			
			
   			IntTimer_Delay(1500);
			while(!UART_MEMB_VER_CHECK(FALSE));

			printUart(DBG_MSG_PC, "======================================");
		}		
		else if (strstr(msg.Data,UART_GET_ALL_RELEASED_VERSION))
		{
			printUart(DBG_MSG_PC, "\r\n");
		    printUart(DBG_MSG_PC, "=========== Firmware Information ===========");

		    showBoardVersion();
			showReleasedVersion(RESET);
			if (!CAN_SendMessage(CAN_TUBE_VER_CHECK, 0, 0, 5000, 1))
        		return RESET;
			if (!CAN_SendMessage(CAN_Sub_Version_Check, 0, 0, 5000, 1))
			{
				printUart(DBG_MSG_PC, "CURRENT GENERATOR FW DOES NOT SUPPORT SUB VERSION");
			}
			if (!CAN_Collimator_SendMessage(CMD_BUILD_CHECK, 0, 0, 5000, 1))
        		return RESET;
			if (!CAN_Collimator_SendMessage(CMD_BUILD_SUB_CHECK, 0, 0, 5000, 1))
        	{
				printUart(DBG_MSG_PC, "CURRENT COLLIMATOR FW DOES NOT SUPPORT SUB VERSION");
			}
			
   			IntTimer_Delay(1500);
			while(!UART_MEMB_VER_CHECK(TRUE));

			printUart(DBG_MSG_PC, "======================================");
		}		
		else if (strstr(msg.Data, UART_COLUMN_UP))
        {
            Column_Control(COLUMN_UP);
        }
        else if (strstr(msg.Data, UART_COLUMN_DOWN))
        {
            Column_Control(COLUMN_DOWN);
        }
        else if (strstr(msg.Data, UART_COLUMN_STOP))
        {
            Column_Control(COLUMN_STOP);
        }
		else if (strstr(msg.Data, UART_GEN1_TEST))
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
			
			if (!CAN_SendMessage(CAN_TUBE_KV_SET, 50, 0, 1000, 2))
			{
				return RESET;
			}
			
			if (!CAN_SendMessage(CAN_TUBE_mA_SET, 50, 0, 1000, 2))
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
			
			UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_END__]");
		}
		else if (strstr(msg.Data, UART_GEN1_READY_ENABLE))
		{
			Tube_CtrlReady(TUBE_READY_ENABLE);
		}
		else if (strstr(msg.Data, UART_GEN1_READY_DISABLE))
		{
			Tube_CtrlReady(TUBE_READY_DISABLE);
		}
		else if (strstr(msg.Data, UART_GEN1_EXPOSE_ENABLE))
		{
			Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);	
		}
		else if (strstr(msg.Data, UART_GEN1_EXPOSE_DISABLE))
		{
			Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);	
		}
		else if (strstr(msg.Data, UART_CAN_TEST_TUBE_COM_CHECK))
		{
			if (!CAN_SendMessage(CAN_TUBE_COM_CHECK, CAN_TUBE_GEN2_COM, 0, 3000, 2))
			{
    			ret = RESET;
			}
		}
		else if (strstr(msg.Data, UART_CAN_TEST_COLI_COMM_CHECK))
        {
            if (!CAN_Collimator_SendMessage(CMD_COMM_CHECK, 0, 0, 3000, 2))
            {
                ret = RESET;
            }    
        }
		else if(strstr(msg.Data, UART_MOTOR_ORG_TEST))
		{
			if(strchr(&(msg.Data[13]), 'P'))
			{
				CurCaptureMode = CAPTURE_PANO;
			}
			else if(strchr(&(msg.Data[13]), 'C'))
			{
				CurCaptureMode = CAPTURE_CT;
			}
			else if(strchr(&(msg.Data[13]), 'S'))
			{
				CurCaptureMode = CAPTURE_SCAN;
			}
			else
			{
				UART_SendMessage(DBG_MSG_PC, "Wrong Message");
				return RESET;
			}

			if (!Motor_MoveInitPosition())
		    {
		    	UART_SendMessage(DBG_MSG_PC, "Motor Init Position Fail.");	
		    }

			CurCaptureMode = DIAGNOSTIC_MODE;
		}
		else if(strstr(msg.Data, UART_MOTOR_SELECTION))
		{
			char message[2] = {0,};
			message[0]=msg.Data[13];
			message[1]=msg.Data[14];
			if(strstr(message, "R_"))
			{
				DiagnosticModeParam.Motor=&Motor_R;
				UART_SendMessage(DBG_MSG_PC, "MOTOR R SELECTED");
			}
			else if(strstr(message, "V_"))
			{
				DiagnosticModeParam.Motor=&Motor_V;
				UART_SendMessage(DBG_MSG_PC, "MOTOR V SELECTED");
			}
			else if(strstr(message, "A_"))
			{
				DiagnosticModeParam.Motor=&Motor_A;
				UART_SendMessage(DBG_MSG_PC, "MOTOR A SELECTED");
			}
			else if(strstr(message, "C_"))
			{
				DiagnosticModeParam.Motor=&Motor_C;
				UART_SendMessage(DBG_MSG_PC, "MOTOR C SELECTED");
			}
			else if(strstr(message, "S_"))
			{
				DiagnosticModeParam.Motor=&Motor_S;
				UART_SendMessage(DBG_MSG_PC, "MOTOR S SELECTED");
			}
			else if(strstr(message, "T_"))
			{
				DiagnosticModeParam.Motor=&Motor_T;
				UART_SendMessage(DBG_MSG_PC, "MOTOR T SELECTED");
			}
			else if(strstr(message, "CN"))
			{
				DiagnosticModeParam.Motor=&Motor_CNS;
				UART_SendMessage(DBG_MSG_PC, "MOTOR CNS SELECTED");
			}
			else if(strstr(message, "CW"))
			{
				DiagnosticModeParam.Motor=&Motor_CWE;
				UART_SendMessage(DBG_MSG_PC, "MOTOR CWE SELECTED");
			}			
		}
		else if(strstr(msg.Data, UART_MOTOR_SPI_DATA))
		{
			long ltemp = strtol(&(msg.Data[9]), NULL, 16);
			if ( (ltemp >= 0x00000) && (ltemp <= 0xFFFFF) ) 
			{
				DiagnosticModeParam.Data=ltemp;
                printUart(DBG_MSG_PC, "data : 0x%05x , DiagData : 0x%05x", ltemp,DiagnosticModeParam.Data);             
            } 
			else 
			{
                printUart(DBG_MSG_PC, "ERROR : Invalid value(%d)", ltemp);
            }			
		}
		else if(strstr(msg.Data, UART_MOTOR_SEND_SPI_DATA))
		{
			TMC2660_SPI_Data_Sending(&DiagnosticModeParam.Motor,DiagnosticModeParam.Data);
		}
		else if(strstr(msg.Data, UART_MOTOR_READ_DATA))
		{
			TMC2660_SPI_ReadData(&DiagnosticModeParam.Motor);
		}
		else if (strstr(msg.Data, UART_MOTOR_R_ORG))		//	200203 HWAN ADD TEST MOTOR
		{
			printUart(DBG_MSG_PC, "MOTOR_R_ORG START");
			TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
			Motor_RunOrgCheck(&Motor_R);
			while(!Motor_GetStatus(&Motor_R, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_R_ORG END");
		}
		else if (strstr(msg.Data, UART_MOTOR_R_END))
		{
			printUart(DBG_MSG_PC, "MOTOR_R_END START");
			TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
			Motor_MoveAbsolutePosition(&Motor_R, 3000, 1000, 420.0 / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO), 1000);
			while(!Motor_GetStatus(&Motor_R, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_R_END END");
		}
		else if (strstr(msg.Data, UART_MOTOR_R_STOP))
		{
			printUart(DBG_MSG_PC, "MOTOR_R STATUS : %d",Motor_R.Status);
			Motor_R.nInitRunStep = 0;
			Motor_Stop(&Motor_R);
			printUart(DBG_MSG_PC, "MOTOR_R STATUS_STOP : %d",Motor_R.Status);
		}
		else if (strstr(msg.Data, UART_MOTOR_R_STEP))
		{
			int16_t nCnt=0;

			if (strchr(&(msg.Data[9]), 'M'))
			{
				nCnt -= atoi(&(msg.Data[10]));
			}
			else if(strchr(&(msg.Data[9]), 'P'))
			{
				nCnt += atoi(&(msg.Data[10]));
			}
			else
			{
				printUart(DBG_MSG_PC, "%s : %s", ERR_CODE_UART_MSG_ERROR, msg.Data);
				return RESET;
			}
			
			printUart(DBG_MSG_PC, "MOTOR_R : %d START",nCnt);
			TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
			Motor_MoveAbsolutePosition(&Motor_R, 3000, 1000, nCnt+Motor_R.CurrentStep, 1000);
			while(!Motor_GetStatus(&Motor_R, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_R : %d  END Current_Step : %d",nCnt,Motor_R.CurrentStep);
		}
		else if (strstr(msg.Data, UART_MOTOR_V_ORG))
		{
			printUart(DBG_MSG_PC, "MOTOR_V_ORG START");
			TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
			Motor_RunOrgCheck(&Motor_V);
			while(!Motor_GetStatus(&Motor_V, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_V_ORG END");
		}
		else if (strstr(msg.Data, UART_MOTOR_V_END))
		{
			printUart(DBG_MSG_PC, "MOTOR_V_END START");
			TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
			Motor_MoveAbsolutePosition(&Motor_V, 15000, 1000, 71000, 1000);
			while(!Motor_GetStatus(&Motor_V, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_V_END END");
		}
		else if (strstr(msg.Data, UART_MOTOR_V_STOP))
		{
			printUart(DBG_MSG_PC, "MOTOR_V STATUS : %d",Motor_V.Status);
			Motor_V.nInitRunStep = 0;
			Motor_Stop(&Motor_V);
			printUart(DBG_MSG_PC, "MOTOR_V STATUS_STOP : %d",Motor_V.Status);
		}
		else if (strstr(msg.Data, UART_MOTOR_V_STEP))
		{
			int16_t nCnt=0;

			if (strchr(&(msg.Data[9]), 'M'))
			{
				nCnt -= atoi(&(msg.Data[10]));
			}
			else if(strchr(&(msg.Data[9]), 'P'))
			{
				nCnt += atoi(&(msg.Data[10]));
			}
			else
			{
				printUart(DBG_MSG_PC, "%s : %s", ERR_CODE_UART_MSG_ERROR, msg.Data);
				return RESET;
			}
			
			printUart(DBG_MSG_PC, "MOTOR_V : %d START",nCnt);
			TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
			Motor_MoveAbsolutePosition(&Motor_V, 15000, 1000, nCnt+Motor_V.CurrentStep, 1000);
			while(!Motor_GetStatus(&Motor_V, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_V : %d  END Current_Step : %d",nCnt,Motor_V.CurrentStep);
		}
		else if (strstr(msg.Data, UART_MOTOR_A_ORG))
		{
			printUart(DBG_MSG_PC, "MOTOR_A_ORG START");
			TMC2660_SetCurrent(Motor_A.MotType, Motor_A.StartCurrent);
			Motor_RunOrgCheck(&Motor_A);
			while(!Motor_GetStatus(&Motor_A, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_A.MotType, Motor_A.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_A_ORG END");
		}
		else if (strstr(msg.Data, UART_MOTOR_A_END))
		{
			printUart(DBG_MSG_PC, "MOTOR_A_END START");
			TMC2660_SetCurrent(Motor_A.MotType, Motor_A.StartCurrent);
			Motor_MoveAbsolutePosition(&Motor_A, 12000, 1000, CT_AUTO_ANGLE / (360.0 / MOTOR_A_MICROSTEP / MOTOR_A_PULLEY_RATIO), 1000);
			while(!Motor_GetStatus(&Motor_A, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_A.MotType, Motor_A.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_A_END END");
		}
		else if (strstr(msg.Data, UART_MOTOR_A_STOP))
		{
			printUart(DBG_MSG_PC, "MOTOR_A STATUS : %d",Motor_A.Status);
			Motor_A.nInitRunStep = 0;
			Motor_Stop(&Motor_A);
			printUart(DBG_MSG_PC, "MOTOR_A STATUS_STOP : %d",Motor_A.Status);
		}
		else if (strstr(msg.Data, UART_MOTOR_A_STEP))
		{
			int16_t nCnt=0;

			if (strchr(&(msg.Data[9]), 'M'))
			{
				nCnt -= atoi(&(msg.Data[10]));
			}
			else if(strchr(&(msg.Data[9]), 'P'))
			{
				nCnt += atoi(&(msg.Data[10]));
			}
			else
			{
				printUart(DBG_MSG_PC, "%s : %s", ERR_CODE_UART_MSG_ERROR, msg.Data);
				return RESET;
			}
			
			printUart(DBG_MSG_PC, "MOTOR_A : %d START",nCnt);
			TMC2660_SetCurrent(Motor_A.MotType, Motor_A.StartCurrent);
			Motor_MoveAbsolutePosition(&Motor_A, 12000, 1000, nCnt+Motor_A.CurrentStep, 1000);
			while(!Motor_GetStatus(&Motor_A, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_A.MotType, Motor_A.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_A : %d  END Current_Step : %d",nCnt,Motor_A.CurrentStep);
		}
		else if (strstr(msg.Data, UART_MOTOR_C_ORG))
		{
			printUart(DBG_MSG_PC, "MOTOR_C_ORG START");
			TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StartCurrent);
			Motor_RunOrgCheck(&Motor_C);
			while(!Motor_GetStatus(&Motor_C, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_C_ORG END");
		}
		else if (strstr(msg.Data, UART_MOTOR_C_END))
		{
			printUart(DBG_MSG_PC, "MOTOR_C_END START");
			TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StartCurrent);
			Motor_MoveAbsolutePosition(&Motor_C, 3000, 1000, 28000, 1000);
			while(!Motor_GetStatus(&Motor_C, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_C_END END");
		}
		else if (strstr(msg.Data, UART_MOTOR_C_STOP))
		{
			printUart(DBG_MSG_PC, "MOTOR_C STATUS : %d",Motor_C.Status);
			Motor_C.nInitRunStep = 0;
			Motor_Stop(&Motor_C);
			printUart(DBG_MSG_PC, "MOTOR_C STATUS_STOP : %d",Motor_C.Status);
		}
		else if (strstr(msg.Data, UART_MOTOR_C_STEP))
		{
			int16_t nCnt=0;

			if (strchr(&(msg.Data[9]), 'M'))
			{
				nCnt -= atoi(&(msg.Data[10]));
			}
			else if(strchr(&(msg.Data[9]), 'P'))
			{
				nCnt += atoi(&(msg.Data[10]));
			}
			else
			{
				printUart(DBG_MSG_PC, "%s : %s", ERR_CODE_UART_MSG_ERROR, msg.Data);
				return RESET;
			}
			
			printUart(DBG_MSG_PC, "MOTOR_C : %d START",nCnt);
			TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StartCurrent);
			Motor_MoveAbsolutePosition(&Motor_C, 3000, 1000, nCnt+Motor_C.CurrentStep, 1000);
			while(!Motor_GetStatus(&Motor_C, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_C.MotType, Motor_C.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_C : %d  END Current_Step : %d",nCnt,Motor_C.CurrentStep);
		}
		else if (strstr(msg.Data, UART_MOTOR_S_ORG))
		{
			printUart(DBG_MSG_PC, "MOTOR_S_ORG START");
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
			Motor_RunOrgCheck(&Motor_S);
			while(!Motor_GetStatus(&Motor_S, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_S_ORG END");
		}
		else if (strstr(msg.Data, UART_MOTOR_S_END))
		{
			printUart(DBG_MSG_PC, "MOTOR_S_END START");
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
			Motor_MoveAbsolutePosition(&Motor_S, 3000, 1000, 28000, 1000);
			while(!Motor_GetStatus(&Motor_S, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_S_END END");
		}
		else if (strstr(msg.Data, UART_MOTOR_S_STOP))
		{
			printUart(DBG_MSG_PC, "MOTOR_S STATUS : %d",Motor_S.Status);
			Motor_S.nInitRunStep = 0;
			Motor_Stop(&Motor_S);
			printUart(DBG_MSG_PC, "MOTOR_S STATUS_STOP : %d",Motor_S.Status);
		}
		else if (strstr(msg.Data, UART_MOTOR_S_STEP))
		{
			int16_t nCnt=0;

			if (strchr(&(msg.Data[9]), 'M'))
			{
				nCnt -= atoi(&(msg.Data[10]));
			}
			else if(strchr(&(msg.Data[9]), 'P'))
			{
				nCnt += atoi(&(msg.Data[10]));
			}
			else
			{
				printUart(DBG_MSG_PC, "%s : %s", ERR_CODE_UART_MSG_ERROR, msg.Data);
				return RESET;
			}
			
			printUart(DBG_MSG_PC, "MOTOR_S : %d START",nCnt);
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
			Motor_MoveAbsolutePosition(&Motor_S, 3000, 1000, nCnt+Motor_S.CurrentStep, 1000);
			while(!Motor_GetStatus(&Motor_S, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_S : %d  END Current_Step : %d",nCnt,Motor_S.CurrentStep);
		}
		else if (strstr(msg.Data, UART_MOTOR_T_ORG))
		{
			printUart(DBG_MSG_PC, "MOTOR_T_ORG START");
			TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StartCurrent);
			Motor_RunOrgCheck(&Motor_T);
			while(!Motor_GetStatus(&Motor_T, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_T_ORG END");
		}
		else if (strstr(msg.Data, UART_MOTOR_T_END))
		{
			printUart(DBG_MSG_PC, "MOTOR_T_END START");
			TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StartCurrent);
			Motor_MoveAbsolutePosition(&Motor_T, 3000, 1000, 8000+ PanoParam.nTAxisOffset, 1000);
			while(!Motor_GetStatus(&Motor_T, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_T_END END");
		}
		else if (strstr(msg.Data, UART_MOTOR_T_CHILD_END))
		{
			printUart(DBG_MSG_PC, "MOTOR_T_END START");
			TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StartCurrent);
			Motor_MoveAbsolutePosition(&Motor_T, 3000, 1000, 8000 + PanoParam.nTAxisChildOffset, 1000);
			while(!Motor_GetStatus(&Motor_T, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_T_END END");
		}
		else if (strstr(msg.Data, UART_MOTOR_T_STOP))
		{
			printUart(DBG_MSG_PC, "MOTOR_T STATUS : %d",Motor_T.Status);
			Motor_T.nInitRunStep = 0;
			Motor_Stop(&Motor_T);
			printUart(DBG_MSG_PC, "MOTOR_T STATUS_STOP : %d",Motor_T.Status);
		}
		else if (strstr(msg.Data, UART_MOTOR_T_STEP))
		{
			int16_t nCnt=0;

			if (strchr(&(msg.Data[9]), 'M'))
			{
				nCnt -= atoi(&(msg.Data[10]));
			}
			else if(strchr(&(msg.Data[9]), 'P'))
			{
				nCnt += atoi(&(msg.Data[10]));
			}
			else
			{
				printUart(DBG_MSG_PC, "%s : %s", ERR_CODE_UART_MSG_ERROR, msg.Data);
				return RESET;
			}
			
			printUart(DBG_MSG_PC, "MOTOR_T : %d START",nCnt);
			TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StartCurrent);
			Motor_MoveAbsolutePosition(&Motor_T, 3000, 1000, nCnt+Motor_T.CurrentStep, 1000);
			while(!Motor_GetStatus(&Motor_T, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_T : %d  END Current_Step : %d",nCnt,Motor_T.CurrentStep);
		}
		else if (strstr(msg.Data, UART_MOTOR_CNS_ORG))
		{
			printUart(DBG_MSG_PC, "MOTOR_CNS_ORG START");
			TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StartCurrent);
			Motor_RunOrgCheck(&Motor_CNS);
			while(!Motor_GetStatus(&Motor_CNS, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_CNS_ORG END");
		}
		else if (strstr(msg.Data, UART_MOTOR_CNS_END))
		{
			printUart(DBG_MSG_PC, "MOTOR_CNS_END START");
			TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StartCurrent);
			Motor_MoveAbsolutePosition(&Motor_CNS, 30000, 3000, 171338, 3000);
			while(!Motor_GetStatus(&Motor_CNS, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_CNS_END END");
		}
		else if (strstr(msg.Data, UART_MOTOR_CNS_STOP))
		{
			printUart(DBG_MSG_PC, "MOTOR_CNS STATUS : %d",Motor_CNS.Status);
			Motor_CNS.nInitRunStep = 0;
			Motor_Stop(&Motor_CNS);
			printUart(DBG_MSG_PC, "MOTOR_CNS STATUS_STOP : %d",Motor_CNS.Status);
		}
		else if (strstr(msg.Data, UART_MOTOR_CNS_STEP))
		{
			int16_t nCnt=0;

			if (strchr(&(msg.Data[9]), 'M'))
			{
				nCnt -= atoi(&(msg.Data[10]));
			}
			else if(strchr(&(msg.Data[9]), 'P'))
			{
				nCnt += atoi(&(msg.Data[10]));
			}
			else
			{
				printUart(DBG_MSG_PC, "%s : %s", ERR_CODE_UART_MSG_ERROR, msg.Data);
				return RESET;
			}

			nCnt*=100;
			
			printUart(DBG_MSG_PC, "MOTOR_CNS : %d START",nCnt);
			TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StartCurrent);
			Motor_MoveAbsolutePosition(&Motor_CNS, 30000, 1000, nCnt+Motor_CNS.CurrentStep, 1000);
			while(!Motor_GetStatus(&Motor_CNS, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_CNS : %d  END Current_Step : %d",nCnt,Motor_CNS.CurrentStep);
		}
		else if (strstr(msg.Data, UART_MOTOR_CWE_ORG))
		{
			printUart(DBG_MSG_PC, "MOTOR_CWE_ORG START");
			TMC2660_SetCurrent(Motor_CWE.MotType, Motor_CWE.StartCurrent);
			Motor_RunOrgCheck(&Motor_CWE);
			while(!Motor_GetStatus(&Motor_CWE, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_CWE.MotType, Motor_CWE.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_CWE_ORG END");
		}
		else if (strstr(msg.Data, UART_MOTOR_CWE_END))
		{
			printUart(DBG_MSG_PC, "MOTOR_CWE_END START");
			TMC2660_SetCurrent(Motor_CWE.MotType, Motor_CWE.StartCurrent);
			Motor_MoveAbsolutePosition(&Motor_CWE, 20000, 2000, 41000, 2000);
			while(!Motor_GetStatus(&Motor_CWE, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_CWE.MotType, Motor_CWE.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_CWE_END END");
		}
		else if (strstr(msg.Data, UART_MOTOR_CWE_STOP))
		{
			printUart(DBG_MSG_PC, "MOTOR_CWE STATUS : %d",Motor_CWE.Status);
			Motor_CWE.nInitRunStep = 0;
			Motor_Stop(&Motor_CWE);
			printUart(DBG_MSG_PC, "MOTOR_CWE STATUS_STOP : %d",Motor_CWE.Status);
		}
		else if (strstr(msg.Data, UART_MOTOR_CWE_STEP))
		{
			int16_t nCnt=0;

			if (strchr(&(msg.Data[9]), 'M'))
			{
				nCnt -= atoi(&(msg.Data[10]));
			}
			else if(strchr(&(msg.Data[9]), 'P'))
			{
				nCnt += atoi(&(msg.Data[10]));
			}
			else
			{
				printUart(DBG_MSG_PC, "%s : %s", ERR_CODE_UART_MSG_ERROR, msg.Data);
				return RESET;
			}
				
			
			printUart(DBG_MSG_PC, "MOTOR_CWE : %d START",nCnt);
			TMC2660_SetCurrent(Motor_CWE.MotType, Motor_CWE.StartCurrent);
			Motor_MoveAbsolutePosition(&Motor_CWE, 20000, 1000, nCnt+Motor_CWE.CurrentStep, 1000);
			while(!Motor_GetStatus(&Motor_CWE, STATUS_STOP));			
			TMC2660_SetCurrent(Motor_CWE.MotType, Motor_CWE.StopCurrent);
			printUart(DBG_MSG_PC, "MOTOR_CWE : %d  END Current_Step : %d",nCnt,Motor_CWE.CurrentStep);
		}
		else if (strstr(msg.Data, UART_HEAD_BEAM_CUSTOM_ON))
        {
        	if (strchr(&(msg.Data[13]), 'P'))
			{
				LaserControl(TYPE_HEAD_PANO, SET);
			}
			else if(strchr(&(msg.Data[13]), 'C'))
			{
				LaserControl(TYPE_HEAD_CT, SET);
			}
			else if(strchr(&(msg.Data[13]), 'S'))
			{
				LaserControl(TYPE_HEAD_CEPH, SET);
			}			
        }
        else if (strstr(msg.Data, UART_HEAD_BEAM_CUSTOM_OFF))
        {
        	if (strchr(&(msg.Data[13]), 'P'))
			{
				LaserControl(TYPE_HEAD_PANO, RESET);
			}
			else if(strchr(&(msg.Data[13]), 'C'))
			{
				LaserControl(TYPE_HEAD_CT, RESET);
			}
			else if(strchr(&(msg.Data[13]), 'S'))
			{
				LaserControl(TYPE_HEAD_CEPH, RESET);
			}			
        }	
		else if (strstr(msg.Data, UART_FOOT_BEAM_ON))
        {
			LaserControl(TYPE_FOOT, SET);
        }
        else if (strstr(msg.Data, UART_FOOT_BEAM_OFF))
        {
			LaserControl(TYPE_FOOT, RESET);
        }
		else if (strstr(msg.Data, UART_COLLIMATOR_FILTER_RIGHT))
		{
			if (!CAN_Collimator_SendMessage(CMD_FILTER_CT, 0, 0, 1000, 2))
            {
                return RESET;
            }
		}
		else if (strstr(msg.Data, UART_COLLIMATOR_FILTER_CENTER))
		{
			if (!CAN_Collimator_SendMessage(CMD_FILTER_CENTER, 0, 0, 1000, 2))
            {
                return RESET;
            }
		}
		else if (strstr(msg.Data, UART_COLLIMATOR_FILTER_LEFT))
		{
			if (!CAN_Collimator_SendMessage(CMD_FILTER_CT_CALIB, 0, 0, 1000, 2))
            {
                return RESET;
            }
		}
		else if (strstr(msg.Data, UART_ALIGN_PANO_1COL_START_POS))
        {                                         
            if (!CAN_Collimator_SendMessage(CMD_COLLI_PANOR, 0, 0, 2000, 2))
            {
                return RESET;
            }    
		}
        else if (strstr(msg.Data, UART_COLLIMATOR_CLOSE))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CLOSE, 0, 0, 2000, 2))
            {
                return RESET;
            }    
        }
        else if (strstr(msg.Data, UART_COLLIMATOR_OPEN))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_OPEN, 0, 0, 2000, 2))
            {
                return RESET;
            }
        }
		else if (strstr(msg.Data, UART_FOV_5X5))
		{
			if (!CAN_Collimator_SendMessage(CMD_COLLI_CT5X5, 0, 0, 1000, 2))
            {
                return RESET;
            }    
		}
        else if (strstr(msg.Data, UART_FOV_8X9))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CT8X9, 0, 0, 1000, 2))
            {
                return RESET;
            }    
        }	
        else if (strstr(msg.Data, UART_FOV_10X9))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CT10X9, 0, 0, 1000, 2))
            {
                return RESET;
            }    
        }	
        else if (strstr(msg.Data, UART_FOV_12X9))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CT12X9, 0, 0, 1000, 2))
            {
                return RESET;
            }    
        }	
        else if (strstr(msg.Data, UART_FOV_15X9))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CT15X9, 0, 0, 1000, 2))
            {
                return RESET;
            }    
        }
		else if (strstr(msg.Data, UART_ALIGN_SCAN_1COL_CENTER))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLI_CEPHSCAN_CENTER, 0, 0, 3000, 2))
            {
                return RESET;
            }    
        }        
        else if (strstr(msg.Data, UART_ALIGN_SCAN_1COL_START_POS))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CEPHSCAN_INIT, 0, 0, 2000, 2))
            {
                return RESET;
            }    
        }        
		else if(strstr(msg.Data, UART_TUBE_TILT_OFF))
		{
			if (!CAN_Collimator_SendMessage(CMD_TILT_OFF, 0, 0, 5000, 2))
            {
                return RESET;
            }
		}
		else if(strstr(msg.Data, UART_TUBE_TILT_ON))
		{
			if (!CAN_Collimator_SendMessage(CMD_TILT_ON, 0, 0, 5000, 2))
            {
                return RESET;
            }
		}
		else if(strstr(msg.Data, UART_COLLI_STATUS_ALL_STOP))
		{
			if (!CAN_Collimator_SendMessage(CMD_COLLI_MOTOR_ALL_STATUS_STOP, 0, 0, 5000, 2))
            {
                return RESET;
            }
		}
		else if(strstr(msg.Data, UART_COLLI_STATUS_TOP_STOP))
		{
			if (!CAN_Collimator_SendMessage(CMD_COLLI_MOTOR_T_STATUS_STOP, 0, 0, 5000, 2))
            {
                return RESET;
            }
		}
		else if(strstr(msg.Data, UART_COLLI_STATUS_BOTTOM_STOP))
		{
			if (!CAN_Collimator_SendMessage(CMD_COLLI_MOTOR_B_STATUS_STOP, 0, 0, 5000, 2))
            {
                return RESET;
            }
		}
		else if(strstr(msg.Data, UART_COLLI_STATUS_LEFT_STOP))
		{
			if (!CAN_Collimator_SendMessage(CMD_COLLI_MOTOR_L_STATUS_STOP, 0, 0, 5000, 2))
            {
                return RESET;
            }
		}
		else if(strstr(msg.Data, UART_COLLI_STATUS_RIGHT_STOP))
		{
			if (!CAN_Collimator_SendMessage(CMD_COLLI_MOTOR_R_STATUS_STOP, 0, 0, 5000, 2))
            {
                return RESET;
            }
		}
		else if(strstr(msg.Data, UART_COLLI_STATUS_FILTER_STOP))
		{
			if (!CAN_Collimator_SendMessage(CMD_COLLI_MOTOR_F_STATUS_STOP, 0, 0, 5000, 2))
            {
                return RESET;
            }
		}
		else if(strstr(msg.Data, UART_COLLI_STATUS_LASER_STOP))
		{
			if (!CAN_Collimator_SendMessage(CMD_COLLI_MOTOR_H_STATUS_STOP, 0, 0, 5000, 2))
            {
                return RESET;
            }
		}
		else if(strstr(msg.Data, UART_COLLI_STATUS_ALL_MOVE))
		{
			int32_t nDistance = atoi(&(msg.Data[10]));

			if (strchr(&(msg.Data[9]), 'P'))
			{
				if (!CAN_Collimator_SendMessage(CMD_COLLI_MOTOR_ALL_RUN, nDistance, 0, 5000, 2))
	            {
	                return RESET;
	            }
			}
			else if(strchr(&(msg.Data[9]), 'M'))
			{
				if (!CAN_Collimator_SendMessage(CMD_COLLI_MOTOR_ALL_RUN, nDistance, 1, 5000, 2))
	            {
	                return RESET;
	            }
			}
			else
			{
				printUart(DBG_MSG_PC, "Wrong Message");
			}
		}
		else if(strstr(msg.Data, UART_COLLI_STATUS_TOP_MOVE))
		{
			int32_t nDistance = atoi(&(msg.Data[10]));

			if (strchr(&(msg.Data[9]), 'P'))
			{
				if (!CAN_Collimator_SendMessage(CMD_COLLI_MOTOR_T_RUN, nDistance, 0, 5000, 2))
	            {
	                return RESET;
	            }
			}
			else if(strchr(&(msg.Data[9]), 'M'))
			{
				if (!CAN_Collimator_SendMessage(CMD_COLLI_MOTOR_T_RUN, nDistance, 1, 5000, 2))
	            {
	                return RESET;
	            }
			}
			else
			{
				printUart(DBG_MSG_PC, "Wrong Message");
			}
		}
		else if(strstr(msg.Data, UART_COLLI_STATUS_BOTTOM_MOVE))
		{
			int32_t nDistance = atoi(&(msg.Data[10]));

			if (strchr(&(msg.Data[9]), 'P'))
			{
				if (!CAN_Collimator_SendMessage(CMD_COLLI_MOTOR_B_RUN, nDistance, 0, 5000, 2))
	            {
	                return RESET;
	            }
			}
			else if(strchr(&(msg.Data[9]), 'M'))
			{
				if (!CAN_Collimator_SendMessage(CMD_COLLI_MOTOR_B_RUN, nDistance, 1, 5000, 2))
	            {
	                return RESET;
	            }
			}
			else
			{
				printUart(DBG_MSG_PC, "Wrong Message");
			}
		}
		else if(strstr(msg.Data, UART_COLLI_STATUS_LEFT_MOVE))
		{
			int32_t nDistance = atoi(&(msg.Data[10]));

			if (strchr(&(msg.Data[9]), 'P'))
			{
				if (!CAN_Collimator_SendMessage(CMD_COLLI_MOTOR_L_RUN, nDistance, 0, 5000, 2))
	            {
	                return RESET;
	            }
			}
			else if(strchr(&(msg.Data[9]), 'M'))
			{
				if (!CAN_Collimator_SendMessage(CMD_COLLI_MOTOR_L_RUN, nDistance, 1, 5000, 2))
	            {
	                return RESET;
	            }
			}
			else
			{
				printUart(DBG_MSG_PC, "Wrong Message");
			}
		}
		else if(strstr(msg.Data, UART_COLLI_STATUS_RIGHT_MOVE))
		{
			int32_t nDistance = atoi(&(msg.Data[10]));

			if (strchr(&(msg.Data[9]), 'P'))
			{
				if (!CAN_Collimator_SendMessage(CMD_COLLI_MOTOR_R_RUN, nDistance, 0, 5000, 2))
	            {
	                return RESET;
	            }
			}
			else if(strchr(&(msg.Data[9]), 'M'))
			{
				if (!CAN_Collimator_SendMessage(CMD_COLLI_MOTOR_R_RUN, nDistance, 1, 5000, 2))
	            {
	                return RESET;
	            }
			}
			else
			{
				printUart(DBG_MSG_PC, "Wrong Message");
			}
		}
		else if(strstr(msg.Data, UART_COLLI_STATUS_FILTER_MOVE))
		{
			int32_t nDistance = atoi(&(msg.Data[10]));

			if (strchr(&(msg.Data[9]), 'P'))
			{
				if (!CAN_Collimator_SendMessage(CMD_COLLI_MOTOR_F_RUN, nDistance, 0, 5000, 2))
	            {
	                return RESET;
	            }
			}
			else if(strchr(&(msg.Data[9]), 'M'))
			{
				if (!CAN_Collimator_SendMessage(CMD_COLLI_MOTOR_F_RUN, nDistance, 1, 5000, 2))
	            {
	                return RESET;
	            }
			}
			else
			{
				printUart(DBG_MSG_PC, "Wrong Message");
			}
		}
		else if(strstr(msg.Data, UART_COLLI_STATUS_LASER_MOVE))
		{
			int32_t nDistance = atoi(&(msg.Data[10]));

			if (strchr(&(msg.Data[9]), 'P'))
			{
				if (!CAN_Collimator_SendMessage(CMD_COLLI_MOTOR_H_RUN, nDistance, 0, 5000, 2))
	            {
	                return RESET;
	            }
			}
			else if(strchr(&(msg.Data[9]), 'M'))
			{
				if (!CAN_Collimator_SendMessage(CMD_COLLI_MOTOR_H_RUN, nDistance, 1, 5000, 2))
	            {
	                return RESET;
	            }
			}
			else
			{
				printUart(DBG_MSG_PC, "Wrong Message");
			}
		}
#ifdef USE_MOTOR_CHINREST_HOR
		else if (strstr(msg.Data, UART_CMD_CHINREST_HOR_MOVE))
		{
			int32_t nDistance = atoi(&(msg.Data[12]));

			printUart(DBG_MSG_PC, "H-Axis Distance = %dmm", nDistance);

			if (nDistance >= 0)
			{
				//if (strchr(&(message.Data[11]), 'L'))
				if (strchr(&(msg.Data[11]), 'R'))
					nDistance = ~nDistance + 1;

				//추후 23?�라??값을 Align Offset?�로 ?�정??�?
				Motor_ControlChinrest(RESET, MOTOR_CWE_INIT_POSITION + nDistance);
			}
		}
#endif /* USE_MOTOR_CHINREST_HOR */
#ifdef USE_MOTOR_CHINREST_VER
		else if (strstr(msg.Data, UART_CMD_CHINREST_VER_MOVE))
		{
			int32_t nDistance = atoi(&(msg.Data[12]));

			printUart(DBG_MSG_PC, "V-Axis Distance = %dmm", nDistance);

			if (nDistance >= 0)
			{
				if (strchr(&(msg.Data[11]), 'D'))
					nDistance = ~nDistance + 1;

				//추후 58?�라??값을 Align Offset?�로 ?�정??�?
				Motor_ControlChinrest(SET, MOTOR_CNS_CT_INIT_POSITION + nDistance);
			}
		}
#endif /* USE_MOTOR_CHINREST_VER */	
		else if (strstr(msg.Data, UART_STICH_TEST))
		{
			uint32_t nCount = atoi(&(msg.Data[7]));
			uint32_t nDelay = atoi(&(msg.Data[11]));
			if (strchr(&(msg.Data[10]), 'D'))
			{
				Motor_MoveStitchPosition(TOP_DOWN,nCount,nDelay);
			}
			else if(strchr(&(msg.Data[10]), 'U'))
			{
				Motor_MoveStitchPosition(BOTTOM_UP,nCount,nDelay);
			}
			else if(strchr(&(msg.Data[10]), 'R'))
			{
				TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StartCurrent);
#ifdef USE_MOTOR_CHINREST_VER
				Motor_RunOrgCheck(&Motor_CNS);
				while(!Motor_GetStatus(&Motor_CNS, STATUS_STOP));
				//jehun
				Motor_MoveAbsolutePosition(&Motor_CNS, 30000, 3000, 5000, 3000); 
//				Motor_MoveAbsolutePosition(&Motor_CNS, 14000, 3000, 5000, 3000); 
				while(!Motor_GetStatus(&Motor_CNS, STATUS_STOP));
#endif /* USE_MOTOR_CHINREST_VER */
				TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StopCurrent);
			}
		}
		else if (strstr(msg.Data, UART_FOV_15X15))
		{
//20190611_myshin : ?�시�??�기???�음
#ifdef USE_MOTOR_CHINREST_VER
#if defined(USE_MOTOR_CNS_SCREW_PITCH_6_35)
			TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StartCurrent);
			Motor_MoveAbsolutePosition(&Motor_CNS, 4000, 2000, 0, 2000); 
			TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StopCurrent);
#else /* not USE_MOTOR_CNS_SCREW_PITCH_6_35 */
			//190726 HWAN 8mm up and reverse stitching			
			int32_t nVerStep = (-16/1.27) * 3200;
			//190821 HWAN TEST
			TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StartCurrent);
			
			//jehun
//			Motor_MoveAbsolutePosition(&Motor_CNS, 14000, 8000, nVerStep, 8000);
			Motor_MoveAbsolutePosition(&Motor_CNS, 30000, 8000, nVerStep, 8000);
			while(!Motor_GetStatus(&Motor_CNS, STATUS_STOP));
			TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StopCurrent);
			UART_SendMessage(DBG_MSG_PC, "[SP_CT_CNSDONE]");
#ifdef USE_TABLET_PC
   			UART_SendMessage(DBG_MSG_TABLET, "[SP_CT_CNSDONE]");
#endif /* USE_TABLET_PC */
#endif /* USE_MOTOR_CNS_SCREW_PITCH_6_35 */
#endif /* USE_MOTOR_CHINREST_VER */				
			CtParam.bStitchMode = SET;
            CtParam.Fov = CT_FOV_15X15;
		}
		else if(strstr(msg.Data, UART_STITCH_CHECK_TEST))
		{
			bool ret=RESET;

			UART_SendMessage(DBG_MSG_PC, "Switch TEST");
			ret = StitchUnderSensor_Check();
			if(ret==RESET)
				printUart(DBG_MSG_PC, "Under Sensor RESET %d", ret);
			else
				printUart(DBG_MSG_PC, "Under Sensor SET %d", ret);

			ret = StitchUpSwitch_Check();
			if(ret==RESET)
				printUart(DBG_MSG_PC, "Up Switch RESET %d", ret);
			else
				printUart(DBG_MSG_PC, "Up Switch SET %d", ret);
			
		}
		else if(strstr(msg.Data, UART_QUEUE_TEST_PROTOCOL))
		{
			int32_t nDistance = atoi(&(msg.Data[12]));

			printUart(DBG_MSG_PC, "The Number is %d ", nDistance);
		}
		else if (strstr(msg.Data, UART_TUBE_KV_ADC_VALUE_CHECK))
		{
			int16_t nOffset = 0;	
			char str[40] = {0,};
			uint8_t i =0;
			nOffset = atoi(&(msg.Data[12]));
			if(nOffset>50)
			{
				sprintf(str, "Range Over");
				UART_SendMessage(DBG_MSG_PC, str);
				return false;
			}
			
			for(i = 0 ; i<nOffset; i++)
			{
				CAN_SendMessage(CAN_TUBE_KV_ADC_VALUE_CHECK, i, 0, 1000, 2);
				IntTimer_Delay(100);
   				while(!IntTimer_GetStatus());
			}			
		}	
		else if (strstr(msg.Data, UART_TUBE_MA_ADC_VALUE_CHECK))
		{
			int16_t nOffset = 0;	
			char str[40] = {0,};
			uint8_t i =0;
			nOffset = atoi(&(msg.Data[12]));
			if(nOffset>50)
			{
				sprintf(str, "Range Over");
				UART_SendMessage(DBG_MSG_PC, str);
				return false;
			}
			for(i = 0 ; i<nOffset; i++)
			{
				CAN_SendMessage(CAN_TUBE_MA_ADC_VALUE_CHECK, i, 0, 1000, 2);
				IntTimer_Delay(100);
   				while(!IntTimer_GetStatus());
			}			
		}
		else if (strstr(msg.Data, UART_TUBE_KV_ADC_VALUE_NUM_CHECK))
		{
			int16_t nOffset = 0;
			char str[40] = {0,};
			nOffset = atoi(&(msg.Data[12]));
			
			sprintf(str, "kV_ADC_INDEX = %d", nOffset);
			UART_SendMessage(DBG_MSG_PC, str);
			
			CAN_SendMessage(CAN_TUBE_KV_ADC_VALUE_CHECK, nOffset, 0, 1000, 2);
		}
		else if (strstr(msg.Data, UART_TUBE_MA_ADC_VALUE_NUM_CHECK))
		{
			int16_t nOffset = 0;
			char str[40] = {0,};
			nOffset = atoi(&(msg.Data[12]));
			
			sprintf(str, "mA_ADC_INDEX = %d", nOffset);
			UART_SendMessage(DBG_MSG_PC, str);
			
			CAN_SendMessage(CAN_TUBE_MA_ADC_VALUE_CHECK, nOffset, 0, 1000, 2);
		}		
		else if (strstr(msg.Data, UART_TUBE_ERROR_VALUE_CHECK))
		{
			char str[40] = {0,};
			uint8_t i=6;
			
			sprintf(str, "TANK VALUE CHECK");
			UART_SendMessage(DBG_MSG_PC, str);
			
			for(;i<10;i++)
			{
				CAN_SendMessage(CAN_TUBE_ERROR_VALUE_CHECK, i, 0, 1000, 2);
				IntTimer_Delay(100);
	   			while(!IntTimer_GetStatus());
			}	
		}	
		else if (strstr(msg.Data, UART_TUBE_ERROR_ADC_RESET))
		{			
			char str[40] = {0,};
			sprintf(str, "TANK ERROR MEMORY RESET");
			UART_SendMessage(DBG_MSG_PC, str);

			CAN_SendMessage(CAN_TUBE_RESET_ERROR_TABLE, 0, 0, 1000, 2);
		}
		else if (strstr(msg.Data, UART_TUBE_INITIAL_VALUE))
		{			
			char str[40] = {0,};
			sprintf(str, "TANK INITIAL");
			UART_SendMessage(DBG_MSG_PC, str);

			CAN_SendMessage(CAN_TUBE_RESET_INIRIAL_VALUE, 0, 0, 1000, 2);
		}
		else if (strstr(msg.Data, UART_TUBE_READY_TIME_CHECK))
		{			
			char str[40] = {0,};
			sprintf(str, "TANK READY TIME CHECK");
			UART_SendMessage(DBG_MSG_PC, str);

			CAN_SendMessage(CAN_TUBE_READY_TIME_CHECK, 0, 0, 1000, 2);
		}
		else if (strstr(msg.Data, UART_TUBE_EXPOSURE_TIME_CHECK))
		{			
			char str[40] = {0,};
			sprintf(str, "TANK EXPOSURE TIME CHECK");
			UART_SendMessage(DBG_MSG_PC, str);

			CAN_SendMessage(CAN_TUBE_EXP_TIME_CHECK, 0, 0, 1000, 2);
		}
		else if (strstr(msg.Data, UART_TUBE_KV_STANDBY_VALUE_CHECK))
		{
			int16_t nOffset = 0;	
			char str[40] = {0,};
			uint8_t i =0;
			nOffset = atoi(&(msg.Data[12]));
			if(nOffset>50)
			{
				sprintf(str, "Range Over");
				UART_SendMessage(DBG_MSG_PC, str);
				return false;
			}
			
			for(i = 0 ; i<nOffset; i++)
			{
				CAN_SendMessage(CAN_TUBE_KV_STANDBY_ADC_VALUE_CHECK, i, 0, 1000, 2);
				IntTimer_Delay(100);
   				while(!IntTimer_GetStatus());
			}			
		}	
		else if (strstr(msg.Data, UART_TUBE_MA_STANDBY_VALUE_CHECK))
		{
			int16_t nOffset = 0;	
			char str[40] = {0,};
			uint8_t i =0;
			nOffset = atoi(&(msg.Data[12]));
			if(nOffset>50)
			{
				sprintf(str, "Range Over");
				UART_SendMessage(DBG_MSG_PC, str);
				return false;
			}
			for(i = 0 ; i<nOffset; i++)
			{
				CAN_SendMessage(CAN_TUBE_MA_STANDBY_ADC_VALUE_CHECK, i, 0, 1000, 2);
				IntTimer_Delay(100);
   				while(!IntTimer_GetStatus());
			}			
		}
		else if(strstr(msg.Data, UART_TUBE_EXPOSURE_FEEDBACK_MAX_MIN_VALUE_CHECK))
		{	
			UART_SendMessage(DBG_MSG_PC, "FEEDBACK MAX MIN CHECK");
			
			CAN_SendMessage(CAN_TUBE_kV_MIN_VALUE_CHECK, 0, 0, 1000, 2);
			IntTimer_Delay(100);
			while(!IntTimer_GetStatus());

			CAN_SendMessage(CAN_TUBE_kV_MAX_VALUE_CHECK, 0, 0, 1000, 2);
			IntTimer_Delay(100);
			while(!IntTimer_GetStatus());
			
			CAN_SendMessage(CAN_TUBE_mA_MIN_VALUE_CHECK, 0, 0, 1000, 2);
			IntTimer_Delay(100);
			while(!IntTimer_GetStatus());
			
			CAN_SendMessage(CAN_TUBE_mA_MAX_VALUE_CHECK, 0, 0, 1000, 2);
			IntTimer_Delay(100);
			while(!IntTimer_GetStatus());
		}
		else if (strstr(msg.Data, UART_TUBE_ERROR_ON_OFF))
		{
			char str[40] = {0,};
			if (strchr(&(msg.Data[13]), 'N'))
			{
				sprintf(str, "KV_MA_ERROR_ENABLE");
				UART_SendMessage(DBG_MSG_PC, str);
				CAN_SendMessage(CAN_TUBE_KV_MA_ERROR_DISABLE, 1, 0, 1000, 2);
			}
			else
			{
				sprintf(str, "KV_MA_ERROR_DISABLE");
				UART_SendMessage(DBG_MSG_PC, str);
				CAN_SendMessage(CAN_TUBE_KV_MA_ERROR_DISABLE, 0, 0, 1000, 2);
			}
		}
		else if (strstr(msg.Data, UART_TUBE_TOTAL_ERROR_ON_OFF))
		{
			char str[40] = {0,};
			if (strchr(&(msg.Data[13]), 'N'))
			{
				sprintf(str, "TOTAL_ERROR_ENABLE");
				UART_SendMessage(DBG_MSG_PC, str);
				CAN_SendMessage(CAN_TUBE_TOTAL_ERROR_CHECK, 1, 0, 1000, 2);
			}
			else
			{
				sprintf(str, "TOTAL_ERROR_DISABLE");
				UART_SendMessage(DBG_MSG_PC, str);
				CAN_SendMessage(CAN_TUBE_TOTAL_ERROR_CHECK, 0, 0, 1000, 2);
			}
		}
		else if (strstr(msg.Data, UART_TUBE_COUNT_RESET))
		{
			if (!CAN_SendMessage(CAN_TUBE_COUNT_RESET, 0, 0, 1000, 2))
			{
    			return RESET;
			}
		}
		else if (strstr(msg.Data, UART_TUBE_COUNT_CHECK))
		{	
			if (!CAN_SendMessage(CAN_TUBE_COUNT_CHECK, 0, 0, 1000, 2))
			{
				return RESET;
			}
		}
		else if (strstr(msg.Data, UART_TUBE_EEPROM_CHECK))
		{	
			char str[40] = {0,};
			uint8_t i=0;
			
			sprintf(str, "TANK EEPROM CHECK");
			UART_SendMessage(DBG_MSG_PC, str);
			
			for(;i<12;i++)
			{
				CAN_SendMessage(CAN_TUBE_EEPROM_VALUE_CHECK, i, 0, 1000, 2);
				IntTimer_Delay(100);
	   			while(!IntTimer_GetStatus());
			}	
		}
		else if (strstr(msg.Data, UART_MEMBRANE_SOUND_ENTRY))
		{	
#ifdef USE_TABLET_PC
	   		UART_SendMessage(DBG_MSG_TABLET, "[SP_MODE_PANO_]");
#endif /* USE_TABLET_PC */
			msg.Data[2] = 'M';
		}
		else if (strstr(msg.Data, UART_MEMBRANE_SOUND_PANO_READY))
		{	
#ifdef USE_TABLET_PC
			UART_SendMessage(DBG_MSG_TABLET, "[SP_PANO_READY]");	
#endif /* USE_TABLET_PC */
			msg.Data[2] = 'M';
		}
		else if (strstr(msg.Data, UART_MEMBRANE_SOUND_CT_READY))
		{	
#ifdef USE_TABLET_PC
			UART_SendMessage(DBG_MSG_TABLET, "[SP_CT___READY]");
#endif /* USE_TABLET_PC */
			msg.Data[2] = 'M';
		}
		else if (strstr(msg.Data, UART_MEMBRANE_SOUND_SCAN_READY))
		{	
#ifdef USE_TABLET_PC
			UART_SendMessage(DBG_MSG_TABLET, "[SP_SCAN_READY]");
#endif /* USE_TABLET_PC */
			msg.Data[2] = 'M';
		}
		else if (strstr(msg.Data, UART_MEMBRANE_SOUND_CAPTURING))
		{	
#ifdef USE_TABLET_PC
			UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_START]");
#endif /* USE_TABLET_PC */
			msg.Data[2] = 'M';
		}
		else if (strstr(msg.Data, UART_MEMBRANE_SOUND_CAPTURE_END))
		{	
#ifdef USE_TABLET_PC
			UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_END__]");
#endif /* USE_TABLET_PC */
			msg.Data[2] = 'M';
		}
		else
		{
			printUart(DBG_MSG_PC, "%s : %s", ERR_CODE_UART_MSG_ERROR, msg.Data);
			return RESET;
		}

        msg.Data[1] = 'E';
        printUart(DBG_MSG_PC, msg.Data);		
	}
		
	return ret;
}


/**
* @ Function Name : UART_QueueInit

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/
