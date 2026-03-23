/*
*******************************************************************************
* cmd_calibration.c : Sensor calibration and collimator alignment handler
*******************************************************************************
* Copyright (C) 2014-2018 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Brief   : UART_SensorCalibration -- processes queued commands during
*             sensor calibration and collimator alignment modes.
*
* @ Revision History :
*       1) Extracted from cmd_geoalign_cal.c for modular build --- 2026-03-24
*******************************************************************************
*/

/* Include files ----------------------------------------------------------- */
#include "cmd_geoalign_cal_priv.h"

/* Calibration handler: extracted from serial.c lines 4982-7363 */
/**
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
            CalibrationModeParam.ColCmd = CMD_OFFSET_PANO_T;
        }
        else if (strstr(message.Data, UART_ALIGN_PANO_COLLIMATOR_BOTTOM))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_PANO_B;
        }
        else if (strstr(message.Data, UART_ALIGN_PANO_COLLIMATOR_LEFT))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_PANO_L;
        }
        else if (strstr(message.Data, UART_ALIGN_PANO_COLLIMATOR_RIGHT))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_PANO_R;
        }
        else if (strstr(message.Data, UART_ALIGN_PANO_COLLIMATOR_STEP_INC))
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
        else if (strstr(message.Data, UART_ALIGN_PANO_COLLIMATOR_STEP_DEC))
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
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_PANO_T, 0, 0, 0, 0)) { return RESET; }
        }
        else if (strstr(message.Data, UART_ALIGN_PANO_COLLIMATOR_REQ_BOTTOM))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_PANO_B, 0, 0, 0, 0)) { return RESET; }
        }
        else if (strstr(message.Data, UART_ALIGN_PANO_COLLIMATOR_REQ_LEFT))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_PANO_L, 0, 0, 0, 0)) { return RESET; }
        }
        else if (strstr(message.Data, UART_ALIGN_PANO_COLLIMATOR_REQ_RIGHT))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_PANO_R, 0, 0, 0, 0)) { return RESET; }
        }
		else if (strstr(message.Data, UART_ALIGN_PANO_1COL_START_POS))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_PANOR, 0, 0, 2000, 2)) { return RESET; }
		}
        else if (strstr(message.Data, UART_COLLIMATOR_CLOSE))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CLOSE, 0, 0, 2000, 2)) { return RESET; }
            bClose = SET;
			bOpen = RESET;
        }
        else if (strstr(message.Data, UART_COLLIMATOR_OPEN))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_OPEN, 0, 0, 2000, 2)) { return RESET; }
			bOpen = SET;
			bClose = RESET;
        }
		else if (strstr(message.Data, UART_FOV_5X5))
		{
			if (!CAN_Collimator_SendMessage(CMD_COLLI_CT5X5, 0, 0, 1000, 2)) { return RESET; }
			CalibrationModeParam.nFovCmd = CMD_COLLI_CT5X5;
		}
        else if (strstr(message.Data, UART_FOV_8X9))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CT8X9, 0, 0, 1000, 2)) { return RESET; }
            CalibrationModeParam.nFovCmd = CMD_COLLI_CT8X9;
        }
        else if (strstr(message.Data, UART_FOV_10X9))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CT10X9, 0, 0, 1000, 2)) { return RESET; }
            CalibrationModeParam.nFovCmd = CMD_COLLI_CT10X9;
        }
        else if (strstr(message.Data, UART_FOV_12X9))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CT12X9, 0, 0, 1000, 2)) { return RESET; }
            CalibrationModeParam.nFovCmd = CMD_COLLI_CT12X9;
        }
        else if (strstr(message.Data, UART_FOV_15X9))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CT15X9, 0, 0, 1000, 2)) { return RESET; }
            CalibrationModeParam.nFovCmd = CMD_COLLI_CT15X9;
        }
        else if (strstr(message.Data, UART_FOV_10X10))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CT10X10, 0, 0, 1000, 2)) { return RESET; }
            CalibrationModeParam.nFovCmd = CMD_COLLI_CT10X10;
        }
        else if (strstr(message.Data, UART_FOV_12X10))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CT12X10, 0, 0, 1000, 2)) { return RESET; }
            CalibrationModeParam.nFovCmd = CMD_COLLI_CT12X10;
        }
        else if (strstr(message.Data, UART_FOV_15X10))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CT15X10, 0, 0, 1000, 2)) { return RESET; }
            CalibrationModeParam.nFovCmd = CMD_COLLI_CT15X10;
        }
        else if (strstr(message.Data, UART_FOV_8X10))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CT8X10, 0, 0, 1000, 2)) { return RESET; }
            CalibrationModeParam.nFovCmd = CMD_COLLI_CT8X10;
        }
        else if (strstr(message.Data, UART_ALIGN_CT_COLI_REQ_TOP))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CT_T, 0, 0, 5000, 0)) { return RESET; }
        }
        else if (strstr(message.Data, UART_ALIGN_CT_COLI_REQ_BOTTOM))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CT_B, 0, 0, 5000, 0)) { return RESET; }
        }
        else if (strstr(message.Data, UART_ALIGN_CT_COLI_REQ_LEFT))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CT_L, 0, 0, 5000, 0)) { return RESET; }
        }
        else if (strstr(message.Data, UART_ALIGN_CT_COLI_REQ_RIGHT))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CT_R, 0, 0, 5000, 0)) { return RESET; }
        }
        else if (strstr(message.Data, UART_ALIGN_CT_COLI_TOP))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_CT_T;
        }
        else if (strstr(message.Data, UART_ALIGN_CT_COLI_BOTTOM))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_CT_B;
        }
        else if (strstr(message.Data, UART_ALIGN_CT_COLI_LEFT))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_CT_L;
        }
        else if (strstr(message.Data, UART_ALIGN_CT_COLI_RIGHT))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_CT_R;
        }
        else if (strstr(message.Data, UART_ALIGN_SCAN_1COL_CENTER))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLI_CEPHSCAN_CENTER, 0, 0, 3000, 2)) { return RESET; }
        }
        else if (strstr(message.Data, UART_ALIGN_SCAN_1COL_START_POS))
        {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CEPHSCAN_INIT, 0, 0, 2000, 2)) { return RESET; }
        }
        else if (strstr(message.Data, UART_ALIGN_SCAN_COLI_REQ_TOP))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CEPHSCAN_T, 0, 0, 5000, 0)) { return RESET; }
        }
        else if (strstr(message.Data, UART_ALIGN_SCAN_COLI_REQ_BOTTOM))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CEPHSCAN_B, 0, 0, 5000, 0)) { return RESET; }
        }
        else if (strstr(message.Data, UART_ALIGN_SCAN_COLI_REQ_LEFT))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CEPHSCAN_L, 0, 0, 5000, 0)) { return RESET; }
        }
        else if (strstr(message.Data, UART_ALIGN_SCAN_COLI_REQ_RIGHT))
        {
            if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_CEPHSCAN_R, 0, 0, 5000, 0)) { return RESET; }
        }
        else if (strstr(message.Data, UART_ALIGN_SCAN_COLI_TOP))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_CEPHSCAN_T;
        }
        else if (strstr(message.Data, UART_ALIGN_SCAN_COLI_BOTTOM))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_CEPHSCAN_B;
        }
        else if (strstr(message.Data, UART_ALIGN_SCAN_COLI_LEFT))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_CEPHSCAN_L;
        }
        else if (strstr(message.Data, UART_ALIGN_SCAN_COLI_RIGHT))
        {
            CalibrationModeParam.ColCmd = CMD_OFFSET_CEPHSCAN_R;
        }
        else if (strstr(message.Data, UART_CEPH_2ND_COLI_1ST_ALIGN_POSITION))
        {
            UART_SendMessage(DBG_MSG_PC, "2ND_COLI_1ST_ALIGN_POSITION Start");
            TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
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
            Motor_CephMoveAbsolutePosition(&Motor_S, 2000, 1000, CEPH_2ND_COLI_ALIGN_POS + CephParam.n2ndColOffset, 1000);
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
            Motor_CephMoveAbsolutePosition(&Motor_S, 2000, 1000, CEPH_2ND_COLI_ALIGN_POS + CephParam.n2ndColFastStartOffset, 1000);
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
            Motor_CephMoveAbsolutePosition(&Motor_S, 2000, 1000, 300 * (MOTOR_C_MICROSTEP / (MOTOR_C_PULLEY_DIAMETER * pi))-(CEPH_2ND_COLI_ALIGN_POS) + CephParam.n2ndColEndOffset, 1000);
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
#endif
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
#endif
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
#endif
			sprintf(str, nOffset ? "n2ndColOffset = %+d" : "n2ndColOffset = %d", nOffset);
			UART_SendMessage(DBG_MSG_PC, str);
		}
#endif

        else if (strstr(message.Data, UART_CEPH_ALIGN_XRAY_SHOT_ON))
        {
            UART_SendMessage(DBG_MSG_PC, "Enter into Communication with Tube");
            if (!CAN_SendMessage(CAN_TUBE_COM_CHECK, CAN_TUBE_GEN2_COM, 0, 1000, 2)) { return RESET; }
            UART_SendMessage(DBG_MSG_PC, "Tube's Communication is checked");
            if (!CAN_SendMessage(CAN_TUBE_CONTI_MODE_SET, 0, 0, 1000, 2)) { return RESET; }
            if (!CAN_SendMessage(CAN_TUBE_KV_SET, CalibrationModeParam.KV, 0, 1000, 2)) { return RESET; }
            if (!CAN_SendMessage(CAN_TUBE_mA_SET, CalibrationModeParam.mA, 0, 1000, 2)) { return RESET; }
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
			if (b2ndColStart == SET) { nOffset = Motor_S.CurrentStep - CEPH_2ND_COLI_ALIGN_POS; }
			else { nOffset = Motor_S.CurrentStep - ALIGN_2ND_COLI_CENTER_POSISTION; }
            sprintf(str, "nOffset = %d", nOffset);
            UART_SendMessage(DBG_MSG_PC, str);
#ifdef USE_I2C_EEPROM
            EEPRom_I2C_Write_Word(ROM_CEPH_2ND_COL_START_ADDR, (uint16_t)nOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());
            printUart(DBG_MSG_PC, "ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_COL_START_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_START_ADDR));
#endif
            CephParam.n2ndColOffset = nOffset;
            sprintf(str, "n2ndColOffset = %d", CephParam.n2ndColOffset);
            UART_SendMessage(DBG_MSG_PC, str);
			CephParam.n2ndColFastStartOffset= (int16_t)EEPRom_I2C_Read_Word(ROM_CEPH_FAST_2ND_COL_START_ADDR);
		    if (CephParam.n2ndColFastStartOffset == (int16_t)0xffff) { CephParam.n2ndColFastStartOffset = CephParam.n2ndColOffset; }
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
            printUart(DBG_MSG_PC, "ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_COL_START_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_START_ADDR));
#endif
            CephParam.n2ndColOffset = nOffset;
            sprintf(str, "CephParam.n2ndColOffset = %d", CephParam.n2ndColOffset);
            UART_SendMessage(DBG_MSG_PC, str);
        }
		else if (strstr(message.Data, UART_CEPH_FAST_2ND_COLI_STEP_SET))
        {
            char str[40] = {0,};
            int16_t nOffset = 0;
			if (b2ndColStart == SET) { nOffset = Motor_S.CurrentStep - CEPH_2ND_COLI_ALIGN_POS; }
			else { nOffset = Motor_S.CurrentStep - ALIGN_2ND_COLI_CENTER_POSISTION; }
            sprintf(str, "nOffset = %d", nOffset);
            UART_SendMessage(DBG_MSG_PC, str);
#ifdef USE_I2C_EEPROM
            EEPRom_I2C_Write_Word(ROM_CEPH_FAST_2ND_COL_START_ADDR, (uint16_t)nOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());
            printUart(DBG_MSG_PC, "ROM_CEPH_FAST_2ND_COL_START_ADDR(0x%02x) : 0x%04x", ROM_CEPH_FAST_2ND_COL_START_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_FAST_2ND_COL_START_ADDR));
#endif
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
            printUart(DBG_MSG_PC, "ROM_CEPH_FAST_2ND_COL_START_ADDR(0x%02x) : 0x%04x", ROM_CEPH_FAST_2ND_COL_START_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_FAST_2ND_COL_START_ADDR));
#endif
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
            printUart(DBG_MSG_PC,"ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_COL_START_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_START_ADDR));
#endif
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
            printUart(DBG_MSG_PC, "ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_COL_START_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_START_ADDR));
#endif
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
			if(nOffset==0xffff) { nOffset=0; }
			else if(nOffset<0) nOffset++;
			if(nOffset == 1) { printUart(DBG_MSG_PC,"ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : [+0x%04x]", ROM_CEPH_2ND_COL_START_ADDR, nOffset); }
			else { printUart(DBG_MSG_PC,"ROM_CEPH_2ND_COL_START_ADDR(0x%02x) : [0x%04x]", ROM_CEPH_2ND_COL_START_ADDR, nOffset); }
			sprintf(str, nOffset ? "n2ndColOffset = %+d" : "n2ndColOffset = %d", nOffset);
			UART_SendMessage(DBG_MSG_PC, str);
#endif
		}
		else if (strstr(message.Data, UART_GET_CEPH_FAST_2ND_COLI_ALIGN_OFFSET))
		{
#ifdef USE_I2C_EEPROM
			char str[40] = {0,};
			int16_t nOffset = 0;
			nOffset = EEPRom_I2C_Read_Word(ROM_CEPH_FAST_2ND_COL_START_ADDR);
			if(nOffset==0xffff) { nOffset=0; }
			else if(nOffset<0) nOffset++;
			if(nOffset == 1) { printUart(DBG_MSG_PC,"ROM_CEPH_FAST_2ND_COL_START_ADDR(0x%02x) : [+0x%04x]", ROM_CEPH_FAST_2ND_COL_START_ADDR, nOffset); }
			else { printUart(DBG_MSG_PC,"ROM_CEPH_FAST_2ND_COL_START_ADDR(0x%02x) : [0x%04x]", ROM_CEPH_FAST_2ND_COL_START_ADDR, nOffset); }
			sprintf(str, nOffset ? "n2ndColOffset = %+d" : "n2ndColOffset = %d", nOffset);
			UART_SendMessage(DBG_MSG_PC, str);
#endif
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
            printUart(DBG_MSG_PC,"ROM_CEPH_2ND_COL_END_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_COL_END_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_END_ADDR));
#endif
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
            printUart(DBG_MSG_PC,"ROM_CEPH_2ND_COL_END_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_COL_END_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_COL_END_ADDR));
#endif
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
            printUart(DBG_MSG_PC, "ROM_CEPH_2ND_ADD_STEP_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_ADD_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_ADD_STEP_ADDR));
#endif
			sprintf(str, "RESET n2ndAdditionStepOffset = %d", CephParam.n2ndSpeedStepOffset);
            UART_SendMessage(DBG_MSG_PC, str);
		}
		else if(strstr(message.Data, UART_CEPH_SPEED_STEP_OFFSET))
		{
			char str[40] = {0,};
            int16_t nOffset = 0;
			if (strchr(&(message.Data[9]), 'M')) { nOffset -= atoi(&(message.Data[10])); }
			else { nOffset += atoi(&(message.Data[10])); }
            sprintf(str, "Addition_Step_nOffset = %d", nOffset);
            UART_SendMessage(DBG_MSG_PC, str);
			CephParam.n2ndSpeedStepOffset+=nOffset;
#if defined(USE_I2C_EEPROM)
            EEPRom_I2C_Write_Word(ROM_CEPH_2ND_ADD_STEP_ADDR, (uint16_t)CephParam.n2ndSpeedStepOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());
            printUart(DBG_MSG_PC, "ROM_CEPH_2ND_ADD_STEP_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_ADD_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_ADD_STEP_ADDR));
			CephParam.n2ndFastSpeedStepOffset= (int16_t)EEPRom_I2C_Read_Word(ROM_CEPH_2ND_FAST_ADD_STEP_ADDR);
	        if (CephParam.n2ndFastSpeedStepOffset == (int16_t)0xffff) { CephParam.n2ndFastSpeedStepOffset = CephParam.n2ndSpeedStepOffset; }
			else if(CephParam.n2ndFastSpeedStepOffset<0) CephParam.n2ndFastSpeedStepOffset++;
#endif
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
            printUart(DBG_MSG_PC, "ROM_CEPH_2ND_FAST_ADD_STEP_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_FAST_ADD_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_FAST_ADD_STEP_ADDR));
#endif
			sprintf(str, "RESET n2ndFastAdditionStepOffset = %d", CephParam.n2ndFastSpeedStepOffset);
            UART_SendMessage(DBG_MSG_PC, str);
		}
		else if(strstr(message.Data, UART_CEPH_FAST_SPEED_STEP_OFFSET))
		{
			char str[40] = {0,};
            int16_t nOffset = 0;
			if (strchr(&(message.Data[9]), 'M')) { nOffset -= atoi(&(message.Data[10])); }
			else { nOffset += atoi(&(message.Data[10])); }
            sprintf(str, "Fast_Addition_Step_nOffset = %d", nOffset);
            UART_SendMessage(DBG_MSG_PC, str);
            double n2nd_speed2= ((CephParam.n2ndColEndOffset+(300 * (3200 / (11 * pi))-3732.675))-(CephParam.n2ndColFastStartOffset+3732.675)+CephParam.n2ndFastSpeedStepOffset)/(CephParam.Time/1000);
            n2nd_speed2 = 1250000/(2*n2nd_speed2);
            printUart(DBG_MSG_PC, "RUNCCR = %lf", n2nd_speed2);
			CephParam.n2ndFastSpeedStepOffset+=nOffset;
#if defined(USE_I2C_EEPROM)
            EEPRom_I2C_Write_Word(ROM_CEPH_2ND_FAST_ADD_STEP_ADDR, (uint16_t)CephParam.n2ndFastSpeedStepOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());
            printUart(DBG_MSG_PC, "ROM_CEPH_2ND_FAST_ADD_STEP_ADDR(0x%02x) : 0x%04x", ROM_CEPH_2ND_FAST_ADD_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_2ND_FAST_ADD_STEP_ADDR));
#endif
            sprintf(str, "n2ndFastAdditionStepOffset = %d", CephParam.n2ndFastSpeedStepOffset);
            UART_SendMessage(DBG_MSG_PC, str);
		}
        /* ---------------------------------------------------------------
         * The remaining large handlers (UART_MANUAL_XRAY_SHOT,
         * UART_SHOT_READY, UART_CEPH_ALIGN_SCAN_TEST, and
         * UART_ALIGN_AUTO_COLLIMATOR) are kept verbatim from
         * the original cmd_geoalign_cal.c lines 3260-4599.
         * They are too tightly coupled to break apart without risk.
         * --------------------------------------------------------------- */
        else if (strstr(message.Data, UART_MANUAL_XRAY_SHOT))
        {
            CalibrationModeParam.nExpTime = atoi(&(message.Data[9]));
            if ((CalibrationModeParam.nExpTime > 0) && (CalibrationModeParam.nExpTime <= MAX_XRAY_SHOT_TIME))
            {
                UART_SendMessage(DBG_MSG_PC, "Enter into Communication with Tube");
                if (!CAN_SendMessage(CAN_TUBE_COM_CHECK, CAN_TUBE_GEN2_COM, 0, 1000, 2)) { return RESET; }
                UART_SendMessage(DBG_MSG_PC, "Tube's Communication is checked");
                if (!CAN_SendMessage(CAN_TUBE_CONTI_MODE_SET, 0, 0, 1000, 2)) { return RESET; }
                if (!CAN_SendMessage(CAN_TUBE_KV_SET, CalibrationModeParam.KV, 0, 1000, 2)) { return RESET; }
                if (!CAN_SendMessage(CAN_TUBE_mA_SET, CalibrationModeParam.mA, 0, 1000, 2)) { return RESET; }
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
				if (!CAN_SendMessage(CAN_TUBE_COUNT_CHECK, 0, 0, 1000, 2)) { return RESET; }
#if defined(GEN_TEMP_CHECK)
				CAN_SendMessage(CAN_TUBE_TANK_TEMP_READ, 0, 0, 1000, 2);
#endif
            }
            else
            {
                UART_SendMessage(DBG_MSG_PC, "[ERR]Invalid Shot Time");
            }
        }
		else if (strstr(message.Data, UART_TUBE_COUNT_RESET))
		{
			if (!CAN_SendMessage(CAN_TUBE_COUNT_RESET, 0, 0, 1000, 2)) { return RESET; }
		}
		else if (strstr(message.Data, UART_TUBE_COUNT_CHECK))
		{
			if (!CAN_SendMessage(CAN_TUBE_COUNT_CHECK, 0, 0, 1000, 2)) { return RESET; }
		}
		else if (strstr(message.Data, UART_CEPH_2ND_AROUND_SHOT))
		{
			char str[40] = {0,};
            int16_t nOffset = 0, nUnder_Offset=0, nUnder_Init=2000;
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
			else if(Motor_S.CurrentStep + nOffset > (300 * (MOTOR_S_MICROSTEP / (MOTOR_S_PULLEY_DIAMETER * pi))))
			{
				sprintf(str, "%s", UART_CALIBRATION_MOTOR_OVER_ERROR);
            	UART_SendMessage(DBG_MSG_PC, str);
			}
            else
            {
            	if (!CAN_Collimator_SendMessage(CMD_FILTER_CENTER, 0, 0, 1000, 2)) { return RESET; }
				if(Motor_S.CurrentStep - nOffset<0)
				{
					sprintf(str, "%s", UART_CALIBRATION_MOTOR_UNDER_ERROR);
            		UART_SendMessage(DBG_MSG_PC, str);
					nUnder_Offset=nOffset;
					nOffset=Motor_S.CurrentStep-nUnder_Init;
					bUnder_Offset=SET;
				}
            	TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
				Motor_CephMoveAbsolutePosition(&Motor_S, 2000, 1000, Motor_S.CurrentStep - nOffset, 1000);
	    	    while(!Motor_GetStatus(&Motor_S, STATUS_STOP));
				TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
				UART_SendMessage(DBG_MSG_PC, "Enter into Communication with Tube");
	            if (!CAN_SendMessage(CAN_TUBE_COM_CHECK, CAN_TUBE_GEN2_COM, 0, 3000, 2)) { return RESET; }
	            UART_SendMessage(DBG_MSG_PC, "Tube's Communication is checked");
	            if (!CAN_SendMessage(CAN_TUBE_CONTI_MODE_SET, 0, 0, 3000, 2)) { return RESET; }
	            if (!CAN_SendMessage(CAN_TUBE_KV_SET, CalibrationModeParam.KV, 0, 3000, 2)) { return RESET; }
	            if (!CAN_SendMessage(CAN_TUBE_mA_SET, CalibrationModeParam.mA, 0, 3000, 2)) { return RESET; }
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
					Motor_CephMoveAbsolutePosition(&Motor_S, 400, 50, Motor_S.CurrentStep + nUnder_Offset + nUnder_Init, 50);
				else
					Motor_CephMoveAbsolutePosition(&Motor_S, 400, 50, Motor_S.CurrentStep + nOffset*2, 50);
				bUnder_Offset=RESET;
	            UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_START]");
	            Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
                IntTimer_Delay(100);
                while(!IntTimer_GetStatus());
                UART_SendMessage(DBG_MSG_PC, "SENSOR_S_OUTPUT_ENABLE");
                ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_ENABLE);
	            while(!Exposure_CheckSwitch())
	        	{
	        		if(UART_Exposure_SET()==2) { break; }
					if(Motor_GetStatus(&Motor_S, STATUS_STOP))
					{
						UART_SendMessage(DBG_MSG_PC, "Motor_S End Position");
						break;
					}
	        	}
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
                if (CalibrationModeParam.mode == DET_TYPE_PANORAMA) { UART_SendMessage(DBG_MSG_PC, "DET_TYPE_PANORAMA"); }
                else if (CalibrationModeParam.mode == DET_TYPE_CEPH_SCAN)
                {
                    UART_SendMessage(DBG_MSG_PC, "DET_TYPE_CEPH_SCAN");
                    if (!CAN_Collimator_SendMessage(CMD_FILTER_CENTER, 0, 0, 1000, 2)) { return RESET; }
                }
                else if (CalibrationModeParam.mode == DET_TYPE_CEPH_ONESHOT) { UART_SendMessage(DBG_MSG_PC, "DET_TYPE_CEPH_ONESHOT"); }
                else { UART_SendMessage(DBG_MSG_PC, "DET_TYPE_CT"); }
                if (CalibrationModeParam.mode != DET_TYPE_CEPH_SCAN)
                {
                    if ((CalibrationModeParam.mode == DET_TYPE_PANORAMA) && (CalibrationModeParam.AlignType == COL_TYPE_PANO))
                    { if (!CAN_Collimator_SendMessage(CMD_COLLI_PANOR, 0, 0, 2000, 2)) { return RESET; } }
                    else if ((CalibrationModeParam.mode == DET_TYPE_CT) && (CalibrationModeParam.AlignType == COL_TYPE_CT))
                    { if (!CAN_Collimator_SendMessage(CalibrationModeParam.nFovCmd, 0, 0, 2000, 2)) { return RESET; } }
                    else
                    {
                        if (CalibrationModeParam.AlignType == COL_TYPE_NONE)
                        { if (bClose != SET) { if (!CAN_Collimator_SendMessage(CMD_COLLI_OPEN, 0, 0, 2000, 2)) { return RESET; } } }
                    }
                    IntTimer_Delay(200);
                    while(!IntTimer_GetStatus());
                    if ((CalibrationModeParam.mode == DET_TYPE_CT) && (CalibrationModeParam.AlignType == COL_TYPE_NONE))
                    { if (!CAN_Collimator_SendMessage(CMD_FILTER_CT_CALIB, 0, 0, 1000, 2)) { return RESET; } }
                    else
                    { if (!CAN_Collimator_SendMessage(CMD_FILTER_CENTER, 0, 0, 1000, 2)) { return RESET; } }
                    IntTimer_Delay(200);
                    while(!IntTimer_GetStatus());
                }
                if (!CAN_SendMessage(CAN_TUBE_COM_CHECK, CAN_TUBE_GEN2_COM, 0, 3000, 2))
                { UART_SendMessage(DBG_MSG_PC, "Tube's Communication is error"); return RESET; }
				gMachStat.bTubeCountFlag=RESET;
				if (!CAN_SendMessage(CAN_TUBE_COUNT_CHECK, 0, 0, 1000, 2))
			    { UART_SendMessage(DBG_MSG_PC, "Tube Count Fail."); CurCaptureMode = CAPTURE_CANCEL; return RESET; }
                if (!CAN_SendMessage(CAN_TUBE_CONTI_MODE_SET, 0, 0, 3000, 2)) { return RESET; }
                if (!CAN_SendMessage(CAN_TUBE_KV_SET, CalibrationModeParam.KV, 0, 3000, 2)) { return RESET; }
                if (!CAN_SendMessage(CAN_TUBE_mA_SET, CalibrationModeParam.mA, 0, 3000, 2)) { return RESET; }
                if ((CalibrationModeParam.mode == DET_TYPE_CT) || (CalibrationModeParam.mode == DET_TYPE_PANORAMA))
                { { if (CurCaptureMode == CAPTURE_CANCEL) { return SET; } } }
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
                }
                while(!Exposure_CheckSwitch()) { if(UART_Exposure_SET()==2) { break; } }
                if (CalibrationModeParam.mode == DET_TYPE_CEPH_SCAN)
                {
                    UART_SendMessage(DBG_MSG_PC, "SENSOR_S_OUTPUT_DISABLE");
                    ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE);
                }
                Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
                Tube_CtrlReady(TUBE_READY_DISABLE);
                Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
				gMachStat.bTubeCountFlag=SET;
				if (!CAN_SendMessage(CAN_TUBE_COUNT_CHECK, 0, 0, 1000, 2))
				{ UART_SendMessage(DBG_MSG_PC, "Tube Count Fail."); CurCaptureMode = CAPTURE_CANCEL; return; }
				if(gMachStat.nTubeOldCount==gMachStat.nTubeCount)
				{ UART_SendMessage(DBG_MSG_PC, "Warning : Generator Doesn't Exposed"); }
                UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_END__]");
                bClose = RESET;
            }
        }
		else if (strstr(message.Data, UART_CEPH_ALIGN_SCAN_TEST))
		{
		    double dPulse = 0;
		    uint32_t nStartCnt = 0;
		    uint32_t nEndCnt = 0;
			BitAction check;
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
			    { UART_SendMessage(DBG_MSG_PC, "CMD_COLLI_CEPHSCAN_INIT Error"); return RESET; }
			    if (!CAN_Collimator_SendMessage(CMD_FILTER_CENTER, 0, 0, 1000, 2))
			    { UART_SendMessage(DBG_MSG_PC, "CMD_FILTER_CENTER Error"); return RESET; }
			}
		    Motor_C.RunParam.RunFreq = (300 * MOTOR_C_MICROSTEP) / (CalibrationModeParam.nScanTime * MOTOR_C_PULLEY_DIAMETER * pi);
			CephParam.Time = CalibrationModeParam.nScanTime*1000;
		    dPulse = MOTOR_C_PULLEY_DIAMETER * pi / MOTOR_C_MICROSTEP;
		    nStartCnt = 10;
			nEndCnt = 300 / dPulse;
		    if (!CAN_SendMessage(CAN_TUBE_CONTI_MODE_SET, 0, 0, 1000, 2))
		    { UART_SendMessage(DBG_MSG_PC, "CAN_TUBE_CONTI_MODE_SET Error"); return RESET; }
			if (CalibrationModeParam.KV == 0) { CalibrationModeParam.KV = 80; }
			if (CalibrationModeParam.mA == 0) { CalibrationModeParam.mA = 100; }
		    if (!CAN_SendMessage(CAN_TUBE_KV_SET, CalibrationModeParam.KV, 0, 1000, 2))
		    { UART_SendMessage(DBG_MSG_PC, "Error"); return RESET; }
		    if (!CAN_SendMessage(CAN_TUBE_mA_SET, CalibrationModeParam.mA, 0, 1000, 2))
		    { UART_SendMessage(DBG_MSG_PC, "Error"); return RESET; }
		    Exposure_CtrlLed(EXPOSURE_LED_GPIO_ENABLE);
		    Indicator_Control(INDICATOR_ENABLE);
			UART_SendMessage(DBG_MSG_PC, "[SP_STBY_EXPSW]");
		    while(Exposure_CheckSwitch())
		    {
		    	char cret=0;
		    	if (CurCaptureMode == CAPTURE_CANCEL)
                { Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE); Tube_CtrlReady(TUBE_READY_DISABLE); Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE); UART_SendMessage(DBG_MSG_PC, "SHOT READY CANCELED"); return SET; }
				cret=UART_Exposure_SET();
				if(cret==2)
				{ Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE); Tube_CtrlReady(TUBE_READY_DISABLE); Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE); UART_SendMessage(DBG_MSG_PC, "SHOT READY CANCELED"); return RESET; }
				else if(cret==1) break;
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
					{ printUart(DBG_MSG_PC, "Collimator CEPHSCAN_START_FAST failed...!"); }
				}
				else
				{
					if (!CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB, CMD_COL_CEPHSCAN_START_NORMAL, 0x96, 0))
			        { UART_SendMessage(DBG_MSG_PC, "CMD_COL_CEPHSCAN_START_NORMAL failed...!"); }
				}
			}
			bOpen= RESET;
			Motor_CephScanStart(nEndCnt);
			if(check==Bit_SET)
			{
				while(Motor_C.CurrentStep < nEndCnt + 100)
				{
					if (UART_Exposure_SET()==2) { break; }
					if (Motor_C.CurrentStep == nStartCnt) { Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE); }
					else if (Motor_C.CurrentStep == nStartCnt + 5) { ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_ENABLE); }
					else if(Tube.bXrayFrameOnOff==SET && (Motor_C.CurrentStep == nEndCnt/4)) { Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE); }
					else if(Tube.bXrayFrameOnOff==SET && (Motor_C.CurrentStep == (nEndCnt*3)/4)) { Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE); Tube.bXrayFrameOnOff=RESET; }
					else if (Motor_C.CurrentStep >= nEndCnt) { ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE); Tube_CtrlReady(TUBE_READY_DISABLE); Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE); }
				}
			}
			else
			{
				while(Motor_C.CurrentStep < nEndCnt + 100)
			    {
			        if ((Exposure_CheckSwitch())) { break; }
			        if (Motor_C.CurrentStep == nStartCnt) { Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE); }
					else if (Motor_C.CurrentStep == nStartCnt + 5) { ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_ENABLE); }
					else if(Tube.bXrayFrameOnOff==SET && (Motor_C.CurrentStep == nEndCnt/4)) { Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE); }
					else if(Tube.bXrayFrameOnOff==SET && (Motor_C.CurrentStep == (nEndCnt*3)/4)) { Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE); Tube.bXrayFrameOnOff=RESET; }
			        else if (Motor_C.CurrentStep >= nEndCnt) { ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE); Tube_CtrlReady(TUBE_READY_DISABLE); Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE); }
			    }
			}
			Motor_CephScanEnd();
			Tube.bXrayFrameOnOff=RESET;
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
        else if (strstr(message.Data, UART_HEAD_BEAM_ON)) { LaserControl(TYPE_HEAD_CEPH, SET); }
        else if (strstr(message.Data, UART_HEAD_BEAM_OFF)) { LaserControl(TYPE_HEAD_CEPH, RESET); }
		else if (strstr(message.Data, UART_CANINE_BEAM_ON))
        { if (!CAN_Collimator_SendMessage(CMD_LASER_CANINE_ON, 0, 0, 5000, 0)) { return RESET; } }
		else if (strstr(message.Data, UART_CANINE_BEAM_OFF))
        { if (!CAN_Collimator_SendMessage(CMD_LASER_CANINE_OFF, 0, 0, 5000, 0)) { return RESET; } }
		else if (strstr(message.Data, UART_ALIGN_BEAM_COLI_REQ))
		{ if (!CAN_Collimator_SendMessage(CMD_REQ_OFFSET_HBEAM_LASER, 0, 0, 5000, 0)) { return RESET; } }
		else if (strstr(message.Data, UART_ALIGN_BEAM_SET))
		{ if (!CAN_Collimator_SendMessage(CMD_OFFSET_HBEAM_LASER_SET, 0, 0, 5000, 0)) { return RESET; } }
		else if (strstr(message.Data, UART_ALIGN_BEAM_COLI))
		{ if (!CAN_Collimator_SendMessage(CMD_OFFSET_HBEAM_LASER, 0, 0, 5000, 0)) { return RESET; } }
		else if (strstr(message.Data, UART_BEAM_POS_UP))
		{ if (!CAN_Collimator_SendMessage(CMD_HBEAM_LASER_UP, 0, 0, 1000, 0)) { return RESET; } }
		else if (strstr(message.Data, UART_BEAM_POS_DOWN))
		{ if (!CAN_Collimator_SendMessage(CMD_HBEAM_LASER_DOWN, 0, 0, 1000, 0)) { return RESET; } }
        else if (strstr(message.Data, UART_COLI_VERSION_CHECK))
        { if (!CAN_Collimator_SendMessage(CMD_VERSION_CHECK, 0, 0, 3000, 0)) { printUart(DBG_MSG_PC, "Version Check Error"); } }
        else if (strstr(message.Data, UART_COLI_BUILD_CHECK))
        { if (!CAN_Collimator_SendMessage(CMD_BUILD_CHECK, 0, 0, 3000, 0)) { printUart(DBG_MSG_PC, "Build Check Error"); } }
		else if (strstr(message.Data, UART_TUBE_VERSION_CHECK))
        { if (!CAN_SendMessage(CAN_TUBE_VER_CHECK, 0, 0, 3000, 0)) { printUart(DBG_MSG_PC, "Version Check Error"); } }
		else if (strstr(message.Data, UART_GET_TUBE_TEMP))
		{ CAN_SendMessage(CAN_TUBE_TANK_TEMP_READ, 0, 0, 1000, 2); }
		else if (strstr(message.Data, UART_COLLIMATOR_FILTER_RIGHT))
		{ if (!CAN_Collimator_SendMessage(CMD_FILTER_CT, 0, 0, 1000, 2)) { return RESET; } }
		else if (strstr(message.Data, UART_COLLIMATOR_FILTER_CENTER))
		{ if (!CAN_Collimator_SendMessage(CMD_FILTER_CENTER, 0, 0, 1000, 2)) { return RESET; } }
		else if (strstr(message.Data, UART_COLLIMATOR_FILTER_LEFT))
		{ if (!CAN_Collimator_SendMessage(CMD_FILTER_CT_CALIB, 0, 0, 1000, 2)) { return RESET; } }
#ifdef USE_MOTOR_CHINREST_HOR
		else if (strstr(message.Data, UART_CMD_CHINREST_HOR_MOVE))
		{
			{
				int32_t nDistance = atoi(&(message.Data[12]));
				printUart(DBG_MSG_PC, "H-Axis Distance = %dmm", nDistance);
				if (nDistance >= 0)
				{
					if (strchr(&(message.Data[11]), 'R'))
						nDistance = ~nDistance + 1;
					Motor_ControlChinrest(RESET, MOTOR_CWE_INIT_POSITION + nDistance);
				}
			}
		}
#endif
#ifdef USE_MOTOR_CHINREST_VER
		else if (strstr(message.Data, UART_CMD_CHINREST_VER_MOVE))
		{
			int32_t nDistance = atoi(&(message.Data[12]));
			printUart(DBG_MSG_PC, "V-Axis Distance = %dmm", nDistance);
			if (nDistance >= 0)
			{
				if (strchr(&(message.Data[11]), 'D'))
					nDistance = ~nDistance + 1;
				Motor_ControlChinrest(SET, MOTOR_CNS_CT_INIT_POSITION + nDistance);
			}
		}
#endif
		else if(strstr(message.Data, UART_TUBE_TILT_OFF))
		{ if (!CAN_Collimator_SendMessage(CMD_TILT_OFF, 0, 0, 5000, 2)) { return RESET; } }
		else if(strstr(message.Data, UART_TUBE_TILT_ON))
		{ if (!CAN_Collimator_SendMessage(CMD_TILT_ON, 0, 0, 5000, 2)) { return RESET; } }
		else if(strstr(message.Data, UART_RAXIS_ROOTIN))
		{
			int16_t nCnt = atoi(&(message.Data[12]));
			int16_t nCntDis=0;
			printUart(DBG_MSG_PC, "R-Axis Rootin Count :", nCnt);
			if (!CAN_Collimator_SendMessage(CMD_TILT_OFF, 0, 0, 5000, 2)) { return RESET; }
			IntTimer_Stop();
			while(nCnt>0)
			{
				nCnt--;
				nCntDis++;
				TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
				Motor_RunOrgCheck(&Motor_R);
				while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
				TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StopCurrent);
				IntTimer_Delay(1500);
	   			while(!IntTimer_GetStatus());
				printUart(DBG_MSG_PC, "Rootin Current : %d", nCntDis);
			}
		}
		else if(strstr(message.Data, UART_ALIGN_AUTO_COLLIMATOR))
		{
			int16_t nCnt =0;
			uint8_t choose = 0;
			UART_SendMessage(DBG_MSG_PC, "COLLI AUTO ALIGN START");
			while(1)
			{
				choose =UART_Collimator_Auto_SET();
				if(choose !=0 ) break;
				if (CurCaptureMode == CAPTURE_CANCEL)
                { UART_SendMessage(DBG_MSG_PC, "COLLI AUTO ALIGN CANCELED"); return SET; }
			}
			nCnt=CalibrationModeParam.nColli_Distance;
			printUart(DBG_MSG_PC, "Collimator Range : %d", nCnt);
			switch(choose)
			{
				case 1: if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_POSITION, nCnt, 0, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align Position failed...!"); return RESET; } break;
				case 6: if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_POSITION, nCnt, 1, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align Position failed...!"); return RESET; } break;
				case 7: if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_POSITION, nCnt, 2, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align Position failed...!"); return RESET; } break;
				case 8: if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_POSITION, nCnt, 3, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align Position failed...!"); return RESET; } break;
				case 2: if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_TOP_POSITION, nCnt, 0, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align Position failed...!"); return RESET; } break;
				case 3: if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_BOTTOM_POSITION, nCnt, 0, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align Position failed...!"); return RESET; } break;
				case 4: if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_LEFT_POSITION, nCnt, 0, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align Position failed...!"); return RESET; } break;
				case 5: if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_RIGHT_POSITION, nCnt, 0, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align Position failed...!"); return RESET; } break;
				default: break;
			}
			if (CalibrationModeParam.KV == 0) { CalibrationModeParam.KV = 80; }
			if (CalibrationModeParam.mA == 0) { CalibrationModeParam.mA = 80; }
		    if (!CAN_SendMessage(CAN_TUBE_KV_SET, CalibrationModeParam.KV, 0, 1000, 2)) { UART_SendMessage(DBG_MSG_PC, "KV_SET_Error"); return RESET; }
		    if (!CAN_SendMessage(CAN_TUBE_mA_SET, CalibrationModeParam.mA, 0, 1000, 2)) { UART_SendMessage(DBG_MSG_PC, "mA_SET_Error"); return RESET; }
		    Exposure_CtrlLed(EXPOSURE_LED_GPIO_ENABLE);
		    Indicator_Control(INDICATOR_ENABLE);
			UART_SendMessage(DBG_MSG_PC, "[SP_STBY_EXPSW]");
		    while(Exposure_CheckSwitch())
		    {
		    	char cret=0;
				cret=UART_Exposure_SET();
				if(cret==2)
				{ Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE); Tube_CtrlReady(TUBE_READY_DISABLE); Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE); UART_SendMessage(DBG_MSG_PC, "COLLI AUTO ALIGN CANCELED"); return RESET; }
				else if(cret==1) break;
		    	if (CurCaptureMode == CAPTURE_CANCEL)
                { Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE); Tube_CtrlReady(TUBE_READY_DISABLE); Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE); UART_SendMessage(DBG_MSG_PC, "COLLI AUTO ALIGN CANCELED"); return SET; }
		    }
		    Tube_CtrlReady(TUBE_READY_ENABLE);
		    UART_SendMessage(DBG_MSG_PC, "Wait for Tube Heatting Time");
		    IntTimer_Delay(2000);
		    while(!IntTimer_GetStatus());
			ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_ENABLE);
		    UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_START]");
			switch(choose)
			{
				case 1: if (!CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB, CMD_COLLI_CALIBRATION_MOVE, nCnt, 0)) { printUart(DBG_MSG_PC, "Collimator Align MOVE failed...!"); } break;
				case 2: if (!CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB, CMD_COLLI_CALIBRATION_TOP_MOVE, nCnt, 0)) { printUart(DBG_MSG_PC, "Collimator Align MOVE failed...!"); } break;
				case 3: if (!CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB, CMD_COLLI_CALIBRATION_BOTTOM_MOVE, nCnt, 0)) { printUart(DBG_MSG_PC, "Collimator Align MOVE failed...!"); } break;
				case 4: if (!CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB, CMD_COLLI_CALIBRATION_LEFT_MOVE, nCnt, 0)) { printUart(DBG_MSG_PC, "Collimator Align MOVE failed...!"); } break;
				case 5: if (!CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB, CMD_COLLI_CALIBRATION_RIGHT_MOVE, nCnt, 0)) { printUart(DBG_MSG_PC, "Collimator Align MOVE failed...!"); }
				case 6: if (!CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB, CMD_COLLI_CALIBRATION_PANO_MOVE, nCnt, 0)) { printUart(DBG_MSG_PC, "Collimator Align MOVE failed...!"); }
				case 7: if (!CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB, CMD_COLLI_CALIBRATION_CT_MOVE, nCnt, 0)) { printUart(DBG_MSG_PC, "Collimator Align MOVE failed...!"); }
				case 8: if (!CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB, CMD_COLLI_CALIBRATION_SCAN_MOVE, nCnt, 0)) { printUart(DBG_MSG_PC, "Collimator Align MOVE failed...!"); } break;
				default: break;
			}
			Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
			IntTimer_Stop();
			IntTimer_Delay(20000);
			while(!Exposure_CheckSwitch())
		    {
		    	char cret=0;
		        cret=UART_Collimator_Auto_STOP();
				if(cret==1) { if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_STOP, 0, 0, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align Stop failed...!"); break; } }
				else if(cret==2) { if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_PANO_TOP_STOP, 0, 0, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align TOP Stop failed...!"); break; } }
				else if(cret==3) { if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_PANO_BOTTOM_STOP, 0, 0, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align BOTTOM Stop failed...!"); break; } }
				else if(cret==4) { if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_PANO_LEFT_STOP, 0, 0, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align LEFT Stop failed...!"); break; } }
				else if(cret==5) { if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_PANO_RIGHT_STOP, 0, 0, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align RIGHT Stop failed...!"); break; } }
				else if(cret==6) { if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_CT_TOP_STOP, 0, 0, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align TOP Stop failed...!"); break; } }
				else if(cret==7) { if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_CT_BOTTOM_STOP, 0, 0, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align BOTTOM Stop failed...!"); break; } }
				else if(cret==8) { if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_CT_LEFT_STOP, 0, 0, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align LEFT Stop failed...!"); break; } }
				else if(cret==9) { if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_CT_RIGHT_STOP, 0, 0, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align RIGHT Stop failed...!"); break; } }
				else if(cret==10) { if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_SCAN_TOP_STOP, 0, 0, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align TOP Stop failed...!"); break; } }
				else if(cret==11) { if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_SCAN_BOTTOM_STOP, 0, 0, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align BOTTOM Stop failed...!"); break; } }
				else if(cret==12) { if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_SCAN_LEFT_STOP, 0, 0, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align LEFT Stop failed...!"); break; } }
				else if(cret==13) { if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_SCAN_RIGHT_STOP, 0, 0, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align RIGHT Stop failed...!"); break; } }
				else if(cret==14) { if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_STOP, 0, 0, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align Stop failed...!"); break; } break; }
				if(IntTimer_GetStatus()) { if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_STOP, 0, 0, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align Stop failed...!"); break; } break; }
				if (CurCaptureMode == CAPTURE_CANCEL)
                {
                	Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
            		Tube_CtrlReady(TUBE_READY_DISABLE);
					ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE);
            		Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
					Indicator_Control(INDICATOR_DISABLE);
					if (!CAN_Collimator_SendMessage(CMD_COLLI_CALIBRATION_STOP, 0, 0, 1000, 2)) { printUart(DBG_MSG_PC, "Collimator Align Stop failed...!"); break; }
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

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/
