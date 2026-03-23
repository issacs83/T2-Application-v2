/*
*******************************************************************************
* cmd_geoalign_axes.c : Axis-specific geometry alignment command handlers
*******************************************************************************
* Copyright (C) 2014-2018 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Brief   : R-axis, V-axis, Ceph R-axis, CT axes, CNS/CWE axes, T-axis
*             handlers extracted from the UART_SetGeoAlignParam if-else chain.
*
* @ Revision History :
*       1) Extracted from cmd_geoalign_cal.c for modular build --- 2026-03-24
*******************************************************************************
*/

/* Include files ----------------------------------------------------------- */
#include "cmd_geoalign_cal_priv.h"

/* =========================================================================
 * GeoAlign_HandleRAxis -- R-axis rotation / offset commands
 * ========================================================================= */
bool GeoAlign_HandleRAxis(UART_MsgTypedef *message)
{
    if (strstr(message->Data, UART_GEO_ALIGN_RAXIS_ROT00))
    {
        TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
        Motor_MoveAbsolutePosition(&Motor_R, 3000, 1500, GEOMETRY_ALIGN_RAXIS_ROTATE_00D, 1500);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_RAXIS_ROT90))
    {
        TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
        Motor_MoveAbsolutePosition(&Motor_R, 3000, 1500, GEOMETRY_ALIGN_RAXIS_ROTATE_90D, 1500);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_RAXIS_STEP_UP))
    {
        TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
        Motor_MoveAbsolutePosition(&Motor_R, 50, 100, Motor_R.CurrentStep + GEOMETRY_ALIGN_RAXIS_STEP_ONE, 100);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_RAXIS_STEP_DOWN))
    {
        TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
        Motor_MoveAbsolutePosition(&Motor_R, 50, 100, Motor_R.CurrentStep - GEOMETRY_ALIGN_RAXIS_STEP_ONE, 100);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_RAXIS_STEP_STOP))
    {
        Motor_R.Update=RESET;
        Motor_Stop(&Motor_R);
        TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StopCurrent);
        printUart(DBG_MSG_PC, "Motor_R.CurrentStep(%d)", Motor_R.CurrentStep);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_RAXIS_SET))
    {
        int16_t nOffset = 0;

        nOffset=EEPRom_I2C_Read_Word(ROM_RAXIS_ALG_ADDR);
        if(nOffset==0xffff)
        {
            nOffset=0;
        }
        else if(nOffset<0) nOffset++;

        if (Motor_R.CurrentStep < 75.0 / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))
        {
#ifdef USE_I2C_EEPROM
            nOffset = nOffset + (Motor_R.CurrentStep - GEOMETRY_ALIGN_RAXIS_ROTATE_00D);
#else
            nOffset = Motor_R.CurrentStep - GEOMETRY_ALIGN_RAXIS_ROTATE_00D;
#endif
        }
        else
        {
#ifdef USE_I2C_EEPROM
            nOffset = nOffset + (Motor_R.CurrentStep - GEOMETRY_ALIGN_RAXIS_ROTATE_90D);
#else
            nOffset = Motor_R.CurrentStep - GEOMETRY_ALIGN_RAXIS_ROTATE_90D;
#endif
        }
        printUart(DBG_MSG_PC, "Current Step is %d.", nOffset);

#ifdef USE_I2C_EEPROM
        EEPRom_I2C_Write_Word(ROM_RAXIS_ALG_ADDR, (uint16_t)nOffset);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "R_AXIS_ADDR(0x%02x) : 0x%04x", ROM_RAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_RAXIS_ALG_ADDR));
#endif
        PanoParam.nRAxisOffset = nOffset;
        printUart(DBG_MSG_PC, "nRAxisOffset = %d", PanoParam.nRAxisOffset);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_RAXIS_RESET))
    {
        int16_t nOffset = 0;
#ifdef USE_I2C_EEPROM
        EEPRom_EraseAddrWord(ROM_RAXIS_ALG_ADDR);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "R_AXIS_ADDR(0x%02x) : 0x%04x", ROM_RAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_RAXIS_ALG_ADDR));
#endif
        PanoParam.nRAxisOffset = nOffset;
        printUart(DBG_MSG_PC, "nRAxisOffset = %d", PanoParam.nRAxisOffset);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_RAXIS_DEC_OFFSET))
    {
        int16_t nOffset = 0, ReadOffset = 0;
        nOffset -= atoi(&(message->Data[9]));
        printUart(DBG_MSG_PC, "nOffset = %d", nOffset);
#ifdef USE_I2C_EEPROM
        ReadOffset = (int16_t)EEPRom_I2C_Read_Word(ROM_RAXIS_ALG_ADDR);
        if(ReadOffset==0xffff) { ReadOffset=0; }
        else if(ReadOffset<0) ReadOffset++;
        nOffset=ReadOffset+nOffset;
        EEPRom_I2C_Write_Word(ROM_RAXIS_ALG_ADDR, (uint16_t)nOffset);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "R_AXIS_ADDR(0x%02x) : 0x%04x", ROM_RAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_RAXIS_ALG_ADDR));
#endif
        PanoParam.nRAxisOffset = nOffset;
        printUart(DBG_MSG_PC, "nRAxisOffset = %d", PanoParam.nRAxisOffset);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_RAXIS_INC_OFFSET))
    {
        int16_t nOffset = 0, ReadOffset = 0;
        nOffset += atoi(&(message->Data[9]));
        printUart(DBG_MSG_PC, "nOffset = %d", nOffset);
#ifdef USE_I2C_EEPROM
        ReadOffset = (int16_t)EEPRom_I2C_Read_Word(ROM_RAXIS_ALG_ADDR);
        if(ReadOffset==0xffff) { ReadOffset=0; }
        else if(ReadOffset<0) ReadOffset++;
        nOffset=ReadOffset+nOffset;
        EEPRom_I2C_Write_Word(ROM_RAXIS_ALG_ADDR, (uint16_t)nOffset);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "R_AXIS_ADDR(0x%02x) : +0x%04x", ROM_RAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_RAXIS_ALG_ADDR));
#endif
        PanoParam.nRAxisOffset = nOffset;
        printUart(DBG_MSG_PC, "nRAxisOffset = %d", PanoParam.nRAxisOffset);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_GET_RAXIS_OFFSET))
    {
        int16_t nOffset = 0;
        nOffset=EEPRom_I2C_Read_Word(ROM_RAXIS_ALG_ADDR);
        if(nOffset==0xffff) { nOffset=0; }
        else if(nOffset<0) nOffset++;
        PanoParam.nRAxisOffset=nOffset;
        printUart(DBG_MSG_PC, "Current nRAxisOffset = %d", PanoParam.nRAxisOffset);
    }

    return SET;
}

/* =========================================================================
 * GeoAlign_HandleVAxis -- V-axis + canine alignment commands
 * ========================================================================= */
bool GeoAlign_HandleVAxis(UART_MsgTypedef *message)
{
    if (strstr(message->Data, UART_GEO_ALIGN_VAXIS_STEP_UP))
    {
        TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
        Motor_MoveAbsolutePosition(&Motor_V, 200, 100, Motor_V.CurrentStep + GEOMETRY_ALIGN_VAXIS_STEP_ONE, 100);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_VAXIS_STEP_DOWN))
    {
        TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
        Motor_MoveAbsolutePosition(&Motor_V, 200, 100, Motor_V.CurrentStep - GEOMETRY_ALIGN_VAXIS_STEP_ONE, 100);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_VAXIS_DEC_OFFSET))
    {
        int16_t nOffset = 0, ReadOffset = 0;
        nOffset -= atoi(&(message->Data[9]));
        printUart(DBG_MSG_PC, "nOffset = %d", nOffset);
#ifdef USE_I2C_EEPROM
        ReadOffset = (int16_t)EEPRom_I2C_Read_Word(ROM_PAXIS_ALG_ADDR);
        if(ReadOffset==0xffff) { ReadOffset=0; }
        else if(ReadOffset<0) ReadOffset++;
        nOffset=ReadOffset+nOffset;
        EEPRom_I2C_Write_Word(ROM_PAXIS_ALG_ADDR, (uint16_t)nOffset);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "PAXIS_ALG_ADDR(0x%02x) : 0x%04x", ROM_PAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_PAXIS_ALG_ADDR));
#endif
        PanoParam.nVAxisOffset = nOffset;
        printUart(DBG_MSG_PC, "nVAxisOffset = %d", PanoParam.nVAxisOffset);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_VAXIS_INC_OFFSET))
    {
        int16_t nOffset = 0, ReadOffset = 0;
        nOffset += atoi(&(message->Data[9]));
        printUart(DBG_MSG_PC, "nOffset = +%d", nOffset);
#ifdef USE_I2C_EEPROM
        ReadOffset = (int16_t)EEPRom_I2C_Read_Word(ROM_PAXIS_ALG_ADDR);
        if(ReadOffset==0xffff) { ReadOffset=0; }
        else if(ReadOffset<0) ReadOffset++;
        nOffset=ReadOffset+nOffset;
        EEPRom_I2C_Write_Word(ROM_PAXIS_ALG_ADDR, (uint16_t)nOffset);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "nVAxisOffset(0x%02x) : +0x%04x", ROM_PAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_PAXIS_ALG_ADDR));
#endif
        PanoParam.nVAxisOffset = nOffset;
        printUart(DBG_MSG_PC, "nVAxisOffset = +%d", PanoParam.nVAxisOffset);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_GET_VAXIS_OFFSET))
    {
        int16_t nOffset = 0;
        nOffset=EEPRom_I2C_Read_Word(ROM_PAXIS_ALG_ADDR);
        if(nOffset==0xffff) { nOffset=0; }
        else if(nOffset<0) nOffset++;
        PanoParam.nVAxisOffset=nOffset;
        printUart(DBG_MSG_PC, "Current nVAxisOffset = %d", PanoParam.nVAxisOffset);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_VAXIS_STEP_STOP))
    {
        Motor_V.Update=FALSE;
        Motor_Stop(&Motor_V);
        TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StopCurrent);
        printUart(DBG_MSG_PC, "Motor_V.CurrentStep(%d)", Motor_V.CurrentStep);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_VAXIS_SET))
    {
        int16_t nOffset = 0;
#ifdef USE_I2C_EEPROM
        nOffset = (int16_t)EEPRom_I2C_Read_Word(ROM_PAXIS_ALG_ADDR);
        if(nOffset==0xffff) { nOffset=0; }
        else if(nOffset<0) nOffset++;
        nOffset= nOffset+ (Motor_V.CurrentStep - PANO_V_STANDARD_OFFSET);
        printUart(DBG_MSG_PC, "nOffset = %d", nOffset);
        EEPRom_I2C_Write_Word(ROM_PAXIS_ALG_ADDR, (uint16_t)nOffset);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "VAXIS_ALG_ADDR(0x%02x) : 0x%04x", ROM_PAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_PAXIS_ALG_ADDR));
#endif
        PanoParam.nVAxisOffset = nOffset;
        printUart(DBG_MSG_PC, "nVAxisOffset = %d", PanoParam.nVAxisOffset);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_VAXIS_RESET))
    {
        int16_t nOffset = 0;
#ifdef USE_I2C_EEPROM
        EEPRom_EraseAddrWord(ROM_PAXIS_ALG_ADDR);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "VAXIS_ALG_ADDR(0x%02x) : 0x%04x", ROM_PAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_PAXIS_ALG_ADDR));
#endif
        PanoParam.nVAxisOffset = nOffset;
        printUart(DBG_MSG_PC, "nVAxisOffset = %d", PanoParam.nVAxisOffset);
    }
    else if (strstr(message->Data, UART_CANINE_ALIGN_MOVE_VAXIS_INIT))
    {
        TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
        Motor_RunOrgCheck(&Motor_V);
        while(!Motor_GetStatus(&Motor_V, STATUS_STOP));
        TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StopCurrent);
        PanoParam.nCanineOffset = Motor_V.CurrentStep;
        printUart(DBG_MSG_PC, "nCanineOffset = %d", PanoParam.nCanineOffset);
    }
    else if (strstr(message->Data, UART_CANINE_ALIGN_VAXIS_STEP_UP))
    {
        printUart(DBG_MSG_PC, "CurStep = %d", Motor_V.CurrentStep);
        TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
        Motor_MoveAbsolutePosition(&Motor_V, 200, 10, Motor_V.CurrentStep + CANINE_ALIGN_VAXIS_STEP, 10);
    }
    else if (strstr(message->Data, UART_CANINE_ALIGN_VAXIS_STEP_DOWN))
    {
        printUart(DBG_MSG_PC, "CurStep = %d", Motor_V.CurrentStep);
        TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
        Motor_MoveAbsolutePosition(&Motor_V, 200, 10, Motor_V.CurrentStep - CANINE_ALIGN_VAXIS_STEP, 10);
    }
    else if (strstr(message->Data, UART_CANINE_ALIGN_VAXIS_STOP))
    {
        Motor_V.Update=FALSE;
        Motor_Stop(&Motor_V);
        TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StopCurrent);
        printUart(DBG_MSG_PC, "CurStep = %d", Motor_V.CurrentStep);
    }
    else if (strstr(message->Data, UART_CANINE_ALIGN_VAXIS_SET))
    {
        int16_t nOffset = (int16_t)Motor_V.CurrentStep;
#ifdef USE_I2C_EEPROM
        EEPRom_I2C_Write_Word(ROM_CANINE_ALIGN_ADDR, (uint16_t)nOffset);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "CANINE_ALIGN_ADDR(0x%02x) : 0x%04x", ROM_CANINE_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CANINE_ALIGN_ADDR));
#endif
        PanoParam.nCanineOffset = nOffset;
        printUart(DBG_MSG_PC, "nCanineOffset = %d", PanoParam.nCanineOffset);
    }
    else if (strstr(message->Data, UART_CANINE_ALIGN_VAXIS_RESET))
    {
        int16_t nOffset = 0;
#ifdef USE_I2C_EEPROM
        EEPRom_EraseAddrWord(ROM_CANINE_ALIGN_ADDR);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "CANINE_ALIGN_ADDR(0x%02x) : 0x%04x", ROM_CANINE_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CANINE_ALIGN_ADDR));
#endif
        PanoParam.nCanineOffset = nOffset;
        printUart(DBG_MSG_PC, "nCanineOffset = %d", PanoParam.nCanineOffset);
    }

    return SET;
}

/* =========================================================================
 * GeoAlign_HandleCephRAxis -- Ceph R-axis alignment + position move
 * ========================================================================= */
bool GeoAlign_HandleCephRAxis(UART_MsgTypedef *message)
{
    if (strstr(message->Data, UART_GEO_CEPH_ALIGN_RAXIS_STEP_UP))
    {
        TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
        Motor_MoveAbsolutePosition(&Motor_R, 50, 100, Motor_R.CurrentStep + GEOMETRY_ALIGN_RAXIS_STEP_ONE, 100);
    }
    else if (strstr(message->Data, UART_GEO_CEPH_ALIGN_RAXIS_STEP_DOWN))
    {
        TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
        Motor_MoveAbsolutePosition(&Motor_R, 50, 100, Motor_R.CurrentStep - GEOMETRY_ALIGN_RAXIS_STEP_ONE, 100);
    }
    else if (strstr(message->Data, UART_GEO_CEPH_ALIGN_RAXIS_MANUAL_UP))
    {
        int16_t nOffset = 0;
        nOffset += atoi(&(message->Data[9]));
#ifdef USE_I2C_EEPROM
        EEPRom_I2C_Write_Word(ROM_CEPH_RAXIS_ALIGN_ADDR, (uint16_t)nOffset);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "CEPH_RAXIS_ALIGN_ADDR(0x%02x) : +0x%04x", ROM_CEPH_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_RAXIS_ALIGN_ADDR));
#endif
        CephParam.nRAxisOffset = nOffset;
        printUart(DBG_MSG_PC, "Ceph nRAxisOffset = +%d", CephParam.nRAxisOffset);
    }
    else if (strstr(message->Data, UART_GEO_CEPH_ALIGN_RAXIS_MANUAL_DOWN))
    {
        int16_t nOffset = 0;
        nOffset -= atoi(&(message->Data[9]));
        printUart(DBG_MSG_PC, "Ceph nOffset = %d", nOffset);
#ifdef USE_I2C_EEPROM
        EEPRom_I2C_Write_Word(ROM_CEPH_RAXIS_ALIGN_ADDR, (uint16_t)nOffset);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "CEPH_RAXIS_ALIGN_ADDR(0x%02x) : 0x%04x", ROM_CEPH_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_RAXIS_ALIGN_ADDR));
#endif
        CephParam.nRAxisOffset = nOffset;
        printUart(DBG_MSG_PC, "Ceph nRAxisOffset = %d", CephParam.nRAxisOffset);
    }
    else if (strstr(message->Data, UART_GEO_CEPH_ALIGN_RAXIS_STEP_STOP))
    {
        char str[32] = {0,};
        Motor_R.Update=FALSE;
        Motor_Stop(&Motor_R);
        TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StopCurrent);
        sprintf(str, "Ceph Motor_R.CurrentStep(%d)", Motor_R.CurrentStep);
        UART_SendMessage(DBG_MSG_PC, str);
    }
    else if (strstr(message->Data, UART_GEO_CEPH_ALIGN_RAXIS_SET))
    {
        int16_t nOffset = 0;
        if (Motor_R.CurrentStep < 75.0 / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))
        {
#ifdef USE_I2C_EEPROM
            nOffset = Motor_R.CurrentStep - GEOMETRY_ALIGN_RAXIS_ROTATE_00D;
#else
            nOffset = Motor_R.CurrentStep - GEOMETRY_ALIGN_RAXIS_ROTATE_00D;
#endif
        }
        else
        {
#if defined(USE_I2C_EEPROM)
            nOffset = Motor_R.CurrentStep - (CEPH_ROTATE_ANGLE/ (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO));
#else
            nOffset = Motor_R.CurrentStep - (CEPH_ROTATE_ANGLE / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO));
#endif
        }
        printUart(DBG_MSG_PC, "Ceph nOffset = %d", nOffset);
        if( nOffset == 0xffff ) { nOffset = 0; }
#ifdef USE_I2C_EEPROM
        EEPRom_I2C_Write_Word(ROM_CEPH_RAXIS_ALIGN_ADDR, (uint16_t)nOffset);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "CEPH_RAXIS_ALIGN_ADDR(0x%02x) : 0x%04x", ROM_CEPH_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_RAXIS_ALIGN_ADDR));
#endif
        CephParam.nRAxisOffset = nOffset;
        printUart(DBG_MSG_PC, "Ceph nRAxisOffset = %d", CephParam.nRAxisOffset);
    }
    else if (strstr(message->Data, UART_GEO_CEPH_ALIGN_RAXIS_RESET))
    {
        int16_t nOffset = 0;
#ifdef USE_I2C_EEPROM
        EEPRom_EraseAddrWord(ROM_CEPH_RAXIS_ALIGN_ADDR);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "CEPH_RAXIS_ALIGN_ADDR(0x%02x) : 0x%04x", ROM_CEPH_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_RAXIS_ALIGN_ADDR));
#endif
        CephParam.nRAxisOffset = nOffset;
        printUart(DBG_MSG_PC, "Ceph nRAxisOffset = %d", CephParam.nRAxisOffset);
    }
    else if (strstr(message->Data, UART_GEO_CEPH_ALIGN_RAXIS_OFFSET_READ))
    {
        int16_t nOffset = 0;
#ifdef USE_I2C_EEPROM
        nOffset = EEPRom_I2C_Read_Word(ROM_CEPH_RAXIS_ALIGN_ADDR);
        if(nOffset==0xffff) { nOffset=0; }
        else if(nOffset<0) nOffset++;
        CephParam.nRAxisOffset = nOffset;
        printUart(DBG_MSG_PC, "R_AXIS_ADDR(0x%02x) : 0x%04x", ROM_CEPH_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_RAXIS_ALIGN_ADDR));
#endif
        printUart(DBG_MSG_PC, "nRAxisOffset = %d", CephParam.nRAxisOffset);
    }
    else if (strstr(message->Data, UART_GEO_CEPH_ALIGN_RAXIS_OFFSET_INIT))
    {
#ifdef USE_I2C_EEPROM
        EEPRom_EraseAddrWord(ROM_CEPH_RAXIS_ALIGN_ADDR);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "CEPH_RAXIS_ALIGN_ADDR(0x%02x) : 0x%04x", ROM_CEPH_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CEPH_RAXIS_ALIGN_ADDR));
#endif
        CephParam.nRAxisOffset = 0;
        printUart(DBG_MSG_PC, "Ceph nRAxisOffset = %d", CephParam.nRAxisOffset);
    }
    else if (strstr(message->Data, UART_GEO_CEPH_POSITION_MOVE))
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

    return SET;
}

/* =========================================================================
 * GeoAlign_HandleCTAxes -- CT P-axis, R-axis, fast R-axis, patient axes
 * ========================================================================= */
bool GeoAlign_HandleCTAxes(UART_MsgTypedef *message)
{
    if (strstr(message->Data, UART_CT_ALIGN_MOVE_CT_POS))
    {
        printUart(DBG_MSG_PC, "wait until the movement of the motor is finished...");
        TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
        TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
        TMC2660_SetCurrent(Motor_A.MotType, Motor_A.StartCurrent);
        Motor_MoveAbsolutePosition(&Motor_V, 15000, 1000, (CT_PAXIS_DIAMETER / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP)) + CtParam.nPaxisPatientOffset, 2000);
        Motor_MoveAbsolutePosition(&Motor_R, 2000, 1000, CT_RAXIS_OFFSET + CtParam.nRaxisPatientOffset, 1000);
#ifdef USE_MOTOR_GANTRY_MS
        Motor_MoveAbsolutePosition(&Motor_A, 12000, 1000, CT_AUTO_ANGLE / (360.0 / MOTOR_A_MICROSTEP / MOTOR_A_PULLEY_RATIO), 1000);
#endif
        while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
        while(!Motor_GetStatus(&Motor_V, STATUS_STOP));
#ifdef USE_MOTOR_GANTRY_MS
        while(!Motor_GetStatus(&Motor_A, STATUS_STOP));
#endif
        TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StopCurrent);
        TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StopCurrent);
        TMC2660_SetCurrent(Motor_A.MotType, Motor_A.StopCurrent);
        printUart(DBG_MSG_PC, "motor movement is completed");
    }
    else if (strstr(message->Data, UART_CT_ALIGN_PAXIS_SET_DOWN))
    {
        signed short nOffset = (signed short)atoi(&(message->Data[11]));
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
#endif
        } else {
            printUart(DBG_MSG_PC, "wrong step value : (%d)", CtParam.nPaxisOffset);
        }
    }
    else if (strstr(message->Data, UART_CT_ALIGN_PAXIS_SET_UP))
    {
        signed short nOffset = (signed short)atoi(&(message->Data[11]));
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
#endif
        } else {
            printUart(DBG_MSG_PC, "wrong step value : (%d)", CtParam.nPaxisOffset);
        }
    }
    else if (strstr(message->Data, UART_CT_ALIGN_PAXIS_RESET))
    {
#ifdef USE_I2C_EEPROM
        EEPRom_EraseAddrWord(ROM_CT_PAXIS_ALIGN_ADDR);
        CtParam.nPaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_PAXIS_ALIGN_ADDR);
        if (CtParam.nPaxisOffset == (signed short)0xffff) { CtParam.nPaxisOffset = 0; }
#endif
    }
    else if (strstr(message->Data, UART_CT_ALIGN_PAXIS_GET_OFFSET))
    {
#ifdef USE_I2C_EEPROM
        CtParam.nPaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_PAXIS_ALIGN_ADDR);
        if (CtParam.nPaxisOffset == (signed short)0xffff) { CtParam.nPaxisOffset = 0; }
        else if(CtParam.nPaxisOffset<0) CtParam.nPaxisOffset++;
#endif
        printUart(DBG_MSG_PC, "CT P-Axis Offset = %d", CtParam.nPaxisOffset);;
    }
    else if (strstr(message->Data, UART_CT_ALIGN_RAXIS_SET_DOWN))
    {
        signed short nOffset = (signed short)atoi(&(message->Data[11]));
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
#endif
        } else {
            printUart(DBG_MSG_PC, "wrong step value : (%d)", CtParam.nRaxisOffset);
        }
    }
    else if (strstr(message->Data, UART_CT_ALIGN_RAXIS_SET_UP))
    {
        signed short nOffset = (signed short)atoi(&(message->Data[11]));
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
#endif
        } else {
            printUart(DBG_MSG_PC, "wrong step value : (%d)", CtParam.nRaxisOffset);
        }
    }
    else if (strstr(message->Data, UART_CT_ALIGN_RAXIS_RESET))
    {
#ifdef USE_I2C_EEPROM
        EEPRom_EraseAddrWord(ROM_CT_RAXIS_ALIGN_ADDR);
        CtParam.nRaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_RAXIS_ALIGN_ADDR);
        if (CtParam.nRaxisOffset == (signed short)0xffff) { CtParam.nRaxisOffset = 0; }
#endif
        printUart(DBG_MSG_PC, "CT R-Axis Offset = %d", CtParam.nRaxisOffset);;
    }
    else if (strstr(message->Data, UART_CT_ALIGN_RAXIS_GET_OFFSET))
    {
#ifdef USE_I2C_EEPROM
        CtParam.nRaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_RAXIS_ALIGN_ADDR);
        if (CtParam.nRaxisOffset == (signed short)0xffff) { CtParam.nRaxisOffset = 0; }
        else if(CtParam.nRaxisOffset<0) CtParam.nRaxisOffset++;
#endif
        printUart(DBG_MSG_PC, "CT R-Axis Offset = %d", CtParam.nRaxisOffset);;
    }
    else if (strstr(message->Data, UART_CT_ALIGN_FAST_RAXIS_SET_DOWN))
    {
        signed short nOffset = (signed short)atoi(&(message->Data[11]));
        printUart(DBG_MSG_PC, "input degree : -%d(')", nOffset);
        nOffset = -(nOffset / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO));
        printUart(DBG_MSG_PC, "converted step value : %d", nOffset);
        nOffset = CtParam.nFastRaxisOffset + nOffset;
        if ( (nOffset >= (CT_RAXIS_DEGREE_MIN / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))) && \
            (nOffset <= (CT_RAXIS_DEGREE_MAX / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))) ) {
            CtParam.nFastRaxisOffset = nOffset;
            printUart(DBG_MSG_PC, "nFastRaxisOffset(%d)", CtParam.nFastRaxisOffset);
#ifdef USE_I2C_EEPROM
            EEPRom_I2C_Write_Word(ROM_CT_FAST_RAXIS_ALIGN_ADDR, (unsigned short)CtParam.nFastRaxisOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());
            printUart(DBG_MSG_PC, "ROM_CT_FAST_RAXIS_ALIGN_ADDR(0x%02x) : 0x%04x", ROM_CT_FAST_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CT_FAST_RAXIS_ALIGN_ADDR));
#endif
        } else {
            printUart(DBG_MSG_PC, "wrong step value : (%d)", CtParam.nFastRaxisOffset);
        }
    }
    else if (strstr(message->Data, UART_CT_ALIGN_FAST_RAXIS_SET_UP))
    {
        signed short nOffset = (signed short)atoi(&(message->Data[11]));
        printUart(DBG_MSG_PC, "input degree : +%d(')", nOffset);
        nOffset = +(nOffset / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO));
        printUart(DBG_MSG_PC, "converted step value : %d", nOffset);
        nOffset = CtParam.nFastRaxisOffset + nOffset;
        if ( (nOffset >= (CT_RAXIS_DEGREE_MIN / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))) && \
            (nOffset <= (CT_RAXIS_DEGREE_MAX / (360.0 / MOTOR_R_MICROSTEP / MOTOR_R_PULLEY_RATIO))) ) {
            CtParam.nFastRaxisOffset = nOffset;
            printUart(DBG_MSG_PC, "nFastRaxisOffset(%d)", CtParam.nFastRaxisOffset);
#ifdef USE_I2C_EEPROM
            EEPRom_I2C_Write_Word(ROM_CT_FAST_RAXIS_ALIGN_ADDR, (unsigned short)CtParam.nFastRaxisOffset);
            IntTimer_Delay(1000);
            while(!IntTimer_GetStatus());
            printUart(DBG_MSG_PC, "ROM_CT_FAST_RAXIS_ALIGN_ADDR(0x%02x) : 0x%04x", ROM_CT_FAST_RAXIS_ALIGN_ADDR, EEPRom_I2C_Read_Word(ROM_CT_FAST_RAXIS_ALIGN_ADDR));
#endif
        } else {
            printUart(DBG_MSG_PC, "wrong step value : (%d)", CtParam.nFastRaxisOffset);
        }
    }
    else if (strstr(message->Data, UART_CT_ALIGN_FAST_RAXIS_RESET))
    {
#ifdef USE_I2C_EEPROM
        EEPRom_EraseAddrWord(ROM_CT_FAST_RAXIS_ALIGN_ADDR);
        CtParam.nFastRaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_FAST_RAXIS_ALIGN_ADDR);
        if (CtParam.nFastRaxisOffset == (signed short)0xffff) { CtParam.nFastRaxisOffset = 0; }
#endif
        printUart(DBG_MSG_PC, "CT Fast R-Axis Offset = %d", CtParam.nFastRaxisOffset);;
    }
    else if (strstr(message->Data, UART_CT_ALIGN_FAST_RAXIS_GET_OFFSET))
    {
#ifdef USE_I2C_EEPROM
        CtParam.nFastRaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_FAST_RAXIS_ALIGN_ADDR);
        if (CtParam.nFastRaxisOffset == (signed short)0xffff) { CtParam.nFastRaxisOffset = 0; }
        else if(CtParam.nFastRaxisOffset<0) CtParam.nFastRaxisOffset++;
#endif
        printUart(DBG_MSG_PC, "CT Fast R-Axis Offset = %d", CtParam.nFastRaxisOffset);;
    }
    /* --- CT Patient P-axis --- */
    else if (strstr(message->Data, UART_CT_PATIENT_PAXIS_SET_DOWN))
    {
        signed short nOffset = (signed short)atoi(&(message->Data[10]));
        printUart(DBG_MSG_PC, "input diameter : -%d", nOffset);
        nOffset = -(nOffset / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP));
        printUart(DBG_MSG_PC, "converted step value : %d", nOffset);
        nOffset = CtParam.nPaxisPatientOffset + nOffset;
        if ( (nOffset >= (CT_PAXIS_DIAMETER_MIN / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP))) && \
            (nOffset <= (CT_PAXIS_DIAMETER_MAX / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP))) ) {
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
#endif
        } else {
            printUart(DBG_MSG_PC, "wrong step value : (%d)", CtParam.nPaxisPatientOffset);
        }
    }
    else if (strstr(message->Data, UART_CT_PATIENT_PAXIS_SET_UP))
    {
        signed short nOffset = (signed short)atoi(&(message->Data[10]));
        printUart(DBG_MSG_PC, "input diameter : +%d", nOffset);
        nOffset = +(nOffset / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP));
        printUart(DBG_MSG_PC, "converted step value : %d", nOffset);
        nOffset = CtParam.nPaxisPatientOffset + nOffset;
        if ( (nOffset >= (CT_PAXIS_DIAMETER_MIN / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP))) && \
            (nOffset <= (CT_PAXIS_DIAMETER_MAX / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP))) ) {
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
#endif
        } else {
            printUart(DBG_MSG_PC, "wrong step value : (%d)", CtParam.nPaxisPatientOffset);
        }
    }
    else if (strstr(message->Data, UART_CT_PATIENT_PAXIS_RESET))
    {
#if defined(USE_I2C_EEPROM)
        EEPRom_EraseAddrWord(ROM_CT_PAXIS_PATIENT_ALG_ADDR);
#endif
        CtParam.nPaxisPatientOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_PAXIS_PATIENT_ALG_ADDR);
        if (CtParam.nPaxisPatientOffset == (signed short)0xffff) { CtParam.nPaxisPatientOffset = 0; }
    }
    else if (strstr(message->Data, UART_CT_PATIENT_PAXIS_GET_OFFSET))
    {
        CtParam.nPaxisPatientOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_PAXIS_PATIENT_ALG_ADDR);
        if (CtParam.nPaxisPatientOffset == (signed short)0xffff) { CtParam.nPaxisPatientOffset = 0; }
        else if(CtParam.nPaxisPatientOffset<0) CtParam.nPaxisPatientOffset++;
        printUart(DBG_MSG_PC, "CT P-Axis Patient Offset = %d", CtParam.nPaxisPatientOffset);;
    }
    /* --- CT Patient R-axis --- */
    else if (strstr(message->Data, UART_CT_PATIENT_RAXIS_SET_DOWN))
    {
        signed short nOffset = (signed short)atoi(&(message->Data[10]));
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
#endif
        } else {
            printUart(DBG_MSG_PC, "wrong step value : (%d)", CtParam.nRaxisPatientOffset);
        }
    }
    else if (strstr(message->Data, UART_CT_PATIENT_RAXIS_SET_UP))
    {
        signed short nOffset = (signed short)atoi(&(message->Data[10]));
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
#endif
        } else {
            printUart(DBG_MSG_PC, "wrong step value : (%d)", CtParam.nRaxisPatientOffset);
        }
    }
    else if (strstr(message->Data, UART_CT_PATIENT_RAXIS_RESET))
    {
        EEPRom_EraseAddrWord(ROM_CT_RAXIS_PATIENT_ALG_ADDR);
        CtParam.nRaxisPatientOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_RAXIS_PATIENT_ALG_ADDR);
        if (CtParam.nRaxisPatientOffset == (signed short)0xffff) { CtParam.nRaxisPatientOffset = 0; }
    }
    else if (strstr(message->Data, UART_CT_PATIENT_RAXIS_GET_OFFSET))
    {
        CtParam.nRaxisPatientOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_RAXIS_PATIENT_ALG_ADDR);
        if (CtParam.nRaxisPatientOffset == (signed short)0xffff) { CtParam.nRaxisPatientOffset = 0; }
        else if(CtParam.nRaxisPatientOffset<0) CtParam.nRaxisPatientOffset++;
        printUart(DBG_MSG_PC, "CT R-Axis Patient Offset = %d", CtParam.nRaxisPatientOffset);;
    }

    return SET;
}

/* =========================================================================
 * GeoAlign_HandleCommon -- Ceph around shot, tube params, coli open, 2nd coli positions
 * ========================================================================= */
bool GeoAlign_HandleCommon(UART_MsgTypedef *message, bool *bChin_res)
{
    if (strstr(message->Data, UART_CEPH_RAXIS_ALIGN_AROUND_SHOT))
    {
        int16_t nOffset = 0;
        int32_t Total_Move = 0;
        char str[40] = {0,};

        nOffset = atoi(&(message->Data[9]));
        printUart(DBG_MSG_PC, "Ceph Around Offset = %d", nOffset);

        if (AlignModeParam.KV == 0 || AlignModeParam.mA == 0)
        {
            char string1[UART_MSG_SIZE * 2 + 3];
            memset(string1, NULL, sizeof(char) * (UART_MSG_SIZE * 2 + 3));
            sprintf(string1, "%s : %s", UART_ALIGN_MSG_ERROR, message->Data);
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
            if (!CAN_Collimator_SendMessage(CMD_FILTER_CENTER, 0, 0, 1000, 2)) { return RESET; }
            TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StartCurrent);
            Motor_MoveAbsolutePosition(&Motor_R, 1000, 100, Motor_R.CurrentStep - nOffset, 100);
            while(!Motor_GetStatus(&Motor_R, STATUS_STOP));
            TMC2660_SetCurrent(Motor_R.MotType, Motor_R.StopCurrent);
            UART_SendMessage(DBG_MSG_PC, "Enter into Communication with Tube");
            if (!CAN_SendMessage(CAN_TUBE_COM_CHECK, CAN_TUBE_GEN2_COM, 0, 3000, 2)) { return RESET; }
            UART_SendMessage(DBG_MSG_PC, "Tube's Communication is checked");
            if (!CAN_SendMessage(CAN_TUBE_CONTI_MODE_SET, 0, 0, 3000, 2)) { return RESET; }
            if (!CAN_SendMessage(CAN_TUBE_KV_SET, AlignModeParam.KV, 0, 3000, 2)) { return RESET; }
            if (!CAN_SendMessage(CAN_TUBE_mA_SET, AlignModeParam.mA, 0, 3000, 2)) { return RESET; }
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
            IntTimer_Delay(100);
            while(!IntTimer_GetStatus());
            UART_SendMessage(DBG_MSG_PC, "SENSOR_S_OUTPUT_ENABLE");
            ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_ENABLE);
            while(!Exposure_CheckSwitch())
            {
                if(UART_Exposure_SET()==2) { break; }
                if(Motor_GetStatus(&Motor_R, STATUS_STOP))
                {
                    UART_SendMessage(DBG_MSG_PC, "Motor_S End Position");
                    break;
                }
            }
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
    else if (strstr(message->Data, UART_TUBE_VOLTAGE))
    {
        AlignModeParam.KV = atoi(&(message->Data[9])) / 100;
    }
    else if (strstr(message->Data, UART_TUBE_CURRENT))
    {
        AlignModeParam.mA = atoi(&(message->Data[9])) / 10;
    }
    else if (strstr(message->Data, UART_COLLIMATOR_OPEN))
    {
        if (!CAN_Collimator_SendMessage(CMD_COLLI_OPEN, 0, 0, 2000, 2)) { return RESET; }
    }
    else if (strstr(message->Data, UART_CEPH_2COL_START_POS))
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
    else if (strstr(message->Data, UART_CEPH_2COL_END_POS))
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
    else if (strstr(message->Data, UART_CEPH_DET_START_POS))
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
    else if (strstr(message->Data, UART_CEPH_DET_END_POS))
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

    return SET;
}

/* =========================================================================
 * GeoAlign_HandleTAxis -- Temple support (T-axis) commands
 * ========================================================================= */
bool GeoAlign_HandleTAxis(UART_MsgTypedef *message)
{
    if (strstr(message->Data, UART_GEO_ALIGN_TAXIS_STEP_UP))
    {
        TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StartCurrent);
        Motor_MoveAbsolutePosition(&Motor_T, 1000, 500, GEOMETRY_ALIGN_TAXIS_MAX, 500);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_TAXIS_STEP_DOWN))
    {
        TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StartCurrent);
        Motor_MoveAbsolutePosition(&Motor_T, 1000, 500, GEOMETRY_ALIGN_TAXIS_MIN, 500);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_TAXIS_STEP_STOP))
    {
        Motor_T.Update=RESET;
        Motor_Stop(&Motor_T);
        TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StopCurrent);
        printUart(DBG_MSG_PC, "nTAxisCurrentSteps = %d", Motor_T.CurrentStep);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_TAXIS_MOVE_STEP))
    {
        int16_t nOffset = 0;
        if (strchr(&(message->Data[10]), 'P'))
            nOffset += atoi(&(message->Data[11]));
        else if(strchr(&(message->Data[10]), 'R'))
            nOffset -= atoi(&(message->Data[11]));
        else
            return RESET;
        printUart(DBG_MSG_PC, "Move %d mm", nOffset);
        TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StartCurrent);
        Motor_MoveAbsolutePosition(&Motor_T, 3000, 1000, Motor_T.CurrentStep+(nOffset*100), 1000);
        while(!Motor_GetStatus(&Motor_T, STATUS_STOP));
        TMC2660_SetCurrent(Motor_T.MotType, Motor_T.StopCurrent);
        printUart(DBG_MSG_PC, "nTAxisCurrentSteps = %d", Motor_T.CurrentStep);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_TAXIS_SET))
    {
        int16_t nOffset = 0;
        printUart(DBG_MSG_PC, "nTAxisCurrentSteps = %d", Motor_T.CurrentStep);
        nOffset=Motor_T.CurrentStep-8000;
#ifdef USE_I2C_EEPROM
        EEPRom_I2C_Write_Word(ROM_TEMPLE_SUPPORT_STEP_ADDR, (uint16_t)nOffset);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "ROM_TEMPLE_SUPPORT_STEP_ADDR(0x%02x) : 0x%04x", ROM_TEMPLE_SUPPORT_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_TEMPLE_SUPPORT_STEP_ADDR));
#endif
        PanoParam.nTAxisOffset = nOffset;
        printUart(DBG_MSG_PC, "nTAxisOffset = %d", PanoParam.nTAxisOffset);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_TAXIS_RESET))
    {
        int16_t nOffset = 0;
#ifdef USE_I2C_EEPROM
        EEPRom_EraseAddrWord(ROM_TEMPLE_SUPPORT_STEP_ADDR);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "ROM_TEMPLE_SUPPORT_STEP_ADDR(0x%02x) : 0x%04x", ROM_TEMPLE_SUPPORT_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_TEMPLE_SUPPORT_STEP_ADDR));
#endif
        PanoParam.nTAxisOffset = nOffset;
        printUart(DBG_MSG_PC, "nTAxisOffset = %d", PanoParam.nTAxisOffset);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_TAXIS_OFFSET))
    {
        printUart(DBG_MSG_PC, "Current Temple Support Offset = %d", PanoParam.nTAxisOffset);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_TAXIS_CHILD_SET))
    {
        int16_t nOffset = 0;
        printUart(DBG_MSG_PC, "nTAxisChildCurrentSteps = %d", Motor_T.CurrentStep);
        nOffset=Motor_T.CurrentStep-8000;
#ifdef USE_I2C_EEPROM
        EEPRom_I2C_Write_Word(ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR, (uint16_t)nOffset);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR(0x%02x) : 0x%04x", ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR));
#endif
        PanoParam.nTAxisChildOffset = nOffset;
        printUart(DBG_MSG_PC, "nTAxisChildOffset = %d", PanoParam.nTAxisChildOffset);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_TAXIS_CHILD_RESET))
    {
        int16_t nOffset = 1000;
#ifdef USE_I2C_EEPROM
        EEPRom_EraseAddrWord(ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR(0x%02x) : 0x%04x", ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR, EEPRom_I2C_Read_Word(ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR));
#endif
        PanoParam.nTAxisChildOffset = nOffset;
        printUart(DBG_MSG_PC, "nTAxisChildOffset = %d", PanoParam.nTAxisChildOffset);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_TAXIS_CHILD_OFFSET))
    {
        PanoParam.nTAxisChildOffset = (signed short)EEPRom_I2C_Read_Word(ROM_TEMPLE_SUPPORT_CHILD_STEP_ADDR);
        if (PanoParam.nTAxisChildOffset == (signed short)0xffff) { PanoParam.nTAxisChildOffset=0; }
        else if(PanoParam.nTAxisChildOffset<0) PanoParam.nTAxisChildOffset++;
        printUart(DBG_MSG_PC, "Current Temple Support ChildOffset = %d", PanoParam.nTAxisChildOffset);
    }

    return SET;
}

/* =========================================================================
 * GeoAlign_HandleCNSCWEAxes -- Pano & CT chinrest CNS/CWE offset commands
 *
 * NOTE: This function is intentionally large because it handles all
 * CNS and CWE axis commands for both Pano and CT modes. The original
 * code had all of these in one giant if-else chain.
 * ========================================================================= */
bool GeoAlign_HandleCNSCWEAxes(UART_MsgTypedef *message, bool *bChin_res)
{
    /* --- Pano CNS --- */
    if (strstr(message->Data, UART_GEO_ALIGN_PANO_CNSAXIS_STEP_UP))
    {
        int16_t nOffset = PanoParam.nCNSaxisOffset;
        printUart(DBG_MSG_PC, "nOffset = +1");
        nOffset++;
        *bChin_res=Motor_ControlChinrest(SET, nOffset+MOTOR_CNS_PANO_INIT_POSITION);
        if(*bChin_res==SET)
        {
            PanoParam.nCNSaxisOffset = nOffset;
            printUart(DBG_MSG_PC, "Pano_nCNSAxisOffset = %d", PanoParam.nCNSaxisOffset);
        }
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_PANO_CNSAXIS_STEP_DOWN))
    {
        int16_t nOffset = PanoParam.nCNSaxisOffset;
        printUart(DBG_MSG_PC, "nOffset = -1");
        nOffset--;
        *bChin_res=Motor_ControlChinrest(SET, nOffset+MOTOR_CNS_PANO_INIT_POSITION);
        if(*bChin_res==SET)
        {
            PanoParam.nCNSaxisOffset = nOffset;
            printUart(DBG_MSG_PC, "Pano_nCNSAxisOffset = %d", PanoParam.nCNSaxisOffset);
        }
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_PANO_CNSAXIS_SET))
    {
        int16_t nOffset = 0;
        nOffset = PanoParam.nCNSaxisOffset;
        printUart(DBG_MSG_PC, "Current Pano_nCNSAxisOffset = %d.", nOffset);
#ifdef USE_I2C_EEPROM
        EEPRom_I2C_Write_Word(ROM_PANO_CNSAXIS_ALG_ADDR, (uint16_t)nOffset);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "PANO_CNS_AXIS_ADDR(0x%02x) : 0x%04x", ROM_PANO_CNSAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_PANO_CNSAXIS_ALG_ADDR));
#endif
        printUart(DBG_MSG_PC, "Pano_nCNSAxisOffset = %d", PanoParam.nCNSaxisOffset);
        CtParam.nCNSaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_CNSAXIS_ALG_ADDR);
        if (CtParam.nCNSaxisOffset== (signed short)0xffff) { CtParam.nCNSaxisOffset = PanoParam.nCNSaxisOffset; }
        else if(CtParam.nCNSaxisOffset<0) CtParam.nCNSaxisOffset++;
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_PANO_CNSAXIS_RESET))
    {
        printUart(DBG_MSG_PC, "Pano_nCNSAxisOffset RESET");
#ifdef USE_I2C_EEPROM
        EEPRom_EraseAddrWord(ROM_PANO_CNSAXIS_ALG_ADDR);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "PANO_CNS_AXIS_ADDR(0x%02x) : 0x%04x", ROM_PANO_CNSAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_PANO_CNSAXIS_ALG_ADDR));
#endif
        PanoParam.nCNSaxisOffset = 0;
        printUart(DBG_MSG_PC, "Pano_nCNSAxisOffset = %d", PanoParam.nCNSaxisOffset);
        CtParam.nCNSaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_CNSAXIS_ALG_ADDR);
        if (CtParam.nCNSaxisOffset== (signed short)0xffff) { CtParam.nCNSaxisOffset = PanoParam.nCNSaxisOffset; }
        else if(CtParam.nCNSaxisOffset<0) CtParam.nCNSaxisOffset++;
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_PANO_CNSAXIS_DEC))
    {
        int16_t nOffset = PanoParam.nCNSaxisOffset;
        nOffset -= atoi(&(message->Data[12]));
        printUart(DBG_MSG_PC, "nOffset = %d", nOffset);
        *bChin_res=Motor_ControlChinrest(SET, nOffset+MOTOR_CNS_PANO_INIT_POSITION);
        if(*bChin_res==SET) { PanoParam.nCNSaxisOffset = nOffset; printUart(DBG_MSG_PC, "Pano_nCNSAxisOffset = %d", PanoParam.nCNSaxisOffset); }
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_PANO_CNSAXIS_INC))
    {
        int16_t nOffset = PanoParam.nCNSaxisOffset;
        nOffset += atoi(&(message->Data[12]));
        printUart(DBG_MSG_PC, "nOffset = %d", nOffset);
        *bChin_res=Motor_ControlChinrest(SET, nOffset+MOTOR_CNS_PANO_INIT_POSITION);
        if(*bChin_res==SET) { PanoParam.nCNSaxisOffset = nOffset; printUart(DBG_MSG_PC, "Pano_nCNSAxisOffset = %d", PanoParam.nCNSaxisOffset); }
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_GET_PANO_CNSAXIS_OFFSET))
    {
        PanoParam.nCNSaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_PANO_CNSAXIS_ALG_ADDR);
        if (PanoParam.nCNSaxisOffset== (signed short)0xffff) { PanoParam.nCNSaxisOffset=0; }
        else if(PanoParam.nCNSaxisOffset<0) PanoParam.nCNSaxisOffset++;
        printUart(DBG_MSG_PC, "Current Pano_nCNSAxisOffset = %d", PanoParam.nCNSaxisOffset);
    }
    /* --- Pano CWE --- */
    else if (strstr(message->Data, UART_GEO_ALIGN_PANO_CWEAXIS_STEP_UP))
    {
        int16_t nOffset = PanoParam.nCWEaxisOffset;
        printUart(DBG_MSG_PC, "nOffset = +1");
        nOffset++;
        *bChin_res=Motor_ControlChinrest(RESET, (double)nOffset/10+MOTOR_CWE_INIT_POSITION);
        if(*bChin_res==SET)
        {
            PanoParam.nCWEaxisOffset = nOffset;
            if(PanoParam.nCWEaxisOffset>=0)
                printUart(DBG_MSG_PC, "Pano_nCWEaxisOffset = %d.%d", PanoParam.nCWEaxisOffset/10,PanoParam.nCWEaxisOffset%10);
            else
                printUart(DBG_MSG_PC, "Pano_nCWEaxisOffset = %d.%d", PanoParam.nCWEaxisOffset/10,((PanoParam.nCWEaxisOffset%10)*-1));
        }
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_PANO_CWEAXIS_STEP_DOWN))
    {
        int16_t nOffset = PanoParam.nCWEaxisOffset;
        printUart(DBG_MSG_PC, "nOffset = -1");
        nOffset--;
        *bChin_res=Motor_ControlChinrest(RESET, (double)nOffset/10+MOTOR_CWE_INIT_POSITION);
        if(*bChin_res==SET)
        {
            PanoParam.nCWEaxisOffset = nOffset;
            if(PanoParam.nCWEaxisOffset>=0)
                printUart(DBG_MSG_PC, "Pano_nCWEaxisOffset = %d.%d", PanoParam.nCWEaxisOffset/10,PanoParam.nCWEaxisOffset%10);
            else
                printUart(DBG_MSG_PC, "Pano_nCWEaxisOffset = %d.%d", PanoParam.nCWEaxisOffset/10,((PanoParam.nCWEaxisOffset%10)*-1));
        }
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_PANO_CWEAXIS_SET))
    {
        int16_t nOffset = PanoParam.nCWEaxisOffset;
        printUart(DBG_MSG_PC, "Current Pano_nCWEaxisOffset = %d.", nOffset);
#ifdef USE_I2C_EEPROM
        EEPRom_I2C_Write_Word(ROM_PANO_CWEAXIS_ALG_ADDR, (uint16_t)nOffset);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "PANO_CWE_AXIS_ADDR(0x%02x) : 0x%04x", ROM_PANO_CWEAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_PANO_CWEAXIS_ALG_ADDR));
#endif
        printUart(DBG_MSG_PC, "Pano_nCWEaxisOffset = %d", PanoParam.nCWEaxisOffset);
        CtParam.nCWEaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_CWEAXIS_ALG_ADDR);
        if (CtParam.nCWEaxisOffset== (signed short)0xffff) { CtParam.nCWEaxisOffset = PanoParam.nCWEaxisOffset; }
        else if(CtParam.nCWEaxisOffset<0) CtParam.nCWEaxisOffset++;
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_PANO_CWEAXIS_RESET))
    {
        printUart(DBG_MSG_PC, "Pano_nCWEaxisOffset RESET");
#ifdef USE_I2C_EEPROM
        EEPRom_EraseAddrWord(ROM_PANO_CWEAXIS_ALG_ADDR);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "PANO_CWE_AXIS_ADDR(0x%02x) : 0x%04x", ROM_PANO_CWEAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_PANO_CWEAXIS_ALG_ADDR));
#endif
        PanoParam.nCWEaxisOffset = 0;
        printUart(DBG_MSG_PC, "Pano_nCWEaxisOffset = %d", PanoParam.nCWEaxisOffset);
        CtParam.nCWEaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_CWEAXIS_ALG_ADDR);
        if (CtParam.nCWEaxisOffset== (signed short)0xffff) { CtParam.nCWEaxisOffset = PanoParam.nCWEaxisOffset; }
        else if(CtParam.nCWEaxisOffset<0) CtParam.nCWEaxisOffset++;
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_PANO_CWEAXIS_DEC))
    {
        int16_t nOffset = PanoParam.nCWEaxisOffset;
        nOffset -= atoi(&(message->Data[12]));
        printUart(DBG_MSG_PC, "nOffset = %d", nOffset);
        *bChin_res=Motor_ControlChinrest(RESET, (double)nOffset/10+MOTOR_CWE_INIT_POSITION);
        if(*bChin_res==SET) { PanoParam.nCWEaxisOffset = nOffset; printUart(DBG_MSG_PC, "Pano_nCWEaxisOffset = %d", PanoParam.nCWEaxisOffset); }
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_PANO_CWEAXIS_INC))
    {
        int16_t nOffset = PanoParam.nCWEaxisOffset;
        nOffset += atoi(&(message->Data[12]));
        printUart(DBG_MSG_PC, "nOffset = %d", nOffset);
        *bChin_res=Motor_ControlChinrest(RESET, (double)nOffset/10+MOTOR_CWE_INIT_POSITION);
        if(*bChin_res==SET) { PanoParam.nCWEaxisOffset = nOffset; printUart(DBG_MSG_PC, "Pano_nCWEaxisOffset = %d", PanoParam.nCWEaxisOffset); }
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_GET_PANO_CWEAXIS_OFFSET))
    {
        PanoParam.nCWEaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_PANO_CWEAXIS_ALG_ADDR);
        if (PanoParam.nCWEaxisOffset== (signed short)0xffff) { PanoParam.nCWEaxisOffset=0; }
        else if(PanoParam.nCWEaxisOffset<0) PanoParam.nCWEaxisOffset++;
        printUart(DBG_MSG_PC, "Current Pano_nCWEaxisOffset = %d", PanoParam.nCWEaxisOffset);
    }
    /* --- CT CNS --- */
    else if (strstr(message->Data, UART_GEO_ALIGN_CT_CNSAXIS_STEP_UP))
    {
        int16_t nOffset = CtParam.nCNSaxisOffset;
        printUart(DBG_MSG_PC, "nOffset = +1");
        nOffset++;
        *bChin_res=Motor_ControlChinrest(SET, nOffset+MOTOR_CNS_CT_INIT_POSITION);
        if(*bChin_res==SET) { CtParam.nCNSaxisOffset = nOffset; printUart(DBG_MSG_PC, "CT_nCNSAxisOffset = %d", CtParam.nCNSaxisOffset); }
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_CT_CNSAXIS_STEP_DOWN))
    {
        int16_t nOffset = CtParam.nCNSaxisOffset;
        printUart(DBG_MSG_PC, "nOffset = -1");
        nOffset--;
        *bChin_res=Motor_ControlChinrest(SET, nOffset+MOTOR_CNS_CT_INIT_POSITION);
        if(*bChin_res==SET) { CtParam.nCNSaxisOffset = nOffset; printUart(DBG_MSG_PC, "CT_nCNSAxisOffset = %d", CtParam.nCNSaxisOffset); }
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_CT_CNSAXIS_SET))
    {
        int16_t nOffset = CtParam.nCNSaxisOffset;
#ifdef USE_I2C_EEPROM
        EEPRom_I2C_Write_Word(ROM_CT_CNSAXIS_ALG_ADDR, (uint16_t)nOffset);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "CT_CNS_AXIS_ADDR(0x%02x) : 0x%04x", ROM_CT_CNSAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_CT_CNSAXIS_ALG_ADDR));
#endif
        printUart(DBG_MSG_PC, "CT_nCNSAxisOffset = %d", CtParam.nCNSaxisOffset);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_CT_CNSAXIS_RESET))
    {
#ifdef USE_I2C_EEPROM
        EEPRom_EraseAddrWord(ROM_CT_CNSAXIS_ALG_ADDR);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
#endif
        CtParam.nCNSaxisOffset = PanoParam.nCNSaxisOffset;
        printUart(DBG_MSG_PC, "CT_nCNSAxisOffset = %d", CtParam.nCNSaxisOffset);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_CT_CNSAXIS_DEC))
    {
        int16_t nOffset = CtParam.nCNSaxisOffset;
        nOffset -= atoi(&(message->Data[12]));
        *bChin_res=Motor_ControlChinrest(SET, nOffset+MOTOR_CNS_CT_INIT_POSITION);
        if(*bChin_res==SET) { CtParam.nCNSaxisOffset = nOffset; printUart(DBG_MSG_PC, "CT_nCNSAxisOffset = %d", CtParam.nCNSaxisOffset); }
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_CT_CNSAXIS_INC))
    {
        int16_t nOffset = CtParam.nCNSaxisOffset;
        nOffset += atoi(&(message->Data[12]));
        *bChin_res=Motor_ControlChinrest(SET, nOffset+MOTOR_CNS_CT_INIT_POSITION);
        if(*bChin_res==SET) { CtParam.nCNSaxisOffset = nOffset; printUart(DBG_MSG_PC, "CT_nCNSAxisOffset = %d", CtParam.nCNSaxisOffset); }
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_GET_CT_CNSAXIS_OFFSET))
    {
        CtParam.nCNSaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_CNSAXIS_ALG_ADDR);
        if (CtParam.nCNSaxisOffset== (signed short)0xffff) { CtParam.nCNSaxisOffset=PanoParam.nCNSaxisOffset; }
        else if(CtParam.nCNSaxisOffset<0) CtParam.nCNSaxisOffset++;
        printUart(DBG_MSG_PC, "Current CT_nCNSAxisOffset = %d", CtParam.nCNSaxisOffset);
    }
    /* --- CT CWE --- */
    else if (strstr(message->Data, UART_GEO_ALIGN_CT_CWEAXIS_STEP_UP))
    {
        int16_t nOffset = CtParam.nCWEaxisOffset;
        printUart(DBG_MSG_PC, "nOffset = +1");
        nOffset++;
        *bChin_res=Motor_ControlChinrest(RESET, (double)nOffset/10+MOTOR_CWE_INIT_POSITION);
        if(*bChin_res==SET) { CtParam.nCWEaxisOffset = nOffset; printUart(DBG_MSG_PC, "CT_nCWEaxisOffset = %d", CtParam.nCWEaxisOffset); }
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_CT_CWEAXIS_STEP_DOWN))
    {
        int16_t nOffset = CtParam.nCWEaxisOffset;
        printUart(DBG_MSG_PC, "nOffset = -1");
        nOffset--;
        *bChin_res=Motor_ControlChinrest(RESET, (double)nOffset/10+MOTOR_CWE_INIT_POSITION);
        if(*bChin_res==SET) { CtParam.nCWEaxisOffset = nOffset; printUart(DBG_MSG_PC, "CT_nCWEaxisOffset = %d", CtParam.nCWEaxisOffset); }
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_CT_CWEAXIS_SET))
    {
        int16_t nOffset = CtParam.nCWEaxisOffset;
#ifdef USE_I2C_EEPROM
        EEPRom_I2C_Write_Word(ROM_CT_CWEAXIS_ALG_ADDR, (uint16_t)nOffset);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
        printUart(DBG_MSG_PC, "CT_CWE_AXIS_ADDR(0x%02x) : 0x%04x", ROM_CT_CWEAXIS_ALG_ADDR, EEPRom_I2C_Read_Word(ROM_CT_CWEAXIS_ALG_ADDR));
#endif
        printUart(DBG_MSG_PC, "CT_nCWEaxisOffset = %d", CtParam.nCWEaxisOffset);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_CT_CWEAXIS_RESET))
    {
#ifdef USE_I2C_EEPROM
        EEPRom_EraseAddrWord(ROM_CT_CWEAXIS_ALG_ADDR);
        IntTimer_Delay(1000);
        while(!IntTimer_GetStatus());
#endif
        CtParam.nCWEaxisOffset = PanoParam.nCWEaxisOffset;
        printUart(DBG_MSG_PC, "CT_nCWEaxisOffset = %d", CtParam.nCWEaxisOffset);
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_CT_CWEAXIS_DEC))
    {
        int16_t nOffset = CtParam.nCWEaxisOffset;
        nOffset -= atoi(&(message->Data[12]));
        *bChin_res=Motor_ControlChinrest(RESET, (double)nOffset/10+MOTOR_CWE_INIT_POSITION);
        if(*bChin_res==SET) { CtParam.nCWEaxisOffset = nOffset; printUart(DBG_MSG_PC, "CT_nCWEaxisOffset = %d", CtParam.nCWEaxisOffset); }
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_CT_CWEAXIS_INC))
    {
        int16_t nOffset = CtParam.nCWEaxisOffset;
        nOffset += atoi(&(message->Data[12]));
        *bChin_res=Motor_ControlChinrest(RESET, (double)nOffset/10+MOTOR_CWE_INIT_POSITION);
        if(*bChin_res==SET) { CtParam.nCWEaxisOffset = nOffset; printUart(DBG_MSG_PC, "CT_nCWEaxisOffset = %d", CtParam.nCWEaxisOffset); }
    }
    else if (strstr(message->Data, UART_GEO_ALIGN_GET_CT_CWEAXIS_OFFSET))
    {
        CtParam.nCWEaxisOffset = (signed short)EEPRom_I2C_Read_Word(ROM_CT_CWEAXIS_ALG_ADDR);
        if (CtParam.nCWEaxisOffset== (signed short)0xffff) { CtParam.nCWEaxisOffset=PanoParam.nCWEaxisOffset; }
        else if(CtParam.nCWEaxisOffset<0) CtParam.nCWEaxisOffset++;
        printUart(DBG_MSG_PC, "Current CT_nCWEaxisOffset = %d", CtParam.nCWEaxisOffset);
    }

    return SET;
}

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/
