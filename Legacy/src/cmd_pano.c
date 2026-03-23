/*
*******************************************************************************
* cmd_pano.c : Panoramic mode command handlers
*******************************************************************************
* Copyright (C) 2014-2018 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Brief   : UART_SetPanoParam -- processes queued commands during
*             panoramic capture parameter setup phase.
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
#include "misc1.h"

/* Private defines --------------------------------------------------------- */
#define UART_PANO_VARIAN        "[SM_PANO_VARIA]"
#define UART_PANO_NORMAL        "[SM_MODE_NORMA]"
#define UART_PANO_LEFT          "[SM_MODE_LEFT_]"
#define UART_PANO_FRONT         "[SM_MODE_FRONT]"
#define UART_PANO_RIGHT         "[SM_MODE_RIGHT]"
#define UART_PANO_3D            "[SM_MODE_3D___]"
#define UART_PANO_MIDDLE_ALIGN  "[SM_MODE_MID__]"

#define UART_SCAN_NORMAL        "[SM_SCAN_NORMA]"
#define UART_SCAN_HIGH          "[SM_SCAN_HIGH_]"

#define UART_TABLET_PANO_SCAN_HD   "[SM_SCAN_HD___]"
#define UART_TABLET_PANO_SCAN_ND   "[SM_SCAN_ND___]"

#ifdef VARIABLE_EXPOSURE
#define UART_VARIABLE_EXP_START_TIME   "[SM_EXPS_"
#define UART_VARIABLE_EXP_END_TIME     "[SM_EXPE_"
#define UART_VARIABLE_EXP_KV          "[SM_EXP_KV_"
#define UART_VARIABLE_EXP_MA          "[SM_EXP_MA_"
#endif /* VARIABLE_EXPOSURE */

/* Public functions -------------------------------------------------------- */

bool UART_SetPanoParam(void)
{
    UART_MsgTypedef message;
    bool ret = RESET;

    while (MSG_QueueCnt(&UART_Queue)) {
        message = MSG_Dequeue(&UART_Queue);

        if (strstr(message.Data, UART_VERTICAL_UP)) {
            Motor_MoveAbsolutePosition(&Motor_V, 2000, 50, 71000, 100);
        }
        else if (strstr(message.Data, UART_VERTICAL_DOWN)) {
            Motor_MoveAbsolutePosition(&Motor_V, 2000, 50, -2000, 100);
        }
        else if (strstr(message.Data, UART_VERTICAL_STOP)) {
            Motor_V.Update = RESET;
            Motor_Stop(&Motor_V);
        }
        else if (strstr(message.Data, UART_BEAM_POS_UP)) {
            if (!CAN_Collimator_SendMessage(CMD_HBEAM_LASER_UP, 0, 0, 1000, 0))
                return RESET;
        }
        else if (strstr(message.Data, UART_BEAM_POS_DOWN)) {
            if (!CAN_Collimator_SendMessage(CMD_HBEAM_LASER_DOWN, 0, 0, 1000, 0))
                return RESET;
        }
        else if (strstr(message.Data, UART_TABLET_FRANKFORT_UP)) {
            if (!CAN_Collimator_SendMessage(CMD_LASER_UP, 0, 0, 1000, 2)) {
                CurCaptureMode = CAPTURE_CANCEL;
                return RESET;
            }
        }
        else if (strstr(message.Data, UART_TABLET_FRANKFORT_DOWN)) {
            if (!CAN_Collimator_SendMessage(CMD_LASER_DOWN, 0, 0, 1000, 2)) {
                CurCaptureMode = CAPTURE_CANCEL;
                return RESET;
            }
        }
        else if (strstr(message.Data, UART_TABLET_FRANKFORT_STOP)) {
            if (!CAN_Collimator_SendMessage(CMD_LASER_STOP, 0, 0, 1000, 2)) {
                CurCaptureMode = CAPTURE_CANCEL;
                return RESET;
            }
        }
        else if (strstr(message.Data, UART_FRANKFORT_UP)) {
            if (!CAN_Collimator_SendMessage(CMD_LASER_UP, 0, 0, 1000, 2)) {
                CurCaptureMode = CAPTURE_CANCEL;
                return RESET;
            }
        }
        else if (strstr(message.Data, UART_FRANKFORT_DOWN)) {
            if (!CAN_Collimator_SendMessage(CMD_LASER_DOWN, 0, 0, 1000, 2)) {
                CurCaptureMode = CAPTURE_CANCEL;
                return RESET;
            }
        }
        else if (strstr(message.Data, UART_FRANKFORT_STOP)) {
            if (!CAN_Collimator_SendMessage(CMD_LASER_STOP, 0, 0, 1000, 2)) {
                CurCaptureMode = CAPTURE_CANCEL;
                return RESET;
            }
        }
#ifdef USE_MOTOR_CHINREST_TS
        else if (strstr(message.Data, UART_TEMPLE_PUSH)) {
            Motor_MoveTempleSupport(SET);
        }
        else if (strstr(message.Data, UART_TEMPLE_RELEASE)) {
            Motor_MoveTempleSupport(RESET);
        }
        else if (strstr(message.Data, UART_EAR_ROD_PUSH)) {
            Motor_MoveTempleSupport(SET);
#ifdef USE_TABLET_PC
            UART_SendMessage(DBG_MSG_TABLET, UART_EAR_ROD_DONE_FOR_MEMB);
#endif /* USE_TABLET_PC */
            IntTimer_Delay(200);
            while (!IntTimer_GetStatus())
                ;
        }
        else if (strstr(message.Data, UART_EAR_ROD_CHILD_PUSH)) {
            Motor_MoveTempleSupport_Child(SET);
#ifdef USE_TABLET_PC
            UART_SendMessage(DBG_MSG_TABLET, UART_EAR_ROD_DONE_FOR_MEMB);
#endif /* USE_TABLET_PC */
            IntTimer_Delay(200);
            while (!IntTimer_GetStatus())
                ;
        }
        else if (strstr(message.Data, UART_EAR_ROD_RELEASE)) {
            Motor_MoveTempleSupport(RESET);
#ifdef USE_TABLET_PC
            UART_SendMessage(DBG_MSG_TABLET, UART_EAR_ROD_DONE_FOR_MEMB);
#endif /* USE_TABLET_PC */
            IntTimer_Delay(200);
            while (!IntTimer_GetStatus())
                ;
        }
#endif /* USE_MOTOR_CHINREST_TS */
        else if (strstr(message.Data, UART_COLUMN_UP)) {
            Column_Control(COLUMN_UP);
            g_bColumnPressed = SET;
        }
        else if (strstr(message.Data, UART_COLUMN_DOWN)) {
            Column_Control(COLUMN_DOWN);
            g_bColumnPressed = SET;
        }
        else if (strstr(message.Data, UART_COLUMN_STOP)) {
            Column_Control(COLUMN_STOP);
            g_bColumnPressed = RESET;
        }
        else if (strstr(message.Data, UART_LASER_ON)) {
            LaserControl(TYPE_HEAD_PANO, SET);
            LaserControl(TYPE_FOOT, SET);
        }
        else if (strstr(message.Data, UART_LASER_OFF)) {
            LaserControl(TYPE_HEAD_PANO, RESET);
            LaserControl(TYPE_FOOT, RESET);
        }
        else if (strstr(message.Data, UART_HEAD_BEAM_ON)) {
            LaserControl(TYPE_HEAD_PANO, SET);
        }
        else if (strstr(message.Data, UART_HEAD_BEAM_OFF)) {
            LaserControl(TYPE_HEAD_PANO, RESET);
        }
        else if (strstr(message.Data, UART_FOOT_BEAM_ON)) {
            LaserControl(TYPE_FOOT, SET);
        }
        else if (strstr(message.Data, UART_FOOT_BEAM_OFF)) {
            LaserControl(TYPE_FOOT, RESET);
        }
        else if (strstr(message.Data, UART_CAPTURE_READY)) {
            /* No action needed */
        }
        else if (strstr(message.Data, UART_PANO_VARIAN)) {
            /* PanoParam.Sensor = PANO_SENSOR_VARIAN; (legacy) */
        }
        else if (strstr(message.Data, UART_CANINE_UP)) {
            Motor_MoveAbsolutePosition(&Motor_V, 1000, 100,
                71000 - (Motor_V.ArchParam.OffsetStep + PanoParam.ModeOffset +
                          PanoParam.nVAxisOffset), 100);
        }
        else if (strstr(message.Data, UART_CANINE_DOWN)) {
            Motor_MoveAbsolutePosition(&Motor_V, 1000, 100, -2000, 100);
        }
        else if (strstr(message.Data, UART_CANINE_STOP)) {
            Motor_V.Update = FALSE;
            Motor_Stop(&Motor_V);
        }
        else if (strstr(message.Data, UART_MANUAL_AAXIS_UP)) {
            int32_t nOffset = atoi(&(message.Data[9]));
            char str[40] = {0,};
            sprintf(str, "nOffset = %d", nOffset);
            UART_SendMessage(DBG_MSG_PC, str);
            nOffset = nOffset / (360.0 / MOTOR_A_MICROSTEP / MOTOR_A_PULLEY_RATIO);
            sprintf(str, "Calculated up_nOffset = %d", nOffset);
            UART_SendMessage(DBG_MSG_PC, str);
#ifdef USE_MOTOR_GANTRY_MS
            Motor_MoveAbsolutePosition(&Motor_A, 1000, 100,
                Motor_A.CurrentStep + nOffset, 100);
            while (!Motor_GetStatus(&Motor_A, STATUS_STOP))
                ;
#endif /* USE_MOTOR_GANTRY_MS */
        }
        else if (strstr(message.Data, UART_MANUAL_AAXIS_DOWN)) {
            int32_t nOffset = atoi(&(message.Data[9]));
            char str[40] = {0,};
            sprintf(str, "nOffset = %d", nOffset);
            UART_SendMessage(DBG_MSG_PC, str);
            nOffset = nOffset / (360.0 / MOTOR_A_MICROSTEP / MOTOR_A_PULLEY_RATIO);
            sprintf(str, "Calculated down_nOffset = %d", nOffset);
            UART_SendMessage(DBG_MSG_PC, str);
#ifdef USE_MOTOR_GANTRY_MS
            Motor_MoveAbsolutePosition(&Motor_A, 1000, 100,
                Motor_A.CurrentStep - nOffset, 100);
            while (!Motor_GetStatus(&Motor_A, STATUS_STOP))
                ;
#endif /* USE_MOTOR_GANTRY_MS */
        }
        else if (strstr(message.Data, UART_MANUAL_VERTICAL_UP)) {
            int32_t nOffset = atoi(&(message.Data[9]));
            char str[40] = {0,};
            sprintf(str, "nOffset = %d", nOffset);
            UART_SendMessage(DBG_MSG_PC, str);
            nOffset = nOffset / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP);
            sprintf(str, "Calculated up_nOffset = %d", nOffset);
            UART_SendMessage(DBG_MSG_PC, str);
            Motor_MoveAbsolutePosition(&Motor_V, 1000, 100,
                Motor_V.CurrentStep + nOffset, 100);
            while (!Motor_GetStatus(&Motor_V, STATUS_STOP))
                ;
        }
        else if (strstr(message.Data, UART_MANUAL_VERTICAL_DOWN)) {
            int32_t nOffset = atoi(&(message.Data[9]));
            char str[40] = {0,};
            sprintf(str, "nOffset = %d", nOffset);
            UART_SendMessage(DBG_MSG_PC, str);
            nOffset = nOffset / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP);
            sprintf(str, "Calculated down_nOffset = %d", nOffset);
            UART_SendMessage(DBG_MSG_PC, str);
            Motor_MoveAbsolutePosition(&Motor_V, 1000, 100,
                Motor_V.CurrentStep - nOffset, 100);
            while (!Motor_GetStatus(&Motor_V, STATUS_STOP))
                ;
        }
        else if (strstr(message.Data, UART_COLLIMATOR_OPEN)) {
            PanoParam.bColliOpen = SET;
        }
        else if (strstr(message.Data, UART_TUBE_VOLTAGE)) {
            PanoParam.KV = atoi(&(message.Data[9])) / 100;
        }
        else if (strstr(message.Data, UART_TUBE_CURRENT)) {
            PanoParam.mA = atoi(&(message.Data[9])) / 10;
        }
        else if (strstr(message.Data, UART_PANO_NORMAL)) {
            PanoParam.Mode = PANO_MODE_NORMAL;
        }
        else if (strstr(message.Data, UART_PANO_LEFT)) {
            PanoParam.Mode = PANO_MODE_LEFT;
        }
        else if (strstr(message.Data, UART_PANO_FRONT)) {
            PanoParam.Mode = PANO_MODE_FRONT;
        }
        else if (strstr(message.Data, UART_PANO_RIGHT)) {
            PanoParam.Mode = PANO_MODE_RIGHT;
        }
        else if (strstr(message.Data, UART_PANO_3D)) {
            PanoParam.Mode = PANO_MODE_3D;
        }
        else if (strstr(message.Data, UART_PANO_MIDDLE_ALIGN)) {
            PanoParam.Mode = PANO_MODE_ALIGN;
        }
        else if (strstr(message.Data, UART_ARCH_STANDARD)) {
            PanoParam.Arch = PANO_ARCH_ADULT;
        }
        else if (strstr(message.Data, UART_ARCH_CHILD)) {
            PanoParam.Arch = PANO_ARCH_CHILD;
#ifdef USE_TABLET_PC
            UART_SendMessage(DBG_MSG_TABLET, UART_ARCH_CHILD_MEMBRANE);
#endif /* USE_TABLET_PC */
            IntTimer_Delay(200);
            while (!IntTimer_GetStatus())
                ;
        }
        else if (strstr(message.Data, UART_ARCH_SINUS)) {
            PanoParam.Mode = PANO_MODE_SINUS;
            PanoParam.Arch = PANO_ARCH_SINUS;
        }
        else if (strstr(message.Data, UART_ARCH_TMJ)) {
            PanoParam.Mode = PANO_MODE_TMJ;
            PanoParam.Arch = PANO_ARCH_TMJ;
#ifdef USE_TABLET_PC
            UART_SendMessage(DBG_MSG_TABLET, message.Data);
#endif /* USE_TABLET_PC */
            IntTimer_Delay(200);
            while (!IntTimer_GetStatus())
                ;
        }
        else if (strstr(message.Data, UART_TABLET_PANO_SCAN_HD)) {
            PanoParam.Scan = PANO_SCAN_HD;
        }
        else if (strstr(message.Data, UART_TABLET_PANO_SCAN_ND)) {
            PanoParam.Scan = PANO_SCAN_ND;
        }
        else if (strstr(message.Data, UART_SCAN_HIGH)) {
            PanoParam.Scan = PANO_SCAN_HD;
        }
        else if (strstr(message.Data, UART_SCAN_NORMAL)) {
            PanoParam.Scan = PANO_SCAN_ND;
        }
#ifdef VARIABLE_EXPOSURE
        else if (strstr(message.Data, UART_VARIABLE_EXP_START_TIME)) {
            PanoParam.nExpStartTime = atoi(&(message.Data[9]));
            printUart(DBG_MSG_PC, "Variable Exposure : Start Time = %d",
                      PanoParam.nExpStartTime);
        }
        else if (strstr(message.Data, UART_VARIABLE_EXP_END_TIME)) {
            PanoParam.nExpEndTime = atoi(&(message.Data[9]));
            printUart(DBG_MSG_PC, "Variable Exposure : End Time = %d",
                      PanoParam.nExpEndTime);
        }
        else if (strstr(message.Data, UART_VARIABLE_EXP_KV)) {
            unsigned int nParam = atoi(&(message.Data[11]));
            if (!(nParam <= MAX_TUBE_KV && nParam >= MIN_TUBE_KV)) {
                printUart(DBG_MSG_PC, "Unsupported range!");
            } else {
                PanoParam.nExpKV = nParam;
                printUart(DBG_MSG_PC, "Variable Exposure : KV = %d",
                          PanoParam.nExpKV);
            }
        }
        else if (strstr(message.Data, UART_VARIABLE_EXP_MA)) {
            unsigned int nParam = atoi(&(message.Data[11]));
            if (!(nParam <= MAX_TUBE_mA && nParam >= MIN_TUBE_mA)) {
                printUart(DBG_MSG_PC, "Unsupported range!");
            } else {
                PanoParam.nExpMA = nParam;
                printUart(DBG_MSG_PC, "Variable Exposure : mA = %d",
                          PanoParam.nExpMA);
            }
        }
#endif /* VARIABLE_EXPOSURE */
        else if (strstr(message.Data, UART_TUBE_SOUND_OFF)) {
            sysInfo.bXrayNoSound = SET;
            CAN_SendMessage(CAN_TUBE_SOUND_OFF, 0, 0, 1000, 2);
        }
        else if (strstr(message.Data, UART_GET_TUBE_TEMP)) {
            CAN_SendMessage(CAN_TUBE_TANK_TEMP_READ, 0, 0, 1000, 2);
        }
        else if (strstr(message.Data, UART_CAPTURE_CONFIRM)) {
            if (CMD_CheckCaptureParam() != RESET) {
                IntTimer_Stop();
                IntTimer_Delay(200);
                while (!IntTimer_GetStatus())
                    ;
#ifdef USE_TABLET_PC
                UART_SendMessage(DBG_MSG_TABLET, UART_MEMBRANE_CAPTURE_CONFIG);
#endif /* USE_TABLET_PC */
                ret = SET;
            }
        }
        else if (strstr(message.Data, UART_CAPTURE_CONFIRM_BY_MEMB)) {
            if (CMD_CheckCaptureParam() != RESET) {
                ret = SET;
                message.Data[2] = 'M';
            }
        }
        else {
            printUart(DBG_MSG_PC, "%s : %s", ERR_CODE_UART_MSG_ERROR,
                      message.Data);
            return RESET;
        }

        message.Data[1] = 'E';
        printUart(DBG_MSG_PC, message.Data);
    }

    return ret;
}

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/
