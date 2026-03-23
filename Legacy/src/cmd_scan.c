/*
*******************************************************************************
* cmd_scan.c : Cephalometric scan mode command handlers
*******************************************************************************
* Copyright (C) 2014-2018 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Brief   : UART_SetCephParam -- processes queued commands during
*             cephalometric scan parameter setup phase.
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
#define UART_SCAN_I3S               "[SM_SCAN_I3SYS]"

#define UART_CEPH_FULL_LATERAL      "[SM_MODE_FULLA]"
#define UART_CEPH_LATERAL           "[SM_MODE_LATER]"
#define UART_CEPH_PA                "[SM_MODE_PA___]"
#define UART_CEPH_AP                "[SM_MODE_AP___]"
#define UART_CEPH_SVM               "[SM_MODE_SVM__]"
#define UART_CEPH_WATERS            "[SM_MODE_WATER]"
#define UART_CEPH_CARPUS            "[SM_MODE_CARPU]"
#define UART_CEPH_ADULT             "[SM_CEPH_ADULT]"
#define UART_CEPH_CHILD             "[SM_CEPH_CHILD]"
#define UART_CEPH_FULL_LATERAL_2    "[SM_CEPH_FULLA]"
#define UART_CEPH_LATERAL_2         "[SM_CEPH_LATER]"
#define UART_CEPH_PA_2              "[SM_CEPH_PA___]"
#define UART_CEPH_AP_2              "[SM_CEPH_AP___]"
#define UART_CEPH_SVM_2             "[SM_CEPH_SVM__]"
#define UART_CEPH_WATERS_2          "[SM_CEPH_WATER]"
#define UART_CEPH_CARPUS_2          "[SM_CEPH_CARPU]"
#define UART_CEPH_EARLOD            "[SM_CEPH_EAROD]"

#define UART_CEPH_RAXIS_ALIGN_SCAN  "[SM_SCAN_ALIGN]"
#define UART_CEPH_EAR_ALIGN_SCAN    "[SM_SCAN_EARAL]"

/* Public functions -------------------------------------------------------- */

bool UART_SetCephParam(void)
{
    UART_MsgTypedef message;
    bool ret = RESET;

    while (MSG_QueueCnt(&UART_Queue)) {
        message = MSG_Dequeue(&UART_Queue);

        if (strstr(message.Data, UART_FRANKFORT_UP)) {
            /* No action in ceph mode */
        }
        else if (strstr(message.Data, UART_FRANKFORT_DOWN)) {
            /* No action in ceph mode */
        }
        else if (strstr(message.Data, UART_FRANKFORT_STOP)) {
            /* No action in ceph mode */
        }
        else if (strstr(message.Data, UART_BEAM_POS_UP)) {
            if (!CAN_Collimator_SendMessage(CMD_HBEAM_LASER_UP, 0, 0, 1000, 0))
                return RESET;
        }
        else if (strstr(message.Data, UART_BEAM_POS_DOWN)) {
            if (!CAN_Collimator_SendMessage(CMD_HBEAM_LASER_DOWN, 0, 0, 1000, 0))
                return RESET;
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
            LaserControl(TYPE_HEAD_CEPH, SET);
            LaserControl(TYPE_FOOT, SET);
        }
        else if (strstr(message.Data, UART_LASER_OFF)) {
            LaserControl(TYPE_HEAD_CEPH, RESET);
            LaserControl(TYPE_FOOT, RESET);
        }
        else if (strstr(message.Data, UART_HEAD_BEAM_ON)) {
            LaserControl(TYPE_HEAD_CEPH, SET);
        }
        else if (strstr(message.Data, UART_HEAD_BEAM_OFF)) {
            LaserControl(TYPE_HEAD_CEPH, RESET);
        }
        else if (strstr(message.Data, UART_FOOT_BEAM_ON)) {
            LaserControl(TYPE_FOOT, SET);
        }
        else if (strstr(message.Data, UART_FOOT_BEAM_OFF)) {
            LaserControl(TYPE_FOOT, SET);  /* NOTE: original code uses SET here */
        }
        else if (strstr(message.Data, UART_CAPTURE_READY)) {
            /* No action */
        }
        else if (strstr(message.Data, UART_SCAN_I3S)) {
            /* No action */
        }
        else if (strstr(message.Data, UART_TUBE_VOLTAGE)) {
            CephParam.KV = atoi(&(message.Data[9])) / 100;
        }
        else if (strstr(message.Data, UART_TUBE_CURRENT)) {
            CephParam.mA = atoi(&(message.Data[9])) / 10;
        }
        else if (strstr(message.Data, UART_CAPTURE_TIME)) {
            CephParam.Time = atoi(&(message.Data[9]));
        }
        else if (strstr(message.Data, UART_CAPTURE_SIZE)) {
            /* No action */
        }
        else if (strstr(message.Data, UART_CEPH_FULL_LATERAL)) {
            CephParam.Mode = Ceph_Full_Lateral;
        }
        else if (strstr(message.Data, UART_CEPH_LATERAL)) {
            CephParam.Mode = Ceph_Lateral;
        }
        else if (strstr(message.Data, UART_CEPH_PA)) {
            CephParam.Mode = Ceph_PA;
        }
        else if (strstr(message.Data, UART_CEPH_AP)) {
            CephParam.Mode = Ceph_AP;
        }
        else if (strstr(message.Data, UART_CEPH_SVM)) {
            CephParam.Mode = Ceph_SVM;
        }
        else if (strstr(message.Data, UART_CEPH_WATERS)) {
            CephParam.Mode = Ceph_Waters;
        }
        else if (strstr(message.Data, UART_CEPH_CARPUS)) {
            CephParam.Mode = Ceph_Carpus;
        }
        else if (strstr(message.Data, UART_CEPH_ADULT)) {
            CephParam.Type = Ceph_Adult;
        }
        else if (strstr(message.Data, UART_CEPH_CHILD)) {
            CephParam.Type = Ceph_Child;
        }
        else if (strstr(message.Data, UART_CEPH_FULL_LATERAL_2)) {
            CephParam.Mode = Ceph_Full_Lateral;
        }
        else if (strstr(message.Data, UART_CEPH_LATERAL_2)) {
            CephParam.Mode = Ceph_Lateral;
        }
        else if (strstr(message.Data, UART_CEPH_PA_2)) {
            CephParam.Mode = Ceph_PA;
        }
        else if (strstr(message.Data, UART_CEPH_AP_2)) {
            CephParam.Mode = Ceph_AP;
        }
        else if (strstr(message.Data, UART_CEPH_SVM_2)) {
            CephParam.Mode = Ceph_SVM;
        }
        else if (strstr(message.Data, UART_CEPH_WATERS_2)) {
            CephParam.Mode = Ceph_Waters;
        }
        else if (strstr(message.Data, UART_CEPH_CARPUS_2)) {
            CephParam.Mode = Ceph_Carpus;
        }
        else if (strstr(message.Data, UART_CEPH_EARLOD)) {
            CephParam.Mode = Ceph_EarLod;
        }
        else if (strstr(message.Data, UART_CEPH_RAXIS_ALIGN_SCAN)) {
            UART_SendMessage(DBG_MSG_PC, "1st Collimator Open");
            CephParam.bScanAlign = SET;
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
        else if (strstr(message.Data, UART_CEPH_EAR_ALIGN_SCAN)) {
            UART_SendMessage(DBG_MSG_PC, "2st Collimator Stop");
            TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StartCurrent);
            Motor_RunOrgCheck(&Motor_S);
            while (!Motor_GetStatus(&Motor_S, STATUS_STOP))
                ;
            TMC2660_SetCurrent(Motor_S.MotType, Motor_S.StopCurrent);
            CephParam.bScanEar = SET;
        }
        else if (strstr(message.Data, UART_TUBE_SOUND_OFF)) {
            sysInfo.bXrayNoSound = SET;
            CAN_SendMessage(CAN_TUBE_SOUND_OFF, 0, 0, 1000, 2);
        }
        else if (strstr(message.Data, UART_GET_TUBE_TEMP)) {
            CAN_SendMessage(CAN_TUBE_TANK_TEMP_READ, 0, 0, 1000, 2);
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
