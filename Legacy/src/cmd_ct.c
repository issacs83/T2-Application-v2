/*
*******************************************************************************
* cmd_ct.c : CT/CBCT mode command handlers
*******************************************************************************
* Copyright (C) 2014-2018 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Brief   : UART_SetCtParam -- processes queued commands during
*             CT capture parameter setup phase.
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
#define UART_CT_VARIAN          "[SM_CT___VARIA]"

#define UART_RESOLUTION_100um   "[SM_RESO_0.1MM]"
#define UART_RESOLUTION_200um   "[SM_RESO_0.2MM]"
#define UART_RESOLUTION_300um   "[SM_RESO_0.3MM]"

#define UART_FOV_NONE           "[SM_FOV__NONE_]"
#define UART_FOV_5X5            "[SM_FOV__5X5__]"
#define UART_FOV_8X9            "[SM_FOV__8X9__]"
#define UART_FOV_8X10           "[SM_FOV__8X10_]"
#define UART_FOV_10X9           "[SM_FOV__10X9_]"
#define UART_FOV_12X9           "[SM_FOV__12X9_]"
#define UART_FOV_15X9           "[SM_FOV__15X9_]"
#define UART_FOV_10X10          "[SM_FOV__10X10]"
#define UART_FOV_12X10          "[SM_FOV__12X10]"
#define UART_FOV_15X10          "[SM_FOV__15X10]"
#define UART_FOV_First          "[SM_FOV__10X00]"
#define UART_FOV_Second         "[SM_FOV__10X01]"
#define UART_FOV_Third          "[SM_FOV__10X02]"

#ifdef USE_CT_STITCH_MODE
#define UART_FOV_15X15          "[SM_FOV__15X15]"
#define UART_FOV_15X15_AGING    "[SM_FOV_AGING_]"
#endif /* USE_CT_STITCH_MODE */

#define UART_BINNING_1X1        "[SM_BINN_1X1__]"
#define UART_BINNING_2X2        "[SM_BINN_2X2__]"

#define UART_MAR_ON             "[SM_MAR__ON___]"
#define UART_MAR_OFF            "[SM_MAR__OFF__]"

#define UART_TUBE_CONTI         "[SM_TUBE_CONTI]"
#define UART_TUBE_PULSE         "[SM_TUBE_PULSE]"

#define UART_XON_TIME_ms        "[SM_XONT_"
#define UART_XOFF_TIME_ms       "[SM_XOFF_"
#define UART_DELAY_TIME_ms      "[SM_DELA_"
#define UART_TOTAL_FRAME        "[SM_FRAM_"
#define UART_CT_EXP_START_POS   "[SM_XOND_"
#define UART_VMOTOR_OFFSET      "[SM_VMOT_"

#define UART_CMD_PAXIS_MOVE     "[SM_PAXS_MV_"

#ifdef USE_MOTOR_CHINREST_HOR
#define UART_CMD_CHINREST_HOR_MOVE  "[SM_HAXS_MV"
#endif /* USE_MOTOR_CHINREST_HOR */

#ifdef USE_MOTOR_CHINREST_VER
#define UART_CMD_CHINREST_VER_MOVE  "[SM_VAXS_MV"
#endif /* USE_MOTOR_CHINREST_VER */

/* Private variables ------------------------------------------------------- */
static bool CT_15X15_RET = SET;

/* Public functions -------------------------------------------------------- */

bool UART_SetCtParam(void)
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
        else if (strstr(message.Data, UART_ARCH_CHILD)) {
#ifdef USE_TABLET_PC
            UART_SendMessage(DBG_MSG_TABLET, UART_ARCH_CHILD_MEMBRANE);
#endif /* USE_TABLET_PC */
            IntTimer_Delay(200);
            while (!IntTimer_GetStatus())
                ;
        }
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
            LaserControl(TYPE_HEAD_CT, SET);
            LaserControl(TYPE_FOOT, SET);
        }
        else if (strstr(message.Data, UART_LASER_OFF)) {
            LaserControl(TYPE_HEAD_CT, RESET);
            LaserControl(TYPE_FOOT, RESET);
        }
        else if (strstr(message.Data, UART_HEAD_BEAM_ON)) {
            LaserControl(TYPE_HEAD_CT, SET);
        }
        else if (strstr(message.Data, UART_HEAD_BEAM_OFF)) {
            LaserControl(TYPE_HEAD_CT, RESET);
        }
        else if (strstr(message.Data, UART_FOOT_BEAM_ON)) {
            LaserControl(TYPE_FOOT, SET);
        }
        else if (strstr(message.Data, UART_FOOT_BEAM_OFF)) {
            LaserControl(TYPE_FOOT, RESET);
        }
        else if (strstr(message.Data, UART_CAPTURE_READY)) {
            /* No action */
        }
        else if (strstr(message.Data, UART_CT_VARIAN)) {
            /* CtParam.Sensor = CT_SENSOR_VARIAN; (legacy) */
        }
        else if (strstr(message.Data, UART_TUBE_VOLTAGE)) {
            CtParam.KV = atoi(&(message.Data[9])) / 100;
        }
        else if (strstr(message.Data, UART_TUBE_CURRENT)) {
            CtParam.mA = atoi(&(message.Data[9])) / 10;
        }
        else if (strstr(message.Data, UART_RESOLUTION_100um)) {
            /* legacy */
        }
        else if (strstr(message.Data, UART_RESOLUTION_200um)) {
            /* legacy */
        }
        else if (strstr(message.Data, UART_RESOLUTION_300um)) {
            /* legacy */
        }
        else if (strstr(message.Data, UART_FOV_NONE)) {
            CtParam.bStitchMode = RESET;
            CtParam.Fov = CT_FOV_NONE;
        }
        else if (strstr(message.Data, UART_FOV_5X5)) {
            CtParam.bStitchMode = RESET;
            CtParam.Fov = CT_FOV_5X5;
        }
        else if (strstr(message.Data, UART_FOV_8X9)) {
            CtParam.bStitchMode = RESET;
            CtParam.Fov = CT_FOV_8X9;
        }
        else if (strstr(message.Data, UART_FOV_10X9)) {
            CtParam.bStitchMode = RESET;
            CtParam.Fov = CT_FOV_10X9;
        }
        else if (strstr(message.Data, UART_FOV_12X9)) {
            CtParam.bStitchMode = RESET;
            CtParam.Fov = CT_FOV_12X9;
        }
        else if (strstr(message.Data, UART_FOV_15X9)) {
            CtParam.bStitchMode = RESET;
            CtParam.Fov = CT_FOV_15X9;
        }
        else if (strstr(message.Data, UART_FOV_10X10)) {
            CtParam.bStitchMode = FALSE;
            CtParam.Fov = CT_FOV_10X10;
        }
        else if (strstr(message.Data, UART_FOV_12X10)) {
            CtParam.bStitchMode = FALSE;
            CtParam.Fov = CT_FOV_12X10;
        }
        else if (strstr(message.Data, UART_FOV_15X10)) {
            CtParam.bStitchMode = FALSE;
            CtParam.Fov = CT_FOV_15X10;
        }
        else if (strstr(message.Data, UART_FOV_8X10)) {
            CtParam.bStitchMode = FALSE;
            CtParam.Fov = CT_FOV_8X10;
        }
        else if (strstr(message.Data, UART_FOV_First)) {
            UART_SendMessage(DBG_MSG_TABLET, "[EM_FOV__10X00]");
            IntTimer_Delay(200);
            while (!IntTimer_GetStatus())
                ;
        }
        else if (strstr(message.Data, UART_FOV_Second)) {
            UART_SendMessage(DBG_MSG_TABLET, "[EM_FOV__10X01]");
            IntTimer_Delay(200);
            while (!IntTimer_GetStatus())
                ;
        }
        else if (strstr(message.Data, UART_FOV_Third)) {
            UART_SendMessage(DBG_MSG_TABLET, "[EM_FOV__10X02]");
            IntTimer_Delay(200);
            while (!IntTimer_GetStatus())
                ;
        }
#ifdef USE_CT_STITCH_MODE
        else if (strstr(message.Data, UART_FOV_15X15)) {
            int32_t nVerStep = (-1 / MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP;
            int32_t nCT_HorStep = MOTOR_CWE_INIT_POSITION /
                                  (MOTOR_CWE_PULLEY_PITCH / MOTOR_CWE_MICROSTEP);
            if (CtParam.bReverse_Status == SET) {
                UART_SendMessage(DBG_MSG_PC,
                    "Reverse Mode does not support 15X15 Stitching");
                return RESET;
            }
            CT_15X15_RET = StitchUnderSensor_Check();
            CtParam.bStitch_Status = CT_15X15_RET;
            if (CtParam.bStitch_Status == SET) {
                UART_SendMessage(DBG_MSG_PC, CT_15X15_OK_MESSAGE);
#ifdef USE_TABLET_PC
                UART_SendMessage(DBG_MSG_TABLET, CT_15X15_OK_MESSAGE);
#endif /* USE_TABLET_PC */
                Lamp_Color_Control(LED_BLACK);
            } else {
                UART_SendMessage(DBG_MSG_PC, CT_15X15_NG_MESSAGE);
#ifdef USE_TABLET_PC
                UART_SendMessage(DBG_MSG_TABLET, CT_15X15_NG_MESSAGE);
#endif /* USE_TABLET_PC */
                Lamp_Color_Control(LED_RED);
            }
            IntTimer_Delay(200);
            while (!IntTimer_GetStatus())
                ;
#ifdef USE_MOTOR_CHINREST_VER
#if defined(USE_MOTOR_CNS_SCREW_PITCH_6_35)
            TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StartCurrent);
            Motor_MoveAbsolutePosition(&Motor_CNS, 4000, 2000, 0, 2000);
            TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StopCurrent);
#else
            TMC2660_SetCurrent(Motor_CWE.MotType, Motor_CWE.StartCurrent);
            TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StartCurrent);
            TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
            Motor_MoveAbsolutePosition(&Motor_CWE, 20000, 2000,
                nCT_HorStep + ((double)CtParam.nCWEaxisOffset / 10) /
                (MOTOR_CWE_PULLEY_PITCH / MOTOR_CWE_MICROSTEP), 2000);
            Motor_MoveAbsolutePosition(&Motor_CNS, 30000, 2000, nVerStep, 2000);
            Motor_MoveAbsolutePosition(&Motor_V, 15000, 2000,
                45.0 / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP) +
                CtParam.nPaxisPatientOffset, 2000);
            while (!Motor_GetStatus(&Motor_CWE, STATUS_STOP))
                ;
            while (!Motor_GetStatus(&Motor_CNS, STATUS_STOP))
                ;
            while (!Motor_GetStatus(&Motor_V, STATUS_STOP))
                ;
            TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StopCurrent);
            TMC2660_SetCurrent(Motor_CWE.MotType, Motor_CWE.StopCurrent);
            TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StopCurrent);
            UART_SendMessage(DBG_MSG_PC, "[SP_CT_CNSDONE]");
#ifdef USE_TABLET_PC
            UART_SendMessage(DBG_MSG_TABLET, "[SP_CT_CNSDONE]");
            IntTimer_Delay(200);
            while (!IntTimer_GetStatus())
                ;
#endif /* USE_TABLET_PC */
#endif /* USE_MOTOR_CNS_SCREW_PITCH_6_35 */
#endif /* USE_MOTOR_CHINREST_VER */
            CtParam.bStitchMode = SET;
            CtParam.Fov = CT_FOV_15X15;
        }
#endif /* USE_CT_STITCH_MODE */
        else if (strstr(message.Data, UART_BINNING_1X1)) {
            CtParam.Binning = CT_BINNING_1X1;
        }
        else if (strstr(message.Data, UART_BINNING_2X2)) {
            CtParam.Binning = CT_BINNING_2X2;
        }
        else if (strstr(message.Data, UART_MAR_ON)) {
            /* legacy */
        }
        else if (strstr(message.Data, UART_MAR_OFF)) {
            /* legacy */
        }
        else if (strstr(message.Data, UART_TUBE_CONTI)) {
#if defined(USE_CT_XRAY_PULSED_MODE)
            CtParam.TubeMode = CT_TUBE_CONTINUOUS;
#endif
        }
        else if (strstr(message.Data, UART_TUBE_PULSE)) {
#if defined(USE_CT_XRAY_PULSED_MODE)
            CtParam.TubeMode = CT_TUBE_PULSED;
#endif
        }
        else if (strstr(message.Data, UART_XON_TIME_ms)) {
            CtParam.XonTime = atoi(&(message.Data[9]));
        }
        else if (strstr(message.Data, UART_XOFF_TIME_ms)) {
            CtParam.XoffTime = atoi(&(message.Data[9]));
        }
        else if (strstr(message.Data, UART_DELAY_TIME_ms)) {
            CtParam.DelayTime = atoi(&(message.Data[9]));
        }
        else if (strstr(message.Data, UART_TOTAL_FRAME)) {
            CtParam.TotalFrame = atoi(&(message.Data[9]));
        }
        else if (strstr(message.Data, UART_CT_EXP_START_POS)) {
            CtParam.nExpStartPos = atoi(&(message.Data[9]));
        }
        else if (strstr(message.Data, UART_VMOTOR_OFFSET)) {
            CtParam.nMotorVStartPos = atoi(&(message.Data[9]));
            TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
            Motor_MoveAbsolutePosition(&Motor_V, 15000, 1000,
                CtParam.nMotorVStartPos / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP),
                1000);
            while (!Motor_GetStatus(&Motor_V, STATUS_STOP))
                ;
            TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StopCurrent);
        }
#ifdef USE_MOTOR_CHINREST_HOR
        else if (strstr(message.Data, UART_CMD_CHINREST_HOR_MOVE)) {
            int32_t nDistance = atoi(&(message.Data[12]));
            double dValue = 0;
            printUart(DBG_MSG_PC, "H-Axis Distance = %dmm", nDistance);
            if (nDistance >= 0) {
                if (strchr(&(message.Data[11]), 'R'))
                    nDistance = ~nDistance + 1;
                dValue = MOTOR_CWE_INIT_POSITION +
                         ((double)CtParam.nCWEaxisOffset / 10) + nDistance;
                if (dValue < -15 || dValue > 64) {
                    printUart(DBG_MSG_PC, "Unsupported range!");
                } else {
                    CtParam.nCWEaxisFovStep = dValue /
                        (MOTOR_CWE_PULLEY_PITCH / MOTOR_CWE_MICROSTEP);
                }
            }
        }
#endif /* USE_MOTOR_CHINREST_HOR */
#ifdef USE_MOTOR_CHINREST_VER
        else if (strstr(message.Data, UART_CMD_CHINREST_VER_MOVE)) {
            int32_t nDistance = atoi(&(message.Data[12]));
            printUart(DBG_MSG_PC, "V-Axis Distance = %dmm", nDistance);
            if (nDistance >= 0) {
                if (strchr(&(message.Data[11]), 'D'))
                    nDistance = ~nDistance + 1;
                nDistance = MOTOR_CNS_CT_INIT_POSITION +
                            CtParam.nCNSaxisOffset + nDistance;
                if (nDistance < -16 || nDistance > 68) {
                    printUart(DBG_MSG_PC, "Unsupported range!");
                } else {
                    CtParam.nCNSaxisFovStep = (nDistance / MOTOR_CNS_SCREW_PITCH) *
                                              MOTOR_CNS_MICROSTEP;
                    TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StartCurrent);
                    TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StartCurrent);
                    TMC2660_SetCurrent(Motor_CWE.MotType, Motor_CWE.StartCurrent);
                    Motor_MoveAbsolutePosition(&Motor_CNS, 30000, 2000,
                        CtParam.nCNSaxisFovStep, 2000);
                    Motor_MoveAbsolutePosition(&Motor_V, 15000, 2000,
                        CtParam.nPaxisFovStep + CtParam.nPaxisPatientOffset, 2000);
                    Motor_MoveAbsolutePosition(&Motor_CWE, 20000, 2000,
                        (int32_t)CtParam.nCWEaxisFovStep, 2000);
                    while (!Motor_GetStatus(&Motor_CNS, STATUS_STOP))
                        ;
                    while (!Motor_GetStatus(&Motor_CWE, STATUS_STOP))
                        ;
                    while (!Motor_GetStatus(&Motor_V, STATUS_STOP))
                        ;
                    TMC2660_SetCurrent(Motor_V.MotType, Motor_V.StopCurrent);
                    TMC2660_SetCurrent(Motor_CNS.MotType, Motor_CNS.StopCurrent);
                    TMC2660_SetCurrent(Motor_CWE.MotType, Motor_CWE.StopCurrent);
                    UART_SendMessage(DBG_MSG_PC, "[SP_CT_CWEDONE]");
#ifdef USE_TABLET_PC
                    UART_SendMessage(DBG_MSG_TABLET, "[SP_CT_CWEDONE]");
#endif /* USE_TABLET_PC */
                    IntTimer_Delay(200);
                    while (!IntTimer_GetStatus())
                        ;
                    UART_SendMessage(DBG_MSG_PC, "[SP_CT_CNSDONE]");
#ifdef USE_TABLET_PC
                    UART_SendMessage(DBG_MSG_TABLET, "[SP_CT_CNSDONE]");
#endif /* USE_TABLET_PC */
                    IntTimer_Delay(200);
                    while (!IntTimer_GetStatus())
                        ;
                    if (sysInfo.bShowLog == TRUE) {
                        printUart(DBG_MSG_PC, "Vertical : CurrentStep(%d)",
                                  Motor_CNS.CurrentStep);
                        printUart(DBG_MSG_PC, "Horizontal : CurrentStep(%d)",
                                  Motor_CWE.CurrentStep);
                    }
                }
            }
        }
#endif /* USE_MOTOR_CHINREST_VER */
        else if (strstr(message.Data, UART_CMD_PAXIS_MOVE)) {
            int32_t nDistance = atoi(&(message.Data[12]));
            printUart(DBG_MSG_PC, "P-Axis Distance = %dmm", nDistance);
            if (nDistance >= 0 && nDistance < 90) {
                CtParam.nPaxisFovStep = nDistance /
                    (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP);
            }
        }
#ifdef USE_CT_STITCH_MODE
        else if (strstr(message.Data, UART_FOV_15X15_AGING)) {
            CtParam.bStitchBack = SET;
        }
#endif /* USE_CT_STITCH_MODE */
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

#ifdef USE_CT_STITCH_MODE
    if (CtParam.bStitchMode == SET) {
        CT_15X15_RET = StitchUnderSensor_Check();
        if (CtParam.bStitch_Status != CT_15X15_RET) {
            CtParam.bStitch_Status = CT_15X15_RET;
            if (CtParam.bStitch_Status == SET) {
                UART_SendMessage(DBG_MSG_PC, CT_15X15_OK_MESSAGE);
#ifdef USE_TABLET_PC
                UART_SendMessage(DBG_MSG_TABLET, CT_15X15_OK_MESSAGE);
#endif /* USE_TABLET_PC */
                Lamp_Color_Control(LED_BLACK);
            } else if (CtParam.bStitch_Status == RESET) {
                UART_SendMessage(DBG_MSG_PC, CT_15X15_NG_MESSAGE);
#ifdef USE_TABLET_PC
                UART_SendMessage(DBG_MSG_TABLET, CT_15X15_NG_MESSAGE);
#endif /* USE_TABLET_PC */
                Lamp_Color_Control(LED_RED);
            }
            IntTimer_Delay(200);
            while (!IntTimer_GetStatus())
                ;
        }
    }
#endif /* USE_CT_STITCH_MODE */

    return ret;
}

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/
