/*
*******************************************************************************
* cmd_system.c : System commands, exposure, collimator, membrane
*******************************************************************************
* Copyright (C) 2014-2018 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Brief   : System-level command handlers: capture param validation,
*             exposure SET, collimator auto-align, membrane management.
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
#include "can.h"
#include "tube.h"
#include "eeprom.h"
#include "calibration.h"
#include "sensor.h"
#include "motor.h"

/* Private defines --------------------------------------------------------- */
#define UART_ALIGN_AUTO_COLLI_ALL       "[SM_COLA_M"
#define UART_ALIGN_AUTO_COLLI_TOP       "[SM_COLT_M"
#define UART_ALIGN_AUTO_COLLI_BOT       "[SM_COLB_M"
#define UART_ALIGN_AUTO_COLLI_LEFT      "[SM_COLL_M"
#define UART_ALIGN_AUTO_COLLI_RIGHT     "[SM_COLR_M"

#define UART_ALIGN_AUTO_COLLI_ALL_PANO  "[SM_COLAPM"
#define UART_ALIGN_AUTO_COLLI_ALL_CT    "[SM_COLACM"
#define UART_ALIGN_AUTO_COLLI_ALL_SCAN  "[SM_COLASM"

#define UART_ALIGN_AUTO_COLLI_ALL_STOP              "[SM_COLA_STOP_]"
#define UART_ALIGN_AUTO_COLLI_PANO_TOP_STOP         "[SM_COLT_PSTOP]"
#define UART_ALIGN_AUTO_COLLI_PANO_BOT_STOP         "[SM_COLB_PSTOP]"
#define UART_ALIGN_AUTO_COLLI_PANO_LEFT_STOP        "[SM_COLL_PSTOP]"
#define UART_ALIGN_AUTO_COLLI_PANO_RIGHT_STOP       "[SM_COLR_PSTOP]"
#define UART_ALIGN_AUTO_COLLI_CT_TOP_STOP           "[SM_COLT_CSTOP]"
#define UART_ALIGN_AUTO_COLLI_CT_BOT_STOP           "[SM_COLB_CSTOP]"
#define UART_ALIGN_AUTO_COLLI_CT_LEFT_STOP          "[SM_COLL_CSTOP]"
#define UART_ALIGN_AUTO_COLLI_CT_RIGHT_STOP         "[SM_COLR_CSTOP]"
#define UART_ALIGN_AUTO_COLLI_SCAN_TOP_STOP         "[SM_COLT_SSTOP]"
#define UART_ALIGN_AUTO_COLLI_SCAN_BOT_STOP         "[SM_COLB_SSTOP]"
#define UART_ALIGN_AUTO_COLLI_SCAN_LEFT_STOP        "[SM_COLL_SSTOP]"
#define UART_ALIGN_AUTO_COLLI_SCAN_RIGHT_STOP       "[SM_COLR_SSTOP]"

/* Public functions -------------------------------------------------------- */

bool CMD_CheckCaptureParam(void)
{
    bool ret = RESET;

    if (CurCaptureMode == CAPTURE_PANO) {
        if ((PanoParam.KV != 0) && (PanoParam.mA != 0) &&
            (PanoParam.Mode != PANO_MODE_NONE) &&
            (PanoParam.Arch != PANO_ARCH_NONE) &&
            (PanoParam.Scan != PANO_SCAN_NONE)) {
            return SET;
        }
    }
    else if (CurCaptureMode == CAPTURE_CT) {
        if ((CtParam.KV != 0) && (CtParam.mA != 0) &&
            (CtParam.Binning != CT_BINNING_NONE) &&
            (CtParam.XonTime != 0) && (CtParam.DelayTime != 0)) {
#ifdef USE_CT_STITCH_MODE
            if (CtParam.bStitchMode == SET) {
                ret = StitchUnderSensor_Check();
                if (ret == SET)
                    return SET;
                else
                    UART_SendMessage(DBG_MSG_PC, CT_15X15_NG_MESSAGE);
            } else
#endif /* USE_CT_STITCH_MODE */
                return SET;
        }
    }
    else if (CurCaptureMode == CAPTURE_SCAN) {
        if ((CephParam.KV != 0) && (CephParam.mA != 0) &&
            (CephParam.Time != 0) && (CephParam.Size != 0)) {
            return SET;
        }
    }
    UART_SendMessage(DBG_MSG_PC, CONFIRM_NG_MESSAGE);
    return RESET;
}

char UART_Exposure_SET(void)
{
    UART_MsgTypedef message;
    char string[UART_MSG_SIZE];
    char ret = RESET;

    memset(string, 0, sizeof(char) * UART_MSG_SIZE);

    while (MSG_QueueCnt(&UART_Queue)) {
        message = MSG_Dequeue(&UART_Queue);

        if (strstr(message.Data, UART_XRAY_EXPOSURE)) {
            ret = 1;
        }
        else if (strstr(message.Data, UART_XRAY_EXPOSURE_STOP)) {
            ret = 2;
        }
        else {
            char string1[UART_MSG_SIZE * 2 + 3];
            memset(string1, 0, sizeof(char) * (UART_MSG_SIZE * 2 + 3));
            sprintf(string1, "%s : %s", ERR_CODE_UART_MSG_ERROR, message.Data);
            UART_SendMessage(DBG_MSG_PC, string1);
            return RESET;
        }

        sprintf(string, "%s", message.Data);
        string[1] = 'E';
        UART_SendMessage(DBG_MSG_PC, string);
    }

    return ret;
}

char UART_Collimator_Auto_SET(void)
{
    UART_MsgTypedef message;
    char string[UART_MSG_SIZE];
    char ret = RESET;

    memset(string, 0, sizeof(char) * UART_MSG_SIZE);
    CalibrationModeParam.nColli_Distance = 0;

    while (MSG_QueueCnt(&UART_Queue)) {
        message = MSG_Dequeue(&UART_Queue);

        if (strstr(message.Data, UART_ALIGN_AUTO_COLLI_ALL)) {
            CalibrationModeParam.nColli_Distance = atoi(&(message.Data[10]));
            ret = 1;
        }
        else if (strstr(message.Data, UART_ALIGN_AUTO_COLLI_TOP)) {
            CalibrationModeParam.nColli_Distance = atoi(&(message.Data[10]));
            ret = 2;
        }
        else if (strstr(message.Data, UART_ALIGN_AUTO_COLLI_BOT)) {
            CalibrationModeParam.nColli_Distance = atoi(&(message.Data[10]));
            ret = 3;
        }
        else if (strstr(message.Data, UART_ALIGN_AUTO_COLLI_LEFT)) {
            CalibrationModeParam.nColli_Distance = atoi(&(message.Data[10]));
            ret = 4;
        }
        else if (strstr(message.Data, UART_ALIGN_AUTO_COLLI_RIGHT)) {
            CalibrationModeParam.nColli_Distance = atoi(&(message.Data[10]));
            ret = 5;
        }
        else if (strstr(message.Data, UART_ALIGN_AUTO_COLLI_ALL_PANO)) {
            CalibrationModeParam.nColli_Distance = atoi(&(message.Data[10]));
            ret = 6;
        }
        else if (strstr(message.Data, UART_ALIGN_AUTO_COLLI_ALL_CT)) {
            CalibrationModeParam.nColli_Distance = atoi(&(message.Data[10]));
            ret = 7;
        }
        else if (strstr(message.Data, UART_ALIGN_AUTO_COLLI_ALL_SCAN)) {
            CalibrationModeParam.nColli_Distance = atoi(&(message.Data[10]));
            ret = 8;
        }
        else {
            char string1[UART_MSG_SIZE * 2 + 3];
            memset(string1, 0, sizeof(char) * (UART_MSG_SIZE * 2 + 3));
            sprintf(string1, "%s : %s", ERR_CODE_UART_MSG_ERROR, message.Data);
            UART_SendMessage(DBG_MSG_PC, string1);
            return RESET;
        }

        sprintf(string, "%s", message.Data);
        string[1] = 'E';
        UART_SendMessage(DBG_MSG_PC, string);
    }

    return ret;
}

char UART_Collimator_Auto_STOP(void)
{
    UART_MsgTypedef message;
    char string[UART_MSG_SIZE];
    char ret = RESET;

    memset(string, 0, sizeof(char) * UART_MSG_SIZE);

    while (MSG_QueueCnt(&UART_Queue)) {
        message = MSG_Dequeue(&UART_Queue);

        if (strstr(message.Data, UART_ALIGN_AUTO_COLLI_ALL_STOP))
            ret = 1;
        else if (strstr(message.Data, UART_ALIGN_AUTO_COLLI_PANO_TOP_STOP))
            ret = 2;
        else if (strstr(message.Data, UART_ALIGN_AUTO_COLLI_PANO_BOT_STOP))
            ret = 3;
        else if (strstr(message.Data, UART_ALIGN_AUTO_COLLI_PANO_LEFT_STOP))
            ret = 4;
        else if (strstr(message.Data, UART_ALIGN_AUTO_COLLI_PANO_RIGHT_STOP))
            ret = 5;
        else if (strstr(message.Data, UART_ALIGN_AUTO_COLLI_CT_TOP_STOP))
            ret = 6;
        else if (strstr(message.Data, UART_ALIGN_AUTO_COLLI_CT_BOT_STOP))
            ret = 7;
        else if (strstr(message.Data, UART_ALIGN_AUTO_COLLI_CT_LEFT_STOP))
            ret = 8;
        else if (strstr(message.Data, UART_ALIGN_AUTO_COLLI_CT_RIGHT_STOP))
            ret = 9;
        else if (strstr(message.Data, UART_ALIGN_AUTO_COLLI_SCAN_TOP_STOP))
            ret = 10;
        else if (strstr(message.Data, UART_ALIGN_AUTO_COLLI_SCAN_BOT_STOP))
            ret = 11;
        else if (strstr(message.Data, UART_ALIGN_AUTO_COLLI_SCAN_LEFT_STOP))
            ret = 12;
        else if (strstr(message.Data, UART_ALIGN_AUTO_COLLI_SCAN_RIGHT_STOP))
            ret = 13;
        else if (strstr(message.Data, UART_XRAY_EXPOSURE_STOP))
            ret = 14;
        else {
            char string1[UART_MSG_SIZE * 2 + 3];
            memset(string1, 0, sizeof(char) * (UART_MSG_SIZE * 2 + 3));
            sprintf(string1, "%s : %s", ERR_CODE_UART_MSG_ERROR, message.Data);
            UART_SendMessage(DBG_MSG_PC, string1);
            return RESET;
        }

        sprintf(string, "%s", message.Data);
        string[1] = 'E';
        UART_SendMessage(DBG_MSG_PC, string);
    }

    return ret;
}

char UART_Memb_Ver_Check(char verFlag)
{
    UART_MsgTypedef message;
    char string[UART_MSG_SIZE];

    memset(string, 0, sizeof(char) * UART_MSG_SIZE);

#ifdef USE_TABLET_PC
    printUart(DBG_MSG_TABLET, "[SP_GET__BUILD]");
#endif /* USE_TABLET_PC */

    if (MSG_QueueCnt(&UART_Queue)) {
        message = MSG_Dequeue(&UART_Queue);

        if (strstr(message.Data, UART_RECEIVE_MEMB_VERSION)) {
            char SubVer = message.Data[8];
            uint16_t BuildVer = atoi(&(message.Data[9]));
            SubVer += 32;
            printUart(DBG_MSG_PC, "Membrane F/W Build : %d.%02d.%02d",
                      BuildVer / 10000, (BuildVer % 10000) / 100,
                      (BuildVer % 100));
            if (verFlag == TRUE)
                printUart(DBG_MSG_PC, "Membrane F/W Sub Build : %c", SubVer);
            IntTimer_Stop();
            return SET;
        } else {
            char string1[UART_MSG_SIZE * 2 + 3];
            memset(string1, 0, sizeof(char) * (UART_MSG_SIZE * 2 + 3));
            sprintf(string1, "%s : %s", ERR_CODE_UART_MSG_ERROR, message.Data);
            UART_SendMessage(DBG_MSG_PC, string1);
        }
    }

    if (IntTimer_GetStatus()) {
        printUart(DBG_MSG_PC, "No Response : Check the Membrane Device");
        return SET;
    }

    return RESET;
}

void UART_Memb_Init_Data(void)
{
    char string[64] = {0,};

    sprintf(string, "[SP_VOLUME_");
#ifdef USE_TABLET_PC
    printUart(DBG_MSG_TABLET, "%s%03d]", string, sysInfo.nMembrane_Sound_Value);
#endif /* USE_TABLET_PC */
    IntTimer_Delay(200);
    while (!IntTimer_GetStatus())
        ;

    sprintf(string, "[SP_MELO_SEL");
#ifdef USE_TABLET_PC
    printUart(DBG_MSG_TABLET, "%s%02d]", string, sysInfo.nMembrane_Sound_Select);
#endif /* USE_TABLET_PC */
    IntTimer_Delay(200);
    while (!IntTimer_GetStatus())
        ;

    sprintf(string, "[SP_SWDLY_");
#ifdef USE_TABLET_PC
    printUart(DBG_MSG_TABLET, "%s%04d]", string, sysInfo.nMembrane_Button_Delay);
#endif /* USE_TABLET_PC */
    IntTimer_Delay(200);
    while (!IntTimer_GetStatus())
        ;

    sprintf(string, "[[SP_CCDLY_");
#ifdef USE_TABLET_PC
    printUart(DBG_MSG_TABLET, "%s%04d]", string, sysInfo.nMembrane_Cancel_Delay);
#endif /* USE_TABLET_PC */
    IntTimer_Delay(200);
    while (!IntTimer_GetStatus())
        ;
}

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/
