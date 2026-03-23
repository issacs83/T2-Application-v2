/*
*******************************************************************************
* msg_protocol.c : UART message protocol and queue
*******************************************************************************
* Copyright (C) 2014-2018 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Brief   : 16-byte bracket message framing, queue, real-time dispatch
*
* @ Revision History :
*       1) Extracted from serial.c for modular build --------- 2026-03-24
*******************************************************************************
*/

/* Include files ----------------------------------------------------------- */
#include <string.h>
#include "msg_protocol.h"
#include "uart_hw.h"
#include "system.h"
#include "error_code.h"
#include "tube.h"
#include "sensor.h"
#include "motor.h"
#include "eeprom.h"
#include "calibration.h"
#include "timer.h"
#include "Emergency.h"
#include "boot.h"
#include "misc1.h"
#include "can.h"

/* Private defines --------------------------------------------------------- */

/* Real-time command strings (processed in ISR context, not queued) */
#define UART_MODE_DOWN              "[SM_MODE_DOWN]"
#define UART_MODE_CANCEL            "[SM_MODE_CANCE]"
#define UART_MODE_PANO              "[SM_MODE_PANO"
#define UART_MODE_CT                "[SM_MODE_CT_"
#define UART_MODE_SCAN              "[SM_MODE_SCAN_]"
#define UART_MODE_CALIBRATION       "[SM_MODE_CALI"
#define UART_MODE_GEO_ALIGN         "[SM_MODE_ALIGN]"
#define UART_MODE_RESET             "[SM_MODE_RESET]"
#define UART_MODE_EEPROM            "[SM_MODE_EEPROM]"
#define UART_MODE_DIAGNOSTIC        "[SM_MODE_DIAG_]"

#define UART_XRAY_ON                "[SM_XRAY_ON___]"
#define UART_XRAY_OFF               "[SM_XRAY_OFF__]"
#define UART_XRAY_FRAME_MODE_ON     "[SM_XFRM_ON___]"
#define UART_XRAY_FRAME_MODE_OFF    "[SM_XFRM_OFF__]"

#define UART_POWER_PANO_SEN_ENABLE  "[SM_POW_PS_ON_]"
#define UART_POWER_PANO_SEN_DISABLE "[SM_POW_PS_OFF]"

#define UART_SHOW_LOG_CHANGE        "[SM_SHOW_LOG_"
#define UART_SHOW_MOTOR_LOG_CHANGE  "[SM_SHOW_MOT_"
#define UART_SHOW_QUEUE_LOG_CHANGE  "[SM_SHOW_QUE_"
#define UART_SHOW_MOTOR_CURRENT_STEP "[SM_SHOW_STEP_]"

#define UART_LIVE_Signal            "[SM_LIVE_SIGN_]"
#define UART_QUEUE_CHECK            "[SM_QUEU_CHECK]"
#define UART_QUEUE_LIST_CHECK       "[SM_QULI_CHECK]"

#define UART_MEMBRANELIVE_Signal            "[SM_MEMB_SIGN_]"
#define UART_MEMBRANELIVE_Signal_send       "[SP_MEMB_SIGN_]"
#define UART_MEMBRANELIVE_Signal_receive    "[SM_MEMB_LIVE_]"
#define UART_MEMBRANELIVE_Signal_receive_send   "[SP_MEMB_LIVE_]"
#define UART_MEMBRANELIVE_Signal_receive_err    "[SM_MEMB_ERR"
#define UART_MEMBRANE_FAULT         "[SM_FAULT_"

#define UART_MEMBRANE_COLUMN_UP     "[SM_COLU_MBUP_]"
#define UART_MEMBRANE_COLUMN_DOWN   "[SM_COLU_MBDN_]"
#define UART_MEMBRANE_COLUMN_STOP   "[SM_COLU_MBST_]"

#define UART_CT_CHINREST_NS_AXIS_SET  "[SM_MCNS_MV"
#define UART_CT_CHINREST_WE_AXIS_SET  "[SM_MCWE_MV"
#define UART_CT_CHINREST_VAXIS_SET    "[SM_MVAX_MV"

#define UART_GET_LOCKED_VERSION     "[SM_GET__VER__]"
#define UART_GET_RELEASED_VERSION   "[SM_GET__BUILD]"

/* Private variables ------------------------------------------------------- */
UART_QueueTypedef UART_Queue;
UART_QueueTypedef UART_LIST_Queue;

/* Private function prototypes --------------------------------------------- */
static void queue_init(UART_QueueTypedef *queue);
static bool check_realtime_message(UART_MsgTypedef message);

/* Public functions -------------------------------------------------------- */

void UART_ParseMessage(char data)
{
    static uint32_t start = 0;
    static uint32_t count = 0;
    static UART_MsgTypedef buffer;
    uint8_t index = 0, cnt = 0;

    switch (data) {
    case '[':
        memset(buffer.Data, 0, sizeof(UART_MsgTypedef));
        start = SET;
        count = 0;
        buffer.Data[count++] = data;
        break;

    case ']':
        if (start && (count >= 13)) {
            buffer.Data[count] = data;

            MSG_LIST_Enqueue(&UART_LIST_Queue, buffer);

            if (!check_realtime_message(buffer)) {
                if (!MSG_Enqueue(&UART_Queue, buffer)) {
                    UART_SendMessage(DBG_MSG_PC, ERR_CODE_UART_QUEUE_OVERFLOW);
                    printUart(DBG_MSG_PC, "Head : %d, Tail : %d",
                              UART_Queue.Head, UART_Queue.Tail);
                    index = UART_Queue.Head;
                    printUart(DBG_MSG_PC, "Received Message : %s",
                              buffer.Data);
                    while (cnt < 20) {
                        printUart(DBG_MSG_PC, "Queue %d : %s", index,
                                  UART_Queue.Message[index].Data);
                        index++;
                        cnt++;
                        if (index > 19)
                            index = 0;
                    }
                    queue_init(&UART_Queue);
                }
            }
        }
        start = RESET;
        count = 0;
        memset(buffer.Data, 0, sizeof(UART_MsgTypedef));
        break;

    default:
        if (start && (count < sizeof(UART_MsgTypedef) - 1)) {
            /* Convert lowercase to uppercase */
            if (data > 96 && data < 123)
                data -= 32;
            buffer.Data[count++] = data;
        } else {
            count = 0;
            start = RESET;
            memset(buffer.Data, 0, sizeof(UART_MsgTypedef));
        }
        break;
    }
}

void UART_QueueInitial(void)
{
    queue_init(&UART_Queue);
}

uint32_t MSG_QueueCnt(UART_QueueTypedef *queue)
{
    return ((queue->Head - queue->Tail + UART_QUEUE_SIZE) % UART_QUEUE_SIZE);
}

bool MSG_Enqueue(UART_QueueTypedef *queue, UART_MsgTypedef message)
{
    if (MSG_QueueCnt(queue) == UART_QUEUE_SIZE - 1) {
        return RESET;
    }

    queue->Message[queue->Head++] = message;
    queue->Head %= UART_QUEUE_SIZE;

    return SET;
}

UART_MsgTypedef MSG_Dequeue(UART_QueueTypedef *queue)
{
    UART_MsgTypedef message;

    message = queue->Message[queue->Tail++];
    queue->Tail %= UART_QUEUE_SIZE;

    return message;
}

void MSG_LIST_Enqueue(UART_QueueTypedef *queue, UART_MsgTypedef message)
{
    queue->Message[queue->Head++] = message;
    queue->Head %= UART_QUEUE_SIZE;
}

void UART_SendMessage(USART_TypeDef *uart, char *string)
{
    while (*string) {
        USART_SendData(uart, (*(string++)));
        while (USART_GetFlagStatus(uart, USART_FLAG_TC) == RESET)
            ;
    }

    USART_SendData(uart, '\r');
    while (USART_GetFlagStatus(uart, USART_FLAG_TC) == RESET)
        ;
    USART_SendData(uart, '\n');
    while (USART_GetFlagStatus(uart, USART_FLAG_TC) == RESET)
        ;
}

char UART_WaitProtocol(const char *str)
{
    UART_MsgTypedef msg;
    char ret = RESET;

    while (MSG_QueueCnt(&UART_Queue)) {
        msg = MSG_Dequeue(&UART_Queue);

        if (strstr(msg.Data, str))
            ret = SET;

        msg.Data[1] = 'E';
        printUart(DBG_MSG_PC, msg.Data);
    }

    return ret;
}

/* Private functions ------------------------------------------------------- */

static void queue_init(UART_QueueTypedef *queue)
{
    queue->Head = 0;
    queue->Tail = 0;
    memset(queue->Message, 0, sizeof(UART_MsgTypedef) * UART_QUEUE_SIZE);
}

/**
 * Check if message is a real-time command (processed immediately in ISR
 * context, not queued for main loop). Returns SET if handled, RESET if not.
 *
 * NOTE: This is the original UART_CheckRealTimeMessage from serial.c.
 * It handles mode switching, X-ray on/off, live signals, and other
 * time-critical commands that cannot wait for main-loop polling.
 */
static bool check_realtime_message(UART_MsgTypedef message)
{
    bool ret = SET;

    if (gMachStat.bBootDone == SET || gMachStat.bBootErr == SET) {
        if (strstr(message.Data, UART_MODE_DOWN)) {
            char string[UART_MSG_SIZE];
            memset(string, 0, sizeof(char) * UART_MSG_SIZE);
            sprintf(string, "T2 SCB Bootloader\r\n");
            UART_SendMessage(DBG_MSG_PC, string);
            CHANGE_BOOT_Mode(BOOTFG_APP_TO_BOOTLOADER);
            BOOT_Ram(USER_BOOT_START_ADDRESS);
        }
        else if (strstr(message.Data, UART_MODE_CANCEL)) {
            printUart(DBG_MSG_PC, "[EM_MODE_CANCE]");
            queue_init(&UART_Queue);
            Tube.bXrayFrameOnOff = RESET;
            CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
            PrevCaptureMode = CAPTURE_CANCEL;
#endif /* USE_AGING_MODE */
        }
        else if (strstr(message.Data, UART_MODE_PANO)) {
            if (strchr(&(message.Data[13]), 'T'))
                PanoParam.Init_Mode = PANO_TMJ;
            else
                PanoParam.Init_Mode = PANO_NORMAL;
            IntTimer.bLampTimer = RESET;
            CurCaptureMode = CAPTURE_PANO;
        }
        else if (strstr(message.Data, UART_MODE_CT)) {
            if (strchr(&(message.Data[12]), 'R')) {
                CtParam.bReverse_Status = SET;
                if (strchr(&(message.Data[13]), 'F'))
                    CtParam.Mode_15by9 = CT_FULL_ARCH;
                else if (strchr(&(message.Data[13]), 'S'))
                    CtParam.Mode_15by9 = CT_SINUS;
                else
                    CtParam.Mode_15by9 = CT_OCCLUSION;
            } else {
                CtParam.bReverse_Status = RESET;
                if (strchr(&(message.Data[13]), 'F'))
                    CtParam.Mode_15by9 = CT_FULL_ARCH;
                else if (strchr(&(message.Data[13]), 'S'))
                    CtParam.Mode_15by9 = CT_SINUS;
                else
                    CtParam.Mode_15by9 = CT_OCCLUSION;
            }
            IntTimer.bLampTimer = RESET;
            CurCaptureMode = CAPTURE_CT;
        }
        else if (strstr(message.Data, UART_MODE_SCAN)) {
            IntTimer.bLampTimer = RESET;
            CurCaptureMode = CAPTURE_SCAN;
        }
        else if (strstr(message.Data, UART_MODE_CALIBRATION)) {
            if (strchr(&(message.Data[13]), 'N'))
                CalibrationModeParam.Cal_Mode = Cal_Not_Move;
            else
                CalibrationModeParam.Cal_Mode = Cal_Move;
            IntTimer.bLampTimer = RESET;
            CurCaptureMode = CALIBRATION_MODE;
        }
        else if (strstr(message.Data, UART_MODE_GEO_ALIGN)) {
            IntTimer.bLampTimer = RESET;
            CurCaptureMode = GEOMETRY_ALIGN_MODE;
        }
        else if (strstr(message.Data, UART_MODE_EEPROM)) {
            CurCaptureMode = EEPROM_MODE;
        }
        else if (strstr(message.Data, UART_MODE_RESET)) {
            IntTimer.bLampTimer = RESET;
            CurCaptureMode = RESET_MODE;
        }
        else if (strstr(message.Data, UART_GET_LOCKED_VERSION)) {
            showLockedVersion(SET);
        }
        else if (strstr(message.Data, UART_GET_RELEASED_VERSION)) {
            showReleasedVersion(SET);
        }
        else if (strstr(message.Data, UART_XRAY_ON)) {
            Tube.bXrayOnOff = SET;
            printUart(DBG_MSG_PC, "[EM_XRAY_ON___]");
        }
        else if (strstr(message.Data, UART_XRAY_OFF)) {
            Tube.bXrayOnOff = RESET;
            printUart(DBG_MSG_PC, "[EM_XRAY_OFF__]");
        }
        else if (strstr(message.Data, UART_XRAY_FRAME_MODE_ON)) {
            Tube.bXrayFrameOnOff = SET;
            printUart(DBG_MSG_PC, "[EM_XFRM_ON___]");
        }
        else if (strstr(message.Data, UART_XRAY_FRAME_MODE_OFF)) {
            Tube.bXrayFrameOnOff = RESET;
            printUart(DBG_MSG_PC, "[EM_XFRM_OFF__]");
        }
        else if (strstr(message.Data, UART_POWER_PANO_SEN_ENABLE)) {
            PanoSensor_PowerControl(SET);
            printUart(DBG_MSG_PC, "Sensor-Power Enable");
        }
        else if (strstr(message.Data, UART_POWER_PANO_SEN_DISABLE)) {
            PanoSensor_PowerControl(RESET);
            printUart(DBG_MSG_PC, "Sensor-Power Disable");
        }
        else if (strstr(message.Data, UART_MODE_DIAGNOSTIC)) {
            CurCaptureMode = DIAGNOSTIC_MODE;
        }
        else if (strstr(message.Data, UART_SHOW_LOG_CHANGE)) {
            if (strchr(&(message.Data[13]), 'Y')) {
                unsigned short nOffset = EEPRom_I2C_Read_Word(ROM_FIRMWARE_SETTING_ADDR);
                printUart(DBG_MSG_PC, "[T2] SHOW LOG : YES");
                if ((nOffset & 0x0001) == 0x0000) {
                    nOffset = nOffset | 0x0001;
                    EEPRom_I2C_Write_Word(ROM_FIRMWARE_SETTING_ADDR, (uint16_t)nOffset);
                }
                sysInfo.bShowLog = SET;
                printUart(DBG_MSG_PC, "DONE");
            } else {
                unsigned short nOffset = EEPRom_I2C_Read_Word(ROM_FIRMWARE_SETTING_ADDR);
                printUart(DBG_MSG_PC, "[T2] SHOW LOG : NO");
                if ((nOffset & 0x0001) == 0x0001) {
                    nOffset = nOffset & 0xFFF0;
                    EEPRom_I2C_Write_Word(ROM_FIRMWARE_SETTING_ADDR, (uint16_t)nOffset);
                }
                sysInfo.bShowLog = RESET;
                printUart(DBG_MSG_PC, "DONE");
            }
        }
        else if (strstr(message.Data, UART_SHOW_MOTOR_LOG_CHANGE)) {
            if (strchr(&(message.Data[13]), 'Y')) {
                unsigned short nOffset = EEPRom_I2C_Read_Word(ROM_FIRMWARE_SETTING_ADDR);
                printUart(DBG_MSG_PC, "[T2] SHOW MOTOR LOG : YES");
                if ((nOffset & 0x0010) == 0x0000) {
                    nOffset = nOffset | 0x0010;
                    EEPRom_I2C_Write_Word(ROM_FIRMWARE_SETTING_ADDR, (uint16_t)nOffset);
                }
                sysInfo.bShowMotorLog = SET;
                printUart(DBG_MSG_PC, "DONE");
            } else {
                unsigned short nOffset = EEPRom_I2C_Read_Word(ROM_FIRMWARE_SETTING_ADDR);
                printUart(DBG_MSG_PC, "[T2] SHOW MOTOR LOG : NO");
                if ((nOffset & 0x0010) == 0x0010) {
                    nOffset = nOffset & 0xFF0F;
                    EEPRom_I2C_Write_Word(ROM_FIRMWARE_SETTING_ADDR, (uint16_t)nOffset);
                }
                sysInfo.bShowMotorLog = RESET;
                printUart(DBG_MSG_PC, "DONE");
            }
        }
        else if (strstr(message.Data, UART_SHOW_QUEUE_LOG_CHANGE)) {
            if (strchr(&(message.Data[13]), 'Y')) {
                unsigned short nOffset = EEPRom_I2C_Read_Word(ROM_FIRMWARE_SETTING_ADDR);
                printUart(DBG_MSG_PC, "[T2] SHOW QUEUE LOG : YES");
                if ((nOffset & 0x0100) == 0x0000) {
                    nOffset = nOffset | 0x0100;
                    EEPRom_I2C_Write_Word(ROM_FIRMWARE_SETTING_ADDR, (uint16_t)nOffset);
                }
                sysInfo.bShowQueueLog = SET;
                printUart(DBG_MSG_PC, "DONE");
            } else {
                unsigned short nOffset = EEPRom_I2C_Read_Word(ROM_FIRMWARE_SETTING_ADDR);
                printUart(DBG_MSG_PC, "[T2] SHOW QUEUE LOG : NO");
                if ((nOffset & 0x0100) == 0x0100) {
                    nOffset = nOffset & 0xF0FF;
                    EEPRom_I2C_Write_Word(ROM_FIRMWARE_SETTING_ADDR, (uint16_t)nOffset);
                }
                sysInfo.bShowQueueLog = RESET;
                printUart(DBG_MSG_PC, "DONE");
            }
        }
        else if (strstr(message.Data, UART_CT_CHINREST_NS_AXIS_SET)) {
            int32_t nDistance = atoi(&(message.Data[12]));
            printUart(DBG_MSG_PC, "CT CNS-Axis Distance = %dmm", nDistance);
            if (nDistance >= 0) {
                if (strchr(&(message.Data[11]), 'D'))
                    nDistance = ~nDistance + 1;
                nDistance = MOTOR_CNS_CT_INIT_POSITION + CtParam.nCNSaxisOffset + nDistance;
                if (nDistance < -16 || nDistance > 68) {
                    printUart(DBG_MSG_PC, "Unsupported range!");
                } else {
                    CtParam.nCNSaxisFovStep = (nDistance / MOTOR_CNS_SCREW_PITCH) * MOTOR_CNS_MICROSTEP;
                    CtParam.bCNSaxisSET = SET;
                }
            }
        }
        else if (strstr(message.Data, UART_CT_CHINREST_WE_AXIS_SET)) {
            int32_t nDistance = atoi(&(message.Data[12]));
            double dValue = 0;
            printUart(DBG_MSG_PC, "CT CWE-Axis Distance = %dmm", nDistance);
            if (nDistance >= 0) {
                if (strchr(&(message.Data[11]), 'R'))
                    nDistance = ~nDistance + 1;
                dValue = MOTOR_CWE_INIT_POSITION + ((double)CtParam.nCWEaxisOffset / 10) + nDistance;
                if (dValue < -15 || dValue > 64) {
                    printUart(DBG_MSG_PC, "Unsupported range!");
                } else {
                    CtParam.nCWEaxisFovStep = dValue / (MOTOR_CWE_PULLEY_PITCH / MOTOR_CWE_MICROSTEP);
                    CtParam.bCWEaxisSET = SET;
                }
            }
        }
        else if (strstr(message.Data, UART_CT_CHINREST_VAXIS_SET)) {
            int32_t nDistance = atoi(&(message.Data[12]));
            printUart(DBG_MSG_PC, "CT V-Axis Distance = %dmm", nDistance);
            if (nDistance >= 0 && nDistance < 90) {
                CtParam.nPaxisFovStep = nDistance / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP) + CtParam.nPaxisPatientOffset;
                CtParam.bPaxisSET = SET;
            }
        }
        else if (strstr(message.Data, UART_SHOW_MOTOR_CURRENT_STEP)) {
            printUart(DBG_MSG_PC, "Motor R::CurrentStep = %d", Motor_R.CurrentStep);
            printUart(DBG_MSG_PC, "Motor V::CurrentStep = %d", Motor_V.CurrentStep);
            printUart(DBG_MSG_PC, "Motor A::CurrentStep = %d", Motor_A.CurrentStep);
            printUart(DBG_MSG_PC, "Motor T::CurrentStep = %d", Motor_T.CurrentStep);
            printUart(DBG_MSG_PC, "Motor C::CurrentStep = %d", Motor_C.CurrentStep);
            printUart(DBG_MSG_PC, "Motor S::CurrentStep = %d", Motor_S.CurrentStep);
            printUart(DBG_MSG_PC, "Motor CNS::CurrentStep = %d", Motor_CNS.CurrentStep);
            printUart(DBG_MSG_PC, "Motor CWE::CurrentStep = %d", Motor_CWE.CurrentStep);
        }
        else if (strstr(message.Data, UART_LIVE_Signal)) {
            if (!EMG_ProcessSwitch()) {
                UART_SendMessage(DBG_MSG_PC, EMERGENCY_SWITCH_ON);
#ifdef USE_TABLET_PC
                UART_SendMessage(DBG_MSG_TABLET, EMERGENCY_SWITCH_ON);
#endif /* USE_TABLET_PC */
            }
            printUart(DBG_MSG_PC, "[EM_LIVE_SIGN_]");
        }
        else if (strstr(message.Data, UART_MEMBRANELIVE_Signal)) {
#ifdef USE_TABLET_PC
            UART_SendMessage(DBG_MSG_TABLET, UART_MEMBRANELIVE_Signal_send);
#endif /* USE_TABLET_PC */
        }
        else if (strstr(message.Data, UART_MEMBRANELIVE_Signal_receive)) {
            UART_SendMessage(DBG_MSG_PC, UART_MEMBRANELIVE_Signal_receive_send);
        }
        else if (strstr(message.Data, UART_MEMBRANELIVE_Signal_receive_err)) {
            message.Data[2] = 'P';
            UART_SendMessage(DBG_MSG_PC, message.Data);
        }
        else if (strstr(message.Data, UART_MEMBRANE_FAULT)) {
            message.Data[2] = 'P';
            UART_SendMessage(DBG_MSG_PC, message.Data);
        }
        else if (strstr(message.Data, UART_QUEUE_CHECK)) {
            uint8_t index = 0, cnt = 0;
            index = UART_Queue.Head;
            while (cnt < 20) {
                printUart(DBG_MSG_PC, "Queue %d : %s", index,
                          UART_Queue.Message[index].Data);
                index++;
                cnt++;
                if (index > 19)
                    index = 0;
            }
        }
        else if (strstr(message.Data, UART_QUEUE_LIST_CHECK)) {
            uint8_t index = 0, cnt = 0;
            index = UART_LIST_Queue.Head;
            while (cnt < 20) {
                printUart(DBG_MSG_PC, "Queue %d : %s", index,
                          UART_LIST_Queue.Message[index].Data);
                index++;
                cnt++;
                if (index > 19)
                    index = 0;
            }
        }
        else if (strstr(message.Data, UART_MEMBRANE_COLUMN_UP)) {
            Column_Control(COLUMN_UP);
            g_bColumnPressed = SET;
        }
        else if (strstr(message.Data, UART_MEMBRANE_COLUMN_DOWN)) {
            Column_Control(COLUMN_DOWN);
            g_bColumnPressed = SET;
        }
        else if (strstr(message.Data, UART_MEMBRANE_COLUMN_STOP)) {
            Column_Control(COLUMN_STOP);
            g_bColumnPressed = RESET;
        }
#ifdef USE_AGING_MODE
#define UART_AGING_MODE_ENTER   "[SM_AGING_ENTE]"
#define UART_AGING_MODE_EXIT    "[SM_AGING_EXIT]"
        else if (strstr(message.Data, UART_AGING_MODE_ENTER)) {
            printUart(DBG_MSG_PC, "[T2] Aging Mode is started >>>>>>>>>>>>>>>>>>");
            CurCaptureMode = CAPTURE_PANO;
            g_bAgingMode = SET;
        }
        else if (strstr(message.Data, UART_AGING_MODE_EXIT)) {
            printUart(DBG_MSG_PC, "[T2] Aging Mode is finished >>>>>>>>>>>>>>>>>>");
            printUart(DBG_MSG_PC, "");
            printUart(DBG_MSG_PC, "[T2] Aging Mode Result :");
            printUart(DBG_MSG_PC, "--------------------------------------------------------------");
            printUart(DBG_MSG_PC, "* Aging Count :: Pano(%d), CT(%d), Ceph(%d)",
                      g_nPanoCnt, g_nCtCnt, g_nCephCnt);
            printUart(DBG_MSG_PC, "--------------------------------------------------------------");
            printUart(DBG_MSG_PC, "");
            CurCaptureMode = CAPTURE_CANCEL;
            g_bAgingMode = RESET;
        }
#endif /* USE_AGING_MODE */
        else {
            ret = RESET;
        }
    }

    return ret;
}

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/
