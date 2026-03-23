/*
 * scan_sequence.c : Non-blocking cephalometric scan state machine
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Converts the blocking ScanCapture() function into a non-blocking
 * step-based state machine. Each call to ScanSequence_Step() processes
 * one state transition and returns immediately.
 *
 * Original logic preserved from Legacy/src/scan_capture.c.
 *
 * Memory: ~2800 bytes Flash, ~140 bytes SRAM.
 */

#include "scan_sequence.h"
#include "motor_sequence.h"

/* Legacy headers */
#include "extern.h"
#include "system.h"
#include "error_code.h"
#include "motor.h"
#include "sensor.h"
#include "can.h"
#include "tube.h"
#include "serial.h"
#include "misc1.h"
#include "timer.h"
#include "safety.h"
#include "motor_compat.h"
#include "debug_log.h"

extern bool CAN_Collimator_SendMessage(uint16_t cmd, uint32_t value,
                                         uint16_t debug, uint32_t timeOut,
                                         uint8_t retryCnt);

/* ------------------------------------------------------------------ */
/* Constants from legacy scan_capture.c                                */
/* ------------------------------------------------------------------ */
#define CEPH_SCAN_TOTAL_DISTANCE        300  /* mm */
#define CEPH_2SEC_SCAN_STOP_DISTANCE    200  /* mm */
#define CEPH_SCAN_TIME_10SEC            10000 /* ms */
#define CEPH_SCAN_TIME_5SEC             5000  /* ms */
#define CEPH_SCAN_TIME_2SEC             2000  /* ms */

/* ------------------------------------------------------------------ */
/* Private state                                                       */
/* ------------------------------------------------------------------ */
static ScanState_t   g_scan_state;
static MotorWait_t   g_scan_mw;
static bool          g_scan_stop_flag;
static BitAction     g_scan_exp_check;

/* Scan parameters computed during setup */
static uint32_t      g_scan_start_cnt;
static uint32_t      g_scan_end_cnt;
static uint32_t      g_scan_two_sec_end_cnt;
static uint32_t      g_scan_distance;
static bool          g_scan_exposure_started;
static bool          g_scan_sensor_started;
static bool          g_scan_exposure_ended;

/* ------------------------------------------------------------------ */
/* Forward declarations                                                */
/* ------------------------------------------------------------------ */
static void scan_clear_param(void);
static void scan_clear_process(void);
static void scan_cancel_process(void);

/* ------------------------------------------------------------------ */
/* State transition helper                                             */
/* ------------------------------------------------------------------ */
#ifdef BUILD_TYPE_DEBUG
static const char* const k_scan_state_names[] = {
    "IDLE", "CHECK_SYS", "CAN_CHK", "TILT_OFF", "CLR_PARAM",
    "MOV_INIT", "TILT_ON", "LASER_ON", "WAIT_PAR", "MOT_START",
    "LASER_OFF", "COLLI_SET", "MODE_SET", "TUBE_SET", "WAIT_2SEC",
    "EXP_SET", "WAIT_EXP", "XRAY_ILK", "TUBE_RDY", "TUBE_HEAT",
    "START_CAP", "EXPOSING", "WAIT_END", "COMPLETE", "STOP",
    "CANCEL", "ERROR"
};
#endif

static void scan_transition(ScanState_t new_state)
{
    g_scan_state = new_state;
#ifdef BUILD_TYPE_DEBUG
    if (new_state < SCAN_STATE_COUNT) {
        DbgLog_Printf("SCAN: -> %s", k_scan_state_names[new_state]);
    }
#endif
}

/* ================================================================== */
/* Public API                                                          */
/* ================================================================== */

void ScanSequence_Init(void)
{
    g_scan_state = SCAN_IDLE;
    g_scan_stop_flag = false;
    MotorWait_Init(&g_scan_mw, 30000);
}

void ScanSequence_Start(void)
{
    if (g_scan_state != SCAN_IDLE) {
        return;
    }
    g_scan_stop_flag = false;
    g_scan_exposure_started = false;
    g_scan_sensor_started = false;
    g_scan_exposure_ended = false;
    scan_transition(SCAN_CHECK_SYSTEM);
}

bool ScanSequence_IsDone(void)
{
    return (g_scan_state == SCAN_IDLE);
}

ScanState_t ScanSequence_GetState(void)
{
    return g_scan_state;
}

void ScanSequence_Cancel(void)
{
    if (g_scan_state == SCAN_IDLE) {
        return;
    }

    Tube_CtrlReady(TUBE_READY_DISABLE);
    Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
    Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
    Indicator_Control(INDICATOR_DISABLE);
    ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE);

    CurCaptureMode = CAPTURE_CANCEL;
    Motor_CephScanEnd();
    scan_cancel_process();
    scan_transition(SCAN_IDLE);
}

void ScanSequence_Run(void)
{
    if (sysInfo.model_id != MODEL_T2_CS) {
        printUart(DBG_MSG_PC, "can't not support cephalo");
        CurCaptureMode = CAPTURE_CANCEL;
        return;
    }

    /* Legacy blocking compatibility wrapper */
    ScanSequence_Init();
    ScanSequence_Start();
    while (!ScanSequence_IsDone()) {
        ScanSequence_Step();
        Safety_IWDG_Kick();
    }
}

/* ================================================================== */
/* Step function                                                       */
/* ================================================================== */

void ScanSequence_Step(void)
{
    switch (g_scan_state) {

    /* -------------------------------------------------------------- */
    case SCAN_IDLE:
        break;

    /* -------------------------------------------------------------- */
    case SCAN_CHECK_SYSTEM:
#ifdef USE_AGING_MODE
        printUart(DBG_MSG_PC, "Cephalo Aging Count :: %d", g_nCephCnt);
        SysTime_MesureStart(__FUNCTION__, __LINE__);
#endif

        if (gMachStat.bFirstEntry == RESET) {
            printUart(DBG_MSG_PC, "==First Entry after boot==");
            gMachStat.bFirstEntry = SET;
        }

        if (checkPreCaptureStatus() != TRUE) {
            printUart(DBG_MSG_PC, "checkPreCaptureStatus :: FALSE");
            scan_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
            PrevCaptureMode = CAPTURE_CANCEL;
#endif
            scan_transition(SCAN_IDLE);
            break;
        }

        UART_SendMessage(DBG_MSG_PC, "[SP_MODE_SCAN_]");
#ifdef USE_TABLET_PC
        UART_SendMessage(DBG_MSG_TABLET, "[SP_MODE_SCAN_]");
#endif

        if (g_bColumnPressed == SET) {
            Column_Control(COLUMN_STOP);
            g_bColumnPressed = FALSE;
        }

        scan_transition(SCAN_CAN_COMM_CHECK);
        break;

    /* -------------------------------------------------------------- */
    case SCAN_CAN_COMM_CHECK:
        if (CAN_SendMessage(CAN_TUBE_COM_CHECK, CAN_TUBE_GEN2_COM, 0, 3000, 2)) {
            CAN_SendMessage(CAN_TUBE_TANK_TEMP_READ, 0, 0, 1000, 2);
        } else {
            UART_SendMessage(DBG_MSG_PC, "Tube Communication Fail.");
            scan_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
            PrevCaptureMode = CAPTURE_CANCEL;
#endif
            scan_transition(SCAN_IDLE);
            break;
        }
        scan_transition(SCAN_COLLIMATOR_TILT_OFF);
        break;

    /* -------------------------------------------------------------- */
    case SCAN_COLLIMATOR_TILT_OFF:
        if (!CAN_Collimator_SendMessage(CMD_TILT_OFF, 0, 0, 5000, 2)) {
            UART_SendMessage(DBG_MSG_PC, "Collimator Communication Fail.");
            scan_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
            PrevCaptureMode = CAPTURE_CANCEL;
#endif
            scan_transition(SCAN_IDLE);
            break;
        }
        g_bTiltStatus = FALSE;
        scan_transition(SCAN_CLEAR_PARAMS);
        break;

    /* -------------------------------------------------------------- */
    case SCAN_CLEAR_PARAMS:
        scan_clear_param();
        scan_transition(SCAN_MOVE_TO_INIT);
        break;

    /* -------------------------------------------------------------- */
    case SCAN_MOVE_TO_INIT:
        if (!Motor_MoveInitPosition()) {
            UART_SendMessage(DBG_MSG_PC, "Motor Init Position Fail.");
            scan_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
            PrevCaptureMode = CAPTURE_CANCEL;
#endif
            scan_transition(SCAN_IDLE);
            break;
        }

        if (CurCaptureMode != CAPTURE_SCAN) {
            scan_cancel_process();
#ifdef USE_AGING_MODE
            PrevCaptureMode = CAPTURE_CANCEL;
#endif
            scan_transition(SCAN_IDLE);
            break;
        }

#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
        ElapseTimerOnOff(TRUE);
#endif
        scan_transition(SCAN_COLLIMATOR_TILT_ON);
        break;

    /* -------------------------------------------------------------- */
    case SCAN_COLLIMATOR_TILT_ON:
        if (!CAN_Collimator_SendMessage(CMD_TILT_ON, 0, 0, 5000, 2)) {
            UART_SendMessage(DBG_MSG_PC, "Tilt Fail.");
            scan_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
            PrevCaptureMode = CAPTURE_CANCEL;
#endif
            scan_transition(SCAN_IDLE);
            break;
        }
        g_bTiltStatus = TRUE;
        scan_transition(SCAN_LASER_ON);
        break;

    /* -------------------------------------------------------------- */
    case SCAN_LASER_ON:
        LaserSetPosition();
        LaserControl(TYPE_HEAD_CEPH, TRUE);

        if (sysInfo.initDir_MotCC == INIT_DIR_REVERSE) {
            CephParam.bInitPosFlag = FALSE;
        }

        UART_SendMessage(DBG_MSG_PC, "[SP_SCAN_READY]");
#ifdef USE_TABLET_PC
        UART_SendMessage(DBG_MSG_TABLET, "[SP_SCAN_READY]");
#endif

        if (sysInfo.bShowLog == TRUE) {
            printUart(DBG_MSG_PC, "Cephalo Init Motor R::CurrentStep = %d", Motor_R.CurrentStep);
            printUart(DBG_MSG_PC, "Cephalo Init Motor V::CurrentStep = %d", Motor_V.CurrentStep);
            printUart(DBG_MSG_PC, "Cephalo Init Motor A::CurrentStep = %d", Motor_A.CurrentStep);
            printUart(DBG_MSG_PC, "Cephalo Init Motor CC::CurrentStep = %d", Motor_C.CurrentStep);
            printUart(DBG_MSG_PC, "Cephalo Init Motor CS::CurrentStep = %d", Motor_S.CurrentStep);
        }

        scan_transition(SCAN_WAIT_PARAMS);
        break;

    /* -------------------------------------------------------------- */
    case SCAN_WAIT_PARAMS:
#ifdef USE_AGING_MODE
        if (g_bAgingMode == TRUE) {
            scan_transition(SCAN_MOTOR_START_POS);
            break;
        }
#endif
        if (UART_SetCephParam()) {
            scan_transition(SCAN_MOTOR_START_POS);
        } else if (CurCaptureMode == CAPTURE_CANCEL) {
            scan_cancel_process();
            scan_transition(SCAN_IDLE);
        }
        break;

    /* -------------------------------------------------------------- */
    case SCAN_MOTOR_START_POS:
        Motor_Ceph_StartPosition();

        if (g_bColumnPressed == TRUE) {
            Column_Control(COLUMN_STOP);
            g_bColumnPressed = FALSE;
        }

        scan_transition(SCAN_LASER_OFF);
        break;

    /* -------------------------------------------------------------- */
    case SCAN_LASER_OFF:
        LaserControl(TYPE_HEAD_CEPH, FALSE);

        if (!bIsExposureSwitchReleased(500)) {
            UART_SendMessage(DBG_MSG_PC, ERR_CODE_EXP_SWITCH);
            CurCaptureMode = CAPTURE_CANCEL;
        }

        if (CurCaptureMode != CAPTURE_SCAN) {
            scan_cancel_process();
            scan_transition(SCAN_IDLE);
            break;
        }

        Lamp_Control(LAMP_RED, LAMP_DISABLE);
        Lamp_Control(LAMP_GREEN, LAMP_ENABLE);
        Lamp_Control(LAMP_BLUE, LAMP_DISABLE);

        scan_transition(SCAN_COLLIMATOR_SETUP);
        break;

    /* -------------------------------------------------------------- */
    case SCAN_COLLIMATOR_SETUP:
        if (!CAN_Collimator_SendMessage(CMD_COLLI_OPEN, 0, 0, 1000, 2)) {
            UART_SendMessage(DBG_MSG_PC, "Collimator Open Fail.");
            scan_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
            scan_transition(SCAN_IDLE);
            break;
        }

        if (CephParam.bScanAlign == FALSE) {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CEPHSCAN_INIT, 0, 0, 1000, 2)) {
                UART_SendMessage(DBG_MSG_PC, "Collimator CEPHSCAN_INI Fail.");
                scan_cancel_process();
                CurCaptureMode = CAPTURE_CANCEL;
                scan_transition(SCAN_IDLE);
                break;
            }
        }

        if (!CAN_Collimator_SendMessage(CMD_FILTER_CENTER, 0, 0, 1000, 2)) {
            UART_SendMessage(DBG_MSG_PC, "Collimator filter move Fail.");
            scan_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
            scan_transition(SCAN_IDLE);
            break;
        }

        scan_transition(SCAN_MODE_SETUP);
        break;

    /* -------------------------------------------------------------- */
    case SCAN_MODE_SETUP:
    {
        double d_pulse = 0;
        double step_per_cm = 0;

        g_scan_distance = CephParam.Size;
        if (CephParam.Time == CEPH_SCAN_TIME_2SEC) {
            g_scan_distance = CEPH_2SEC_SCAN_STOP_DISTANCE;
        }

        Motor_C.RunParam.RunFreq = (g_scan_distance * MOTOR_C_MICROSTEP) /
                                    (CephParam.Time * msec_To_sec * MOTOR_C_PULLEY_DIAMETER * pi);

        d_pulse = MOTOR_C_PULLEY_DIAMETER * pi / MOTOR_C_MICROSTEP;
        g_scan_start_cnt = 10;
        g_scan_end_cnt = (uint32_t)(g_scan_distance / d_pulse);

        if (CephParam.Time == CEPH_SCAN_TIME_2SEC) {
            g_scan_two_sec_end_cnt = (uint32_t)(CEPH_2SEC_SCAN_STOP_DISTANCE / d_pulse);
            g_scan_end_cnt = (uint32_t)(CEPH_SCAN_TOTAL_DISTANCE / d_pulse);
        }

        step_per_cm = (double)g_scan_end_cnt / (double)30;

        switch (CephParam.Mode) {
        case Ceph_Full_Lateral:
            if (CephParam.Type == Ceph_Adult) {
                g_scan_end_cnt = g_scan_end_cnt - (uint32_t)(step_per_cm * 2);
            } else {
                g_scan_start_cnt = (uint32_t)(step_per_cm * 0.5);
                g_scan_end_cnt = g_scan_end_cnt - (uint32_t)(step_per_cm * 3.5);
                if (!CAN_Collimator_SendMessage(CMD_COL_CEPHSCAN_CHILD, 0, 0, 1000, 2)) {
                    UART_SendMessage(DBG_MSG_PC, "Collimator cephalo child move Fail.");
                    scan_cancel_process();
                    CurCaptureMode = CAPTURE_CANCEL;
                    scan_transition(SCAN_IDLE);
                    return;
                }
            }
            break;
        case Ceph_Lateral:
            if (CephParam.Type == Ceph_Adult) {
                g_scan_end_cnt = g_scan_end_cnt - (uint32_t)(step_per_cm * 9);
            } else {
                g_scan_start_cnt = (uint32_t)(step_per_cm * 0.5);
                g_scan_end_cnt = g_scan_end_cnt - (uint32_t)(step_per_cm * 10);
                if (!CAN_Collimator_SendMessage(CMD_COL_CEPHSCAN_CHILD, 0, 0, 1000, 2)) {
                    UART_SendMessage(DBG_MSG_PC, "Collimator cephalo child move Fail.");
                    scan_cancel_process();
                    CurCaptureMode = CAPTURE_CANCEL;
                    scan_transition(SCAN_IDLE);
                    return;
                }
            }
            break;
        case Ceph_Carpus:
        case Ceph_PA:
        case Ceph_AP:
        case Ceph_Waters:
        case Ceph_SVM:
            g_scan_start_cnt = (uint32_t)(step_per_cm * 3);
            g_scan_end_cnt = g_scan_end_cnt - (uint32_t)(step_per_cm * 3);
            if (!CAN_Collimator_SendMessage(CMD_COLLI_CEPH_AP_PA, 0, 0, 1000, 2)) {
                UART_SendMessage(DBG_MSG_PC, "Collimator cephalo AP PA move Fail.");
                scan_cancel_process();
                CurCaptureMode = CAPTURE_CANCEL;
                scan_transition(SCAN_IDLE);
                return;
            }
            break;
        case Ceph_EarLod:
            g_scan_start_cnt = (uint32_t)(step_per_cm * 10);
            g_scan_end_cnt = g_scan_end_cnt - (uint32_t)(step_per_cm * 10);
            break;
        default:
            break;
        }

        scan_transition(SCAN_TUBE_SETUP);
        break;
    }

    /* -------------------------------------------------------------- */
    case SCAN_TUBE_SETUP:
        gMachStat.bTubeCountFlag = RESET;
        if (!CAN_SendMessage(CAN_TUBE_COUNT_CHECK, 0, 0, 1000, 2)) {
            UART_SendMessage(DBG_MSG_PC, "Tube Count Fail.");
            scan_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
            scan_transition(SCAN_IDLE);
            break;
        }

        if (!CAN_SendMessage(CAN_TUBE_CONTI_MODE_SET, 0, 0, 1000, 2)) {
            UART_SendMessage(DBG_MSG_PC, "Tube Continuous mode Fail.");
            scan_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
            scan_transition(SCAN_IDLE);
            break;
        }

        if (!CAN_SendMessage(CAN_TUBE_KV_SET, CephParam.KV, 0, 1000, 2)) {
            UART_SendMessage(DBG_MSG_PC, "Tube KV Send Fail.");
            scan_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
            scan_transition(SCAN_IDLE);
            break;
        }

        if (!CAN_SendMessage(CAN_TUBE_mA_SET, CephParam.mA, 0, 1000, 2)) {
            UART_SendMessage(DBG_MSG_PC, "Tube mA Send Fail.");
            scan_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
            scan_transition(SCAN_IDLE);
            break;
        }

        if (CephParam.Time == CEPH_SCAN_TIME_2SEC) {
            scan_transition(SCAN_WAIT_2SEC_CONFIRM);
        } else {
            scan_transition(SCAN_EXPOSURE_SETUP);
        }
        break;

    /* -------------------------------------------------------------- */
    case SCAN_WAIT_2SEC_CONFIRM:
        if (UART_WaitProtocol("[SM_2SEC_CONFI]")) {
            scan_transition(SCAN_EXPOSURE_SETUP);
        } else if (CurCaptureMode == CAPTURE_CANCEL) {
            scan_cancel_process();
            scan_transition(SCAN_IDLE);
        }
        break;

    /* -------------------------------------------------------------- */
    case SCAN_EXPOSURE_SETUP:
        if (sysInfo.bShowLog == TRUE) {
            printUart(DBG_MSG_PC, "Cephalo ::Mode = %d, Type = %d, Time=%d",
                      CephParam.Mode, CephParam.Type, CephParam.Time);
            printUart(DBG_MSG_PC, "Cephalo ::kV = %d, mA = %d",
                      CephParam.KV, CephParam.mA);
            printUart(DBG_MSG_PC, "Cephalo Start Motor CC::CurrentStep = %d", Motor_C.CurrentStep);
            printUart(DBG_MSG_PC, "Cephalo Start Motor CS::CurrentStep = %d", Motor_S.CurrentStep);
            printUart(DBG_MSG_PC, "Cephalo Start Cnt = %d, End Cnt = %d",
                      g_scan_start_cnt, g_scan_end_cnt);
        }

        Exposure_CtrlLed(EXPOSURE_LED_GPIO_ENABLE);
        UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_CONFI]");
        UART_SendMessage(DBG_MSG_PC, "[SP_STBY_EXPSW]");
        Indicator_Control(INDICATOR_ENABLE);

        scan_transition(SCAN_WAIT_EXPOSURE_SW);
        break;

    /* -------------------------------------------------------------- */
    case SCAN_WAIT_EXPOSURE_SW:
        if (!Exposure_CheckSwitch() || UART_Exposure_SET() == 1) {
            g_scan_exp_check = (BitAction)GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7);
            scan_transition(SCAN_XRAY_INTERLOCK);
        } else if (CurCaptureMode != CAPTURE_SCAN) {
            scan_cancel_process();
            scan_transition(SCAN_IDLE);
        }
        break;

    /* -------------------------------------------------------------- */
    case SCAN_XRAY_INTERLOCK:
        if (!Safety_VerifyXrayInterlock()) {
            printUart(DBG_MSG_PC, "SAFETY: X-ray interlock fault, aborting scan");
            scan_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
            scan_transition(SCAN_IDLE);
            break;
        }
        scan_transition(SCAN_TUBE_READY);
        break;

    /* -------------------------------------------------------------- */
    case SCAN_TUBE_READY:
        Tube_CtrlReady(TUBE_READY_ENABLE);
        UART_SendMessage(DBG_MSG_PC, "Wait for Tube Heatting Time");
        IntTimer_Delay(2000);
        scan_transition(SCAN_TUBE_HEAT_WAIT);
        break;

    /* -------------------------------------------------------------- */
    case SCAN_TUBE_HEAT_WAIT:
        if (IntTimer_GetStatus()) {
            scan_transition(SCAN_START_CAPTURE);
        }
        break;

    /* -------------------------------------------------------------- */
    case SCAN_START_CAPTURE:
        UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_START]");
#ifdef USE_TABLET_PC
        UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_START]");
#endif
        gMachStat.bTubeCountCompareFlag = TRUE;

        /* Start collimator scan based on time mode */
        if (CephParam.bScanAlign == FALSE) {
            if (CephParam.Time == CEPH_SCAN_TIME_10SEC) {
                CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB,
                                            CMD_COL_CEPHSCAN_START_NORMAL, 0x96, 0);
            } else if (CephParam.Time == CEPH_SCAN_TIME_5SEC) {
                CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB,
                                            CMD_COL_CEPHSCAN_START_FAST, 0x96, 0);
            } else if (CephParam.Time == CEPH_SCAN_TIME_2SEC) {
                CAN_SendMessage_NoResponse(CAN_EXT_ID_SYSTEM_TO_SUBB,
                                            CMD_COL_CEPHSCAN_START_2SEC, 0x32, 0);
            } else {
                printUart(DBG_MSG_PC, "Scan Time Error!");
            }
        }

        Motor_CephScanStart(g_scan_end_cnt);
        g_scan_stop_flag = false;
        g_scan_exposure_started = false;
        g_scan_sensor_started = false;
        g_scan_exposure_ended = false;

        scan_transition(SCAN_EXPOSING);
        break;

    /* -------------------------------------------------------------- */
    case SCAN_EXPOSING:
    {
        bool is_2sec_mode = (CephParam.Time == 2000 && g_scan_distance == 200);

        /* Check for user stop */
        if (g_scan_exp_check == Bit_SET) {
            if (UART_Exposure_SET() == 2) {
                g_scan_stop_flag = true;
                scan_transition(SCAN_WAIT_END);
                break;
            }
        } else {
            if (Exposure_CheckSwitch()) {
                g_scan_stop_flag = true;
                scan_transition(SCAN_WAIT_END);
                break;
            }
        }

        /* Motor step-based exposure control */
        if (!g_scan_exposure_started && Motor_C.CurrentStep == g_scan_start_cnt) {
            Tube_CtrlExposure(TUBE_EXPOSURE_ENABLE);
#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
            typElapsedTimer.nExposeTime = vSetCurrentMilliSec();
#endif
            g_scan_exposure_started = true;
        }

        if (!g_scan_sensor_started && g_scan_exposure_started &&
            Motor_C.CurrentStep == g_scan_start_cnt + 5) {
            ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_ENABLE);
            g_scan_sensor_started = true;
        }

        /* 2-sec mode early end */
        if (is_2sec_mode && Motor_C.CurrentStep >= g_scan_two_sec_end_cnt &&
            !g_scan_exposure_ended) {
            ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE);
            Tube_CtrlReady(TUBE_READY_DISABLE);
            Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
            typElapsedTimer.nExposeTime = nElapsedMilliSec(typElapsedTimer.nExposeTime);
#endif
            g_scan_exposure_ended = true;
            scan_transition(SCAN_WAIT_END);
            break;
        }

        /* Normal end */
        if (!is_2sec_mode && Motor_C.CurrentStep >= g_scan_end_cnt &&
            !g_scan_exposure_ended) {
            ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE);
            Tube_CtrlReady(TUBE_READY_DISABLE);
            Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
            typElapsedTimer.nExposeTime = nElapsedMilliSec(typElapsedTimer.nExposeTime);
#endif
            g_scan_exposure_ended = true;
        }

        /* Motor reached past end position */
        if (Motor_C.CurrentStep >= g_scan_end_cnt + 100) {
            scan_transition(SCAN_WAIT_END);
        }
        break;
    }

    /* -------------------------------------------------------------- */
    case SCAN_WAIT_END:
        Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
        Indicator_Control(INDICATOR_DISABLE);

        /* Wait for C and S motors to stop (non-blocking) */
        if (!MotorWait_IsStopped(&Motor_C)) {
            break;
        }
        if (!MotorWait_IsStopped(&Motor_S)) {
            break;
        }

        if (!g_scan_stop_flag) {
            scan_transition(SCAN_COMPLETE);
        } else {
            scan_transition(SCAN_STOP);
        }
        break;

    /* -------------------------------------------------------------- */
    case SCAN_COMPLETE:
        UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_END__]");
#ifdef USE_TABLET_PC
        UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_END__]");
#endif

        Motor_CephScanEnd();

        if (sysInfo.bShowLog == TRUE) {
            printUart(DBG_MSG_PC, "Cephalo END Motor CC::CurrentStep = %d", Motor_C.CurrentStep);
            printUart(DBG_MSG_PC, "Cephalo END Motor CS::CurrentStep = %d", Motor_S.CurrentStep);
        }

        CurCaptureMode = CAPTURE_CANCEL;

#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
        printUart(DBG_MSG_PC, "Exposed Time : %d(ms)", typElapsedTimer.nExposeTime);
        ElapseTimerOnOff(FALSE);
#endif

        scan_clear_process();
        scan_transition(SCAN_IDLE);
        break;

    /* -------------------------------------------------------------- */
    case SCAN_STOP:
        Tube_CtrlReady(TUBE_READY_DISABLE);
        Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
        Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
        ScanSensor_CtrlOutput(SENSOR_S_OUTPUT_DISABLE);

        Motor_CephScanEnd();

        if (sysInfo.bShowLog == TRUE) {
            printUart(DBG_MSG_PC, "Cephalo Stop Motor CC::CurrentStep = %d", Motor_C.CurrentStep);
            printUart(DBG_MSG_PC, "Cephalo Stop Motor CS::CurrentStep = %d", Motor_S.CurrentStep);
        }

        UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_STOP_]");
#ifdef USE_TABLET_PC
        UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_STOP_]");
#endif

        CurCaptureMode = CAPTURE_CANCEL;

#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
        printUart(DBG_MSG_PC, "Exposed Time : %d(ms)", typElapsedTimer.nExposeTime);
        ElapseTimerOnOff(FALSE);
#endif

        UART_SendMessage(DBG_MSG_PC, ERR_CODE_SCAN_CAPT_STOP);

        if (sysInfo.initDir_MotCC == INIT_DIR_REVERSE) {
            CephParam.bInitPosFlag = FALSE;
        }

        scan_cancel_process();
        scan_transition(SCAN_IDLE);
        break;

    /* -------------------------------------------------------------- */
    case SCAN_CANCEL:
        scan_cancel_process();
        scan_transition(SCAN_IDLE);
        break;

    /* -------------------------------------------------------------- */
    case SCAN_ERROR:
        scan_cancel_process();
        scan_transition(SCAN_IDLE);
        break;

    default:
        scan_transition(SCAN_IDLE);
        break;
    }
}

/* ================================================================== */
/* Private helpers (from legacy scan_capture.c)                        */
/* ================================================================== */

static void scan_clear_param(void)
{
#ifdef USE_AGING_MODE
    CephParam.KV = 80;
    CephParam.mA = 80;
#else
    CephParam.KV = 0;
    CephParam.mA = 0;
#endif
    CephParam.Time = CEPH_SCAN_TIME_10SEC;
    CephParam.Size = CEPH_SCAN_TOTAL_DISTANCE;
    CephParam.Mode = Ceph_Lateral;
    CephParam.Type = Ceph_Adult;
    CephParam.bScanAlign = FALSE;
    CephParam.bScanEar = FALSE;
}

static void scan_cancel_process(void)
{
    UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_CANCE]");
#ifdef USE_TABLET_PC
    UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_CANCE]");
#endif
    scan_clear_process();
}

static void scan_clear_process(void)
{
    UART_SendMessage(DBG_MSG_PC, "Run ScanCapture_ClearProcess");

    scan_clear_param();

    if (sysInfo.bXrayNoSound == TRUE) {
        sysInfo.bXrayNoSound = FALSE;
        CAN_SendMessage(CAN_TUBE_SOUND_ON, 0, 0, 1000, 2);
    }

    if (gMachStat.bTubeCountCompareFlag == TRUE) {
        gMachStat.bTubeCountFlag = SET;
        CAN_SendMessage(CAN_TUBE_COUNT_CHECK, 0, 0, 1000, 2);
        if (gMachStat.nTubeOldCount == gMachStat.nTubeCount) {
            UART_SendMessage(DBG_MSG_PC, "Warning : Generator Doesn't Exposed");
        }
    }
    gMachStat.bTubeCountCompareFlag = FALSE;

#ifdef USE_AGING_MODE
    PrevCaptureMode = CAPTURE_SCAN;
    g_nCephCnt++;
    SysTime_MesureEnd(__FUNCTION__, __LINE__);
#endif

    LampTimer_Enable();
}
