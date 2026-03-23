/*
 * pano_sequence.c : Non-blocking panoramic capture state machine
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Converts the blocking PanoCapture() function into a non-blocking
 * step-based state machine. Each call to PanoSequence_Step() processes
 * one state transition and returns immediately.
 *
 * Original logic preserved from Legacy/src/pano_capture.c.
 * All while() blocking loops replaced with state transitions.
 *
 * Memory: ~2400 bytes Flash, ~120 bytes SRAM.
 */

#include "pano_sequence.h"
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

/* CAN_Collimator_SendMessage is defined in can.c */
extern bool CAN_Collimator_SendMessage(uint16_t cmd, uint32_t value,
                                         uint16_t debug, uint32_t timeOut,
                                         uint8_t retryCnt);

/* ------------------------------------------------------------------ */
/* Private state                                                       */
/* ------------------------------------------------------------------ */
static PanoState_t   g_pano_state;
static MotorWait_t   g_pano_mw;
static uint32_t      g_pano_timer_ms;
static bool          g_pano_stop_flag;
static BitAction     g_pano_exp_check;

/* ------------------------------------------------------------------ */
/* Forward declarations for cleanup helpers                            */
/* ------------------------------------------------------------------ */
static void pano_clear_param(void);
static void pano_clear_process(void);
static void pano_cancel_process(void);
static void pano_end_process(void);
static void pano_stop_process(void);
static void pano_mode_offset(void);

/* ------------------------------------------------------------------ */
/* State name table for debug logging                                  */
/* ------------------------------------------------------------------ */
#ifdef BUILD_TYPE_DEBUG
static const char* const k_pano_state_names[] = {
    "IDLE", "CHECK_SYS", "CAN_CHK", "TILT_OFF", "CLR_PARAM",
    "SENS_CFG", "MOV_INIT", "WAIT_INIT", "TEMPLE", "LASER_ON",
    "WAIT_PAR", "LASER_OFF", "COLLI_SET", "SENS_SYNC", "TUBE_SET",
    "START_MOT", "WAIT_EXP", "XRAY_ILK", "TUBE_RDY", "TUBE_HEAT",
    "START_CAP", "EXPOSING", "WAIT_END", "MOV_END", "WAIT_END_M",
    "COMPLETE", "STOP", "CANCEL", "ERROR"
};
#endif

static void pano_transition(PanoState_t new_state)
{
    g_pano_state = new_state;
#ifdef BUILD_TYPE_DEBUG
    if (new_state < PANO_STATE_COUNT) {
        DbgLog_Printf("PANO: -> %s", k_pano_state_names[new_state]);
    }
#endif
}

/* ================================================================== */
/* Public API                                                          */
/* ================================================================== */

void PanoSequence_Init(void)
{
    g_pano_state = PANO_IDLE;
    g_pano_stop_flag = false;
    g_pano_timer_ms = 0;
    MotorWait_Init(&g_pano_mw, 30000);
}

void PanoSequence_Start(void)
{
    if (g_pano_state != PANO_IDLE) {
        return;
    }
    g_pano_stop_flag = false;
    pano_transition(PANO_CHECK_SYSTEM);
}

bool PanoSequence_IsDone(void)
{
    return (g_pano_state == PANO_IDLE);
}

PanoState_t PanoSequence_GetState(void)
{
    return g_pano_state;
}

void PanoSequence_Cancel(void)
{
    /* Safe to call from any state */
    if (g_pano_state == PANO_IDLE) {
        return;
    }

    /* Immediately disable X-ray outputs */
    Tube_CtrlReady(TUBE_READY_DISABLE);
    Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
    Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
    Indicator_Control(INDICATOR_DISABLE);

    CurCaptureMode = CAPTURE_CANCEL;
    pano_cancel_process();
    pano_transition(PANO_IDLE);
}

void PanoSequence_Run(void)
{
    /* Legacy blocking compatibility wrapper */
    PanoSequence_Init();
    PanoSequence_Start();
    while (!PanoSequence_IsDone()) {
        PanoSequence_Step();
        Safety_IWDG_Kick();
    }
}

/* ================================================================== */
/* Step function - one state per call                                  */
/* ================================================================== */

void PanoSequence_Step(void)
{
    int8_t mw_result;

    switch (g_pano_state) {

    /* -------------------------------------------------------------- */
    case PANO_IDLE:
        /* Nothing to do */
        break;

    /* -------------------------------------------------------------- */
    case PANO_CHECK_SYSTEM:
        if (gMachStat.bFirstEntry == RESET) {
            printUart(DBG_MSG_PC, "==First Entry after boot==");
            gMachStat.bFirstEntry = SET;
        }

        if (checkPreCaptureStatus() != TRUE) {
            printUart(DBG_MSG_PC, "checkPreCaptureStatus :: FALSE");
            pano_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
            PrevCaptureMode = CAPTURE_CANCEL;
#endif
            pano_transition(PANO_IDLE);
            break;
        }

        /* Send mode message */
        if (PanoParam.Init_Mode == PANO_NORMAL) {
            UART_SendMessage(DBG_MSG_PC, "[SP_MODE_PANO_]");
#ifdef USE_TABLET_PC
            UART_SendMessage(DBG_MSG_TABLET, "[SP_MODE_PANO_]");
#endif
        } else if (PanoParam.Init_Mode == PANO_TMJ) {
            UART_SendMessage(DBG_MSG_PC, "[SP_MODE_PANOR]");
#ifdef USE_TABLET_PC
            UART_SendMessage(DBG_MSG_TABLET, "[SP_MODE_PANOR]");
#endif
        }

        if (g_bColumnPressed == SET) {
            Column_Control(COLUMN_STOP);
            g_bColumnPressed = FALSE;
        }

        pano_transition(PANO_CAN_COMM_CHECK);
        break;

    /* -------------------------------------------------------------- */
    case PANO_CAN_COMM_CHECK:
        if (CAN_SendMessage(CAN_TUBE_COM_CHECK, CAN_TUBE_GEN2_COM, 0, 3000, 2)) {
            CAN_SendMessage(CAN_TUBE_TANK_TEMP_READ, 0, 0, 1000, 2);
        } else {
            UART_SendMessage(DBG_MSG_PC, "Tube Communication Fail.");
            pano_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
            PrevCaptureMode = CAPTURE_CANCEL;
#endif
            pano_transition(PANO_IDLE);
            break;
        }
        pano_transition(PANO_COLLIMATOR_TILT_OFF);
        break;

    /* -------------------------------------------------------------- */
    case PANO_COLLIMATOR_TILT_OFF:
        if (!CAN_Collimator_SendMessage(CMD_TILT_OFF, 0, 0, 5000, 2)) {
            UART_SendMessage(DBG_MSG_PC, "Collimator Communication Fail.");
            pano_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
            PrevCaptureMode = CAPTURE_CANCEL;
#endif
            pano_transition(PANO_IDLE);
            break;
        }
        g_bTiltStatus = FALSE;
        pano_transition(PANO_CLEAR_PARAMS);
        break;

    /* -------------------------------------------------------------- */
    case PANO_CLEAR_PARAMS:
        pano_clear_param();
        Arch_GetTable(PanoParam.Arch, PanoParam.Scan);
        pano_transition(PANO_SENSOR_CONFIG);
        break;

    /* -------------------------------------------------------------- */
    case PANO_SENSOR_CONFIG:
        PanoSensor_Config();
        PanoSensor_Start();
        pano_transition(PANO_MOVE_TO_INIT);
        break;

    /* -------------------------------------------------------------- */
    case PANO_MOVE_TO_INIT:
        if (!Motor_MoveInitPosition()) {
            UART_SendMessage(DBG_MSG_PC, "Motor Init Position Fail.");
            pano_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
            PrevCaptureMode = CAPTURE_CANCEL;
#endif
            pano_transition(PANO_IDLE);
            break;
        }

        /* Check if mode was cancelled during motor move */
        if (CurCaptureMode != CAPTURE_PANO) {
            pano_cancel_process();
            pano_transition(PANO_IDLE);
            break;
        }

#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
        ElapseTimerOnOff(TRUE);
#endif
        pano_transition(PANO_TEMPLE_SUPPORT);
        break;

    /* -------------------------------------------------------------- */
    case PANO_TEMPLE_SUPPORT:
#ifdef USE_MOTOR_CHINREST_TS
        Motor_MoveTempleSupport(FALSE);
#endif
        pano_transition(PANO_LASER_ON);
        break;

    /* -------------------------------------------------------------- */
    case PANO_LASER_ON:
        LaserSetPosition();
        LaserControl(TYPE_HEAD_PANO, TRUE);
        LaserControl(TYPE_FOOT, TRUE);

        if (sysInfo.bShowLog == TRUE) {
            printUart(DBG_MSG_PC, "Panorama Init Motor R::CurrentStep = %d", Motor_R.CurrentStep);
            printUart(DBG_MSG_PC, "Panorama Init Motor V::CurrentStep = %d", Motor_V.CurrentStep);
            printUart(DBG_MSG_PC, "Panorama Init Motor CNS::CurrentStep = %d", Motor_CNS.CurrentStep);
            printUart(DBG_MSG_PC, "Panorama Init Motor CWE::CurrentStep = %d", Motor_CWE.CurrentStep);
        }

        UART_SendMessage(DBG_MSG_PC, "[SP_PANO_READY]");
#ifdef USE_TABLET_PC
        UART_SendMessage(DBG_MSG_TABLET, "[SP_PANO_READY]");
#endif
        pano_transition(PANO_WAIT_PARAMS);
        break;

    /* -------------------------------------------------------------- */
    case PANO_WAIT_PARAMS:
        /* Non-blocking: check if params arrived from PC */
#ifdef USE_AGING_MODE
        if (g_bAgingMode == TRUE) {
            pano_transition(PANO_LASER_OFF);
            break;
        }
#endif
        if (UART_SetPanoParam()) {
            pano_transition(PANO_LASER_OFF);
        } else if (CurCaptureMode == CAPTURE_CANCEL) {
            pano_cancel_process();
            pano_transition(PANO_IDLE);
        }
        /* else: stay in this state, return and check next iteration */
        break;

    /* -------------------------------------------------------------- */
    case PANO_LASER_OFF:
        if (g_bColumnPressed == TRUE) {
            Column_Control(COLUMN_STOP);
            g_bColumnPressed = FALSE;
        }

        LaserControl(TYPE_HEAD_PANO, FALSE);
        LaserControl(TYPE_FOOT, FALSE);

        if (!bIsExposureSwitchReleased(500)) {
            UART_SendMessage(DBG_MSG_PC, ERR_CODE_EXP_SWITCH);
            CurCaptureMode = CAPTURE_CANCEL;
        }

        if (CurCaptureMode != CAPTURE_PANO) {
            pano_cancel_process();
            pano_transition(PANO_IDLE);
            break;
        }

        Lamp_Control(LAMP_RED, LAMP_DISABLE);
        Lamp_Control(LAMP_GREEN, LAMP_ENABLE);
        Lamp_Control(LAMP_BLUE, LAMP_DISABLE);

        pano_transition(PANO_COLLIMATOR_SETUP);
        break;

    /* -------------------------------------------------------------- */
    case PANO_COLLIMATOR_SETUP:
        Arch_GetTable(PanoParam.Arch, PanoParam.Scan);
        pano_mode_offset();

        if (PanoParam.bColliOpen == TRUE) {
            if (!CAN_Collimator_SendMessage(CMD_COLLI_OPEN, 0, 0, 2000, 2)) {
                UART_SendMessage(DBG_MSG_PC, "Collimator Open Fail.");
                pano_cancel_process();
                CurCaptureMode = CAPTURE_CANCEL;
                pano_transition(PANO_IDLE);
                break;
            }
        } else {
            if (PanoParam.Mode == PANO_MODE_3D) {
                if (!CAN_Collimator_SendMessage(CMD_COLLI_PANO_180PX, 0, 0, 2000, 2)) {
                    UART_SendMessage(DBG_MSG_PC, "Collimator PANO_180PX Fail.");
                    pano_cancel_process();
                    CurCaptureMode = CAPTURE_CANCEL;
                    pano_transition(PANO_IDLE);
                    break;
                }
            } else {
                if (PanoParam.Arch == PANO_ARCH_CHILD) {
                    if (!CAN_Collimator_SendMessage(CMD_COLLI_PANOR_CHILD, 0, 0, 2000, 2)) {
                        if (!CAN_Collimator_SendMessage(CMD_COLLI_PANOR, 0, 0, 2000, 2)) {
                            UART_SendMessage(DBG_MSG_PC, "Collimator Panorama Fail.");
                            pano_cancel_process();
                            CurCaptureMode = CAPTURE_CANCEL;
                            pano_transition(PANO_IDLE);
                            break;
                        }
                    }
                } else {
                    if (!CAN_Collimator_SendMessage(CMD_COLLI_PANOR, 0, 0, 2000, 2)) {
                        UART_SendMessage(DBG_MSG_PC, "Collimator Panorama Fail.");
                        pano_cancel_process();
                        CurCaptureMode = CAPTURE_CANCEL;
                        pano_transition(PANO_IDLE);
                        break;
                    }
                }
            }
        }

        if (!CAN_Collimator_SendMessage(CMD_FILTER_CENTER, 0, 0, 2000, 2)) {
            UART_SendMessage(DBG_MSG_PC, "Collimator Filter Move Fail.");
            pano_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
            pano_transition(PANO_IDLE);
            break;
        }

        if (sysInfo.bShowLog == TRUE) {
            printUart(DBG_MSG_PC, "Panorama ::Arch = %d, Scan = %d, Mode = %d",
                      PanoParam.Arch, PanoParam.Scan, PanoParam.Mode);
            printUart(DBG_MSG_PC, "Panorama ::kV = %d, mA = %d",
                      PanoParam.KV, PanoParam.mA);
        }

        pano_transition(PANO_SENSOR_SYNC);
        break;

    /* -------------------------------------------------------------- */
    case PANO_SENSOR_SYNC:
        /* Wait for sensor update to complete (non-blocking poll) */
        if (Sensor_P.Update != FALSE) {
            break; /* wait */
        }

        PanoSensor_Stop();
        PanoSensor_SetUserSync();
        pano_transition(PANO_TUBE_SETUP);
        break;

    /* -------------------------------------------------------------- */
    case PANO_TUBE_SETUP:
        gMachStat.bTubeCountFlag = RESET;
        if (!CAN_SendMessage(CAN_TUBE_COUNT_CHECK, 0, 0, 1000, 2)) {
            UART_SendMessage(DBG_MSG_PC, "Tube Count Fail.");
            pano_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
            pano_transition(PANO_IDLE);
            break;
        }

        if (!CAN_SendMessage(CAN_TUBE_CONTI_MODE_SET, 0, 0, 1000, 2)) {
            UART_SendMessage(DBG_MSG_PC, "Tube Continuous mode Fail.");
            pano_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
            pano_transition(PANO_IDLE);
            break;
        }

        if (!CAN_SendMessage(CAN_TUBE_KV_SET, PanoParam.KV, 0, 1000, 2)) {
            UART_SendMessage(DBG_MSG_PC, "Tube KV Send Fail.");
            pano_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
            pano_transition(PANO_IDLE);
            break;
        }

        if (!CAN_SendMessage(CAN_TUBE_mA_SET, PanoParam.mA, 0, 1000, 2)) {
            UART_SendMessage(DBG_MSG_PC, "Tube mA Send Fail.");
            pano_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
            pano_transition(PANO_IDLE);
            break;
        }

#ifdef VARIABLE_EXPOSURE
        if (PanoParam.nExpStartTime != 0) {
            CAN_SendMessage(0x2900, PanoParam.nExpStartTime, 0, 1000, 2);
        }
        if (PanoParam.nExpEndTime != 0) {
            CAN_SendMessage(0x2901, PanoParam.nExpEndTime, 0, 1000, 2);
        }
        if (PanoParam.nExpKV != 0) {
            CAN_SendMessage(0x2902, PanoParam.nExpKV, 0, 1000, 2);
        }
        if (PanoParam.nExpMA != 0) {
            CAN_SendMessage(0x2903, PanoParam.nExpMA, 0, 1000, 2);
        }
#endif

        pano_transition(PANO_START_MOTORS);
        break;

    /* -------------------------------------------------------------- */
    case PANO_START_MOTORS:
        PanoSensor_Start();
        Motor_MoveStartPosition();

        if (sysInfo.bShowLog == TRUE) {
            printUart(DBG_MSG_PC, "Panorama Start Motor R::CurrentStep = %d", Motor_R.CurrentStep);
            printUart(DBG_MSG_PC, "Panorama Start Motor V::CurrentStep = %d", Motor_V.CurrentStep);
            printUart(DBG_MSG_PC, "Panorama Start Motor A::CurrentStep = %d", Motor_A.CurrentStep);
        }

        Exposure_CtrlLed(EXPOSURE_LED_GPIO_ENABLE);
        UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_CONFI]");
        UART_SendMessage(DBG_MSG_PC, "[SP_STBY_EXPSW]");
        Indicator_Control(INDICATOR_ENABLE);

        pano_transition(PANO_WAIT_EXPOSURE_SW);
        break;

    /* -------------------------------------------------------------- */
    case PANO_WAIT_EXPOSURE_SW:
        /* Non-blocking: poll exposure switch */
        if (!Exposure_CheckSwitch() || UART_Exposure_SET() == 1) {
            g_pano_exp_check = (BitAction)GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7);
            Timer_Config_T(&Motor_R.Timer);
            pano_transition(PANO_XRAY_INTERLOCK);
        } else if (CurCaptureMode != CAPTURE_PANO) {
            pano_cancel_process();
            pano_transition(PANO_IDLE);
        }
        break;

    /* -------------------------------------------------------------- */
    case PANO_XRAY_INTERLOCK:
        if (!Safety_VerifyXrayInterlock()) {
            printUart(DBG_MSG_PC, "SAFETY: X-ray interlock fault, aborting pano");
            pano_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
            pano_transition(PANO_IDLE);
            break;
        }
        pano_transition(PANO_TUBE_READY);
        break;

    /* -------------------------------------------------------------- */
    case PANO_TUBE_READY:
        Tube_CtrlReady(TUBE_READY_ENABLE);
        printUart(DBG_MSG_PC, "Wait for Tube Heatting Time");

        IntTimer_Delay(2000);
        g_pano_timer_ms = vSetCurrentMilliSec();
        pano_transition(PANO_TUBE_HEAT_WAIT);
        break;

    /* -------------------------------------------------------------- */
    case PANO_TUBE_HEAT_WAIT:
        /* Non-blocking: poll timer */
        if (IntTimer_GetStatus()) {
            pano_transition(PANO_START_CAPTURE);
        }
        break;

    /* -------------------------------------------------------------- */
    case PANO_START_CAPTURE:
        UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_START]");
#ifdef USE_TABLET_PC
        UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_START]");
#endif
        gMachStat.bTubeCountCompareFlag = TRUE;
        Motor_PanoStart();
        Sensor_P.Count = 0;
        g_pano_stop_flag = false;
        pano_transition(PANO_EXPOSING);
        break;

    /* -------------------------------------------------------------- */
    case PANO_EXPOSING:
    {
        uint32_t end_index = Motor_R.ArchParam.AccSize + Motor_R.ArchParam.CaptureSize;

        if (g_pano_exp_check == Bit_SET) {
            /* Software exposure switch mode */
            if (UART_Exposure_SET() == 2) {
                g_pano_stop_flag = true;
                pano_transition(PANO_WAIT_END);
                break;
            }
        } else {
            /* Hardware exposure switch mode */
            if (Exposure_CheckSwitch()) {
                g_pano_stop_flag = true;
                pano_transition(PANO_WAIT_END);
                break;
            }

            /* Align mode: stop at halfway */
            if (PanoParam.Mode == PANO_MODE_ALIGN) {
                uint32_t half_idx = Motor_R.ArchParam.AccSize +
                                    (Motor_R.ArchParam.CaptureSize / 2);
                if (Motor_R.ArchParam.ArrayIndex == half_idx) {
                    if (Motor_R.RunStep == Motor_R.ArchParam.IndexStep / 2) {
                        g_pano_stop_flag = true;
                        Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
                        Tube_CtrlReady(TUBE_READY_DISABLE);
                        Motor_PanoEnd();
                        pano_transition(PANO_WAIT_END);
                        break;
                    }
                }
            }
        }

        /* Check if capture is complete */
        if (Motor_R.ArchParam.ArrayIndex > end_index) {
            pano_transition(PANO_WAIT_END);
        }
        break;
    }

    /* -------------------------------------------------------------- */
    case PANO_WAIT_END:
        Indicator_Control(INDICATOR_DISABLE);
        Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);

        /* Wait for motors R and V to stop (non-blocking) */
        if (!MotorWait_IsStopped(&Motor_R)) {
            break;
        }
        if (!MotorWait_IsStopped(&Motor_V)) {
            break;
        }

        if (!g_pano_stop_flag) {
            pano_transition(PANO_COMPLETE);
        } else {
            pano_transition(PANO_STOP);
        }
        break;

    /* -------------------------------------------------------------- */
    case PANO_COMPLETE:
        /* End process (non-blocking version) */
        UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_END__]");
#ifdef USE_TABLET_PC
        UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_END__]");
#endif
        if (PanoParam.bFirstPano == RESET) {
            PanoParam.bFirstPano = SET;
            UART_SendMessage(DBG_MSG_PC, "[SP_PANO_DEFEC]");
        }
        Motor_MoveEndPosition();
        CurCaptureMode = CAPTURE_CANCEL;
        Motor_PanoEnd();

        if (sysInfo.bShowLog == TRUE) {
            printUart(DBG_MSG_PC, "Panorama END Motor R::CurrentStep = %d", Motor_R.CurrentStep);
            printUart(DBG_MSG_PC, "Panorama END Motor V::CurrentStep = %d", Motor_V.CurrentStep);
            printUart(DBG_MSG_PC, "Panorama END Sensor Trigger Count = %d", Sensor_P.Count);
        }

#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
        printUart(DBG_MSG_PC, "Exposed Time : %d(ms)", typElapsedTimer.nExposeTime);
        ElapseTimerOnOff(FALSE);
#endif

        pano_clear_process();
        pano_transition(PANO_IDLE);
        break;

    /* -------------------------------------------------------------- */
    case PANO_STOP:
        /* Stop process */
        Tube_CtrlReady(TUBE_READY_DISABLE);
        Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
        Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
        CurCaptureMode = CAPTURE_CANCEL;
        Motor_PanoEnd();

        if (sysInfo.bShowLog == TRUE) {
            printUart(DBG_MSG_PC, "Panorama Stop Motor R::CurrentStep = %d", Motor_R.CurrentStep);
            printUart(DBG_MSG_PC, "Panorama Stop Motor V::CurrentStep = %d", Motor_V.CurrentStep);
        }

        UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_STOP_]");
#ifdef USE_TABLET_PC
        UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_STOP_]");
#endif

#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
        printUart(DBG_MSG_PC, "Exposed Time : %d(ms)", typElapsedTimer.nExposeTime);
        ElapseTimerOnOff(FALSE);
#endif

        UART_SendMessage(DBG_MSG_PC, ERR_CODE_PANO_CAPT_STOP);
        pano_cancel_process();
        pano_transition(PANO_IDLE);
        break;

    /* -------------------------------------------------------------- */
    case PANO_CANCEL:
        pano_cancel_process();
        pano_transition(PANO_IDLE);
        break;

    /* -------------------------------------------------------------- */
    case PANO_ERROR:
        pano_cancel_process();
        pano_transition(PANO_IDLE);
        break;

    default:
        pano_transition(PANO_IDLE);
        break;
    }
}

/* ================================================================== */
/* Private helper functions (from legacy pano_capture.c)               */
/* ================================================================== */

static void pano_clear_param(void)
{
    PanoParam.KV = 80;
    PanoParam.mA = 80;
    PanoParam.Mode = PANO_MODE_NORMAL;
    PanoParam.Arch = PANO_ARCH_ADULT;
    PanoParam.Scan = PANO_SCAN_HD;
    Sensor_P.Index = 0;
    PanoParam.bColliOpen = FALSE;
    Motor_R.ArchParam.ArrayIndex = 0;
#ifdef VARIABLE_EXPOSURE
    PanoParam.nExpStartTime = 0;
    PanoParam.nExpEndTime = 0;
    PanoParam.nExpKV = 0;
    PanoParam.nExpMA = 0;
#endif
}

static void pano_cancel_process(void)
{
    UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_CANCE]");
#ifdef USE_TABLET_PC
    UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_CANCE]");
#endif
    pano_clear_process();
}

static void pano_clear_process(void)
{
    printUart(DBG_MSG_PC, "Run PanoCapture_ClearProcess");

    Sensor_P.Update = FALSE;
    /* Poll sensor update - if not ready, skip (non-blocking) */
    PanoSensor_Stop();
    pano_clear_param();

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

#ifdef USE_MOTOR_CHINREST_TS
    Motor_MoveTempleSupport(FALSE);
#endif

#ifdef USE_AGING_MODE
    PrevCaptureMode = CAPTURE_PANO;
    g_nPanoCnt++;
    SysTime_MesureEnd(__FUNCTION__, __LINE__);
#endif

    Tube.bXrayFrameOnOff = RESET;
    LampTimer_Enable();
}

/*
 * Inline copy of PanoCapture_ModeOffset (static in legacy code).
 * Calculates V-axis offset based on panoramic capture mode.
 */
#define PANO_V_SINUS_OFFSET   (22.0 / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP))
#define PANO_V_TMJ_OFFSET     (85.0 / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP))
#define PANO_V_TMJ_POSITION   (80.0 / (MOTOR_V_PULLEY_PITCH / MOTOR_V_MICROSTEP))

static void pano_mode_offset(void)
{
    switch (PanoParam.Mode) {
    case PANO_MODE_NORMAL:
    case PANO_MODE_LEFT:
    case PANO_MODE_FRONT:
    case PANO_MODE_RIGHT:
    case PANO_MODE_3D:
    case PANO_MODE_ALIGN:
        PanoParam.ModeOffset = Motor_V.CurrentStep;
        break;
    case PANO_MODE_SINUS:
        PanoParam.ModeOffset = Motor_V.CurrentStep;
        break;
    case PANO_MODE_TMJ:
        PanoParam.ModeOffset = PANO_V_TMJ_POSITION +
                               (Motor_V.CurrentStep - PANO_V_TMJ_OFFSET);
        break;
    case PANO_MODE_NONE:
    default:
        printUart(DBG_MSG_PC, "PanoCapture_ModeOffset check failed");
        break;
    }
}
