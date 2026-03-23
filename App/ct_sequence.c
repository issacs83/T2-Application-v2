/*
 * ct_sequence.c : Non-blocking CT/CBCT capture state machine
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Converts the blocking CtCapture() function into a non-blocking
 * step-based state machine. Handles both standard CT and stitch mode
 * (15x15 FOV with two rotations at different heights).
 *
 * Original logic preserved from Legacy/src/ct_capture.c.
 *
 * Memory: ~3200 bytes Flash, ~160 bytes SRAM.
 */

#include "ct_sequence.h"
#include "motor_sequence.h"

/* Legacy headers */
#include "extern.h"
#include "system.h"
#include "error_code.h"
#include "motor.h"
#include "can.h"
#include "tube.h"
#include "serial.h"
#include "misc1.h"
#include "timer.h"
#include "safety.h"
#include "motor_compat.h"
#include "debug_log.h"

#if defined(USE_CT_XRAY_PULSED_MODE)
#include "sensor.h"
#endif

extern bool CAN_Collimator_SendMessage(uint16_t cmd, uint32_t value,
                                         uint16_t debug, uint32_t timeOut,
                                         uint8_t retryCnt);

/* ------------------------------------------------------------------ */
/* Constants from legacy ct_capture.c                                  */
/* ------------------------------------------------------------------ */
#define CT_MOTORV_START_OFFSET_DEFAULT      10.0
#define EXPOSURE_START_POSITION             30.0
#define EXPOSURE_FAST_START_POSITION        30.0
#define EXPOSURE_STITCH_START_POSITION      25.0

#define MOTOR_R_ACCEL_ANGLE                 25.0
#define MOTOR_R_DECEL_ANGLE                 20.0
#define MOTOR_R_FAST_ACCEL_ANGLE            30.0
#define MOTOR_R_FAST_DECEL_ANGLE            38.0

#define END_STEP_MARGIN                     1000
#define CT_EXPOSURE_TOTAL_ANGLE             365.0

#define CT_ROTATE_XON_ANGLE_DEFAULT         30.0
#define CT_ROTATE_XOFF_ANGLE_DEFAULT        395.0
#define CT_ROTATE_TOTAL_ANGLE_DEFAULT       415.0

#define RECON_22SEC_FRAME_NUMBER            600
#define RECON_14SEC_FRAME_NUMBER            400
#define RECON_07SEC_FRAME_NUMBER            400

/* ------------------------------------------------------------------ */
/* Private state                                                       */
/* ------------------------------------------------------------------ */
static CtState_t     g_ct_state;
static MotorWait_t   g_ct_mw;
static bool          g_ct_stop_flag;
static BitAction     g_ct_exp_check;

/* Rotation parameters (computed once per sequence) */
static double        g_ct_fps;
static double        g_ct_time;
static uint32_t      g_ct_rotate_total_step;
static uint32_t      g_ct_acc_step;
static uint32_t      g_ct_dec_step;
static uint16_t      g_ct_collimator_cmd;
static char          g_ct_capture_count;
static double        g_ct_motor_r_degree_step;
static double        g_ct_motor_r_microstep;

/* ------------------------------------------------------------------ */
/* Forward declarations                                                */
/* ------------------------------------------------------------------ */
static void ct_clear_param(void);
static void ct_clear_process(void);
static void ct_cancel_process(void);

/* ------------------------------------------------------------------ */
/* State transition helper                                             */
/* ------------------------------------------------------------------ */
#ifdef BUILD_TYPE_DEBUG
static const char* const k_ct_state_names[] = {
    "IDLE", "CHECK_SYS", "SENS_CFG", "TILT_OFF", "CAN_CHK",
    "MOV_INIT", "CLR_PARAM", "LASER_ON", "WAIT_PAR", "LASER_OFF",
    "FOV_SET", "FILTER", "PULSE_CFG", "CALC_PAR", "TUBE_SET",
    "MOT_START", "STITCH_SET", "WAIT_STCH", "EXP_SET", "WAIT_EXP",
    "XRAY_ILK", "TUBE_RDY", "TUBE_HEAT", "START_ROT", "EXPOSING",
    "WAIT_ROT", "STCH_CHK", "MOV_END", "WAIT_END", "STCH_BACK",
    "COMPLETE", "STOP", "CANCEL", "ERROR"
};
#endif

static void ct_transition(CtState_t new_state)
{
    g_ct_state = new_state;
#ifdef BUILD_TYPE_DEBUG
    if (new_state < CT_STATE_COUNT) {
        DbgLog_Printf("CT: -> %s", k_ct_state_names[new_state]);
    }
#endif
}

/* ================================================================== */
/* Public API                                                          */
/* ================================================================== */

void CtSequence_Init(void)
{
    g_ct_state = CT_IDLE;
    g_ct_stop_flag = false;
    g_ct_capture_count = 0;
    MotorWait_Init(&g_ct_mw, 30000);
}

void CtSequence_Start(void)
{
    if (g_ct_state != CT_IDLE) {
        return;
    }
    g_ct_stop_flag = false;
    g_ct_capture_count = 0;
    ct_transition(CT_CHECK_SYSTEM);
}

bool CtSequence_IsDone(void)
{
    return (g_ct_state == CT_IDLE);
}

CtState_t CtSequence_GetState(void)
{
    return g_ct_state;
}

void CtSequence_Cancel(void)
{
    if (g_ct_state == CT_IDLE) {
        return;
    }

    Tube_CtrlReady(TUBE_READY_DISABLE);
    Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
    Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
    Indicator_Control(INDICATOR_DISABLE);

    CurCaptureMode = CAPTURE_CANCEL;
    Motor_CTEnd();
    ct_cancel_process();
    ct_transition(CT_IDLE);
}

void CtSequence_Run(void)
{
    /* Legacy blocking compatibility wrapper */
    CtSequence_Init();
    CtSequence_Start();
    while (!CtSequence_IsDone()) {
        CtSequence_Step();
        Safety_IWDG_Kick();
    }
}

/* ================================================================== */
/* Step function                                                       */
/* ================================================================== */

void CtSequence_Step(void)
{
    switch (g_ct_state) {

    /* -------------------------------------------------------------- */
    case CT_IDLE:
        break;

    /* -------------------------------------------------------------- */
    case CT_CHECK_SYSTEM:
        if (gMachStat.bFirstEntry == RESET) {
            printUart(DBG_MSG_PC, "==First Entry after boot==");
            gMachStat.bFirstEntry = SET;
        }

        if (checkPreCaptureStatus() != TRUE) {
            printUart(DBG_MSG_PC, "checkPreCaptureStatus :: FALSE");
            ct_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
            PrevCaptureMode = CAPTURE_CANCEL;
#endif
            ct_transition(CT_IDLE);
            break;
        }

        /* Send mode message */
        if (CtParam.bReverse_Status == RESET) {
            UART_SendMessage(DBG_MSG_PC, "[SP_MODE_CT___]");
        } else {
            UART_SendMessage(DBG_MSG_PC, "[SP_MODE_CT_R_]");
        }
#ifdef USE_TABLET_PC
        if (CtParam.bReverse_Status == RESET) {
            UART_SendMessage(DBG_MSG_TABLET, "[SP_MODE_CT___]");
        } else {
            UART_SendMessage(DBG_MSG_TABLET, "[SP_MODE_CT_R_]");
        }
#endif

        if (g_bColumnPressed == SET) {
            Column_Control(COLUMN_STOP);
            g_bColumnPressed = FALSE;
        }

        ct_transition(CT_SENSOR_CONFIG);
        break;

    /* -------------------------------------------------------------- */
    case CT_SENSOR_CONFIG:
#if defined(USE_CT_XRAY_PULSED_MODE) && defined(USE_SENSOR_USER_SYNC)
        if (CtParam.TubeMode == CT_TUBE_PULSED) {
            CtSensor_Config();
            CtSensor_Start();
        }
#endif
        ct_transition(CT_COLLIMATOR_TILT_OFF);
        break;

    /* -------------------------------------------------------------- */
    case CT_COLLIMATOR_TILT_OFF:
        if (!CAN_Collimator_SendMessage(CMD_TILT_OFF, 0, 0, 5000, 2)) {
            UART_SendMessage(DBG_MSG_PC, "Collimator Communication Fail.");
            ct_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
            PrevCaptureMode = CAPTURE_CANCEL;
#endif
            ct_transition(CT_IDLE);
            break;
        }
        g_bTiltStatus = FALSE;
        ct_transition(CT_CAN_COMM_CHECK);
        break;

    /* -------------------------------------------------------------- */
    case CT_CAN_COMM_CHECK:
        if (CAN_SendMessage(CAN_TUBE_COM_CHECK, CAN_TUBE_GEN2_COM, 0, 3000, 2)) {
            CAN_SendMessage(CAN_TUBE_TANK_TEMP_READ, 0, 0, 1000, 2);
        } else {
            UART_SendMessage(DBG_MSG_PC, "Tube Communication Fail.");
            ct_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
            PrevCaptureMode = CAPTURE_CANCEL;
#endif
            ct_transition(CT_IDLE);
            break;
        }
        ct_transition(CT_MOVE_TO_INIT);
        break;

    /* -------------------------------------------------------------- */
    case CT_MOVE_TO_INIT:
        if (!Motor_MoveInitPosition()) {
            UART_SendMessage(DBG_MSG_PC, "Motor Init Position Fail.");
            ct_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
#ifdef USE_AGING_MODE
            PrevCaptureMode = CAPTURE_CANCEL;
#endif
            ct_transition(CT_IDLE);
            break;
        }
        ct_transition(CT_CLEAR_PARAMS);
        break;

    /* -------------------------------------------------------------- */
    case CT_CLEAR_PARAMS:
        ct_clear_param();

        if (CurCaptureMode != CAPTURE_CT) {
            ct_cancel_process();
            ct_transition(CT_IDLE);
            break;
        }

#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
        ElapseTimerOnOff(SET);
#endif
        ct_transition(CT_LASER_ON);
        break;

    /* -------------------------------------------------------------- */
    case CT_LASER_ON:
        LaserControl(TYPE_HEAD_CT, SET);
        LaserControl(TYPE_FOOT, SET);

        UART_SendMessage(DBG_MSG_PC, "[SP_CT___READY]");
#ifdef USE_TABLET_PC
        UART_SendMessage(DBG_MSG_TABLET, "[SP_CT___READY]");
#endif

        if (sysInfo.bShowLog == SET) {
            printUart(DBG_MSG_PC, "CT Mode = %d", CtParam.Mode_15by9);
            printUart(DBG_MSG_PC, "CT Init Motor R::CurrentStep = %d", Motor_R.CurrentStep);
            printUart(DBG_MSG_PC, "CT Init Motor V::CurrentStep = %d", Motor_V.CurrentStep);
            printUart(DBG_MSG_PC, "CT Init Motor A::CurrentStep = %d", Motor_A.CurrentStep);
            printUart(DBG_MSG_PC, "CT Init Motor CNS::CurrentStep = %d", Motor_CNS.CurrentStep);
            printUart(DBG_MSG_PC, "CT Init Motor CWE::CurrentStep = %d", Motor_CWE.CurrentStep);
        }

        ct_transition(CT_WAIT_PARAMS);
        break;

    /* -------------------------------------------------------------- */
    case CT_WAIT_PARAMS:
#ifdef USE_AGING_MODE
        if (g_bAgingMode == SET) {
            ct_transition(CT_LASER_OFF);
            break;
        }
#endif
        if (UART_SetCtParam()) {
            ct_transition(CT_LASER_OFF);
        } else if (CurCaptureMode == CAPTURE_CANCEL) {
            ct_cancel_process();
            ct_transition(CT_IDLE);
        }
        break;

    /* -------------------------------------------------------------- */
    case CT_LASER_OFF:
        g_ct_motor_r_microstep = 16 * 200;
        g_ct_motor_r_degree_step = (360.0 / g_ct_motor_r_microstep / MOTOR_R_PULLEY_RATIO);

        if (g_bColumnPressed == SET) {
            Column_Control(COLUMN_STOP);
            g_bColumnPressed = FALSE;
        }

        LaserControl(TYPE_HEAD_CT, FALSE);
        LaserControl(TYPE_FOOT, FALSE);

        if (!bIsExposureSwitchReleased(500)) {
            UART_SendMessage(DBG_MSG_PC, ERR_CODE_EXP_SWITCH);
            CurCaptureMode = CAPTURE_CANCEL;
        }

        if (CurCaptureMode != CAPTURE_CT) {
            ct_cancel_process();
            ct_transition(CT_IDLE);
            break;
        }

        CAN_SendMessage(CAN_TUBE_TANK_TEMP_READ, 0, 0, 1000, 2);

        Lamp_Control(LAMP_RED, LAMP_DISABLE);
        Lamp_Control(LAMP_GREEN, LAMP_ENABLE);
        Lamp_Control(LAMP_BLUE, LAMP_DISABLE);

        ct_transition(CT_FOV_SETUP);
        break;

    /* -------------------------------------------------------------- */
    case CT_FOV_SETUP:
        switch (CtParam.Fov) {
        case CT_FOV_NONE:   g_ct_collimator_cmd = CMD_COLLI_OPEN;    break;
        case CT_FOV_5X5:    g_ct_collimator_cmd = CMD_COLLI_CT5X5;   break;
        case CT_FOV_8X9:    g_ct_collimator_cmd = CMD_COLLI_CT8X9;   break;
        case CT_FOV_8X10:   g_ct_collimator_cmd = CMD_COLLI_CT8X10;  break;
        case CT_FOV_10X9:   g_ct_collimator_cmd = CMD_COLLI_CT10X9;  break;
        case CT_FOV_12X9:   g_ct_collimator_cmd = CMD_COLLI_CT12X9;  break;
        case CT_FOV_15X9:   g_ct_collimator_cmd = CMD_COLLI_CT15X9;  break;
        case CT_FOV_10X10:  g_ct_collimator_cmd = CMD_COLLI_CT10X10; break;
        case CT_FOV_12X10:  g_ct_collimator_cmd = CMD_COLLI_CT12X10; break;
        case CT_FOV_15X10:  g_ct_collimator_cmd = CMD_COLLI_CT15X10; break;
        default:            g_ct_collimator_cmd = CMD_COLLI_CT15X9;  break;
        }

        if (sysInfo.bShowLog == SET) {
            printUart(DBG_MSG_PC, "FOV Set::CurrentStep = %d, nPaxisFovStep = %d",
                      Motor_V.CurrentStep, CtParam.nPaxisFovStep);
        }

        if (!CAN_Collimator_SendMessage(g_ct_collimator_cmd, 0, 0, 4000, 2)) {
            UART_SendMessage(DBG_MSG_PC, "Collimator FOV Move Fail.");
            ct_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
            ct_transition(CT_IDLE);
            break;
        }

        ct_transition(CT_FILTER_SETUP);
        break;

    /* -------------------------------------------------------------- */
    case CT_FILTER_SETUP:
        if (!CAN_Collimator_SendMessage(CMD_FILTER_CT, 0, 0, 4000, 2)) {
            UART_SendMessage(DBG_MSG_PC, "Collimator Filter Move Fail.");
            ct_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
            ct_transition(CT_IDLE);
            break;
        }
        ct_transition(CT_PULSED_MODE_SETUP);
        break;

    /* -------------------------------------------------------------- */
    case CT_PULSED_MODE_SETUP:
#if defined(USE_CT_XRAY_PULSED_MODE)
        if (CtParam.TubeMode == CT_TUBE_PULSED) {
            double readout = 0;
            double period = 0;

#if defined(USE_SENSOR_USER_SYNC)
            /* Wait for sensor update to finish (non-blocking poll) */
            if (Sensor_C.Update != FALSE) {
                break; /* stay in this state */
            }
            CtSensor_Stop();
#endif

            if (CtParam.Binning == CT_BINNING_1X1) {
                readout = 35.0;
            } else {
                readout = 17.3;
            }

            period = (CtParam.XonTime + CtParam.DelayTime * 2 + readout) / 1000;
            g_ct_fps = 1 / period;
            g_ct_time = CtParam.TotalFrame / g_ct_fps;

            Tube.Param.XonTime = CtParam.XonTime;
            Tube.Param.XoffTime = CtParam.XoffTime;
            Tube.Param.DelayTime = CtParam.DelayTime;
            Tube.Param.XonTimeCcr = ms_To_CCR(Tube.Param.XonTime);
            Tube.Param.XoffTimeCcr = ms_To_CCR(Tube.Param.XoffTime);
            Tube.Param.DelayTimeCcr = ms_To_CCR(Tube.Param.DelayTime);

            Sensor_C.Param.SyncLowTime = CtParam.XonTime + CtParam.XoffTime;
            Sensor_C.Param.SyncHighTime = 4.5;
            Sensor_C.Param.SyncLowTimeCcr = (double)(TIM_COUNT_CLOCK / 2 * Sensor_C.Param.SyncLowTime) / sec_To_msec;
            Sensor_C.Param.SyncHighTimeCcr = (double)(TIM_COUNT_CLOCK / 2 * Sensor_C.Param.SyncHighTime) / sec_To_msec;

#if defined(USE_SENSOR_USER_SYNC)
            CtSensor_Start();
#endif
        }
#endif /* USE_CT_XRAY_PULSED_MODE */
        ct_transition(CT_CALC_PARAMS);
        break;

    /* -------------------------------------------------------------- */
    case CT_CALC_PARAMS:
        if (CtParam.Binning == CT_BINNING_1X1) {
            g_ct_fps = 28.0;
            g_ct_acc_step = (uint32_t)(MOTOR_R_ACCEL_ANGLE / g_ct_motor_r_degree_step);
            g_ct_dec_step = (uint32_t)(MOTOR_R_DECEL_ANGLE / g_ct_motor_r_degree_step);
        } else {
            g_ct_fps = 56.0;
            if (CtParam.TotalFrame != RECON_22SEC_FRAME_NUMBER) {
                CtParam.nExpStartPos = EXPOSURE_FAST_START_POSITION;
            }
            g_ct_acc_step = (uint32_t)(MOTOR_R_FAST_ACCEL_ANGLE / g_ct_motor_r_degree_step);
            g_ct_dec_step = (uint32_t)(MOTOR_R_FAST_DECEL_ANGLE / g_ct_motor_r_degree_step);
        }

        g_ct_time = CtParam.TotalFrame / g_ct_fps;

        if (CtParam.nExpStartPos != 0) {
            CtParam.nXrayOnStep = (uint32_t)(CtParam.nExpStartPos / g_ct_motor_r_degree_step);
            CtParam.nXrayOffStep = CtParam.nXrayOnStep +
                                   (uint32_t)(CT_EXPOSURE_TOTAL_ANGLE / g_ct_motor_r_degree_step);
            g_ct_rotate_total_step = CtParam.nXrayOffStep + g_ct_dec_step +
                                     (uint32_t)(5.0 / g_ct_motor_r_degree_step);
        } else {
            CtParam.nXrayOnStep = (uint32_t)(CT_ROTATE_XON_ANGLE_DEFAULT / g_ct_motor_r_degree_step);
            CtParam.nXrayOffStep = (uint32_t)(CT_ROTATE_XOFF_ANGLE_DEFAULT / g_ct_motor_r_degree_step);
            g_ct_rotate_total_step = (uint32_t)(CT_ROTATE_TOTAL_ANGLE_DEFAULT / g_ct_motor_r_degree_step);
        }

        CtParam.nReverseTotalStep = g_ct_rotate_total_step;

        if (sysInfo.bShowLog == SET) {
            printUart(DBG_MSG_PC, "CtParam.nXrayOnStep(%d), CtParam.nXrayOffStep(%d)",
                      CtParam.nXrayOnStep, CtParam.nXrayOffStep);
            printUart(DBG_MSG_PC, "nAccStep(%d), nDecStep(%d), nRotateTotalStep(%d), CtParam.nEndStep(%d)",
                      g_ct_acc_step, g_ct_dec_step, g_ct_rotate_total_step, CtParam.nEndStep);
        }

        CtParam.nEndStep = g_ct_rotate_total_step - END_STEP_MARGIN;

        ct_transition(CT_TUBE_SETUP);
        break;

    /* -------------------------------------------------------------- */
    case CT_TUBE_SETUP:
        gMachStat.bTubeCountFlag = RESET;
        if (!CAN_SendMessage(CAN_TUBE_COUNT_CHECK, 0, 0, 1000, 2)) {
            UART_SendMessage(DBG_MSG_PC, "Tube Count Fail.");
            ct_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
            ct_transition(CT_IDLE);
            break;
        }

#if defined(USE_CT_XRAY_PULSED_MODE)
        if (CtParam.TubeMode == CT_TUBE_PULSED) {
            if (!CAN_SendMessage(CAN_TUBE_PULSE_MODE_SET, 0, 0, 1000, 2)) {
                ct_cancel_process();
                CurCaptureMode = CAPTURE_CANCEL;
                Motor_CTEnd();
                ct_transition(CT_IDLE);
                break;
            }
        } else {
            if (!CAN_SendMessage(CAN_TUBE_CONTI_MODE_SET, 0, 0, 1000, 2)) {
                ct_cancel_process();
                CurCaptureMode = CAPTURE_CANCEL;
                Motor_CTEnd();
                ct_transition(CT_IDLE);
                break;
            }
        }
#else
        if (!CAN_SendMessage(CAN_TUBE_CONTI_MODE_SET, 0, 0, 1000, 2)) {
            UART_SendMessage(DBG_MSG_PC, "Tube Continuous mode Fail.");
            ct_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
            Motor_CTEnd();
            ct_transition(CT_IDLE);
            break;
        }
#endif

        if (!CAN_SendMessage(CAN_TUBE_KV_SET, CtParam.KV, 0, 1000, 2)) {
            UART_SendMessage(DBG_MSG_PC, "Tube KV Send Fail.");
            ct_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
            Motor_CTEnd();
            ct_transition(CT_IDLE);
            break;
        }

        if (!CAN_SendMessage(CAN_TUBE_mA_SET, CtParam.mA, 0, 1000, 2)) {
            UART_SendMessage(DBG_MSG_PC, "Tube mA Send Fail.");
            ct_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
            Motor_CTEnd();
            ct_transition(CT_IDLE);
            break;
        }

        ct_transition(CT_MOTOR_START_POS);
        break;

    /* -------------------------------------------------------------- */
    case CT_MOTOR_START_POS:
#ifdef USE_CT_STITCH_MODE
        if (!g_ct_capture_count) {
            Motor_MoveStartPosition();
        }
#else
        Motor_R.CurrentStep = 0;
#endif
        if (sysInfo.bShowLog == SET) {
            printUart(DBG_MSG_PC, "CT ::TotalFrame = %d, dTime = %f ", CtParam.TotalFrame, g_ct_time);
            printUart(DBG_MSG_PC, "CT ::kV = %d , mA = %d", CtParam.KV, CtParam.mA);
            printUart(DBG_MSG_PC, "CT Start Motor R::CurrentStep = %d", Motor_R.CurrentStep);
            printUart(DBG_MSG_PC, "CT Start Motor V::CurrentStep = %d", Motor_V.CurrentStep);
        }

        ct_transition(CT_STITCH_SETUP);
        break;

    /* -------------------------------------------------------------- */
    case CT_STITCH_SETUP:
#ifdef USE_CT_STITCH_MODE
    {
        bool stitch_success = true;

        if ((CtParam.bStitchMode == TRUE) && (g_ct_capture_count)) {
            stitch_success = Motor_MoveStitchPosition(TOP_DOWN, 317, 325);

            if (stitch_success == FALSE) {
                /* Stitch positioning failed */
                Tube_CtrlReady(TUBE_READY_DISABLE);
                Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
                Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
                CurCaptureMode = CAPTURE_CANCEL;
                Motor_CTEnd();
                UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_STOP_]");
#ifdef USE_TABLET_PC
                UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_STOP_]");
#endif
                ct_cancel_process();
                ct_transition(CT_IDLE);
                break;
            }

            ct_transition(CT_WAIT_STITCH_CONFIRM);
            break;
        }
    }
#endif
        ct_transition(CT_EXPOSURE_SETUP);
        break;

    /* -------------------------------------------------------------- */
    case CT_WAIT_STITCH_CONFIRM:
#ifdef USE_CT_STITCH_MODE
        if (UART_WaitProtocol("[SM_CAPT_CONFI]")) {
            CtParam.bStitchORGCheck = SET;
            ct_transition(CT_EXPOSURE_SETUP);
        } else if (CurCaptureMode == CAPTURE_CANCEL) {
            Motor_CTEnd();
            ct_cancel_process();
            ct_transition(CT_IDLE);
        }
        /* else: stay and poll next iteration */
#else
        ct_transition(CT_EXPOSURE_SETUP);
#endif
        break;

    /* -------------------------------------------------------------- */
    case CT_EXPOSURE_SETUP:
#ifdef USE_CT_STITCH_MODE
        Motor_ClearCurrentStep(&Motor_R);

        if (sysInfo.bShowLog == TRUE) {
            printUart(DBG_MSG_PC, "CurrentStep(%d), capture_count(%d)",
                      Motor_R.CurrentStep, g_ct_capture_count);
        }
#endif

        Exposure_CtrlLed(EXPOSURE_LED_GPIO_ENABLE);
        UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_CONFI]");
        UART_SendMessage(DBG_MSG_PC, "[SP_STBY_EXPSW]");
        Indicator_Control(INDICATOR_ENABLE);

        Timer_Config_T(&Motor_R.Timer);

        ct_transition(CT_WAIT_EXPOSURE_SW);
        break;

    /* -------------------------------------------------------------- */
    case CT_WAIT_EXPOSURE_SW:
        if (!Exposure_CheckSwitch() || UART_Exposure_SET() == 1) {
            g_ct_exp_check = (BitAction)GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7);
            ct_transition(CT_XRAY_INTERLOCK);
        } else if (CurCaptureMode != CAPTURE_CT) {
            Motor_CTEnd();
            ct_cancel_process();
            ct_transition(CT_IDLE);
        }
        break;

    /* -------------------------------------------------------------- */
    case CT_XRAY_INTERLOCK:
        if (!Safety_VerifyXrayInterlock()) {
            printUart(DBG_MSG_PC, "SAFETY: X-ray interlock fault, aborting CT");
            Motor_CTEnd();
            ct_cancel_process();
            CurCaptureMode = CAPTURE_CANCEL;
            ct_transition(CT_IDLE);
            break;
        }
        ct_transition(CT_TUBE_READY);
        break;

    /* -------------------------------------------------------------- */
    case CT_TUBE_READY:
        Tube_CtrlReady(TUBE_READY_ENABLE);
        UART_SendMessage(DBG_MSG_PC, "Wait for Tube Heatting Time");
        IntTimer_Delay(2000);
        ct_transition(CT_TUBE_HEAT_WAIT);
        break;

    /* -------------------------------------------------------------- */
    case CT_TUBE_HEAT_WAIT:
        if (IntTimer_GetStatus()) {
            ct_transition(CT_START_ROTATION);
        }
        break;

    /* -------------------------------------------------------------- */
    case CT_START_ROTATION:
        UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_START]");
#ifdef USE_TABLET_PC
        UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_START]");
#endif
        gMachStat.bTubeCountCompareFlag = TRUE;

#ifdef USE_CT_STITCH_MODE
    {
        char reverse = FALSE;
        if ((CtParam.bStitchMode == TRUE) && (g_ct_capture_count)) {
            reverse = TRUE;
        }
        if (CtParam.bReverse_Status == SET) {
            reverse = SET;
        }
        MotorR_MoveAbsoluteDistance(&Motor_R, g_ct_time, g_ct_acc_step,
                                     g_ct_rotate_total_step, g_ct_dec_step, reverse);
    }
#else
        MotorR_MoveAbsoluteDistance(&Motor_R, g_ct_time, g_ct_acc_step,
                                     g_ct_rotate_total_step, g_ct_dec_step);
#endif

        g_ct_stop_flag = false;
        ct_transition(CT_EXPOSING);
        break;

    /* -------------------------------------------------------------- */
    case CT_EXPOSING:
        if (g_ct_exp_check == Bit_SET) {
            /* Software exposure control */
            if (UART_Exposure_SET() == 2) {
                g_ct_stop_flag = true;
            }
        } else {
            /* Hardware exposure switch */
            if (Exposure_CheckSwitch()) {
                g_ct_stop_flag = true;
            }
        }

#ifdef USE_TUBE_PPS_TYPE_IO
        if (Sensor_C.bSetFrameCount == TRUE) {
            if (PanoSensor_CheckInput()) {
                Tube_CtrlPps(Bit_RESET);
            } else {
                Tube_CtrlPps(Bit_SET);
            }
        }
#endif

        if (g_ct_stop_flag && !CtParam.bCaptEnd) {
            Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
            Indicator_Control(INDICATOR_DISABLE);
            ct_transition(CT_WAIT_ROTATION_END);
            break;
        }

        if (CtParam.bCaptEnd == TRUE) {
            Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
            Indicator_Control(INDICATOR_DISABLE);
            ct_transition(CT_WAIT_ROTATION_END);
        }
        break;

    /* -------------------------------------------------------------- */
    case CT_WAIT_ROTATION_END:
        /* Wait for R motor to stop (non-blocking) */
        if (!MotorWait_IsStopped(&Motor_R)) {
            break;
        }

        if (sysInfo.bShowLog == TRUE) {
            printUart(DBG_MSG_PC, "Current Step : %d", Motor_R.CurrentStep);
        }

        ct_transition(CT_STITCH_CHECK);
        break;

    /* -------------------------------------------------------------- */
    case CT_STITCH_CHECK:
#ifdef USE_CT_STITCH_MODE
        if (g_ct_stop_flag && !CtParam.bCaptEnd) {
            /* User stopped - not a clean capture */
#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
            typElapsedTimer.nExposeTime = nElapsedMilliSec(typElapsedTimer.nExposeTime);
#endif
            ct_transition(CT_STOP);
            break;
        }

        if ((CtParam.bStitchMode == TRUE) && (!g_ct_capture_count) && (CtParam.bCaptEnd == TRUE)) {
            /* First stitch capture done, prepare for second */
            CurCaptureMode = CAPTURE_CANCEL;
            Motor_CTEnd();
            CurCaptureMode = CAPTURE_CT;
            CtParam.bCaptEnd = FALSE;

            UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_END__]");
#ifdef USE_TABLET_PC
            UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_END_F]");
#endif
            g_ct_capture_count++;
            /* Loop back to tube setup for second capture */
            ct_transition(CT_TUBE_SETUP);
            break;
        }

        if (CtParam.bCaptEnd == TRUE) {
            ct_transition(CT_COMPLETE);
        } else {
            ct_transition(CT_STOP);
        }
#else
        if (g_ct_stop_flag) {
#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
            typElapsedTimer.nExposeTime = nElapsedMilliSec(typElapsedTimer.nExposeTime);
#endif
            ct_transition(CT_STOP);
        } else {
            ct_transition(CT_COMPLETE);
        }
#endif
        break;

    /* -------------------------------------------------------------- */
    case CT_COMPLETE:
    {
        /* Wait for R motor to fully stop */
        if (!MotorWait_IsStopped(&Motor_R)) {
            break;
        }

        UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_END__]");
#ifdef USE_TABLET_PC
        UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_END__]");
#endif

        if (CtParam.bFirstCT == RESET) {
            CtParam.bFirstCT = SET;
            UART_SendMessage(DBG_MSG_PC, "[SP_CT___DEFEC]");
        }

#ifdef USE_CT_STITCH_MODE
        if (CtParam.bStitchMode == TRUE || CtParam.bReverse_Status == SET) {
            if (CtParam.TotalFrame == 400) {
                Motor_SetDirMovePosistion(&Motor_R, DIR_LIMIT, 3000,
                                           9600 - CtParam.nFastRaxisOffset);
            } else {
                Motor_SetDirMovePosistion(&Motor_R, DIR_LIMIT, 3000,
                                           9600 - CtParam.nRaxisOffset);
            }
            /* Wait for R motor in next iteration */
            ct_transition(CT_MOVE_TO_END);
            break;
        }
#endif
        Motor_MoveEndPosition();
        ct_transition(CT_MOVE_TO_END);
        break;
    }

    /* -------------------------------------------------------------- */
    case CT_MOVE_TO_END:
        /* Wait for R motor to stop after end move */
        if (!MotorWait_IsStopped(&Motor_R)) {
            break;
        }

        CurCaptureMode = CAPTURE_CANCEL;
        Motor_CTEnd();

        if (sysInfo.bShowLog == TRUE) {
            printUart(DBG_MSG_PC, "CT END Motor R::CurrentStep = %d", Motor_R.CurrentStep);
        }

#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
        printUart(DBG_MSG_PC, "Exposed Time : %d(ms)", typElapsedTimer.nExposeTime);
        ElapseTimerOnOff(FALSE);
#endif

#ifdef USE_CT_STITCH_MODE
        if (CtParam.bStitchBack == TRUE) {
            ct_transition(CT_STITCH_BACK);
            break;
        }
#endif
        ct_clear_process();
        ct_transition(CT_IDLE);
        break;

    /* -------------------------------------------------------------- */
    case CT_STITCH_BACK:
#ifdef USE_CT_STITCH_MODE
        Motor_MoveStitchPosition(BOTTOM_UP, 317, 325);
#endif
        ct_clear_process();
        ct_transition(CT_IDLE);
        break;

    /* -------------------------------------------------------------- */
    case CT_STOP:
        Tube_CtrlReady(TUBE_READY_DISABLE);
        Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
        Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);

        CurCaptureMode = CAPTURE_CANCEL;
        Motor_CTEnd();

        UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_STOP_]");
#ifdef USE_TABLET_PC
        UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_STOP_]");
#endif

#if defined(BUILD_TYPE_DEBUG) && !defined(USE_AGING_MODE)
        printUart(DBG_MSG_PC, "Exposed Time : %d(ms)", typElapsedTimer.nExposeTime);
        ElapseTimerOnOff(FALSE);
#endif

        UART_SendMessage(DBG_MSG_PC, ERR_CODE_CT_CAPT_STOP);

        if (MotorR_CheckOrgSequence() == SET) {
            if (sysInfo.bShowLog == TRUE) {
                printUart(DBG_MSG_PC, "R-Axis Org Sequence 2 : CurStep(%d)", Motor_R.CurrentStep);
            }
        }
        if (sysInfo.bShowLog == TRUE) {
            printUart(DBG_MSG_PC, "CT Stop Motor R::CurrentStep = %d", Motor_R.CurrentStep);
        }

        ct_cancel_process();
        ct_transition(CT_IDLE);
        break;

    /* -------------------------------------------------------------- */
    case CT_CANCEL:
        ct_cancel_process();
        ct_transition(CT_IDLE);
        break;

    /* -------------------------------------------------------------- */
    case CT_ERROR:
        ct_cancel_process();
        ct_transition(CT_IDLE);
        break;

    default:
        ct_transition(CT_IDLE);
        break;
    }
}

/* ================================================================== */
/* Private helpers (from legacy ct_capture.c)                          */
/* ================================================================== */

static void ct_clear_param(void)
{
    CtParam.KV = 80;
    CtParam.mA = 80;
    CtParam.Fov = CT_FOV_15X9;
    CtParam.Binning = CT_BINNING_1X1;
#if defined(USE_CT_XRAY_PULSED_MODE)
    CtParam.TubeMode = CT_TUBE_PULSED;
#endif
    CtParam.XonTime = 15;
    CtParam.XoffTime = 1;
    CtParam.DelayTime = 1;
    CtParam.TotalFrame = 600;
    CtParam.nMotorVStartPos = CT_MOTORV_START_OFFSET_DEFAULT;
    CtParam.nExpStartPos = EXPOSURE_START_POSITION;
    CtParam.nRunStep = 0;
    CtParam.nXrayOnStep = 0;
    CtParam.nXrayOffStep = 0;
    CtParam.nEndStep = 0;
    CtParam.bCaptEnd = FALSE;
    CtParam.nPaxisFovStep = 0;
    CtParam.bPaxisSET = RESET;
    CtParam.bCNSaxisSET = RESET;
    CtParam.bCWEaxisSET = RESET;
#ifdef USE_CT_STITCH_MODE
    CtParam.bStitchMode = FALSE;
    CtParam.bStitchBack = FALSE;
    CtParam.bStitchORGCheck = FALSE;
    g_ct_fps = 0;
    g_ct_time = 0;
    g_ct_rotate_total_step = 0;
    g_ct_acc_step = 0;
    g_ct_dec_step = 0;
    g_ct_capture_count = 0;
#endif
    CtParam.bStitch_Status = StitchUnderSensor_Check();
}

static void ct_cancel_process(void)
{
    UART_SendMessage(DBG_MSG_PC, "[SP_CAPT_CANCE]");
#ifdef USE_TABLET_PC
    UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_CANCE]");
#endif
    ct_clear_process();
}

static void ct_clear_process(void)
{
    if (sysInfo.bShowLog == TRUE) {
        UART_SendMessage(DBG_MSG_PC, "Run CtCapture_ClearProcess");
    }

    ct_clear_param();

    if (sysInfo.bXrayNoSound == TRUE) {
        sysInfo.bXrayNoSound = FALSE;
        CAN_SendMessage(CAN_TUBE_SOUND_ON, 0, 0, 1000, 2);
    }

    Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);

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

#if defined(USE_CT_XRAY_PULSED_MODE) && defined(USE_SENSOR_USER_SYNC)
    {
        /* Skip blocking wait - just stop sensor */
        CtSensor_Stop();
    }
#endif

#ifdef USE_AGING_MODE
    PrevCaptureMode = CAPTURE_CT;
    g_nCtCnt++;
    SysTime_MesureEnd(__FUNCTION__, __LINE__);
#endif

    LampTimer_Enable();
}
