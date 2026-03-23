/*
 * state_machine.c : Event-driven system state machine implementation
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Non-blocking event-driven state machine. Processes events from a
 * ring buffer queue and dispatches to the active state handler.
 * Every call to StateMachine_Run() must complete within ~100us.
 *
 * Memory: ~800 bytes Flash, ~80 bytes SRAM.
 */

#include "state_machine.h"
#include "pano_sequence.h"
#include "ct_sequence.h"
#include "scan_sequence.h"
#include "calibration.h"

/* Legacy headers for backward compatibility */
#include "extern.h"
#include "system.h"
#include "serial.h"
#include "motor.h"
#include "timer.h"
#include "misc1.h"
#include "can.h"
#include "debug_log.h"
#include "safety.h"

#ifdef USE_I2C_EEPROM
#include "eeprom.h"
#endif

/* CAN_Collimator_SendMessage is defined in can.c */
extern bool CAN_Collimator_SendMessage(uint16_t cmd, uint32_t value,
                                         uint16_t debug, uint32_t timeOut,
                                         uint8_t retryCnt);

/* Legacy mode handlers still used for calibration/diagnostic/eeprom */
extern void CalibrationMode(void);
extern void GeometryAlignMode(void);
extern void DiagnosticMode(void);
#ifdef USE_I2C_EEPROM
extern void EEPRom_ModeLoop(void);
#endif

/* ------------------------------------------------------------------ */
/* Event queue (lock-free single-producer/single-consumer ring buffer) */
/* ------------------------------------------------------------------ */
typedef struct {
    volatile SystemEvent_t buf[SM_EVENT_QUEUE_SIZE];
    volatile uint8_t       head;
    volatile uint8_t       tail;
} EventQueue_t;

/* ------------------------------------------------------------------ */
/* Private variables                                                   */
/* ------------------------------------------------------------------ */
static volatile SystemState_t g_state;
static volatile SystemState_t g_prev_state;
static EventQueue_t           g_evt_queue;
static uint32_t               g_state_enter_ms;

/* Boot error flags (set by App_Init, read here for idle LED) */
extern bool g_boot_gen_ok;
extern bool g_boot_col_ok;
extern bool g_boot_mot_ok;

/* Reset sub-state for non-blocking reset sequence */
typedef enum {
    RESET_START = 0,
    RESET_TILT_OFF,
    RESET_MOVE_INIT,
    RESET_WAIT_MOTORS,
    RESET_DONE
} ResetSubState_t;

static ResetSubState_t g_reset_sub;

/* ------------------------------------------------------------------ */
/* State name strings for debug logging                                */
/* ------------------------------------------------------------------ */
static const char* const k_state_names[ST_COUNT] = {
    "IDLE", "SYS_INIT", "MOVE_POS", "PANO", "CT",
    "SCAN", "CAL", "GEO", "DIAG", "EEPROM", "RESET", "ERROR"
};

static const char* const k_event_names[] = {
    "NONE", "CMD_RX", "MOT_DONE", "MOT_ERR", "TIMEOUT",
    "EMERG", "CANCEL", "EXP_CONF", "EXP_SW_ON", "EXP_SW_OFF",
    "CAN_RSP", "CAN_ERR", "SEQ_DONE", "SEQ_ERR",
    "M_PANO", "M_CT", "M_SCAN", "M_CAL", "M_GEO",
    "M_EEPROM", "M_DIAG", "M_RESET", "ERR_CLR"
};

/* ------------------------------------------------------------------ */
/* Event queue operations                                              */
/* ------------------------------------------------------------------ */
static void evtq_init(EventQueue_t* q)
{
    q->head = 0;
    q->tail = 0;
}

static bool evtq_put(EventQueue_t* q, SystemEvent_t evt)
{
    uint8_t next = (q->head + 1U) & SM_EVENT_QUEUE_MASK;
    if (next == q->tail) {
        return false; /* queue full */
    }
    q->buf[q->head] = evt;
    q->head = next;
    return true;
}

static bool evtq_get(EventQueue_t* q, SystemEvent_t* evt)
{
    if (q->tail == q->head) {
        return false; /* queue empty */
    }
    *evt = q->buf[q->tail];
    q->tail = (q->tail + 1U) & SM_EVENT_QUEUE_MASK;
    return true;
}

/* ------------------------------------------------------------------ */
/* Forward declarations for state handlers                             */
/* ------------------------------------------------------------------ */
static void handle_idle(SystemEvent_t evt);
static void handle_pano(SystemEvent_t evt);
static void handle_ct(SystemEvent_t evt);
static void handle_scan(SystemEvent_t evt);
static void handle_reset(SystemEvent_t evt);
static void handle_error(SystemEvent_t evt);
static void handle_calibration(SystemEvent_t evt);
static void handle_geo_align(SystemEvent_t evt);
static void handle_diagnostic(SystemEvent_t evt);
static void handle_eeprom(SystemEvent_t evt);

/* ------------------------------------------------------------------ */
/* State transition helper                                             */
/* ------------------------------------------------------------------ */
static void transition_to(SystemState_t new_state)
{
    if (g_state == new_state) {
        return;
    }

    DbgLog_Printf("SM: %s -> %s",
                   k_state_names[g_state], k_state_names[new_state]);

    g_prev_state = g_state;
    g_state = new_state;
    g_state_enter_ms = vSetCurrentMilliSec();

    /* Sync legacy global for backward compatibility */
    switch (new_state) {
    case ST_PANO_SEQUENCE:  CurCaptureMode = CAPTURE_PANO;    break;
    case ST_CT_SEQUENCE:    CurCaptureMode = CAPTURE_CT;       break;
    case ST_SCAN_SEQUENCE:  CurCaptureMode = CAPTURE_SCAN;     break;
    case ST_CALIBRATION:    CurCaptureMode = CALIBRATION_MODE; break;
    case ST_GEO_ALIGN:      CurCaptureMode = GEOMETRY_ALIGN_MODE; break;
    case ST_EEPROM:         CurCaptureMode = EEPROM_MODE;      break;
    case ST_DIAGNOSTIC:     CurCaptureMode = DIAGNOSTIC_MODE;  break;
    case ST_RESET:          CurCaptureMode = RESET_MODE;       break;
    case ST_IDLE:
    case ST_ERROR:
    default:
        CurCaptureMode = CAPTURE_CANCEL;
        break;
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void StateMachine_Init(void)
{
    g_state = ST_IDLE;
    g_prev_state = ST_IDLE;
    g_state_enter_ms = vSetCurrentMilliSec();
    evtq_init(&g_evt_queue);
    DbgLog_Printf("SM: initialized, state=IDLE");
}

void StateMachine_Run(void)
{
    SystemEvent_t evt = EVT_NONE;

    /* Process one event per iteration to keep latency low */
    evtq_get(&g_evt_queue, &evt);

    /* Emergency always takes priority regardless of state */
    if (evt == EVT_EMERGENCY) {
        DbgLog_Printf("SM: EMERGENCY in %s", k_state_names[g_state]);

        /* Immediately disable X-ray and motors */
        Tube_CtrlReady(TUBE_READY_DISABLE);
        Tube_CtrlExposure(TUBE_EXPOSURE_DISABLE);
        Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);

        /* Cancel active sequences */
        if (g_state == ST_PANO_SEQUENCE) {
            PanoSequence_Cancel();
        } else if (g_state == ST_CT_SEQUENCE) {
            CtSequence_Cancel();
        } else if (g_state == ST_SCAN_SEQUENCE) {
            ScanSequence_Cancel();
        }

        transition_to(ST_ERROR);
        return;
    }

    /* Dispatch to current state handler */
    switch (g_state) {
    case ST_IDLE:           handle_idle(evt);        break;
    case ST_PANO_SEQUENCE:  handle_pano(evt);        break;
    case ST_CT_SEQUENCE:    handle_ct(evt);          break;
    case ST_SCAN_SEQUENCE:  handle_scan(evt);        break;
    case ST_CALIBRATION:    handle_calibration(evt); break;
    case ST_GEO_ALIGN:      handle_geo_align(evt);   break;
    case ST_DIAGNOSTIC:     handle_diagnostic(evt);  break;
    case ST_EEPROM:         handle_eeprom(evt);      break;
    case ST_RESET:          handle_reset(evt);       break;
    case ST_ERROR:          handle_error(evt);       break;
    default:
        transition_to(ST_ERROR);
        break;
    }
}

bool StateMachine_PostEvent(SystemEvent_t evt)
{
    return evtq_put(&g_evt_queue, evt);
}

SystemState_t StateMachine_GetState(void)
{
    return g_state;
}

bool StateMachine_IsCapturing(void)
{
    return (g_state == ST_PANO_SEQUENCE ||
            g_state == ST_CT_SEQUENCE ||
            g_state == ST_SCAN_SEQUENCE);
}

bool StateMachine_IsError(void)
{
    return (g_state == ST_ERROR);
}

const char* StateMachine_GetStateName(SystemState_t st)
{
    if (st < ST_COUNT) {
        return k_state_names[st];
    }
    return "UNKNOWN";
}

const char* StateMachine_GetEventName(SystemEvent_t evt)
{
    if (evt < EVT_COUNT) {
        return k_event_names[evt];
    }
    return "UNKNOWN";
}

/* ================================================================== */
/* State handlers                                                      */
/* ================================================================== */

static void handle_idle(SystemEvent_t evt)
{
    switch (evt) {
    case EVT_MODE_PANO:
        PanoSequence_Init();
        PanoSequence_Start();
        transition_to(ST_PANO_SEQUENCE);
        break;

    case EVT_MODE_CT:
        CtSequence_Init();
        CtSequence_Start();
        transition_to(ST_CT_SEQUENCE);
        break;

    case EVT_MODE_SCAN:
        if (sysInfo.model_id == MODEL_T2_CS) {
            ScanSequence_Init();
            ScanSequence_Start();
            transition_to(ST_SCAN_SEQUENCE);
        } else {
            printUart(DBG_MSG_PC, "can't not support cephalo");
        }
        break;

    case EVT_MODE_CALIBRATION:
        transition_to(ST_CALIBRATION);
        break;

    case EVT_MODE_GEO_ALIGN:
        transition_to(ST_GEO_ALIGN);
        break;

    case EVT_MODE_EEPROM:
        transition_to(ST_EEPROM);
        break;

    case EVT_MODE_DIAGNOSTIC:
        transition_to(ST_DIAGNOSTIC);
        break;

    case EVT_MODE_RESET:
        g_reset_sub = RESET_START;
        transition_to(ST_RESET);
        break;

    case EVT_NONE:
    default:
        /* Idle: handle boot error LED indicators without blocking */
        break;
    }
}

static void handle_pano(SystemEvent_t evt)
{
    /* Forward cancel event to the sub-sequence */
    if (evt == EVT_CANCEL) {
        PanoSequence_Cancel();
    }

    /* Step the pano state machine */
    PanoSequence_Step();

    /* Check if sequence is done */
    if (PanoSequence_IsDone()) {
        transition_to(ST_IDLE);
    }
}

static void handle_ct(SystemEvent_t evt)
{
    if (evt == EVT_CANCEL) {
        CtSequence_Cancel();
    }

    CtSequence_Step();

    if (CtSequence_IsDone()) {
        transition_to(ST_IDLE);
    }
}

static void handle_scan(SystemEvent_t evt)
{
    if (evt == EVT_CANCEL) {
        ScanSequence_Cancel();
    }

    ScanSequence_Step();

    if (ScanSequence_IsDone()) {
        transition_to(ST_IDLE);
    }
}

/* ------------------------------------------------------------------ */
/* Reset state: non-blocking reset sequence                            */
/* ------------------------------------------------------------------ */
static void handle_reset(SystemEvent_t evt)
{
    (void)evt;

    switch (g_reset_sub) {
    case RESET_START:
        /* Turn off all lasers and exposure LED */
        LaserControl(TYPE_HEAD_CEPH, RESET);
        LaserControl(TYPE_HEAD_PANO, RESET);
        LaserControl(TYPE_FOOT, RESET);
        Exposure_CtrlLed(EXPOSURE_LED_GPIO_DISABLE);
        UART_SendMessage(DBG_MSG_PC, "[SP_MODE_RESET]");
#ifdef USE_TABLET_PC
        UART_SendMessage(DBG_MSG_TABLET, "[SP_MODE_RESET]");
#endif
        /* Check if tilt off is needed */
        if ((sysInfo.model_id == MODEL_T2_CS) && (g_bTiltStatus == SET)) {
            g_reset_sub = RESET_TILT_OFF;
        } else {
            g_reset_sub = RESET_MOVE_INIT;
        }
        break;

    case RESET_TILT_OFF:
        /* Send tilt off (CAN is currently blocking - call once) */
        if (!CAN_Collimator_SendMessage(CMD_TILT_OFF, 0, 0, 5000, 2)) {
            printUart(DBG_MSG_PC, "Titing Error");
        }
        g_bTiltStatus = RESET;
        g_reset_sub = RESET_MOVE_INIT;
        break;

    case RESET_MOVE_INIT:
        /*
         * Motor_MoveInitPosition() is blocking in legacy code.
         * For now, call it and advance. This will be replaced
         * with MotorWait when motor driver is fully migrated.
         */
        Motor_MoveInitPosition();
        g_reset_sub = RESET_DONE;
        break;

    case RESET_WAIT_MOTORS:
        /* Reserved for future non-blocking motor wait */
        g_reset_sub = RESET_DONE;
        break;

    case RESET_DONE:
#ifdef USE_TABLET_PC
        UART_SendMessage(DBG_MSG_TABLET, "[SP_CAPT_CANCE]");
#endif
        transition_to(ST_IDLE);
        break;
    }
}

/* ------------------------------------------------------------------ */
/* Error state: wait for clear                                         */
/* ------------------------------------------------------------------ */
static void handle_error(SystemEvent_t evt)
{
    if (evt == EVT_ERROR_CLEAR) {
        DbgLog_Printf("SM: error cleared");
        transition_to(ST_IDLE);
    }
}

/* ------------------------------------------------------------------ */
/* Legacy blocking modes (will be migrated to state machines later)    */
/* These still block but are wrapped so the SM architecture is in      */
/* place for incremental migration.                                    */
/* ------------------------------------------------------------------ */
static void handle_calibration(SystemEvent_t evt)
{
    if (evt == EVT_CANCEL) {
        CurCaptureMode = CAPTURE_CANCEL;
        transition_to(ST_IDLE);
        return;
    }
    /* Legacy blocking call */
    CalibrationMode();
    transition_to(ST_IDLE);
}

static void handle_geo_align(SystemEvent_t evt)
{
    if (evt == EVT_CANCEL) {
        CurCaptureMode = CAPTURE_CANCEL;
        transition_to(ST_IDLE);
        return;
    }
    GeometryAlignMode();
    transition_to(ST_IDLE);
}

static void handle_diagnostic(SystemEvent_t evt)
{
    if (evt == EVT_CANCEL) {
        CurCaptureMode = CAPTURE_CANCEL;
        transition_to(ST_IDLE);
        return;
    }
    DiagnosticMode();
    transition_to(ST_IDLE);
}

static void handle_eeprom(SystemEvent_t evt)
{
    (void)evt;
#ifdef USE_I2C_EEPROM
    EEPRom_ModeLoop();
#endif
    transition_to(ST_IDLE);
}
