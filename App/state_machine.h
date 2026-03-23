/*
 * state_machine.h : Event-driven system state machine
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Central state machine for the T2 dental X-ray system.
 * Replaces CurCaptureMode-based blocking dispatch with non-blocking
 * event-driven architecture. No function in App/ may block.
 *
 * Memory: ~200 bytes Flash (header), see .c for implementation.
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/* System states                                                       */
/* ------------------------------------------------------------------ */
typedef enum {
    ST_IDLE = 0,            /* No capture active, waiting for command */
    ST_SYSTEM_INIT,         /* Motor origin search at boot */
    ST_MOVE_TO_POSITION,    /* Motors moving to init/end position */
    ST_PANO_SEQUENCE,       /* Panoramic capture sequence */
    ST_CT_SEQUENCE,         /* CT/CBCT capture sequence */
    ST_SCAN_SEQUENCE,       /* Cephalometric scan sequence */
    ST_CALIBRATION,         /* Calibration mode */
    ST_GEO_ALIGN,           /* Geometry alignment mode */
    ST_DIAGNOSTIC,          /* Diagnostic mode */
    ST_EEPROM,              /* EEPROM configuration mode */
    ST_RESET,               /* Reset/cleanup in progress */
    ST_ERROR,               /* Error state */
    ST_COUNT
} SystemState_t;

/* ------------------------------------------------------------------ */
/* System events                                                       */
/* ------------------------------------------------------------------ */
typedef enum {
    EVT_NONE = 0,
    EVT_CMD_RECEIVED,           /* UART command arrived */
    EVT_MOTOR_DONE,             /* All waited motors stopped */
    EVT_MOTOR_ERROR,            /* Motor timeout or fault */
    EVT_TIMEOUT,                /* State timeout expired */
    EVT_EMERGENCY,              /* Emergency switch pressed */
    EVT_CANCEL,                 /* Cancel command from PC */
    EVT_EXPOSURE_CONFIRM,       /* PC confirms capture ready */
    EVT_EXPOSURE_SWITCH_ON,     /* Exposure hand switch pressed */
    EVT_EXPOSURE_SWITCH_OFF,    /* Exposure hand switch released */
    EVT_CAN_RESPONSE,           /* CAN response received */
    EVT_CAN_ERROR,              /* CAN communication error */
    EVT_SEQUENCE_DONE,          /* Sub-sequence completed */
    EVT_SEQUENCE_ERROR,         /* Sub-sequence error */
    EVT_MODE_PANO,              /* Enter pano mode */
    EVT_MODE_CT,                /* Enter CT mode */
    EVT_MODE_SCAN,              /* Enter scan mode */
    EVT_MODE_CALIBRATION,       /* Enter calibration mode */
    EVT_MODE_GEO_ALIGN,         /* Enter geo alignment mode */
    EVT_MODE_EEPROM,            /* Enter EEPROM mode */
    EVT_MODE_DIAGNOSTIC,        /* Enter diagnostic mode */
    EVT_MODE_RESET,             /* Enter reset mode */
    EVT_ERROR_CLEAR,            /* Clear error state */
    EVT_COUNT
} SystemEvent_t;

/* ------------------------------------------------------------------ */
/* Event queue size (must be power of 2)                               */
/* ------------------------------------------------------------------ */
#define SM_EVENT_QUEUE_SIZE     16U
#define SM_EVENT_QUEUE_MASK     (SM_EVENT_QUEUE_SIZE - 1U)

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

/**
 * Initialize state machine to ST_IDLE.
 * Must be called once after all peripherals are initialized.
 */
void StateMachine_Init(void);

/**
 * Run one iteration of the state machine.
 * Called every main loop iteration (~1ms). Must return within 100us.
 * Processes queued events and dispatches to the active state handler.
 */
void StateMachine_Run(void);

/**
 * Post an event to the state machine queue.
 * ISR-safe (no dynamic allocation, no blocking).
 *
 * @param evt  Event to enqueue
 * @return true if event was queued, false if queue is full
 */
bool StateMachine_PostEvent(SystemEvent_t evt);

/**
 * Get current system state.
 */
SystemState_t StateMachine_GetState(void);

/**
 * Check if system is in any capture state.
 */
bool StateMachine_IsCapturing(void);

/**
 * Check if system is in error state.
 */
bool StateMachine_IsError(void);

/**
 * Get the name string for a given state (for debug logging).
 */
const char* StateMachine_GetStateName(SystemState_t st);

/**
 * Get the name string for a given event (for debug logging).
 */
const char* StateMachine_GetEventName(SystemEvent_t evt);

#endif /* STATE_MACHINE_H */
