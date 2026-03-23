/*
 * state_machine.h : System state machine (IDLE -> READY -> CAPTURE -> ERROR)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Central state machine for the T2 dental X-ray system.
 * Maps to the existing CurCaptureMode-based dispatch.
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * System states.
 * These map to the existing OpStatus_t values for backward compatibility.
 */
typedef enum {
    SYS_STATE_IDLE = 0,         /* No capture active, waiting for command */
    SYS_STATE_PANO,             /* Panoramic capture mode */
    SYS_STATE_CT,               /* CT/CBCT capture mode */
    SYS_STATE_SCAN,             /* Cephalometric scan mode */
    SYS_STATE_CALIBRATION,      /* Calibration mode */
    SYS_STATE_GEO_ALIGN,        /* Geometry alignment mode */
    SYS_STATE_EEPROM,           /* EEPROM configuration mode */
    SYS_STATE_DIAGNOSTIC,       /* Diagnostic mode */
    SYS_STATE_RESET,            /* Reset/cleanup in progress */
    SYS_STATE_ERROR,            /* Error state */
    SYS_STATE_COUNT
} SystemState_t;

/**
 * State transition event.
 */
typedef enum {
    SYS_EVT_NONE = 0,
    SYS_EVT_MODE_PANO,
    SYS_EVT_MODE_CT,
    SYS_EVT_MODE_SCAN,
    SYS_EVT_MODE_CALIBRATION,
    SYS_EVT_MODE_GEO_ALIGN,
    SYS_EVT_MODE_EEPROM,
    SYS_EVT_MODE_DIAGNOSTIC,
    SYS_EVT_MODE_RESET,
    SYS_EVT_MODE_CANCEL,
    SYS_EVT_CAPTURE_DONE,
    SYS_EVT_ERROR,
    SYS_EVT_ERROR_CLEAR,
    SYS_EVT_EMERGENCY,
    SYS_EVT_COUNT
} SystemEvent_t;

/**
 * Initialize the state machine to IDLE.
 */
void StateMachine_Init(void);

/**
 * Get current state.
 */
SystemState_t StateMachine_GetState(void);

/**
 * Process an event and transition to the appropriate state.
 *
 * @param evt  Event to process
 * @return New state after transition
 */
SystemState_t StateMachine_ProcessEvent(SystemEvent_t evt);

/**
 * Check if the system is in any capture state.
 */
bool StateMachine_IsCapturing(void);

/**
 * Check if system is in error state.
 */
bool StateMachine_IsError(void);

#endif /* STATE_MACHINE_H */
