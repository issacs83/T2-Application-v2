/*
 * state_machine.c : System state machine implementation
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * TODO: This module is scaffolding for the target architecture migration.
 *       Currently not called from the main loop (App_Run uses legacy
 *       CurCaptureMode dispatch). Will replace the switch/case dispatch
 *       when migration to event-driven state machine is complete.
 *       Do NOT delete — represents the target architecture.
 *
 * Memory: ~300 bytes Flash, 4 bytes SRAM.
 */

#include "state_machine.h"

/* Private variables */
static volatile SystemState_t g_sys_state = SYS_STATE_IDLE;

/* ------------------------------------------------------------------ */
void StateMachine_Init(void)
{
    g_sys_state = SYS_STATE_IDLE;
}

/* ------------------------------------------------------------------ */
SystemState_t StateMachine_GetState(void)
{
    return g_sys_state;
}

/* ------------------------------------------------------------------ */
SystemState_t StateMachine_ProcessEvent(SystemEvent_t evt)
{
    switch (evt) {
    case SYS_EVT_MODE_PANO:
        g_sys_state = SYS_STATE_PANO;
        break;

    case SYS_EVT_MODE_CT:
        g_sys_state = SYS_STATE_CT;
        break;

    case SYS_EVT_MODE_SCAN:
        g_sys_state = SYS_STATE_SCAN;
        break;

    case SYS_EVT_MODE_CALIBRATION:
        g_sys_state = SYS_STATE_CALIBRATION;
        break;

    case SYS_EVT_MODE_GEO_ALIGN:
        g_sys_state = SYS_STATE_GEO_ALIGN;
        break;

    case SYS_EVT_MODE_EEPROM:
        g_sys_state = SYS_STATE_EEPROM;
        break;

    case SYS_EVT_MODE_DIAGNOSTIC:
        g_sys_state = SYS_STATE_DIAGNOSTIC;
        break;

    case SYS_EVT_MODE_RESET:
        g_sys_state = SYS_STATE_RESET;
        break;

    case SYS_EVT_MODE_CANCEL:
    case SYS_EVT_CAPTURE_DONE:
        g_sys_state = SYS_STATE_IDLE;
        break;

    case SYS_EVT_ERROR:
    case SYS_EVT_EMERGENCY:
        g_sys_state = SYS_STATE_ERROR;
        break;

    case SYS_EVT_ERROR_CLEAR:
        if (g_sys_state == SYS_STATE_ERROR) {
            g_sys_state = SYS_STATE_IDLE;
        }
        break;

    case SYS_EVT_NONE:
    default:
        break;
    }

    return g_sys_state;
}

/* ------------------------------------------------------------------ */
bool StateMachine_IsCapturing(void)
{
    return (g_sys_state == SYS_STATE_PANO ||
            g_sys_state == SYS_STATE_CT ||
            g_sys_state == SYS_STATE_SCAN);
}

/* ------------------------------------------------------------------ */
bool StateMachine_IsError(void)
{
    return (g_sys_state == SYS_STATE_ERROR);
}
