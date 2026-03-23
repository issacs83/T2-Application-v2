/*
 * scan_sequence.h : Non-blocking cephalometric scan state machine
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Replaces the blocking ScanCapture() function with a step-based
 * state machine that returns within ~100us per call.
 *
 * Memory: see .c for details.
 */

#ifndef SCAN_SEQUENCE_H
#define SCAN_SEQUENCE_H

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/* Scan sub-states                                                     */
/* ------------------------------------------------------------------ */
typedef enum {
    SCAN_IDLE = 0,
    SCAN_CHECK_SYSTEM,          /* checkPreCaptureStatus */
    SCAN_CAN_COMM_CHECK,        /* generator comm check */
    SCAN_COLLIMATOR_TILT_OFF,   /* tilt off */
    SCAN_CLEAR_PARAMS,          /* clear parameters */
    SCAN_MOVE_TO_INIT,          /* move all motors to init */
    SCAN_COLLIMATOR_TILT_ON,    /* tilt on for ceph */
    SCAN_LASER_ON,              /* enable lasers, send READY */
    SCAN_WAIT_PARAMS,           /* wait for PC ceph params */
    SCAN_MOTOR_START_POS,       /* ceph start position */
    SCAN_LASER_OFF,             /* lasers off, check exp sw */
    SCAN_COLLIMATOR_SETUP,      /* open, cephscan init, filter */
    SCAN_MODE_SETUP,            /* mode-specific collimator */
    SCAN_TUBE_SETUP,            /* tube count, mode, KV, mA */
    SCAN_WAIT_2SEC_CONFIRM,     /* 2-sec mode PC confirm */
    SCAN_EXPOSURE_SETUP,        /* exposure LED, indicators */
    SCAN_WAIT_EXPOSURE_SW,      /* wait for exposure switch */
    SCAN_XRAY_INTERLOCK,        /* verify safety interlock */
    SCAN_TUBE_READY,            /* enable tube ready */
    SCAN_TUBE_HEAT_WAIT,        /* wait 2s for tube heating */
    SCAN_START_CAPTURE,         /* start collimator + motor */
    SCAN_EXPOSING,              /* monitoring motor steps */
    SCAN_WAIT_END,              /* wait motors stop */
    SCAN_COMPLETE,              /* done, cleanup */
    SCAN_STOP,                  /* stopped by user */
    SCAN_CANCEL,                /* cancel cleanup */
    SCAN_ERROR,                 /* error state */
    SCAN_STATE_COUNT
} ScanState_t;

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

/**
 * Initialize the scan sequence module.
 */
void ScanSequence_Init(void);

/**
 * Begin a cephalometric scan sequence.
 */
void ScanSequence_Start(void);

/**
 * Execute one step of the scan sequence.
 * Called every main loop iteration when ST_SCAN_SEQUENCE is active.
 */
void ScanSequence_Step(void);

/**
 * Cancel the scan sequence from any state.
 */
void ScanSequence_Cancel(void);

/**
 * Check if the scan sequence has completed or been cancelled.
 */
bool ScanSequence_IsDone(void);

/**
 * Get current scan sub-state (for debugging).
 */
ScanState_t ScanSequence_GetState(void);

/**
 * Legacy compatibility: blocking run.
 */
void ScanSequence_Run(void);

#endif /* SCAN_SEQUENCE_H */
