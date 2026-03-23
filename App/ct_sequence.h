/*
 * ct_sequence.h : Non-blocking CT/CBCT capture state machine
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Replaces the blocking CtCapture() function with a step-based
 * state machine that returns within ~100us per call.
 *
 * Memory: see .c for details.
 */

#ifndef CT_SEQUENCE_H
#define CT_SEQUENCE_H

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/* CT sub-states                                                       */
/* ------------------------------------------------------------------ */
typedef enum {
    CT_IDLE = 0,
    CT_CHECK_SYSTEM,            /* checkPreCaptureStatus */
    CT_SENSOR_CONFIG,           /* CT sensor config (pulsed mode) */
    CT_COLLIMATOR_TILT_OFF,     /* tilt off */
    CT_CAN_COMM_CHECK,          /* generator comm check */
    CT_MOVE_TO_INIT,            /* move all motors to init pos */
    CT_CLEAR_PARAMS,            /* clear parameters */
    CT_LASER_ON,                /* enable lasers, send READY */
    CT_WAIT_PARAMS,             /* wait for PC to send params */
    CT_LASER_OFF,               /* lasers off, check exposure sw */
    CT_FOV_SETUP,               /* FOV collimator setup */
    CT_FILTER_SETUP,            /* filter positioning */
    CT_PULSED_MODE_SETUP,       /* pulsed mode timing config */
    CT_CALC_PARAMS,             /* calculate rotation params */
    CT_TUBE_SETUP,              /* tube count, mode, KV, mA */
    CT_MOTOR_START_POS,         /* move to start position */
    CT_STITCH_SETUP,            /* stitch positioning (15x15) */
    CT_WAIT_STITCH_CONFIRM,     /* wait 2nd capture confirm */
    CT_EXPOSURE_SETUP,          /* exposure LED, indicators */
    CT_WAIT_EXPOSURE_SW,        /* wait for exposure switch */
    CT_XRAY_INTERLOCK,          /* verify safety interlock */
    CT_TUBE_READY,              /* enable tube ready */
    CT_TUBE_HEAT_WAIT,          /* wait 2s for tube heating */
    CT_START_ROTATION,          /* start R-axis rotation */
    CT_EXPOSING,                /* X-ray ON, monitoring rotation */
    CT_WAIT_ROTATION_END,       /* wait for rotation motor stop */
    CT_STITCH_CHECK,            /* check if stitch 2nd capture */
    CT_MOVE_TO_END,             /* move to end position */
    CT_WAIT_END_MOTORS,         /* wait end position motors */
    CT_STITCH_BACK,             /* move stitch back position */
    CT_COMPLETE,                /* done, cleanup */
    CT_STOP,                    /* stopped by user */
    CT_CANCEL,                  /* cancel cleanup */
    CT_ERROR,                   /* error state */
    CT_STATE_COUNT
} CtState_t;

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

/**
 * Initialize the CT sequence module.
 */
void CtSequence_Init(void);

/**
 * Begin a CT capture sequence.
 */
void CtSequence_Start(void);

/**
 * Execute one step of the CT sequence.
 * Called every main loop iteration when ST_CT_SEQUENCE is active.
 */
void CtSequence_Step(void);

/**
 * Cancel the CT sequence from any state.
 */
void CtSequence_Cancel(void);

/**
 * Check if the CT sequence has completed or been cancelled.
 */
bool CtSequence_IsDone(void);

/**
 * Get current CT sub-state (for debugging).
 */
CtState_t CtSequence_GetState(void);

/**
 * Legacy compatibility: blocking run.
 */
void CtSequence_Run(void);

#endif /* CT_SEQUENCE_H */
