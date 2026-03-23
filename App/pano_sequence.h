/*
 * pano_sequence.h : Non-blocking panoramic capture state machine
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Replaces the blocking PanoCapture() function with a step-based
 * state machine that returns within ~100us per call.
 *
 * Memory: see .c for details.
 */

#ifndef PANO_SEQUENCE_H
#define PANO_SEQUENCE_H

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/* Pano sub-states                                                     */
/* ------------------------------------------------------------------ */
typedef enum {
    PANO_IDLE = 0,
    PANO_CHECK_SYSTEM,          /* checkPreCaptureStatus */
    PANO_CAN_COMM_CHECK,        /* generator/collimator comm check */
    PANO_COLLIMATOR_TILT_OFF,   /* tilt off */
    PANO_CLEAR_PARAMS,          /* clear parameters, load arch */
    PANO_SENSOR_CONFIG,         /* pano sensor config + start */
    PANO_MOVE_TO_INIT,          /* move all motors to init pos */
    PANO_WAIT_INIT_MOTORS,      /* wait for motors to reach init */
    PANO_TEMPLE_SUPPORT,        /* temple support positioning */
    PANO_LASER_ON,              /* enable lasers, send READY */
    PANO_WAIT_PARAMS,           /* wait for PC to send params */
    PANO_LASER_OFF,             /* lasers off, check exposure sw */
    PANO_COLLIMATOR_SETUP,      /* FOV set, collimator position */
    PANO_SENSOR_SYNC,           /* sensor sync setup */
    PANO_TUBE_SETUP,            /* tube count, mode, KV, mA */
    PANO_START_MOTORS,          /* start R+V arch motion */
    PANO_WAIT_EXPOSURE_SW,      /* wait for exposure switch press */
    PANO_XRAY_INTERLOCK,        /* verify safety interlock */
    PANO_TUBE_READY,            /* enable tube ready */
    PANO_TUBE_HEAT_WAIT,        /* wait 2s for tube heating */
    PANO_START_CAPTURE,         /* start pano motor + sensor */
    PANO_EXPOSING,              /* X-ray ON, monitoring arch index */
    PANO_WAIT_END,              /* wait for motors to stop */
    PANO_MOVE_TO_END,           /* move to end position */
    PANO_WAIT_END_MOTORS,       /* wait for end position motors */
    PANO_COMPLETE,              /* done, cleanup */
    PANO_STOP,                  /* stopped by user */
    PANO_CANCEL,                /* cancel cleanup */
    PANO_ERROR,                 /* error state */
    PANO_STATE_COUNT
} PanoState_t;

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

/**
 * Initialize the pano sequence module.
 * Sets internal state to PANO_IDLE.
 */
void PanoSequence_Init(void);

/**
 * Begin a panoramic capture sequence.
 * Transitions from PANO_IDLE to PANO_CHECK_SYSTEM.
 */
void PanoSequence_Start(void);

/**
 * Execute one step of the pano sequence.
 * Called every main loop iteration when ST_PANO_SEQUENCE is active.
 * Must return within ~100us.
 */
void PanoSequence_Step(void);

/**
 * Cancel the pano sequence from any state.
 * Safely shuts down X-ray, motors, sensors.
 */
void PanoSequence_Cancel(void);

/**
 * Check if the pano sequence has completed or been cancelled.
 *
 * @return true if sequence is done (PANO_IDLE)
 */
bool PanoSequence_IsDone(void);

/**
 * Get current pano sub-state (for debugging).
 */
PanoState_t PanoSequence_GetState(void);

/**
 * Legacy compatibility: blocking run (calls Start + Step loop).
 * Will be removed after full migration.
 */
void PanoSequence_Run(void);

#endif /* PANO_SEQUENCE_H */
