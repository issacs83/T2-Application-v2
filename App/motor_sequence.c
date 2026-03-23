/*
 * motor_sequence.c : Non-blocking motor wait helper implementation
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Provides MotorWait_Check() which polls motor status without blocking.
 * Uses legacy Motor_GetStatus() for backward compatibility.
 *
 * Memory: ~300 bytes Flash, 0 bytes static SRAM (per-instance in caller).
 */

#include "motor_sequence.h"
#include "motor.h"
#include "timer.h"

/* ------------------------------------------------------------------ */
void MotorWait_Init(MotorWait_t* mw, uint32_t timeout_ms)
{
    uint8_t i;
    for (i = 0; i < MOTOR_WAIT_MAX_COUNT; i++) {
        mw->motors[i] = NULL;
    }
    mw->count = 0;
    mw->timeout_ms = timeout_ms;
    mw->start_ms = 0;
    mw->started = false;
}

/* ------------------------------------------------------------------ */
bool MotorWait_AddByPtr(MotorWait_t* mw, Motor_Typedef* motor)
{
    if (mw->count >= MOTOR_WAIT_MAX_COUNT) {
        return false;
    }
    if (motor == NULL) {
        return false;
    }
    mw->motors[mw->count] = motor;
    mw->count++;
    return true;
}

/* ------------------------------------------------------------------ */
void MotorWait_Start(MotorWait_t* mw)
{
    mw->start_ms = vSetCurrentMilliSec();
    mw->started = true;
}

/* ------------------------------------------------------------------ */
int8_t MotorWait_Check(MotorWait_t* mw)
{
    uint8_t i;
    uint32_t elapsed;

    if (!mw->started) {
        return -2;
    }

    /* Check timeout */
    if (mw->timeout_ms > 0) {
        elapsed = nElapsedMilliSec(mw->start_ms);
        if (elapsed >= mw->timeout_ms) {
            return -1;
        }
    }

    /* Check all motors */
    for (i = 0; i < mw->count; i++) {
        if (mw->motors[i] != NULL) {
            if (!Motor_GetStatus(mw->motors[i], STATUS_STOP)) {
                return 0; /* at least one motor still running */
            }
        }
    }

    return 1; /* all motors stopped */
}

/* ------------------------------------------------------------------ */
void MotorWait_Reset(MotorWait_t* mw)
{
    MotorWait_Init(mw, mw->timeout_ms);
}

/* ------------------------------------------------------------------ */
bool MotorWait_IsStopped(Motor_Typedef* motor)
{
    if (motor == NULL) {
        return true;
    }
    return Motor_GetStatus(motor, STATUS_STOP) ? true : false;
}
