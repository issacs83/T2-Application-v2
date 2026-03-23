/*
 * motor_sequence.h : Non-blocking motor wait helper
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Replaces while(!Motor_GetStatus(..., STATUS_STOP)) blocking loops
 * with a poll-based check that returns immediately.
 *
 * Usage:
 *   MotorWait_t mw;
 *   MotorWait_Init(&mw, 5000);       // 5 second timeout
 *   MotorWait_AddByPtr(&mw, &Motor_R);
 *   MotorWait_AddByPtr(&mw, &Motor_V);
 *   MotorWait_Start(&mw);
 *
 *   // In main loop step function:
 *   int8_t result = MotorWait_Check(&mw);
 *   if (result == 1) { ... all done ... }
 *   if (result == -1) { ... timeout ... }
 *   // result == 0 means still waiting, return and check next iteration
 *
 * Memory: ~80 bytes SRAM per instance.
 */

#ifndef MOTOR_SEQUENCE_H
#define MOTOR_SEQUENCE_H

#include <stdint.h>
#include <stdbool.h>
#include "motor.h"

/* Maximum number of motors that can be waited on simultaneously */
#define MOTOR_WAIT_MAX_COUNT    8U

/**
 * Motor wait context.
 * Tracks a set of motors and polls their status each iteration.
 */
typedef struct {
    Motor_Typedef* motors[MOTOR_WAIT_MAX_COUNT];
    uint8_t   count;
    uint32_t  timeout_ms;
    uint32_t  start_ms;
    bool      started;
} MotorWait_t;

/**
 * Initialize a motor wait context.
 *
 * @param mw          Motor wait context
 * @param timeout_ms  Maximum wait time in milliseconds (0 = no timeout)
 */
void MotorWait_Init(MotorWait_t* mw, uint32_t timeout_ms);

/**
 * Add a motor to the wait list by pointer to Motor_Typedef.
 *
 * @param mw     Motor wait context
 * @param motor  Pointer to Motor_Typedef instance
 * @return true if added, false if list is full
 */
bool MotorWait_AddByPtr(MotorWait_t* mw, Motor_Typedef* motor);

/**
 * Mark the wait as started (captures start timestamp).
 * Call this after all motors have been commanded to move.
 *
 * @param mw  Motor wait context
 */
void MotorWait_Start(MotorWait_t* mw);

/**
 * Poll all motors in the wait list.
 * Must be called every main loop iteration.
 *
 * @param mw  Motor wait context
 * @return  0 = still waiting
 *          1 = all motors stopped
 *         -1 = timeout
 *         -2 = not started
 */
int8_t MotorWait_Check(MotorWait_t* mw);

/**
 * Reset the wait context for reuse.
 *
 * @param mw  Motor wait context
 */
void MotorWait_Reset(MotorWait_t* mw);

/**
 * Check if a single legacy motor has stopped.
 * Non-blocking replacement for while(!Motor_GetStatus(&m, STATUS_STOP)).
 *
 * @param motor  Pointer to Motor_Typedef instance
 * @return true if motor is stopped
 */
bool MotorWait_IsStopped(Motor_Typedef* motor);

#endif /* MOTOR_SEQUENCE_H */
