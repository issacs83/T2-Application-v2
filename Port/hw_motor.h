/*
 * hw_motor.h : Timer + GPIO + DMA for motor step pulse with MotorInterface_t
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Provides the hardware abstraction for motor control.
 * Uses motor_ctrl.c for timer/GPIO init and ISR handling.
 * Exposes MotorInterface_t for App layer consumption.
 */

#ifndef HW_MOTOR_H
#define HW_MOTOR_H

#include <stdint.h>
#include <stdbool.h>

/* Callback type for motor completion notification */
typedef void (*MotorDoneCallback_t)(uint8_t motor_id);

/**
 * Motor hardware interface (function pointer vtable).
 */
typedef struct {
    void  (*Init)(void);
    void  (*Start)(uint8_t id);
    void  (*Stop)(uint8_t id);
    void  (*EmergencyStopAll)(void);
    bool  (*IsRunning)(uint8_t id);
    int32_t (*GetPosition)(uint8_t id);
    void  (*SetDoneCallback)(MotorDoneCallback_t cb);
} MotorInterface_t;

/* Global motor interface instance */
extern MotorInterface_t Motor;

/* Initialize motor hardware (timers, GPIO, TMC2660) */
void HwMotor_Init(void);

/**
 * Build trapezoidal profile for a motor.
 * @param id            Motor index (0-7)
 * @param steps         Total steps (negative = toward ORG)
 * @param max_freq_hz   Cruise frequency
 * @param accel_hz_per_s Acceleration rate
 * @return 0 on success
 */
int32_t HwMotor_BuildTrapProfile(uint8_t id, int32_t steps,
                                   uint32_t max_freq_hz,
                                   uint32_t accel_hz_per_s);

/**
 * Build arch profile from pre-computed CCR array.
 * @param id          Motor index
 * @param ccr_array   CCR value array
 * @param array_size  Number of entries
 * @return 0 on success
 */
int32_t HwMotor_BuildArchProfile(uint8_t id, const uint16_t* ccr_array,
                                   uint32_t array_size);

/**
 * Set motor direction.
 * @param id   Motor index
 * @param dir  0 = ORG, 1 = LIMIT
 */
void HwMotor_SetDirection(uint8_t id, uint8_t dir);

/**
 * Set motor position (after homing).
 */
void HwMotor_SetPosition(uint8_t id, int32_t pos);

/**
 * Start multiple motors synchronously.
 */
void HwMotor_StartSync(const uint8_t* ids, uint8_t count);

/**
 * TMC2660 driver control functions.
 */
void HwMotor_TMC2660_Init(uint8_t id);
void HwMotor_TMC2660_SetCurrent(uint8_t id, uint16_t run, uint16_t hold);
void HwMotor_TMC2660_SetMicrostep(uint8_t id, uint8_t resolution);

#endif /* HW_MOTOR_H */
