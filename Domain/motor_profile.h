/*
 * motor_profile.h : Buffer-based motion profile generation (pure math, no HW)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Generates trapezoidal and arch velocity profiles as CCR (timer reload)
 * value arrays. Zero hardware dependency - operates on caller-provided
 * buffers only.
 *
 * Fixed-point Q16.16 arithmetic for profile building (no floating point).
 *
 * Relationship with motion_profile.h:
 *   - motor_profile:  BUFFER-BASED, pre-computes all CCR values into an array.
 *                     Simple ISR (just reads buf[idx++]). Uses more SRAM.
 *   - motion_profile: STATE-MACHINE-BASED, computes CCR on-the-fly per ISR tick.
 *                     Adds S-curve, arch interpolation, homing. Uses ~64 bytes.
 *   Both modules are valid and can coexist. Use motor_profile for legacy
 *   compatibility with existing pre-computed arch tables. Use motion_profile
 *   for new S-curve motions and memory-constrained configurations.
 *
 * Memory: ~400 bytes Flash, 0 bytes SRAM.
 */

#ifndef MOTOR_PROFILE_H
#define MOTOR_PROFILE_H

#include <stdint.h>

/* Fixed-point Q16.16 helpers */
#define MP_FP_SHIFT                 16
#define MP_FP_ONE                   (1UL << MP_FP_SHIFT)
#define MP_FP_FROM_INT(x)           ((uint32_t)(x) << MP_FP_SHIFT)
#define MP_FP_TO_INT(x)             ((uint32_t)(x) >> MP_FP_SHIFT)

/* Timer clock constant (must match hardware timer configuration) */
#define MP_TIM_CLK                  1250000U
#define MP_TIM_TOGGLE               2U

/* CCR value limits */
#define MP_CCR_MAX                  0xFFFFU
#define MP_CCR_MIN                  10U

/* Default start frequency for trapezoidal ramp (Hz) */
#define MP_TRAP_START_FREQ_HZ       500U

/* Minimum allowed frequency */
#define MP_MIN_FREQ_HZ              10U

/* Maximum profile buffer size */
#define MP_PROFILE_BUF_SIZE         4096U

/**
 * Profile metadata returned after building.
 */
typedef struct {
    uint32_t total_steps;       /* Number of valid CCR entries */
    uint32_t accel_steps;       /* Steps in acceleration phase */
    uint32_t cruise_steps;      /* Steps in cruise phase */
    uint32_t decel_steps;       /* Steps in deceleration phase */
} MotorProfileInfo_t;

/**
 * Build trapezoidal velocity profile (integer-only math).
 *
 * @param buf           Output CCR buffer (caller-allocated, >= steps entries)
 * @param buf_size      Maximum buffer capacity
 * @param steps         Total step count (must be > 0, <= buf_size)
 * @param max_freq_hz   Cruise frequency (Hz)
 * @param accel_hz_per_s Acceleration rate (Hz per second)
 * @param info          Optional output metadata (can be NULL)
 * @return 0 on success, -1 on invalid parameters
 */
int32_t MotorProfile_BuildTrap(uint16_t* buf,
                                uint32_t buf_size,
                                int32_t steps,
                                uint32_t max_freq_hz,
                                uint32_t accel_hz_per_s,
                                MotorProfileInfo_t* info);

/**
 * Build arch profile from pre-computed CCR array (copy with clamping).
 *
 * @param buf           Output CCR buffer
 * @param buf_size      Maximum buffer capacity
 * @param ccr_array     Source CCR values
 * @param array_size    Number of source entries
 * @return 0 on success, -1 on overflow
 */
int32_t MotorProfile_BuildArch(uint16_t* buf,
                                uint32_t buf_size,
                                const uint16_t* ccr_array,
                                uint32_t array_size);

/**
 * Convert frequency in Hz to CCR (timer reload value).
 * CCR = TIM_CLK / (TOGGLE * freq_hz)
 *
 * @param freq_hz  Frequency in Hz (must be >= MP_MIN_FREQ_HZ)
 * @return CCR value clamped to [MP_CCR_MIN, MP_CCR_MAX]
 */
uint16_t MotorProfile_FreqToCCR(uint32_t freq_hz);

#endif /* MOTOR_PROFILE_H */
