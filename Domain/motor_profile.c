/*
 * motor_profile.c : Motion profile generation (pure math)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Memory: ~500 bytes Flash, 0 bytes SRAM.
 */

#include "motor_profile.h"

/* ------------------------------------------------------------------ */
static inline uint16_t mp_clamp_ccr(uint32_t val)
{
    if (val < MP_CCR_MIN) {
        return MP_CCR_MIN;
    }
    if (val > MP_CCR_MAX) {
        return MP_CCR_MAX;
    }
    return (uint16_t)val;
}

/* ------------------------------------------------------------------ */
uint16_t MotorProfile_FreqToCCR(uint32_t freq_hz)
{
    uint32_t ccr;

    if (freq_hz < MP_MIN_FREQ_HZ) {
        freq_hz = MP_MIN_FREQ_HZ;
    }

    ccr = MP_TIM_CLK / (MP_TIM_TOGGLE * freq_hz);
    return mp_clamp_ccr(ccr);
}

/* ------------------------------------------------------------------ */
int32_t MotorProfile_BuildTrap(uint16_t* buf,
                                uint32_t buf_size,
                                int32_t steps,
                                uint32_t max_freq_hz,
                                uint32_t accel_hz_per_s,
                                MotorProfileInfo_t* info)
{
    uint32_t total;
    uint32_t start_freq;
    uint32_t accel_steps;
    uint32_t decel_steps;
    uint32_t cruise_steps;
    uint32_t start_fp;
    uint32_t max_fp;
    uint32_t delta_fp;
    uint32_t freq_fp;
    uint32_t freq_int;
    uint32_t ccr_val;
    uint32_t i;

    if (buf == (void*)0) {
        return -1;
    }
    if (steps <= 0 || (uint32_t)steps > buf_size) {
        return -1;
    }
    if (max_freq_hz < MP_MIN_FREQ_HZ || accel_hz_per_s == 0U) {
        return -1;
    }

    total = (uint32_t)steps;
    start_freq = MP_TRAP_START_FREQ_HZ;

    if (max_freq_hz <= start_freq) {
        max_freq_hz = start_freq + 1U;
    }

    /* Q16.16 fixed-point frequency range */
    start_fp = MP_FP_FROM_INT(start_freq);
    max_fp   = MP_FP_FROM_INT(max_freq_hz);

    /* Compute acceleration steps using average frequency approximation */
    {
        uint32_t freq_range = max_freq_hz - start_freq;
        uint64_t avg_freq = ((uint64_t)start_freq + (uint64_t)max_freq_hz) / 2U;
        uint64_t accel_steps_64 = ((uint64_t)freq_range * avg_freq)
                                  / (uint64_t)accel_hz_per_s;

        if (accel_steps_64 > total / 2U) {
            accel_steps_64 = total / 2U;
        }
        accel_steps = (uint32_t)accel_steps_64;
    }

    if (accel_steps == 0U) {
        accel_steps = 1U;
    }

    decel_steps = accel_steps;

    /* Handle triangular profile */
    if (accel_steps + decel_steps >= total) {
        accel_steps = total / 2U;
        decel_steps = total - accel_steps;
        cruise_steps = 0U;
    } else {
        cruise_steps = total - accel_steps - decel_steps;
    }

    /* Frequency increment per step (Q16.16) */
    if (accel_steps > 0U) {
        delta_fp = (max_fp - start_fp) / accel_steps;
    } else {
        delta_fp = 0U;
    }

    /* Acceleration phase */
    freq_fp = start_fp;
    for (i = 0; i < accel_steps; i++) {
        freq_int = MP_FP_TO_INT(freq_fp);
        if (freq_int < MP_MIN_FREQ_HZ) {
            freq_int = MP_MIN_FREQ_HZ;
        }
        ccr_val = MP_TIM_CLK / (MP_TIM_TOGGLE * freq_int);
        buf[i] = mp_clamp_ccr(ccr_val);
        freq_fp += delta_fp;
    }

    /* Cruise phase */
    {
        uint16_t cruise_ccr;
        freq_int = MP_FP_TO_INT(freq_fp);
        if (freq_int < MP_MIN_FREQ_HZ) {
            freq_int = MP_MIN_FREQ_HZ;
        }
        ccr_val = MP_TIM_CLK / (MP_TIM_TOGGLE * freq_int);
        cruise_ccr = mp_clamp_ccr(ccr_val);

        for (i = accel_steps; i < accel_steps + cruise_steps; i++) {
            buf[i] = cruise_ccr;
        }
    }

    /* Deceleration phase (mirror of acceleration) */
    freq_fp = max_fp;
    for (i = 0; i < decel_steps; i++) {
        freq_int = MP_FP_TO_INT(freq_fp);
        if (freq_int < MP_MIN_FREQ_HZ) {
            freq_int = MP_MIN_FREQ_HZ;
        }
        ccr_val = MP_TIM_CLK / (MP_TIM_TOGGLE * freq_int);
        buf[accel_steps + cruise_steps + i] = mp_clamp_ccr(ccr_val);
        freq_fp -= delta_fp;
        if (freq_fp < start_fp) {
            freq_fp = start_fp;
        }
    }

    /* Fill output metadata */
    if (info != (void*)0) {
        info->total_steps  = total;
        info->accel_steps  = accel_steps;
        info->cruise_steps = cruise_steps;
        info->decel_steps  = decel_steps;
    }

    return 0;
}

/* ------------------------------------------------------------------ */
int32_t MotorProfile_BuildArch(uint16_t* buf,
                                uint32_t buf_size,
                                const uint16_t* ccr_array,
                                uint32_t array_size)
{
    uint32_t i;

    if (buf == (void*)0 || ccr_array == (void*)0) {
        return -1;
    }
    if (array_size == 0U || array_size > buf_size) {
        return -1;
    }

    for (i = 0; i < array_size; i++) {
        buf[i] = mp_clamp_ccr((uint32_t)ccr_array[i]);
    }

    return 0;
}
