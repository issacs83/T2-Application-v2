/*
 * motion_profile.c : Real-time S-curve / trapezoidal motion profile
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Pure integer arithmetic, zero float/double, zero hardware dependency.
 * Designed for Cortex-M ISR context (~40-60 cycles per tick).
 *
 * Memory: ~1400 bytes Flash, 0 bytes static SRAM (all in caller structs).
 */

#include "motion_profile.h"

/* ================================================================== */
/* Internal helpers                                                   */
/* ================================================================== */

/*
 * Clamp CCR to valid range. ~3 cycles.
 */
static inline uint16_t clamp_ccr(uint32_t val)
{
    if (val < CCR_MIN) {
        return (uint16_t)CCR_MIN;
    }
    if (val > CCR_MAX) {
        return (uint16_t)CCR_MAX;
    }
    return (uint16_t)val;
}

/*
 * Clamp ufp32_t frequency to [start_freq_fp, max_freq_fp]. ~4 cycles.
 */
static inline ufp32_t clamp_freq(ufp32_t f, ufp32_t lo, ufp32_t hi)
{
    if (f < lo) {
        return lo;
    }
    if (f > hi) {
        return hi;
    }
    return f;
}

/*
 * Integer square root (Cortex-M optimized, ~30 cycles).
 * Used for triangle profile calculation.
 */
static uint32_t isqrt32(uint32_t n)
{
    uint32_t x;
    uint32_t x1;

    if (n == 0U) {
        return 0U;
    }

    /* Initial estimate via bit scan */
    x = n;
    x1 = 1U;
    if (x >= 0x10000U) { x >>= 16; x1 <<= 8; }
    if (x >= 0x100U)   { x >>= 8;  x1 <<= 4; }
    if (x >= 0x10U)    { x >>= 4;  x1 <<= 2; }
    if (x >= 0x4U)     { x1 <<= 1; }

    /* Newton iterations (3 are enough for 32-bit) */
    x = x1;
    x = (x + n / x) >> 1;
    x = (x + n / x) >> 1;
    x = (x + n / x) >> 1;

    /* Final correction */
    if (x * x > n) {
        x--;
    }
    return x;
}

/* ================================================================== */
/* S-Curve Planner                                                    */
/* ================================================================== */

/*
 * Compute steps needed for one jerk phase at given jerk rate.
 * During jerk phase: accel ramps linearly, velocity ramps quadratically.
 * Steps_jerk = a_max / jerk (in step-domain terms).
 *
 * More precisely, with jerk applied per step:
 *   n_j = sqrt(2 * delta_v_jerk / jerk_per_step)
 *   where delta_v_jerk = a_max^2 / (2 * jerk)
 *
 * Simplified: n_j = a_max / jerk_per_step
 * But jerk_per_step depends on current freq, so we approximate:
 *   n_j = (a_max * avg_freq) / jerk
 *
 * For planning, we use: n_j = accel / (jerk / avg_freq)
 * ~20 cycles
 */
static uint32_t calc_jerk_steps(uint32_t accel, uint32_t jerk,
                                 uint32_t avg_freq)
{
    uint64_t num;
    uint32_t result;

    if (jerk == 0U || avg_freq == 0U) {
        return 0U;
    }

    num = (uint64_t)accel * (uint64_t)avg_freq;
    result = (uint32_t)(num / (uint64_t)jerk);

    if (result == 0U) {
        result = 1U;
    }
    return result;
}

/* ------------------------------------------------------------------ */
int32_t MotionProfile_Plan(ProfileState_t *state, const ProfileParams_t *params)
{
    uint32_t max_f;
    uint32_t start_f;
    uint32_t accel;
    uint32_t jerk;
    uint32_t total;
    uint32_t freq_range;
    uint32_t avg_freq;
    uint32_t jerk_steps;
    uint32_t accel_steps;
    uint32_t ramp_steps;
    uint32_t cruise_steps;
    uint32_t half_steps;
    uint32_t i;

    if (state == (void *)0 || params == (void *)0) {
        return -1;
    }

    max_f   = params->max_freq;
    start_f = params->start_freq;
    accel   = params->accel;
    jerk    = params->jerk;
    total   = params->total_steps;

    /* Validate parameters */
    if (total == 0U || max_f < FREQ_MIN_HZ || accel == 0U) {
        return -1;
    }
    if (start_f < FREQ_MIN_HZ) {
        start_f = FREQ_MIN_HZ;
    }
    if (max_f <= start_f) {
        max_f = start_f + 1U;
    }

    freq_range = max_f - start_f;
    avg_freq = (start_f + max_f) / 2U;

    /* Zero all phase lengths */
    for (i = 0U; i < 7U; i++) {
        state->phase_len[i] = 0U;
    }

    if (jerk == 0U) {
        /*
         * Trapezoidal fallback: no jerk phases (J1=J2=J3=J4=0).
         * Accel steps = freq_range * avg_freq / accel
         * ~15 cycles
         */
        uint64_t accel_64 = ((uint64_t)freq_range * (uint64_t)avg_freq)
                            / (uint64_t)accel;

        if (accel_64 > total / 2U) {
            accel_64 = total / 2U;
        }
        accel_steps = (uint32_t)accel_64;
        if (accel_steps == 0U) {
            accel_steps = 1U;
        }

        /* Symmetric deceleration */
        ramp_steps = accel_steps * 2U;

        if (ramp_steps >= total) {
            /* Triangle profile */
            accel_steps = total / 2U;
            if (accel_steps == 0U) {
                accel_steps = 1U;
            }
            cruise_steps = total - accel_steps * 2U;
        } else {
            cruise_steps = total - ramp_steps;
        }

        /* Phase lengths: only A (accel), CRUISE, D (decel) */
        state->phase_len[0] = 0U;              /* J1 = 0 */
        state->phase_len[1] = accel_steps;      /* A      */
        state->phase_len[2] = 0U;              /* J2 = 0 */
        state->phase_len[3] = cruise_steps;     /* CRUISE */
        state->phase_len[4] = 0U;              /* J3 = 0 */
        state->phase_len[5] = accel_steps;      /* D      */
        state->phase_len[6] = 0U;              /* J4 = 0 */

        /* Compute per-step accel increment (Q16.16) */
        state->jerk_fp = 0;
        state->accel_inc = 0;
        if (accel_steps > 0U) {
            state->accel_fp = fp_div(FP_FROM_INT((int32_t)freq_range),
                                     FP_FROM_INT((int32_t)accel_steps));
        } else {
            state->accel_fp = 0;
        }
    } else {
        /*
         * S-curve: 7-segment profile.
         * J1: accel ramps 0 -> a_max       (jerk_steps steps)
         * A:  accel = a_max constant        (accel_steps steps)
         * J2: accel ramps a_max -> 0        (jerk_steps steps)
         * CRUISE: v = v_max                 (cruise_steps steps)
         * J3: accel ramps 0 -> -a_max       (jerk_steps steps)
         * D:  accel = -a_max constant       (accel_steps steps)
         * J4: accel ramps -a_max -> 0       (jerk_steps steps)
         * ~40 cycles
         */
        jerk_steps = calc_jerk_steps(accel, jerk, avg_freq);

        /* Velocity gained during one jerk phase (quadratic) */
        /* v_jerk = jerk_steps * accel_max / 2 (in step domain) */
        /* Velocity gained during accel phase (linear) */
        /* We need: 2*v_jerk + v_accel = freq_range */
        /* v_jerk ≈ jerk_steps * accel_per_step / 2 */

        /* Compute accel steps for the constant-accel phase */
        {
            uint64_t v_jerk_steps;
            uint64_t remaining_v;

            /* Velocity gained in one jerk phase ≈ jerk_steps^2 * jerk / (2 * avg_freq) */
            v_jerk_steps = ((uint64_t)jerk_steps * (uint64_t)jerk_steps
                           * (uint64_t)jerk) / ((uint64_t)avg_freq * 2U);

            if (v_jerk_steps * 2U >= freq_range) {
                /* Jerk phases alone cover the full velocity range */
                /* Reduce jerk_steps and eliminate constant accel phase */
                jerk_steps = isqrt32((uint32_t)(
                    ((uint64_t)freq_range * (uint64_t)avg_freq)
                    / (uint64_t)jerk));
                if (jerk_steps == 0U) {
                    jerk_steps = 1U;
                }
                accel_steps = 0U;
            } else {
                remaining_v = (uint64_t)freq_range - v_jerk_steps * 2U;
                /* accel_steps = remaining_v * avg_freq / accel */
                accel_steps = (uint32_t)(
                    (remaining_v * (uint64_t)avg_freq) / (uint64_t)accel);
            }
        }

        /* Total ramp = 2*(jerk_steps + accel_steps + jerk_steps) */
        /* = 4*jerk_steps + 2*accel_steps */
        ramp_steps = 4U * jerk_steps + 2U * accel_steps;

        if (ramp_steps >= total) {
            /* Triangle: scale down proportionally */
            half_steps = total / 2U;
            if (half_steps == 0U) {
                half_steps = 1U;
            }

            if (accel_steps == 0U) {
                /* Pure jerk triangle */
                jerk_steps = half_steps / 2U;
                if (jerk_steps == 0U) {
                    jerk_steps = 1U;
                }
                accel_steps = 0U;
            } else {
                /* Scale both proportionally */
                uint32_t one_ramp = 2U * jerk_steps + accel_steps;
                if (one_ramp > 0U) {
                    uint32_t scale = half_steps;
                    jerk_steps = (jerk_steps * scale) / one_ramp;
                    accel_steps = (accel_steps * scale) / one_ramp;
                    if (jerk_steps == 0U) {
                        jerk_steps = 1U;
                    }
                }
            }
            cruise_steps = 0U;

            /* Recalculate total to match */
            ramp_steps = 4U * jerk_steps + 2U * accel_steps;
            if (ramp_steps < total) {
                cruise_steps = total - ramp_steps;
            }
        } else {
            cruise_steps = total - ramp_steps;
        }

        /* Phase lengths (symmetric accel/decel) */
        state->phase_len[0] = jerk_steps;       /* J1 */
        state->phase_len[1] = accel_steps;       /* A  */
        state->phase_len[2] = jerk_steps;        /* J2 */
        state->phase_len[3] = cruise_steps;      /* CRUISE */
        state->phase_len[4] = jerk_steps;        /* J3 */
        state->phase_len[5] = accel_steps;       /* D  */
        state->phase_len[6] = jerk_steps;        /* J4 */

        /* Compute per-step jerk increment (Q16.16) */
        /* accel_inc_per_step during J1 = max_accel / jerk_steps */
        if (jerk_steps > 0U) {
            /* max_accel in per-step terms = freq_range / (2*jerk_steps + accel_steps) */
            uint32_t one_ramp = 2U * jerk_steps + accel_steps;
            if (one_ramp == 0U) {
                one_ramp = 1U;
            }
            state->max_accel_fp = fp_div(FP_FROM_INT((int32_t)freq_range),
                                         FP_FROM_INT((int32_t)one_ramp));
            state->jerk_fp = fp_div(state->max_accel_fp,
                                    FP_FROM_INT((int32_t)jerk_steps));
        } else {
            state->max_accel_fp = 0;
            state->jerk_fp = 0;
        }

        state->accel_inc = 0;
        state->accel_fp = 0;
    }

    /* Initialize runtime state */
    state->freq_fp      = UFP_FROM_INT(start_f);
    state->max_freq_fp  = UFP_FROM_INT(max_f);
    state->start_freq_fp = UFP_FROM_INT(start_f);
    state->phase        = PHASE_J1;
    state->phase_step   = 0U;
    state->steps_done   = 0U;
    state->steps_total  = total;
    state->active       = 1U;

    /* Skip empty leading phases */
    while (state->phase < PHASE_DONE &&
           state->phase_len[state->phase] == 0U) {
        state->phase = (ProfilePhase_t)(state->phase + 1);
    }

    return 0;
}

/* ================================================================== */
/* ISR Tick: Next CCR                                                 */
/* ================================================================== */

/* ~40-60 cycles per call */
uint16_t MotionProfile_NextCCR(ProfileState_t *state)
{
    uint16_t ccr;

    if (state->active == 0U || state->phase == PHASE_DONE) {
        return 0U;
    }

    /* Convert current frequency to CCR */
    ccr = fp_freq_to_ccr(state->freq_fp);

    /* Advance step counter */
    state->steps_done++;
    state->phase_step++;

    /* Check phase transition */
    if (state->phase_step >= state->phase_len[state->phase]) {
        /* Move to next non-empty phase */
        state->phase_step = 0U;
        state->phase = (ProfilePhase_t)(state->phase + 1);

        while (state->phase < PHASE_DONE &&
               state->phase_len[state->phase] == 0U) {
            state->phase = (ProfilePhase_t)(state->phase + 1);
        }

        /* Reset accel at phase boundaries for S-curve */
        switch (state->phase) {
        case PHASE_J1:
            state->accel_fp = 0;
            break;
        case PHASE_A:
            /* accel_fp should be at max_accel from J1 */
            break;
        case PHASE_J2:
            /* accel starts decreasing from max_accel */
            break;
        case PHASE_CRUISE:
            state->accel_fp = 0;
            state->freq_fp = state->max_freq_fp;
            break;
        case PHASE_J3:
            state->accel_fp = 0;
            break;
        case PHASE_D:
            /* accel_fp should be at -max_accel from J3 */
            break;
        case PHASE_J4:
            /* accel starts increasing from -max_accel toward 0 */
            break;
        case PHASE_DONE:
            state->active = 0U;
            return ccr;
        default:
            state->active = 0U;
            return 0U;
        }
    }

    /* Update frequency based on current phase */
    switch (state->phase) {
    case PHASE_J1:
        /* Jerk > 0: accel increasing, freq accelerating */
        state->accel_fp += state->jerk_fp;
        state->freq_fp = (ufp32_t)((fp32_t)state->freq_fp + state->accel_fp);
        break;

    case PHASE_A:
        /* Constant accel (trapezoidal: this is the only accel phase) */
        if (state->jerk_fp == 0) {
            /* Trapezoidal mode: use pre-computed accel_fp as delta per step */
            state->freq_fp = (ufp32_t)((fp32_t)state->freq_fp + state->accel_fp);
        } else {
            /* S-curve: accel is constant at max_accel */
            state->freq_fp = (ufp32_t)((fp32_t)state->freq_fp
                             + state->max_accel_fp);
        }
        break;

    case PHASE_J2:
        /* Jerk < 0: accel decreasing, still accelerating but slowing */
        state->accel_fp -= state->jerk_fp;
        if (state->accel_fp < 0) {
            state->accel_fp = 0;
        }
        state->freq_fp = (ufp32_t)((fp32_t)state->freq_fp + state->accel_fp);
        break;

    case PHASE_CRUISE:
        /* No frequency change */
        break;

    case PHASE_J3:
        /* Jerk < 0: decel increasing (accel going negative) */
        state->accel_fp -= state->jerk_fp;
        state->freq_fp = (ufp32_t)((fp32_t)state->freq_fp + state->accel_fp);
        break;

    case PHASE_D:
        /* Constant decel */
        if (state->jerk_fp == 0) {
            /* Trapezoidal: symmetric decel */
            state->freq_fp = (ufp32_t)((fp32_t)state->freq_fp - state->accel_fp);
        } else {
            /* S-curve: constant decel at -max_accel */
            state->freq_fp = (ufp32_t)((fp32_t)state->freq_fp
                             - state->max_accel_fp);
        }
        break;

    case PHASE_J4:
        /* Jerk > 0: decel decreasing (accel going toward 0) */
        state->accel_fp += state->jerk_fp;
        if (state->accel_fp > 0) {
            state->accel_fp = 0;
        }
        state->freq_fp = (ufp32_t)((fp32_t)state->freq_fp + state->accel_fp);
        break;

    case PHASE_DONE:
    default:
        state->active = 0U;
        return ccr;
    }

    /* Clamp frequency to valid range */
    state->freq_fp = clamp_freq(state->freq_fp,
                                state->start_freq_fp,
                                state->max_freq_fp);

    /* Check for total completion */
    if (state->steps_done >= state->steps_total) {
        state->phase = PHASE_DONE;
        state->active = 0U;
    }

    return ccr;
}

/* ------------------------------------------------------------------ */
void MotionProfile_Abort(ProfileState_t *state)
{
    /* ~3 cycles */
    state->phase = PHASE_DONE;
    state->active = 0U;
}

/* ================================================================== */
/* Arch Segment Interpolator                                          */
/* ================================================================== */

/*
 * Set up interpolation for the current segment. ~10 cycles.
 */
static void arch_setup_segment(ArchInterpState_t *state)
{
    const ArchSegment_t *seg;
    int32_t delta;

    if (state->seg_idx >= state->seg_count) {
        state->active = 0U;
        return;
    }

    seg = &state->segments[state->seg_idx];
    delta = seg->delta_steps;

    /* Direction from sign of delta_steps */
    if (delta >= 0) {
        state->direction = 1;
        state->seg_abs_steps = (uint32_t)delta;
    } else {
        state->direction = -1;
        state->seg_abs_steps = (uint32_t)(-delta);
    }

    state->seg_step = 0U;
    state->ccr_fp = FP_FROM_INT((int32_t)seg->start_ccr);

    /* Compute CCR increment per step (Q16.16) */
    if (state->seg_abs_steps > 0U) {
        int32_t ccr_delta = (int32_t)seg->end_ccr - (int32_t)seg->start_ccr;
        state->ccr_inc_fp = fp_div(FP_FROM_INT(ccr_delta),
                                   FP_FROM_INT((int32_t)state->seg_abs_steps));
    } else {
        state->ccr_inc_fp = 0;
        /* Zero-length segment: skip immediately */
        state->seg_idx++;
        arch_setup_segment(state);
    }
}

/* ------------------------------------------------------------------ */
int32_t ArchInterp_Init(ArchInterpState_t *state,
                         const ArchSegment_t *segments,
                         uint32_t count)
{
    /* ~30 cycles + segment setup */
    if (state == (void *)0 || segments == (void *)0) {
        return -1;
    }
    if (count == 0U || count > ARCH_MAX_SEGMENTS) {
        return -1;
    }

    state->segments = segments;
    state->seg_count = count;
    state->seg_idx = 0U;
    state->active = 1U;

    arch_setup_segment(state);

    return 0;
}

/* ------------------------------------------------------------------ */
uint16_t ArchInterp_NextCCR(ArchInterpState_t *state, int8_t *dir_out)
{
    uint16_t ccr;
    uint32_t ccr_int;

    /* ~20 cycles per step, ~40 at segment boundary */
    if (state->active == 0U) {
        if (dir_out != (void *)0) {
            *dir_out = 1;
        }
        return 0U;
    }

    /* Output direction */
    if (dir_out != (void *)0) {
        *dir_out = state->direction;
    }

    /* Get current CCR */
    ccr_int = (uint32_t)FP_TO_INT(state->ccr_fp);
    ccr = clamp_ccr(ccr_int);

    /* Advance within segment */
    state->seg_step++;
    state->ccr_fp += state->ccr_inc_fp;

    /* Check segment completion */
    if (state->seg_step >= state->seg_abs_steps) {
        state->seg_idx++;
        if (state->seg_idx >= state->seg_count) {
            state->active = 0U;
        } else {
            arch_setup_segment(state);
        }
    }

    return ccr;
}

/* ================================================================== */
/* Homing State Machine                                               */
/* ================================================================== */

/*
 * Build a simple trapezoidal approach profile for homing.
 * Uses MotionProfile_Plan with jerk=0. ~200 cycles.
 */
static int32_t homing_build_approach(HomingState_t *state,
                                      uint32_t freq,
                                      uint32_t steps)
{
    ProfileParams_t pp;
    pp.max_freq    = freq;
    pp.start_freq  = FREQ_MIN_HZ;
    pp.accel       = state->config.accel;
    pp.jerk        = 0U;
    pp.total_steps = steps;
    return MotionProfile_Plan(&state->profile, &pp);
}

/* ------------------------------------------------------------------ */
int32_t Homing_Start(HomingState_t *state, const HomingConfig_t *config)
{
    int32_t ret;

    /* ~250 cycles */
    if (state == (void *)0 || config == (void *)0) {
        return -1;
    }
    if (config->max_steps == 0U || config->fast_freq < FREQ_MIN_HZ ||
        config->slow_freq < FREQ_MIN_HZ || config->accel == 0U) {
        return -1;
    }

    state->config = *config;
    state->steps_moved = 0U;
    state->backoff_count = 0U;
    state->sensor_prev = 0U;

    /* Build fast approach profile */
    ret = homing_build_approach(state, config->fast_freq, config->max_steps);
    if (ret != 0) {
        state->phase = HOMING_ERROR;
        return -1;
    }

    state->phase = HOMING_FAST_APPROACH;
    return 0;
}

/* ------------------------------------------------------------------ */
uint16_t Homing_Tick(HomingState_t *state, uint8_t sensor_hit)
{
    uint16_t ccr;

    switch (state->phase) {
    case HOMING_IDLE:
        return 0U;

    case HOMING_FAST_APPROACH:
        /*
         * Moving toward sensor at fast speed.
         * When sensor triggers, stop and start backoff.
         * ~50 cycles (MotionProfile_NextCCR + sensor check)
         */
        if (sensor_hit != 0U && state->sensor_prev == 0U) {
            /* Sensor edge detected: abort fast approach, start backoff */
            MotionProfile_Abort(&state->profile);
            state->backoff_count = state->config.backoff_steps;
            if (state->backoff_count == 0U) {
                /* No backoff: go straight to slow approach */
                int32_t ret = homing_build_approach(
                    state, state->config.slow_freq, state->config.max_steps);
                if (ret != 0) {
                    state->phase = HOMING_ERROR;
                    return 0U;
                }
                state->phase = HOMING_SLOW_APPROACH;
            } else {
                /* Build backoff profile (away from sensor) */
                int32_t ret = homing_build_approach(
                    state, state->config.slow_freq, state->backoff_count);
                if (ret != 0) {
                    state->phase = HOMING_ERROR;
                    return 0U;
                }
                state->phase = HOMING_BACKOFF_WAIT;
            }
            state->sensor_prev = sensor_hit;
            return freq_to_ccr(state->config.slow_freq);
        }

        ccr = MotionProfile_NextCCR(&state->profile);
        state->sensor_prev = sensor_hit;

        if (ccr == 0U) {
            /* Profile exhausted without finding sensor */
            state->phase = HOMING_ERROR;
            return 0U;
        }

        state->steps_moved++;
        if (state->steps_moved >= state->config.max_steps) {
            state->phase = HOMING_ERROR;
            return 0U;
        }
        return ccr;

    case HOMING_BACKOFF_WAIT:
        /*
         * Backing off from sensor.
         * Continue until backoff profile completes.
         * ~50 cycles
         */
        ccr = MotionProfile_NextCCR(&state->profile);
        state->sensor_prev = sensor_hit;

        if (ccr == 0U) {
            /* Backoff complete: start slow approach */
            int32_t ret = homing_build_approach(
                state, state->config.slow_freq, state->config.max_steps);
            if (ret != 0) {
                state->phase = HOMING_ERROR;
                return 0U;
            }
            state->phase = HOMING_SLOW_APPROACH;
            return freq_to_ccr(state->config.slow_freq);
        }
        return ccr;

    case HOMING_SLOW_APPROACH:
        /*
         * Approaching sensor slowly for precise edge detection.
         * When sensor triggers, we are at home position.
         * ~50 cycles
         */
        if (sensor_hit != 0U && state->sensor_prev == 0U) {
            /* Sensor edge detected at slow speed: homing complete */
            MotionProfile_Abort(&state->profile);
            state->phase = HOMING_DONE;
            state->sensor_prev = sensor_hit;
            return 0U;
        }

        ccr = MotionProfile_NextCCR(&state->profile);
        state->sensor_prev = sensor_hit;

        if (ccr == 0U) {
            /* Profile exhausted without finding sensor */
            state->phase = HOMING_ERROR;
            return 0U;
        }

        state->steps_moved++;
        if (state->steps_moved >= state->config.max_steps) {
            state->phase = HOMING_ERROR;
            return 0U;
        }
        return ccr;

    case HOMING_DONE:
        return 0U;

    case HOMING_ERROR:
        return 0U;

    default:
        state->phase = HOMING_ERROR;
        return 0U;
    }
}
