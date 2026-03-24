/*
 * motion_profile.h : Real-time S-curve / trapezoidal motion profile (no HW)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Computes next CCR value per ISR tick using a 7-segment S-curve model.
 * Zero hardware dependency. All integer Q16.16 fixed-point arithmetic.
 * No pre-allocated profile buffer needed (state machine based).
 *
 * Also provides:
 *   - Arch interpolator: linear CCR interpolation between segments
 *   - Homing state machine: sensor-driven approach/backoff/settle
 *
 * Memory: ~1200 bytes Flash, ~64 bytes SRAM per ProfileState_t.
 */

#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include <stdint.h>

/* ================================================================== */
/* Q16.16 Fixed-Point Types and Arithmetic                            */
/* ================================================================== */

/** Signed Q16.16 fixed-point: 16-bit integer + 16-bit fraction */
typedef int32_t     fp32_t;

/** Unsigned Q16.16 fixed-point */
typedef uint32_t    ufp32_t;

/** Signed Q32.32 fixed-point for intermediate calculations */
typedef int64_t     fp64_t;

#define FP_SHIFT            16
#define FP_ONE              ((fp32_t)(1L << FP_SHIFT))       /* 0x00010000 */
#define FP_HALF             ((fp32_t)(1L << (FP_SHIFT - 1))) /* 0x00008000 */

/* Conversion macros */
#define FP_FROM_INT(x)      ((fp32_t)((int32_t)(x) << FP_SHIFT))
#define UFP_FROM_INT(x)     ((ufp32_t)((uint32_t)(x) << FP_SHIFT))
#define FP_TO_INT(x)        ((int32_t)((fp32_t)(x) >> FP_SHIFT))
#define UFP_TO_INT(x)       ((uint32_t)((ufp32_t)(x) >> FP_SHIFT))
#define FP_FRAC(x)          ((fp32_t)(x) & 0xFFFF)

/*
 * Multiply: (a * b) >> 16
 * Uses 64-bit intermediate to avoid overflow.
 * ~5 cycles on Cortex-M with HW multiply
 */
static inline fp32_t fp_mul(fp32_t a, fp32_t b)
{
    return (fp32_t)(((fp64_t)a * (fp64_t)b) >> FP_SHIFT);
}

/*
 * Divide: (a << 16) / b
 * ~12 cycles on Cortex-M with HW divide (SDIV)
 */
static inline fp32_t fp_div(fp32_t a, fp32_t b)
{
    if (b == 0) {
        return (a >= 0) ? (fp32_t)0x7FFFFFFF : (fp32_t)(-0x7FFFFFFF);
    }
    return (fp32_t)(((fp64_t)a << FP_SHIFT) / (fp64_t)b);
}

/* Unsigned variants */
static inline ufp32_t ufp_mul(ufp32_t a, ufp32_t b)
{
    return (ufp32_t)(((uint64_t)a * (uint64_t)b) >> FP_SHIFT);
}

static inline ufp32_t ufp_div(ufp32_t a, ufp32_t b)
{
    if (b == 0U) {
        return 0xFFFFFFFFU;
    }
    return (ufp32_t)(((uint64_t)a << FP_SHIFT) / (uint64_t)b);
}

/* ================================================================== */
/* Timer/CCR Conversion Constants                                     */
/* ================================================================== */

/** Timer clock in Hz (must match hardware timer config) */
#define MP_TIM_CLK          1250000U

/** Toggle factor: each step = 2 timer events (high + low) */
#define MP_TIM_TOGGLE       2U

/** Half-clock constant for CCR conversion: TIM_CLK / TIM_TOGGLE */
#define MP_HALF_CLK         (MP_TIM_CLK / MP_TIM_TOGGLE)  /* 625000 */

/** CCR clamping bounds */
#define CCR_MIN             10U
#define CCR_MAX             65535U

/** Minimum frequency to avoid CCR overflow (Hz) */
#define FREQ_MIN_HZ         10U

/* ================================================================== */
/* Frequency <-> CCR Conversion                                       */
/* ================================================================== */

/*
 * Convert frequency (Hz) to CCR timer reload value.
 * CCR = HALF_CLK / freq_hz, clamped to [CCR_MIN, CCR_MAX].
 * ~3 cycles (UDIV + clamp)
 */
static inline uint16_t freq_to_ccr(uint32_t freq_hz)
{
    uint32_t ccr;
    if (freq_hz < FREQ_MIN_HZ) {
        freq_hz = FREQ_MIN_HZ;
    }
    ccr = MP_HALF_CLK / freq_hz;
    if (ccr < CCR_MIN) {
        return (uint16_t)CCR_MIN;
    }
    if (ccr > CCR_MAX) {
        return (uint16_t)CCR_MAX;
    }
    return (uint16_t)ccr;
}

/*
 * Convert CCR timer reload value to frequency (Hz).
 * freq = HALF_CLK / ccr
 * ~3 cycles (UDIV)
 */
static inline uint32_t ccr_to_freq(uint16_t ccr)
{
    if (ccr < CCR_MIN) {
        return MP_HALF_CLK / CCR_MIN;
    }
    return MP_HALF_CLK / (uint32_t)ccr;
}

/*
 * Convert Q16.16 frequency to CCR.
 * freq_fp is in Q16.16 Hz. Extract integer part, then convert.
 * ~5 cycles
 */
static inline uint16_t fp_freq_to_ccr(ufp32_t freq_fp)
{
    uint32_t freq_int = UFP_TO_INT(freq_fp);
    return freq_to_ccr(freq_int);
}

/* ================================================================== */
/* 7-Segment S-Curve Profile                                          */
/* ================================================================== */

/**
 * Profile phase (7-segment S-curve).
 *
 * Velocity profile:
 *   J1: jerk (+j)  -> accel ramps from 0 to a_max
 *   A:  const accel -> velocity increases linearly
 *   J2: jerk (-j)  -> accel ramps from a_max to 0
 *   CRUISE: v = v_max
 *   J3: jerk (-j)  -> accel ramps from 0 to -a_max
 *   D:  const decel -> velocity decreases linearly
 *   J4: jerk (+j)  -> accel ramps from -a_max to 0
 *   DONE: motion complete
 */
typedef enum {
    PHASE_J1 = 0,   /* accel increasing (jerk > 0)  */
    PHASE_A,         /* constant acceleration        */
    PHASE_J2,        /* accel decreasing (jerk < 0)  */
    PHASE_CRUISE,    /* constant velocity             */
    PHASE_J3,        /* decel increasing (jerk < 0)  */
    PHASE_D,         /* constant deceleration         */
    PHASE_J4,        /* decel decreasing (jerk > 0)  */
    PHASE_DONE       /* motion complete               */
} ProfilePhase_t;

/**
 * Profile parameters (input to planner).
 * All frequencies in Hz, accel in Hz/s, jerk in Hz/s^2.
 * Set jerk=0 for trapezoidal (no S-curve) fallback.
 */
typedef struct {
    uint32_t    max_freq;       /* target cruise frequency (Hz)        */
    uint32_t    start_freq;     /* initial/final frequency (Hz)        */
    uint32_t    accel;          /* max acceleration (Hz/s)             */
    uint32_t    jerk;           /* jerk rate (Hz/s^2), 0=trapezoidal  */
    uint32_t    total_steps;    /* total steps to execute              */
} ProfileParams_t;

/**
 * Profile runtime state (~64 bytes).
 * Maintained per motor. Updated each ISR tick.
 */
typedef struct {
    ProfilePhase_t  phase;          /* current phase                    */
    ufp32_t         freq_fp;        /* current frequency (Q16.16 Hz)   */
    fp32_t          accel_fp;       /* current acceleration (Q16.16)   */
    fp32_t          jerk_fp;        /* jerk increment per step (Q16.16)*/
    fp32_t          accel_inc;      /* accel increment per step (Q16.16)*/
    ufp32_t         max_freq_fp;    /* target cruise freq (Q16.16)     */
    ufp32_t         start_freq_fp;  /* start/end freq (Q16.16)         */
    fp32_t          max_accel_fp;   /* max acceleration (Q16.16)       */
    uint32_t        phase_step;     /* steps completed in current phase*/
    uint32_t        phase_len[7];   /* steps per phase (J1..J4)        */
    uint32_t        steps_done;     /* total steps completed           */
    uint32_t        steps_total;    /* total steps to execute          */
    uint8_t         active;         /* 1 = profile running, 0 = idle   */
    uint8_t         _pad[3];        /* alignment padding               */
} ProfileState_t;

/**
 * Plan a 7-segment S-curve motion profile.
 * Pre-computes phase lengths. Call from main context (not ISR).
 *
 * If jerk == 0, produces a trapezoidal profile (phases J1-J4 have 0 length).
 * Handles triangle profile when total_steps is too few for full cruise.
 *
 * ~200 cycles typical
 *
 * @param state   Profile state to initialize
 * @param params  Motion parameters
 * @return 0 on success, -1 on invalid parameters
 */
int32_t MotionProfile_Plan(ProfileState_t *state, const ProfileParams_t *params);

/**
 * Compute next CCR value (call from ISR, one per step).
 *
 * Advances the profile state machine by one step. Returns the CCR
 * reload value for the timer, or 0 when motion is complete.
 *
 * ~40-60 cycles per call (switch + fp_mul + freq_to_ccr)
 *
 * @param state  Profile state (must be planned first)
 * @return CCR value [CCR_MIN..CCR_MAX], or 0 if DONE
 */
uint16_t MotionProfile_NextCCR(ProfileState_t *state);

/**
 * Abort a running profile immediately.
 * Sets phase to DONE and active to 0.
 * Safe to call from ISR.
 * ~3 cycles
 */
void MotionProfile_Abort(ProfileState_t *state);

/**
 * Get remaining steps in the profile.
 * ~3 cycles
 */
static inline uint32_t MotionProfile_Remaining(const ProfileState_t *state)
{
    if (state->steps_done >= state->steps_total) {
        return 0U;
    }
    return state->steps_total - state->steps_done;
}

/* ================================================================== */
/* Arch Segment Interpolator                                          */
/* ================================================================== */

/** Maximum segments per arch trajectory */
#define ARCH_MAX_SEGMENTS   64U

/**
 * One segment of an arch trajectory.
 * CCR linearly interpolated from start_ccr to end_ccr over delta_steps.
 * Negative delta_steps means reverse direction.
 */
typedef struct {
    int32_t     delta_steps;    /* signed step count for this segment  */
    uint16_t    start_ccr;      /* CCR at segment start                */
    uint16_t    end_ccr;        /* CCR at segment end                  */
} ArchSegment_t;

/**
 * Arch interpolation state (~32 bytes).
 */
typedef struct {
    const ArchSegment_t *segments;  /* pointer to segment array         */
    uint32_t    seg_count;          /* total number of segments          */
    uint32_t    seg_idx;            /* current segment index             */
    uint32_t    seg_step;           /* step within current segment       */
    uint32_t    seg_abs_steps;      /* abs(delta_steps) of current seg   */
    fp32_t      ccr_fp;             /* current CCR in Q16.16             */
    fp32_t      ccr_inc_fp;         /* CCR increment per step (Q16.16)  */
    int8_t      direction;          /* +1 or -1 for current segment      */
    uint8_t     active;             /* 1 = running, 0 = idle             */
    uint8_t     _pad[2];            /* alignment padding                 */
} ArchInterpState_t;

/**
 * Initialize arch interpolator with segment array.
 * Call from main context.
 *
 * ~30 cycles + first segment setup
 *
 * @param state     Interpolation state
 * @param segments  Array of segments (must remain valid during motion)
 * @param count     Number of segments (must be > 0, <= ARCH_MAX_SEGMENTS)
 * @return 0 on success, -1 on invalid parameters
 */
int32_t ArchInterp_Init(ArchInterpState_t *state,
                         const ArchSegment_t *segments,
                         uint32_t count);

/**
 * Get next CCR value from arch interpolator (call from ISR).
 *
 * ~20 cycles per step, ~40 cycles at segment boundary
 *
 * @param state      Interpolation state
 * @param dir_out    Output: direction for this step (+1 or -1)
 * @return CCR value [CCR_MIN..CCR_MAX], or 0 if all segments done
 */
uint16_t ArchInterp_NextCCR(ArchInterpState_t *state, int8_t *dir_out);

/* ================================================================== */
/* Homing State Machine                                               */
/* ================================================================== */

/**
 * Homing phase.
 */
typedef enum {
    HOMING_IDLE = 0,        /* not started                          */
    HOMING_FAST_APPROACH,   /* moving toward sensor at high speed   */
    HOMING_BACKOFF_WAIT,    /* backing off from sensor, waiting     */
    HOMING_SLOW_APPROACH,   /* approaching sensor at low speed      */
    HOMING_DONE,            /* sensor edge found, position is 0     */
    HOMING_ERROR            /* timeout or profile error             */
} HomingPhase_t;

/**
 * Homing configuration (input).
 */
typedef struct {
    uint32_t    fast_freq;      /* fast approach frequency (Hz)         */
    uint32_t    slow_freq;      /* slow approach frequency (Hz)         */
    uint32_t    backoff_steps;  /* steps to back off after first detect */
    uint32_t    accel;          /* acceleration for approach (Hz/s)     */
    uint32_t    max_steps;      /* max steps before timeout error       */
    uint8_t     approach_dir;   /* direction toward sensor (0 or 1)     */
    uint8_t     _pad[3];
} HomingConfig_t;

/**
 * Homing runtime state (~100 bytes including embedded ProfileState_t).
 */
typedef struct {
    HomingPhase_t   phase;
    HomingConfig_t  config;
    ProfileState_t  profile;        /* embedded motion profile          */
    uint32_t        steps_moved;    /* total steps since homing start   */
    uint32_t        backoff_count;  /* steps remaining in backoff       */
    uint8_t         sensor_prev;    /* previous sensor reading          */
    uint8_t         _pad[3];
} HomingState_t;

/**
 * Start homing sequence.
 * Configures and starts the fast approach profile.
 * Call from main context.
 *
 * ~250 cycles (includes MotionProfile_Plan)
 *
 * @param state   Homing state to initialize
 * @param config  Homing parameters
 * @return 0 on success, -1 on invalid config
 */
int32_t Homing_Start(HomingState_t *state, const HomingConfig_t *config);

/**
 * Homing tick (call from ISR or periodic task).
 *
 * Drives the homing state machine. The sensor input is passed
 * as a parameter (Domain layer does NOT access GPIO).
 *
 * @param state       Homing state
 * @param sensor_hit  1 = sensor triggered, 0 = not triggered
 * @return CCR value for timer, or 0 if homing complete/error
 */
uint16_t Homing_Tick(HomingState_t *state, uint8_t sensor_hit);

/**
 * Get current homing phase.
 * ~1 cycle
 */
static inline HomingPhase_t Homing_GetPhase(const HomingState_t *state)
{
    return state->phase;
}

#endif /* MOTION_PROFILE_H */
