/*
 * motor_controller.h : Motor state machine and position tracking (no HW)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Pure domain logic: state machine transitions, profile management,
 * position accounting. Does NOT touch any hardware registers.
 * Hardware actions are performed via function pointer callbacks.
 *
 * Supports three profile modes:
 *   1. Buffer-based trapezoidal (via motor_profile.h)
 *   2. Real-time S-curve / trapezoidal (via motion_profile.h)
 *   3. Arch segment interpolation (via motion_profile.h)
 *   4. Homing state machine (via motion_profile.h)
 *
 * Memory: ~500 bytes Flash, ~200 bytes SRAM per motor.
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <stdint.h>
#include "motor_profile.h"
#include "motion_profile.h"

/* Maximum number of motors in the system */
#define MC_MOTOR_COUNT          8U

/* Profile buffer size per motor (for buffer-based mode) */
#define MC_PROFILE_BUF_SIZE     4096U

/**
 * Motor state machine states.
 */
typedef enum {
    MC_STATE_IDLE = 0,
    MC_STATE_ACCEL,
    MC_STATE_CRUISE,
    MC_STATE_DECEL,
    MC_STATE_HOMING,
    MC_STATE_DONE,
    MC_STATE_ERROR
} MotorState_t;

/**
 * Motor direction.
 */
typedef enum {
    MC_DIR_ORG = 0,
    MC_DIR_LIMIT = 1
} MotorDir_t;

/**
 * Active profile mode discriminator.
 */
typedef enum {
    MC_PROF_NONE = 0,
    MC_PROF_BUFFER,         /* buffer-based (motor_profile.h) */
    MC_PROF_REALTIME,       /* real-time S-curve (motion_profile.h) */
    MC_PROF_ARCH_INTERP,    /* arch segment interpolation */
    MC_PROF_HOMING          /* homing state machine */
} MotorProfMode_t;

/**
 * Hardware callback interface - set by Port layer at init time.
 * The domain layer calls these to perform hardware actions.
 */
typedef struct {
    void (*EnableTimer)(uint8_t motor_id);
    void (*DisableTimer)(uint8_t motor_id);
    void (*SetDirection)(uint8_t motor_id, uint8_t dir);
    void (*WriteARR)(uint8_t motor_id, uint16_t arr_val);
    void (*SetStepPin)(uint8_t motor_id, uint8_t high);
    void (*ClearITPending)(uint8_t motor_id);
} MotorHWCallbacks_t;

/**
 * Motor control block (one per motor).
 * The profile buffer is stored externally to allow flexible memory placement.
 * Real-time profiles (S-curve, arch interp, homing) use embedded state structs.
 */
typedef struct {
    volatile MotorState_t   state;
    volatile int32_t        position;
    int32_t                 target_pos;
    uint8_t                 direction;
    volatile uint8_t        done_flag;
    volatile uint8_t        error_flag;
    volatile uint8_t        toggle;

    /* Active profile mode */
    MotorProfMode_t         prof_mode;

    /* Buffer-based profile (legacy) */
    uint16_t*               profile_buf;    /* pointer to external buffer */
    uint32_t                profile_size;   /* buffer capacity */
    volatile uint32_t       profile_rd_idx;
    uint32_t                profile_total;  /* valid entries count */

    /* Real-time profile state (S-curve / trapezoidal) */
    ProfileState_t          rt_profile;

    /* Arch interpolation state */
    ArchInterpState_t       arch_interp;

    /* Homing state */
    HomingState_t           homing;
} MotorCtrlBlock_t;

/**
 * Initialize a motor control block.
 *
 * @param mcb       Motor control block to initialize
 * @param buf       External profile buffer (can be NULL if only using real-time)
 * @param buf_size  Buffer capacity (number of uint16_t entries)
 */
void MotorCtrl_InitBlock(MotorCtrlBlock_t* mcb, uint16_t* buf, uint32_t buf_size);

/**
 * Prepare a motor for start (reset indices, set state to ACCEL).
 * Returns the first ARR value to load, or 0 on error.
 *
 * @param mcb  Motor control block
 * @return First CCR value, or 0 if no profile loaded
 */
uint16_t MotorCtrl_Prepare(MotorCtrlBlock_t* mcb);

/**
 * ISR tick handler - called from timer ISR for each motor.
 * Dispatches to the appropriate profile engine based on prof_mode.
 *
 * @param mcb       Motor control block
 * @param motor_id  Motor index (passed through to HW callbacks)
 * @param hw        Hardware callbacks
 * @return Next ARR value to write, or 0 if motion complete
 */
uint16_t MotorCtrl_ISRTick(MotorCtrlBlock_t* mcb, uint8_t motor_id,
                            const MotorHWCallbacks_t* hw);

/**
 * Stop a motor immediately.
 */
void MotorCtrl_StopBlock(MotorCtrlBlock_t* mcb);

/**
 * Emergency stop - set error state without disabling timer
 * (caller disables timer directly for speed).
 */
void MotorCtrl_EmergencyStopBlock(MotorCtrlBlock_t* mcb);

/**
 * Build a buffer-based trapezoidal profile into the motor's buffer.
 * Delegates to MotorProfile_BuildTrap() (legacy buffer mode).
 *
 * @return 0 on success, -1 on error
 */
int32_t MotorCtrl_LoadTrapProfile(MotorCtrlBlock_t* mcb,
                                   int32_t steps,
                                   uint32_t max_freq_hz,
                                   uint32_t accel_hz_per_s);

/**
 * Load an arch profile from pre-computed CCR array (legacy buffer mode).
 *
 * @return 0 on success, -1 on error
 */
int32_t MotorCtrl_LoadArchProfile(MotorCtrlBlock_t* mcb,
                                   const uint16_t* ccr_array,
                                   uint32_t array_size);

/**
 * Start a real-time trapezoidal profile (jerk=0, no buffer needed).
 * Uses MotionProfile_Plan internally.
 *
 * @param mcb           Motor control block
 * @param steps         Total steps (absolute, direction set separately)
 * @param max_freq_hz   Cruise frequency (Hz)
 * @param accel_hz_per_s Acceleration rate (Hz/s)
 * @return 0 on success, -1 on error
 */
int32_t MCtrl_StartTrapezoidal(MotorCtrlBlock_t* mcb,
                                uint32_t steps,
                                uint32_t max_freq_hz,
                                uint32_t accel_hz_per_s);

/**
 * Start a real-time S-curve profile (jerk>0, no buffer needed).
 * Uses MotionProfile_Plan internally.
 *
 * @param mcb           Motor control block
 * @param steps         Total steps (absolute, direction set separately)
 * @param max_freq_hz   Cruise frequency (Hz)
 * @param accel_hz_per_s Acceleration rate (Hz/s)
 * @param jerk_hz_per_s2 Jerk rate (Hz/s^2)
 * @return 0 on success, -1 on error
 */
int32_t MCtrl_StartSCurve(MotorCtrlBlock_t* mcb,
                           uint32_t steps,
                           uint32_t max_freq_hz,
                           uint32_t accel_hz_per_s,
                           uint32_t jerk_hz_per_s2);

/**
 * Start arch segment interpolation (no buffer needed).
 * Uses ArchInterp_Init internally.
 *
 * @param mcb       Motor control block
 * @param segments  Array of arch segments (must remain valid during motion)
 * @param count     Number of segments
 * @return 0 on success, -1 on error
 */
int32_t MCtrl_StartArch(MotorCtrlBlock_t* mcb,
                         const ArchSegment_t* segments,
                         uint32_t count);

/**
 * Start homing sequence.
 * Uses Homing_Start internally.
 *
 * @param mcb       Motor control block
 * @param config    Homing configuration
 * @return 0 on success, -1 on error
 */
int32_t MCtrl_StartHoming(MotorCtrlBlock_t* mcb,
                           const HomingConfig_t* config);

/**
 * Provide sensor input for homing tick (call from ISR context).
 * Only valid when prof_mode == MC_PROF_HOMING.
 *
 * @param mcb         Motor control block
 * @param sensor_hit  1 = sensor triggered, 0 = not triggered
 */
void MCtrl_HomingSensorUpdate(MotorCtrlBlock_t* mcb, uint8_t sensor_hit);

/**
 * Query functions.
 */
uint8_t      MotorCtrl_IsRunning(const MotorCtrlBlock_t* mcb);
uint8_t      MotorCtrl_IsDone(const MotorCtrlBlock_t* mcb);
int32_t      MotorCtrl_GetPosition(const MotorCtrlBlock_t* mcb);
void         MotorCtrl_SetPosition(MotorCtrlBlock_t* mcb, int32_t pos);
MotorState_t MotorCtrl_GetState(const MotorCtrlBlock_t* mcb);

/**
 * Get active profile mode.
 */
static inline MotorProfMode_t MotorCtrl_GetProfMode(const MotorCtrlBlock_t* mcb)
{
    return mcb->prof_mode;
}

#endif /* MOTOR_CONTROLLER_H */
