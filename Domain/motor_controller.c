/*
 * motor_controller.c : Motor state machine and position tracking
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Supports buffer-based (legacy), real-time S-curve, arch interpolation,
 * and homing profile modes. Zero hardware dependency.
 *
 * Memory: ~700 bytes Flash, 0 bytes static SRAM (all in caller blocks).
 */

#include "motor_controller.h"
#include <string.h>

/* Sensor input latch for homing ISR ticks (set by MCtrl_HomingSensorUpdate) */
/* Per-block storage avoids global state; stored in homing.sensor_prev field */

/* ================================================================== */
/* Initialization                                                     */
/* ================================================================== */

void MotorCtrl_InitBlock(MotorCtrlBlock_t* mcb, uint16_t* buf, uint32_t buf_size)
{
    memset(mcb, 0, sizeof(MotorCtrlBlock_t));
    mcb->profile_buf  = buf;
    mcb->profile_size = buf_size;
    mcb->state        = MC_STATE_IDLE;
    mcb->prof_mode    = MC_PROF_NONE;
}

/* ================================================================== */
/* Prepare                                                            */
/* ================================================================== */

uint16_t MotorCtrl_Prepare(MotorCtrlBlock_t* mcb)
{
    mcb->done_flag  = 0;
    mcb->error_flag = 0;
    mcb->toggle     = 0;

    switch (mcb->prof_mode) {
    case MC_PROF_BUFFER:
        if (mcb->profile_total == 0U) {
            mcb->error_flag = 1;
            mcb->state = MC_STATE_ERROR;
            return 0;
        }
        mcb->profile_rd_idx = 0;
        mcb->state = MC_STATE_ACCEL;
        return mcb->profile_buf[0];

    case MC_PROF_REALTIME:
        if (mcb->rt_profile.active == 0U) {
            mcb->error_flag = 1;
            mcb->state = MC_STATE_ERROR;
            return 0;
        }
        mcb->state = MC_STATE_ACCEL;
        /* Return CCR for the start frequency */
        return fp_freq_to_ccr(mcb->rt_profile.start_freq_fp);

    case MC_PROF_ARCH_INTERP:
        if (mcb->arch_interp.active == 0U) {
            mcb->error_flag = 1;
            mcb->state = MC_STATE_ERROR;
            return 0;
        }
        mcb->state = MC_STATE_ACCEL;
        /* Return CCR from the first segment's start */
        if (mcb->arch_interp.segments != (void *)0 &&
            mcb->arch_interp.seg_count > 0U) {
            return mcb->arch_interp.segments[0].start_ccr;
        }
        return (uint16_t)CCR_MAX;

    case MC_PROF_HOMING:
        if (mcb->homing.phase == HOMING_ERROR ||
            mcb->homing.phase == HOMING_IDLE) {
            mcb->error_flag = 1;
            mcb->state = MC_STATE_ERROR;
            return 0;
        }
        mcb->state = MC_STATE_HOMING;
        return freq_to_ccr(mcb->homing.config.fast_freq);

    case MC_PROF_NONE:
    default:
        mcb->error_flag = 1;
        mcb->state = MC_STATE_ERROR;
        return 0;
    }
}

/* ================================================================== */
/* ISR Tick: Buffer-based                                              */
/* ================================================================== */

/* ~30 cycles */
static uint16_t isr_tick_buffer(MotorCtrlBlock_t* mcb, uint8_t motor_id,
                                 const MotorHWCallbacks_t* hw)
{
    uint32_t idx;

    /* Toggle step pulse */
    if (mcb->toggle == 0U) {
        if (hw != (void *)0 && hw->SetStepPin != (void *)0) {
            hw->SetStepPin(motor_id, 1);
        }
        mcb->toggle = 1U;
        return mcb->profile_buf[mcb->profile_rd_idx];
    }

    /* Falling edge */
    if (hw != (void *)0 && hw->SetStepPin != (void *)0) {
        hw->SetStepPin(motor_id, 0);
    }
    mcb->toggle = 0U;

    /* Update position */
    if (mcb->direction != 0U) {
        mcb->position++;
    } else {
        mcb->position--;
    }

    /* Advance to next step */
    idx = mcb->profile_rd_idx + 1U;

    if (idx >= mcb->profile_total) {
        mcb->state     = MC_STATE_DONE;
        mcb->done_flag = 1;
        mcb->profile_rd_idx = idx;
        return 0;
    }

    mcb->profile_rd_idx = idx;
    return mcb->profile_buf[idx];
}

/* ================================================================== */
/* ISR Tick: Real-time S-curve/trapezoidal                            */
/* ================================================================== */

/* ~50 cycles */
static uint16_t isr_tick_realtime(MotorCtrlBlock_t* mcb, uint8_t motor_id,
                                   const MotorHWCallbacks_t* hw)
{
    uint16_t ccr;

    /* Toggle step pulse */
    if (mcb->toggle == 0U) {
        if (hw != (void *)0 && hw->SetStepPin != (void *)0) {
            hw->SetStepPin(motor_id, 1);
        }
        mcb->toggle = 1U;
        /* Return current CCR (no advance on rising edge) */
        return fp_freq_to_ccr(mcb->rt_profile.freq_fp);
    }

    /* Falling edge */
    if (hw != (void *)0 && hw->SetStepPin != (void *)0) {
        hw->SetStepPin(motor_id, 0);
    }
    mcb->toggle = 0U;

    /* Update position */
    if (mcb->direction != 0U) {
        mcb->position++;
    } else {
        mcb->position--;
    }

    /* Get next CCR from motion profile */
    ccr = MotionProfile_NextCCR(&mcb->rt_profile);

    if (ccr == 0U || mcb->rt_profile.active == 0U) {
        mcb->state     = MC_STATE_DONE;
        mcb->done_flag = 1;
        return 0;
    }

    /* Update state based on profile phase */
    switch (mcb->rt_profile.phase) {
    case PHASE_J1:
    case PHASE_A:
    case PHASE_J2:
        mcb->state = MC_STATE_ACCEL;
        break;
    case PHASE_CRUISE:
        mcb->state = MC_STATE_CRUISE;
        break;
    case PHASE_J3:
    case PHASE_D:
    case PHASE_J4:
        mcb->state = MC_STATE_DECEL;
        break;
    case PHASE_DONE:
        mcb->state     = MC_STATE_DONE;
        mcb->done_flag = 1;
        return 0;
    default:
        break;
    }

    return ccr;
}

/* ================================================================== */
/* ISR Tick: Arch interpolation                                       */
/* ================================================================== */

/* ~30 cycles */
static uint16_t isr_tick_arch(MotorCtrlBlock_t* mcb, uint8_t motor_id,
                               const MotorHWCallbacks_t* hw)
{
    uint16_t ccr;
    int8_t dir;

    /* Toggle step pulse */
    if (mcb->toggle == 0U) {
        if (hw != (void *)0 && hw->SetStepPin != (void *)0) {
            hw->SetStepPin(motor_id, 1);
        }
        mcb->toggle = 1U;
        /* Return current CCR */
        {
            uint32_t ccr_int = (uint32_t)FP_TO_INT(mcb->arch_interp.ccr_fp);
            if (ccr_int < CCR_MIN) { ccr_int = CCR_MIN; }
            if (ccr_int > CCR_MAX) { ccr_int = CCR_MAX; }
            return (uint16_t)ccr_int;
        }
    }

    /* Falling edge */
    if (hw != (void *)0 && hw->SetStepPin != (void *)0) {
        hw->SetStepPin(motor_id, 0);
    }
    mcb->toggle = 0U;

    /* Get next CCR and direction */
    ccr = ArchInterp_NextCCR(&mcb->arch_interp, &dir);

    /* Update position based on interpolator direction */
    if (dir > 0) {
        mcb->position++;
    } else {
        mcb->position--;
    }

    /* Update direction GPIO if callback available */
    if (hw != (void *)0 && hw->SetDirection != (void *)0) {
        mcb->direction = (dir > 0) ? 1U : 0U;
        hw->SetDirection(motor_id, mcb->direction);
    }

    if (ccr == 0U || mcb->arch_interp.active == 0U) {
        mcb->state     = MC_STATE_DONE;
        mcb->done_flag = 1;
        return 0;
    }

    return ccr;
}

/* ================================================================== */
/* ISR Tick: Homing                                                   */
/* ================================================================== */

/* Sensor latch is stored in homing.sensor_prev by MCtrl_HomingSensorUpdate */

/* ~60 cycles */
static uint16_t isr_tick_homing(MotorCtrlBlock_t* mcb, uint8_t motor_id,
                                 const MotorHWCallbacks_t* hw)
{
    uint16_t ccr;
    uint8_t sensor;

    /* Toggle step pulse */
    if (mcb->toggle == 0U) {
        if (hw != (void *)0 && hw->SetStepPin != (void *)0) {
            hw->SetStepPin(motor_id, 1);
        }
        mcb->toggle = 1U;
        return freq_to_ccr(mcb->homing.config.slow_freq);
    }

    /* Falling edge */
    if (hw != (void *)0 && hw->SetStepPin != (void *)0) {
        hw->SetStepPin(motor_id, 0);
    }
    mcb->toggle = 0U;

    /* Update position */
    if (mcb->direction != 0U) {
        mcb->position++;
    } else {
        mcb->position--;
    }

    /* Read latched sensor value */
    sensor = mcb->homing.sensor_prev;

    /* Drive homing state machine */
    ccr = Homing_Tick(&mcb->homing, sensor);

    /* Update direction based on homing phase */
    if (mcb->homing.phase == HOMING_BACKOFF_WAIT) {
        /* Backoff: reverse direction */
        uint8_t backoff_dir = (mcb->homing.config.approach_dir != 0U) ? 0U : 1U;
        if (hw != (void *)0 && hw->SetDirection != (void *)0) {
            hw->SetDirection(motor_id, backoff_dir);
        }
        mcb->direction = backoff_dir;
    } else if (mcb->homing.phase == HOMING_FAST_APPROACH ||
               mcb->homing.phase == HOMING_SLOW_APPROACH) {
        if (hw != (void *)0 && hw->SetDirection != (void *)0) {
            hw->SetDirection(motor_id, mcb->homing.config.approach_dir);
        }
        mcb->direction = mcb->homing.config.approach_dir;
    }

    if (ccr == 0U) {
        if (mcb->homing.phase == HOMING_DONE) {
            mcb->state     = MC_STATE_DONE;
            mcb->done_flag = 1;
            mcb->position  = 0;  /* home position */
        } else {
            mcb->state      = MC_STATE_ERROR;
            mcb->error_flag = 1;
        }
        return 0;
    }

    return ccr;
}

/* ================================================================== */
/* ISR Tick Dispatcher                                                */
/* ================================================================== */

uint16_t MotorCtrl_ISRTick(MotorCtrlBlock_t* mcb, uint8_t motor_id,
                            const MotorHWCallbacks_t* hw)
{
    /* If not actively running, signal stop */
    if (mcb->state == MC_STATE_IDLE || mcb->state == MC_STATE_DONE ||
        mcb->state == MC_STATE_ERROR) {
        return 0;
    }

    switch (mcb->prof_mode) {
    case MC_PROF_BUFFER:
        return isr_tick_buffer(mcb, motor_id, hw);
    case MC_PROF_REALTIME:
        return isr_tick_realtime(mcb, motor_id, hw);
    case MC_PROF_ARCH_INTERP:
        return isr_tick_arch(mcb, motor_id, hw);
    case MC_PROF_HOMING:
        return isr_tick_homing(mcb, motor_id, hw);
    case MC_PROF_NONE:
    default:
        return 0;
    }
}

/* ================================================================== */
/* Stop / Emergency Stop                                              */
/* ================================================================== */

void MotorCtrl_StopBlock(MotorCtrlBlock_t* mcb)
{
    /* Abort any active real-time profile */
    if (mcb->prof_mode == MC_PROF_REALTIME) {
        MotionProfile_Abort(&mcb->rt_profile);
    }
    mcb->state     = MC_STATE_IDLE;
    mcb->done_flag = 1;
}

void MotorCtrl_EmergencyStopBlock(MotorCtrlBlock_t* mcb)
{
    if (mcb->prof_mode == MC_PROF_REALTIME) {
        MotionProfile_Abort(&mcb->rt_profile);
    }
    mcb->state      = MC_STATE_ERROR;
    mcb->error_flag = 1;
    mcb->done_flag  = 0;
}

/* ================================================================== */
/* Buffer-based Profile Loading (Legacy)                              */
/* ================================================================== */

int32_t MotorCtrl_LoadTrapProfile(MotorCtrlBlock_t* mcb,
                                   int32_t steps,
                                   uint32_t max_freq_hz,
                                   uint32_t accel_hz_per_s)
{
    int32_t ret;

    if (mcb->profile_buf == (void *)0) {
        return -1;
    }

    ret = MotorProfile_BuildTrap(mcb->profile_buf, mcb->profile_size,
                                  steps, max_freq_hz, accel_hz_per_s, (void *)0);

    if (ret == 0) {
        mcb->profile_total  = (uint32_t)steps;
        mcb->profile_rd_idx = 0;
        mcb->prof_mode      = MC_PROF_BUFFER;
        mcb->state          = MC_STATE_IDLE;
        mcb->done_flag      = 0;
        mcb->error_flag     = 0;
    }

    return ret;
}

int32_t MotorCtrl_LoadArchProfile(MotorCtrlBlock_t* mcb,
                                   const uint16_t* ccr_array,
                                   uint32_t array_size)
{
    int32_t ret;

    if (mcb->profile_buf == (void *)0) {
        return -1;
    }

    ret = MotorProfile_BuildArch(mcb->profile_buf, mcb->profile_size,
                                  ccr_array, array_size);

    if (ret == 0) {
        mcb->profile_total  = array_size;
        mcb->profile_rd_idx = 0;
        mcb->prof_mode      = MC_PROF_BUFFER;
        mcb->state          = MC_STATE_IDLE;
        mcb->done_flag      = 0;
        mcb->error_flag     = 0;
    }

    return ret;
}

/* ================================================================== */
/* Real-time Profile Start                                            */
/* ================================================================== */

int32_t MCtrl_StartTrapezoidal(MotorCtrlBlock_t* mcb,
                                uint32_t steps,
                                uint32_t max_freq_hz,
                                uint32_t accel_hz_per_s)
{
    ProfileParams_t pp;
    int32_t ret;

    pp.max_freq    = max_freq_hz;
    pp.start_freq  = MP_TRAP_START_FREQ_HZ;
    pp.accel       = accel_hz_per_s;
    pp.jerk        = 0U;
    pp.total_steps = steps;

    ret = MotionProfile_Plan(&mcb->rt_profile, &pp);
    if (ret != 0) {
        return -1;
    }

    mcb->prof_mode  = MC_PROF_REALTIME;
    mcb->state      = MC_STATE_IDLE;
    mcb->done_flag  = 0;
    mcb->error_flag = 0;

    return 0;
}

int32_t MCtrl_StartSCurve(MotorCtrlBlock_t* mcb,
                           uint32_t steps,
                           uint32_t max_freq_hz,
                           uint32_t accel_hz_per_s,
                           uint32_t jerk_hz_per_s2)
{
    ProfileParams_t pp;
    int32_t ret;

    if (jerk_hz_per_s2 == 0U) {
        /* Caller requested S-curve but jerk=0; fall back to trapezoidal */
        return MCtrl_StartTrapezoidal(mcb, steps, max_freq_hz, accel_hz_per_s);
    }

    pp.max_freq    = max_freq_hz;
    pp.start_freq  = MP_TRAP_START_FREQ_HZ;
    pp.accel       = accel_hz_per_s;
    pp.jerk        = jerk_hz_per_s2;
    pp.total_steps = steps;

    ret = MotionProfile_Plan(&mcb->rt_profile, &pp);
    if (ret != 0) {
        return -1;
    }

    mcb->prof_mode  = MC_PROF_REALTIME;
    mcb->state      = MC_STATE_IDLE;
    mcb->done_flag  = 0;
    mcb->error_flag = 0;

    return 0;
}

/* ================================================================== */
/* Arch Interpolation Start                                           */
/* ================================================================== */

int32_t MCtrl_StartArch(MotorCtrlBlock_t* mcb,
                         const ArchSegment_t* segments,
                         uint32_t count)
{
    int32_t ret;

    ret = ArchInterp_Init(&mcb->arch_interp, segments, count);
    if (ret != 0) {
        return -1;
    }

    mcb->prof_mode  = MC_PROF_ARCH_INTERP;
    mcb->state      = MC_STATE_IDLE;
    mcb->done_flag  = 0;
    mcb->error_flag = 0;

    return 0;
}

/* ================================================================== */
/* Homing Start                                                       */
/* ================================================================== */

int32_t MCtrl_StartHoming(MotorCtrlBlock_t* mcb,
                           const HomingConfig_t* config)
{
    int32_t ret;

    ret = Homing_Start(&mcb->homing, config);
    if (ret != 0) {
        return -1;
    }

    mcb->prof_mode  = MC_PROF_HOMING;
    mcb->state      = MC_STATE_IDLE;
    mcb->done_flag  = 0;
    mcb->error_flag = 0;

    return 0;
}

/* ================================================================== */
/* Homing Sensor Update                                               */
/* ================================================================== */

void MCtrl_HomingSensorUpdate(MotorCtrlBlock_t* mcb, uint8_t sensor_hit)
{
    if (mcb->prof_mode == MC_PROF_HOMING) {
        mcb->homing.sensor_prev = sensor_hit;
    }
}

/* ================================================================== */
/* Query Functions                                                    */
/* ================================================================== */

uint8_t MotorCtrl_IsRunning(const MotorCtrlBlock_t* mcb)
{
    MotorState_t st = mcb->state;
    return (st == MC_STATE_ACCEL || st == MC_STATE_CRUISE ||
            st == MC_STATE_DECEL || st == MC_STATE_HOMING) ? 1U : 0U;
}

uint8_t MotorCtrl_IsDone(const MotorCtrlBlock_t* mcb)
{
    return mcb->done_flag;
}

int32_t MotorCtrl_GetPosition(const MotorCtrlBlock_t* mcb)
{
    return mcb->position;
}

void MotorCtrl_SetPosition(MotorCtrlBlock_t* mcb, int32_t pos)
{
    mcb->position = pos;
}

MotorState_t MotorCtrl_GetState(const MotorCtrlBlock_t* mcb)
{
    return mcb->state;
}
