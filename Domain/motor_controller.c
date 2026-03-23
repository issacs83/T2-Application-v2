/*
 * motor_controller.c : Motor state machine and position tracking
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Memory: ~400 bytes Flash, 0 bytes static SRAM (all in caller blocks).
 */

#include "motor_controller.h"
#include <string.h>

/* ------------------------------------------------------------------ */
void MotorCtrl_InitBlock(MotorCtrlBlock_t* mcb, uint16_t* buf, uint32_t buf_size)
{
    memset(mcb, 0, sizeof(MotorCtrlBlock_t));
    mcb->profile_buf  = buf;
    mcb->profile_size = buf_size;
    mcb->state        = MC_STATE_IDLE;
}

/* ------------------------------------------------------------------ */
uint16_t MotorCtrl_Prepare(MotorCtrlBlock_t* mcb)
{
    if (mcb->profile_total == 0U) {
        mcb->error_flag = 1;
        mcb->state = MC_STATE_ERROR;
        return 0;
    }

    mcb->profile_rd_idx = 0;
    mcb->done_flag  = 0;
    mcb->error_flag = 0;
    mcb->toggle     = 0;
    mcb->state      = MC_STATE_ACCEL;

    return mcb->profile_buf[0];
}

/* ------------------------------------------------------------------ */
uint16_t MotorCtrl_ISRTick(MotorCtrlBlock_t* mcb, uint8_t motor_id,
                            const MotorHWCallbacks_t* hw)
{
    uint32_t idx;

    /* If not actively running, signal stop */
    if (mcb->state == MC_STATE_IDLE || mcb->state == MC_STATE_DONE ||
        mcb->state == MC_STATE_ERROR) {
        return 0;
    }

    /* Toggle step pulse */
    if (mcb->toggle == 0U) {
        /* Rising edge = step pulse active */
        if (hw != (void*)0 && hw->SetStepPin != (void*)0) {
            hw->SetStepPin(motor_id, 1);
        }
        mcb->toggle = 1U;
        /* Return current ARR (no change needed) */
        return mcb->profile_buf[mcb->profile_rd_idx];
    }

    /* Falling edge */
    if (hw != (void*)0 && hw->SetStepPin != (void*)0) {
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

    /* Check if profile is exhausted */
    if (idx >= mcb->profile_total) {
        mcb->state     = MC_STATE_DONE;
        mcb->done_flag = 1;
        mcb->profile_rd_idx = idx;
        return 0;  /* signal completion */
    }

    mcb->profile_rd_idx = idx;
    return mcb->profile_buf[idx];
}

/* ------------------------------------------------------------------ */
void MotorCtrl_StopBlock(MotorCtrlBlock_t* mcb)
{
    mcb->state     = MC_STATE_IDLE;
    mcb->done_flag = 1;
}

/* ------------------------------------------------------------------ */
void MotorCtrl_EmergencyStopBlock(MotorCtrlBlock_t* mcb)
{
    mcb->state      = MC_STATE_ERROR;
    mcb->error_flag = 1;
    mcb->done_flag  = 0;
}

/* ------------------------------------------------------------------ */
int32_t MotorCtrl_LoadTrapProfile(MotorCtrlBlock_t* mcb,
                                   int32_t steps,
                                   uint32_t max_freq_hz,
                                   uint32_t accel_hz_per_s)
{
    int32_t ret;

    if (mcb->profile_buf == (void*)0) {
        return -1;
    }

    ret = MotorProfile_BuildTrap(mcb->profile_buf, mcb->profile_size,
                                  steps, max_freq_hz, accel_hz_per_s, (void*)0);

    if (ret == 0) {
        mcb->profile_total = (uint32_t)steps;
        mcb->profile_rd_idx = 0;
        mcb->state = MC_STATE_IDLE;
        mcb->done_flag = 0;
        mcb->error_flag = 0;
    }

    return ret;
}

/* ------------------------------------------------------------------ */
int32_t MotorCtrl_LoadArchProfile(MotorCtrlBlock_t* mcb,
                                   const uint16_t* ccr_array,
                                   uint32_t array_size)
{
    int32_t ret;

    if (mcb->profile_buf == (void*)0) {
        return -1;
    }

    ret = MotorProfile_BuildArch(mcb->profile_buf, mcb->profile_size,
                                  ccr_array, array_size);

    if (ret == 0) {
        mcb->profile_total = array_size;
        mcb->profile_rd_idx = 0;
        mcb->state = MC_STATE_IDLE;
        mcb->done_flag = 0;
        mcb->error_flag = 0;
    }

    return ret;
}

/* ------------------------------------------------------------------ */
uint8_t MotorCtrl_IsRunning(const MotorCtrlBlock_t* mcb)
{
    MotorState_t st = mcb->state;
    return (st == MC_STATE_ACCEL || st == MC_STATE_CRUISE ||
            st == MC_STATE_DECEL) ? 1U : 0U;
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
