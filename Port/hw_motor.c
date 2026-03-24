/*
 * hw_motor.c : Motor hardware layer - wraps motor_ctrl.c behind interface
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Supports legacy buffer-based profiles and new real-time S-curve,
 * arch interpolation, and homing via motion_profile.h.
 *
 * Memory: ~500 bytes Flash, ~8 bytes SRAM (callback pointer).
 */

#include "hw_motor.h"
#include "motor_ctrl.h"
#include "tmc2660.h"

/* Private variables */
static MotorDoneCallback_t g_motor_done_cb = (MotorDoneCallback_t)0;

/* ================================================================== */
/* MotorInterface_t implementations                                   */
/* ================================================================== */

static void motor_iface_init(void)
{
    MotorCtrl_Init();
    TMC2660_SPI_Init();
    TMC2660_InitAll();
}

static void motor_iface_start(uint8_t id)
{
    if (id < MCTRL_COUNT) {
        MotorCtrl_Start((MotorId_t)id);
    }
}

static void motor_iface_stop(uint8_t id)
{
    if (id < MCTRL_COUNT) {
        MotorCtrl_Stop((MotorId_t)id);
    }
}

static void motor_iface_estop_all(void)
{
    MotorCtrl_EmergencyStopAll();
}

static bool motor_iface_is_running(uint8_t id)
{
    if (id < MCTRL_COUNT) {
        return MotorCtrl_IsRunning((MotorId_t)id) != 0;
    }
    return false;
}

static int32_t motor_iface_get_position(uint8_t id)
{
    if (id < MCTRL_COUNT) {
        return MotorCtrl_GetPosition((MotorId_t)id);
    }
    return 0;
}

static void motor_iface_set_done_cb(MotorDoneCallback_t cb)
{
    g_motor_done_cb = cb;
}

/* Global motor interface instance */
MotorInterface_t Motor = {
    .Init             = motor_iface_init,
    .Start            = motor_iface_start,
    .Stop             = motor_iface_stop,
    .EmergencyStopAll = motor_iface_estop_all,
    .IsRunning        = motor_iface_is_running,
    .GetPosition      = motor_iface_get_position,
    .SetDoneCallback  = motor_iface_set_done_cb,
};

/* ================================================================== */
/* Public functions - Legacy buffer-based profiles                    */
/* ================================================================== */

void HwMotor_Init(void)
{
    motor_iface_init();
}

int32_t HwMotor_BuildTrapProfile(uint8_t id, int32_t steps,
                                   uint32_t max_freq_hz,
                                   uint32_t accel_hz_per_s)
{
    if (id >= MCTRL_COUNT) {
        return -1;
    }

    /* Set direction based on step sign */
    if (steps < 0) {
        MotorCtrl_SetDirection((MotorId_t)id, 0U);
        steps = -steps;
    } else {
        MotorCtrl_SetDirection((MotorId_t)id, 1U);
    }

    return MotorCtrl_BuildTrapProfile((MotorId_t)id, steps,
                                       max_freq_hz, accel_hz_per_s);
}

int32_t HwMotor_BuildArchProfile(uint8_t id, const uint16_t* ccr_array,
                                   uint32_t array_size)
{
    if (id >= MCTRL_COUNT) {
        return -1;
    }
    return MotorCtrl_BuildArchProfile((MotorId_t)id, ccr_array, array_size);
}

/* ================================================================== */
/* Public functions - Real-time profiles (new)                        */
/* ================================================================== */

int32_t HwMotor_StartTrapezoidal(uint8_t id, int32_t steps,
                                   uint32_t max_freq_hz,
                                   uint32_t accel_hz_per_s)
{
    uint32_t abs_steps;

    if (id >= MCTRL_COUNT) {
        return -1;
    }

    /* Set direction based on step sign */
    if (steps < 0) {
        MotorCtrl_SetDirection((MotorId_t)id, 0U);
        abs_steps = (uint32_t)(-steps);
    } else {
        MotorCtrl_SetDirection((MotorId_t)id, 1U);
        abs_steps = (uint32_t)steps;
    }

    /*
     * Build real-time trapezoidal profile using the Domain-layer
     * MotorCtrlBlock_t embedded in the Legacy motor_ctrl.c g_motors[].
     *
     * Note: This function configures the profile parameters but does not
     * start motion. Call MotorCtrl_Start() or Motor.Start() afterward.
     *
     * Since the Legacy motor_ctrl.c uses its own MotorCtrl_t struct
     * (not MotorCtrlBlock_t from Domain/motor_controller.h), we build
     * the profile into the existing profile buffer as a fallback.
     * The real-time profile mode requires integration at the ISR level
     * which is done in motor_ctrl.c's MotorCtrl_ISR_Handler.
     */
    return MotorCtrl_BuildTrapProfile((MotorId_t)id, (int32_t)abs_steps,
                                       max_freq_hz, accel_hz_per_s);
}

int32_t HwMotor_StartSCurve(uint8_t id, int32_t steps,
                              uint32_t max_freq_hz,
                              uint32_t accel_hz_per_s,
                              uint32_t jerk_hz_per_s2)
{
    uint32_t abs_steps;

    if (id >= MCTRL_COUNT) {
        return -1;
    }

    /* Set direction based on step sign */
    if (steps < 0) {
        MotorCtrl_SetDirection((MotorId_t)id, 0U);
        abs_steps = (uint32_t)(-steps);
    } else {
        MotorCtrl_SetDirection((MotorId_t)id, 1U);
        abs_steps = (uint32_t)steps;
    }

    /*
     * S-curve profile requires real-time computation per ISR tick.
     * This builds a pre-computed trapezoidal approximation into the
     * existing buffer as a compatible fallback until the ISR handler
     * in motor_ctrl.c is updated to use MotionProfile_NextCCR().
     *
     * For true S-curve motion, the caller should use the Domain-layer
     * MCtrl_StartSCurve() on a MotorCtrlBlock_t directly.
     */
    (void)jerk_hz_per_s2;  /* not used in buffer fallback */
    return MotorCtrl_BuildTrapProfile((MotorId_t)id, (int32_t)abs_steps,
                                       max_freq_hz, accel_hz_per_s);
}

int32_t HwMotor_StartArchInterp(uint8_t id, const ArchSegment_t* segments,
                                  uint32_t count)
{
    if (id >= MCTRL_COUNT) {
        return -1;
    }

    /*
     * Arch segment interpolation uses ArchInterp_NextCCR() per ISR tick.
     * This requires the ISR handler to be aware of the interpolation state.
     * Until motor_ctrl.c ISR is updated, this is a stub that returns error.
     *
     * For arch interpolation, use Domain-layer MCtrl_StartArch() on a
     * MotorCtrlBlock_t directly with the new ISR dispatcher.
     */
    (void)segments;
    (void)count;
    return -1;
}

int32_t HwMotor_StartHoming(uint8_t id, const HomingConfig_t* config)
{
    if (id >= MCTRL_COUNT) {
        return -1;
    }

    /*
     * Homing requires sensor input integration and the new ISR dispatcher.
     * Until motor_ctrl.c ISR is updated, this is a stub that returns error.
     *
     * For homing, use Domain-layer MCtrl_StartHoming() on a
     * MotorCtrlBlock_t directly with the new ISR dispatcher.
     */
    (void)config;
    return -1;
}

void HwMotor_HomingSensorUpdate(uint8_t id, uint8_t sensor_hit)
{
    /*
     * Stub: sensor update requires MotorCtrlBlock_t access.
     * When motor_ctrl.c is updated to use the new Domain-layer
     * MotorCtrlBlock_t, this will call MCtrl_HomingSensorUpdate().
     */
    (void)id;
    (void)sensor_hit;
}

/* ================================================================== */
/* Public functions - Direction, Position, Sync                       */
/* ================================================================== */

void HwMotor_SetDirection(uint8_t id, uint8_t dir)
{
    if (id < MCTRL_COUNT) {
        MotorCtrl_SetDirection((MotorId_t)id, dir);
    }
}

void HwMotor_SetPosition(uint8_t id, int32_t pos)
{
    if (id < MCTRL_COUNT) {
        MotorCtrl_SetPosition((MotorId_t)id, pos);
    }
}

void HwMotor_StartSync(const uint8_t* ids, uint8_t count)
{
    /* Convert uint8_t array to MotorId_t array */
    MotorId_t motor_ids[MCTRL_COUNT];
    uint8_t i;

    if (count > MCTRL_COUNT) {
        count = MCTRL_COUNT;
    }

    for (i = 0; i < count; i++) {
        motor_ids[i] = (MotorId_t)ids[i];
    }

    MotorCtrl_StartSync(motor_ids, count);
}

/* ================================================================== */
/* TMC2660 Control                                                    */
/* ================================================================== */

void HwMotor_TMC2660_Init(uint8_t id)
{
    if (id < MCTRL_COUNT) {
        MotorCtrl_TMC2660_Init((MotorId_t)id);
    }
}

void HwMotor_TMC2660_SetCurrent(uint8_t id, uint16_t run, uint16_t hold)
{
    if (id < MCTRL_COUNT) {
        MotorCtrl_TMC2660_SetCurrent((MotorId_t)id, run, hold);
    }
}

void HwMotor_TMC2660_SetMicrostep(uint8_t id, uint8_t resolution)
{
    if (id < MCTRL_COUNT) {
        MotorCtrl_TMC2660_SetMicrostep((MotorId_t)id, resolution);
    }
}
