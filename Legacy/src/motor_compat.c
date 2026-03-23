/*
*******************************************************************************************
* motor_compat.c : Compatibility layer — old Motor API -> new MotorCtrl API
*******************************************************************************************
* Copyright (C) 2016-2026 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date    : 2026-03-24
* @ Brief   : Thin wrapper implementations for gradual motor API migration.
*
*   Memory usage:
*     - .text: ~200 bytes Flash
*     - .bss:  0 bytes SRAM
*     - Stack: max ~16 bytes
*
* @ Revision History :
*       1) initial creation. ------------------------------------ 2026-03-24
*******************************************************************************************
*/

/* Include files --------------------------------------------------------- */
#include "motor_compat.h"
#include "debug_log.h"

/* Private define -------------------------------------------------------- */
/* Private variables ----------------------------------------------------- */
/* Private functions ----------------------------------------------------- */
/* Exported functions ---------------------------------------------------- */

MotorId_t Motor_GetId(Motor_Typedef* motor)
{
    if (motor == &Motor_R)   return MCTRL_R;
    if (motor == &Motor_V)   return MCTRL_V;
    if (motor == &Motor_C)   return MCTRL_C;
    if (motor == &Motor_T)   return MCTRL_T;
    if (motor == &Motor_S)   return MCTRL_S;
#ifdef USE_MOTOR_GANTRY_MS
    if (motor == &Motor_A)   return MCTRL_A;
#endif /* USE_MOTOR_GANTRY_MS */
#ifdef USE_MOTOR_CHINREST_VER
    if (motor == &Motor_CNS) return MCTRL_CNS;
#endif /* USE_MOTOR_CHINREST_VER */
#ifdef USE_MOTOR_CHINREST_HOR
    if (motor == &Motor_CWE) return MCTRL_CWE;
#endif /* USE_MOTOR_CHINREST_HOR */

    DbgLog_Put("COMPAT: Unknown motor pointer");
    return MCTRL_INVALID;
}

void Motor_MoveAbsolutePosition_New(Motor_Typedef* motor,
                                     int32_t steps,
                                     uint32_t max_freq,
                                     uint32_t accel)
{
    MotorId_t id = Motor_GetId(motor);
    if (id == MCTRL_INVALID) {
        return;
    }

    /* Set direction based on step sign */
    if (steps < 0) {
        MotorCtrl_SetDirection(id, 0U);  /* toward ORG */
        steps = -steps;
    } else {
        MotorCtrl_SetDirection(id, 1U);  /* toward LIMIT */
    }

    /* Build trapezoidal profile and start */
    if (MotorCtrl_BuildTrapProfile(id, steps, max_freq, accel) == 0) {
        MotorCtrl_Start(id);
    } else {
        DbgLog_Printf("COMPAT: Profile build failed for motor %d", (int)id);
    }
}

void Motor_MoveArchPosition_New(Motor_Typedef* motor,
                                 const uint16_t* arch_table,
                                 uint32_t size)
{
    MotorId_t id = Motor_GetId(motor);
    if (id == MCTRL_INVALID) {
        return;
    }

    if (MotorCtrl_BuildArchProfile(id, arch_table, size) == 0) {
        MotorCtrl_Start(id);
    } else {
        DbgLog_Printf("COMPAT: Arch profile overflow for motor %d", (int)id);
    }
}

void Motor_Stop_New(Motor_Typedef* motor)
{
    MotorId_t id = Motor_GetId(motor);
    if (id != MCTRL_INVALID) {
        MotorCtrl_Stop(id);
    }
}

uint8_t Motor_IsRunning_New(Motor_Typedef* motor)
{
    MotorId_t id = Motor_GetId(motor);
    if (id == MCTRL_INVALID) {
        return 0U;
    }
    return MotorCtrl_IsRunning(id);
}

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/
