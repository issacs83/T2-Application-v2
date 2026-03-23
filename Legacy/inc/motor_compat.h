/*
*******************************************************************************************
* motor_compat.h : Compatibility layer — old Motor API -> new MotorCtrl API
*******************************************************************************************
* Copyright (C) 2016-2026 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date    : 2026-03-24
* @ Brief   : Thin wrappers that map the legacy Motor_Typedef-based API to the
*             new MotorCtrl_* functions. Allows gradual migration of capture
*             modules without rewriting all callers at once.
*
*   Usage:
*     - Include this header in capture modules that need both old and new APIs.
*     - Call Motor_GetId() to translate a Motor_Typedef pointer to MotorId_t.
*     - Use the _New() wrapper functions when ready to switch individual calls.
*     - The old motor.h API remains fully functional alongside these wrappers.
*
*   Memory: ~60 bytes Flash (.text), 0 bytes SRAM.
*
* @ Revision History :
*       1) initial creation. ------------------------------------ 2026-03-24
*******************************************************************************************
*/

/* Define to prevent recursive inclusion ---------------------------------- */
#ifndef __MOTOR_COMPAT_H__
#define __MOTOR_COMPAT_H__

/* Includes -------------------------------------------------------------- */
#include "motor_ctrl.h"
#include "motor.h"

/* Exported types -------------------------------------------------------- */
/* Exported define ------------------------------------------------------- */

/* Invalid motor ID sentinel */
#define MCTRL_INVALID   ((MotorId_t)0xFF)

/* Exported functions ---------------------------------------------------- */

/**
 * Map old Motor_Typedef pointer to new MotorId_t enum.
 * Compares pointer address against known global Motor_R, Motor_V, etc.
 *
 * @param motor  Pointer to legacy Motor_Typedef instance
 * @return       MotorId_t enum value, or MCTRL_INVALID if not recognized
 */
MotorId_t Motor_GetId(Motor_Typedef* motor);

/**
 * Wrapper: old Motor_MoveAbsolutePosition -> new MotorCtrl API.
 * Builds a trapezoidal profile and starts the motor.
 *
 * @param motor      Legacy motor struct pointer
 * @param steps      Total step count
 * @param max_freq   Maximum frequency (Hz)
 * @param accel      Acceleration (Hz/s)
 */
void Motor_MoveAbsolutePosition_New(Motor_Typedef* motor,
                                     int32_t steps,
                                     uint32_t max_freq,
                                     uint32_t accel);

/**
 * Wrapper: old Motor_MoveArchPosition -> new MotorCtrl arch profile.
 *
 * @param motor       Legacy motor struct pointer
 * @param arch_table  Pre-computed CCR array
 * @param size        Number of entries
 */
void Motor_MoveArchPosition_New(Motor_Typedef* motor,
                                 const uint16_t* arch_table,
                                 uint32_t size);

/**
 * Wrapper: old Motor_Stop -> new MotorCtrl_Stop.
 *
 * @param motor  Legacy motor struct pointer
 */
void Motor_Stop_New(Motor_Typedef* motor);

/**
 * Check if motor is running via new API.
 *
 * @param motor  Legacy motor struct pointer
 * @return       1 if running, 0 if idle/done
 */
uint8_t Motor_IsRunning_New(Motor_Typedef* motor);

#endif /* __MOTOR_COMPAT_H__ */

/***************************** END OF FILE *****************************/
