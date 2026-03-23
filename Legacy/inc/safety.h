/*
*******************************************************************************************
* safety.h : Watchdog, emergency stop, and X-ray interlock safety module
*******************************************************************************************
* Copyright (C) 2016-2026 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date    : 2026-03-24
* @ Brief   : Safety-critical functions for dental X-ray imaging system.
*
*   Features:
*     1. IWDG (Independent Watchdog) — ~2 second timeout, LSI-clocked
*     2. Emergency stop handler — ISR-safe (no UART, no blocking)
*     3. X-ray hardware interlock readback verification
*     4. Safety flag management for main-loop error handling
*
*   IWDG Configuration:
*     - LSI clock: ~32 kHz (typical, varies 17-47 kHz across temperature)
*     - Prescaler: /32 → 1 kHz effective counter clock
*     - Reload: 2000 → ~2 second timeout
*     - Must be kicked in main loop; if main loop hangs, system resets
*
*   Memory: ~80 bytes Flash (.text), 4 bytes SRAM (.bss)
*
* @ Revision History :
*       1) initial creation. ------------------------------------ 2026-03-24
*******************************************************************************************
*/

/* Define to prevent recursive inclusion ---------------------------------- */
#ifndef __SAFETY_H__
#define __SAFETY_H__

/* Includes -------------------------------------------------------------- */
#include "stm32f2xx.h"
#include <stdint.h>

/* Exported define ------------------------------------------------------- */

/* IWDG prescaler and reload for ~2 second timeout.
 * LSI ~= 32 kHz (per RM0033 datasheet typical).
 * Prescaler /32 → effective clock = 1000 Hz.
 * Reload 2000 → timeout = 2000 / 1000 = 2.0 seconds.
 *
 * Worst case (LSI = 17 kHz): timeout = 2000 / (17000/32) = 3.76 s
 * Best case  (LSI = 47 kHz): timeout = 2000 / (47000/32) = 1.36 s
 */
#define SAFETY_IWDG_PRESCALER       IWDG_Prescaler_32
#define SAFETY_IWDG_RELOAD          2000U

/* X-ray interlock GPIO (matches tube.c defines):
 * Tube Ready    = GPIOC Pin 4
 * Tube Exposure = GPIOC Pin 5
 */
#define SAFETY_XRAY_PORT            GPIOC
#define SAFETY_XRAY_READY_PIN       GPIO_Pin_4
#define SAFETY_XRAY_EXPOSURE_PIN    GPIO_Pin_5

/* Error flags (bitmask) */
#define SAFETY_ERR_NONE             0x00U
#define SAFETY_ERR_EMERGENCY        0x01U
#define SAFETY_ERR_XRAY_INTERLOCK   0x02U
#define SAFETY_ERR_WATCHDOG         0x04U
#define SAFETY_ERR_MOTOR_FAULT      0x08U

/* Exported types -------------------------------------------------------- */

typedef struct {
    volatile uint8_t    error_flags;        /* Bitmask of SAFETY_ERR_xxx */
    volatile uint8_t    emergency_active;   /* 1 = EMG switch pressed */
    volatile uint8_t    emergency_set;      /* Write-only in ISR, cleared in main loop */
    uint8_t             iwdg_started;       /* 1 after IWDG init */
} Safety_State_t;

/* Exported variables ---------------------------------------------------- */
extern Safety_State_t g_safety;

/* Exported functions ---------------------------------------------------- */

/**
 * Initialize IWDG with ~2 second timeout.
 * Once started, IWDG cannot be stopped (hardware limitation).
 * Call once at startup after all peripheral init is complete.
 */
void Safety_IWDG_Init(void);

/**
 * Kick (reload) the IWDG counter.
 * Must be called from main loop at least once per timeout period.
 */
void Safety_IWDG_Kick(void);

/**
 * Check if last reset was caused by IWDG timeout.
 * @return 1 if watchdog reset occurred, 0 otherwise
 */
uint8_t Safety_WasWatchdogReset(void);

/**
 * Emergency stop handler — ISR-safe.
 * Actions (in order):
 *   1. Disable all motor timers via MotorCtrl_EmergencyStopAll()
 *   2. Disable X-ray: Tube Ready OFF, Tube Exposure OFF (direct GPIO)
 *   3. Set SAFETY_ERR_EMERGENCY flag
 *   4. Set g_emergency_flag for main loop notification
 *
 * This function does NOT call printUart or any blocking function.
 * All logging is deferred to main loop via debug_log ring buffer.
 *
 * Execution time: < 2 us at 120 MHz (direct register writes only).
 */
void Safety_EmergencyStop(void);

/**
 * Main-loop safety check routine.
 * Checks emergency flag, verifies X-ray interlock, handles deferred logging.
 * Call from main loop every iteration.
 */
void Safety_CheckMainLoop(void);

/**
 * Verify X-ray hardware interlock readback.
 * Reads back the Tube Ready and Tube Exposure GPIO pins and compares
 * against expected OFF state. Sets SAFETY_ERR_XRAY_INTERLOCK if mismatch.
 *
 * @return 1 if interlock is safe (both OFF), 0 if interlock fault detected
 */
uint8_t Safety_VerifyXrayInterlock(void);

/**
 * Get current emergency flag state.
 * @return 1 if emergency stop is active, 0 if clear
 */
uint8_t Safety_GetEmergencyFlag(void);

/**
 * Clear emergency flag after handling in main loop.
 * Does NOT re-enable motors or X-ray — that requires explicit restart.
 */
void Safety_ClearEmergencyFlag(void);

/**
 * Get current error flags bitmask.
 * @return Bitmask of SAFETY_ERR_xxx values
 */
uint8_t Safety_GetErrorFlags(void);

/**
 * Clear specific error flags.
 * @param flags  Bitmask of SAFETY_ERR_xxx to clear
 */
void Safety_ClearErrorFlags(uint8_t flags);

#endif /* __SAFETY_H__ */

/***************************** END OF FILE *****************************/
