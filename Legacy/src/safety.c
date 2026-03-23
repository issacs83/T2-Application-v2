/*
*******************************************************************************************
* safety.c : Watchdog, emergency stop, and X-ray interlock safety module
*******************************************************************************************
* Copyright (C) 2016-2026 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date    : 2026-03-24
* @ Brief   : Implementation of safety-critical functions.
*
*   Memory usage:
*     - .bss: 4 bytes (Safety_State_t)
*     - .text: ~280 bytes Flash
*     - Stack: max ~16 bytes (no deep call chains)
*
* @ Revision History :
*       1) initial creation. ------------------------------------ 2026-03-24
*******************************************************************************************
*/

/* Include files --------------------------------------------------------- */
#include "safety.h"
#include <stdbool.h>
#include "motor_ctrl.h"
#include "tube.h"
#include "debug_log.h"
#include "shared_vars.h"
#include "serial.h"
#include "error_code.h"

/* Private define -------------------------------------------------------- */

/* Direct GPIO write macros for ISR context (bypass HAL overhead).
 * BSRRL sets pin (active low = disable for tube control).
 * BSRRH clears pin.
 *
 * Tube ready/exposure are active-low (Bit_RESET = enable, Bit_SET = disable).
 * So to DISABLE, we SET the pin → write to BSRRL.
 */
#define XRAY_READY_OFF()    (SAFETY_XRAY_PORT->BSRRL = SAFETY_XRAY_READY_PIN)
#define XRAY_EXPOSURE_OFF() (SAFETY_XRAY_PORT->BSRRL = SAFETY_XRAY_EXPOSURE_PIN)

/* Readback: 0 means enabled (active low), non-zero means disabled (safe) */
#define XRAY_READY_IS_OFF() \
    (GPIO_ReadInputDataBit(SAFETY_XRAY_PORT, SAFETY_XRAY_READY_PIN) != Bit_RESET)
#define XRAY_EXPOSURE_IS_OFF() \
    (GPIO_ReadInputDataBit(SAFETY_XRAY_PORT, SAFETY_XRAY_EXPOSURE_PIN) != Bit_RESET)

/* Private variables ----------------------------------------------------- */

Safety_State_t g_safety;

/* Private functions ----------------------------------------------------- */
/* Exported functions ---------------------------------------------------- */

void Safety_IWDG_Init(void)
{
    /* Check if previous reset was from IWDG */
    if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET) {
        g_safety.error_flags |= SAFETY_ERR_WATCHDOG;
        RCC_ClearFlag();
    }

    /* Enable write access to IWDG_PR and IWDG_RLR registers */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

    /* Set prescaler: LSI / 32 = ~1 kHz */
    IWDG_SetPrescaler(SAFETY_IWDG_PRESCALER);

    /* Set reload value: 2000 counts = ~2 seconds */
    IWDG_SetReload(SAFETY_IWDG_RELOAD);

    /* Reload counter before enabling */
    IWDG_ReloadCounter();

    /* Enable IWDG — once enabled, cannot be disabled until reset */
    IWDG_Enable();

    g_safety.iwdg_started = 1U;
}

void Safety_IWDG_Kick(void)
{
    if (g_safety.iwdg_started) {
        IWDG_ReloadCounter();
    }
}

uint8_t Safety_WasWatchdogReset(void)
{
    return (g_safety.error_flags & SAFETY_ERR_WATCHDOG) ? 1U : 0U;
}

void Safety_EmergencyStop(void)
{
    /*
     * CRITICAL: This function may be called from ISR (EXTI9_5_IRQHandler).
     * No UART output, no blocking calls, no heap allocation.
     *
     * Step 1: Kill all motor timers immediately.
     *         MotorCtrl_EmergencyStopAll() uses direct CR1 register writes
     *         to disable all 8 timer peripherals. (~500 ns at 120 MHz)
     */
    MotorCtrl_EmergencyStopAll();

    /*
     * Step 2: Disable X-ray output via direct GPIO register write.
     *         Active-low control: SET pin = disable output.
     *         Two register writes: ~50 ns total.
     */
    XRAY_READY_OFF();
    XRAY_EXPOSURE_OFF();

    /*
     * Step 3: Set dedicated write-only flag for main loop to merge.
     *         Avoids non-atomic RMW (LDRB+ORR+STRB) on error_flags in ISR.
     *         Main loop merges into error_flags under __disable_irq().
     */
    g_safety.emergency_set = 1U;
    g_safety.emergency_active = 1U;

    /*
     * Step 4: Set shared flag for main loop notification.
     */
    g_emergency_flag = 1U;

    /*
     * Step 5: Log via ISR-safe ring buffer (not UART).
     *         DbgLog_Put uses short critical section internally.
     */
    DbgLog_Put("EMG: Emergency stop activated");
}

void Safety_CheckMainLoop(void)
{
    static bool emergency_logged = false;

    /* Kick watchdog FIRST — before any UART or blocking calls */
    Safety_IWDG_Kick();

    /*
     * Merge ISR-written emergency_set into error_flags atomically.
     * emergency_set is only WRITTEN (=1) in ISR, only CLEARED (=0) here.
     * No RMW race possible.
     */
    if (g_safety.emergency_set) {
        __disable_irq();
        g_safety.error_flags |= SAFETY_ERR_EMERGENCY;
        g_safety.emergency_set = 0U;
        __enable_irq();
    }

    /* Check and handle emergency flag — log only once */
    if (g_safety.emergency_active) {
        if (!emergency_logged) {
            /* Deferred UART logging — safe in main loop context */
            printUart(DBG_MSG_PC, "SAFETY: Emergency stop active, all motors disabled");
            UART_SendMessage(DBG_MSG_PC, EMERGENCY_SWITCH_ON);
#ifdef USE_TABLET_PC
            UART_SendMessage(DBG_MSG_TABLET, EMERGENCY_SWITCH_ON);
#endif /* USE_TABLET_PC */
            emergency_logged = true;
        }
    } else {
        /* Reset log-once flag when emergency is cleared */
        emergency_logged = false;
    }

    /* Verify X-ray interlock state */
    if (g_safety.emergency_active || (g_safety.error_flags & SAFETY_ERR_EMERGENCY)) {
        if (!Safety_VerifyXrayInterlock()) {
            /* X-ray still active despite emergency — force disable again */
            XRAY_READY_OFF();
            XRAY_EXPOSURE_OFF();
            DbgLog_Put("EMG: X-ray interlock fault, forced disable");
        }
    }

    /* Flush any queued ISR debug messages to UART */
    DbgLog_Flush();
}

uint8_t Safety_VerifyXrayInterlock(void)
{
    uint8_t ready_off = XRAY_READY_IS_OFF();
    uint8_t exposure_off = XRAY_EXPOSURE_IS_OFF();

    if (ready_off && exposure_off) {
        /* Interlock verified — both outputs are safely disabled */
        g_safety.error_flags &= (uint8_t)~SAFETY_ERR_XRAY_INTERLOCK;
        return 1U;
    }

    /* Interlock fault detected */
    g_safety.error_flags |= SAFETY_ERR_XRAY_INTERLOCK;

    if (!ready_off) {
        DbgLog_Put("SAFETY: Tube Ready pin stuck active");
    }
    if (!exposure_off) {
        DbgLog_Put("SAFETY: Tube Exposure pin stuck active");
    }

    return 0U;
}

uint8_t Safety_GetEmergencyFlag(void)
{
    return g_safety.emergency_active;
}

void Safety_ClearEmergencyFlag(void)
{
    g_safety.emergency_active = 0U;
    g_safety.error_flags &= (uint8_t)~SAFETY_ERR_EMERGENCY;
    g_emergency_flag = 0U;
}

uint8_t Safety_GetErrorFlags(void)
{
    return g_safety.error_flags;
}

void Safety_ClearErrorFlags(uint8_t flags)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    g_safety.error_flags &= (uint8_t)~flags;
    __set_PRIMASK(primask);
}

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/
