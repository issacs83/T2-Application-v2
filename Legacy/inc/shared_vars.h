/*
*******************************************************************************************
* shared_vars.h : ISR-shared variable declarations (volatile extern)
*******************************************************************************************
* Copyright (C) 2016-2026 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date    : 2026-03-24
* @ Brief   : Centralized extern declarations for all variables shared between
*             ISR context and main-loop context. Every variable listed here
*             MUST be declared volatile to prevent compiler reordering.
*
*   Rules:
*     1. extern declarations ONLY in this header.
*     2. Definitions (storage) go in exactly ONE .c file (noted per variable).
*     3. All types must be naturally aligned (no packed structs across ISR boundary).
*     4. Access wider than bus width (>32-bit on Cortex-M3) must use critical section.
*
* @ Revision History :
*       1) initial creation. ------------------------------------ 2026-03-24
*******************************************************************************************
*/

/* Define to prevent recursive inclusion ---------------------------------- */
#ifndef __SHARED_VARS_H__
#define __SHARED_VARS_H__

/* Includes -------------------------------------------------------------- */
#include "stm32f2xx.h"
#include "system.h"

#ifndef __cplusplus
#ifndef bool
typedef enum {FALSE = 0, TRUE = !FALSE} bool;
#endif /* not bool */
#endif /* not __cplusplus */

/* -----------------------------------------------------------------------
 * Variable                  | Defined in     | Set by     | Read by
 * --------------------------+----------------+------------+-----------
 * CurCaptureMode            | main.c         | main loop  | ISR (motor_R)
 * g_bTiltStatus             | system.c       | main loop  | ISR / main
 * nHallSensorCount          | misc1.c        | ISR        | main loop
 * bColumnStop               | misc1.c        | ISR        | main loop
 * bTsMotorInterlock         | misc1.c        | main loop  | ISR
 * limit_count               | misc1.c        | main loop  | ISR
 * g_can_rx_flag             | isr_new.c      | CAN ISR    | main loop
 * g_emergency_flag          | safety.c       | EMG ISR    | main loop
 * ----------------------------------------------------------------------- */

/* Capture mode — set in main loop, read in Motor_R ISR */
extern volatile OpStatus_t CurCaptureMode;

/* Tilt status — shared between main loop and ISR context */
extern volatile bool g_bTiltStatus;

/* Hall sensor step counter — incremented in ISR */
extern volatile uint32_t nHallSensorCount;

/* Column stop flag — set in ISR when hall count reaches limit */
extern volatile char bColumnStop;

/* Temple support motor interlock — set in main loop, checked in ISR */
extern volatile char bTsMotorInterlock;

/* Hall sensor limit count threshold — set in main loop */
extern volatile uint32_t limit_count;

/* CAN receive flag — set in CAN ISR, cleared in main loop */
extern volatile uint8_t g_can_rx_flag;

/* Emergency stop flag — set in EMG ISR, cleared in main loop */
extern volatile uint8_t g_emergency_flag;

#endif /* __SHARED_VARS_H__ */

/***************************** END OF FILE *****************************/
