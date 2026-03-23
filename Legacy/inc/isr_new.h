/*
*******************************************************************************************
* isr_new.h : Lightweight ISR handlers for T2 system
*******************************************************************************************
* Copyright (C) 2016-2026 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date    : 2026-03-24
* @ Brief   : Replacement ISR module with minimal handlers.
*
*   Design principles:
*     1. Motor ISRs delegate ALL work to MotorCtrl_ISR_Handler() (~30 cycles)
*     2. UART ISRs read byte + call parser only (no processing)
*     3. CAN ISR reads message + sets flag (process in main loop)
*     4. Timer ISRs increment counter only
*     5. Emergency ISR calls Safety_EmergencyStop() (no UART, no blocking)
*     6. NO printUart() calls in any ISR
*
*   ISR-to-Timer mapping (STM32F207 shared IRQ vectors):
*     TIM8_CC_IRQHandler           → Motor R (Rotation)
*     TIM1_CC_IRQHandler           → Motor V (Vertical)
*     TIM2_IRQHandler              → Motor C (Collimator)
*     TIM5_IRQHandler              → Motor T (Chin rest TS)
*     TIM1_UP_TIM10_IRQHandler     → Motor S (Slit / 2nd Collimator)
*     TIM1_BRK_TIM9_IRQHandler     → Motor A (Gantry)
*     TIM3_IRQHandler              → Motor CNS (Chin rest vertical)
*     TIM1_TRG_COM_TIM11_IRQHandler → Motor CWE (Chin rest horizontal)
*     TIM8_BRK_TIM12_IRQHandler    → Sensor P (CT pulsed trigger)
*     TIM8_TRG_COM_TIM14_IRQHandler → Tube (X-ray pulse)
*     TIM6_DAC_IRQHandler          → Internal timer (counter)
*     TIM7_IRQHandler              → Elapsed timer (ms counter)
*     EXTI9_5_IRQHandler           → Emergency switch
*
* @ Revision History :
*       1) initial creation. ------------------------------------ 2026-03-24
*******************************************************************************************
*/

/* Define to prevent recursive inclusion ---------------------------------- */
#ifndef __ISR_NEW_H__
#define __ISR_NEW_H__

/* Includes -------------------------------------------------------------- */
#include "stm32f2xx.h"

/* Exported define ------------------------------------------------------- */
/* Exported types -------------------------------------------------------- */
/* Exported variables ---------------------------------------------------- */

/* CAN receive message buffer (shared with main loop) */
extern CanRxMsg g_can_rx_msg;

/* Exported functions ---------------------------------------------------- */

/*
 * All ISR handler functions are defined in isr_new.c.
 * They are named per STM32F2xx startup file vector table
 * and do not need explicit extern declarations (weak symbol override).
 *
 * This header is provided for reference and documentation only.
 * Include it in any module that needs to reference ISR-shared data
 * (g_can_rx_msg, g_can_rx_flag).
 */

#endif /* __ISR_NEW_H__ */

/***************************** END OF FILE *****************************/
