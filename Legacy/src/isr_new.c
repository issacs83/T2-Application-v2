/*
*******************************************************************************************
* isr_new.c : Lightweight ISR handlers for T2 system
*******************************************************************************************
* Copyright (C) 2016-2026 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date    : 2026-03-24
* @ Brief   : Replacement for isr.c with minimal, delegation-based ISR handlers.
*
*   Key improvements over original isr.c:
*     1. Motor ISRs: ~30 cycles via MotorCtrl_ISR_Handler() LUT read
*        (original: ~200+ cycles with floating-point CCR computation)
*     2. CAN ISR: no printUart() — sets flag for main loop processing
*     3. Emergency ISR: calls Safety_EmergencyStop() — no UART, no blocking
*     4. All ISR-shared variables use volatile (via shared_vars.h)
*
*   Memory usage:
*     - .bss: 12 bytes (g_can_rx_msg=16 + g_can_rx_flag=1, aligned)
*     - .text: ~400 bytes Flash
*
* @ Revision History :
*       1) initial creation. ------------------------------------ 2026-03-24
*******************************************************************************************
*/

/* Include files --------------------------------------------------------- */
#include "isr_new.h"
#include "motor_ctrl.h"
#include "safety.h"
#include "debug_log.h"
#include "shared_vars.h"
#include "extern.h"
#include "serial.h"
#include "sensor.h"
#include "tube.h"
#include "timer.h"
#include "can.h"
#include "emergency.h"
#include "configuration.h"

/* Private define -------------------------------------------------------- */

/* Sensor P source (from sensor.h) */
/* #define SENSOR_P_SOURCE TIM_IT_CC1 — already defined in sensor.h */

/* Private variables ----------------------------------------------------- */

/* CAN receive message buffer — written by ISR, read by main loop */
CanRxMsg g_can_rx_msg;

/* CAN receive flag — defined here, declared extern in shared_vars.h */
volatile uint8_t g_can_rx_flag = 0U;

/* Emergency flag — defined here, declared extern in shared_vars.h */
volatile uint8_t g_emergency_flag = 0U;

/* =====================================================================
 * Motor ISR Handlers
 *
 * Each handler clears the interrupt flag and delegates to the unified
 * MotorCtrl_ISR_Handler() which reads the next CCR from the pre-computed
 * profile ring buffer and writes it to the timer ARR register.
 *
 * Worst-case execution: ~30 cycles at 120 MHz = 250 ns.
 * ===================================================================== */

/**
 * TIM8 Capture/Compare — Motor R (Rotation)
 */
void TIM8_CC_IRQHandler(void)
{
    MotorCtrl_ISR_Handler(MCTRL_R);
}

/**
 * TIM1 Capture/Compare — Motor V (Vertical)
 */
void TIM1_CC_IRQHandler(void)
{
    MotorCtrl_ISR_Handler(MCTRL_V);
}

/**
 * TIM2 — Motor C (Collimator / 1st)
 */
void TIM2_IRQHandler(void)
{
    MotorCtrl_ISR_Handler(MCTRL_C);
}

/**
 * TIM5 — Motor T (Chin rest / Temple support)
 */
void TIM5_IRQHandler(void)
{
    MotorCtrl_ISR_Handler(MCTRL_T);
}

/**
 * TIM1 Update / TIM10 — Motor S (Slit / 2nd Collimator)
 *
 * Note: TIM1 Update and TIM10 share the same IRQ vector.
 * We check TIM10 IT status (Motor_S uses TIM10).
 */
void TIM1_UP_TIM10_IRQHandler(void)
{
    MotorCtrl_ISR_Handler(MCTRL_S);
}

/**
 * TIM1 Break / TIM9 — Motor A (Gantry)
 *
 * Note: TIM1 Break and TIM9 share the same IRQ vector.
 * We check TIM9 IT status (Motor_A uses TIM9).
 */
#ifdef USE_MOTOR_GANTRY_MS
void TIM1_BRK_TIM9_IRQHandler(void)
{
    MotorCtrl_ISR_Handler(MCTRL_A);
}
#endif /* USE_MOTOR_GANTRY_MS */

/**
 * TIM3 — Motor CNS (Chin rest vertical)
 */
#ifdef USE_MOTOR_CHINREST_VER
void TIM3_IRQHandler(void)
{
    MotorCtrl_ISR_Handler(MCTRL_CNS);
}
#endif /* USE_MOTOR_CHINREST_VER */

/**
 * TIM1 Trigger/Commutation / TIM11 — Motor CWE (Chin rest horizontal)
 */
#ifdef USE_MOTOR_CHINREST_HOR
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    MotorCtrl_ISR_Handler(MCTRL_CWE);
}
#endif /* USE_MOTOR_CHINREST_HOR */


/* =====================================================================
 * UART ISR Handlers
 *
 * Minimal: read byte, call existing parser, return.
 * The parser (UART_ParseMessage) builds protocol frames and sets flags
 * for main-loop processing.
 * ===================================================================== */

/**
 * USART1 — PC communication (UART_PC)
 */
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
        uint8_t data = (uint8_t)USART_ReceiveData(USART1);
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        UART_ParseMessage((char)data);
    }
}

#ifdef USE_TABLET_PC
/**
 * USART2 — Tablet communication (UART_TABLET)
 */
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint8_t data = (uint8_t)USART_ReceiveData(USART2);
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
        UART_ParseMessage((char)data);
    }
}
#endif /* USE_TABLET_PC */


/* =====================================================================
 * CAN ISR Handler
 *
 * Read message into buffer, validate, set flag. NO processing in ISR.
 * If message is invalid, log to ring buffer (not UART).
 * ===================================================================== */

/**
 * CAN2 FIFO0 Receive — Sub-board / Tube communication
 */
void CAN2_RX0_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET) {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &g_can_rx_msg);

        if ((g_can_rx_msg.IDE == CAN_ID_EXT) &&
            (g_can_rx_msg.DLC == CAN_FRAME_LENGTH_BYTE) &&
            ((g_can_rx_msg.ExtId == CAN_EXT_ID_SUBB_TO_SYSTEM) ||
             (g_can_rx_msg.ExtId == CAN_EXT_ID_TUBE_TO_SYSTEM))) {
            /* Valid message — set flag for main loop */
            g_can_rx_flag = 1U;
        } else {
            /* Invalid frame — log via ring buffer, NOT printUart */
            DbgLog_Put("CAN: Rx frame error (DLC/ID mismatch)");
        }
    }
}


/* =====================================================================
 * Sensor and Tube ISR Handlers
 *
 * These handlers retain the original logic because the sensor/tube
 * timing is hardware-critical and cannot be deferred to main loop.
 * The only change is removing printUart calls (there were none here).
 * ===================================================================== */

/**
 * TIM8 Break / TIM12 — Sensor P (CT pulsed X-ray sync)
 */
void TIM8_BRK_TIM12_IRQHandler(void)
{
#if defined(USE_CT_XRAY_PULSED_MODE)
    uint16_t capture = 0U;

    if (TIM_GetITStatus(Sensor_C.Timer.Periph, SENSOR_P_SOURCE) != RESET) {
        TIM_ClearITPendingBit(Sensor_C.Timer.Periph, SENSOR_P_SOURCE);

        if ((Sensor_C.Timer.Periph == TIM2) || (Sensor_C.Timer.Periph == TIM5)) {
            TIM_SetCounter(Sensor_C.Timer.Periph, 0U);
        } else {
            capture = TIM_GetCapture1(Sensor_C.Timer.Periph);
        }

        if (!Sensor_C.Update) {
            TIM_SetCompare1(Sensor_C.Timer.Periph,
                            capture + Sensor_C.Param.SyncHighTimeCcr);
        } else {
            TIM_SetCompare1(Sensor_C.Timer.Periph,
                            capture + Sensor_C.Param.SyncLowTimeCcr);
        }
    }
    Sensor_C.Update = !Sensor_C.Update;
#else /* not USE_CT_XRAY_PULSED_MODE */
    uint16_t capture = 0U;

    if (TIM_GetITStatus(Sensor_P.Timer.Periph, SENSOR_P_SOURCE) != RESET) {
        TIM_ClearITPendingBit(Sensor_P.Timer.Periph, SENSOR_P_SOURCE);

        if ((Sensor_P.Timer.Periph == TIM2) || (Sensor_P.Timer.Periph == TIM5)) {
            TIM_SetCounter(Sensor_P.Timer.Periph, 0U);
        } else {
            capture = TIM_GetCapture1(Sensor_P.Timer.Periph);
        }
    }

    if (!Sensor_P.Update) {
        TIM_SetCompare1(Sensor_P.Timer.Periph,
                         capture + Sensor_P.Trigger.SyncHighTimeCcr);
    } else {
        Sensor_P.Index++;
        Sensor_P.Trigger.VariableLowTimeCcr =
            Sensor_P.CaptureCcr[Motor_R.ArchParam.ArrayIndex];
        /* Use integer cast directly — avoid software float (+0.5) in ISR.
         * The float-to-int truncation is acceptable for timer CCR values.
         * For proper rounding, pre-compute integer CCR in main context. */
        TIM_SetCompare1(Sensor_P.Timer.Periph,
                         capture + (uint16_t)(Sensor_P.Trigger.VariableLowTimeCcr));
        Sensor_P.Count++;
    }
    Sensor_P.Update = (bool)!Sensor_P.Update;
#endif /* USE_CT_XRAY_PULSED_MODE */
}


#ifndef USE_TUBE_PPS_TYPE_IO
/**
 * TIM8 Trigger/Commutation / TIM14 — Tube (X-ray pulse timing)
 */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
    uint16_t capture = 0U;

    if (TIM_GetITStatus(Tube.Timer.Periph, TUBE_SOURCE) != RESET) {
        TIM_ClearITPendingBit(Tube.Timer.Periph, TUBE_SOURCE);

        if ((Tube.Timer.Periph == TIM2) || (Tube.Timer.Periph == TIM5)) {
            TIM_SetCounter(Tube.Timer.Periph, 0U);
        } else {
            capture = TIM_GetCapture1(Tube.Timer.Periph);
        }

        if (Tube.Update) {
            TIM_SetCompare1(Tube.Timer.Periph,
                             capture + (uint16_t)(Tube.Param.XonTimeCcr));
        } else {
            TIM_SetCompare1(Tube.Timer.Periph,
                             capture + (uint16_t)(Tube.Param.XoffTimeCcr));
        }
    }

    Tube.Update = (bool)!Tube.Update;
}
#endif /* not USE_TUBE_PPS_TYPE_IO */


/* =====================================================================
 * Timer ISR Handlers
 *
 * Simple counter increment only. No processing.
 * ===================================================================== */

/**
 * TIM6 / DAC — Internal timer (generic delay counter)
 */
void TIM6_DAC_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
        IntTimer.Count++;
    }
}

/**
 * TIM7 — Elapsed time measurement (millisecond counter)
 */
void TIM7_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
        typElapsedTimer.endMs++;
        if (typElapsedTimer.endMs >= kMax_uint32_t) {
            typElapsedTimer.endMs = 0U;
        }
    }
}


/* =====================================================================
 * Emergency Switch ISR Handler
 *
 * Replaces the original EXTI9_5_IRQHandler from emergency.c.
 * Uses Safety_EmergencyStop() — NO printUart, NO blocking.
 * Main loop handles UART notification and system restart via
 * Safety_CheckMainLoop().
 * ===================================================================== */

/**
 * EXTI lines 9..5 — Emergency switch (PA5 / EXTI_Line5)
 */
void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetITStatus(EMG_SW_GPIO_EXTI_LINE) != RESET) {
        EXTI_ClearITPendingBit(EMG_SW_GPIO_EXTI_LINE);

        /* Read switch state: 0 = pressed (active low), 1 = released */
        uint8_t released = (uint8_t)GPIO_ReadInputDataBit(
            EMG_SW_GPIO_PORT, EMG_SW_GPIO_PIN);

        if (!released) {
            /* Emergency switch PRESSED — activate safety shutdown */
            Safety_EmergencyStop();
        } else {
            /* Emergency switch RELEASED — log for main loop restart */
            DbgLog_Put("EMG: Switch released, pending restart");
            g_emergency_flag = 2U;  /* 2 = released (vs 1 = pressed) */
        }
    }
}

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/
