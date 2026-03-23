/*
 * hw_isr.h : All ISR handlers (ultra-lightweight, Port layer)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Re-exports isr_new.h for documentation. All ISR handlers are
 * defined in the existing src/isr_new.c which is part of the Port layer.
 *
 * ISR handlers delegate to:
 *   - Motor timers -> MotorCtrl_ISR_Handler() (motor_ctrl.c)
 *   - UART RX      -> UART_ParseMessage() (msg_protocol.c)
 *   - CAN RX       -> flag set for main loop
 *   - Emergency     -> Safety_EmergencyStop() (safety.c)
 *   - Sensor/Tube   -> direct timer manipulation (timing-critical)
 *   - Delay timers  -> counter increment only
 */

#ifndef HW_ISR_H
#define HW_ISR_H

#include "isr_new.h"

/* All ISR functions are declared via weak symbols in the startup file.
 * The implementations in isr_new.c override them.
 * No additional declarations needed here. */

#endif /* HW_ISR_H */
