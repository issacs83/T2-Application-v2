/*
 * hw_serial.h : UART DMA + Ring Buffer wrapper with CommInterface_t
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Wraps the existing uart_hw.c functionality behind the CommInterface_t
 * abstraction. Also provides the legacy printUart/UART_SendMessage APIs
 * for backward compatibility during migration.
 */

#ifndef HW_SERIAL_H
#define HW_SERIAL_H

#include "stm32f2xx.h"
#include "hw_comm.h"
#include "../Domain/ring_buffer.h"

/* Initialize all UART hardware and populate CommInterface_t instances */
void HwSerial_Init(void);

/* Legacy API wrappers (delegate to existing uart_hw.c functions) */
void HwSerial_SendByte(USART_TypeDef* uart, uint8_t byte);
void HwSerial_SendString(USART_TypeDef* uart, const char* str);
void HwSerial_Printf(USART_TypeDef* uart, char* format, ...);
void HwSerial_SendMessage(USART_TypeDef* uart, char* string);

/* ISR-safe debug ring buffer (used by hw_isr.c) */
void HwSerial_LogFromISR(const char* msg);
void HwSerial_FlushDebugRing(void);

#endif /* HW_SERIAL_H */
