/*
*******************************************************************************
* uart_hw.h : UART hardware abstraction layer
*******************************************************************************
* Copyright (C) 2014-2018 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Brief   : UART peripheral init, byte/string send, debug print
*
* @ Revision History :
*       1) Extracted from serial.c for modular build --------- 2026-03-24
*******************************************************************************
*/

#ifndef __UART_HW_H__
#define __UART_HW_H__

/* Includes ---------------------------------------------------------------- */
#include "extern.h"

/* Exported defines -------------------------------------------------------- */
#ifdef USE_TABLET_PC
#define UART_TABLET                 USART2
#define DBG_MSG_TABLET              UART_TABLET
#endif /* USE_TABLET_PC */
#define UART_PC                     USART1
#define DBG_MSG_PC                  UART_PC

/* Ring buffer for ISR-safe debug logging */
#define DBG_RING_BUF_SIZE           512

/* Exported functions ------------------------------------------------------ */

/**
 * Initialize UART1 (PC) and UART2 (Tablet) GPIO, clocks, NVIC, baudrate.
 * Called once from System_Init or main before any UART usage.
 */
void UART_HW_Config(void);

/**
 * Send a single byte (blocking, polled TC flag).
 */
void UART_SendByte(USART_TypeDef *uart, uint8_t byte);

/**
 * Send a null-terminated string followed by CR+LF (blocking).
 */
void UART_SendString(USART_TypeDef *uart, const char *str);

/**
 * Variadic printf-style debug output via UART (blocking).
 * Appends CR+LF. Max 128 chars per call.
 */
void printUart(USART_TypeDef *uart, char *format, ...);

/**
 * ISR-safe debug log: enqueue a message into a ring buffer.
 * The main loop must call DBG_FlushRingBuffer() to actually transmit.
 */
void DBG_LogFromISR(const char *msg);

/**
 * Flush the ISR-safe debug ring buffer out via UART_PC.
 * Call this periodically from the main loop.
 */
void DBG_FlushRingBuffer(void);

#endif /* __UART_HW_H__ */
