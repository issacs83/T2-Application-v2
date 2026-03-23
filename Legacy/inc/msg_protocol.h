/*
*******************************************************************************
* msg_protocol.h : UART message protocol and queue
*******************************************************************************
* Copyright (C) 2014-2018 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Brief   : 16-byte bracket message framing, queue, real-time dispatch
*
* @ Revision History :
*       1) Extracted from serial.c/serial.h ------------------- 2026-03-24
*******************************************************************************
*/

#ifndef __MSG_PROTOCOL_H__
#define __MSG_PROTOCOL_H__

/* Includes ---------------------------------------------------------------- */
#include "extern.h"

/* Exported defines -------------------------------------------------------- */
#define UART_MSG_SIZE       16
#define UART_QUEUE_SIZE     20

/* Direction prefixes:
 *   SM_ = PC -> SCB (System Master to SCB)
 *   SP_ = SCB -> PC (SCB to PC)
 *   MB_ = Membrane -> SCB
 *   MP_ = SCB -> Membrane (via Tablet UART)
 *   EM_ = Echo/Mirror (debug echo)
 */

/* Exported types ---------------------------------------------------------- */
typedef struct {
    char Data[UART_MSG_SIZE];
} UART_MsgTypedef;

typedef struct {
    UART_MsgTypedef Message[UART_QUEUE_SIZE];
    volatile uint32_t Head;
    volatile uint32_t Tail;
} UART_QueueTypedef;

/* Exported variables ------------------------------------------------------ */

/* Main command queue and list queue (accessible from handler modules) */
extern UART_QueueTypedef UART_Queue;
extern UART_QueueTypedef UART_LIST_Queue;

/* Exported functions ------------------------------------------------------ */

/**
 * Called from ISR: parse one incoming byte, frame into 16-byte bracket
 * messages, enqueue into UART_Queue. Minimal work in ISR context.
 */
void UART_ParseMessage(char data);

/**
 * Initialize the message queues (Head=Tail=0, clear buffers).
 */
void UART_QueueInitial(void);

/**
 * Get the number of messages currently in the queue.
 */
uint32_t MSG_QueueCnt(UART_QueueTypedef *queue);

/**
 * Enqueue a message. Returns SET on success, RESET on overflow.
 */
bool MSG_Enqueue(UART_QueueTypedef *queue, UART_MsgTypedef message);

/**
 * Dequeue a message. Caller must check MSG_QueueCnt > 0 first.
 */
UART_MsgTypedef MSG_Dequeue(UART_QueueTypedef *queue);

/**
 * Enqueue into the LIST queue (no overflow check -- diagnostic log).
 */
void MSG_LIST_Enqueue(UART_QueueTypedef *queue, UART_MsgTypedef message);

/**
 * Send a formatted 16-byte bracket message response via UART.
 * string must be a complete bracket message e.g. "[SP_MODE_RESET]"
 */
void UART_SendMessage(USART_TypeDef *uart, char *string);

/**
 * Blocking wait: dequeue and match against str. Returns SET if found.
 */
char UART_WaitProtocol(const char *str);

#endif /* __MSG_PROTOCOL_H__ */
