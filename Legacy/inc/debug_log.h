/*
*******************************************************************************************
* debug_log.h : ISR-safe debug logging via ring buffer
*******************************************************************************************
* Copyright (C) 2016-2026 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date    : 2026-03-24
* @ Brief   : Lock-free ring buffer for debug messages originating from ISR.
*             Messages are enqueued with DbgLog_Put() (ISR-safe, non-blocking)
*             and flushed to UART in main loop with DbgLog_Flush().
*
*   Design:
*     - Single-producer / single-consumer ring buffer (no lock needed if
*       only ISR writes and main loop reads, or vice versa).
*     - For multiple ISR producers: short critical section on write.
*     - Fixed max message length to avoid variable-length complexity in ISR.
*     - DbgLog_Printf() is main-loop ONLY (uses vsnprintf).
*
*   Memory: 1024 bytes ring buffer + 16 bytes overhead = ~1040 bytes SRAM.
*
* @ Revision History :
*       1) initial creation. ------------------------------------ 2026-03-24
*******************************************************************************************
*/

/* Define to prevent recursive inclusion ---------------------------------- */
#ifndef __DEBUG_LOG_H__
#define __DEBUG_LOG_H__

/* Includes -------------------------------------------------------------- */
#include <stdint.h>

/* Exported define ------------------------------------------------------- */

/* Ring buffer total size (must be power of 2) */
#define DBGLOG_BUF_SIZE         1024U
#define DBGLOG_BUF_MASK         (DBGLOG_BUF_SIZE - 1U)

/* Maximum length of a single message (including null terminator) */
#define DBGLOG_MSG_MAX_LEN      80U

/* Exported types -------------------------------------------------------- */
typedef struct {
    volatile uint32_t   wr_idx;     /* Write index (ISR side) */
    volatile uint32_t   rd_idx;     /* Read index (main loop side) */
    uint8_t             buf[DBGLOG_BUF_SIZE];
} DbgLog_t;

/* Exported variables ---------------------------------------------------- */
extern DbgLog_t g_dbg_log;

/* Exported functions ---------------------------------------------------- */

/**
 * Initialize the debug log ring buffer.
 * Call once at startup before any ISR can fire.
 */
void DbgLog_Init(void);

/**
 * Enqueue a fixed-length message string into the ring buffer.
 * ISR-safe, non-blocking. If buffer is full, message is silently dropped.
 *
 * @param msg  Null-terminated string (max DBGLOG_MSG_MAX_LEN-1 chars used)
 */
void DbgLog_Put(const char* msg);

/**
 * Format and enqueue a message. Main-loop context ONLY (uses vsnprintf).
 * NOT safe to call from ISR.
 *
 * @param fmt  printf-style format string
 * @param ...  variadic arguments
 */
void DbgLog_Printf(const char* fmt, ...);

/**
 * Flush all queued messages to UART (DBG_MSG_PC).
 * Call from main loop periodically.
 *
 * @return Number of bytes flushed
 */
uint32_t DbgLog_Flush(void);

/**
 * @return Number of bytes currently queued in the ring buffer.
 */
uint32_t DbgLog_Pending(void);

#endif /* __DEBUG_LOG_H__ */

/***************************** END OF FILE *****************************/
