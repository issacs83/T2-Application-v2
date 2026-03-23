/*
*******************************************************************************************
* debug_log.c : ISR-safe debug logging via ring buffer
*******************************************************************************************
* Copyright (C) 2016-2026 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date    : 2026-03-24
* @ Brief   : Implementation of lock-free ring buffer debug logger.
*
*   Memory usage:
*     - .bss: 1024 (buffer) + 8 (indices) = 1032 bytes SRAM
*     - .text: ~300 bytes Flash
*     - Stack: DbgLog_Printf uses ~120 bytes (vsnprintf + local buffer)
*
* @ Revision History :
*       1) initial creation. ------------------------------------ 2026-03-24
*******************************************************************************************
*/

/* Include files --------------------------------------------------------- */
#include "debug_log.h"
#include "serial.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Private define -------------------------------------------------------- */
/* Private variables ----------------------------------------------------- */

DbgLog_t g_dbg_log;

/* Private functions ----------------------------------------------------- */

/**
 * Write a single byte to the ring buffer.
 * @return 1 on success, 0 if buffer full
 */
static uint32_t dbglog_write_byte(uint8_t byte)
{
    uint32_t next_wr = (g_dbg_log.wr_idx + 1U) & DBGLOG_BUF_MASK;

    /* Check if buffer is full */
    if (next_wr == g_dbg_log.rd_idx) {
        return 0U;
    }

    g_dbg_log.buf[g_dbg_log.wr_idx] = byte;
    g_dbg_log.wr_idx = next_wr;
    return 1U;
}

/* Exported functions ---------------------------------------------------- */

void DbgLog_Init(void)
{
    g_dbg_log.wr_idx = 0U;
    g_dbg_log.rd_idx = 0U;
    memset(g_dbg_log.buf, 0, DBGLOG_BUF_SIZE);
}

void DbgLog_Put(const char* msg)
{
    uint32_t primask;
    uint32_t i;

    if (msg == NULL) {
        return;
    }

    /* Enter critical section (disable interrupts).
     * Required because multiple ISRs might call DbgLog_Put concurrently. */
    primask = __get_PRIMASK();
    __disable_irq();

    /* Write message bytes followed by newline delimiter.
     * Limit to DBGLOG_MSG_MAX_LEN-1 chars to prevent runaway. */
    for (i = 0U; i < (DBGLOG_MSG_MAX_LEN - 1U); i++) {
        if (msg[i] == '\0') {
            break;
        }
        if (!dbglog_write_byte((uint8_t)msg[i])) {
            /* Buffer full — drop remainder silently */
            break;
        }
    }

    /* Append newline as message delimiter */
    dbglog_write_byte((uint8_t)'\n');

    /* Restore interrupt state */
    __set_PRIMASK(primask);
}

void DbgLog_Printf(const char* fmt, ...)
{
    char tmp_buf[DBGLOG_MSG_MAX_LEN];
    va_list args;

    if (fmt == NULL) {
        return;
    }

    va_start(args, fmt);
    vsnprintf(tmp_buf, sizeof(tmp_buf), fmt, args);
    va_end(args);

    /* Null-terminate safety */
    tmp_buf[DBGLOG_MSG_MAX_LEN - 1U] = '\0';

    /* Enqueue via the ISR-safe path (uses critical section internally) */
    DbgLog_Put(tmp_buf);
}

uint32_t DbgLog_Flush(void)
{
    uint32_t count = 0U;
    uint8_t line_buf[DBGLOG_MSG_MAX_LEN];
    uint32_t line_idx = 0U;

    while (g_dbg_log.rd_idx != g_dbg_log.wr_idx) {
        uint8_t byte = g_dbg_log.buf[g_dbg_log.rd_idx];
        g_dbg_log.rd_idx = (g_dbg_log.rd_idx + 1U) & DBGLOG_BUF_MASK;
        count++;

        if (byte == (uint8_t)'\n') {
            /* End of one message — send it via UART */
            line_buf[line_idx] = '\0';
            if (line_idx > 0U) {
                printUart(DBG_MSG_PC, "%s", (char*)line_buf);
            }
            line_idx = 0U;
        } else {
            if (line_idx < (DBGLOG_MSG_MAX_LEN - 1U)) {
                line_buf[line_idx] = byte;
                line_idx++;
            }
            /* else: truncate silently */
        }
    }

    /* Flush any remaining partial message (no trailing newline) */
    if (line_idx > 0U) {
        line_buf[line_idx] = '\0';
        printUart(DBG_MSG_PC, "%s", (char*)line_buf);
    }

    return count;
}

uint32_t DbgLog_Pending(void)
{
    uint32_t wr = g_dbg_log.wr_idx;
    uint32_t rd = g_dbg_log.rd_idx;
    return (wr - rd) & DBGLOG_BUF_MASK;
}

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/
