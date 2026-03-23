/*
 * cmd_handler.h : Command dispatch table and handler routing (no HW)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Table-driven command dispatch: each entry maps a command prefix string
 * to a handler function. The dispatch function scans the table linearly
 * and calls the first matching handler.
 *
 * Memory: ~100 bytes Flash + N*12 bytes per command entry.
 */

#ifndef CMD_HANDLER_H
#define CMD_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include "packet_parser.h"

/**
 * Command handler function type.
 *
 * @param msg  Parsed message that triggered this handler
 * @param ctx  User context pointer
 * @return true if message was fully handled, false to continue dispatch
 */
typedef bool (*CmdHandlerFunc_t)(const PacketMsg_t* msg, void* ctx);

/**
 * Command table entry.
 */
typedef struct {
    const char*      prefix;    /* command prefix to match */
    CmdHandlerFunc_t handler;   /* handler function */
} CmdEntry_t;

/**
 * Command dispatch context.
 */
typedef struct {
    const CmdEntry_t*  table;       /* command table array */
    uint16_t           table_size;  /* number of entries */
    void*              user_ctx;    /* passed to handlers */
} CmdDispatch_t;

/**
 * Initialize command dispatcher with a command table.
 *
 * @param disp       Dispatcher instance
 * @param table      Array of CmdEntry_t
 * @param table_size Number of entries
 * @param user_ctx   Context pointer passed to all handlers
 */
void CmdDispatch_Init(CmdDispatch_t* disp, const CmdEntry_t* table,
                       uint16_t table_size, void* user_ctx);

/**
 * Dispatch a parsed message through the command table.
 * Scans table linearly, calls first matching handler.
 *
 * @param disp  Dispatcher instance
 * @param msg   Parsed message to dispatch
 * @return true if a handler was found and returned true
 */
bool CmdDispatch_Process(CmdDispatch_t* disp, const PacketMsg_t* msg);

#endif /* CMD_HANDLER_H */
