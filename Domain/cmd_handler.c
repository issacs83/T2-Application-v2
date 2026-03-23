/*
 * cmd_handler.c : Command dispatch table implementation
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Memory: ~100 bytes Flash, 0 bytes static SRAM.
 */

#include "cmd_handler.h"

/* ------------------------------------------------------------------ */
void CmdDispatch_Init(CmdDispatch_t* disp, const CmdEntry_t* table,
                       uint16_t table_size, void* user_ctx)
{
    disp->table      = table;
    disp->table_size = table_size;
    disp->user_ctx   = user_ctx;
}

/* ------------------------------------------------------------------ */
bool CmdDispatch_Process(CmdDispatch_t* disp, const PacketMsg_t* msg)
{
    uint16_t i;

    if (disp == (void*)0 || msg == (void*)0 || disp->table == (void*)0) {
        return false;
    }

    for (i = 0; i < disp->table_size; i++) {
        if (PacketParser_Match(msg, disp->table[i].prefix)) {
            if (disp->table[i].handler != (CmdHandlerFunc_t)0) {
                if (disp->table[i].handler(msg, disp->user_ctx)) {
                    return true;
                }
            }
        }
    }

    return false;
}
