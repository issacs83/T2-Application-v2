/*
 * packet_parser.c : UART bracket message framing and validation
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Memory: ~300 bytes Flash, 0 bytes static SRAM (all in caller struct).
 */

#include "packet_parser.h"
#include <string.h>

/* ------------------------------------------------------------------ */
void PacketParser_Init(PacketParser_t* parser, PacketCallback_t callback,
                        void* ctx)
{
    memset(parser, 0, sizeof(PacketParser_t));
    parser->callback = callback;
    parser->context  = ctx;
}

/* ------------------------------------------------------------------ */
void PacketParser_Feed(PacketParser_t* parser, uint8_t byte)
{
    char c = (char)byte;

    switch (c) {
    case PKT_START_CHAR:
        memset(parser->buffer.data, 0, PKT_MSG_SIZE);
        parser->started = 1;
        parser->count   = 0;
        parser->buffer.data[parser->count++] = c;
        break;

    case PKT_END_CHAR:
        if (parser->started && (parser->count >= PKT_MIN_PAYLOAD_LEN)) {
            if (parser->count < PKT_MSG_SIZE) {
                parser->buffer.data[parser->count] = c;
            }

            /* Deliver complete message via callback */
            if (parser->callback != (PacketCallback_t)0) {
                parser->callback(&parser->buffer, parser->context);
            }
        }
        /* Reset state */
        parser->started = 0;
        parser->count   = 0;
        memset(parser->buffer.data, 0, PKT_MSG_SIZE);
        break;

    default:
        if (parser->started && (parser->count < (PKT_MSG_SIZE - 1U))) {
            /* Convert lowercase to uppercase */
            parser->buffer.data[parser->count++] = PacketParser_ToUpper(c);
        } else {
            /* Overflow or not started: reset */
            parser->count   = 0;
            parser->started = 0;
            memset(parser->buffer.data, 0, PKT_MSG_SIZE);
        }
        break;
    }
}

/* ------------------------------------------------------------------ */
void PacketParser_Reset(PacketParser_t* parser)
{
    parser->started = 0;
    parser->count   = 0;
    memset(parser->buffer.data, 0, PKT_MSG_SIZE);
}

/* ------------------------------------------------------------------ */
bool PacketParser_Match(const PacketMsg_t* msg, const char* cmd)
{
    uint8_t i;

    if (msg == (void*)0 || cmd == (void*)0) {
        return false;
    }

    for (i = 0; i < PKT_MSG_SIZE; i++) {
        if (cmd[i] == '\0') {
            return true;  /* all chars matched */
        }
        if (msg->data[i] != cmd[i]) {
            return false;
        }
    }

    return true;
}

/* ------------------------------------------------------------------ */
char PacketParser_CharAt(const PacketMsg_t* msg, uint8_t pos)
{
    if (msg == (void*)0 || pos >= PKT_MSG_SIZE) {
        return '\0';
    }
    return msg->data[pos];
}

/* ------------------------------------------------------------------ */
char PacketParser_ToUpper(char c)
{
    if (c >= 'a' && c <= 'z') {
        return c - 32;
    }
    return c;
}
