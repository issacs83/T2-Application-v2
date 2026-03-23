/*
 * packet_parser.h : UART bracket message framing and validation (no HW)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Parses the T2 16-byte bracket protocol: [XX_XXXX_XXXXX]
 * Hardware-independent: operates on byte streams via callback.
 *
 * Memory: ~100 bytes Flash, ~24 bytes SRAM per parser instance.
 */

#ifndef PACKET_PARSER_H
#define PACKET_PARSER_H

#include <stdint.h>
#include <stdbool.h>

/* Protocol constants */
#define PKT_MSG_SIZE            16U
#define PKT_MIN_PAYLOAD_LEN     13U     /* minimum chars between [ and ] */
#define PKT_START_CHAR          '['
#define PKT_END_CHAR            ']'

/**
 * Parsed message structure.
 */
typedef struct {
    char data[PKT_MSG_SIZE];
} PacketMsg_t;

/**
 * Callback type: called when a complete valid message is parsed.
 *
 * @param msg  Complete parsed message
 * @param ctx  User context pointer
 */
typedef void (*PacketCallback_t)(const PacketMsg_t* msg, void* ctx);

/**
 * Parser state machine instance.
 */
typedef struct {
    PacketMsg_t     buffer;
    uint8_t         count;
    uint8_t         started;
    PacketCallback_t callback;
    void*           context;
} PacketParser_t;

/**
 * Initialize a parser instance.
 *
 * @param parser    Parser instance to initialize
 * @param callback  Function called for each valid parsed message
 * @param ctx       User context passed to callback
 */
void PacketParser_Init(PacketParser_t* parser, PacketCallback_t callback,
                        void* ctx);

/**
 * Feed one byte to the parser. Processes the T2 bracket protocol.
 * When a complete message is detected, calls the registered callback.
 *
 * @param parser  Parser instance
 * @param byte    Received byte
 */
void PacketParser_Feed(PacketParser_t* parser, uint8_t byte);

/**
 * Reset parser state (discard partial message).
 */
void PacketParser_Reset(PacketParser_t* parser);

/**
 * Compare a message against a command string prefix.
 *
 * @param msg  Parsed message
 * @param cmd  Command string to match (e.g. "[SM_MODE_PANO")
 * @return true if msg starts with cmd
 */
bool PacketParser_Match(const PacketMsg_t* msg, const char* cmd);

/**
 * Extract a character at a specific position in the message.
 *
 * @param msg   Parsed message
 * @param pos   Position index (0-based)
 * @return Character at position, or '\0' if out of bounds
 */
char PacketParser_CharAt(const PacketMsg_t* msg, uint8_t pos);

/**
 * Convert lowercase ASCII to uppercase in-place.
 *
 * @param c  Character to convert
 * @return Uppercase character
 */
char PacketParser_ToUpper(char c);

#endif /* PACKET_PARSER_H */
