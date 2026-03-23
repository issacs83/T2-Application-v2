/*
 * ring_buffer.c : Generic ring buffer implementation
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Lock-free single-producer/single-consumer design.
 * For multi-producer use, caller must provide external synchronization.
 *
 * Memory: ~200 bytes Flash, 0 bytes SRAM (state in caller-provided struct).
 */

#include "ring_buffer.h"

bool RingBuffer_Init(RingBuffer_t* rb, uint8_t* buf, uint16_t size)
{
    /* Validate power of 2 — required for bitmask index wrapping */
    if (size == 0 || (size & (size - 1U)) != 0) {
        return false;
    }

    rb->buffer = buf;
    rb->size   = size;
    rb->head   = 0;
    rb->tail   = 0;
    return true;
}

bool RingBuffer_Put(RingBuffer_t* rb, uint8_t data)
{
    uint16_t next = (rb->head + 1U) & (rb->size - 1U);

    if (next == rb->tail) {
        return false;  /* full */
    }

    rb->buffer[rb->head] = data;
    rb->head = next;
    return true;
}

bool RingBuffer_Get(RingBuffer_t* rb, uint8_t* data)
{
    if (rb->head == rb->tail) {
        return false;  /* empty */
    }

    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1U) & (rb->size - 1U);
    return true;
}

uint16_t RingBuffer_Available(RingBuffer_t* rb)
{
    return (rb->head - rb->tail) & (rb->size - 1U);
}

bool RingBuffer_IsEmpty(RingBuffer_t* rb)
{
    return (rb->head == rb->tail);
}

bool RingBuffer_IsFull(RingBuffer_t* rb)
{
    uint16_t next = (rb->head + 1U) & (rb->size - 1U);
    return (next == rb->tail);
}

void RingBuffer_Flush(RingBuffer_t* rb)
{
    rb->tail = rb->head;
}

uint16_t RingBuffer_PutBlock(RingBuffer_t* rb, const uint8_t* data, uint16_t len)
{
    uint16_t i;

    for (i = 0; i < len; i++) {
        if (!RingBuffer_Put(rb, data[i])) {
            break;
        }
    }
    return i;
}

uint16_t RingBuffer_GetBlock(RingBuffer_t* rb, uint8_t* data, uint16_t max_len)
{
    uint16_t i;

    for (i = 0; i < max_len; i++) {
        if (!RingBuffer_Get(rb, &data[i])) {
            break;
        }
    }
    return i;
}
