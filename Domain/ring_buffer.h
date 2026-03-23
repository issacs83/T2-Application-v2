/*
 * ring_buffer.h : Generic ring buffer (FIFO) for UART, CAN, debug log
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Hardware-independent. No STM32 headers included.
 *
 * Memory: sizeof(RingBuffer_t) = 8 bytes + user-provided buffer.
 */

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t*          buffer;
    uint16_t          size;       /* must be power of 2 */
    volatile uint16_t head;
    volatile uint16_t tail;
} RingBuffer_t;

/**
 * Initialize ring buffer. Size MUST be a power of 2.
 * @return true on success, false if size is not power of 2
 */
bool     RingBuffer_Init(RingBuffer_t* rb, uint8_t* buf, uint16_t size);
bool     RingBuffer_Put(RingBuffer_t* rb, uint8_t data);
bool     RingBuffer_Get(RingBuffer_t* rb, uint8_t* data);
uint16_t RingBuffer_Available(RingBuffer_t* rb);
bool     RingBuffer_IsEmpty(RingBuffer_t* rb);
bool     RingBuffer_IsFull(RingBuffer_t* rb);
void     RingBuffer_Flush(RingBuffer_t* rb);

/**
 * Write a block of bytes. Returns number of bytes actually written.
 */
uint16_t RingBuffer_PutBlock(RingBuffer_t* rb, const uint8_t* data, uint16_t len);

/**
 * Read a block of bytes. Returns number of bytes actually read.
 */
uint16_t RingBuffer_GetBlock(RingBuffer_t* rb, uint8_t* data, uint16_t max_len);

#endif /* RING_BUFFER_H */
