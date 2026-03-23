/*
 * hw_serial.c : UART hardware + CommInterface_t implementation
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Wraps uart_hw.c functions behind CommInterface_t and provides
 * Ring Buffer based receive for future DMA upgrade path.
 *
 * Memory: ~800 bytes Flash, ~600 bytes SRAM (ring buffers).
 */

#include "hw_serial.h"
#include "uart_hw.h"
#include "msg_protocol.h"
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

/* Ring buffer sizes (power of 2) */
#define PC_RX_BUF_SIZE      256U
#define TABLET_RX_BUF_SIZE  256U

/* Ring buffer storage */
static uint8_t pc_rx_storage[PC_RX_BUF_SIZE];
static RingBuffer_t pc_rx_ring;

#ifdef USE_TABLET_PC
static uint8_t tablet_rx_storage[TABLET_RX_BUF_SIZE];
static RingBuffer_t tablet_rx_ring;
#endif

/* ================================================================== */
/* CommInterface_t implementations for Serial_PC                      */
/* ================================================================== */

static bool serial_pc_init(void)
{
    /* Hardware already initialized by UART_HW_Config() */
    RingBuffer_Init(&pc_rx_ring, pc_rx_storage, PC_RX_BUF_SIZE);
    return true;
}

static uint16_t serial_pc_read(uint8_t* buffer, uint16_t length)
{
    return RingBuffer_GetBlock(&pc_rx_ring, buffer, length);
}

static bool serial_pc_write(const uint8_t* data, uint16_t length)
{
    uint16_t i;
    for (i = 0; i < length; i++) {
        UART_SendByte(UART_PC, data[i]);
    }
    return true;
}

static uint16_t serial_pc_available(void)
{
    return RingBuffer_Available(&pc_rx_ring);
}

static void serial_pc_flush(void)
{
    RingBuffer_Flush(&pc_rx_ring);
}

CommInterface_t Serial_PC = {
    .Init      = serial_pc_init,
    .Read      = serial_pc_read,
    .Write     = serial_pc_write,
    .Available = serial_pc_available,
    .Flush     = serial_pc_flush,
};

/* ================================================================== */
/* CommInterface_t implementations for Serial_Tablet                  */
/* ================================================================== */

#ifdef USE_TABLET_PC
static bool serial_tablet_init(void)
{
    RingBuffer_Init(&tablet_rx_ring, tablet_rx_storage, TABLET_RX_BUF_SIZE);
    return true;
}

static uint16_t serial_tablet_read(uint8_t* buffer, uint16_t length)
{
    return RingBuffer_GetBlock(&tablet_rx_ring, buffer, length);
}

static bool serial_tablet_write(const uint8_t* data, uint16_t length)
{
    uint16_t i;
    for (i = 0; i < length; i++) {
        UART_SendByte(UART_TABLET, data[i]);
    }
    return true;
}

static uint16_t serial_tablet_available(void)
{
    return RingBuffer_Available(&tablet_rx_ring);
}

static void serial_tablet_flush(void)
{
    RingBuffer_Flush(&tablet_rx_ring);
}

CommInterface_t Serial_Tablet = {
    .Init      = serial_tablet_init,
    .Read      = serial_tablet_read,
    .Write     = serial_tablet_write,
    .Available = serial_tablet_available,
    .Flush     = serial_tablet_flush,
};
#else
CommInterface_t Serial_Tablet = { 0 };
#endif /* USE_TABLET_PC */

/* ================================================================== */
/* Initialization                                                     */
/* ================================================================== */

void HwSerial_Init(void)
{
    UART_HW_Config();
    UART_QueueInitial();

    serial_pc_init();
#ifdef USE_TABLET_PC
    serial_tablet_init();
#endif
}

/* ================================================================== */
/* Legacy API wrappers                                                */
/* ================================================================== */

void HwSerial_SendByte(USART_TypeDef* uart, uint8_t byte)
{
    UART_SendByte(uart, byte);
}

void HwSerial_SendString(USART_TypeDef* uart, const char* str)
{
    UART_SendString(uart, str);
}

void HwSerial_Printf(USART_TypeDef* uart, char* format, ...)
{
    char buf[128];
    va_list arg;

    memset(buf, 0, sizeof(buf));
    va_start(arg, format);
    vsnprintf(buf, sizeof(buf), format, arg);
    va_end(arg);

    printUart(uart, "%s", buf);
}

void HwSerial_SendMessage(USART_TypeDef* uart, char* string)
{
    UART_SendMessage(uart, string);
}

void HwSerial_LogFromISR(const char* msg)
{
    DBG_LogFromISR(msg);
}

void HwSerial_FlushDebugRing(void)
{
    DBG_FlushRingBuffer();
}
