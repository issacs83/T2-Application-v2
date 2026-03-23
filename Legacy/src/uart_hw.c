/*
*******************************************************************************
* uart_hw.c : UART hardware abstraction layer
*******************************************************************************
* Copyright (C) 2014-2018 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Brief   : UART peripheral init, byte/string send, debug print
*
* @ Revision History :
*       1) Extracted from serial.c for modular build --------- 2026-03-24
*******************************************************************************
*/

/* Include files ----------------------------------------------------------- */
#include <stdarg.h>
#include <string.h>
#include "uart_hw.h"
#include "configuration.h"

/* Private defines --------------------------------------------------------- */
#ifdef USE_TABLET_PC
#define UART_TABLET_CLOCK               RCC_APB1Periph_USART2
#define UART_TABLET_GPIO                GPIOD
#define UART_TABLET_GPIO_CLOCK          RCC_AHB1Periph_GPIOD
#define UART_TABLET_GPIO_PIN            GPIO_Pin_5 | GPIO_Pin_6
#define UART_TABLET_GPIO_PinSource_Tx   GPIO_PinSource5
#define UART_TABLET_GPIO_PinSource_Rx   GPIO_PinSource6
#define UART_TABLET_GPIO_AF             GPIO_AF_USART2
#define UART_TABLET_INTRRUPT            USART2_IRQn
#define UART_TABLET_BAUDRATE            (uint32_t)19200
#endif /* USE_TABLET_PC */

#define UART_PC_CLOCK                   RCC_APB2Periph_USART1
#define UART_PC_GPIO                    GPIOA
#define UART_PC_GPIO_CLOCK              RCC_AHB1Periph_GPIOA
#define UART_PC_GPIO_PIN                GPIO_Pin_9 | GPIO_Pin_10
#define UART_PC_GPIO_PinSource_Tx       GPIO_PinSource9
#define UART_PC_GPIO_PinSource_Rx       GPIO_PinSource10
#define UART_PC_GPIO_AF                 GPIO_AF_USART1
#define UART_PC_INTRRUPT                USART1_IRQn
#define UART_PC_BAUDRATE                (uint32_t)115200

#ifdef USE_TABLET_PC
#define UART_GPIO_CLOCK     UART_TABLET_GPIO_CLOCK | UART_PC_GPIO_CLOCK
#else
#define UART_GPIO_CLOCK     UART_PC_GPIO_CLOCK
#endif /* USE_TABLET_PC */

#define PRINT_BUF_SIZE      128

/* Private types ----------------------------------------------------------- */
typedef struct {
    char     buf[DBG_RING_BUF_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
} dbg_ring_t;

/* Private variables ------------------------------------------------------- */
static dbg_ring_t dbg_ring = { .head = 0, .tail = 0 };

/* Private function prototypes --------------------------------------------- */
static void uart_periph_init(USART_TypeDef *uart, uint32_t baudrate,
                             uint8_t interrupt);

/* Public functions -------------------------------------------------------- */

void UART_HW_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

#ifdef USE_TABLET_PC
    /* Enable USART clock */
    if ((UART_TABLET_CLOCK == RCC_APB2Periph_USART1) ||
        (UART_TABLET_CLOCK == RCC_APB2Periph_USART6))
        RCC_APB2PeriphClockCmd(UART_TABLET_CLOCK, ENABLE);
    else
        RCC_APB1PeriphClockCmd(UART_TABLET_CLOCK, ENABLE);
#endif /* USE_TABLET_PC */

    if ((UART_PC_CLOCK == RCC_APB2Periph_USART1) ||
        (UART_PC_CLOCK == RCC_APB2Periph_USART6))
        RCC_APB2PeriphClockCmd(UART_PC_CLOCK, ENABLE);
    else
        RCC_APB1PeriphClockCmd(UART_PC_CLOCK, ENABLE);

    /* Enable GPIO clock */
    RCC_AHB1PeriphClockCmd(UART_GPIO_CLOCK, ENABLE);

#ifdef USE_TABLET_PC
    /* Configure USART RX and TX pins */
    GPIO_InitStructure.GPIO_Pin   = UART_TABLET_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(UART_TABLET_GPIO, &GPIO_InitStructure);
#endif /* USE_TABLET_PC */

    GPIO_InitStructure.GPIO_Pin   = UART_PC_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(UART_PC_GPIO, &GPIO_InitStructure);

#ifdef USE_TABLET_PC
    GPIO_PinAFConfig(UART_TABLET_GPIO, UART_TABLET_GPIO_PinSource_Tx,
                     UART_TABLET_GPIO_AF);
    GPIO_PinAFConfig(UART_TABLET_GPIO, UART_TABLET_GPIO_PinSource_Rx,
                     UART_TABLET_GPIO_AF);
#endif /* USE_TABLET_PC */

    GPIO_PinAFConfig(UART_PC_GPIO, UART_PC_GPIO_PinSource_Tx, UART_PC_GPIO_AF);
    GPIO_PinAFConfig(UART_PC_GPIO, UART_PC_GPIO_PinSource_Rx, UART_PC_GPIO_AF);

#ifdef USE_TABLET_PC
    uart_periph_init(UART_TABLET, UART_TABLET_BAUDRATE, UART_TABLET_INTRRUPT);
#endif /* USE_TABLET_PC */
    uart_periph_init(UART_PC, UART_PC_BAUDRATE, UART_PC_INTRRUPT);
}

void UART_SendByte(USART_TypeDef *uart, uint8_t byte)
{
    USART_SendData(uart, byte);
    while (USART_GetFlagStatus(uart, USART_FLAG_TC) == RESET)
        ;
}

void UART_SendString(USART_TypeDef *uart, const char *str)
{
    while (*str) {
        UART_SendByte(uart, (uint8_t)(*str++));
    }
    UART_SendByte(uart, '\r');
    UART_SendByte(uart, '\n');
}

void printUart(USART_TypeDef *uart, char *format, ...)
{
    char buf[PRINT_BUF_SIZE];
    int idx = 0;
    va_list arg;

    memset(buf, 0, sizeof(buf));
    va_start(arg, format);
    vsprintf(buf, format, arg);
    va_end(arg);

    while (buf[idx] != '\0') {
        USART_SendData(uart, buf[idx++]);
        while (USART_GetFlagStatus(uart, USART_FLAG_TC) == RESET)
            ;
    }

    USART_SendData(uart, '\r');
    while (USART_GetFlagStatus(uart, USART_FLAG_TC) == RESET)
        ;
    USART_SendData(uart, '\n');
    while (USART_GetFlagStatus(uart, USART_FLAG_TC) == RESET)
        ;
}

void DBG_LogFromISR(const char *msg)
{
    uint32_t next;

    while (*msg) {
        next = (dbg_ring.head + 1) % DBG_RING_BUF_SIZE;
        if (next == dbg_ring.tail) {
            /* Ring buffer full -- drop character */
            return;
        }
        dbg_ring.buf[dbg_ring.head] = *msg++;
        dbg_ring.head = next;
    }
    /* Enqueue newline separator */
    next = (dbg_ring.head + 1) % DBG_RING_BUF_SIZE;
    if (next != dbg_ring.tail) {
        dbg_ring.buf[dbg_ring.head] = '\n';
        dbg_ring.head = next;
    }
}

void DBG_FlushRingBuffer(void)
{
    while (dbg_ring.tail != dbg_ring.head) {
        char c = dbg_ring.buf[dbg_ring.tail];
        dbg_ring.tail = (dbg_ring.tail + 1) % DBG_RING_BUF_SIZE;

        if (c == '\n') {
            UART_SendByte(UART_PC, '\r');
            UART_SendByte(UART_PC, '\n');
        } else {
            UART_SendByte(UART_PC, (uint8_t)c);
        }
    }
}

/* Private functions ------------------------------------------------------- */

static void uart_periph_init(USART_TypeDef *uart, uint32_t baudrate,
                             uint8_t interrupt)
{
    NVIC_InitTypeDef  NVIC_InitStructure;
    USART_InitTypeDef UART_InitStructure;

    UART_InitStructure.USART_BaudRate            = baudrate;
    UART_InitStructure.USART_WordLength           = USART_WordLength_8b;
    UART_InitStructure.USART_StopBits             = USART_StopBits_1;
    UART_InitStructure.USART_Parity               = USART_Parity_No;
    UART_InitStructure.USART_HardwareFlowControl  = USART_HardwareFlowControl_None;
    UART_InitStructure.USART_Mode                  = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(uart, &UART_InitStructure);

    /* USART RX interrupt enable */
    USART_ITConfig(uart, USART_IT_RXNE, ENABLE);

    /* Enable the USART Interrupt */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannel                   = interrupt;
    NVIC_Init(&NVIC_InitStructure);

    /* USART enable */
    USART_Cmd(uart, ENABLE);
}

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/
