/*
*******************************************************************************
* serial.h : Backward-compatible serial interface (facade)
*******************************************************************************
* Copyright (C) 2016-2018 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date    :
* @ Brief   : This header now acts as a facade that re-exports all
*             public APIs from the modular uart_hw / msg_protocol /
*             cmd_dispatch subsystem. Existing callers that #include
*             "serial.h" will continue to work without changes.
*
* @ Revision History :
*       1) initial creation. -------------------------------------- 2018-11-12
*       2) Refactored into modular headers ----------------------- 2026-03-24
*******************************************************************************
*/

/* Define to prevent recursive inclusion ------------------------------------ */
#ifndef __SERIAL_H__
#define __SERIAL_H__

/* Pull in all modular headers --------------------------------------------- */
#include "uart_hw.h"
#include "msg_protocol.h"
#include "cmd_dispatch.h"

/*
 * Backward compatibility: UART_Config() maps to UART_HW_Config() +
 * UART_QueueInitial(). Defined in serial.c (thin wrapper).
 */
void UART_Config(void);

#endif /* __SERIAL_H__ */
