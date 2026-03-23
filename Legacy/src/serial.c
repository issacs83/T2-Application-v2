/*
*******************************************************************************
* serial.c : Backward-compatible serial initialization (thin wrapper)
*******************************************************************************
* Copyright (C) 2014-2018 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Brief   : This file is now a thin compatibility wrapper. The original
*             9000+ line serial.c has been split into:
*               - uart_hw.c     : UART hardware init, byte/string send
*               - msg_protocol.c: Message parsing, queue, real-time dispatch
*               - cmd_system.c  : Exposure, collimator, membrane, validation
*               - cmd_pano.c    : Panoramic mode command handlers
*               - cmd_ct.c      : CT/CBCT mode command handlers
*               - cmd_scan.c    : Cephalometric scan command handlers
*             Plus GeoAlign/Calibration/EEPROM/Diagnostic handlers remain
*             in their respective cmd_*.c files.
*
* @ Revision History :
*       1) initial creation. ------------------------------------- 2014-11-16
*       2) Refactored into modular files ------------------------- 2026-03-24
*******************************************************************************
*/

/* Include files ----------------------------------------------------------- */
#include "serial.h"

/* Public functions -------------------------------------------------------- */

/**
 * UART_Config: backward-compatible entry point.
 * Initializes UART hardware and message queues.
 */
void UART_Config(void)
{
    UART_HW_Config();
    UART_QueueInitial();
}

/************************ (C) COPYRIGHT Osstem Implant *****END OF FILE****/
