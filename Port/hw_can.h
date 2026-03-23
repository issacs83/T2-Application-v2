/*
 * hw_can.h : CAN Tx/Rx wrapper with CommInterface_t
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Wraps existing can.c functionality behind CommInterface_t.
 * Provides separate interfaces for Generator and Collimator targets.
 */

#ifndef HW_CAN_H
#define HW_CAN_H

#include "stm32f2xx.h"
#include "hw_comm.h"

/* Initialize CAN hardware and populate CommInterface_t instances */
void HwCAN_Init(void);

/**
 * Send a CAN message to a specific target with timeout and retry.
 * This is the legacy CAN_SendMessage wrapper.
 *
 * @param ext_id   CAN extended ID for the target
 * @param cmd      16-bit command code
 * @param value    32-bit value payload
 * @param debug    16-bit debug field
 * @param timeout  Timeout in ms
 * @param retry    Retry count
 * @return true on success
 */
bool HwCAN_SendToTarget(uint32_t ext_id, uint16_t cmd, uint32_t value,
                          uint16_t debug, uint32_t timeout, uint8_t retry);

/**
 * Process a received CAN message (called from main loop when flag set).
 * Delegates to existing CAN_ParseMessage().
 */
void HwCAN_ProcessReceived(void);

#endif /* HW_CAN_H */
