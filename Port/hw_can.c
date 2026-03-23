/*
 * hw_can.c : CAN hardware wrapper with CommInterface_t
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Memory: ~300 bytes Flash, ~32 bytes SRAM.
 */

#include "hw_can.h"
#include "can.h"
#include "shared_vars.h"
#include "isr_new.h"

/* CAN send functions defined in can.c but not declared in can.h */
extern bool CAN_SendMessage(uint16_t cmd, uint32_t value,
                              uint16_t debug, uint32_t timeOut, uint8_t retryCnt);
extern bool CAN_Collimator_SendMessage(uint16_t cmd, uint32_t value,
                                         uint16_t debug, uint32_t timeOut,
                                         uint8_t retryCnt);

/* ================================================================== */
/* CommInterface_t implementations for CAN_Generator                  */
/* ================================================================== */

static bool can_gen_init(void)
{
    /* CAN hardware initialized by CAN_Config() in System_Init() */
    return true;
}

static uint16_t can_gen_read(uint8_t* buffer, uint16_t length)
{
    (void)buffer;
    (void)length;
    /* CAN rx handled via ISR flag + CAN_ParseMessage in main loop */
    return 0;
}

static bool can_gen_write(const uint8_t* data, uint16_t length)
{
    (void)data;
    (void)length;
    /* Use HwCAN_SendToTarget() for structured CAN messages */
    return false;
}

static uint16_t can_gen_available(void)
{
    return g_can_rx_flag ? 1U : 0U;
}

static void can_gen_flush(void)
{
    g_can_rx_flag = 0;
}

CommInterface_t CAN_Generator = {
    .Init      = can_gen_init,
    .Read      = can_gen_read,
    .Write     = can_gen_write,
    .Available = can_gen_available,
    .Flush     = can_gen_flush,
};

/* ================================================================== */
/* CommInterface_t implementations for CAN_Collimator                 */
/* ================================================================== */

static bool can_col_init(void)
{
    return true;
}

static uint16_t can_col_read(uint8_t* buffer, uint16_t length)
{
    (void)buffer;
    (void)length;
    return 0;
}

static bool can_col_write(const uint8_t* data, uint16_t length)
{
    (void)data;
    (void)length;
    return false;
}

static uint16_t can_col_available(void)
{
    return 0;
}

static void can_col_flush(void)
{
    /* nothing */
}

CommInterface_t CAN_Collimator = {
    .Init      = can_col_init,
    .Read      = can_col_read,
    .Write     = can_col_write,
    .Available = can_col_available,
    .Flush     = can_col_flush,
};

/* ================================================================== */
/* Public functions                                                   */
/* ================================================================== */

void HwCAN_Init(void)
{
    CAN_Config();
}

bool HwCAN_SendToTarget(uint32_t ext_id, uint16_t cmd, uint32_t value,
                          uint16_t debug, uint32_t timeout, uint8_t retry)
{
    /* Delegate to the existing CAN send infrastructure in can.c based on
     * target ext_id. CAN_SendMessage handles frame building, TX mailbox
     * wait, and response matching. */
    if (ext_id == CAN_EXT_ID_SYSTEM_TO_TUBE) {
        return CAN_SendMessage(cmd, value, debug, timeout, retry);
    } else if (ext_id == CAN_EXT_ID_SYSTEM_TO_SUBB) {
        return CAN_Collimator_SendMessage(cmd, value, debug, timeout, retry);
    }
    return false;
}

void HwCAN_ProcessReceived(void)
{
    if (g_can_rx_flag) {
        __disable_irq();
        CAN_RxMessage = g_can_rx_msg;
        g_can_rx_flag = 0;
        __enable_irq();
        CAN_ParseMessage();
    }
}
