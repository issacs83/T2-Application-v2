/*
 * hw_comm.h : Communication interface abstraction (function pointer based)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Provides CommInterface_t for UART and CAN communication channels.
 * App and Domain layers use these interfaces without knowing the
 * underlying hardware.
 */

#ifndef HW_COMM_H
#define HW_COMM_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Generic communication interface (function pointer vtable).
 */
typedef struct {
    bool     (*Init)(void);
    uint16_t (*Read)(uint8_t* buffer, uint16_t length);
    bool     (*Write)(const uint8_t* data, uint16_t length);
    uint16_t (*Available)(void);
    void     (*Flush)(void);
} CommInterface_t;

/* UART communication channels */
extern CommInterface_t Serial_PC;       /* USART1 (PC, 115200) */
extern CommInterface_t Serial_Tablet;   /* USART2 (Tablet, 19200) */

/* CAN communication channels */
extern CommInterface_t CAN_Generator;   /* CAN2 Generator */
extern CommInterface_t CAN_Collimator;  /* CAN2 Collimator */

#endif /* HW_COMM_H */
