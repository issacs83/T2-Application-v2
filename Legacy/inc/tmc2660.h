/*
*******************************************************************************************
* tmc2660.h : TMC2660 stepper motor driver - SPI register interface
*******************************************************************************************
* Copyright (C) 2016-2026 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date    : 2026-03-24
* @ Brief   : Clean TMC2660 driver extracted from motor.c.
*             Table-driven design using g_motor_hw[] for CS pin lookup.
*             Hardware SPI3 (PC10=SCK, PC11=MISO, PC12=MOSI).
*
* @ Revision History :
*       1) initial creation (extracted from motor.c) ---------- 2026-03-24
*******************************************************************************************
*/

/* Define to prevent recursive inclusion ---------------------------------- */
#ifndef __TMC2660_H__
#define __TMC2660_H__

/* Includes -------------------------------------------------------------- */
#include "stm32f2xx.h"
#include <stdint.h>

/* Forward declaration of MotorId_t from motor_ctrl.h */
#include "motor_ctrl.h"

/* ========================================================================
 * TMC2660 Register Address Bits (bits [19:17] of 20-bit datagram)
 * ======================================================================== */
#define TMC2660_REG_DRVCTRL            0x00000U   /* [19:18] = 00 */
#define TMC2660_REG_CHOPCONF           0x80000U   /* [19:17] = 100 */
#define TMC2660_REG_SMARTEN            0xA0000U   /* [19:17] = 101 */
#define TMC2660_REG_SGCSCONF           0xC0000U   /* [19:17] = 110 */
#define TMC2660_REG_DRVCONF            0xE0000U   /* [19:17] = 111 */

/* ========================================================================
 * TMC2660 DRVCTRL Register (STEP/DIR mode) - bit field masks
 * ======================================================================== */
#define TMC2660_DRVCTRL_INTPOL         (1U << 9)  /* Step interpolation enable */
#define TMC2660_DRVCTRL_DEDGE          (1U << 8)  /* Double edge step pulses */
#define TMC2660_DRVCTRL_MRES_MASK      0x0FU      /* Microstep resolution [3:0] */

/* ========================================================================
 * TMC2660 CHOPCONF Register - bit field masks
 * ======================================================================== */
#define TMC2660_CHOP_TBL_SHIFT         15         /* Blanking time [16:15] */
#define TMC2660_CHOP_TBL_MASK          (0x3U << TMC2660_CHOP_TBL_SHIFT)
#define TMC2660_CHOP_CHM               (1U << 14) /* Chopper mode */
#define TMC2660_CHOP_RNDTF             (1U << 13) /* Random TOFF time */
#define TMC2660_CHOP_HDEC_SHIFT        11         /* Hysteresis decrement [12:11] */
#define TMC2660_CHOP_HDEC_MASK         (0x3U << TMC2660_CHOP_HDEC_SHIFT)
#define TMC2660_CHOP_HEND_SHIFT        7          /* Hysteresis end value [10:7] */
#define TMC2660_CHOP_HEND_MASK         (0xFU << TMC2660_CHOP_HEND_SHIFT)
#define TMC2660_CHOP_HSTRT_SHIFT       4          /* Hysteresis start [6:4] */
#define TMC2660_CHOP_HSTRT_MASK        (0x7U << TMC2660_CHOP_HSTRT_SHIFT)
#define TMC2660_CHOP_TOFF_SHIFT        0          /* Off time [3:0] */
#define TMC2660_CHOP_TOFF_MASK         (0xFU << TMC2660_CHOP_TOFF_SHIFT)

/* ========================================================================
 * TMC2660 SMARTEN (coolStep) Register - bit field masks
 * ======================================================================== */
#define TMC2660_SMART_SEIMIN           (1U << 15) /* Min coolStep current */
#define TMC2660_SMART_SEDN_SHIFT       13         /* Current decrement speed [14:13] */
#define TMC2660_SMART_SEDN_MASK        (0x3U << TMC2660_SMART_SEDN_SHIFT)
#define TMC2660_SMART_SEMAX_SHIFT      8          /* Upper threshold [11:8] */
#define TMC2660_SMART_SEMAX_MASK       (0xFU << TMC2660_SMART_SEMAX_SHIFT)
#define TMC2660_SMART_SEUP_SHIFT       5          /* Current increment [6:5] */
#define TMC2660_SMART_SEUP_MASK        (0x3U << TMC2660_SMART_SEUP_SHIFT)
#define TMC2660_SMART_SEMIN_SHIFT      0          /* Min threshold [3:0] */
#define TMC2660_SMART_SEMIN_MASK       (0xFU << TMC2660_SMART_SEMIN_SHIFT)

/* ========================================================================
 * TMC2660 SGCSCONF (stallGuard2) Register - bit field masks
 * ======================================================================== */
#define TMC2660_SGCS_SFILT             (1U << 16) /* SG2 filter enable */
#define TMC2660_SGCS_SGT_SHIFT         8          /* SG2 threshold [14:8] (signed) */
#define TMC2660_SGCS_SGT_MASK          (0x7FU << TMC2660_SGCS_SGT_SHIFT)
#define TMC2660_SGCS_CS_SHIFT          0          /* Current scale [4:0] */
#define TMC2660_SGCS_CS_MASK           (0x1FU << TMC2660_SGCS_CS_SHIFT)

/* ========================================================================
 * TMC2660 DRVCONF Register - bit field masks
 * ======================================================================== */
#define TMC2660_DRV_TST                (1U << 16) /* Test mode */
#define TMC2660_DRV_SLPH_SHIFT         14         /* Slope ctrl high [15:14] */
#define TMC2660_DRV_SLPH_MASK          (0x3U << TMC2660_DRV_SLPH_SHIFT)
#define TMC2660_DRV_SLPL_SHIFT         12         /* Slope ctrl low [13:12] */
#define TMC2660_DRV_SLPL_MASK          (0x3U << TMC2660_DRV_SLPL_SHIFT)
#define TMC2660_DRV_DISS2G             (1U << 10) /* Short to GND protection disable */
#define TMC2660_DRV_TS2G_SHIFT         8          /* Short detection timer [9:8] */
#define TMC2660_DRV_TS2G_MASK          (0x3U << TMC2660_DRV_TS2G_SHIFT)
#define TMC2660_DRV_SDOFF              (1U << 7)  /* STEP/DIR interface disable */
#define TMC2660_DRV_VSENSE             (1U << 6)  /* Sense resistor voltage select */
#define TMC2660_DRV_RDSEL_SHIFT        4          /* Read select [5:4] */
#define TMC2660_DRV_RDSEL_MASK         (0x3U << TMC2660_DRV_RDSEL_SHIFT)

/* ========================================================================
 * TMC2660 Status Response - bit field masks (20-bit SPI response >> 4)
 * ======================================================================== */
#define TMC2660_STATUS_STST            (1U << 7)  /* Standstill indicator */
#define TMC2660_STATUS_OLB             (1U << 6)  /* Open load phase B */
#define TMC2660_STATUS_OLA             (1U << 5)  /* Open load phase A */
#define TMC2660_STATUS_S2GB            (1U << 4)  /* Short to GND phase B */
#define TMC2660_STATUS_S2GA            (1U << 3)  /* Short to GND phase A */
#define TMC2660_STATUS_OTPW            (1U << 2)  /* Overtemperature warning (100C) */
#define TMC2660_STATUS_OT              (1U << 1)  /* Overtemperature shutdown (150C) */
#define TMC2660_STATUS_SG              (1U << 0)  /* StallGuard2 active */

/* Error mask: any fault condition */
#define TMC2660_STATUS_ERROR_MASK      (TMC2660_STATUS_OLB  | TMC2660_STATUS_OLA  | \
                                        TMC2660_STATUS_S2GB | TMC2660_STATUS_S2GA | \
                                        TMC2660_STATUS_OT)

/* ========================================================================
 * Default Init Register Values (from existing TMC2660_SPI_Init)
 *
 * CHOPCONF  0x81548: TBL=16, SpreadCycle, HDEC=48clk, HEND=7, HSTRT=5, TOFF=10
 * SMARTEN   0xAA120: SEIMIN=1/4CS, SEDN=8, SEMAX=1, SEUP=2, SEMIN=0(disabled)
 * DRVCONF   0xEF010: SLPH=max, SLPL=max, S2G=on, TS2G=3.2us, STEP/DIR, 305mV, SG2 readback
 * DRVCTRL   0x00204: INTPOL=on, 16x microstep
 * ======================================================================== */
#define TMC2660_INIT_CHOPCONF          0x81548U
#define TMC2660_INIT_SMARTEN           0xAA120U
#define TMC2660_INIT_DRVCONF           0xEF010U
#define TMC2660_INIT_DRVCTRL_X16       0x00204U

/* ========================================================================
 * SGCSCONF base value for current control
 * SG filter=1, threshold=0x09, CS field at [4:0]
 * ======================================================================== */
#define TMC2660_SGCSCONF_BASE          0xD0900U

/* ========================================================================
 * Microstep Resolution Enum
 * Index matches DRVCTRL MRES field value.
 * ======================================================================== */
typedef enum {
    TMC2660_MRES_256 = 0,   /* 256 microsteps per full step */
    TMC2660_MRES_128 = 1,
    TMC2660_MRES_64  = 2,
    TMC2660_MRES_32  = 3,
    TMC2660_MRES_16  = 4,   /* Default */
    TMC2660_MRES_8   = 5,
    TMC2660_MRES_4   = 6,
    TMC2660_MRES_COUNT       /* 7 total */
} TMC2660_Microstep_t;

/* ========================================================================
 * Current Setting Enum
 * Values are pre-computed SGCSCONF register values for common current levels.
 * Based on 305mV sense voltage and specific sense resistor on T2 board.
 * ======================================================================== */
typedef enum {
    TMC2660_CURR_0_0A  = 0,   /* 0.0A (disabled) - CS=0x00 */
    TMC2660_CURR_0_1A  = 1,   /* ~0.125A         - CS=0x01 */
    TMC2660_CURR_0_2A  = 2,   /* ~0.25A          - CS=0x02 */
    TMC2660_CURR_0_5A  = 3,   /* ~0.5A           - CS=0x03 */
    TMC2660_CURR_0_7A  = 4,   /* ~0.625A         - CS=0x04 */
    TMC2660_CURR_1_0A  = 5,   /* ~1.0A           - CS=0x07 */
    TMC2660_CURR_1_3A  = 6,   /* ~1.25A          - CS=0x09 */
    TMC2660_CURR_1_4A  = 7,   /* ~1.375A         - CS=0x0A */
    TMC2660_CURR_1_8A  = 8,   /* ~1.75A          - CS=0x0D */
    TMC2660_CURR_2_0A  = 9,   /* ~2.0A           - CS=0x0F */
    TMC2660_CURR_2_4A  = 10,  /* ~2.25A          - CS=0x12 */
    TMC2660_CURR_2_9A  = 11,  /* ~2.75A          - CS=0x16 */
    TMC2660_CURR_COUNT          /* 12 total */
} TMC2660_Current_t;

/* ========================================================================
 * Chopper Configuration (for TMC2660_SetChopperConfig)
 * ======================================================================== */
typedef struct {
    uint8_t tbl;        /* Blanking time: 0-3 (16/24/36/54 clocks) */
    uint8_t chm;        /* Chopper mode: 0=SpreadCycle, 1=constant TOFF */
    uint8_t rndtf;      /* Random TOFF: 0=fixed, 1=random */
    uint8_t hdec;       /* Hysteresis decrement: 0-3 (16/32/48/64 clocks) */
    uint8_t hend;       /* Hysteresis end value: 0-15 (offset -3..12) */
    uint8_t hstrt;      /* Hysteresis start: 0-7 (offset 1..8) */
    uint8_t toff;       /* Off time: 0-15 (0=driver disabled) */
} TMC2660_ChopperCfg_t;

/* ========================================================================
 * Exported Functions
 * ======================================================================== */

/**
 * Initialize TMC2660 registers with default values for one motor.
 * Sends CHOPCONF, SMARTEN, DRVCONF, DRVCTRL in sequence.
 *
 * @param id  Motor index (0..MCTRL_COUNT-1), uses g_motor_hw[id] for CS pin
 */
void TMC2660_Init(MotorId_t id);

/**
 * Initialize all TMC2660 drivers (all motors).
 * Calls TMC2660_Init() for each motor in g_motor_hw[].
 */
void TMC2660_InitAll(void);

/**
 * Initialize SPI3 peripheral and all CS GPIO pins.
 * Must be called once at startup before any TMC2660 functions.
 */
void TMC2660_SPI_Init(void);

/**
 * Write a 20-bit register value to a motor's TMC2660.
 *
 * @param id    Motor index (0..MCTRL_COUNT-1)
 * @param data  20-bit register value (address bits [19:17] + payload)
 * @return      20-bit status response (right-shifted by 4)
 */
uint32_t TMC2660_WriteRegister(MotorId_t id, uint32_t data);

/**
 * Read TMC2660 status by sending a NOP (DRVCTRL=0x00000).
 *
 * @param id  Motor index (0..MCTRL_COUNT-1)
 * @return    20-bit status response, or 0xFFFFF on invalid id
 */
uint32_t TMC2660_ReadStatus(MotorId_t id);

/**
 * Set microstep resolution via DRVCTRL register.
 * INTPOL (interpolation to 256) is always enabled.
 *
 * @param id          Motor index
 * @param resolution  TMC2660_MRES_256..TMC2660_MRES_4
 */
void TMC2660_SetMicrostep(MotorId_t id, TMC2660_Microstep_t resolution);

/**
 * Set motor current via SGCSCONF register.
 *
 * @param id       Motor index
 * @param run_ma   Run current (TMC2660_CURR_0_0A..TMC2660_CURR_2_9A)
 * @param hold_ma  Hold current (TMC2660_CURR_0_0A..TMC2660_CURR_2_9A)
 *
 * Note: run current is sent immediately. Hold current is stored for
 * application when motor stops (caller responsibility).
 */
void TMC2660_SetCurrent(MotorId_t id,
                         TMC2660_Current_t run_ma,
                         TMC2660_Current_t hold_ma);

/**
 * Set run current only (immediate SPI write).
 *
 * @param id       Motor index
 * @param current  TMC2660_CURR_0_0A..TMC2660_CURR_2_9A
 */
void TMC2660_SetRunCurrent(MotorId_t id, TMC2660_Current_t current);

/**
 * Set chopper configuration via CHOPCONF register.
 *
 * @param id      Motor index
 * @param config  Pointer to chopper configuration struct
 */
void TMC2660_SetChopperConfig(MotorId_t id, const TMC2660_ChopperCfg_t* config);

/**
 * Enable motor driver output (set TOFF > 0 in CHOPCONF).
 * Uses the last written CHOPCONF value with TOFF restored.
 *
 * @param id  Motor index
 */
void TMC2660_Enable(MotorId_t id);

/**
 * Disable motor driver output (set TOFF = 0 in CHOPCONF).
 * Motor coils become unpowered.
 *
 * @param id  Motor index
 */
void TMC2660_Disable(MotorId_t id);

/**
 * Check if any error flags are set in the last status response.
 *
 * @param status  Status value from TMC2660_WriteRegister or TMC2660_ReadStatus
 * @return        Non-zero if any error flag is set
 */
static inline uint32_t TMC2660_HasError(uint32_t status)
{
    return status & TMC2660_STATUS_ERROR_MASK;
}

#endif /* __TMC2660_H__ */
