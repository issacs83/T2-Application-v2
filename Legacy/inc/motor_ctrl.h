/*
*******************************************************************************************
* motor_ctrl.h : Optimized unified motor control module
*******************************************************************************************
* Copyright (C) 2016-2026 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date    : 2026-03-23
* @ Brief   : Unified motor control with LUT-based profiles, integer-only ISR,
*             and table-driven TMC2660 configuration.
*
*   Design principles:
*     - Single MotorCtrl_t struct for all 8 motors (indexed by MotorId_t)
*     - Hardware config (pins, timers) separated from motion state (const flash table)
*     - LUT-based pre-computed profiles in ring buffer (4096 entries max)
*     - Supports trapezoidal AND arch (pre-computed CCR array) profiles
*     - All ISR-shared data marked volatile
*     - Fixed-point Q16.16 arithmetic for profile building (NO floating point)
*     - ISR handler: ~30 cycles worst-case (read buffer, write ARR, toggle GPIO)
*
*   Memory budget (STM32F207, 128KB SRAM, 1MB Flash):
*     - Profile buffers: 8 motors * 4096 * 2 bytes = 64 KB SRAM
*     - MotorCtrl_t array: 8 * ~8232 bytes = ~65.9 KB total
*     - HW config table: 8 * 28 bytes = 224 bytes Flash
*     - Total SRAM: ~66 KB (51% of 128KB)
*
* @ Revision History :
*       1) initial creation. ------------------------------------ 2026-03-23
*******************************************************************************************
*/

/* Define to prevent recursive inclusion ---------------------------------- */
#ifndef __MOTOR_CTRL_H__
#define __MOTOR_CTRL_H__

/* Includes -------------------------------------------------------------- */
#include "stm32f2xx.h"
#include <stdint.h>

/* Exported define ------------------------------------------------------- */

/* Timer base clock (must match TIM_COUNT_CLOCK in timer.h) */
#define MCTRL_TIM_CLK               1250000U
#define MCTRL_TIM_TOGGLE            2U

/* Fixed-point Q16.16 helpers (integer-only, for profile builder) */
#define FP_SHIFT                    16
#define FP_ONE                      (1UL << FP_SHIFT)
#define FP_FROM_INT(x)              ((uint32_t)(x) << FP_SHIFT)
#define FP_TO_INT(x)                ((uint32_t)(x) >> FP_SHIFT)
#define FP_MUL(a, b)                ((uint32_t)(((uint64_t)(a) * (uint64_t)(b)) >> FP_SHIFT))
#define FP_DIV(a, b)                ((uint32_t)(((uint64_t)(a) << FP_SHIFT) / (uint64_t)(b)))

/* Default start frequency for trapezoidal ramp (Hz) */
#define MCTRL_TRAP_START_FREQ_HZ    500U

/* Profile ring buffer */
#define PROFILE_BUF_SIZE            4096U
#define PROFILE_BUF_MASK            (PROFILE_BUF_SIZE - 1U)

/* CCR value limits */
#define MCTRL_CCR_MAX               0xFFFFU
#define MCTRL_CCR_MIN               10U

/* Direct GPIO for SPI CS (bypasses StdPeriph for speed) */
#define MCTRL_CS_LOW(port, pin)     ((port)->BSRRH = (pin))
#define MCTRL_CS_HIGH(port, pin)    ((port)->BSRRL = (pin))

/* TMC2660 register address bits (bits 19:17) */
#define TMC_CHOPCONF                0x80000U
#define TMC_SMARTEN                 0xA0000U
#define TMC_SGCSCONF                0xC0000U
#define TMC_DRVCONF                 0xE0000U
#define TMC_DRVCTRL                 0x00000U

/* TMC2660 default register values (from existing TMC2660_SPI_Init) */
#define TMC_INIT_CHOPCONF           0x81548U
#define TMC_INIT_SMARTEN            0xAA120U
#define TMC_INIT_DRVCONF            0xEF010U
#define TMC_INIT_DRVCTRL_X16        0x00204U

/* TMC2660 resolution DRVCTRL register values (with INTPOL=1) */
/* Index matches MotorResolution enum from motor.h */
#define TMC_RES_X256                0x00200U
#define TMC_RES_X128                0x00201U
#define TMC_RES_X64                 0x00202U
#define TMC_RES_X32                 0x00203U
#define TMC_RES_X16                 0x00204U
#define TMC_RES_X8                  0x00205U
#define TMC_RES_X4                  0x00206U

/* TMC2660 current SGCSCONF base (SG filter=1, threshold=0) + CS field */
#define TMC_SGCSCONF_BASE           0xD0900U

/* Exported types -------------------------------------------------------- */

/**
 * Motor identification.
 * Order matches g_motor_hw[] config table index.
 */
typedef enum {
    MCTRL_R = 0,    /* Rotation       TIM8  CC3   PC8  */
    MCTRL_V,        /* Vertical       TIM1  CC1   PA8  */
    MCTRL_C,        /* Collimator     TIM2  CC2   PA1  */
    MCTRL_T,        /* Chin rest TS   TIM5  UPD   PC6  */
    MCTRL_S,        /* Slit           TIM10 CC1   PB8  */
    MCTRL_A,        /* Gantry         TIM9  CC1   PE5  */
    MCTRL_CNS,      /* Chin rest NS   TIM3  CC3   PB0  */
    MCTRL_CWE,      /* Chin rest WE   TIM11 CC1   PB9  */
    MCTRL_COUNT     /* 8 total */
} MotorId_t;

/**
 * Motor state machine.
 */
typedef enum {
    MSTATE_IDLE = 0,
    MSTATE_ACCEL,
    MSTATE_CRUISE,
    MSTATE_DECEL,
    MSTATE_DONE,
    MSTATE_ERROR
} MotorState_t;

/**
 * Profile type discriminator.
 */
typedef enum {
    PROF_NONE = 0,
    PROF_TRAPEZOID,
    PROF_ARCH
} ProfileType_t;

/**
 * Hardware configuration (const, flash-resident).
 * One entry per motor in g_motor_hw[MCTRL_COUNT].
 *
 * No enable pin: TMC2660 uses SPI current control (CS field = 0 disables).
 * Size: 28 bytes per entry, 224 bytes total.
 */
typedef struct {
    TIM_TypeDef*    tim;            /* Timer peripheral base address */
    uint32_t        tim_rcc;        /* RCC peripheral clock bit */
    IRQn_Type       tim_irqn;       /* NVIC IRQ number */
    uint16_t        tim_it_src;     /* TIM_IT_CCx or TIM_IT_Update */
    GPIO_TypeDef*   step_port;      /* Step pulse GPIO port */
    uint16_t        step_pin;       /* Step pulse GPIO pin mask */
    GPIO_TypeDef*   dir_port;       /* Direction GPIO port */
    uint16_t        dir_pin;        /* Direction GPIO pin mask */
    GPIO_TypeDef*   cs_port;        /* TMC2660 SPI CS port */
    uint16_t        cs_pin;         /* TMC2660 SPI CS pin mask */
    uint8_t         nvic_prio;      /* NVIC preemption priority (0=highest) */
} MotorHW_t;

/**
 * Pre-computed CCR profile ring buffer.
 *
 * The profile builder fills buf[0..total_steps-1] with timer ARR reload
 * values. The ISR reads them sequentially via rd_idx.
 *
 * Memory: 4096 * 2 = 8192 bytes per motor.
 */
typedef struct {
    uint16_t            buf[PROFILE_BUF_SIZE];
    volatile uint32_t   rd_idx;         /* ISR read index */
    uint32_t            wr_idx;         /* Builder write index */
    uint32_t            total_steps;    /* Number of valid entries */
} MotorProfile_t;

/**
 * Runtime motor control block.
 * One instance per motor in g_motors[MCTRL_COUNT].
 *
 * Size: ~8232 bytes per motor (dominated by profile buffer).
 */
typedef struct {
    const MotorHW_t*        hw;             /* -> flash HW config */
    MotorProfile_t          profile;        /* Pre-computed CCR LUT */
    volatile MotorState_t   state;          /* Current motion phase */
    volatile int32_t        position;       /* Absolute position (steps) */
    int32_t                 target_pos;     /* Target position */
    uint8_t                 direction;      /* 0=ORG, 1=LIMIT */
    volatile uint8_t        done_flag;      /* ISR sets when motion complete */
    volatile uint8_t        error_flag;     /* ISR sets on fault */
    ProfileType_t           prof_type;      /* Active profile type */
    volatile uint8_t        toggle;         /* Step pulse phase (0/1) */
    /* TMC2660 configuration shadow */
    uint8_t                 microstep_res;  /* Resolution code 0-6 */
    uint16_t                run_current;    /* SGCSCONF CS field for run */
    uint16_t                hold_current;   /* SGCSCONF CS field for hold */
} MotorCtrl_t;

/* Exported variables ---------------------------------------------------- */
extern MotorCtrl_t          g_motors[MCTRL_COUNT];
extern const MotorHW_t     g_motor_hw[MCTRL_COUNT];

/* Exported functions ---------------------------------------------------- */

/**
 * Initialize all motor control structures and configure hardware.
 * Call once at startup after system clocks are configured.
 */
void MotorCtrl_Init(void);

/**
 * Start motion on a single motor. Profile must be built first.
 */
void MotorCtrl_Start(MotorId_t id);

/**
 * Immediately stop a single motor (timer disable).
 */
void MotorCtrl_Stop(MotorId_t id);

/**
 * Emergency stop all 8 motors via direct register access.
 * Execution time: < 500 ns at 120 MHz.
 */
void MotorCtrl_EmergencyStopAll(void);

/**
 * Build trapezoidal velocity profile (integer-only math).
 *
 * @param id            Motor index
 * @param steps         Total step count (must be > 0, <= PROFILE_BUF_SIZE)
 * @param max_freq_hz   Cruise frequency (Hz)
 * @param accel_hz_per_s Acceleration rate (Hz per second)
 * @return 0 on success, -1 on invalid parameters
 */
int32_t MotorCtrl_BuildTrapProfile(MotorId_t id,
                                    int32_t steps,
                                    uint32_t max_freq_hz,
                                    uint32_t accel_hz_per_s);

/**
 * Build arch profile from pre-computed CCR array.
 *
 * @param id          Motor index
 * @param ccr_array   Array of uint16_t CCR values
 * @param array_size  Number of entries (<= PROFILE_BUF_SIZE)
 * @return 0 on success, -1 on overflow
 */
int32_t MotorCtrl_BuildArchProfile(MotorId_t id,
                                    const uint16_t* ccr_array,
                                    uint32_t array_size);

/**
 * Set direction GPIO for a motor.
 * @param dir  0 = toward ORG, 1 = toward LIMIT
 */
void MotorCtrl_SetDirection(MotorId_t id, uint8_t dir);

/** @return 1 if motor is in ACCEL/CRUISE/DECEL state */
uint8_t MotorCtrl_IsRunning(MotorId_t id);

/** @return 1 if motor reached DONE state */
uint8_t MotorCtrl_IsDone(MotorId_t id);

/** @return Current absolute position in steps */
int32_t MotorCtrl_GetPosition(MotorId_t id);

/** Override current position (use after homing) */
void MotorCtrl_SetPosition(MotorId_t id, int32_t pos);

/**
 * Start multiple motors synchronously (minimal skew).
 * All specified timers enabled in a critical section.
 */
void MotorCtrl_StartSync(const MotorId_t* ids, uint8_t count);

/** Send TMC2660 default init registers for one motor */
void MotorCtrl_TMC2660_Init(MotorId_t id);

/**
 * Set TMC2660 run/hold current via SGCSCONF register.
 * @param run_ma   CS field value (0-31), not milliamps
 * @param hold_ma  CS field value (0-31)
 */
void MotorCtrl_TMC2660_SetCurrent(MotorId_t id,
                                   uint16_t run_ma,
                                   uint16_t hold_ma);

/**
 * Set TMC2660 microstep resolution via DRVCTRL register.
 * @param resolution  0=256x, 1=128x, 2=64x, 3=32x, 4=16x, 5=8x, 6=4x
 */
void MotorCtrl_TMC2660_SetMicrostep(MotorId_t id, uint8_t resolution);

/**
 * ISR handler for one motor. Called from TIMx_IRQHandler.
 *
 * Cycle budget: ~30 cycles worst case.
 * Operations: clear IT flag, read CCR from LUT, write ARR, toggle step GPIO,
 *             update position, advance index, check completion.
 */
void MotorCtrl_ISR_Handler(MotorId_t id);

#endif /* __MOTOR_CTRL_H__ */
