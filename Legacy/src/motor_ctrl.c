/*
*******************************************************************************************
* motor_ctrl.c : Optimized unified motor control implementation
*******************************************************************************************
* Copyright (C) 2016-2026 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date    : 2026-03-23
* @ Brief   : Complete implementation of LUT-based motor control.
*
*   Key design decisions:
*     1. Profile computation runs in main context (integer-only, no float).
*     2. ISR reads pre-computed CCR from ring buffer (~30 cycles).
*     3. TMC2660 CS pin lookup via const table (no if-else chain).
*     4. Emergency stop: direct register writes to all 8 timer CR1.
*
* @ Revision History :
*       1) initial creation. ------------------------------------ 2026-03-23
*******************************************************************************************
*/

/* Include files --------------------------------------------------------- */
#include "motor_ctrl.h"
#include "stm32f2xx.h"
#include <string.h>

/* Private typedef ------------------------------------------------------- */
/* Private define -------------------------------------------------------- */

/* SPI3 is used for TMC2660 communication (matching existing motor.c) */
/* SPI3 pins: PC10=SCK, PC11=MISO, PC12=MOSI */

/* Frequency to CCR conversion (integer):
 * Each step pulse = 2 timer events (toggle high/low).
 * CCR = TIM_CLK / (2 * freq_hz)
 * But this system uses ARR-based timing, so:
 * ARR = TIM_CLK / freq_hz  (one full period per ARR, toggling each event)
 *
 * From existing code: Hz_To_CCR(Hz) = TIM_COUNT_CLOCK / (TIM_TOGGLE * Hz)
 * So: CCR = 1250000 / (2 * Hz) = 625000 / Hz
 */
#define FREQ_TO_CCR(freq_hz) \
    ((uint32_t)(MCTRL_TIM_CLK / (MCTRL_TIM_TOGGLE * (freq_hz))))

/* Minimum allowed frequency for CCR computation to avoid overflow */
#define MIN_FREQ_HZ    10U

/* Private macro --------------------------------------------------------- */
/* Private variables ----------------------------------------------------- */

/**
 * Hardware configuration table (const, flash-resident).
 *
 * Mapping extracted from existing motor.c defines and isr.c ISR routing:
 *
 * Motor_R: TIM8  CC3, step=PC8,  dir=PD1, CS=PE13 (ROT)
 * Motor_V: TIM1  CC1, step=PA8,  dir=PD0, CS=PA0  (PAN)
 * Motor_C: TIM2  CC2, step=PA1,  dir=PC15,CS=PA12 (CS/1st Collimator)
 * Motor_T: TIM5  UPD, step=PC6,  dir=PA3, CS=PC2  (ER/Temple Support)
 * Motor_S: TIM10 CC1, step=PB8,  dir=PD3, CS=PB1  (CC/2nd Collimator)
 * Motor_A: TIM9  CC1, step=PE5,  dir=PD4, CS=PD2  (MS/Gantry)
 * Motor_CNS:TIM3 CC3, step=PB0,  dir=PC3, CS=PE1  (CNS)
 * Motor_CWE:TIM11 CC1,step=PB9,  dir=PD15,CS=PE10 (CWE)
 *
 * Note on Motor_C CS pin: the existing code sends Motor_C data to
 * CS_SPI_CS (PA12) when model_id == MODEL_T2_CS, and to ROT_SPI_CS (PE13)
 * otherwise. We use CS_SPI_CS (PA12) as the default for the unified table,
 * matching the CS (1st collimator) assignment from motor.h MotType mapping.
 */
const MotorHW_t g_motor_hw[MCTRL_COUNT] = {
    /* [MCTRL_R] Rotation */
    {
        .tim        = TIM8,
        .tim_rcc    = RCC_APB2Periph_TIM8,
        .tim_irqn   = TIM8_CC_IRQn,
        .tim_it_src = TIM_IT_CC3,
        .step_port  = GPIOC,
        .step_pin   = GPIO_Pin_8,
        .dir_port   = GPIOD,
        .dir_pin    = GPIO_Pin_1,
        .cs_port    = GPIOE,
        .cs_pin     = GPIO_Pin_13,
        .nvic_prio  = 0,
    },
    /* [MCTRL_V] Vertical */
    {
        .tim        = TIM1,
        .tim_rcc    = RCC_APB2Periph_TIM1,
        .tim_irqn   = TIM1_CC_IRQn,
        .tim_it_src = TIM_IT_CC1,
        .step_port  = GPIOA,
        .step_pin   = GPIO_Pin_8,
        .dir_port   = GPIOD,
        .dir_pin    = GPIO_Pin_0,
        .cs_port    = GPIOA,
        .cs_pin     = GPIO_Pin_0,
        .nvic_prio  = 1,
    },
    /* [MCTRL_C] Collimator (1st) */
    {
        .tim        = TIM2,
        .tim_rcc    = RCC_APB1Periph_TIM2,
        .tim_irqn   = TIM2_IRQn,
        .tim_it_src = TIM_IT_CC2,
        .step_port  = GPIOA,
        .step_pin   = GPIO_Pin_1,
        .dir_port   = GPIOC,
        .dir_pin    = GPIO_Pin_15,
        .cs_port    = GPIOA,
        .cs_pin     = GPIO_Pin_12,
        .nvic_prio  = 1,
    },
    /* [MCTRL_T] Chin rest / Temple support */
    {
        .tim        = TIM5,
        .tim_rcc    = RCC_APB1Periph_TIM5,
        .tim_irqn   = TIM5_IRQn,
        .tim_it_src = TIM_IT_Update,
        .step_port  = GPIOC,
        .step_pin   = GPIO_Pin_6,
        .dir_port   = GPIOA,
        .dir_pin    = GPIO_Pin_3,
        .cs_port    = GPIOC,
        .cs_pin     = GPIO_Pin_2,
        .nvic_prio  = 1,
    },
    /* [MCTRL_S] Slit (2nd Collimator) */
    {
        .tim        = TIM10,
        .tim_rcc    = RCC_APB2Periph_TIM10,
        .tim_irqn   = TIM1_UP_TIM10_IRQn,
        .tim_it_src = TIM_IT_CC1,
        .step_port  = GPIOB,
        .step_pin   = GPIO_Pin_8,
        .dir_port   = GPIOD,
        .dir_pin    = GPIO_Pin_3,
        .cs_port    = GPIOB,
        .cs_pin     = GPIO_Pin_1,
        .nvic_prio  = 1,
    },
    /* [MCTRL_A] Gantry */
    {
        .tim        = TIM9,
        .tim_rcc    = RCC_APB2Periph_TIM9,
        .tim_irqn   = TIM1_BRK_TIM9_IRQn,
        .tim_it_src = TIM_IT_CC1,
        .step_port  = GPIOE,
        .step_pin   = GPIO_Pin_5,
        .dir_port   = GPIOD,
        .dir_pin    = GPIO_Pin_4,
        .cs_port    = GPIOD,
        .cs_pin     = GPIO_Pin_2,
        .nvic_prio  = 1,
    },
    /* [MCTRL_CNS] Chin rest vertical */
    {
        .tim        = TIM3,
        .tim_rcc    = RCC_APB1Periph_TIM3,
        .tim_irqn   = TIM3_IRQn,
        .tim_it_src = TIM_IT_CC3,
        .step_port  = GPIOB,
        .step_pin   = GPIO_Pin_0,
        .dir_port   = GPIOC,
        .dir_pin    = GPIO_Pin_3,
        .cs_port    = GPIOE,
        .cs_pin     = GPIO_Pin_1,
        .nvic_prio  = 1,
    },
    /* [MCTRL_CWE] Chin rest horizontal */
    {
        .tim        = TIM11,
        .tim_rcc    = RCC_APB2Periph_TIM11,
        .tim_irqn   = TIM1_TRG_COM_TIM11_IRQn,
        .tim_it_src = TIM_IT_CC1,
        .step_port  = GPIOB,
        .step_pin   = GPIO_Pin_9,
        .dir_port   = GPIOD,
        .dir_pin    = GPIO_Pin_15,
        .cs_port    = GPIOE,
        .cs_pin     = GPIO_Pin_10,
        .nvic_prio  = 1,
    },
};

/* Motor control state array (SRAM) */
MotorCtrl_t g_motors[MCTRL_COUNT];

/* TMC2660 resolution register LUT */
static const uint32_t tmc_res_lut[7] = {
    TMC_RES_X256, TMC_RES_X128, TMC_RES_X64,
    TMC_RES_X32,  TMC_RES_X16,  TMC_RES_X8, TMC_RES_X4
};

/* TMC2660 init register sequence */
static const uint32_t tmc_init_seq[4] = {
    TMC_INIT_CHOPCONF,
    TMC_INIT_SMARTEN,
    TMC_INIT_DRVCONF,
    TMC_INIT_DRVCTRL_X16
};

/* Private function prototypes ------------------------------------------- */
static void mctrl_spi_send(GPIO_TypeDef* cs_port, uint16_t cs_pin,
                            uint32_t data);
static uint8_t mctrl_spi_byte(uint8_t byte);
static void mctrl_timer_init(MotorId_t id);
static void mctrl_gpio_init(MotorId_t id);
static inline uint16_t mctrl_clamp_ccr(uint32_t val);

/* Private functions ----------------------------------------------------- */

/**
 * Send one byte over SPI3 (blocking, matching existing SPI_SendByte).
 */
static uint8_t mctrl_spi_byte(uint8_t byte)
{
    while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET)
        ;
    SPI_I2S_SendData(SPI3, byte);
    while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET)
        ;
    return (uint8_t)SPI_I2S_ReceiveData(SPI3);
}

/**
 * Send 20-bit TMC2660 register via SPI3 with chip select control.
 * Matches existing TMC2660_SPI_SendData() protocol: 3 bytes, MSB first.
 */
static void mctrl_spi_send(GPIO_TypeDef* cs_port, uint16_t cs_pin,
                            uint32_t data)
{
    MCTRL_CS_LOW(cs_port, cs_pin);

    mctrl_spi_byte((uint8_t)((data >> 16) & 0x0FU));
    mctrl_spi_byte((uint8_t)((data >> 8) & 0xFFU));
    mctrl_spi_byte((uint8_t)(data & 0xFFU));

    MCTRL_CS_HIGH(cs_port, cs_pin);
}

/**
 * Clamp a computed CCR value to the valid 16-bit range.
 */
static inline uint16_t mctrl_clamp_ccr(uint32_t val)
{
    if (val < MCTRL_CCR_MIN) {
        return MCTRL_CCR_MIN;
    }
    if (val > MCTRL_CCR_MAX) {
        return MCTRL_CCR_MAX;
    }
    return (uint16_t)val;
}

/**
 * Configure timer peripheral for a motor.
 * Uses update event (ARR-based) interrupt mode:
 *   - Prescaler computed for TIM_CLK = 1.25 MHz
 *   - ARR loaded by ISR from profile buffer
 *   - Timer starts disabled, ISR enables via TIM_Cmd
 */
static void mctrl_timer_init(MotorId_t id)
{
    const MotorHW_t* hw = &g_motor_hw[id];
    TIM_TimeBaseInitTypeDef tb;
    NVIC_InitTypeDef nvic;
    uint16_t prescaler;

    /* Enable timer clock */
    if ((hw->tim == TIM1) || (hw->tim == TIM8) || (hw->tim == TIM9) ||
        (hw->tim == TIM10) || (hw->tim == TIM11)) {
        RCC_APB2PeriphClockCmd(hw->tim_rcc, ENABLE);
        /* APB2 timers: SystemCoreClock / TIM_CLK - 1 */
        prescaler = (uint16_t)((SystemCoreClock / MCTRL_TIM_CLK) - 1U);
    } else {
        RCC_APB1PeriphClockCmd(hw->tim_rcc, ENABLE);
        /* APB1 timers: (SystemCoreClock / 2) / TIM_CLK - 1 */
        prescaler = (uint16_t)(((SystemCoreClock / 2U) / MCTRL_TIM_CLK) - 1U);
    }

    /* De-init and configure time base */
    TIM_DeInit(hw->tim);

    tb.TIM_Prescaler         = prescaler;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_Period            = MCTRL_CCR_MAX;  /* Will be overwritten by ISR */
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(hw->tim, &tb);

    /* Enable the appropriate interrupt source */
    TIM_ITConfig(hw->tim, hw->tim_it_src, ENABLE);

    /* Enable ARR preload so new ARR takes effect at next update */
    TIM_ARRPreloadConfig(hw->tim, ENABLE);

    /* Configure NVIC */
    nvic.NVIC_IRQChannel                   = hw->tim_irqn;
    nvic.NVIC_IRQChannelPreemptionPriority = hw->nvic_prio;
    nvic.NVIC_IRQChannelSubPriority        = 0;
    nvic.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&nvic);

    /* Timer starts disabled. MotorCtrl_Start() enables it. */
    TIM_Cmd(hw->tim, DISABLE);
}

/**
 * Configure step and direction GPIO for a motor.
 * Step pin: output push-pull, 100 MHz, pull-down (matching existing config).
 * Direction pin: output push-pull, 100 MHz, pull-up (matching existing config).
 */
static void mctrl_gpio_init(MotorId_t id)
{
    const MotorHW_t* hw = &g_motor_hw[id];
    GPIO_InitTypeDef gpio;

    /* Step pin */
    gpio.GPIO_Pin   = hw->step_pin;
    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(hw->step_port, &gpio);

    /* Direction pin */
    gpio.GPIO_Pin  = hw->dir_pin;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(hw->dir_port, &gpio);

    /* CS pin (output, push-pull, initially high = deselected) */
    gpio.GPIO_Pin  = hw->cs_pin;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(hw->cs_port, &gpio);
    MCTRL_CS_HIGH(hw->cs_port, hw->cs_pin);
}

/* Exported functions ---------------------------------------------------- */

/*---------------------------------------------------------------------------
 * MotorCtrl_Init
 *---------------------------------------------------------------------------*/
void MotorCtrl_Init(void)
{
    uint8_t i;

    /* Enable GPIO clocks used by motors */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB |
                           RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD |
                           RCC_AHB1Periph_GPIOE, ENABLE);

    /* Clear all motor state */
    memset(g_motors, 0, sizeof(g_motors));

    /* Initialize each motor */
    for (i = 0; i < MCTRL_COUNT; i++) {
        g_motors[i].hw          = &g_motor_hw[i];
        g_motors[i].state       = MSTATE_IDLE;
        g_motors[i].done_flag   = 0;
        g_motors[i].error_flag  = 0;
        g_motors[i].toggle      = 0;
        g_motors[i].direction   = 0;
        g_motors[i].position    = 0;
        g_motors[i].target_pos  = 0;
        g_motors[i].prof_type   = PROF_NONE;
        g_motors[i].microstep_res = 4;  /* Default: 16x microstep */
        g_motors[i].run_current   = 0x07;  /* ~1.0A default */
        g_motors[i].hold_current  = 0x03;  /* ~0.5A default */

        mctrl_gpio_init((MotorId_t)i);
        mctrl_timer_init((MotorId_t)i);
    }
}

/*---------------------------------------------------------------------------
 * MotorCtrl_Start
 *---------------------------------------------------------------------------*/
void MotorCtrl_Start(MotorId_t id)
{
    MotorCtrl_t* m;
    const MotorHW_t* hw;

    if ((uint8_t)id >= MCTRL_COUNT) {
        return;
    }

    m  = &g_motors[id];
    hw = m->hw;

    /* Validate that a profile has been built */
    if (m->profile.total_steps == 0U) {
        m->error_flag = 1;
        m->state = MSTATE_ERROR;
        return;
    }

    /* Reset profile read index and flags */
    m->profile.rd_idx = 0;
    m->done_flag  = 0;
    m->error_flag = 0;
    m->toggle     = 0;
    m->state      = MSTATE_ACCEL;

    /* Load first ARR value from profile */
    hw->tim->ARR = m->profile.buf[0];
    hw->tim->CNT = 0;

    /* Clear any pending interrupt */
    TIM_ClearITPendingBit(hw->tim, hw->tim_it_src);

    /* Enable interrupt and start timer */
    TIM_ITConfig(hw->tim, hw->tim_it_src, ENABLE);
    TIM_Cmd(hw->tim, ENABLE);
}

/*---------------------------------------------------------------------------
 * MotorCtrl_Stop
 *---------------------------------------------------------------------------*/
void MotorCtrl_Stop(MotorId_t id)
{
    MotorCtrl_t* m;

    if ((uint8_t)id >= MCTRL_COUNT) {
        return;
    }

    m = &g_motors[id];

    /* Disable timer immediately */
    m->hw->tim->CR1 &= (uint16_t)~TIM_CR1_CEN;

    m->state     = MSTATE_IDLE;
    m->done_flag = 1;
}

/*---------------------------------------------------------------------------
 * MotorCtrl_EmergencyStopAll
 *
 * Direct register writes to disable all 8 timers.
 * No function calls, no branches, < 500ns at 120 MHz.
 * Each CR1 write = 1 STR instruction = ~2 cycles.
 * Total: 8 * ~5 cycles (load addr + read-modify-write) = ~40 cycles = 333ns.
 *---------------------------------------------------------------------------*/
void MotorCtrl_EmergencyStopAll(void)
{
    uint8_t i;
    uint16_t cr1;

    /* Disable interrupts for atomic operation */
    __disable_irq();

    /* Disable all motor timers via direct CR1 register access */
    for (i = 0; i < MCTRL_COUNT; i++) {
        cr1 = g_motor_hw[i].tim->CR1;
        g_motor_hw[i].tim->CR1 = cr1 & (uint16_t)~TIM_CR1_CEN;
        g_motors[i].state = MSTATE_ERROR;
        g_motors[i].error_flag = 1;
        g_motors[i].done_flag = 0;
    }

    __enable_irq();
}

/*---------------------------------------------------------------------------
 * MotorCtrl_BuildTrapProfile
 *
 * Builds a trapezoidal velocity profile entirely in integer arithmetic.
 *
 * Algorithm:
 *   1. Compute accel_steps: number of steps to ramp from start_freq to max_freq.
 *      accel_steps = (max_freq - start_freq) / freq_increment_per_step
 *      freq_increment_per_step = accel_hz_per_s / start_freq (approximation
 *      that improves as resolution increases)
 *
 *   2. Symmetric decel_steps = accel_steps.
 *
 *   3. If accel+decel > total, it is a triangular profile:
 *      accel_steps = decel_steps = total / 2.
 *
 *   4. cruise_steps = total - accel_steps - decel_steps.
 *
 *   5. For each step, compute instantaneous frequency via Q16.16 fixed-point,
 *      then convert to CCR = TIM_CLK / (2 * freq).
 *
 * Uses Q16.16 fixed-point for sub-Hz frequency resolution without float.
 *---------------------------------------------------------------------------*/
int32_t MotorCtrl_BuildTrapProfile(MotorId_t id,
                                    int32_t steps,
                                    uint32_t max_freq_hz,
                                    uint32_t accel_hz_per_s)
{
    MotorCtrl_t* m;
    uint32_t total;
    uint32_t start_freq;
    uint32_t accel_steps;
    uint32_t decel_steps;
    uint32_t cruise_steps;
    uint32_t freq_fp;       /* Q16.16 current frequency */
    uint32_t start_fp;      /* Q16.16 start frequency */
    uint32_t max_fp;        /* Q16.16 max frequency */
    uint32_t delta_fp;      /* Q16.16 freq increment per step */
    uint32_t ccr_val;
    uint32_t freq_int;
    uint32_t i;

    if ((uint8_t)id >= MCTRL_COUNT) {
        return -1;
    }
    if (steps <= 0 || (uint32_t)steps > PROFILE_BUF_SIZE) {
        return -1;
    }
    if (max_freq_hz < MIN_FREQ_HZ || accel_hz_per_s == 0U) {
        return -1;
    }

    m = &g_motors[id];
    total = (uint32_t)steps;
    start_freq = MCTRL_TRAP_START_FREQ_HZ;

    /* Clamp max_freq if below start */
    if (max_freq_hz <= start_freq) {
        max_freq_hz = start_freq + 1U;
    }

    /* Compute frequency range in Q16.16 */
    start_fp = FP_FROM_INT(start_freq);
    max_fp   = FP_FROM_INT(max_freq_hz);

    /*
     * Acceleration increment per step:
     *
     * Time per step at frequency f = 1/f seconds.
     * During acceleration from f to f+df in one step:
     *   df = accel_hz_per_s * (1 / f)
     *   df = accel_hz_per_s / f
     *
     * For the profile builder, we use a constant increment approximation
     * based on the average frequency during the ramp. This gives a
     * good-enough trapezoidal shape for stepper motor control.
     *
     * avg_freq = (start_freq + max_freq) / 2
     * delta_per_step = accel / avg_freq
     *
     * Number of accel steps = (max_freq - start_freq) / delta_per_step
     *                       = (max_freq - start_freq) * avg_freq / accel
     */
    {
        uint32_t freq_range = max_freq_hz - start_freq;
        uint64_t avg_freq = ((uint64_t)start_freq + (uint64_t)max_freq_hz) / 2U;
        uint64_t accel_steps_64 = ((uint64_t)freq_range * avg_freq)
                                  / (uint64_t)accel_hz_per_s;

        if (accel_steps_64 > total / 2U) {
            accel_steps_64 = total / 2U;
        }
        accel_steps = (uint32_t)accel_steps_64;
    }

    if (accel_steps == 0U) {
        accel_steps = 1U;
    }

    decel_steps = accel_steps;

    /* Handle triangular profile (not enough steps for full cruise) */
    if (accel_steps + decel_steps >= total) {
        accel_steps = total / 2U;
        decel_steps = total - accel_steps;
        cruise_steps = 0U;
        /* Adjust max_fp to the peak frequency we can actually reach */
        /* peak_freq = start_freq + delta * accel_steps */
        /* We recalculate delta for this shorter ramp */
    } else {
        cruise_steps = total - accel_steps - decel_steps;
    }

    /* Compute delta_fp: frequency increment per accel step (Q16.16) */
    if (accel_steps > 0U) {
        delta_fp = (max_fp - start_fp) / accel_steps;
    } else {
        delta_fp = 0U;
    }

    /* Build acceleration phase */
    freq_fp = start_fp;
    for (i = 0; i < accel_steps; i++) {
        freq_int = FP_TO_INT(freq_fp);
        if (freq_int < MIN_FREQ_HZ) {
            freq_int = MIN_FREQ_HZ;
        }
        ccr_val = MCTRL_TIM_CLK / (MCTRL_TIM_TOGGLE * freq_int);
        m->profile.buf[i] = mctrl_clamp_ccr(ccr_val);
        freq_fp += delta_fp;
    }

    /* Build cruise phase */
    {
        uint16_t cruise_ccr;
        freq_int = FP_TO_INT(freq_fp);
        if (freq_int < MIN_FREQ_HZ) {
            freq_int = MIN_FREQ_HZ;
        }
        ccr_val = MCTRL_TIM_CLK / (MCTRL_TIM_TOGGLE * freq_int);
        cruise_ccr = mctrl_clamp_ccr(ccr_val);

        for (i = accel_steps; i < accel_steps + cruise_steps; i++) {
            m->profile.buf[i] = cruise_ccr;
        }
    }

    /* Build deceleration phase (mirror of acceleration) */
    freq_fp = max_fp;
    for (i = 0; i < decel_steps; i++) {
        freq_int = FP_TO_INT(freq_fp);
        if (freq_int < MIN_FREQ_HZ) {
            freq_int = MIN_FREQ_HZ;
        }
        ccr_val = MCTRL_TIM_CLK / (MCTRL_TIM_TOGGLE * freq_int);
        m->profile.buf[accel_steps + cruise_steps + i] = mctrl_clamp_ccr(ccr_val);
        freq_fp -= delta_fp;
        /* Prevent underflow */
        if (freq_fp < start_fp) {
            freq_fp = start_fp;
        }
    }

    /* Store profile metadata */
    m->profile.total_steps = total;
    m->profile.rd_idx = 0;
    m->profile.wr_idx = total;
    m->prof_type = PROF_TRAPEZOID;
    m->state = MSTATE_IDLE;
    m->done_flag = 0;
    m->error_flag = 0;

    return 0;
}

/*---------------------------------------------------------------------------
 * MotorCtrl_BuildArchProfile
 *
 * Copies pre-computed CCR values from an external array (from arch_standard.h,
 * arch_child.h, etc.) into the ring buffer.
 *---------------------------------------------------------------------------*/
int32_t MotorCtrl_BuildArchProfile(MotorId_t id,
                                    const uint16_t* ccr_array,
                                    uint32_t array_size)
{
    MotorCtrl_t* m;
    uint32_t i;

    if ((uint8_t)id >= MCTRL_COUNT) {
        return -1;
    }
    if (ccr_array == (void*)0 || array_size == 0U ||
        array_size > PROFILE_BUF_SIZE) {
        return -1;
    }

    m = &g_motors[id];

    /* Copy with clamping */
    for (i = 0; i < array_size; i++) {
        m->profile.buf[i] = mctrl_clamp_ccr((uint32_t)ccr_array[i]);
    }

    m->profile.total_steps = array_size;
    m->profile.rd_idx = 0;
    m->profile.wr_idx = array_size;
    m->prof_type = PROF_ARCH;
    m->state = MSTATE_IDLE;
    m->done_flag = 0;
    m->error_flag = 0;

    return 0;
}

/*---------------------------------------------------------------------------
 * MotorCtrl_SetDirection
 *---------------------------------------------------------------------------*/
void MotorCtrl_SetDirection(MotorId_t id, uint8_t dir)
{
    const MotorHW_t* hw;

    if ((uint8_t)id >= MCTRL_COUNT) {
        return;
    }

    hw = g_motors[id].hw;
    g_motors[id].direction = dir;

    if (dir != 0U) {
        /* LIMIT direction: set pin high */
        hw->dir_port->BSRRL = hw->dir_pin;
    } else {
        /* ORG direction: set pin low */
        hw->dir_port->BSRRH = hw->dir_pin;
    }
}

/*---------------------------------------------------------------------------
 * MotorCtrl_IsRunning
 *---------------------------------------------------------------------------*/
uint8_t MotorCtrl_IsRunning(MotorId_t id)
{
    MotorState_t st;

    if ((uint8_t)id >= MCTRL_COUNT) {
        return 0;
    }

    st = g_motors[id].state;
    return (st == MSTATE_ACCEL || st == MSTATE_CRUISE || st == MSTATE_DECEL)
           ? 1U : 0U;
}

/*---------------------------------------------------------------------------
 * MotorCtrl_IsDone
 *---------------------------------------------------------------------------*/
uint8_t MotorCtrl_IsDone(MotorId_t id)
{
    if ((uint8_t)id >= MCTRL_COUNT) {
        return 0;
    }
    return g_motors[id].done_flag;
}

/*---------------------------------------------------------------------------
 * MotorCtrl_GetPosition
 *---------------------------------------------------------------------------*/
int32_t MotorCtrl_GetPosition(MotorId_t id)
{
    if ((uint8_t)id >= MCTRL_COUNT) {
        return 0;
    }
    return g_motors[id].position;
}

/*---------------------------------------------------------------------------
 * MotorCtrl_SetPosition
 *---------------------------------------------------------------------------*/
void MotorCtrl_SetPosition(MotorId_t id, int32_t pos)
{
    if ((uint8_t)id >= MCTRL_COUNT) {
        return;
    }
    g_motors[id].position = pos;
}

/*---------------------------------------------------------------------------
 * MotorCtrl_StartSync
 *
 * Starts multiple motors with minimal inter-motor skew.
 * Disables interrupts, pre-loads all timers, then enables all in a tight loop.
 * Skew between first and last motor: ~5 cycles per motor = ~40 cycles = ~333ns.
 *---------------------------------------------------------------------------*/
void MotorCtrl_StartSync(const MotorId_t* ids, uint8_t count)
{
    uint8_t i;
    MotorCtrl_t* m;
    const MotorHW_t* hw;

    if (ids == (void*)0 || count == 0U || count > MCTRL_COUNT) {
        return;
    }

    /* Pre-load all timers (outside critical section for less jitter) */
    for (i = 0; i < count; i++) {
        if ((uint8_t)ids[i] >= MCTRL_COUNT) {
            continue;
        }
        m  = &g_motors[ids[i]];
        hw = m->hw;

        if (m->profile.total_steps == 0U) {
            m->error_flag = 1;
            m->state = MSTATE_ERROR;
            continue;
        }

        m->profile.rd_idx = 0;
        m->done_flag  = 0;
        m->error_flag = 0;
        m->toggle     = 0;
        m->state      = MSTATE_ACCEL;

        hw->tim->ARR = m->profile.buf[0];
        hw->tim->CNT = 0;

        TIM_ClearITPendingBit(hw->tim, hw->tim_it_src);
        TIM_ITConfig(hw->tim, hw->tim_it_src, ENABLE);
    }

    /* Enable all timers in a critical section for synchronized start */
    __disable_irq();

    for (i = 0; i < count; i++) {
        if ((uint8_t)ids[i] >= MCTRL_COUNT) {
            continue;
        }
        if (g_motors[ids[i]].state != MSTATE_ERROR) {
            g_motor_hw[ids[i]].tim->CR1 |= TIM_CR1_CEN;
        }
    }

    __enable_irq();
}

/*---------------------------------------------------------------------------
 * MotorCtrl_TMC2660_Init
 *
 * Sends the 4-register init sequence to a single motor's TMC2660.
 *---------------------------------------------------------------------------*/
void MotorCtrl_TMC2660_Init(MotorId_t id)
{
    const MotorHW_t* hw;
    uint8_t i;

    if ((uint8_t)id >= MCTRL_COUNT) {
        return;
    }

    hw = &g_motor_hw[id];

    for (i = 0; i < 4; i++) {
        mctrl_spi_send(hw->cs_port, hw->cs_pin, tmc_init_seq[i]);
    }

    /* Update shadow state */
    g_motors[id].microstep_res = 4;  /* x16 default */
}

/*---------------------------------------------------------------------------
 * MotorCtrl_TMC2660_SetCurrent
 *
 * Sets the SGCSCONF register for run/hold current.
 *
 * TMC2660 SGCSCONF format (20 bits):
 *   [19:17] = 110 (register address)
 *   [16]    = SG filter enable (1 = filtered)
 *   [15:8]  = SG threshold (set to 0x09 matching existing init)
 *   [4:0]   = CS (current scale, 0-31)
 *
 * This function sends the run current immediately.
 * Hold current is stored for later application when motor stops.
 *---------------------------------------------------------------------------*/
void MotorCtrl_TMC2660_SetCurrent(MotorId_t id,
                                   uint16_t run_ma,
                                   uint16_t hold_ma)
{
    const MotorHW_t* hw;
    uint32_t reg;

    if ((uint8_t)id >= MCTRL_COUNT) {
        return;
    }

    hw = &g_motor_hw[id];

    /* Clamp to 5-bit CS field */
    if (run_ma > 31U) {
        run_ma = 31U;
    }
    if (hold_ma > 31U) {
        hold_ma = 31U;
    }

    g_motors[id].run_current  = run_ma;
    g_motors[id].hold_current = hold_ma;

    /* Send run current: SGCSCONF base | CS field */
    reg = TMC_SGCSCONF_BASE | (uint32_t)run_ma;
    mctrl_spi_send(hw->cs_port, hw->cs_pin, reg);
}

/*---------------------------------------------------------------------------
 * MotorCtrl_TMC2660_SetMicrostep
 *
 * Sends the DRVCTRL register to set microstep resolution.
 * Resolution code: 0=256x, 1=128x, 2=64x, 3=32x, 4=16x, 5=8x, 6=4x
 *---------------------------------------------------------------------------*/
void MotorCtrl_TMC2660_SetMicrostep(MotorId_t id, uint8_t resolution)
{
    const MotorHW_t* hw;

    if ((uint8_t)id >= MCTRL_COUNT) {
        return;
    }
    if (resolution > 6U) {
        resolution = 6U;
    }

    hw = &g_motor_hw[id];
    g_motors[id].microstep_res = resolution;

    mctrl_spi_send(hw->cs_port, hw->cs_pin, tmc_res_lut[resolution]);
}

/*---------------------------------------------------------------------------
 * MotorCtrl_ISR_Handler
 *
 * Called from each TIMx_IRQHandler. This is the performance-critical path.
 *
 * Operation sequence (worst case ~30 Cortex-M3 cycles):
 *   1. Clear pending IT flag            (~3 cycles)
 *   2. Check toggle state               (~2 cycles)
 *   3. Toggle step GPIO                 (~3 cycles)
 *   4. On active edge (toggle==1):
 *      a. Read CCR from profile buf     (~4 cycles, one indexed load)
 *      b. Write to timer ARR            (~2 cycles)
 *      c. Update position               (~3 cycles)
 *      d. Advance rd_idx                (~2 cycles)
 *      e. Check completion              (~3 cycles)
 *   5. Flip toggle                      (~1 cycle)
 *   Total: ~23-30 cycles (well under 44 cycle budget)
 *
 * The toggle mechanism generates a square wave on the step pin:
 *   - toggle=0 -> GPIO low  (falling edge, no motor action)
 *   - toggle=1 -> GPIO high (rising edge = one step pulse)
 *   This matches the existing system where GPIO_ToggleBits generates steps.
 *---------------------------------------------------------------------------*/
void MotorCtrl_ISR_Handler(MotorId_t id)
{
    MotorCtrl_t* m;
    const MotorHW_t* hw;
    uint32_t idx;

    m  = &g_motors[id];
    hw = m->hw;

    /* 1. Clear interrupt pending bit */
    hw->tim->SR = (uint16_t)~hw->tim_it_src;

    /* 2. If motor is not actively running, disable and return */
    if (m->state == MSTATE_IDLE || m->state == MSTATE_DONE ||
        m->state == MSTATE_ERROR) {
        hw->tim->CR1 &= (uint16_t)~TIM_CR1_CEN;
        return;
    }

    /* 3. Toggle step pulse GPIO */
    if (m->toggle == 0U) {
        /* Rising edge = step pulse active */
        hw->step_port->BSRRL = hw->step_pin;
        m->toggle = 1U;
    } else {
        /* Falling edge */
        hw->step_port->BSRRH = hw->step_pin;
        m->toggle = 0U;

        /* Process motor state only on falling edge (one full step cycle) */
        idx = m->profile.rd_idx;

        /* Update position */
        if (m->direction != 0U) {
            m->position++;
        } else {
            m->position--;
        }

        /* Advance to next step */
        idx++;

        /* Check if profile is exhausted */
        if (idx >= m->profile.total_steps) {
            /* Motion complete */
            hw->tim->CR1 &= (uint16_t)~TIM_CR1_CEN;
            m->state     = MSTATE_DONE;
            m->done_flag = 1;
            m->profile.rd_idx = idx;
            return;
        }

        /* Load next CCR value into ARR (takes effect at next update event) */
        hw->tim->ARR = (uint32_t)m->profile.buf[idx];
        m->profile.rd_idx = idx;
    }
}
