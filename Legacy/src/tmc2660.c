/*
*******************************************************************************************
* tmc2660.c : TMC2660 stepper motor driver - SPI register interface
*******************************************************************************************
* Copyright (C) 2016-2026 Osstem Implant, Inc
*
* @ Authors : Dental Imaging Electronics Firmware Development Team
* @ Date    : 2026-03-24
* @ Brief   : Clean TMC2660 driver extracted from motor.c.
*             All CS pin selection via g_motor_hw[] table lookup.
*             Hardware SPI3: PC10=SCK, PC11=MISO, PC12=MOSI.
*
*   Memory usage:
*     Flash (const):  ~280 bytes (LUTs + init sequence)
*     Flash (code):   ~600 bytes
*     RAM:            ~40 bytes (shadow registers per motor = 5 * MCTRL_COUNT)
*
* @ Revision History :
*       1) initial creation (extracted from motor.c) ---------- 2026-03-24
*******************************************************************************************
*/

/* Includes -------------------------------------------------------------- */
#include "tmc2660.h"
#include "motor_ctrl.h"
#include "stm32f2xx.h"
#include "stm32f2xx_spi.h"
#include "stm32f2xx_gpio.h"
#include "stm32f2xx_rcc.h"

/* Private define -------------------------------------------------------- */

/* SPI3 bus pins: PC10=SCK, PC11=MISO, PC12=MOSI */
#define TMC_SPI_PERIPH              SPI3
#define TMC_SPI_RCC_CMD             RCC_APB1PeriphClockCmd
#define TMC_SPI_RCC_PERIPH          RCC_APB1Periph_SPI3
#define TMC_SPI_GPIO_PORT           GPIOC
#define TMC_SPI_GPIO_RCC            RCC_AHB1Periph_GPIOC
#define TMC_SPI_SCK_PIN             GPIO_Pin_10
#define TMC_SPI_MISO_PIN            GPIO_Pin_11
#define TMC_SPI_MOSI_PIN            GPIO_Pin_12
#define TMC_SPI_SCK_PINSRC          GPIO_PinSource10
#define TMC_SPI_MISO_PINSRC         GPIO_PinSource11
#define TMC_SPI_MOSI_PINSRC         GPIO_PinSource12
#define TMC_SPI_AF                  GPIO_AF_SPI3

/* CS pin direct register access for speed */
#define TMC_CS_LOW(port, pin)       ((port)->BSRRH = (pin))
#define TMC_CS_HIGH(port, pin)      ((port)->BSRRL = (pin))

/* TMC2660 datagram is 20 bits = 3 bytes (4 MSB of first byte are address) */
#define TMC_DATAGRAM_BYTES          3U

/* Number of registers in the init sequence */
#define TMC_INIT_SEQ_SIZE           4U

/* Private types --------------------------------------------------------- */

/* Per-motor shadow state for CHOPCONF (needed for enable/disable) */
typedef struct {
    uint32_t chopconf;      /* Last written CHOPCONF value */
    uint32_t sgcsconf;      /* Last written SGCSCONF value (run current) */
    uint32_t sgcsconf_hold; /* SGCSCONF value for hold current */
} TMC2660_Shadow_t;

/* Private variables ----------------------------------------------------- */

/* Shadow registers for each motor */
static TMC2660_Shadow_t tmc_shadow[MCTRL_COUNT];

/* DRVCTRL values with INTPOL=1 for each microstep resolution.
 * Index matches TMC2660_Microstep_t enum. */
static const uint32_t tmc_mres_lut[TMC2660_MRES_COUNT] = {
    0x00200U,   /* 256x */
    0x00201U,   /* 128x */
    0x00202U,   /*  64x */
    0x00203U,   /*  32x */
    0x00204U,   /*  16x (default) */
    0x00205U,   /*   8x */
    0x00206U,   /*   4x */
};

/* SGCSCONF register values for each current level.
 * Base = 0xD0900 (SG filter=1, threshold=0x09).
 * Index matches TMC2660_Current_t enum.
 *
 * Current mapping (305mV sense, board-specific resistor):
 *   CS=0x00 -> 0A,    CS=0x01 -> 0.125A, CS=0x02 -> 0.25A,
 *   CS=0x03 -> 0.5A,  CS=0x04 -> 0.625A, CS=0x07 -> 1.0A,
 *   CS=0x09 -> 1.25A, CS=0x0A -> 1.375A, CS=0x0D -> 1.75A,
 *   CS=0x0F -> 2.0A,  CS=0x12 -> 2.25A,  CS=0x16 -> 2.75A
 */
static const uint32_t tmc_current_lut[TMC2660_CURR_COUNT] = {
    0xD0900U,   /* 0.0A   (CS=0x00) */
    0xD0901U,   /* 0.125A (CS=0x01) */
    0xD0902U,   /* 0.25A  (CS=0x02) */
    0xD0903U,   /* 0.5A   (CS=0x03) */
    0xD0904U,   /* 0.625A (CS=0x04) */
    0xD0907U,   /* 1.0A   (CS=0x07) */
    0xD0909U,   /* 1.25A  (CS=0x09) */
    0xD090AU,   /* 1.375A (CS=0x0A) */
    0xD090DU,   /* 1.75A  (CS=0x0D) */
    0xD090FU,   /* 2.0A   (CS=0x0F) */
    0xD0912U,   /* 2.25A  (CS=0x12) */
    0xD0916U,   /* 2.75A  (CS=0x16) */
};

/* Default init register sequence: CHOPCONF, SMARTEN, DRVCONF, DRVCTRL */
static const uint32_t tmc_init_seq[TMC_INIT_SEQ_SIZE] = {
    TMC2660_INIT_CHOPCONF,
    TMC2660_INIT_SMARTEN,
    TMC2660_INIT_DRVCONF,
    TMC2660_INIT_DRVCTRL_X16,
};

/* Private function prototypes ------------------------------------------- */
static uint8_t tmc_spi_xfer_byte(uint8_t byte);
static uint32_t tmc_spi_xfer_20bit(GPIO_TypeDef* cs_port, uint16_t cs_pin,
                                    uint32_t data);

/* Private functions ----------------------------------------------------- */

/**
 * Transfer one byte over SPI3 (blocking, full-duplex).
 * Waits for TX empty, sends byte, waits for RX, returns received byte.
 *
 * Timing: ~8 SPI clock cycles = ~1.1us at SPI3_CLK/8 (3.75 MHz).
 */
static uint8_t tmc_spi_xfer_byte(uint8_t byte)
{
    while (SPI_I2S_GetFlagStatus(TMC_SPI_PERIPH, SPI_I2S_FLAG_TXE) == RESET) {
        /* Wait for TX buffer empty */
    }
    SPI_I2S_SendData(TMC_SPI_PERIPH, byte);

    while (SPI_I2S_GetFlagStatus(TMC_SPI_PERIPH, SPI_I2S_FLAG_RXNE) == RESET) {
        /* Wait for RX buffer full */
    }
    return (uint8_t)SPI_I2S_ReceiveData(TMC_SPI_PERIPH);
}

/**
 * Perform a 20-bit SPI transaction with a specific CS pin.
 *
 * TMC2660 protocol: 20-bit datagram sent as 3 bytes MSB-first.
 *   TX: [19:16][15:8][7:0]
 *   RX: 20-bit status in 3 bytes, right-shifted by 4 to align.
 *
 * @param cs_port  CS GPIO port
 * @param cs_pin   CS GPIO pin mask
 * @param data     20-bit register data to send
 * @return         20-bit status response (>> 4)
 */
static uint32_t tmc_spi_xfer_20bit(GPIO_TypeDef* cs_port, uint16_t cs_pin,
                                    uint32_t data)
{
    uint32_t rx = 0U;

    TMC_CS_LOW(cs_port, cs_pin);

    rx |= (uint32_t)tmc_spi_xfer_byte((uint8_t)((data >> 16) & 0x0FU));
    rx <<= 8;
    rx |= (uint32_t)tmc_spi_xfer_byte((uint8_t)((data >> 8) & 0xFFU));
    rx <<= 8;
    rx |= (uint32_t)tmc_spi_xfer_byte((uint8_t)(data & 0xFFU));

    TMC_CS_HIGH(cs_port, cs_pin);

    /* TMC2660 returns status in upper 20 bits of 24-bit response */
    rx >>= 4;

    return rx;
}

/* Exported functions ---------------------------------------------------- */

/*---------------------------------------------------------------------------
 * TMC2660_SPI_Init
 *
 * Configure SPI3 peripheral and all motor CS pins.
 * SPI mode 3 (CPOL=1, CPHA=1), 8-bit, MSB first, master, ~3.75 MHz.
 * Must be called once before any TMC2660 register access.
 *---------------------------------------------------------------------------*/
void TMC2660_SPI_Init(void)
{
    SPI_InitTypeDef spi;
    GPIO_InitTypeDef gpio;
    uint8_t i;

    /* Enable SPI3 clock (APB1) */
    TMC_SPI_RCC_CMD(TMC_SPI_RCC_PERIPH, ENABLE);

    /* Enable GPIO clocks for SPI bus and all CS ports */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB |
                           RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD |
                           RCC_AHB1Periph_GPIOE, ENABLE);

    /* Configure SPI3 bus pins: PC10=SCK, PC11=MISO, PC12=MOSI */
    gpio.GPIO_Pin   = TMC_SPI_SCK_PIN | TMC_SPI_MISO_PIN | TMC_SPI_MOSI_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(TMC_SPI_GPIO_PORT, &gpio);

    GPIO_PinAFConfig(TMC_SPI_GPIO_PORT, TMC_SPI_SCK_PINSRC,  TMC_SPI_AF);
    GPIO_PinAFConfig(TMC_SPI_GPIO_PORT, TMC_SPI_MISO_PINSRC, TMC_SPI_AF);
    GPIO_PinAFConfig(TMC_SPI_GPIO_PORT, TMC_SPI_MOSI_PINSRC, TMC_SPI_AF);

    /* Configure all motor CS pins as output push-pull, initially high */
    gpio.GPIO_Mode  = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    for (i = 0; i < MCTRL_COUNT; i++) {
        const MotorHW_t* hw = &g_motor_hw[i];
        gpio.GPIO_Pin = hw->cs_pin;
        GPIO_Init(hw->cs_port, &gpio);
        TMC_CS_HIGH(hw->cs_port, hw->cs_pin);
    }

    /* Configure SPI3: mode 3 (CPOL=1, CPHA=1), master, 8-bit, MSB first */
    spi.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode              = SPI_Mode_Master;
    spi.SPI_DataSize          = SPI_DataSize_8b;
    spi.SPI_CPOL              = SPI_CPOL_High;
    spi.SPI_CPHA              = SPI_CPHA_2Edge;
    spi.SPI_NSS               = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;  /* ~3.75 MHz @ 30 MHz APB1 */
    spi.SPI_FirstBit          = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial     = 7;
    SPI_Init(TMC_SPI_PERIPH, &spi);

    SPI_Cmd(TMC_SPI_PERIPH, ENABLE);
}

/*---------------------------------------------------------------------------
 * TMC2660_Init
 *
 * Send default register init sequence to one motor's TMC2660.
 * Registers: CHOPCONF, SMARTEN, DRVCONF, DRVCTRL (16x microstep).
 *---------------------------------------------------------------------------*/
void TMC2660_Init(MotorId_t id)
{
    const MotorHW_t* hw;
    uint8_t i;

    if ((uint8_t)id >= MCTRL_COUNT) {
        return;
    }

    hw = &g_motor_hw[id];

    for (i = 0; i < TMC_INIT_SEQ_SIZE; i++) {
        tmc_spi_xfer_20bit(hw->cs_port, hw->cs_pin, tmc_init_seq[i]);
    }

    /* Initialize shadow registers */
    tmc_shadow[id].chopconf      = TMC2660_INIT_CHOPCONF;
    tmc_shadow[id].sgcsconf      = tmc_current_lut[TMC2660_CURR_1_0A];
    tmc_shadow[id].sgcsconf_hold = tmc_current_lut[TMC2660_CURR_0_5A];
}

/*---------------------------------------------------------------------------
 * TMC2660_InitAll
 *
 * Initialize all TMC2660 drivers in sequence.
 *---------------------------------------------------------------------------*/
void TMC2660_InitAll(void)
{
    uint8_t i;
    for (i = 0; i < MCTRL_COUNT; i++) {
        TMC2660_Init((MotorId_t)i);
    }
}

/*---------------------------------------------------------------------------
 * TMC2660_WriteRegister
 *
 * Write a 20-bit register value to a specific motor's TMC2660.
 * Returns the 20-bit status response.
 *---------------------------------------------------------------------------*/
uint32_t TMC2660_WriteRegister(MotorId_t id, uint32_t data)
{
    const MotorHW_t* hw;

    if ((uint8_t)id >= MCTRL_COUNT) {
        return 0xFFFFFU;  /* Error: invalid motor ID */
    }

    hw = &g_motor_hw[id];
    return tmc_spi_xfer_20bit(hw->cs_port, hw->cs_pin, data);
}

/*---------------------------------------------------------------------------
 * TMC2660_ReadStatus
 *
 * Read current status by sending a NOP command (DRVCTRL = 0).
 *---------------------------------------------------------------------------*/
uint32_t TMC2660_ReadStatus(MotorId_t id)
{
    if ((uint8_t)id >= MCTRL_COUNT) {
        return 0xFFFFFU;
    }

    return TMC2660_WriteRegister(id, TMC2660_REG_DRVCTRL);
}

/*---------------------------------------------------------------------------
 * TMC2660_SetMicrostep
 *
 * Set microstep resolution via DRVCTRL register.
 * INTPOL (step interpolation to 256) is always enabled.
 * Bounds-checked: clamps to TMC2660_MRES_4 if out of range.
 *---------------------------------------------------------------------------*/
void TMC2660_SetMicrostep(MotorId_t id, TMC2660_Microstep_t resolution)
{
    if ((uint8_t)id >= MCTRL_COUNT) {
        return;
    }

    /* Bounds check resolution */
    if ((uint8_t)resolution >= TMC2660_MRES_COUNT) {
        resolution = TMC2660_MRES_4;
    }

    TMC2660_WriteRegister(id, tmc_mres_lut[resolution]);
}

/*---------------------------------------------------------------------------
 * TMC2660_SetCurrent
 *
 * Set run and hold current via SGCSCONF register.
 * Run current is written to the TMC2660 immediately.
 * Hold current is stored in shadow for later application.
 * Bounds-checked: clamps to TMC2660_CURR_2_9A if out of range.
 *---------------------------------------------------------------------------*/
void TMC2660_SetCurrent(MotorId_t id,
                         TMC2660_Current_t run_ma,
                         TMC2660_Current_t hold_ma)
{
    if ((uint8_t)id >= MCTRL_COUNT) {
        return;
    }

    /* Bounds check current settings */
    if ((uint8_t)run_ma >= TMC2660_CURR_COUNT) {
        run_ma = TMC2660_CURR_2_9A;
    }
    if ((uint8_t)hold_ma >= TMC2660_CURR_COUNT) {
        hold_ma = TMC2660_CURR_2_9A;
    }

    /* Store both in shadow */
    tmc_shadow[id].sgcsconf      = tmc_current_lut[run_ma];
    tmc_shadow[id].sgcsconf_hold = tmc_current_lut[hold_ma];

    /* Write run current immediately */
    TMC2660_WriteRegister(id, tmc_current_lut[run_ma]);
}

/*---------------------------------------------------------------------------
 * TMC2660_SetRunCurrent
 *
 * Set run current only and write immediately.
 *---------------------------------------------------------------------------*/
void TMC2660_SetRunCurrent(MotorId_t id, TMC2660_Current_t current)
{
    if ((uint8_t)id >= MCTRL_COUNT) {
        return;
    }

    if ((uint8_t)current >= TMC2660_CURR_COUNT) {
        current = TMC2660_CURR_2_9A;
    }

    tmc_shadow[id].sgcsconf = tmc_current_lut[current];
    TMC2660_WriteRegister(id, tmc_current_lut[current]);
}

/*---------------------------------------------------------------------------
 * TMC2660_SetChopperConfig
 *
 * Set CHOPCONF register from a structured configuration.
 * Validates field ranges and builds the 20-bit register value.
 *---------------------------------------------------------------------------*/
void TMC2660_SetChopperConfig(MotorId_t id, const TMC2660_ChopperCfg_t* config)
{
    uint32_t reg;

    if ((uint8_t)id >= MCTRL_COUNT) {
        return;
    }
    if (config == (const TMC2660_ChopperCfg_t*)0) {
        return;
    }

    /* Build CHOPCONF register value with bounds-clamped fields */
    reg  = TMC2660_REG_CHOPCONF;
    reg |= ((uint32_t)(config->tbl   & 0x03U)) << TMC2660_CHOP_TBL_SHIFT;
    reg |= (config->chm   != 0U) ? TMC2660_CHOP_CHM   : 0U;
    reg |= (config->rndtf != 0U) ? TMC2660_CHOP_RNDTF : 0U;
    reg |= ((uint32_t)(config->hdec  & 0x03U)) << TMC2660_CHOP_HDEC_SHIFT;
    reg |= ((uint32_t)(config->hend  & 0x0FU)) << TMC2660_CHOP_HEND_SHIFT;
    reg |= ((uint32_t)(config->hstrt & 0x07U)) << TMC2660_CHOP_HSTRT_SHIFT;
    reg |= ((uint32_t)(config->toff  & 0x0FU)) << TMC2660_CHOP_TOFF_SHIFT;

    /* Update shadow and send */
    tmc_shadow[id].chopconf = reg;
    TMC2660_WriteRegister(id, reg);
}

/*---------------------------------------------------------------------------
 * TMC2660_Enable
 *
 * Enable driver output by restoring TOFF from shadow CHOPCONF.
 * If shadow CHOPCONF has TOFF=0 (never configured), uses the init default.
 *---------------------------------------------------------------------------*/
void TMC2660_Enable(MotorId_t id)
{
    uint32_t reg;

    if ((uint8_t)id >= MCTRL_COUNT) {
        return;
    }

    reg = tmc_shadow[id].chopconf;

    /* If TOFF is already 0, restore from init default */
    if ((reg & TMC2660_CHOP_TOFF_MASK) == 0U) {
        reg = TMC2660_INIT_CHOPCONF;
        tmc_shadow[id].chopconf = reg;
    }

    TMC2660_WriteRegister(id, reg);
}

/*---------------------------------------------------------------------------
 * TMC2660_Disable
 *
 * Disable driver output by setting TOFF=0 in CHOPCONF.
 * The shadow retains the original TOFF value for re-enable.
 *---------------------------------------------------------------------------*/
void TMC2660_Disable(MotorId_t id)
{
    uint32_t reg;

    if ((uint8_t)id >= MCTRL_COUNT) {
        return;
    }

    /* Clear TOFF field to disable driver, but do NOT update shadow */
    reg = tmc_shadow[id].chopconf & ~TMC2660_CHOP_TOFF_MASK;
    TMC2660_WriteRegister(id, reg);
}
