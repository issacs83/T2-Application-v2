/*
 * hw_gpio.h : All GPIO pin definitions + init (Port layer)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Consolidates GPIO definitions from define.h and misc1.c.
 * Provides clean interface for exposure switch, laser, column,
 * lamp, and indicator control.
 */

#ifndef HW_GPIO_H
#define HW_GPIO_H

#include <stdbool.h>
#include <stdint.h>

/* Beam type identifiers */
typedef enum {
    HW_BEAM_HEAD_CT = 0,
    HW_BEAM_HEAD_PANO,
    HW_BEAM_HEAD_CEPH,
    HW_BEAM_FOOT,
} HwBeamType_t;

/* Column control */
typedef enum {
    HW_COLUMN_STOP = 0,
    HW_COLUMN_UP,
    HW_COLUMN_DOWN,
} HwColumnCtrl_t;

/* Indicator LED colors */
typedef enum {
    HW_LED_BLACK = 0,
    HW_LED_WHITE,
    HW_LED_GREEN,
    HW_LED_YELLOW,
    HW_LED_RED,
    HW_LED_PURPLE,
    HW_LED_BLUE,
    HW_LED_CYAN,
} HwLedColor_t;

/* Initialize all GPIO (exposure, laser, column, lamp, indicator) */
void HwGPIO_Init(void);

/* Exposure switch */
void HwGPIO_ExposureLedCtrl(bool enable);
bool HwGPIO_ExposureSwitchCheck(void);
bool HwGPIO_ExposureSwitchReleased(uint32_t millis);

/* Laser control */
void HwGPIO_LaserControl(HwBeamType_t type, bool enable);
void HwGPIO_LaserSetPosition(void);

/* Column control */
void HwGPIO_ColumnControl(HwColumnCtrl_t ctrl);
void HwGPIO_ColumnEncControl(bool enable);

/* Lamp / LED control */
void HwGPIO_LampControl(uint8_t lamp_id, bool enable);
void HwGPIO_IndicatorColor(HwLedColor_t color);
void HwGPIO_IndicatorControl(bool enable);

/* DIP LED */
void HwGPIO_DipLedControl(bool enable);

#endif /* HW_GPIO_H */
