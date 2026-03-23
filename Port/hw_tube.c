/*
 * hw_tube.c : X-ray tube control (delegates to tube.c)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Memory: ~200 bytes Flash, 0 bytes SRAM.
 */

#include "hw_tube.h"
#include "tube.h"
#include "configuration.h"
#include "stm32f2xx.h"
#include "stm32f2xx_gpio.h"

/* Tube GPIO pins (from safety.h / tube.h) */
#define TUBE_READY_PORT         GPIOC
#define TUBE_READY_PIN          GPIO_Pin_4
#define TUBE_EXPOSURE_PORT      GPIOC
#define TUBE_EXPOSURE_PIN       GPIO_Pin_5

void HwTube_Init(void)
{
    Tube_Config();
}

void HwTube_SetReady(bool enable)
{
    Tube_CtrlReady(enable ? TUBE_READY_ENABLE : TUBE_READY_DISABLE);
}

void HwTube_SetExposure(bool enable)
{
    Tube_CtrlExposure(enable ? TUBE_EXPOSURE_ENABLE : TUBE_EXPOSURE_DISABLE);
}

void HwTube_StartPulse(void)
{
#ifndef USE_TUBE_PPS_TYPE_IO
    Tube_Start();
#endif
}

void HwTube_StopPulse(void)
{
#ifndef USE_TUBE_PPS_TYPE_IO
    Tube_Stop();
#endif
}

void HwTube_SetPPS(bool enable)
{
#ifdef USE_TUBE_PPS_TYPE_IO
    Tube_CtrlPps(enable ? Bit_SET : Bit_RESET);
#else
    (void)enable;
#endif
}

bool HwTube_IsReadyOff(void)
{
    return (GPIO_ReadInputDataBit(TUBE_READY_PORT, TUBE_READY_PIN) != Bit_RESET);
}

bool HwTube_IsExposureOff(void)
{
    return (GPIO_ReadInputDataBit(TUBE_EXPOSURE_PORT, TUBE_EXPOSURE_PIN) != Bit_RESET);
}
