/*
 * hw_gpio.c : GPIO abstraction (delegates to misc1.c functions)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Memory: ~400 bytes Flash, 0 bytes SRAM.
 */

#include "hw_gpio.h"
#include "misc1.h"
#include "configuration.h"
#include "stm32f2xx.h"

void HwGPIO_Init(void)
{
    Exposure_Config();
    LaserConfig();
    Column_Config();
    Lamp_Config();
    Indicator_Config();
    Led_dip_Config();
#ifdef USE_CT_STITCH_MODE
    StichSensor_Config();
#endif
}

void HwGPIO_ExposureLedCtrl(bool enable)
{
    Exposure_CtrlLed(enable ? EXPOSURE_LED_GPIO_ENABLE : EXPOSURE_LED_GPIO_DISABLE);
}

bool HwGPIO_ExposureSwitchCheck(void)
{
    return Exposure_CheckSwitch();
}

bool HwGPIO_ExposureSwitchReleased(uint32_t millis)
{
    return bIsExposureSwitchReleased(millis);
}

void HwGPIO_LaserControl(HwBeamType_t type, bool enable)
{
    LaserControl((BeamType_t)type, enable);
}

void HwGPIO_LaserSetPosition(void)
{
    LaserSetPosition();
}

void HwGPIO_ColumnControl(HwColumnCtrl_t ctrl)
{
    Column_Control((uint8_t)ctrl);
}

void HwGPIO_ColumnEncControl(bool enable)
{
#ifdef USE_CT_STITCH_MODE
    Column_EncControl(enable);
#else
    (void)enable;
#endif
}

void HwGPIO_LampControl(uint8_t lamp_id, bool enable)
{
    Lamp_Control(lamp_id, enable ? LAMP_ENABLE : LAMP_DISABLE);
}

void HwGPIO_IndicatorColor(HwLedColor_t color)
{
    Lamp_Color_Control((Indicator_Typedef)color);
}

void HwGPIO_IndicatorControl(bool enable)
{
    Indicator_Control(enable ? INDICATOR_ENABLE : INDICATOR_DISABLE);
}

void HwGPIO_DipLedControl(bool enable)
{
    Led_dip_Control(enable ? Bit_RESET : Bit_SET);
}
