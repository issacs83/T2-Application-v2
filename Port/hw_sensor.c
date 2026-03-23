/*
 * hw_sensor.c : Sensor hardware abstraction (delegates to sensor.c)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Memory: ~200 bytes Flash, 0 bytes SRAM.
 */

#include "hw_sensor.h"
#include "sensor.h"
#include "configuration.h"

void HwSensor_Init(void)
{
    PanoSensor_Config();
    PanoSensor_PowerConfig();
    ScanSensor_Config();
    ScanSensor_PowerConfig();
#if defined(USE_CT_XRAY_PULSED_MODE)
    CtSensor_Config();
#endif
}

void HwSensor_PanoConfig(void)
{
    PanoSensor_Config();
}

void HwSensor_PanoStart(void)
{
    PanoSensor_Start();
}

void HwSensor_PanoStop(void)
{
    PanoSensor_Stop();
}

bool HwSensor_PanoCheckInput(void)
{
    return (PanoSensor_CheckInput() == Bit_SET);
}

void HwSensor_PanoPowerControl(bool enable)
{
    PanoSensor_PowerControl(enable ? 1 : 0);
}

void HwSensor_PanoSetUserSync(void)
{
    PanoSensor_SetUserSync();
}

void HwSensor_ScanConfig(void)
{
    ScanSensor_Config();
}

void HwSensor_ScanCtrlOutput(bool enable)
{
    ScanSensor_CtrlOutput(enable ? Bit_SET : Bit_RESET);
}

void HwSensor_ScanPowerControl(bool enable)
{
    ScanSensor_PowerControl(enable ? 1 : 0);
}

#if defined(USE_CT_XRAY_PULSED_MODE)
void HwSensor_CtConfig(void)
{
    CtSensor_Config();
}

void HwSensor_CtStart(void)
{
    CtSensor_Start();
}

void HwSensor_CtStop(void)
{
    CtSensor_Stop();
}
#endif
