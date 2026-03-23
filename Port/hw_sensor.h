/*
 * hw_sensor.h : Pano/CT/Scan sensor hardware abstraction
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Wraps sensor.c functionality for the Port layer.
 * All sensor hardware configuration and control goes through here.
 */

#ifndef HW_SENSOR_H
#define HW_SENSOR_H

#include "stm32f2xx.h"
#include <stdbool.h>

/* Initialize all sensor hardware */
void HwSensor_Init(void);

/* Pano sensor control */
void HwSensor_PanoConfig(void);
void HwSensor_PanoStart(void);
void HwSensor_PanoStop(void);
bool HwSensor_PanoCheckInput(void);
void HwSensor_PanoPowerControl(bool enable);
void HwSensor_PanoSetUserSync(void);

/* Scan (Ceph) sensor control */
void HwSensor_ScanConfig(void);
void HwSensor_ScanCtrlOutput(bool enable);
void HwSensor_ScanPowerControl(bool enable);

/* CT sensor control (pulsed mode) */
#if defined(USE_CT_XRAY_PULSED_MODE)
void HwSensor_CtConfig(void);
void HwSensor_CtStart(void);
void HwSensor_CtStop(void);
#endif

#endif /* HW_SENSOR_H */
