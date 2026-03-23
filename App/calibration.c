/*
 * calibration.c : Calibration mode orchestration
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Memory: ~50 bytes Flash, 0 bytes SRAM.
 */

#include "calibration.h"

/* CalibrationMode() and GeometryAlignMode() are in src/calibration.c */
extern void CalibrationMode(void);
extern void GeometryAlignMode(void);

void AppCalibration_Run(void)
{
    CalibrationMode();
}

void AppGeoAlign_Run(void)
{
    GeometryAlignMode();
}
