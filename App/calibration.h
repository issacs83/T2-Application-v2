/*
 * calibration.h : Calibration mode (App layer)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Orchestrates calibration and geometry alignment modes.
 */

#ifndef APP_CALIBRATION_H
#define APP_CALIBRATION_H

/**
 * Execute calibration mode.
 * Called from main super-loop when CurCaptureMode == CALIBRATION_MODE.
 */
void AppCalibration_Run(void);

/**
 * Execute geometry alignment mode.
 * Called from main super-loop when CurCaptureMode == GEOMETRY_ALIGN_MODE.
 */
void AppGeoAlign_Run(void);

#endif /* APP_CALIBRATION_H */
