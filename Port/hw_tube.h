/*
 * hw_tube.h : X-ray tube control GPIO (Port layer)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Wraps tube.c GPIO control for the Port layer.
 */

#ifndef HW_TUBE_H
#define HW_TUBE_H

#include <stdbool.h>
#include <stdint.h>

/* Initialize tube hardware */
void HwTube_Init(void);

/* Ready signal control (active low) */
void HwTube_SetReady(bool enable);

/* Exposure signal control (active low) */
void HwTube_SetExposure(bool enable);

/* Pulse timing start/stop */
void HwTube_StartPulse(void);
void HwTube_StopPulse(void);

/* PPS (Pulse Per Second) control for IO-based tube */
void HwTube_SetPPS(bool enable);

/* Read-back: verify tube outputs are in expected state */
bool HwTube_IsReadyOff(void);
bool HwTube_IsExposureOff(void);

#endif /* HW_TUBE_H */
