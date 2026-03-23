/*
 * pano_sequence.c : Pano capture sequence orchestration
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Memory: ~50 bytes Flash, 0 bytes SRAM.
 */

#include "pano_sequence.h"
#include "extern.h"

/* PanoCapture() is defined in src/pano_capture.c */
extern void PanoCapture(void);

void PanoSequence_Run(void)
{
    PanoCapture();
}
