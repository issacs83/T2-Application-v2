/*
 * ct_sequence.c : CT capture sequence orchestration
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Memory: ~50 bytes Flash, 0 bytes SRAM.
 */

#include "ct_sequence.h"
#include "extern.h"

/* CtCapture() is defined in src/ct_capture.c */
extern void CtCapture(void);

void CtSequence_Run(void)
{
    CtCapture();
}
