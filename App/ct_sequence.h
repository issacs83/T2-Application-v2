/*
 * ct_sequence.h : CT capture sequence (App layer)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Orchestrates CT/CBCT capture workflow. Delegates to existing
 * ct_capture.c (src/) for the actual capture logic.
 */

#ifndef CT_SEQUENCE_H
#define CT_SEQUENCE_H

/**
 * Execute CT capture sequence.
 * This is the top-level entry point called from the main super-loop
 * when CurCaptureMode == CAPTURE_CT.
 *
 * Internally calls CtCapture() from ct_capture.c.
 */
void CtSequence_Run(void);

#endif /* CT_SEQUENCE_H */
