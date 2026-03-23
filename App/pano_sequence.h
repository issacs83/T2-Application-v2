/*
 * pano_sequence.h : Pano capture sequence (App layer)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Orchestrates panoramic capture workflow.
 */

#ifndef PANO_SEQUENCE_H
#define PANO_SEQUENCE_H

/**
 * Execute panoramic capture sequence.
 * Called from main super-loop when CurCaptureMode == CAPTURE_PANO.
 */
void PanoSequence_Run(void);

#endif /* PANO_SEQUENCE_H */
