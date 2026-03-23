/*
 * scan_sequence.h : Ceph scan sequence (App layer)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * Orchestrates cephalometric scan capture workflow.
 */

#ifndef SCAN_SEQUENCE_H
#define SCAN_SEQUENCE_H

/**
 * Execute cephalometric scan sequence.
 * Called from main super-loop when CurCaptureMode == CAPTURE_SCAN.
 */
void ScanSequence_Run(void);

#endif /* SCAN_SEQUENCE_H */
