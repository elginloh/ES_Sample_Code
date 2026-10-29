/*
 *	drv_ir.h
 *	Three digital IR sensors (Buddy 3 owns them):
 *	  IR_LEFT   GP2   (D0; A0 on GP3 is wired but unused)
 *	  IR_CENTER GP6   (D0)
 *	  IR_RIGHT  GP27  (D0) -- also the barcode edge source
 *
 *	Barcode edge capture (Tribunal 1, Version B): the shared GPIO
 *	interrupt timestamps every IR_RIGHT edge into a single-producer /
 *	single-consumer ring.  The ISR only records; the Barcode Decoding task
 *	consumes.  One 32-bit record per edge (minimum-bit representation):
 *	    bits 31..1  time in microseconds (bit 0 dropped: 2 us resolution)
 *	    bit  0      raw level AFTER the edge
 *	Overflow is flagged, never silent.
 *
 *	Backup (Tribunal 1: "retain 1 ms periodic sensor polling using
 *	timer"): build with -DS3_CAPTURE_POLL_1MS=1 to sample IR_RIGHT every
 *	1 ms from RP2040 TIMER alarm 3 instead of the GPIO interrupt.  The
 *	kernel tick (10 ms) is too coarse for this, hence the hardware alarm.
 */

#ifndef DRV_IR_H
#define DRV_IR_H

#include <tk/tkernel.h>

#define IR_LEFT_PIN		2
#define IR_CENTER_PIN		6
#define IR_RIGHT_PIN		27

/* 1 = this car has the IR_RIGHT (GP27, barcode) sensor on its LEFT side,
   on the outrigger bracket, and IR_LEFT on its right.  The line follower
   keeps its own labels (the barcode handling stays on GP27); task_line.c
   mirrors its steering and the IMU heading it is given, and the "ir:"
   console line prints the car's real left and right.  Seen on the car:
   the left sensor on the line turned it clockwise. */
#ifndef IR_SIDES_MIRRORED
#define IR_SIDES_MIRRORED	1
#endif

#ifndef S3_CAPTURE_POLL_1MS
#define S3_CAPTURE_POLL_1MS	0
#endif

#define IR_EDGE_RING		64	/* power of two */
#define IR_EDGE_LEVEL(rec)	((UB)((rec) & 1U))
#define IR_EDGE_TIME(rec)	((rec) & ~1UL)

IMPORT ER   ir_init(void);		/* pins + capture source (init phase) */
IMPORT void ir_set_wake_task(ID tid);	/* task woken on every edge          */
IMPORT ER   ir_capture_start(void);	/* 1 ms poll mode: arm the alarm      */
IMPORT UB   ir_read_raw(void);		/* bit0 L, bit1 C, bit2 R (raw levels) */
IMPORT BOOL ir_edge_get(UW *rec);	/* consumer side; FALSE when empty     */
IMPORT BOOL ir_edge_overflow_take(void);	/* TRUE once after an overflow */
IMPORT void ir_edge_flush(void);
IMPORT UW   ir_edge_count(void);	/* edges recorded (diagnostics)        */

#endif /* DRV_IR_H */
