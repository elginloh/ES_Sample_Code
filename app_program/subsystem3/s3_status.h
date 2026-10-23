/*
 *	s3_status.h
 *	Packed status words published by subsystem 3 (pure definitions and
 *	pack/unpack helpers, host-testable).  Masks and shifts only.
 *
 *	Line status word (writer: Line Following task)
 *	    bits  0..2   IR black bits     (3)  bit0 L, bit1 C, bit2 R
 *	    bits  3..5   line position     (3)  LP_*
 *	    bits  6..7   line state        (2)  LS_*
 *	    bits  8..11  sequence          (4)  +1 every period (freshness)
 *	    bits 12..18  steer             (7)  two's complement -63..63
 *	    bit  19      calibrated        (1)
 *	    bits 20..23  junction count    (4)  modulo 16
 *	    bits 24..27  line-lost count   (4)  modulo 16
 *	    bits 28..31  speed fraction    (4)  0..15
 *
 *	Barcode status word (writer: Barcode Decoding task)
 *	    bits  0..1   state             (2)  BCS_*
 *	    bits  2..7   last symbol       (6)  Code 39 index, 63 = none
 *	    bits  8..9   last result       (2)  BCR_*
 *	    bits 10..11  last nav command  (2)  NAV_*
 *	    bit  12      nav command valid (1)
 *	    bit  13      read reversed     (1)
 *	    bits 14..19  valid reads       (6)  modulo 64
 *	    bits 20..25  invalid reads     (6)  modulo 64
 *	    bits 26..27  motion result     (2)  MRES_* of the last nav command
 *	    bit  28      nav executed      (1)
 *	    bits 29..31  last error class  (3)  S3_BCE_*
 */

#ifndef S3_STATUS_H
#define S3_STATUS_H

#include <tk/typedef.h>

typedef struct {
	UB	ir_black;
	UB	pos;
	UB	state;
	UB	seq;
	B	steer;
	UB	calibrated;
	UB	junctions;
	UB	lost_count;
	UB	speed_frac;
	UW	time_ms;	/* publication time (separate word) */
} S3_LINE_STATUS;

#define S3_BCE_NONE	0U
#define S3_BCE_COUNT	1U
#define S3_BCE_WIDTH	2U
#define S3_BCE_PATTERN	3U
#define S3_BCE_FRAME	4U	/* start/stop or gap      */
#define S3_BCE_LONG	5U	/* element too long / overflow */
#define S3_BCE_WINDOW	6U	/* window / stalled       */
#define S3_BCE_ABORT	7U

typedef struct {
	UB	state;
	UB	symbol;
	UB	result;
	UB	nav;
	UB	nav_valid;
	UB	reversed;
	UB	n_valid;
	UB	n_invalid;
	UB	mres;
	UB	executed;
	UB	err_class;
} S3_BC_STATUS;

IMPORT UW   s3_line_pack(const S3_LINE_STATUS *s);
IMPORT void s3_line_unpack(UW w, S3_LINE_STATUS *s);
IMPORT UW   s3_bc_pack(const S3_BC_STATUS *s);
IMPORT void s3_bc_unpack(UW w, S3_BC_STATUS *s);
IMPORT UB   s3_err_class(INT err);

#endif /* S3_STATUS_H */
