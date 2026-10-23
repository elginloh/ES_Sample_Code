/*
 *	s3_status.c
 *	Pack/unpack of subsystem 3 status words (see s3_status.h).
 */

#include "s3_status.h"
#include "barcode_dec.h"

#define F(v, mask, sh)	(((UW)(v) & (mask)) << (sh))
#define G(w, mask, sh)	((UB)(((w) >> (sh)) & (mask)))

EXPORT UW s3_line_pack(const S3_LINE_STATUS *s)
{
	INT	st = s->steer;

	if (st > 63) {
		st = 63;
	} else if (st < -63) {
		st = -63;
	}
	return F(s->ir_black, 0x7UL, 0) | F(s->pos, 0x7UL, 3)
	     | F(s->state, 0x3UL, 6) | F(s->seq, 0xFUL, 8)
	     | F((UW)st, 0x7FUL, 12) | F(s->calibrated, 0x1UL, 19)
	     | F(s->junctions, 0xFUL, 20) | F(s->lost_count, 0xFUL, 24)
	     | F(s->speed_frac, 0xFUL, 28);
}

EXPORT void s3_line_unpack(UW w, S3_LINE_STATUS *s)
{
	INT	st = (INT)((w >> 12) & 0x7FUL);

	s->ir_black = G(w, 0x7UL, 0);
	s->pos = G(w, 0x7UL, 3);
	s->state = G(w, 0x3UL, 6);
	s->seq = G(w, 0xFUL, 8);
	s->steer = (B)((st & 0x40) ? (st - 0x80) : st);
	s->calibrated = G(w, 0x1UL, 19);
	s->junctions = G(w, 0xFUL, 20);
	s->lost_count = G(w, 0xFUL, 24);
	s->speed_frac = G(w, 0xFUL, 28);
}

EXPORT UW s3_bc_pack(const S3_BC_STATUS *s)
{
	return F(s->state, 0x3UL, 0) | F(s->symbol, 0x3FUL, 2)
	     | F(s->result, 0x3UL, 8) | F(s->nav, 0x3UL, 10)
	     | F(s->nav_valid, 0x1UL, 12) | F(s->reversed, 0x1UL, 13)
	     | F(s->n_valid, 0x3FUL, 14) | F(s->n_invalid, 0x3FUL, 20)
	     | F(s->mres, 0x3UL, 26) | F(s->executed, 0x1UL, 28)
	     | F(s->err_class, 0x7UL, 29);
}

EXPORT void s3_bc_unpack(UW w, S3_BC_STATUS *s)
{
	s->state = G(w, 0x3UL, 0);
	s->symbol = G(w, 0x3FUL, 2);
	s->result = G(w, 0x3UL, 8);
	s->nav = G(w, 0x3UL, 10);
	s->nav_valid = G(w, 0x1UL, 12);
	s->reversed = G(w, 0x1UL, 13);
	s->n_valid = G(w, 0x3FUL, 14);
	s->n_invalid = G(w, 0x3FUL, 20);
	s->mres = G(w, 0x3UL, 26);
	s->executed = G(w, 0x1UL, 28);
	s->err_class = G(w, 0x7UL, 29);
}

EXPORT UB s3_err_class(INT err)
{
	switch (err) {
	case 0:			return S3_BCE_NONE;
	case C39_ERR_COUNT:	return S3_BCE_COUNT;
	case C39_ERR_WIDTH:	return S3_BCE_WIDTH;
	case C39_ERR_PATTERN:	return S3_BCE_PATTERN;
	case C39_ERR_STARTSTOP:
	case C39_ERR_GAP:	return S3_BCE_FRAME;
	case BC_ERR_LONG:
	case BC_ERR_OVERFLOW:
	case C39_ERR_SPACE:	return S3_BCE_LONG;
	case BC_ERR_WINDOW:
	case BC_ERR_STALLED:	return S3_BCE_WINDOW;
	default:		return S3_BCE_ABORT;
	}
}
