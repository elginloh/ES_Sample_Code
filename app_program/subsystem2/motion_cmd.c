/*
 *	motion_cmd.c
 *	Validation and status packing for the Motion Control interface
 *	(pure logic, host-testable).
 *
 *	Packed motion status (two 32-bit words, written only by the Motion
 *	Control task, read under a sequence counter):
 *	  word 0  bits  0..10  speed left   (11, two's compl., mm/s; -1024 invalid)
 *	          bits 11..21  speed right  (11)
 *	          bits 22..23  motion state (2)  MS_*
 *	          bits 24..25  source       (2)  MSRC_*
 *	          bit  26      command active
 *	          bit  27      stall left
 *	          bit  28      stall right
 *	          bit  29      encoders OK
 *	  word 1  bits  0..15  odometer     (16, cm, saturating)
 *	          bits 16..23  heading / 2  (8, 0..179 -> 0..358 deg)
 *	          bits 24..29  cruise / 10  (6, 0..63 -> 0..630 mm/s)
 */

#include "motion_msg.h"

#define SPD_BITS_MASK	0x7FFUL
#define SPD_MAX		1023

LOCAL UW pack_speed(H v)
{
	INT	s = v;

	if (s != MSTAT_SPEED_INVALID) {
		if (s > SPD_MAX) {
			s = SPD_MAX;
		} else if (s < -SPD_MAX) {
			s = -SPD_MAX;
		}
	}
	return (UW)s & SPD_BITS_MASK;
}

LOCAL H unpack_speed(UW bits)
{
	INT	v = (INT)(bits & SPD_BITS_MASK);

	return (H)((v & 0x400) ? (v - 0x800) : v);	/* sign-extend 11 bits */
}

EXPORT void mstat_pack(const MOTION_STATUS *s, UW out[2])
{
	UW	heading2 = (UW)(s->heading_deg % 360U) / 2U;
	UW	cruise10 = (UW)s->cruise_mms / 10U;

	if (cruise10 > 63U) {
		cruise10 = 63U;
	}
	out[0] = pack_speed(s->speed_l_mms)
	       | (pack_speed(s->speed_r_mms) << 11)
	       | (((UW)s->mstate & 0x3UL) << 22)
	       | (((UW)s->source & 0x3UL) << 24)
	       | ((s->cmd_active ? 1UL : 0UL) << 26)
	       | ((s->stall_l ? 1UL : 0UL) << 27)
	       | ((s->stall_r ? 1UL : 0UL) << 28)
	       | ((s->enc_ok ? 1UL : 0UL) << 29);
	out[1] = ((UW)s->odometer_cm & 0xFFFFUL)
	       | ((heading2 & 0xFFUL) << 16)
	       | ((cruise10 & 0x3FUL) << 24);
}

EXPORT void mstat_unpack(const UW in[2], MOTION_STATUS *s)
{
	s->speed_l_mms = unpack_speed(in[0]);
	s->speed_r_mms = unpack_speed(in[0] >> 11);
	s->mstate      = (UB)((in[0] >> 22) & 0x3UL);
	s->source      = (UB)((in[0] >> 24) & 0x3UL);
	s->cmd_active  = (UB)((in[0] >> 26) & 0x1UL);
	s->stall_l     = (UB)((in[0] >> 27) & 0x1UL);
	s->stall_r     = (UB)((in[0] >> 28) & 0x1UL);
	s->enc_ok      = (UB)((in[0] >> 29) & 0x1UL);
	s->odometer_cm = (UH)(in[1] & 0xFFFFUL);
	s->heading_deg = (UH)(((in[1] >> 16) & 0xFFUL) * 2U);
	s->cruise_mms  = (UH)(((in[1] >> 24) & 0x3FUL) * 10U);
}

/* Which command types each source may send (bit n = MCMD type n).
   Week 6 / Tribunal 1: remote commands are high-level supervisory
   requests, never direct motor moves; line following steers through the
   steering word, not the queue. */
LOCAL const UB allowed_types[MSRC_COUNT] = {
	0U,							/* LINE    */
	(1U << MCMD_MOVE) | (1U << MCMD_TURN) | (1U << MCMD_STOP),	/* BARCODE */
	(1U << MCMD_MOVE) | (1U << MCMD_TURN) | (1U << MCMD_STOP),	/* AVOID   */
	(1U << MCMD_STOP) | (1U << MCMD_SET_SPEED)
		| (1U << MCMD_RESUME) | (1U << MCMD_CLEAR)		/* SUPER   */
};

EXPORT UW mcmd_validate(UW w)
{
	UW	type = mcmd_type(w);
	UW	src = mcmd_src(w);
	INT	arg = mcmd_arg(w);
	INT	mag = (arg < 0) ? -arg : arg;

	if ((w & MCMD_RSV_MASK) != 0UL || type >= MCMD_COUNT) {
		return MREJ_FORMAT;
	}
	if ((allowed_types[src] & (1U << type)) == 0U) {
		return MREJ_SOURCE;
	}
	switch (type) {
	case MCMD_MOVE:
		if (mag < 1 || mag > MCMD_MOVE_MAX_MM) {
			return MREJ_RANGE;
		}
		break;
	case MCMD_TURN:
		if (mag < 1 || mag > MCMD_TURN_MAX_DEG) {
			return MREJ_RANGE;
		}
		break;
	case MCMD_SET_SPEED:
		if (arg < MOTION_SPEED_MIN_MMS || arg > MOTION_SPEED_MAX_MMS
		    || mcmd_speed(w) != 0U) {
			return MREJ_RANGE;
		}
		break;
	default:		/* STOP, RESUME, CLEAR carry no argument */
		if (arg != 0 || mcmd_speed(w) != 0U) {
			return MREJ_RANGE;
		}
		break;
	}
	return MREJ_NONE;
}
