/*
 *	motion_msg.h
 *	Packed message formats of the Motion Control interface (Buddy 2).
 *	Pure definitions + inline pack/unpack (masks and shifts, never C
 *	bit-field layout, so the format is identical on target and host).
 *
 *	Producers: Buddy 3 (barcode navigation, line steering), Buddy 5
 *	(avoidance, speed cap), Buddy 1 (validated supervisory requests).
 *	Consumer: Motion Control task only (sole owner of the motors).
 *
 *	Command priority (Week 6 section 8.3, Tribunal 1 interface map):
 *	    Safe Stop/Fault > Obstacle Avoidance > Barcode Navigation
 *	    > Normal Line Following.
 *	It is fixed by the SOURCE field; a supervisory STOP is a bypass that
 *	is always honoured.
 */

#ifndef MOTION_MSG_H
#define MOTION_MSG_H

#include <tk/typedef.h>

/* ---------------------------------------------------------------------
 *  Sources (2 bits).  Numeric order == priority order for queue sources.
 * --------------------------------------------------------------------- */
#define MSRC_LINE	0U	/* line steering (uses the steering word)  */
#define MSRC_BARCODE	1U	/* barcode navigation commands             */
#define MSRC_AVOID	2U	/* obstacle avoidance / recovery           */
#define MSRC_SUPER	3U	/* validated supervisory (MQTT) requests   */
#define MSRC_COUNT	4U

/* ---------------------------------------------------------------------
 *  Command types (3 bits).  6 and 7 are invalid.
 * --------------------------------------------------------------------- */
#define MCMD_MOVE	0U	/* arg: signed distance mm, 1..MOVE_MAX_MM  */
#define MCMD_TURN	1U	/* arg: signed angle deg, + = clockwise (right) */
#define MCMD_STOP	2U	/* controlled stop, arg 0                    */
#define MCMD_SET_SPEED	3U	/* supervisory: cruise speed mm/s            */
#define MCMD_RESUME	4U	/* supervisory: leave Safe Stop -> INIT      */
#define MCMD_CLEAR	5U	/* supervisory: clear latched fault events   */
#define MCMD_COUNT	6U

/* Argument limits */
#define MCMD_MOVE_MAX_MM	2000	/* longest single straight move     */
#define MCMD_TURN_MAX_DEG	360	/* one full turn                    */
#define MOTION_SPEED_MIN_MMS	50	/* SET_SPEED lower bound            */
#define MOTION_SPEED_MAX_MMS	400	/* SET_SPEED upper bound            */
#define MOTION_SPEED_STEP_MMS	25	/* speed level n == n * 25 mm/s     */

/* ---------------------------------------------------------------------
 *  Command word (32 bits, 28 used)
 *    bits  0..2   type      (3)  MCMD_*
 *    bits  3..4   source    (2)  MSRC_*
 *    bits  5..8   seq       (4)  per-source sequence, echoed in result
 *    bits  9..11  epoch     (3)  vehicle-mode epoch when built (VSW)
 *    bits 12..15  speed     (4)  0 = default, n = n*25 mm/s
 *    bits 16..27  arg       (12) two's complement, -2048..2047
 *    bits 28..31  reserved  (4)  must be 0
 * --------------------------------------------------------------------- */
#define MCMD_TYPE_SHIFT		0U
#define MCMD_TYPE_MASK		0x7UL
#define MCMD_SRC_SHIFT		3U
#define MCMD_SRC_MASK		0x3UL
#define MCMD_SEQ_SHIFT		5U
#define MCMD_SEQ_MASK		0xFUL
#define MCMD_EPOCH_SHIFT	9U
#define MCMD_EPOCH_MASK		0x7UL
#define MCMD_SPD_SHIFT		12U
#define MCMD_SPD_MASK		0xFUL
#define MCMD_ARG_SHIFT		16U
#define MCMD_ARG_MASK		0xFFFUL
#define MCMD_RSV_MASK		0xF0000000UL

static inline UW mcmd_pack(UW type, UW src, UW seq, UW epoch, UW spd, INT arg)
{
	return ((type & MCMD_TYPE_MASK) << MCMD_TYPE_SHIFT)
	     | ((src & MCMD_SRC_MASK) << MCMD_SRC_SHIFT)
	     | ((seq & MCMD_SEQ_MASK) << MCMD_SEQ_SHIFT)
	     | ((epoch & MCMD_EPOCH_MASK) << MCMD_EPOCH_SHIFT)
	     | ((spd & MCMD_SPD_MASK) << MCMD_SPD_SHIFT)
	     | (((UW)arg & MCMD_ARG_MASK) << MCMD_ARG_SHIFT);
}

static inline UW mcmd_type(UW w)	{ return (w >> MCMD_TYPE_SHIFT) & MCMD_TYPE_MASK; }
static inline UW mcmd_src(UW w)		{ return (w >> MCMD_SRC_SHIFT) & MCMD_SRC_MASK; }
static inline UW mcmd_seq(UW w)		{ return (w >> MCMD_SEQ_SHIFT) & MCMD_SEQ_MASK; }
static inline UW mcmd_epoch(UW w)	{ return (w >> MCMD_EPOCH_SHIFT) & MCMD_EPOCH_MASK; }
static inline UW mcmd_speed(UW w)	{ return (w >> MCMD_SPD_SHIFT) & MCMD_SPD_MASK; }

static inline INT mcmd_arg(UW w)
{
	INT	v = (INT)((w >> MCMD_ARG_SHIFT) & MCMD_ARG_MASK);

	return (v & 0x800) ? (v - 0x1000) : v;	/* sign-extend 12 bits */
}

/* ---------------------------------------------------------------------
 *  Rejection reasons (3 bits), carried in the result word
 * --------------------------------------------------------------------- */
#define MREJ_NONE	0U
#define MREJ_FORMAT	1U	/* reserved bits / unknown type       */
#define MREJ_RANGE	2U	/* argument out of range              */
#define MREJ_SOURCE	3U	/* type not allowed for this source   */
#define MREJ_MODE	4U	/* not allowed in current mode        */
#define MREJ_STALE	5U	/* built under an older mode epoch    */
#define MREJ_PRIORITY	6U	/* higher-priority activity running   */
#define MREJ_QUEUE	7U	/* per-source queue full              */

/* ---------------------------------------------------------------------
 *  Result word (13 bits): one per source, written by Motion Control,
 *  signalled by flg_motion_done bit (1 << source).
 *    bits 0..3  seq of the command it answers
 *    bits 4..5  status  MRES_*
 *    bits 6..8  command type
 *    bits 9..11 reject reason MREJ_*
 *    bit  12    valid
 * --------------------------------------------------------------------- */
#define MRES_DONE	0U
#define MRES_ABORTED	1U
#define MRES_FAULT	2U
#define MRES_REJECTED	3U

#define MRES_SEQ_SHIFT		0U
#define MRES_SEQ_MASK		0xFUL
#define MRES_ST_SHIFT		4U
#define MRES_ST_MASK		0x3UL
#define MRES_TYPE_SHIFT		6U
#define MRES_TYPE_MASK		0x7UL
#define MRES_REJ_SHIFT		9U
#define MRES_REJ_MASK		0x7UL
#define MRES_VALID		(1UL << 12)

static inline UW mres_pack(UW cmd, UW status, UW reason)
{
	return ((mcmd_seq(cmd) & MRES_SEQ_MASK) << MRES_SEQ_SHIFT)
	     | ((status & MRES_ST_MASK) << MRES_ST_SHIFT)
	     | ((mcmd_type(cmd) & MRES_TYPE_MASK) << MRES_TYPE_SHIFT)
	     | ((reason & MRES_REJ_MASK) << MRES_REJ_SHIFT)
	     | MRES_VALID;
}

static inline UW mres_seq(UW r)		{ return (r >> MRES_SEQ_SHIFT) & MRES_SEQ_MASK; }
static inline UW mres_status(UW r)	{ return (r >> MRES_ST_SHIFT) & MRES_ST_MASK; }
static inline UW mres_type(UW r)	{ return (r >> MRES_TYPE_SHIFT) & MRES_TYPE_MASK; }
static inline UW mres_reason(UW r)	{ return (r >> MRES_REJ_SHIFT) & MRES_REJ_MASK; }

/* ---------------------------------------------------------------------
 *  Steering word (16 bits): single writer = Line Following task, single
 *  reader = Motion Control.  Newest value wins, so a slow consumer can
 *  never overflow anything.  Freshness = the seq must change within
 *  MOTION_STEER_STALE_MS, otherwise Motion Control stops following.
 *    bit  0      valid
 *    bits 1..4   seq       (4)
 *    bits 5..11  steer     (7)  -63..+63, + = turn right; -64 invalid
 *    bits 12..15 speed     (4)  fraction of cruise, 0..15 (/15);
 *                               0 = pivot on the spot
 * --------------------------------------------------------------------- */
#define STEER_VALID		(1UL << 0)
#define STEER_SEQ_SHIFT		1U
#define STEER_SEQ_MASK		0xFUL
#define STEER_VAL_SHIFT		5U
#define STEER_VAL_MASK		0x7FUL
#define STEER_SPD_SHIFT		12U
#define STEER_SPD_MASK		0xFUL
#define STEER_MAX		63
#define STEER_SPD_FULL		15U

static inline UW steer_pack(UW seq, INT steer, UW spd)
{
	if (steer > STEER_MAX) {
		steer = STEER_MAX;
	} else if (steer < -STEER_MAX) {
		steer = -STEER_MAX;
	}
	return STEER_VALID
	     | ((seq & STEER_SEQ_MASK) << STEER_SEQ_SHIFT)
	     | (((UW)steer & STEER_VAL_MASK) << STEER_VAL_SHIFT)
	     | ((spd & STEER_SPD_MASK) << STEER_SPD_SHIFT);
}

static inline UW steer_seq(UW s)	{ return (s >> STEER_SEQ_SHIFT) & STEER_SEQ_MASK; }
static inline UW steer_speed(UW s)	{ return (s >> STEER_SPD_SHIFT) & STEER_SPD_MASK; }

static inline INT steer_value(UW s)
{
	INT	v = (INT)((s >> STEER_VAL_SHIFT) & STEER_VAL_MASK);

	return (v & 0x40) ? (v - 0x80) : v;	/* sign-extend 7 bits */
}

/* ---------------------------------------------------------------------
 *  Speed cap word (15 bits): single writer = Obstacle Scan task.
 *    bit  0     valid
 *    bits 1..4  seq   (4)  must keep changing (freshness)
 *    bits 5..14 cap   (10) mm/s, 0..1023
 * --------------------------------------------------------------------- */
#define SCAP_VALID		(1UL << 0)
#define SCAP_SEQ_SHIFT		1U
#define SCAP_SEQ_MASK		0xFUL
#define SCAP_VAL_SHIFT		5U
#define SCAP_VAL_MASK		0x3FFUL

static inline UW scap_pack(UW seq, UW cap_mms)
{
	if (cap_mms > SCAP_VAL_MASK) {
		cap_mms = SCAP_VAL_MASK;
	}
	return SCAP_VALID
	     | ((seq & SCAP_SEQ_MASK) << SCAP_SEQ_SHIFT)
	     | ((cap_mms & SCAP_VAL_MASK) << SCAP_VAL_SHIFT);
}

static inline UW scap_seq(UW c)		{ return (c >> SCAP_SEQ_SHIFT) & SCAP_SEQ_MASK; }
static inline UW scap_value(UW c)	{ return (c >> SCAP_VAL_SHIFT) & SCAP_VAL_MASK; }

/* ---------------------------------------------------------------------
 *  Motion state (Week 6 Motion Control statechart): 4 states -> 2 bits
 * --------------------------------------------------------------------- */
#define MS_IDLE		0U
#define MS_STRAIGHT	1U	/* Moving Straight (incl. line following) */
#define MS_TURNING	2U
#define MS_FAULT	3U

/* ---------------------------------------------------------------------
 *  Motion status snapshot.  Stored packed in two words (layout in
 *  motion_cmd.c / DATA_REPRESENTATION.md); this is the unpacked view
 *  handed to readers.
 * --------------------------------------------------------------------- */
#define MSTAT_SPEED_INVALID	(-1024)

typedef struct {
	H	speed_l_mms;	/* -1023..1023, MSTAT_SPEED_INVALID if unknown */
	H	speed_r_mms;
	UB	mstate;		/* MS_* */
	UB	source;		/* MSRC_* of the active command */
	UB	cmd_active;	/* 0/1 */
	UB	stall_l;	/* 0/1 */
	UB	stall_r;	/* 0/1 */
	UB	enc_ok;		/* 0/1 */
	UH	odometer_cm;	/* 0..65535 (saturating) */
	UH	heading_deg;	/* 0..358, 2 deg resolution (encoder estimate) */
	UH	cruise_mms;	/* 0..630, 10 mm/s resolution */
} MOTION_STATUS;

IMPORT void mstat_pack(const MOTION_STATUS *s, UW out[2]);
IMPORT void mstat_unpack(const UW in[2], MOTION_STATUS *s);

/* Validation of a command word against format/range/source rules
   (mode, priority and epoch are checked by Motion Control itself).
   Returns MREJ_NONE if acceptable. */
IMPORT UW mcmd_validate(UW w);

#endif /* MOTION_MSG_H */
