/*
 *	line_follow.c
 *	Line position, steering and line-loss recovery (see line_follow.h).
 */

#include "line_follow.h"

/* Common LM393 IR modules pull D0 high over black (no reflection).  Only
   used until calibration has measured the real polarity. */
#define LF_DEFAULT_BLACK	1U

EXPORT void lf_reset_tracking(LINE_FOLLOW *lf)
{
	lf->state = lf->calibrated ? LS_ON_LINE : LS_UNKNOWN;
	lf->pos = LP_CENTER;
	lf->err = 0;
	lf->prev_err = 0;
	lf->lost_ms = 0;
	lf->lost_start_mm = 0;
	lf->lost_pending = FALSE;
	lf->settle_ms = 0;
	lf->pivot_dir = 0;
	lf->pivot_ms = 0;
	lf->in_junction = FALSE;
	lf->junction_start_mm = 0;
}

EXPORT void lf_init(LINE_FOLLOW *lf)
{
	UINT	i;

	for (i = 0; i < 3U; i++) {
		lf->black_level[i] = LF_DEFAULT_BLACK;
	}
	lf->calibrated = FALSE;
	lf_reset_tracking(lf);
}

EXPORT INT lf_calibrate(LINE_FOLLOW *lf, const UB *raw, UINT n)
{
	UINT	ones[3] = {0, 0, 0};
	UB	major[3];
	UINT	i, k;

	if (n == 0U) {
		return LF_CAL_UNSTABLE;
	}
	for (k = 0; k < n; k++) {
		for (i = 0; i < 3U; i++) {
			ones[i] += (raw[k] >> i) & 1U;
		}
	}
	for (i = 0; i < 3U; i++) {
		UINT	minority;

		major[i] = (ones[i] * 2U > n) ? 1U : 0U;
		minority = major[i] ? (n - ones[i]) : ones[i];
		if (minority * 5U > n) {
			return LF_CAL_UNSTABLE;		/* > 20 % flicker while still */
		}
	}
	if (major[IR_C] == major[IR_L]) {
		return LF_CAL_AMBIGUOUS;	/* cannot tell black from white */
	}
	/* Identical modules: one polarity for all three sensors */
	for (i = 0; i < 3U; i++) {
		lf->black_level[i] = major[IR_C];
	}
	lf->calibrated = TRUE;
	lf_reset_tracking(lf);
	return LF_CAL_OK;
}

EXPORT UB lf_black_bits(const LINE_FOLLOW *lf, UB raw)
{
	UB	b = 0;
	UINT	i;

	for (i = 0; i < 3U; i++) {
		if (((raw >> i) & 1U) == lf->black_level[i]) {
			b |= (UB)IR_BIT(i);
		}
	}
	return b;
}

LOCAL void classify(LINE_FOLLOW *lf, UB black, W odo_mm, LF_OUT *out)
{
	BOOL	l = (black & IR_BIT(IR_L)) != 0U;
	BOOL	c = (black & IR_BIT(IR_C)) != 0U;
	BOOL	r = (black & IR_BIT(IR_R)) != 0U;

	if (l && c && r) {
		/* crossing line under all three sensors */
		if (!lf->in_junction) {
			lf->in_junction = TRUE;
			lf->junction_start_mm = odo_mm;
		} else if (lf->junction_start_mm != (W)0x7FFFFFFF
			   && odo_mm - lf->junction_start_mm >= LF_JUNCTION_MM) {
			out->events |= LF_EV_JUNCTION;
			lf->junction_start_mm = (W)0x7FFFFFFF;	/* report once */
		}
		lf->pos = LP_JUNCTION;
		lf->err = 0;
		return;
	}
	lf->in_junction = FALSE;
	/* Otherwise IR_R is the barcode sensor: whatever it reads, it never
	   steers */
	if (c && l) {
		lf->pos = LP_LEFT_NEAR;
		lf->err = -1;
	} else if (c) {
		lf->pos = LP_CENTER;
		lf->err = 0;
	} else if (l) {
		lf->pos = LP_LEFT_FAR;
		lf->err = -2;
	} else {
		lf->pos = LP_LOST_RIGHT;		/* turn towards IR_R */
		lf->err = 3 * LF_LOST_DIR;
	}
}

LOCAL UB speed_for(INT err)
{
	INT	a = (err < 0) ? -err : err;

	/* Any steer runs at about half cruise (|err| 1 was 11/15) so the car
	   turns onto the line instead of running over it.  |err| 2 (IR_L
	   only) pivots on the spot: see LF_SIDE_PIVOT_STEER in lf_step(). */
	return (UB)((a == 0) ? 15 : (a == 1) ? 8 : 0);
}

EXPORT void lf_step(LINE_FOLLOW *lf, UB black, W odo_mm, UW dt_ms,
		    BOOL navigating, LF_OUT *out)
{
	BOOL	c = (black & IR_BIT(IR_C)) != 0U;
	BOOL	l = (black & IR_BIT(IR_L)) != 0U;
	BOOL	was_lost;
	BOOL	on_line;

	out->steer = 0;
	out->speed_frac = 15;
	out->events = 0;
	out->steering = FALSE;
	if (!lf->calibrated) {
		return;
	}

	was_lost = (lf->pos == LP_LOST_RIGHT) ? TRUE : FALSE;
	classify(lf, black, odo_mm, out);
	/* Side pivot: the line under IR_L alone (centre white) commits the
	   car to pivot that way until the centre has it.  IR_L often goes
	   white again before the centre reaches the line; the commitment
	   stops the lost-line turn from turning the car back. */
	if (c) {
		if (lf->pivot_dir != 0) {
			lf->settle_ms = LF_SETTLE_MS;
			lf->prev_err = lf->err;	/* no D kick back the other way */
		}
		lf->pivot_dir = 0;
		lf->pivot_ms = 0;
	} else {
		if (navigating && l && lf->pivot_ms <= LF_SIDE_PIVOT_MAX_MS) {
			lf->pivot_dir = -1;
		}
		if (lf->pivot_dir != 0) {
			lf->pivot_ms += dt_ms;
			if (lf->pivot_ms > LF_SIDE_PIVOT_MAX_MS) {
				lf->pivot_dir = 0;	/* not found: lost-line turn */
			}
		}
	}
	on_line = c || l || lf->pivot_dir != 0;
	if (on_line && was_lost) {
		/* The lost-line turn found it: settle in, no D kick from the
		   turn's error */
		lf->settle_ms = LF_SETTLE_MS;
		lf->prev_err = lf->err;
	}

	if (on_line) {
		if (lf->state != LS_ON_LINE) {
			out->events |= LF_EV_FOUND;	/* line reacquired */
		}
		lf->state = LS_ON_LINE;
		lf->lost_ms = 0;
		lf->lost_pending = FALSE;
	} else if (lf->state == LS_ON_LINE) {
		if (!lf->lost_pending) {
			lf->lost_pending = TRUE;
			lf->lost_start_mm = odo_mm;
			lf->lost_ms = 0;
		}
		lf->lost_ms += dt_ms;
		if (odo_mm - lf->lost_start_mm >= LF_LOST_CONFIRM_MM
		    || lf->lost_ms >= LF_LOST_CONFIRM_MS) {
			lf->state = navigating ? LS_SEARCHING : LS_LOST;
			out->events |= LF_EV_LOST;
		}
	} else if (lf->state == LS_LOST && navigating) {
		lf->state = LS_SEARCHING;	/* lost outside Normal Navigation */
	}

	if (navigating) {
		out->steering = TRUE;
		if (lf->pivot_dir != 0) {
			/* side pivot: no forward travel until the centre has
			   the line, then LF_SETTLE_MS at half speed */
			out->steer = lf->pivot_dir * LF_SIDE_PIVOT_STEER;
			out->speed_frac = 0;
		} else if (!on_line) {
			/* centre and IR_L white: turn on the spot towards IR_R
			   (the car's left) until one of them sees black, also
			   before LF_EV_LOST.  Never gives up. */
			out->steer = LF_LOST_DIR * LF_LOST_PIVOT_STEER;
			out->speed_frac = 0;
		} else {
			out->steer = LF_KP * lf->err + LF_KD * (lf->err - lf->prev_err);
			out->speed_frac = speed_for(lf->err);
			if (lf->settle_ms > 0U && out->speed_frac > LF_SETTLE_SPEED) {
				out->speed_frac = LF_SETTLE_SPEED;
			}
		}
	}
	lf->prev_err = lf->err;
	lf->settle_ms = (lf->settle_ms > dt_ms) ? lf->settle_ms - dt_ms : 0U;
}
