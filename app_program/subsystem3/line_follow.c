/*
 *	line_follow.c
 *	Line position, steering and line-loss recovery (see line_follow.h).
 */

#include "line_follow.h"

/* This robot's LM393 D0 wiring is high over black (no reflection). */
#define LF_DEFAULT_BLACK	LF_CONFIGURED_BLACK_LEVEL

EXPORT void lf_reset_tracking(LINE_FOLLOW *lf)
{
	lf->state = lf->calibrated ? LS_ON_LINE : LS_UNKNOWN;
	lf->pos = LP_CENTER;
	lf->err = 0;
	lf->prev_err = 0;
	lf->steer_cmd = 0;
	lf->edge_side = 0;
	lf->edge_ms = 0;
	lf->hold_side = 0;
	lf->hold_ms = 0;
	lf->lost_ms = 0;
	lf->lost_start_mm = 0;
	lf->lost_pending = FALSE;
	lf->center_gap_ms = 0;
	lf->settle_ms = 0;
	lf->straight_ms = 0;
	lf->turning = FALSE;
	lf->turn_start_mm = 0;
	lf->turn_ms = 0;
	lf->realign_ms = 0;
	lf->pivot_dir = 0;
	lf->pivot_ms = 0;
	lf->in_junction = FALSE;
	lf->junction_start_mm = 0;
	lf->junction_ms = 0;
	lf->post_turn = FALSE;
	lf->post_turn_ms = 0;
	lf->post_pivot = 0;
}

EXPORT void lf_turn_done(LINE_FOLLOW *lf, BOOL centre_acquired)
{
	lf_reset_tracking(lf);
	if (lf->calibrated) {
		lf->post_turn = !centre_acquired;
		lf->straight_ms = centre_acquired ? LF_RIGHT_TURN_STRAIGHT_MS : 0;
	}
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
	UINT	i, k;

	if (n == 0U) {
		return LF_CAL_UNSTABLE;
	}
	for (k = 0; k < n; k++) {
		for (i = 0; i < 3U; i++) {
			ones[i] += (raw[k] >> i) & 1U;
		}
	}
	/* GP26 may be crossing barcode bars while GP6/GP2 are stable.
	   Its transitions must never block qualification of line tracking. */
	for (i = 0; i < 2U; i++) {
		UINT	minority;
		UB	major;

		major = (ones[i] * 2U > n) ? 1U : 0U;
		minority = major ? (n - ones[i]) : ones[i];
		if (minority * 5U > n) {
			return LF_CAL_UNSTABLE;		/* > 20 % flicker while still */
		}
	}
	/* Starting position is deliberately irrelevant: all-white, all-black,
	   centre-on-line and right-sensor-on-line must all enter NORMAL. */
	for (i = 0; i < 3U; i++) {
		lf->black_level[i] = LF_CONFIGURED_BLACK_LEVEL;
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

LOCAL void classify(LINE_FOLLOW *lf, UB black, W odo_mm, UW dt_ms, LF_OUT *out)
{
	BOOL	l = (black & IR_BIT(IR_L)) != 0U;
	BOOL	c = (black & IR_BIT(IR_C)) != 0U;

	if (l && c) {
		/* Only the two tracking sensors define a junction. GP26 is
		   the separate barcode sensor and need not see the crossing. */
		if (!lf->in_junction) {
			lf->in_junction = TRUE;
			lf->junction_start_mm = odo_mm;
			lf->junction_ms = 0;
		}
		lf->junction_ms += dt_ms;
		if (lf->junction_start_mm != (W)0x7FFFFFFF
		    && lf->junction_ms >= LF_JUNCTION_CONFIRM_MS) {
			out->events |= LF_EV_JUNCTION;
			lf->junction_start_mm = (W)0x7FFFFFFF;	/* report once */
		}
		lf->pos = LP_JUNCTION;
		lf->err = 0;
		return;
	}
	lf->in_junction = FALSE;
	lf->junction_ms = 0;
	/* Otherwise IR_R is the barcode sensor: whatever it reads, it never
	   steers.  C and L bracket the desired white corridor. */
	if (c) {
		lf->pos = LP_RIGHT_NEAR;
		lf->err = 1;	/* task_line mirrors this to physical LEFT */
	} else if (l) {
		lf->pos = LP_LEFT_NEAR;
		lf->err = -1;
	} else if (lf->state == LS_ON_LINE && !lf->turning) {
		lf->pos = LP_CENTER;
		lf->err = LF_EDGE_GAP_ERR;
	} else {
		lf->pos = LP_LOST_LEFT;	/* search towards the tracking gap */
		lf->err = 3 * LF_LOST_DIR;
	}
}

LOCAL UB speed_for(INT err)
{
	INT	a = (err < 0) ? -err : err;

	/* Edge corrections remain moving arcs; only confirmed line-loss
	   recovery is allowed to pivot. */
	return (UB)((a == 0) ? LF_CRUISE_SPEED : LF_NEAR_SPEED);
}

LOCAL INT steer_slew(INT current, INT target)
{
	if (target > current + LF_STEER_SLEW_STEP) {
		return current + LF_STEER_SLEW_STEP;
	}
	if (target < current - LF_STEER_SLEW_STEP) {
		return current - LF_STEER_SLEW_STEP;
	}
	return target;
}

/* Steering target while on the line.  A sensor on black turns the car
   towards it (task_line mirrors the sign: C black -> right wheel faster),
   harder the longer it stays black.  When a sustained contact ends, the
   line may have slipped past the sensor rather than back into the
   corridor, so keep turning that way for LF_HOLD_MS instead of driving
   straight off it. */
LOCAL INT edge_steer(LINE_FOLLOW *lf, UW dt_ms)
{
	INT	mag;

	if (lf->pos == LP_JUNCTION) {
		lf->edge_side = 0;
		lf->edge_ms = 0;
		lf->hold_ms = 0;
		return 0;			/* crossing line: straight */
	}
	if (lf->err != 0) {
		if (lf->err != lf->edge_side) {
			lf->edge_side = lf->err;
			lf->edge_ms = 0;
		}
		mag = LF_EDGE_STEER_MIN + (INT)(lf->edge_ms / LF_EDGE_GROW_MS);
		if (mag > LF_EDGE_STEER_MAX) {
			mag = LF_EDGE_STEER_MAX;
		}
		lf->edge_ms += dt_ms;
		lf->hold_ms = 0;
		return lf->err * mag;
	}
	if (lf->edge_side != 0 && lf->edge_ms >= LF_HOLD_MIN_MS) {
		lf->hold_side = lf->edge_side;	/* sustained contact just ended */
		lf->hold_ms = LF_HOLD_MS;
	}
	lf->edge_side = 0;
	lf->edge_ms = 0;
	if (lf->hold_ms > 0U) {
		lf->hold_ms = (lf->hold_ms > dt_ms) ? lf->hold_ms - dt_ms : 0U;
		return lf->hold_side * LF_HOLD_STEER;
	}
	return 0;
}

/* Realign after a barcode turn (see LF_POST_TURN_*).  Returns TRUE while
   it owns the steering; FALSE once done (normal tracking from then on).
   err sign convention: +1 centre black, -1 GP2 (IR_L) black. */
LOCAL BOOL post_turn_step(LINE_FOLLOW *lf, BOOL c, BOOL l, UW dt_ms,
			  LF_OUT *out)
{
	INT	side = (c && !l) ? 1 : ((l && !c) ? -1 : 0);

	lf->post_turn_ms += dt_ms;
	if (lf->post_pivot == 0 && side != 0) {
		/* turned past the line (left turn: GP2 on it) or short of it
		   (centre on it): pivot towards that sensor */
		lf->post_pivot = side;
		lf->post_turn_ms = 0;
	}
	if (lf->post_pivot != 0) {
		if (side == lf->post_pivot
		    && lf->post_turn_ms < LF_POST_TURN_PIVOT_MAX_MS) {
			out->steer = side * LF_POST_TURN_PIVOT_STEER;
			out->speed_frac = 0;
			lf->steer_cmd = out->steer;
			return TRUE;
		}
		/* sensor white again: the line is between the two sensors */
		lf->settle_ms = LF_SETTLE_MS;
	} else if (!c && !l && lf->post_turn_ms < LF_POST_TURN_MS) {
		/* line already between the sensors: straight on */
		out->steer = 0;
		out->speed_frac = LF_CRUISE_SPEED;
		lf->steer_cmd = 0;
		return TRUE;
	}
	lf->post_turn = FALSE;
	lf->post_pivot = 0;
	lf->post_turn_ms = 0;
	lf->steer_cmd = 0;		/* aligned: start from straight */
	lf->edge_side = 0;
	lf->edge_ms = 0;
	lf->hold_ms = 0;
	return FALSE;
}

EXPORT void lf_step(LINE_FOLLOW *lf, UB black, W odo_mm, UW dt_ms,
		    BOOL navigating, LF_OUT *out)
{
	BOOL	c;
	BOOL	l;
	BOOL	on_line;

	out->steer = 0;
	out->speed_frac = LF_CRUISE_SPEED;
	out->events = 0;
	out->steering = FALSE;
	if (!lf->calibrated) {
		return;
	}

	/* Both-white is the intended tracking corridor, not evidence of
	   losing the line. Elapsed time alone cannot distinguish those cases. */
	c = (black & IR_BIT(IR_C)) != 0U;
	l = (black & IR_BIT(IR_L)) != 0U;
	lf->center_gap_ms = 0;	/* legacy timer no longer controls tracking */

	classify(lf, black, odo_mm, dt_ms, out);
	/* Ordinary edge contact is always a moving correction. */
	lf->pivot_dir = 0;
	lf->pivot_ms = 0;
	/* During avoidance Buddy 5 needs true sensor-based LOST/FOUND events,
	   so the virtual white corridor applies only while Buddy 3 owns
	   steering in Normal Navigation. */
	on_line = c || l
		  || (navigating && lf->state == LS_ON_LINE && !lf->turning);
	if (on_line) {
		if (lf->turning) {
			lf->prev_err = lf->err;	/* no D kick from the lost error */
			lf->steer_cmd = 0;	/* found: ramp from straight, not the pivot */
			if (lf->pivot_dir == 0 && lf->turn_ms >= LF_REALIGN_MIN_MS) {
				/* A long lost-line turn found it: the car
				   still points across the curve, keep turning
				   that way while moving on.  Not after a short
				   one: the car is about straight, and turning
				   on swung it ~10 deg across a straight line,
				   which then weaved right and left. */
				lf->realign_ms = LF_REALIGN_MS;
			}
		}
		lf->turning = FALSE;
	} else {
		if (!lf->turning) {
			lf->turning = TRUE;
			lf->turn_start_mm = odo_mm;
			lf->turn_ms = 0;
		}
		lf->turn_ms += dt_ms;
	}
	if (lf->pivot_dir != 0) {
		lf->realign_ms = 0;	/* the pivot side, not the turn side */
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

	if (!navigating) {
		lf->post_turn = FALSE;
		lf->post_pivot = 0;
		lf->straight_ms = 0;
	}
	if (navigating) {
		out->steering = TRUE;
		if (on_line && lf->straight_ms > 0U) {
			out->steer = 0;
			out->speed_frac = LF_SETTLE_SPEED;
		} else if (on_line && lf->post_turn
		    && post_turn_step(lf, c, l, dt_ms, out)) {
			/* realigning after a barcode turn */
		} else if (!on_line && lf->turn_ms <= LF_LOST_ARC_MS
			   && odo_mm - lf->turn_start_mm < LF_LOST_ARC_MM) {
			/* centre and IR_L white: arc towards IR_R (the car's
			   left) at once while moving on, a curve ... */
			out->steer = LF_LOST_DIR * LF_LOST_ARC_STEER;
			out->speed_frac = LF_LOST_ARC_SPEED;
		} else if (!on_line) {
			/* ... still white: a sharp corner, turn on the spot
			   until one of them sees black.  Never gives up. */
			out->steer = LF_LOST_DIR * LF_LOST_PIVOT_STEER;
			out->speed_frac = 0;
		} else if (lf->realign_ms > 0U && lf->err == 0) {
			/* just found by that turn, centre only: keep turning
			   gently the same way while moving on */
			out->steer = LF_LOST_DIR * LF_REALIGN_STEER;
			out->speed_frac = LF_REALIGN_SPEED;
		} else {
			INT target = edge_steer(lf, dt_ms);

			out->steer = steer_slew(lf->steer_cmd, target);
			lf->steer_cmd = out->steer;
			out->speed_frac = speed_for(lf->err);
			if (lf->settle_ms > 0U && out->speed_frac > LF_SETTLE_SPEED) {
				out->speed_frac = LF_SETTLE_SPEED;
			}
		}
		if (!on_line) {
			/* Recovery has its own bounded arc/pivot values.  Start
			   normal slew limiting from that value when the line is found. */
			lf->steer_cmd = out->steer;
			lf->edge_side = 0;
			lf->edge_ms = 0;
			lf->hold_ms = 0;
		}
	}
	lf->prev_err = lf->err;
	lf->straight_ms = (lf->straight_ms > dt_ms) ? lf->straight_ms - dt_ms : 0U;
	lf->settle_ms = (lf->settle_ms > dt_ms) ? lf->settle_ms - dt_ms : 0U;
	lf->realign_ms = (lf->realign_ms > dt_ms) ? lf->realign_ms - dt_ms : 0U;
}
