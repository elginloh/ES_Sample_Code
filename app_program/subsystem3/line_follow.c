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
	lf->last_side = 0;
	lf->lost_ms = 0;
	lf->lost_start_mm = 0;
	lf->lost_pending = FALSE;
	lf->gap_ms = 0;
	lf->realign_ms = 0;
	lf->realign_dir = 0;
	lf->turn_dir = 0;
	lf->turn_ms = 0;
	lf->backing = FALSE;
	lf->backup_start_mm = 0;
	lf->search_ms = 0;
	lf->search_origin = LF_HEADING_NONE;
	lf->search_deg = 0;
	lf->search_phase = 0;
	lf->retries = 0;
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

LOCAL void classify(LINE_FOLLOW *lf, UB black, W odo_mm, UW dt_ms, LF_OUT *out)
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
	if (c || l) {
		lf->gap_ms = 0;			/* line seen: end any gap */
	}
	if (!c && !l && lf->realign_ms > 0U) {
		/* Just re-found by a search sweep and lost again: the line
		   lies on the sweep side, not "probably right". */
		lf->pos = (lf->realign_dir < 0) ? LP_LOST_LEFT : LP_LOST_RIGHT;
		lf->err = 3 * lf->realign_dir;
		lf->last_side = lf->realign_dir;
		return;
	}
	if (c && !l) {
		lf->pos = LP_CENTER;
		lf->err = 0;
		lf->last_side = 0;
	} else if (c && l) {
		lf->pos = LP_LEFT_NEAR;
		lf->err = -1;
		lf->last_side = -1;
	} else if (l) {
		lf->pos = LP_LEFT_FAR;
		lf->err = -2;
		lf->last_side = -1;
	} else if (lf->prev_err < 0 || lf->last_side < 0) {
		lf->pos = LP_LOST_LEFT;		/* went beyond the left sensor */
		lf->err = -3;
	} else if (lf->prev_err == 0 && lf->last_side == 0 && !r
		   && lf->gap_ms < LF_GAP_HOLD_MS) {
		/* Centre just went white with no side information: a gap,
		   a worn patch or a noisy reading.  Hold straight briefly
		   instead of turning right at once.  Black under the right
		   sensor means the line really went right (a right-hand
		   corner): no hold. */
		lf->gap_ms += dt_ms;
		lf->pos = LP_CENTER;
		lf->err = 0;
	} else if (lf->prev_err == 0 && lf->last_side == 0 && !r
		   && lf->turn_dir > 0) {
		/* Was steering right when the centre lost it: the car
		   turned past the line, which is now on the LEFT. */
		lf->pos = LP_LOST_LEFT;
		lf->err = -2;
		lf->last_side = -1;
	} else if (lf->prev_err == 0 && lf->last_side == 0) {
		/* Straight or steering left: the line went right. */
		lf->pos = LP_RIGHT;		/* left the centre sensor to the right */
		lf->err = 2;
		lf->last_side = 1;
	} else {
		lf->pos = LP_LOST_RIGHT;
		lf->err = 3;
		lf->last_side = 1;
	}
}

LOCAL UB speed_for(INT err)
{
	INT	a = (err < 0) ? -err : err;

	/* |err| 3 (line beyond a side sensor, e.g. a 90 deg corner): pivot
	   on the spot.  Driving on while turning carried the axle past the
	   corner, and the sensors' arc then missed the new line. */
	return (UB)((a == 0) ? 15 : (a == 1) ? 11 : (a == 2) ? 7 : 0);
}

/* Current sweep direction: -1 left, +1 right */
LOCAL INT search_dir(const LINE_FOLLOW *lf)
{
	INT	first = (lf->last_side < 0) ? -1 : 1;

	return (lf->search_phase == 0U) ? first : -first;
}

/* Sweep angle from the search origin, + = right (the steering sign).
   The IMU heading grows turning left, so the difference is negated. */
LOCAL INT angle_right(INT origin, INT heading)
{
	return 180 - (heading - origin + 540) % 360;
}

LOCAL void search(LINE_FOLLOW *lf, W odo_mm, INT heading, UW dt_ms, LF_OUT *out)
{
	INT	dir = search_dir(lf);
	UW	limit;
	BOOL	done;

	if (lf->backing) {
		/* Phase 0: back up straight so the sensors come back over
		   a line they overshot, then sweep.  The odometer counts
		   down while reversing: compare the distance, not its sign. */
		W	moved = odo_mm - lf->backup_start_mm;

		if (moved < 0) {
			moved = -moved;
		}
		if (moved < LF_BACKUP_MM) {
			out->steer = 0;
			out->speed_frac = 15;		/* capped to creep */
			out->reverse = TRUE;
			return;
		}
		lf->backing = FALSE;
	}
	/* First sweep: centre -> LF_SEARCH_DEG.  Every later sweep runs edge
	   to edge, so repeats keep covering both sides. */
	limit = (lf->search_phase == 0U && lf->retries == 0U)
		? LF_SEARCH_T1_MS : 2U * LF_SEARCH_T1_MS;

	lf->search_ms += dt_ms;
	if (lf->search_origin != LF_HEADING_NONE && heading != LF_HEADING_NONE) {
		/* Measured: turn until LF_SEARCH_DEG from where the line was
		   lost.  Half as long again as the timed sweep ends one whose
		   heading does not follow the turn (motors bending the field). */
		lf->search_deg = angle_right(lf->search_origin, heading);
		done = (dir * lf->search_deg >= LF_SEARCH_DEG
			|| lf->search_ms >= limit + limit / 2U) ? TRUE : FALSE;
	} else {
		done = (lf->search_ms >= limit) ? TRUE : FALSE;
	}
	if (done) {
		lf->search_ms = 0;
		if (lf->search_phase == 0U) {
			lf->search_phase = 1;		/* sweep back past the start */
		} else {
			lf->search_phase = 0;
			lf->retries++;
			/* Keep searching while the line is missing.  A temporary
			   loss must not latch SAFE_STOP and require reflashing.
			   The existing obstacle/collision safety logic still applies. */
			if (lf->retries > LF_SEARCH_RETRIES) {
				lf->retries = LF_SEARCH_RETRIES;	/* saturate, not 0:
							   0 means the first sweep */
			}
		}
		dir = search_dir(lf);			/* turn the new way now */
	}
	out->steer = dir * LF_SEARCH_PIVOT;
	out->speed_frac = 0;				/* pivot on the spot */
}

EXPORT void lf_step(LINE_FOLLOW *lf, UB black, W odo_mm, INT heading_deg,
		    UW dt_ms, BOOL navigating, LF_OUT *out)
{
	BOOL	on_line;
	INT	sweep;

	out->steer = 0;
	out->speed_frac = 15;
	out->events = 0;
	out->steering = FALSE;
	out->reverse = FALSE;
	if (!lf->calibrated) {
		return;
	}

	sweep = search_dir(lf);			/* before classify moves last_side */
	classify(lf, black, odo_mm, dt_ms, out);
	on_line = (black & (IR_BIT(IR_L) | IR_BIT(IR_C))) != 0U;

	if (on_line) {
		if (lf->state == LS_SEARCHING) {
			/* Found mid-sweep: the car still points across the
			   line.  Keep turning the sweep way while moving on. */
			lf->realign_dir = sweep;
			lf->realign_ms = LF_REALIGN_MS;
		}
		if (lf->state != LS_ON_LINE) {
			out->events |= LF_EV_FOUND;	/* line reacquired */
		}
		lf->state = LS_ON_LINE;
		lf->backing = FALSE;
		lf->lost_ms = 0;
		lf->lost_pending = FALSE;
		lf->search_ms = 0;
		lf->search_phase = 0;
		lf->retries = 0;
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
			lf->backing = (navigating && LF_BACKUP_MM > 0) ? TRUE : FALSE;
			lf->backup_start_mm = odo_mm;
			lf->search_origin = heading_deg;	/* sweeps measured from here */
			lf->search_deg = 0;
			out->events |= LF_EV_LOST;
		}
	} else if (lf->state == LS_LOST && navigating && lf->retries == 0U) {
		lf->state = LS_SEARCHING;	/* lost outside Normal Navigation */
		lf->backing = (LF_BACKUP_MM > 0) ? TRUE : FALSE;
		lf->backup_start_mm = odo_mm;
		lf->search_origin = heading_deg;
		lf->search_deg = 0;
	}

	if (navigating) {
		out->steering = TRUE;
		if (lf->state == LS_ON_LINE && lf->realign_ms > 0U
		    && (lf->err == 0 || lf->err == 3 * lf->realign_dir)) {
			/* Settling in after a sweep: an arc while moving on,
			   also when the centre slips off (never a spot pivot,
			   which spun the car past the line). */
			out->steer = lf->realign_dir * LF_REALIGN_STEER;
			out->speed_frac = LF_REALIGN_SPEED;
		} else if (lf->state == LS_ON_LINE) {
			INT	kp = (lf->err == 3 || lf->err == -3) ? LF_KP_PIVOT : LF_KP;

			out->steer = kp * lf->err + LF_KD * (lf->err - lf->prev_err);
			out->speed_frac = speed_for(lf->err);
		} else if (lf->state == LS_SEARCHING) {
			search(lf, odo_mm, heading_deg, dt_ms, out);
		} else {
			out->steer = 0;			/* LOST: hold still */
			out->speed_frac = 0;
		}
	}
	/* Remember which way the car was recently steering on the line.
	   Uses the steering intent (P term / realign), not out->steer: the
	   D term's brief counter-steer as the line re-centres would
	   otherwise read as a turn the other way. */
	if (navigating && lf->state == LS_ON_LINE) {
		INT	intent = (lf->realign_ms > 0U && lf->err == 0)
				 ? lf->realign_dir * LF_REALIGN_STEER
				 : LF_KP * lf->err;

		if (intent >= LF_TURN_MIN_STEER) {
			lf->turn_dir = 1;
			lf->turn_ms = 0;
		} else if (intent <= -LF_TURN_MIN_STEER) {
			lf->turn_dir = -1;
			lf->turn_ms = 0;
		} else if (lf->turn_ms < LF_TURN_MEMORY_MS) {
			lf->turn_ms += dt_ms;
		} else {
			lf->turn_dir = 0;	/* straight long enough: forget */
		}
	}
	lf->prev_err = lf->err;
	lf->realign_ms = (lf->realign_ms > dt_ms) ? lf->realign_ms - dt_ms : 0U;
}
