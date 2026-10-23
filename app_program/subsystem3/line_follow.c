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
	lf->search_ms = 0;
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
	} else if (lf->prev_err == 0 && lf->last_side == 0) {
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

	return (UB)((a == 0) ? 15 : (a == 1) ? 12 : (a == 2) ? 9 : 6);
}

LOCAL void search(LINE_FOLLOW *lf, UW dt_ms, LF_OUT *out)
{
	INT	first = (lf->last_side < 0) ? -1 : 1;
	INT	dir = (lf->search_phase == 0U) ? first : -first;
	UW	limit = (lf->search_phase == 0U) ? LF_SEARCH_T1_MS : 2U * LF_SEARCH_T1_MS;

	lf->search_ms += dt_ms;
	if (lf->search_ms >= limit) {
		lf->search_ms = 0;
		if (lf->search_phase == 0U) {
			lf->search_phase = 1;		/* sweep back past the start */
		} else {
			lf->search_phase = 0;
			lf->retries++;
			if (lf->retries >= LF_SEARCH_RETRIES) {
				lf->state = LS_LOST;	/* retries exhausted */
				out->events |= LF_EV_RECOVERY_FAIL;
				out->steer = 0;
				out->speed_frac = 0;
				return;
			}
		}
	}
	out->steer = dir * LF_SEARCH_PIVOT;
	out->speed_frac = 0;				/* pivot on the spot */
}

EXPORT void lf_step(LINE_FOLLOW *lf, UB black, W odo_mm, UW dt_ms,
		    BOOL navigating, LF_OUT *out)
{
	BOOL	on_line;

	out->steer = 0;
	out->speed_frac = 15;
	out->events = 0;
	out->steering = FALSE;
	if (!lf->calibrated) {
		return;
	}

	classify(lf, black, odo_mm, out);
	on_line = (black & (IR_BIT(IR_L) | IR_BIT(IR_C))) != 0U;

	if (on_line) {
		if (lf->state != LS_ON_LINE) {
			out->events |= LF_EV_FOUND;	/* line reacquired */
		}
		lf->state = LS_ON_LINE;
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
			out->events |= LF_EV_LOST;
		}
	} else if (lf->state == LS_LOST && navigating && lf->retries == 0U) {
		lf->state = LS_SEARCHING;	/* lost outside Normal Navigation */
	}

	if (navigating) {
		out->steering = TRUE;
		if (lf->state == LS_ON_LINE) {
			out->steer = LF_KP * lf->err + LF_KD * (lf->err - lf->prev_err);
			out->speed_frac = speed_for(lf->err);
		} else if (lf->state == LS_SEARCHING) {
			search(lf, dt_ms, out);
		} else {
			out->steer = 0;			/* LOST: hold still */
			out->speed_frac = 0;
		}
	}
	lf->prev_err = lf->err;
}
