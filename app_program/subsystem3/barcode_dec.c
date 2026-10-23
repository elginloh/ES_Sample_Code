/*
 *	barcode_dec.c
 *	Barcode capture state machine (see barcode_dec.h).
 */

#include "barcode_dec.h"

LOCAL UH usable_speed(BC_DEC *d, W speed_mms)
{
	if (speed_mms < 0) {
		speed_mms = -speed_mms;		/* reading while reversing */
	}
	if (speed_mms > 0) {
		d->last_speed = (UH)((speed_mms > 0xFFFF) ? 0xFFFF : speed_mms);
		return d->last_speed;
	}
	return (d->last_speed != 0U) ? d->last_speed : (UH)BC_FALLBACK_MMS;
}

/* time (us) x speed (mm/s) / 1000 = micrometres */
LOCAL UW width_um(UW dt_us, UW v_mms)
{
	return (UW)(((UD)dt_us * (UD)v_mms) / 1000U);
}

EXPORT void bc_init(BC_DEC *d)
{
	d->state = BCS_IDLE;
	d->level = 0;
	d->level_known = FALSE;
	d->t_last_us = 0;
	d->n_edges = 0;
	d->start_odo_mm = 0;
	d->lockout = FALSE;
	d->lockout_end_mm = 0;
	d->have_last = FALSE;
	d->last_sym = C39_NONE;
	d->last_sym_mm = 0;
	d->widest_um = 0;
	d->last_speed = 0;
}

EXPORT BOOL bc_capturing(const BC_DEC *d)
{
	return (d->state == BCS_CAPTURING) ? TRUE : FALSE;
}

LOCAL void clear_result(BC_RESULT *res)
{
	res->result = BCR_NONE;
	res->n_data = 0;
	res->reversed = FALSE;
	res->err = 0;
	res->n_elements = 0;
}

LOCAL void fail(BC_DEC *d, BC_RESULT *res, INT err)
{
	res->result = BCR_INVALID;
	res->err = err;
	d->state = BCS_IDLE;
	d->n_edges = 0;
}

EXPORT void bc_abort(BC_DEC *d, BC_RESULT *res)
{
	clear_result(res);
	if (d->state == BCS_CAPTURING) {
		fail(d, res, BC_ERR_ABORTED);
	}
}

/* Build element widths from the edge list, merge glitches, decode */
LOCAL void finish(BC_DEC *d, W odo_mm, BC_RESULT *res)
{
	UW	el[BC_MAX_EDGES];
	UINT	n = 0, i;
	INT	r;

	for (i = 0; i + 1U < d->n_edges; i++) {
		UW	v = ((UW)d->edge_v[i] + (UW)d->edge_v[i + 1U]) / 2U;

		el[n++] = width_um(d->edge_t[i + 1U] - d->edge_t[i], v);
	}
	/* Merge an interior element shorter than BC_MIN_ELEMENT_UM into its
	   neighbours (a dropout inside a bar or a speck inside a space). */
	i = 1;
	while (i + 1U < n) {
		if (el[i] < BC_MIN_ELEMENT_UM) {
			UINT	k;

			el[i - 1U] += el[i] + el[i + 1U];
			for (k = i; k + 2U < n; k++) {
				el[k] = el[k + 2U];
			}
			n -= 2U;
		} else {
			i++;
		}
	}
	res->n_elements = (UB)n;
	r = c39_decode(el, n, res->data, BC_MAX_DATA, &res->reversed);
	d->state = BCS_IDLE;
	d->n_edges = 0;
	if (r < 0) {
		res->result = BCR_INVALID;
		res->err = r;
		return;
	}
	res->n_data = (UB)r;
	/* Duplicate-reading prevention: the same symbol again shortly after */
	if (d->have_last && d->last_sym == res->data[0]
	    && (odo_mm - d->last_sym_mm) < BC_DUP_WINDOW_MM) {
		res->result = BCR_DUPLICATE;
	} else {
		res->result = BCR_VALID;
	}
	d->have_last = TRUE;
	d->last_sym = res->data[0];
	d->last_sym_mm = odo_mm;
	d->lockout = TRUE;
	d->lockout_end_mm = odo_mm + BC_LOCKOUT_MM;
}

EXPORT void bc_edge(BC_DEC *d, UW t_us, UB black, W speed_mms, W odo_mm,
		    BC_RESULT *res)
{
	UH	v = usable_speed(d, speed_mms);

	clear_result(res);
	black = (black != 0U) ? 1U : 0U;
	if (d->level_known && black == d->level) {
		return;				/* lost edge pair: ignore */
	}

	if (d->state == BCS_IDLE) {
		if (black != 0U) {
			UW	quiet = d->level_known
				      ? width_um(t_us - d->t_last_us, v) : BC_QUIET_MIN_UM;

			if (d->lockout && odo_mm < d->lockout_end_mm) {
				/* still on the barcode just read */
			} else if (quiet >= BC_QUIET_MIN_UM) {
				d->lockout = FALSE;
				d->state = BCS_CAPTURING;
				d->n_edges = 0;
				d->widest_um = 0;
				d->start_odo_mm = odo_mm;
				d->edge_t[d->n_edges] = t_us;
				d->edge_v[d->n_edges] = v;
				d->n_edges++;
			}
		}
	} else {
		UW	el = width_um(t_us - d->t_last_us,
				      ((UW)d->edge_v[d->n_edges - 1U] + v) / 2U);

		if (el > BC_MAX_ELEMENT_UM) {
			fail(d, res, BC_ERR_LONG);	/* e.g. the line itself */
		} else if (d->n_edges >= BC_MAX_EDGES) {
			fail(d, res, BC_ERR_OVERFLOW);
		} else {
			if (el > d->widest_um) {
				d->widest_um = el;
			}
			d->edge_t[d->n_edges] = t_us;
			d->edge_v[d->n_edges] = v;
			d->n_edges++;
		}
	}
	d->level = black;
	d->level_known = TRUE;
	d->t_last_us = t_us;
}

EXPORT void bc_poll(BC_DEC *d, UW now_us, W speed_mms, W odo_mm,
		    BC_RESULT *res)
{
	UH	v = usable_speed(d, speed_mms);
	UW	quiet;
	UW	end_q;

	clear_result(res);
	if (d->state != BCS_CAPTURING) {
		return;
	}
	if ((UW)(now_us - d->t_last_us) > BC_IDLE_TIMEOUT_US) {
		fail(d, res, BC_ERR_STALLED);
		return;
	}
	if ((odo_mm - d->start_odo_mm) * 1000 > BC_MAX_LENGTH_UM) {
		fail(d, res, BC_ERR_WINDOW);
		return;
	}
	if (d->level == 0U) {
		quiet = width_um(now_us - d->t_last_us, v);
		end_q = d->widest_um * BC_END_QUIET_X10 / 10U;
		if (end_q < BC_END_QUIET_UM) {
			end_q = BC_END_QUIET_UM;
		}
		if (quiet >= end_q) {
			finish(d, odo_mm, res);	/* completion: trailing quiet zone */
		}
	}
}
