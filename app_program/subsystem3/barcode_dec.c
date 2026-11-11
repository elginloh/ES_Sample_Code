/*
 *	barcode_dec.c
 *	Barcode capture state machine (see barcode_dec.h).
 */

#include "barcode_dec.h"

/* Only the digital sensor timestamps determine element widths. */
#define BC_NOISE_US 1000U
#define BC_START_QUIET_US 30000U
#define BC_END_QUIET_US 100000U
#define BC_DUP_US 2000000UL

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
	d->last_decode_us = 0;
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
	res->n_diag = 0;
}

LOCAL void fail(BC_DEC *d, BC_RESULT *res, INT err)
{
	res->result = BCR_INVALID;
	res->err = err;
	res->n_elements = d->n_edges > 0U ? (UB)(d->n_edges - 1U) : 0U;
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
		el[n++] = d->edge_t[i + 1U] - d->edge_t[i];
	}
	/* Merge an interior element shorter than BC_MIN_ELEMENT_UM into its
	   neighbours (a dropout inside a bar or a speck inside a space). */
	i = 1;
	while (i + 1U < n) {
		if (el[i] < BC_NOISE_US) {
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
	res->n_diag = (UB)n;
	for (i = 0; i < n; i++) {
		res->element_um[i] = el[i];
	}
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
	    && (UW)(d->t_last_us - d->last_decode_us) < BC_DUP_US) {
		res->result = BCR_DUPLICATE;
	} else {
		res->result = BCR_VALID;
	}
	d->have_last = TRUE;
	d->last_sym = res->data[0];
	d->last_decode_us = d->t_last_us;
	(void)odo_mm;
}

EXPORT void bc_edge(BC_DEC *d, UW t_us, UB black, W speed_mms, W odo_mm,
		    BC_RESULT *res)
{
	(void)speed_mms;
	(void)odo_mm;

	clear_result(res);
	black = (black != 0U) ? 1U : 0U;
	if (d->level_known && black == d->level) {
		return;				/* lost edge pair: ignore */
	}

	if (d->state == BCS_IDLE) {
		if (black != 0U) {
			UW	quiet = d->level_known
				      ? t_us - d->t_last_us : BC_START_QUIET_US;

			if (quiet >= BC_START_QUIET_US) {
				d->lockout = FALSE;
				d->state = BCS_CAPTURING;
				d->n_edges = 0;
				d->widest_um = 0;
				d->start_odo_mm = odo_mm;
				d->edge_t[d->n_edges] = t_us;
				d->n_edges++;
			}
		}
	} else {
		UW	el = t_us - d->t_last_us;

		if (el > BC_IDLE_TIMEOUT_US) {
			fail(d, res, BC_ERR_LONG);	/* e.g. the line itself */
		} else if (d->n_edges >= BC_MAX_EDGES) {
			fail(d, res, BC_ERR_OVERFLOW);
		} else {
			if (el > d->widest_um) {
				d->widest_um = el;
			}
			d->edge_t[d->n_edges] = t_us;
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
	UW	quiet;
	UW	end_q;
	(void)speed_mms;

	clear_result(res);
	if (d->state != BCS_CAPTURING) {
		return;
	}
	quiet = now_us - d->t_last_us;
	/* A narrow first bar cannot establish the maximum space duration.
	   Observe a whole symbol before deciding that white ends the scan. */
	if (d->level == 0U && d->n_edges >= C39_ELEMENTS + 1U) {
		UW reference = d->widest_um;

		if (d->n_edges == C39_MERGED_ELEMENTS + 1U) {
			UW longest = 0, second = 0;
			UINT i;

			/* One merged run may be much longer than any real space. */
			for (i = 1U; i < d->n_edges; i++) {
				UW width = d->edge_t[i] - d->edge_t[i - 1U];
				if (width >= longest) {
					second = longest;
					longest = width;
				} else if (width > second) {
					second = width;
				}
			}
			reference = second;
		}
		end_q = reference * BC_END_QUIET_X10 / 10U;
		if (end_q < BC_END_QUIET_US) {
			end_q = BC_END_QUIET_US;
		}
		if (end_q > BC_IDLE_TIMEOUT_US) {
			end_q = BC_IDLE_TIMEOUT_US;
		}
		if (quiet >= end_q) {
			finish(d, odo_mm, res);	/* completion: trailing quiet zone */
			return;
		}
	}
	if (quiet > BC_IDLE_TIMEOUT_US) {
		fail(d, res, BC_ERR_STALLED);
	}
}
