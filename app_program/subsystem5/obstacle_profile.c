/*
 *	obstacle_profile.c
 *	Obstacle profile generation (see obstacle_profile.h).
 */

#include "obstacle_profile.h"

LOCAL W iabs(W v)
{
	return (v < 0) ? -v : v;
}

/* Bhaskara I approximation, integer degrees, |error| < 0.002 */
EXPORT W op_sin_q15(INT deg)
{
	W	x = deg % 360;
	W	sign = 1;
	D	p;

	if (x < 0) {
		x += 360;
	}
	if (x > 180) {
		x -= 180;
		sign = -1;
	}
	p = (D)x * (180 - x);
	return sign * (W)(((D)4 * p * 32768) / ((D)40500 - p));
}

EXPORT W op_cos_q15(INT deg)
{
	return op_sin_q15(90 - deg);
}

/* atan(y / x) in whole degrees, x > 0: (pi/4) z + 0.273 z (1 - z) */
EXPORT INT op_atan_deg(W y, W x)
{
	W	ay = iabs(y);
	W	z, a;

	if (x <= 0) {
		return (y < 0) ? -90 : 90;
	}
	if (ay <= x) {
		z = (W)(((D)ay << 15) / x);
		a = (W)(((D)45 * z + (D)1564 * z / 100 * (32768 - z) / 32768) / 32768);
	} else {
		z = (W)(((D)x << 15) / ay);
		a = 90 - (W)(((D)45 * z + (D)1564 * z / 100 * (32768 - z) / 32768) / 32768);
	}
	return (INT)((y < 0) ? -a : a);
}

LOCAL W lateral(W d, INT deg)
{
	return (W)(((D)d * op_sin_q15(deg)) / 32768);
}

LOCAL W forward(W d, INT deg)
{
	return (W)(((D)d * op_cos_q15(deg)) / 32768);
}

/* Sort a copy by angle (insertion sort, n <= OP_MAX_SAMPLES) */
LOCAL UINT sorted_copy(const OP_SAMPLE *s, UINT n, OP_SAMPLE *o)
{
	UINT	i, j;

	if (n > OP_MAX_SAMPLES) {
		n = OP_MAX_SAMPLES;
	}
	for (i = 0; i < n; i++) {
		OP_SAMPLE	t = s[i];

		for (j = i; j > 0U && o[j - 1U].angle_deg > t.angle_deg; j--) {
			o[j] = o[j - 1U];
		}
		o[j] = t;
	}
	return n;
}

LOCAL void init_out(S5_PROFILE *out, UB status)
{
	out->status = status;
	out->dist_mm = -1;
	out->bearing_deg = 0;
	out->width_mm = -1;
	out->left_mm = 0;
	out->right_mm = 0;
	out->clear_left_mm = -1;
	out->clear_right_mm = -1;
}

/* Frozen servo: every valid reading identical across a wide sweep */
LOCAL BOOL looks_frozen(const OP_SAMPLE *o, UINT n)
{
	W	dmin = 0x7FFFFFFF, dmax = -1;
	INT	amin = 1000, amax = -1000;
	UINT	i, valid = 0;

	for (i = 0; i < n; i++) {
		if (o[i].status != SS_OK) {
			continue;
		}
		valid++;
		if (o[i].dist_mm < dmin) dmin = o[i].dist_mm;
		if (o[i].dist_mm > dmax) dmax = o[i].dist_mm;
		if (o[i].angle_deg < amin) amin = o[i].angle_deg;
		if (o[i].angle_deg > amax) amax = o[i].angle_deg;
	}
	return (valid >= OP_MIN_VALID && amax - amin >= OP_FROZEN_SPAN_DEG
		&& dmax - dmin <= OP_FROZEN_SPREAD_MM) ? TRUE : FALSE;
}

/* Lateral position, on the obstacle's front plane y, of bearing deg */
LOCAL W planar(W y, INT deg)
{
	W	c = op_cos_q15(deg);

	return (c > 0) ? (W)(((D)y * op_sin_q15(deg)) / c) : 0;
}

/* Clearance on one side: from the obstacle edge to the nearest other
   object on that side, or to the lateral reach of the scan at the
   obstacle's plane if nothing else was seen. */
LOCAL W side_clearance(const OP_SAMPLE *o, const H *y, UINT n, INT from, INT step,
		       W edge_x, W y_near, INT scan_limit_deg, BOOL edge_known)
{
	W	best = -1;
	W	reach;
	INT	i;

	if (!edge_known) {
		return -1;			/* unknown, never "free" */
	}
	for (i = from; i >= 0 && i < (INT)n; i += step) {
		W	x;

		if (o[i].status != SS_OK || y[i] > y_near + OP_NEIGHBOUR_DEPTH_MM) {
			continue;
		}
		x = lateral(o[i].dist_mm, o[i].angle_deg);
		if (best < 0 || iabs(x - edge_x) < best) {
			best = iabs(x - edge_x);
		}
	}
	reach = iabs(planar(y_near, (step > 0) ? scan_limit_deg : -scan_limit_deg) - edge_x);
	if (best < 0 || best > reach) {
		best = reach;			/* free as far as the scan can see */
	}
	return (best > OP_CLEAR_CAP_MM) ? OP_CLEAR_CAP_MM : best;
}

EXPORT void op_build(const OP_SAMPLE *s, UINT n, INT scan_limit_deg,
		     S5_PROFILE *out)
{
	OP_SAMPLE	o[OP_MAX_SAMPLES];
	H		y[OP_MAX_SAMPLES];	/* forward distance of each echo, mm */
	UINT		i, readings = 0, hits = 0;
	INT		near_i = -1, li, ri, k;
	W		y_near = 0x7FFFFFFF;
	INT		edge_l, edge_r;
	BOOL		l_known, r_known;

	n = sorted_copy(s, n, o);
	init_out(out, PRF_INVALID);

	for (i = 0; i < n; i++) {
		y[i] = -1;
		if (o[i].status == SS_NO_ECHO) {
			readings++;			/* valid: nothing within 4 m */
		}
		if (o[i].status != SS_OK) {
			continue;
		}
		readings++;
		y[i] = (H)forward(o[i].dist_mm, o[i].angle_deg);	/* <= 4000 */
		if (iabs(o[i].angle_deg) <= OP_FORWARD_CONE_DEG && y[i] < y_near) {
			y_near = y[i];
			near_i = (INT)i;
		}
	}
	if (readings < OP_MIN_VALID) {
		return;					/* PRF_INVALID: sensor data missing */
	}
	if (looks_frozen(o, n)) {
		return;					/* PRF_INVALID: servo suspect */
	}
	if (near_i < 0 || y_near > OP_DETECT_RANGE_MM) {
		init_out(out, PRF_NO_OBSTACLE);		/* path is clear */
		return;
	}

	/* An echo belongs to the obstacle when it lies on (or just behind)
	   the nearest front plane: forward distance, not range, so a flat
	   face or a wall is recognised as one surface. */
#define HIT(j)	(o[(j)].status == SS_OK && y[(j)] <= y_near + OP_DEPTH_TOL_MM)
	li = near_i;
	while (li - 1 >= 0 && (HIT(li - 1) || o[li - 1].status == SS_INVALID)) {
		li--;
	}
	ri = near_i;
	while (ri + 1 < (INT)n && (HIT(ri + 1) || o[ri + 1].status == SS_INVALID)) {
		ri++;
	}
	while (li < near_i && !HIT(li)) {
		li++;				/* do not end the region on a gap */
	}
	while (ri > near_i && !HIT(ri)) {
		ri--;
	}
	for (k = li; k <= ri; k++) {
		if (HIT(k)) {
			hits++;
		}
	}
#undef HIT
	if (hits < OP_MIN_HITS) {
		return;					/* PRF_INVALID: one stray echo */
	}

	/* edge = midway between the last hit and the first miss */
	l_known = (li > 0) ? TRUE : FALSE;
	r_known = (ri < (INT)n - 1) ? TRUE : FALSE;
	edge_l = l_known ? (o[li].angle_deg + o[li - 1].angle_deg) / 2 : o[li].angle_deg;
	edge_r = r_known ? (o[ri].angle_deg + o[ri + 1].angle_deg) / 2 : o[ri].angle_deg;
	if (US_BEAM_COMP_DEG > 0 && edge_r - edge_l > 2 * US_BEAM_COMP_DEG) {
		edge_l += US_BEAM_COMP_DEG;
		edge_r -= US_BEAM_COMP_DEG;
	}

	out->dist_mm = y_near;
	out->left_mm = planar(y_near, edge_l);
	out->right_mm = planar(y_near, edge_r);
	out->width_mm = out->right_mm - out->left_mm;
	if (out->width_mm < 0) {
		out->width_mm = 0;
	}
	out->bearing_deg = (edge_l + edge_r) / 2;
	out->clear_left_mm = side_clearance(o, y, n, li - 1, -1, out->left_mm,
					    y_near, scan_limit_deg, l_known);
	out->clear_right_mm = side_clearance(o, y, n, ri + 1, 1, out->right_mm,
					     y_near, scan_limit_deg, r_known);
	out->status = (l_known || r_known) ? PRF_VALID : PRF_INSUFFICIENT;
}
