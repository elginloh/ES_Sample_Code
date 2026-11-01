/*
 *	avoid_plan.c
 *	Avoidance planning (see avoid_plan.h).
 */

#include "avoid_plan.h"
#include "obstacle_profile.h"

LOCAL W clamp_move(W mm)
{
	if (mm > MCMD_MOVE_MAX_MM) {
		return MCMD_MOVE_MAX_MM;
	}
	return (mm < 1) ? 1 : mm;
}

LOCAL void add(AV_PLAN *plan, UB kind, W arg)
{
	if (plan->n < AV_MAX_STEPS) {
		plan->step[plan->n].kind = kind;
		plan->step[plan->n].arg = (INT)arg;
		plan->n++;
	}
}

/* Select the same bypass side for planning and reverse calculation, then
   derive the centre-line offset which gives the configured edge gap. */
LOCAL INT select_path(const S5_PROFILE *p, UB *side, W *offset)
{
	W	required = AV_REQUIRED_CORRIDOR_MM;
	W	hw = AV_ROBOT_WIDTH_MM / 2;
	W	l_offset = -p->left_mm + hw + AV_SIDE_GAP_MM + AV_EDGE_UNCERTAINTY_MM;
	W	r_offset = p->right_mm + hw + AV_SIDE_GAP_MM + AV_EDGE_UNCERTAINTY_MM;
	BOOL	l_ok;
	BOOL	r_ok;

	if (l_offset < AV_SIDE_GAP_MM) l_offset = AV_SIDE_GAP_MM;
	if (r_offset < AV_SIDE_GAP_MM) r_offset = AV_SIDE_GAP_MM;

#if AV_ASSUME_OPEN_BOARD
	/* The road is declared laterally open.  A known edge is enough to
	   calculate the requested obstacle gap; the ultrasonic scan reach or a
	   spurious side reflection must not be treated as a board boundary. */
	l_ok = (p->clear_left_mm >= 0) ? TRUE : FALSE;
	r_ok = (p->clear_right_mm >= 0) ? TRUE : FALSE;
	(void)required;
#else
	l_ok = (p->clear_left_mm >= required) ? TRUE : FALSE;
	r_ok = (p->clear_right_mm >= required) ? TRUE : FALSE;
#endif

	if (!l_ok && !r_ok) {
		return AVP_NO_CLEARANCE;
	}
	if (l_ok && r_ok) {
#if AV_ASSUME_OPEN_BOARD
		/* Side echoes are unreliable board-width measurements.  Choose the
		   route requiring less deviation; use the configured preference only
		   when the geometry is equal. */
		if (r_offset != l_offset) {
			*side = (r_offset < l_offset) ? AV_SIDE_RIGHT : AV_SIDE_LEFT;
		} else {
			*side = AV_PREFER_RIGHT ? AV_SIDE_RIGHT : AV_SIDE_LEFT;
		}
#else
		if (p->clear_right_mm != p->clear_left_mm) {
			*side = (p->clear_right_mm > p->clear_left_mm)
			      ? AV_SIDE_RIGHT : AV_SIDE_LEFT;
		} else {
			*side = AV_PREFER_RIGHT ? AV_SIDE_RIGHT : AV_SIDE_LEFT;
		}
#endif
	} else {
		*side = r_ok ? AV_SIDE_RIGHT : AV_SIDE_LEFT;
	}
	*offset = (*side == AV_SIDE_RIGHT) ? r_offset : l_offset;
	return AVP_OK;
}

/* Forward progress while travelling the lateral offset at angle deg. */
LOCAL W diagonal_forward(W offset, INT deg)
{
	W	s = op_sin_q15(deg);
	W	c = op_cos_q15(deg);

	return (W)(((D)offset * c + s - 1) / s);
}

/* Extra forward reach of the front inner corner at a particular heading,
   relative to the centred front bumper where the ultrasonic gap is measured. */
LOCAL W corner_extra(INT deg)
{
	W	hl = AV_ROBOT_LENGTH_MM / 2;
	W	hw = AV_ROBOT_WIDTH_MM / 2;
	W	projection;

	if (deg < 0) deg = -deg;
	projection = (W)(((D)hl * op_cos_q15(deg)
			 + (D)hw * op_sin_q15(deg) + 32767) / 32768);
	return (projection > hl) ? projection - hl : 0;
}

/* A turn sweeps through every intermediate heading. */
LOCAL W turn_sweep_extra(INT deg)
{
	W	max_extra = 0;
	INT	a;

	if (deg < 0) deg = -deg;
	for (a = 0; a <= deg; a++) {
		W extra = corner_extra(a);

		if (extra > max_extra) max_extra = extra;
	}
	return max_extra;
}

/* Bumper gap needed for the pivot and the diagonal departure at this angle. */
LOCAL W path_front_need(W offset, INT deg)
{
	W	pivot_need = turn_sweep_extra(deg) + AV_FRONT_PATH_ALLOWANCE_MM;
	W	move_need = diagonal_forward(offset, deg) + corner_extra(deg)
		      + AV_FRONT_PATH_ALLOWANCE_MM;
	W	need = (pivot_need > move_need) ? pivot_need : move_need;

	return (need < AV_MIN_FRONT_CLEARANCE_MM) ? AV_MIN_FRONT_CLEARANCE_MM : need;
}

/* Minimum front gap from which the maximum allowed turn can clear the face. */
LOCAL W required_front_gap(W offset)
{
	return path_front_need(offset, AV_TURN_MAX_DEG);
}

/* Body pose while returning toward the obstacle.  Coordinates are normalised
   so the obstacle is on the negative-x side and its far edge is y = 0.
   'advance' is how far the front-mounted sensor has moved beyond that edge. */
LOCAL void return_corner(W x, W y, W advance, INT deg, W *rx, W *ry)
{
	W	s = op_sin_q15(deg);
	W	c = op_cos_q15(deg);
	W	front_axle = AV_ROBOT_LENGTH_MM - AV_ROBOT_LENGTH_MM / 2
			     - AV_SENSOR_BEHIND_FRONT_MM;

	*rx = (W)(((D)x * c - (D)y * s) / 32768);
	*ry = advance - front_axle
	    + (W)(((D)x * s + (D)y * c) / 32768);
}

/* Check both the in-place return pivot and the following diagonal move.  A
   point may move inward only through the calibrated uncertainty reserve; the
   requested 10 cm obstacle gap and AV_RETURN_PATH_ALLOWANCE_MM remain intact.
   Checking the four corners plus every y=0 edge intersection is sufficient
   for the convex rectangular body. */
LOCAL BOOL return_path_safe(W advance, INT deg, W measured_side_gap_mm)
{
	W	hw = AV_ROBOT_WIDTH_MM / 2;
	W	hr = AV_ROBOT_LENGTH_MM / 2;
	W	hf = AV_ROBOT_LENGTH_MM - hr;
	W	reserve = measured_side_gap_mm - AV_SIDE_GAP_MM
			  - AV_RETURN_PATH_ALLOWANCE_MM;
	W	allowed_x;
	W	lx[4] = { -hw, hw, hw, -hw };
	W	ly[4] = { -hr, -hr, hf, hf };
	W	x[4], y[4];
	INT	a;
	UINT	i;

	if (reserve < 0) reserve = 0;
	allowed_x = -hw - reserve;
	for (a = 0; a <= deg; a++) {
		for (i = 0; i < 4U; i++) {
			return_corner(lx[i], ly[i], advance, a, &x[i], &y[i]);
			if (y[i] <= 0 && x[i] < allowed_x) return FALSE;
		}
		for (i = 0; i < 4U; i++) {
			UINT j = (i + 1U) & 3U;

			if ((y[i] < 0 && y[j] > 0) || (y[i] > 0 && y[j] < 0)) {
				W xi = x[i] + (W)(((D)(x[j] - x[i]) * -y[i])
						 / (y[j] - y[i]));

				if (xi < allowed_x) return FALSE;
			}
		}
	}

	/* Once the pivot is complete, each still-overlapping corner reaches the
	   far-edge plane while moving diagonally.  That crossing is its closest
	   lateral approach to the obstacle. */
	{
		W s = op_sin_q15(deg);
		W c = op_cos_q15(deg);

		if (c <= 0) return FALSE;
		for (i = 0; i < 4U; i++) {
			if (y[i] < 0) {
				W cross_x = x[i] + (W)(((D)y[i] * s) / c);

				if (cross_x < allowed_x) return FALSE;
			}
		}
	}
	return TRUE;
}

/* Minimum straight advance after the front sensor reaches the far edge.  The
   old approximation always returned almost the full 215 mm body length.  This
   search uses the actual body corners and normally permits the safe inward
   return after roughly 0.4 body lengths. */
EXPORT W av_return_approach_mm(INT deg, W measured_side_gap_mm)
{
	W	body_behind_sensor = AV_ROBOT_LENGTH_MM - AV_SENSOR_BEHIND_FRONT_MM;
	W	advance;

	for (advance = 0; advance <= body_behind_sensor; advance++) {
		if (return_path_safe(advance, deg, measured_side_gap_mm)) return advance;
	}
	return body_behind_sensor;
}

EXPORT W av_reverse_needed_mm(const S5_PROFILE *p)
{
	UB	side;
	W	offset, front_gap, target, need;

	if (p->status != PRF_VALID || p->dist_mm < 0) {
		return -1;
	}
	front_gap = p->dist_mm - AV_SENSOR_BEHIND_FRONT_MM;
	if (front_gap < 0) {
		front_gap = 0;
	}
	/* Reverse only when the scan already proves an eventual bypass side and
	   its measured edge produces a bounded lateral path. */
	if (select_path(p, &side, &offset) != AVP_OK) {
		return -1;
	}
	(void)side;
	target = required_front_gap(offset);
	if (front_gap >= target) {
		return 0;
	}
	target += AV_REVERSE_RESCAN_MARGIN_MM;
	need = target - front_gap;
	return (need > 0 && need <= AV_REVERSE_MAX_MM) ? need : -1;
}

EXPORT INT av_plan(const S5_PROFILE *p, AV_PLAN *plan)
{
	W	sin_return;
	W	s, d1, d_return, front_required, remaining, progress;
	INT	pr, turn, return_turn;

	plan->side = AV_SIDE_NONE;
	plan->n = 0;
	plan->offset_mm = 0;
	plan->required_mm = AV_REQUIRED_CORRIDOR_MM;
	plan->front_gap_mm = 0;
	plan->side_gap_mm = 0;
	plan->approach_mm = 0;
	plan->turn_deg = 0;
	plan->required_front_mm = 0;
	plan->forward_before_pass_mm = 0;
	plan->return_approach_mm = 0;
	plan->return_turn_deg = 0;

	if (p->status != PRF_VALID || p->dist_mm < 0) {
		return AVP_NO_PROFILE;
	}
	plan->front_gap_mm = p->dist_mm - AV_SENSOR_BEHIND_FRONT_MM;
	if (plan->front_gap_mm < 0) {
		plan->front_gap_mm = 0;
	}
	pr = select_path(p, &plan->side, &plan->offset_mm);
	if (pr != AVP_OK) {
		return pr;
	}
	s = (plan->side == AV_SIDE_RIGHT) ? 1 : -1;

	plan->side_gap_mm = plan->offset_mm
			  - ((s > 0) ? p->right_mm : -p->left_mm)
			  - AV_ROBOT_WIDTH_MM / 2;
	front_required = required_front_gap(plan->offset_mm);
	plan->required_front_mm = front_required;
	if (plan->front_gap_mm < front_required) {
		return AVP_NO_CLEARANCE;
	}

	/* Use all measured spare front clearance before turning.  Then choose the
	   smallest integer angle whose diagonal travel and real front-corner sweep
	   remain behind the measured obstacle face. */
	plan->approach_mm = plan->front_gap_mm - front_required;
	remaining = plan->front_gap_mm - plan->approach_mm;
	turn = AV_TURN_MIN_DEG;
	while (turn <= AV_TURN_MAX_DEG
	       && path_front_need(plan->offset_mm, turn) > remaining) {
		turn++;
	}
	if (turn > AV_TURN_MAX_DEG) {
		return AVP_NO_CLEARANCE;
	}
	plan->turn_deg = turn;
	progress = diagonal_forward(plan->offset_mm, turn);
	plan->forward_before_pass_mm = plan->approach_mm + progress;
	d1 = (W)(((D)plan->offset_mm * 32768
		  + op_sin_q15(turn) - 1) / op_sin_q15(turn));
	return_turn = AV_RETURN_TURN_MIN_DEG;
	for (;;) {
		if (return_turn > AV_RETURN_TURN_MAX_DEG) {
			return AVP_NO_CLEARANCE;
		}
		sin_return = op_sin_q15(return_turn);
		d_return = (W)(((D)plan->offset_mm * 32768 + sin_return - 1)
			 / sin_return);
		if (d_return + AV_SEARCH_EXTRA_MM <= MCMD_MOVE_MAX_MM) {
			break;
		}
		return_turn++;
	}
	plan->return_turn_deg = return_turn;
	/* The profile edge allowance cannot also be spent on an early return.
	   Keep a conservative fallback in the plan; PASS replaces it with the
	   live side-ultrasonic gap through av_return_approach_mm(). */
	plan->return_approach_mm = AV_ROBOT_LENGTH_MM - AV_SENSOR_BEHIND_FRONT_MM;
	/* These are actuator/command limits, not assumptions about board size. */
	if (d1 > MCMD_MOVE_MAX_MM || d_return + AV_SEARCH_EXTRA_MM > MCMD_MOVE_MAX_MM
	    || plan->approach_mm > MCMD_MOVE_MAX_MM) {
		return AVP_NO_CLEARANCE;
	}
	if (plan->approach_mm > 0) {
		add(plan, AVS_MOVE, plan->approach_mm);		/* use front space */
	}
	add(plan, AVS_TURN, s * turn);				/* calculated departure */
	add(plan, AVS_MOVE, clamp_move(d1));
	add(plan, AVS_TURN, -s * turn);				/* parallel again */
	add(plan, AVS_PASS, AV_SIDE_PASS_MAX_MM);		/* find the far edge */
	add(plan, AVS_TURN, -s * return_turn);			/* calculated return */
	add(plan, AVS_SEARCH, clamp_move(d_return + AV_SEARCH_EXTRA_MM));
	add(plan, AVS_TURN, s * return_turn);			/* align with it */
	return AVP_OK;
}
