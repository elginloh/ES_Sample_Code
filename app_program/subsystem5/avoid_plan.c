/*
 *	avoid_plan.c
 *	Avoidance planning (see avoid_plan.h).
 */

#include "avoid_plan.h"
#include "obstacle_profile.h"
#include "../subsystem2/motion_msg.h"

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

EXPORT W av_reverse_needed_mm(const S5_PROFILE *p)
{
	W	required = AV_ROBOT_WIDTH_MM + 2 * AV_SIDE_GAP_MM;
	W	front_gap;
	W	need;

	if (p->status != PRF_VALID || p->dist_mm < 0) {
		return -1;
	}
	front_gap = p->dist_mm - AV_SENSOR_BEHIND_FRONT_MM;
	if (front_gap < 0) {
		front_gap = 0;
	}
	if (front_gap >= AV_MIN_FRONT_CLEARANCE_MM) {
		return 0;
	}
	/* Reverse only when the scan already proves an eventual bypass side. */
	if (p->clear_left_mm < required && p->clear_right_mm < required) {
		return -1;
	}
	need = AV_REVERSE_TARGET_GAP_MM - front_gap;
	return (need > 0 && need <= AV_REVERSE_MAX_MM) ? need : -1;
}

EXPORT INT av_plan(const S5_PROFILE *p, AV_PLAN *plan)
{
	W	hw = AV_ROBOT_WIDTH_MM / 2;
	W	sin_t = op_sin_q15(AV_TURN_DEG);
	W	sin_return = op_sin_q15(AV_RETURN_TURN_DEG);
	BOOL	l_ok, r_ok;
	W	s, d1, d_return;

	plan->side = AV_SIDE_NONE;
	plan->n = 0;
	plan->offset_mm = 0;
	plan->required_mm = AV_ROBOT_WIDTH_MM + 2 * AV_SIDE_GAP_MM;
	plan->front_gap_mm = 0;
	plan->side_gap_mm = 0;

	if (p->status != PRF_VALID || p->dist_mm < 0) {
		return AVP_NO_PROFILE;
	}
	plan->front_gap_mm = p->dist_mm - AV_SENSOR_BEHIND_FRONT_MM;
	if (plan->front_gap_mm < 0) {
		plan->front_gap_mm = 0;
	}
	l_ok = (p->clear_left_mm >= plan->required_mm) ? TRUE : FALSE;
	r_ok = (p->clear_right_mm >= plan->required_mm) ? TRUE : FALSE;
	if (!l_ok && !r_ok) {
		return AVP_NO_CLEARANCE;
	}
	if (l_ok && r_ok) {
		if (p->clear_right_mm != p->clear_left_mm) {
			plan->side = (p->clear_right_mm > p->clear_left_mm)
				   ? AV_SIDE_RIGHT : AV_SIDE_LEFT;
		} else {
			plan->side = AV_PREFER_RIGHT ? AV_SIDE_RIGHT : AV_SIDE_LEFT;
		}
	} else {
		plan->side = r_ok ? AV_SIDE_RIGHT : AV_SIDE_LEFT;
	}
	s = (plan->side == AV_SIDE_RIGHT) ? 1 : -1;

	/* lateral offset so the near side of the car clears the edge */
	plan->offset_mm = (s > 0) ? (p->right_mm + hw + AV_SIDE_GAP_MM)
				  : (-p->left_mm + hw + AV_SIDE_GAP_MM);
	if (plan->offset_mm < AV_SIDE_GAP_MM) {
		plan->offset_mm = AV_SIDE_GAP_MM;
	}
	plan->side_gap_mm = plan->offset_mm
			  - ((s > 0) ? p->right_mm : -p->left_mm) - hw;
	if (plan->offset_mm > AV_MAX_LATERAL_OFFSET_MM) {
		plan->side = AV_SIDE_NONE;	/* never drive out on a noisy width */
		return AVP_NO_CLEARANCE;
	}
	if (plan->front_gap_mm < AV_MIN_FRONT_CLEARANCE_MM) {
		plan->side = AV_SIDE_NONE;	/* too close to pivot: stop */
		return AVP_NO_CLEARANCE;
	}
	d1 = (W)(((D)plan->offset_mm * 32768) / sin_t);
	d_return = (W)(((D)plan->offset_mm * 32768) / sin_return);
	add(plan, AVS_TURN, s * AV_TURN_DEG);			/* leave the line */
	add(plan, AVS_MOVE, clamp_move(d1));
	add(plan, AVS_TURN, -s * AV_TURN_DEG);			/* parallel again */
	add(plan, AVS_PASS, AV_SIDE_PASS_MAX_MM);		/* find the far edge */
	add(plan, AVS_TURN, -s * AV_RETURN_TURN_DEG);		/* shallow return */
	add(plan, AVS_SEARCH, clamp_move(d_return + AV_SEARCH_EXTRA_MM));
	add(plan, AVS_TURN, s * AV_RETURN_TURN_DEG);		/* align with it */
	return AVP_OK;
}
