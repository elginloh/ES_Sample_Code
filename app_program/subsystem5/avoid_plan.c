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

EXPORT INT av_plan(const S5_PROFILE *p, AV_PLAN *plan)
{
	W	hw = AV_ROBOT_WIDTH_MM / 2;
	W	sin_t = op_sin_q15(AV_TURN_DEG);
	W	cos_t = op_cos_q15(AV_TURN_DEG);
	BOOL	l_ok, r_ok;
	W	s, d1, prog1, pass_y, d2;

	plan->side = AV_SIDE_NONE;
	plan->n = 0;
	plan->offset_mm = 0;
	plan->required_mm = AV_ROBOT_WIDTH_MM + 2 * AV_MARGIN_MM;

	if (p->status != PRF_VALID || p->dist_mm < 0) {
		return AVP_NO_PROFILE;
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
	plan->offset_mm = (s > 0) ? (p->right_mm + hw + AV_MARGIN_MM)
				  : (-p->left_mm + hw + AV_MARGIN_MM);
	if (plan->offset_mm < AV_MARGIN_MM) {
		plan->offset_mm = AV_MARGIN_MM;
	}
	if (p->dist_mm < AV_MIN_PIVOT_CLEARANCE_MM) {
		plan->side = AV_SIDE_NONE;	/* too close to pivot: stop */
		return AVP_NO_CLEARANCE;
	}
	d1 = (W)(((D)plan->offset_mm * 32768) / sin_t);
	prog1 = (W)(((D)plan->offset_mm * cos_t) / sin_t);
	pass_y = p->dist_mm + AV_OBSTACLE_DEPTH_MM + AV_ROBOT_LENGTH_MM
	       + AV_MARGIN_MM;
	d2 = pass_y - prog1;
	if (d2 < AV_MARGIN_MM) {
		d2 = AV_MARGIN_MM;
	}

	add(plan, AVS_TURN, s * AV_TURN_DEG);			/* leave the line */
	add(plan, AVS_MOVE, clamp_move(d1));
	add(plan, AVS_TURN, -s * AV_TURN_DEG);			/* parallel again */
	add(plan, AVS_MOVE, clamp_move(d2));			/* pass the obstacle */
	add(plan, AVS_TURN, -s * AV_TURN_DEG);			/* head for the line */
	add(plan, AVS_SEARCH, clamp_move(d1 * 3 / 2 + AV_SEARCH_EXTRA_MM));
	add(plan, AVS_TURN, s * AV_TURN_DEG);			/* align with it */
	return AVP_OK;
}
