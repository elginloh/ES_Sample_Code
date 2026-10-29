/*
 *	avoid_plan.h
 *	Avoidance planning (Buddy 5, Avoidance & Recovery).  Pure logic.
 *
 *	Project Briefing p.8 workflow: detect -> measure width -> "choose
 *	direction: pick the side with more clearance" -> go off line &
 *	bypass -> search & reacquire the line behind the obstacle -> collect
 *	back to the line.  Plus "plan stop, turn or reverse actions": the
 *	plan reverses first when the obstacle is too close to pivot.
 *
 *	A side is usable only if its clearance is KNOWN and at least the car
 *	width plus two margins.  If neither side qualifies the plan fails
 *	(FR12 -> Safe Stop): no bypass is ever invented.
 *
 *	Plan for side s (+1 right, -1 left), turn angle T (45 deg):
 *	    [MOVE -r]  TURN s*T  MOVE d1  TURN -s*T  MOVE d2  TURN -s*T
 *	    SEARCH (move until the line is found, at most dS)  TURN s*T
 *	d1 = lateral offset / sin T; d2 brings the whole car past the far
 *	face (assumed depth: the ultrasonic cannot see behind the face).
 */

#ifndef AVOID_PLAN_H
#define AVOID_PLAN_H

#include <tk/typedef.h>
#include "s5_msg.h"

/* Vehicle geometry and margins [CAL]: measure the actual car */
#define AV_ROBOT_WIDTH_MM	150
#define AV_ROBOT_LENGTH_MM	200
#define AV_MARGIN_MM		60
#define AV_TURN_DEG		45
#define AV_MIN_FRONT_MM		180	/* room in front needed to pivot   */
#define AV_ALLOW_REVERSE	0	/* 0 = never back away: closer than
					   AV_MIN_FRONT_MM there is no plan
					   (AVP_NO_CLEARANCE, Safe Stop); 1 =
					   reverse first to make room */
#define AV_OBSTACLE_DEPTH_MM	200	/* [CAL] assumed obstacle depth    */
#define AV_SEARCH_EXTRA_MM	150
#define AV_PREFER_RIGHT		1	/* tie-break when clearances equal */
#define AV_MAX_STEPS		8

#define AV_SIDE_NONE		0U
#define AV_SIDE_LEFT		1U
#define AV_SIDE_RIGHT		2U

#define AVS_TURN		0U
#define AVS_MOVE		1U
#define AVS_SEARCH		2U	/* move forward until the line is found */

#define AVP_OK			0
#define AVP_NO_PROFILE		(-1)
#define AVP_NO_CLEARANCE	(-2)

typedef struct {
	UB	kind;		/* AVS_* */
	INT	arg;		/* deg (TURN) or mm (MOVE/SEARCH) */
} AV_STEP;

typedef struct {
	UB	side;		/* AV_SIDE_* */
	UB	n;
	AV_STEP	step[AV_MAX_STEPS];
	W	offset_mm;	/* lateral offset of the bypass */
	W	required_mm;	/* clearance the car needs */
} AV_PLAN;

IMPORT INT av_plan(const S5_PROFILE *p, AV_PLAN *plan);

#endif /* AVOID_PLAN_H */
