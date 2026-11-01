/*
 *	avoid_plan.h
 *	Avoidance planning (Buddy 5, Avoidance & Recovery).  Pure logic.
 *
 *	Project Briefing p.8 workflow: detect -> measure width -> choose a
 *	bypass direction -> go off line &
 *	bypass -> search & reacquire the line behind the obstacle -> return
 *	back to the line.  If there is not enough room to pivot in front of
 *	the obstacle, planning fails and the vehicle enters Safe Stop.
 *
 *	With AV_ASSUME_OPEN_BOARD, a side is usable when that obstacle edge is
 *	known; the shorter measured-obstacle route is chosen and scan reach is not
 *	treated as a board boundary.  With it disabled, clearance must fit the car,
 *	configured gap and measurement allowance.  If neither side qualifies the plan fails
 *	(FR12 -> Safe Stop): no bypass is ever invented.
 *
 *	Plan for side s (+1 right, -1 left): optionally move forward, calculate
 *	a safe outward angle T from measured front clearance and lateral offset,
 *	then calculate a shallow return so forward and inward motion happen together:
 *	    [MOVE approach] TURN s*T MOVE d1 TURN -s*T PASS TURN -s*R
 *	    SEARCH (move until the line is found, at most dS) TURN s*R
 */

#ifndef AVOID_PLAN_H
#define AVOID_PLAN_H

#include <tk/typedef.h>
#include "s5_msg.h"
#include "../subsystem2/motion_msg.h"

/* Vehicle geometry and clearances [CAL]: measured on the actual car */
#define AV_ROBOT_WIDTH_MM	152
#define AV_ROBOT_LENGTH_MM	215
#define AV_SENSOR_BEHIND_FRONT_MM 0	/* sensor is directly above front bumper */
#define AV_SIDE_GAP_MM		100	/* requested closest side gap (10 cm) */
#define AV_EDGE_UNCERTAINTY_MM	25	/* [CAL] scan/turn/odometry allowance */
#define AV_ASSUME_OPEN_BOARD	1	/* road has no known lateral boundary */
#define AV_REQUIRED_CORRIDOR_MM	(AV_ROBOT_WIDTH_MM + AV_SIDE_GAP_MM \
				 + AV_EDGE_UNCERTAINTY_MM)
#define AV_TURN_MIN_DEG		30	/* calculated outward turn bounds */
#define AV_TURN_MAX_DEG		75	/* steeper departure reduces reverse distance */
#define AV_FRONT_PATH_ALLOWANCE_MM 10	/* [CAL] turn/rounding allowance */
#define AV_RETURN_TURN_MIN_DEG	30	/* encoder-reliable shallow return */
#define AV_RETURN_TURN_MAX_DEG	30	/* do not cut sharply toward obstacle */
#define AV_RETURN_PATH_ALLOWANCE_MM 5	/* retain margin during return */
#define AV_DETECT_FRONT_GAP_MM	150	/* stop near 15 cm before planning */
#define AV_MIN_FRONT_CLEARANCE_MM	120	/* closest calculated turn-start gap */
#define AV_EMERGENCY_FRONT_CLEARANCE_MM	100	/* bumper gap while moving */
#define AV_REVERSE_MAX_MM	150	/* bounded retreat on travelled path */
#define AV_REVERSE_RESCAN_MARGIN_MM 20	/* absorb profile/odometry variation */
#define AV_SIDE_PASS_MAX_MM	MCMD_MOVE_MAX_MM /* actuator safety bound, not obstacle length */
#define AV_SEARCH_EXTRA_MM	150
#define AV_LINE_SENSOR_AHEAD_MM	80	/* [CAL] centre IR sensor to wheel axle */
#define AV_PREFER_RIGHT		1	/* tie-break when clearances equal */
#define AV_MAX_STEPS		8

#define AV_SIDE_NONE		0U
#define AV_SIDE_LEFT		1U
#define AV_SIDE_RIGHT		2U

#define AVS_TURN		0U
#define AVS_MOVE		1U
#define AVS_SEARCH		2U	/* move forward until the line is found */
#define AVS_PASS		3U	/* side-ultrasonic terminated move */

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
	W	front_gap_mm;	/* obstacle gap from front bumper */
	W	side_gap_mm;	/* predicted closest side gap */
	W	approach_mm;	/* forward move before outward turn */
	W	turn_deg;	/* calculated outward turn, 30..75 */
	W	required_front_mm; /* geometry needed at maximum turn */
	W	forward_before_pass_mm; /* approach + diagonal forward progress */
	W	return_approach_mm; /* conservative fallback before live side range */
	W	return_turn_deg;	/* configured encoder-reliable return */
} AV_PLAN;

IMPORT INT av_plan(const S5_PROFILE *p, AV_PLAN *plan);
/* -1 = reversing is unsafe/unhelpful, 0 = not needed, positive = reverse mm */
IMPORT W   av_reverse_needed_mm(const S5_PROFILE *p);
/* Safe straight distance after the sensor reaches the far edge, using the
   live obstacle-to-body side gap measured during PASS. */
IMPORT W   av_return_approach_mm(INT deg, W measured_side_gap_mm);

#endif /* AVOID_PLAN_H */
