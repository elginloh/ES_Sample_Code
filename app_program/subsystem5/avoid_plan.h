/*
 *	avoid_plan.h
 *	Avoidance planning (Buddy 5, Avoidance & Recovery).  Pure logic.
 *
 *	Project Briefing p.8 workflow: detect -> measure width -> "choose
 *	direction: pick the side with more clearance" -> go off line &
 *	bypass -> search & reacquire the line behind the obstacle -> collect
 *	back to the line.  If there is not enough room to pivot in front of
 *	the obstacle, planning fails and the vehicle enters Safe Stop.
 *
 *	A side is usable only if its clearance is KNOWN and at least the car
 *	width plus two configured side gaps.  If neither side qualifies the plan fails
 *	(FR12 -> Safe Stop): no bypass is ever invented.
 *
 *	Plan for side s (+1 right, -1 left): leave at 45 deg, PASS parallel,
 *	then return at 30 deg so forward and inward motion happen together:
 *	    TURN s*45  MOVE d1  TURN -s*45  PASS  TURN -s*30
 *	    SEARCH (move until the line is found, at most dS)  TURN s*30
 */

#ifndef AVOID_PLAN_H
#define AVOID_PLAN_H

#include <tk/typedef.h>
#include "s5_msg.h"

/* Vehicle geometry and clearances [CAL]: measured on the actual car */
#define AV_ROBOT_WIDTH_MM	152
#define AV_ROBOT_LENGTH_MM	215
#define AV_SENSOR_BEHIND_FRONT_MM 0	/* sensor is directly above front bumper */
#define AV_SIDE_GAP_MM		100	/* requested closest side gap (10 cm) */
#define AV_MAX_LATERAL_OFFSET_MM 320	/* permits 10 cm gap; rejects extreme drift */
#define AV_TURN_DEG		45
#define AV_RETURN_TURN_DEG	30	/* earlier, shallower return to line */
#define AV_MIN_FRONT_CLEARANCE_MM	200	/* requested stopped front gap (20 cm) */
#define AV_EMERGENCY_FRONT_CLEARANCE_MM	100	/* bumper gap while moving */
#define AV_REVERSE_TARGET_GAP_MM	220	/* rescan with 20 mm pivot margin */
#define AV_REVERSE_MAX_MM	150	/* bounded retreat on travelled path */
#define AV_SIDE_PASS_MAX_MM	1000	/* bounded search for far edge     */
#define AV_RETURN_APPROACH_MM	160	/* after edge loss, then start 30-deg return */
#define AV_SEARCH_EXTRA_MM	150
#define AV_LINE_SENSOR_AHEAD_MM	80	/* [CAL] centre IR sensor to wheel axle */
#define AV_PREFER_RIGHT		1	/* tie-break when clearances equal */
#define AV_MAX_STEPS		7

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
} AV_PLAN;

IMPORT INT av_plan(const S5_PROFILE *p, AV_PLAN *plan);
/* -1 = reversing is unsafe/unhelpful, 0 = not needed, positive = reverse mm */
IMPORT W   av_reverse_needed_mm(const S5_PROFILE *p);

#endif /* AVOID_PLAN_H */
