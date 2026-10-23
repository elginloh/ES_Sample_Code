/*
 *	obstacle_profile.h
 *	Obstacle profile generation (Buddy 5, Obstacle Profile task): convert
 *	servo-angle / distance samples into obstacle position, width and the
 *	free lateral clearance on each side.  Pure logic, host-testable.
 *
 *	Project Briefing p.8: "Obstacle Width W ~ f(dL, dC, dR, theta)
 *	(estimated using distance geometry & scan angle)".  Each valid sample
 *	becomes a point (x = d sin a lateral, y = d cos a forward).  Samples
 *	within DEPTH_TOL of the nearest forward return belong to the obstacle;
 *	its edges lie between the outermost "hit" and the first "miss" on each
 *	side (the fine scan narrows that gap to the fine step).  A sweep that
 *	reaches the scan limit while still on the obstacle cannot establish
 *	that side's edge: that side's clearance is UNKNOWN and is never
 *	treated as free (the brief: never invent a bypass).
 *
 *	Tribunal 1: "Assuming ultrasonic scanning determines exact obstacle
 *	geometry" was rejected as unrealistic -- the HC-SR04 beam widens the
 *	apparent obstacle; US_BEAM_COMP_DEG lets the team subtract a measured
 *	beam allowance (0 = conservative: the obstacle looks wider).
 */

#ifndef OBSTACLE_PROFILE_H
#define OBSTACLE_PROFILE_H

#include <tk/typedef.h>
#include "s5_msg.h"

#define OP_MAX_SAMPLES		64
#define OP_FORWARD_CONE_DEG	30	/* nearest face searched in +-30 deg */
#define OP_DETECT_RANGE_MM	700	/* farther = no obstacle to avoid    */
#define OP_DEPTH_TOL_MM		150	/* returns this close to the nearest
					   face belong to the same obstacle */
#define OP_NEIGHBOUR_DEPTH_MM	500	/* other objects that limit clearance */
#define OP_MIN_HITS		2
#define OP_MIN_VALID		5
#define OP_FROZEN_SPREAD_MM	10	/* identical readings everywhere ...  */
#define OP_FROZEN_SPAN_DEG	60	/* ... over this sweep = servo stuck  */
#define OP_CLEAR_CAP_MM		2540
#ifndef US_BEAM_COMP_DEG
#define US_BEAM_COMP_DEG	0	/* [CAL] beam half-width allowance   */
#endif

/* One scan sample as the Profile task keeps it, at the widths its ranges
   need (servo +-80 deg, HC-SR04 0..4000 mm): 4 bytes, where an INT / W /
   UB layout takes 12 (Change Requirement 1; 64 samples = 256 bytes). */
typedef struct {
	B	angle_deg;	/* -80..80, + = right */
	UB	status;		/* SS_* */
	H	dist_mm;	/* 0..4000; -1 when status != SS_OK */
} OP_SAMPLE;

/* samples need not be sorted; scan_limit_deg = largest |angle| scanned */
IMPORT void op_build(const OP_SAMPLE *s, UINT n, INT scan_limit_deg,
		     S5_PROFILE *out);

IMPORT W    op_sin_q15(INT deg);	/* integer sine, Q15 */
IMPORT W    op_cos_q15(INT deg);
IMPORT INT  op_atan_deg(W y, W x);	/* bearing of lateral y at forward x, deg */

#endif /* OBSTACLE_PROFILE_H */
