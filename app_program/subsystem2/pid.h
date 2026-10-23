/*
 *	pid.h
 *	Integer PID speed controller (pure logic, host-testable).
 *
 *	Internal precision is deliberately NOT reduced by the minimum-bit
 *	requirement: errors are mm/s in 32-bit integers and gains are Q8
 *	fixed point (gain * 256), so control stability is unaffected.  Only
 *	stored/transmitted application data is packed (Change Requirement 1
 *	applies to application data, not controller state).
 */

#ifndef PID_H
#define PID_H

#include <tk/typedef.h>

typedef struct {
	W	kp_q8;		/* output units per (mm/s) error, *256        */
	W	ki_q8;		/* per (mm/s * cycle) of accumulated error    */
	W	kd_q8;		/* per (mm/s) change of measurement per cycle */
	W	i_acc;		/* integral accumulator, (mm/s) * cycles      */
	W	i_lim;		/* |i_acc| limit (anti-windup)                */
	W	prev_meas;	/* derivative on measurement (no kick)        */
	W	out_min;	/* output clamp, permille                     */
	W	out_max;
	BOOL	primed;		/* prev_meas valid                            */
} PID;

IMPORT void pid_init(PID *p, W kp_q8, W ki_q8, W kd_q8, W i_lim,
		     W out_min, W out_max);
IMPORT void pid_reset(PID *p);

/* One control step.  ff is a feed-forward term in output units.
   Returns the clamped output (permille duty). */
IMPORT W pid_update(PID *p, W target, W measured, W ff);

#endif /* PID_H */
