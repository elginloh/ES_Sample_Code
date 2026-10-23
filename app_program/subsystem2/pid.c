/*
 *	pid.c
 *	Integer PID with feed-forward, derivative on measurement and
 *	conditional-integration anti-windup.
 */

#include "pid.h"

EXPORT void pid_init(PID *p, W kp_q8, W ki_q8, W kd_q8, W i_lim,
		     W out_min, W out_max)
{
	p->kp_q8 = kp_q8;
	p->ki_q8 = ki_q8;
	p->kd_q8 = kd_q8;
	p->i_lim = (i_lim < 0) ? -i_lim : i_lim;
	p->out_min = out_min;
	p->out_max = out_max;
	pid_reset(p);
}

EXPORT void pid_reset(PID *p)
{
	p->i_acc = 0;
	p->prev_meas = 0;
	p->primed = FALSE;
}

EXPORT W pid_update(PID *p, W target, W measured, W ff)
{
	W	err = target - measured;
	W	dmeas = p->primed ? (measured - p->prev_meas) : 0;
	W	i_next = p->i_acc + err;
	W	out;

	if (i_next > p->i_lim) {
		i_next = p->i_lim;
	} else if (i_next < -p->i_lim) {
		i_next = -p->i_lim;
	}

	out = ff + (p->kp_q8 * err + p->ki_q8 * i_next - p->kd_q8 * dmeas) / 256;

	/* Conditional integration: keep the new integral only if it does not
	   push an already saturated output further into saturation. */
	if (out > p->out_max) {
		out = p->out_max;
		if (err < 0) {
			p->i_acc = i_next;
		}
	} else if (out < p->out_min) {
		out = p->out_min;
		if (err > 0) {
			p->i_acc = i_next;
		}
	} else {
		p->i_acc = i_next;
	}

	p->prev_meas = measured;
	p->primed = TRUE;
	return out;
}
