/*
 *	motion_ctrl.h
 *	Motion Control core (Buddy 2): command arbitration, move/turn
 *	execution, line-steering, PID speed control, stall/encoder fault
 *	detection with bounded retry and escalation.
 *
 *	Pure logic: no kernel calls and no hardware access, so the whole
 *	statechart can be exercised on the host against a simulated plant.
 *	The Motion Control task (task_motion.c) feeds it measurements every
 *	MOTION_PERIOD_MS and applies the duties it returns.
 *
 *	Statechart (Week 6 section 6.2 + Tribunal 1 changes):
 *	    IDLE -> MOVING STRAIGHT / TURNING   on a command (or line steering)
 *	    STRAIGHT <-> TURNING                next queued command, via creep
 *	                                        speed instead of a full stop
 *	    any -> IDLE                          queue empty / stop()
 *	    any -> FAULT                         stall, encoder fault, timeout
 *	    FAULT -> IDLE                        condition clear for 300 ms
 *	    FAULT count >= MAX_FAULT_RETRIES     -> SYS_FLT_MOTION_ESCALATED
 *	                                        (system Safe Stop; no endless
 *	                                        Fault <-> Idle cycling)
 */

#ifndef MOTION_CTRL_H
#define MOTION_CTRL_H

#include <tk/typedef.h>
#include "motion_msg.h"
#include "sys_defs.h"
#include "pid.h"

#define MC_QUEUE_DEPTH	4	/* commands queued behind the active one */

/* Inputs for one control step */
typedef struct {
	W	dist_l_um;	/* cumulative signed wheel travel, um */
	W	dist_r_um;
	W	speed_l_mms;	/* measured wheel speeds, mm/s */
	W	speed_r_mms;
	BOOL	enc_valid;	/* encoder snapshot readable */
	UW	vmode;		/* VM_* vehicle mode */
	BOOL	armed;		/* motors allowed to move */
	UW	steer;		/* steering word (motion_msg.h) */
	BOOL	steer_fresh;	/* seq changed within STEER_STALE_MS */
	UW	cap_mms;	/* obstacle speed cap, 0 = none */
	BOOL	line_lost_cap;	/* cap to creep while the line is lost */
} MC_IN;

/* Outputs of submit/step calls (accumulate over one task cycle) */
typedef struct {
	W	duty_l_pm;	/* -1000..1000 */
	W	duty_r_pm;
	INT	dir_l;		/* commanded direction, for 1-channel encoders */
	INT	dir_r;
	UW	result[MSRC_COUNT];	/* newest result word per source */
	UW	result_mask;		/* bit n set: result[n] is new */
	UW	flt_set;		/* fault bits to raise */
	UW	flt_clr;		/* fault bits to clear */
	BOOL	nav_started;		/* a barcode command began */
	BOOL	nav_finished;		/* the barcode command ended */
} MC_OUT;

typedef struct {
	UB	state;		/* MS_* */
	BOOL	has_active;
	UW	active;		/* running command word */
	UW	queue[MC_QUEUE_DEPTH];
	UB	q_head;
	UB	q_count;
	W	start_l_um;	/* wheel travel when the command started */
	W	start_r_um;
	W	target_um;	/* distance (MOVE) or wheel arc (TURN) */
	UW	elapsed_ms;
	UW	timeout_ms;
	W	tgt_l_mms;	/* ramped wheel speed targets */
	W	tgt_r_mms;
	PID	pid_l;
	PID	pid_r;
	W	cruise_mms;
	W	last_l_um;
	W	last_r_um;
	BOOL	primed;
	BOOL	hard_stop;	/* next step outputs zero immediately */
	UW	stall_ms_l;
	UW	stall_ms_r;
	W	moved_l_um;	/* left travel while the right wheel was silent */
	W	moved_r_um;
	BOOL	stall_l;
	BOOL	stall_r;
	UB	fault_retries;	/* faults since the last successful command */
	UW	fault_clear_ms;
	UW	fault_bits;	/* local fault bits currently raised */
	UW	healthy_ms;
	BOOL	escalated;
} MOTION_CTRL;

IMPORT void mc_init(MOTION_CTRL *mc);
IMPORT void mc_out_clear(MC_OUT *out);

/* Offer a command word.  Checks format, source, epoch, mode and
   priority; returns MREJ_NONE if accepted (a REJECTED result is posted
   in 'out' otherwise).  RESUME/CLEAR are handled by the task itself. */
IMPORT UW   mc_submit(MOTION_CTRL *mc, UW cmd, UW vmode, UW epoch, MC_OUT *out);

/* stop() bypass: abort everything, zero output on the next step */
IMPORT void mc_force_stop(MOTION_CTRL *mc, MC_OUT *out);

/* Called after the vehicle mode changed */
IMPORT void mc_mode_changed(MOTION_CTRL *mc, UW new_mode, MC_OUT *out);

/* One control period */
IMPORT void mc_step(MOTION_CTRL *mc, const MC_IN *in, UW dt_ms, MC_OUT *out);

IMPORT W    mc_creep_speed(const MOTION_CTRL *mc);
IMPORT void mc_set_cruise(MOTION_CTRL *mc, W mms);

/* Wheel arc for an in-place turn of 'deg' degrees, micrometres */
IMPORT W    mc_turn_arc_um(INT deg);

#endif /* MOTION_CTRL_H */
