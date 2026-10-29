/*
 *	motion_ctrl.c
 *	Motion Control core (pure logic).  See motion_ctrl.h for the
 *	statechart.  Units: distances in micrometres (W), speeds in mm/s (W),
 *	duties in permille (-1000..1000), time in ms.
 */

#include "motion_ctrl.h"
#include "motion_cfg.h"

LOCAL W iabs(W v)
{
	return (v < 0) ? -v : v;
}

LOCAL W isign(W v)
{
	return (v > 0) ? 1 : ((v < 0) ? -1 : 0);
}

LOCAL W clampw(W v, W lo, W hi)
{
	if (v < lo) {
		return lo;
	}
	if (v > hi) {
		return hi;
	}
	return v;
}

EXPORT W mc_turn_arc_um(INT deg)
{
	/* arc = deg/360 * pi * track; pi*1e6 = 3141593, mm -> um = /1000 */
	UD	num = (UD)(UW)((deg < 0) ? -deg : deg) * (UD)TRACK_WIDTH_MM
		      * 3141593ULL;

	return (W)(num / 360000ULL);
}

EXPORT W mc_creep_speed(const MOTION_CTRL *mc)
{
	W	creep = mc->cruise_mms * CREEP_PCT / 100;

	return (creep > MIN_CTRL_SPEED_MMS) ? creep : MIN_CTRL_SPEED_MMS;
}

EXPORT void mc_set_cruise(MOTION_CTRL *mc, W mms)
{
	mc->cruise_mms = clampw(mms, MOTION_SPEED_MIN_MMS, MOTION_SPEED_MAX_MMS);
}

EXPORT void mc_out_clear(MC_OUT *out)
{
	UW	i;

	out->duty_l_pm = 0;
	out->duty_r_pm = 0;
	out->dir_l = 0;
	out->dir_r = 0;
	for (i = 0; i < MSRC_COUNT; i++) {
		out->result[i] = 0;
	}
	out->result_mask = 0;
	out->flt_set = 0;
	out->flt_clr = 0;
	out->nav_started = FALSE;
	out->nav_finished = FALSE;
}

LOCAL void reset_control(MOTION_CTRL *mc)
{
	mc->tgt_l_mms = 0;
	mc->tgt_r_mms = 0;
	pid_reset(&mc->pid_l);
	pid_reset(&mc->pid_r);
	mc->stall_ms_l = 0;
	mc->stall_ms_r = 0;
	mc->moved_l_um = 0;
	mc->moved_r_um = 0;
	mc->stall_l = FALSE;
	mc->stall_r = FALSE;
}

EXPORT void mc_init(MOTION_CTRL *mc)
{
	mc->state = MS_IDLE;
	mc->has_active = FALSE;
	mc->active = 0;
	mc->q_head = 0;
	mc->q_count = 0;
	mc->start_l_um = 0;
	mc->start_r_um = 0;
	mc->target_um = 0;
	mc->elapsed_ms = 0;
	mc->timeout_ms = 0;
	pid_init(&mc->pid_l, PID_KP_Q8, PID_KI_Q8, PID_KD_Q8, PID_I_LIMIT,
		 -DUTY_MAX_PM, DUTY_MAX_PM);
	pid_init(&mc->pid_r, PID_KP_Q8, PID_KI_Q8, PID_KD_Q8, PID_I_LIMIT,
		 -DUTY_MAX_PM, DUTY_MAX_PM);
	mc->cruise_mms = CRUISE_DEFAULT_MMS;
	mc->last_l_um = 0;
	mc->last_r_um = 0;
	mc->primed = FALSE;
	mc->hard_stop = FALSE;
	mc->fault_retries = 0;
	mc->fault_clear_ms = 0;
	mc->fault_bits = 0;
	mc->healthy_ms = 0;
	mc->escalated = FALSE;
	reset_control(mc);
}

/* ------------------------------------------------------------------ */
/*  Results and queue                                                  */

LOCAL void post_result(MC_OUT *out, UW cmd, UW status, UW reason)
{
	UW	src = mcmd_src(cmd);

	out->result[src] = mres_pack(cmd, status, reason);
	out->result_mask |= (1UL << src);
}

LOCAL BOOL dequeue(MOTION_CTRL *mc, UW *cmd)
{
	if (mc->q_count == 0U) {
		return FALSE;
	}
	*cmd = mc->queue[mc->q_head];
	mc->q_head = (UB)((mc->q_head + 1U) % MC_QUEUE_DEPTH);
	mc->q_count--;
	return TRUE;
}

LOCAL BOOL enqueue(MOTION_CTRL *mc, UW cmd)
{
	if (mc->q_count >= MC_QUEUE_DEPTH) {
		return FALSE;
	}
	mc->queue[(mc->q_head + mc->q_count) % MC_QUEUE_DEPTH] = cmd;
	mc->q_count++;
	return TRUE;
}

LOCAL void flush_queue(MOTION_CTRL *mc, MC_OUT *out)
{
	UW	cmd;

	while (dequeue(mc, &cmd)) {
		post_result(out, cmd, MRES_ABORTED, MREJ_NONE);
		if (mcmd_src(cmd) == MSRC_BARCODE) {
			out->nav_finished = TRUE;
		}
	}
}

LOCAL W cmd_speed(const MOTION_CTRL *mc, UW cmd, W dflt)
{
	UW	lvl = mcmd_speed(cmd);
	W	v = (lvl == 0U) ? dflt : (W)(lvl * MOTION_SPEED_STEP_MMS);
	W	creep = mc_creep_speed(mc);

	if (v > MOTION_SPEED_MAX_MMS) {
		v = MOTION_SPEED_MAX_MMS;
	}
	return (v < creep) ? creep : v;
}

LOCAL void start_cmd(MOTION_CTRL *mc, UW cmd, MC_OUT *out)
{
	INT	arg = mcmd_arg(cmd);
	W	creep = mc_creep_speed(mc);
	W	mm;

	mc->active = cmd;
	mc->has_active = TRUE;
	mc->start_l_um = mc->last_l_um;
	mc->start_r_um = mc->last_r_um;
	mc->elapsed_ms = 0;
	mc->stall_ms_l = 0;
	mc->stall_ms_r = 0;

	if (mcmd_type(cmd) == MCMD_TURN) {
		mc->target_um = mc_turn_arc_um(arg);
		mc->state = MS_TURNING;
	} else {
		mc->target_um = iabs(arg) * 1000;
		mc->state = MS_STRAIGHT;
	}
	/* Generous time limit: the whole distance at creep speed, plus
	   slack.  The stall detector is the primary failure detector. */
	mm = mc->target_um / 1000;
	mc->timeout_ms = (UW)(mm * 1000 / creep) + CMD_TIMEOUT_EXTRA_MS;

	if (mcmd_src(cmd) == MSRC_BARCODE) {
		out->nav_started = TRUE;
	}
}

/* End the active command; on success continue with the next queued one
   (NFR12: no full stop between chained manoeuvres). */
LOCAL void finish_active(MOTION_CTRL *mc, MC_OUT *out, UW status)
{
	UW	cmd = mc->active;
	UW	next;

	post_result(out, cmd, status, MREJ_NONE);
	mc->has_active = FALSE;

	if (status == MRES_DONE) {
		mc->fault_retries = 0;		/* proven healthy again */
		if (dequeue(mc, &next)) {
			start_cmd(mc, next, out);
			return;
		}
	} else {
		flush_queue(mc, out);
	}
	mc->state = MS_IDLE;
	if (mcmd_src(cmd) == MSRC_BARCODE) {
		out->nav_finished = TRUE;
	}
}

LOCAL void abort_all(MOTION_CTRL *mc, MC_OUT *out)
{
	if (mc->has_active) {
		finish_active(mc, out, MRES_ABORTED);
	}
	flush_queue(mc, out);
}

/* ------------------------------------------------------------------ */
/*  Arbitration                                                        */

/* Which queue sources are allowed in which vehicle mode (Tribunal 1:
   steering only in Normal Navigation; Buddy 5 only once the mode is
   Obstacle Handling; nothing moves in INIT or SAFE_STOP). */
LOCAL UW mode_allows(UW src, UW type, UW vmode)
{
	if (src == MSRC_SUPER) {
		if (type == MCMD_SET_SPEED && vmode == VM_OBSTACLE) {
			return MREJ_MODE;	/* no speed changes mid-avoidance */
		}
		return MREJ_NONE;
	}
	switch (vmode) {
	case VM_NORMAL:
	case VM_NAV_ACTION:
		return (src == MSRC_BARCODE) ? MREJ_NONE : MREJ_MODE;
	case VM_OBSTACLE:
		return (src == MSRC_AVOID) ? MREJ_NONE : MREJ_MODE;
	default:
		return MREJ_MODE;
	}
}

EXPORT UW mc_submit(MOTION_CTRL *mc, UW cmd, UW vmode, UW epoch, MC_OUT *out)
{
	UW	reason = mcmd_validate(cmd);
	UW	src = mcmd_src(cmd);
	UW	type = mcmd_type(cmd);

	if (reason == MREJ_NONE && (type == MCMD_RESUME || type == MCMD_CLEAR)) {
		reason = MREJ_FORMAT;		/* handled by the task, not here */
	}
	if (reason == MREJ_NONE && src != MSRC_SUPER && mcmd_epoch(cmd) != epoch) {
		reason = MREJ_STALE;		/* built before a mode change */
	}
	if (reason == MREJ_NONE) {
		reason = mode_allows(src, type, vmode);
	}
	if (reason == MREJ_NONE && !MOTION_ALLOW_REVERSE
	    && type == MCMD_MOVE && mcmd_arg(cmd) < 0) {
		reason = MREJ_RANGE;		/* no driving backwards */
	}
	if (reason == MREJ_NONE && mc->state == MS_FAULT && type != MCMD_STOP
	    && type != MCMD_SET_SPEED) {
		reason = MREJ_MODE;		/* Fault overrides queued work */
	}
	if (reason != MREJ_NONE) {
		post_result(out, cmd, MRES_REJECTED, reason);
		return reason;
	}

	if (type == MCMD_STOP) {
		mc_force_stop(mc, out);
		post_result(out, cmd, MRES_DONE, MREJ_NONE);
		return MREJ_NONE;
	}
	if (type == MCMD_SET_SPEED) {
		mc_set_cruise(mc, mcmd_arg(cmd));
		post_result(out, cmd, MRES_DONE, MREJ_NONE);
		return MREJ_NONE;
	}

	/* MOVE / TURN */
	if (mc->has_active) {
		UW	asrc = mcmd_src(mc->active);

		if (src > asrc) {			/* higher priority: preempt */
			abort_all(mc, out);
			start_cmd(mc, cmd, out);
		} else if (src == asrc) {
			if (!enqueue(mc, cmd)) {
				post_result(out, cmd, MRES_REJECTED, MREJ_QUEUE);
				return MREJ_QUEUE;
			}
		} else {
			post_result(out, cmd, MRES_REJECTED, MREJ_PRIORITY);
			return MREJ_PRIORITY;
		}
	} else {
		start_cmd(mc, cmd, out);
	}
	return MREJ_NONE;
}

EXPORT void mc_force_stop(MOTION_CTRL *mc, MC_OUT *out)
{
	abort_all(mc, out);
	reset_control(mc);
	mc->hard_stop = TRUE;
	if (mc->state != MS_FAULT) {
		mc->state = MS_IDLE;
	}
}

EXPORT void mc_mode_changed(MOTION_CTRL *mc, UW new_mode, MC_OUT *out)
{
	switch (new_mode) {
	case VM_INIT:
		/* fresh start after Safe Stop: forget the escalation history */
		mc->fault_retries = 0;
		mc->escalated = FALSE;
		mc_force_stop(mc, out);
		break;
	case VM_SAFE_STOP:
	case VM_OBSTACLE:
		/* Safe Stop / obstacle: stop now (FR8, NFR6).  Buddy 5 then
		   issues its own commands under the new epoch. */
		mc_force_stop(mc, out);
		break;
	default:
		break;
	}
}

/* ------------------------------------------------------------------ */
/*  Command execution                                                  */

/* Approach speed: never above the commanded speed, slowing linearly to
   the end speed (creep if another command follows, else 0) and never
   below creep before arrival. */
LOCAL W approach_speed(const MOTION_CTRL *mc, W v_cmd, W rem_um)
{
	W	creep = mc_creep_speed(mc);
	W	v_end = (mc->q_count > 0U) ? creep : 0;
	W	v_allow = v_end + DECEL_GAIN * (rem_um / 1000);
	W	v = (v_cmd < v_allow) ? v_cmd : v_allow;

	return (v < creep) ? creep : v;
}

/* Correction from left/right travel mismatch, limited to half the speed */
LOCAL W balance(W pl_um, W pr_um, W v)
{
	W	corr = HEADING_GAIN * ((pl_um - pr_um) / 1000);

	return clampw(corr, -v / 2, v / 2);
}

LOCAL void run_move(MOTION_CTRL *mc, W *des_l, W *des_r, MC_OUT *out)
{
	W	dir = isign(mcmd_arg(mc->active));
	W	pl = (mc->last_l_um - mc->start_l_um) * dir;
	W	pr = (mc->last_r_um - mc->start_r_um) * dir;
	W	rem = mc->target_um - (pl + pr) / 2;
	W	v, corr;

	if (rem <= DONE_TOL_UM) {
		finish_active(mc, out, MRES_DONE);
		*des_l = 0;
		*des_r = 0;
		return;
	}
	v = approach_speed(mc, cmd_speed(mc, mc->active, mc->cruise_mms), rem);
	corr = balance(pl, pr, v);
	*des_l = dir * (v - corr);
	*des_r = dir * (v + corr);
	mc->state = MS_STRAIGHT;
}

LOCAL void run_turn(MOTION_CTRL *mc, W *des_l, W *des_r, MC_OUT *out)
{
	W	dir = isign(mcmd_arg(mc->active));	/* +1 = clockwise */
	W	pl = (mc->last_l_um - mc->start_l_um) * dir;
	W	pr = (mc->last_r_um - mc->start_r_um) * -dir;
	W	rem = mc->target_um - (pl + pr) / 2;
	W	v, corr;

	if (rem <= DONE_TOL_UM) {
		finish_active(mc, out, MRES_DONE);
		*des_l = 0;
		*des_r = 0;
		return;
	}
	v = approach_speed(mc, cmd_speed(mc, mc->active, TURN_SPEED_MMS), rem);
	corr = balance(pl, pr, v);
	*des_l = dir * (v - corr);		/* clockwise: left forward  */
	*des_r = -dir * (v + corr);		/*            right backward */
	mc->state = MS_TURNING;
}

LOCAL void run_steering(MOTION_CTRL *mc, UW steer, W *des_l, W *des_r)
{
	W	base = mc->cruise_mms * (W)steer_speed(steer) / (W)STEER_SPD_FULL;
	W	diff = (W)steer_value(steer) * mc->cruise_mms / STEER_UNIT_DIV;

	if ((steer & STEER_REV) != 0UL) {
		/* line search back-up; without MOTION_ALLOW_REVERSE: stand */
		base = MOTION_ALLOW_REVERSE ? -base : 0;
	}

	*des_l = base + diff;			/* + steer = turn right */
	*des_r = base - diff;
	if (base == 0 && diff == 0) {
		mc->state = MS_IDLE;
	} else {
		mc->state = (base == 0) ? MS_TURNING : MS_STRAIGHT;
	}
}

/* Scale both wheels down together so the path curvature is kept */
LOCAL void apply_cap(W *des_l, W *des_r, W cap)
{
	W	m = iabs(*des_l) > iabs(*des_r) ? iabs(*des_l) : iabs(*des_r);

	if (cap > 0 && m > cap) {
		*des_l = *des_l * cap / m;
		*des_r = *des_r * cap / m;
	}
}

LOCAL W ramp(W cur, W des, W step)
{
	if (des > cur + step) {
		return cur + step;
	}
	if (des < cur - step) {
		return cur - step;
	}
	return des;
}

LOCAL W wheel_control(PID *pid, W target, W meas)
{
	W	ff;

	if (target == 0 && iabs(meas) < MIN_CTRL_SPEED_MMS / 2) {
		pid_reset(pid);			/* parked: no integrator creep */
		return 0;
	}
	ff = (target == 0) ? 0
	   : isign(target) * (FF_OFFSET_PM + FF_GAIN_Q8 * iabs(target) / 256);
#if !ENC_QUADRATURE
	/* No direction sense: never drive against the target direction.  A
	   wheel that is too fast coasts instead of braking with reverse
	   duty, so the edges are always counted the way it really turns. */
	pid->out_min = (target > 0) ? 0 : -DUTY_MAX_PM;
	pid->out_max = (target < 0) ? 0 : DUTY_MAX_PM;
#endif
	return pid_update(pid, target, meas, ff);
}

/* ------------------------------------------------------------------ */
/*  Fault handling                                                     */

LOCAL void enter_fault(MOTION_CTRL *mc, MC_OUT *out, UW bit)
{
	if (mc->has_active) {
		finish_active(mc, out, MRES_FAULT);
	}
	flush_queue(mc, out);
	reset_control(mc);
	mc->hard_stop = TRUE;
	mc->state = MS_FAULT;
	mc->fault_clear_ms = 0;
	mc->fault_bits |= bit;
	out->flt_set |= bit;

	if (mc->fault_retries < 255U) {
		mc->fault_retries++;
	}
	if (mc->fault_retries >= MAX_FAULT_RETRIES && !mc->escalated) {
		mc->escalated = TRUE;		/* Tribunal 1: escalate, do not cycle */
		out->flt_set |= SYS_FLT_MOTION_ESCALATED;
	}
}

/* Numeric stall criterion (Tribunal 1): a wheel driven with at least
   STALL_DUTY_MIN_PM that produces no encoder edge for STALL_TIME_MS. */
LOCAL void stall_check(MOTION_CTRL *mc, MC_OUT *out, W dl, W dr, UW dt)
{
	if (iabs(out->duty_l_pm) >= STALL_DUTY_MIN_PM && dl == 0) {
		mc->stall_ms_l += dt;
	} else {
		mc->stall_ms_l = 0;
		mc->moved_r_um = 0;
	}
	if (iabs(out->duty_r_pm) >= STALL_DUTY_MIN_PM && dr == 0) {
		mc->stall_ms_r += dt;
	} else {
		mc->stall_ms_r = 0;
		mc->moved_l_um = 0;
	}
	mc->moved_l_um += iabs(dl);	/* left travel while right is silent */
	mc->moved_r_um += iabs(dr);	/* right travel while left is silent */

	mc->stall_l = (mc->stall_ms_l >= STALL_TIME_MS);
	mc->stall_r = (mc->stall_ms_r >= STALL_TIME_MS);

	if (mc->stall_l && mc->stall_r) {
		enter_fault(mc, out, SYS_FLT_MOTOR_STALL);
	} else if (mc->stall_l || mc->stall_r) {
		W	other = mc->stall_l ? mc->moved_r_um : mc->moved_l_um;

		/* One wheel silent while the other really moved: its
		   encoder (or that wheel) has failed. */
		enter_fault(mc, out, (other >= ENC_FAULT_MIN_MM * 1000)
				     ? SYS_FLT_ENCODER : SYS_FLT_MOTOR_STALL);
	}
}

/* ------------------------------------------------------------------ */
/*  Control step                                                       */

EXPORT void mc_step(MOTION_CTRL *mc, const MC_IN *in, UW dt_ms, MC_OUT *out)
{
	W	dl, dr;
	W	des_l = 0, des_r = 0;
	W	step = ACCEL_MMS2 * (W)dt_ms / 1000;
	W	cap = 0;
	W	meas_l = in->enc_valid ? in->speed_l_mms : 0;
	W	meas_r = in->enc_valid ? in->speed_r_mms : 0;

	if (!mc->primed) {
		mc->last_l_um = in->dist_l_um;
		mc->last_r_um = in->dist_r_um;
		mc->primed = TRUE;
	}
	dl = in->dist_l_um - mc->last_l_um;
	dr = in->dist_r_um - mc->last_r_um;
	mc->last_l_um = in->dist_l_um;
	mc->last_r_um = in->dist_r_um;

	out->duty_l_pm = 0;
	out->duty_r_pm = 0;

	/* FAULT: motors off; leave only after the condition stayed clear
	   for FAULT_CLEAR_MS (Week 6 debounce).  Repeats are bounded by
	   the escalation in enter_fault(). */
	if (mc->state == MS_FAULT) {
		reset_control(mc);
		mc->fault_clear_ms = in->enc_valid ? (mc->fault_clear_ms + dt_ms) : 0;
		if (mc->fault_clear_ms >= FAULT_CLEAR_MS) {
			out->flt_clr |= mc->fault_bits;
			mc->fault_bits = 0;
			mc->state = MS_IDLE;
		}
		mc->hard_stop = FALSE;
		return;
	}

	/* Nothing may move in INIT / SAFE_STOP or when not armed */
	if (!in->armed || in->vmode == VM_INIT || in->vmode == VM_SAFE_STOP) {
		if (mc->has_active || mc->q_count != 0U) {
			abort_all(mc, out);
		}
		reset_control(mc);
		mc->state = MS_IDLE;
		mc->hard_stop = FALSE;
		return;
	}

	if (mc->has_active) {
		mc->elapsed_ms += dt_ms;
		if (mcmd_type(mc->active) == MCMD_TURN) {
			run_turn(mc, &des_l, &des_r, out);
		} else {
			run_move(mc, &des_l, &des_r, out);
		}
		if (mc->has_active && mc->elapsed_ms > mc->timeout_ms) {
			enter_fault(mc, out, SYS_FLT_MOTOR_STALL);
			return;
		}
	} else if (in->vmode == VM_NORMAL && in->steer_fresh
		   && (in->steer & STEER_VALID) != 0UL) {
		run_steering(mc, in->steer, &des_l, &des_r);
		cap = (W)in->cap_mms;
		if (in->line_lost_cap) {
			W	creep = mc_creep_speed(mc);

			cap = (cap == 0 || cap > creep) ? creep : cap;
		}
	} else {
		mc->state = MS_IDLE;		/* no fresh steering: stop */
	}
	if (in->vmode == VM_NAV_ACTION && in->cap_mms != 0UL) {
		cap = (W)in->cap_mms;		/* obstacle caution still applies */
	}
	apply_cap(&des_l, &des_r, cap);

	if (mc->hard_stop) {
		/* stop(): PWM forced to zero this period (Tribunal 1); from the
		   next period the speed loop holds the wheels at 0. */
		reset_control(mc);
		mc->hard_stop = FALSE;
		return;
	}
	mc->tgt_l_mms = ramp(mc->tgt_l_mms, des_l, step);
	mc->tgt_r_mms = ramp(mc->tgt_r_mms, des_r, step);

	out->duty_l_pm = clampw(wheel_control(&mc->pid_l, mc->tgt_l_mms, meas_l),
				-DUTY_MAX_PM, DUTY_MAX_PM);
	out->duty_r_pm = clampw(wheel_control(&mc->pid_r, mc->tgt_r_mms, meas_r),
				-DUTY_MAX_PM, DUTY_MAX_PM);
	/* Direction for single-channel counting.  The duty never opposes
	   the target (wheel_control), so both agree; 0 keeps the last one
	   while the wheel coasts. */
	out->dir_l = (INT)isign(mc->tgt_l_mms);
	out->dir_r = (INT)isign(mc->tgt_r_mms);

	stall_check(mc, out, dl, dr, dt_ms);
	if (mc->state == MS_FAULT) {
		out->duty_l_pm = 0;
		out->duty_r_pm = 0;
		return;
	}

	/* Fault-free driving also proves health (line following issues no
	   discrete commands that could complete). */
	if (mc->state != MS_IDLE) {
		mc->healthy_ms += dt_ms;
		if (mc->healthy_ms >= HEALTHY_RESET_MS) {
			mc->fault_retries = 0;
			mc->healthy_ms = 0;
		}
	}
}
