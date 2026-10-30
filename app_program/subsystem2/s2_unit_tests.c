/*
 *	s2_unit_tests.c
 *	Unit tests of subsystem 2's pure logic (motion command format,
 *	arbitration, PID, move/turn execution, stall detection and bounded
 *	escalation, vehicle mode FSM, fault policy).
 *
 *	Built into the on-target test image (make TEST=1, called from
 *	subsystem2_test.c) and into the host harness (host_test/), so the
 *	same checks run in both places.  Test IDs: S2-Uxx.
 *	A simple first-order wheel model stands in for the motors.
 */

#include <tk/typedef.h>

#if APP_TEST || HOST_TEST

#include "../test_report.h"
#include "motion_msg.h"
#include "motion_ctrl.h"
#include "motion_cfg.h"
#include "vehicle_fsm.h"
#include "sys_defs.h"
#include "pid.h"
#include "s2_tests.h"

/* ------------------------------------------------------------------ */
/*  Simulated plant: v_ss = (duty - FF_OFFSET) / 2 mm/s per permille,   */
/*  first-order lag, optional stuck wheel, encoder quantisation.        */

EXPORT void plant_init(PLANT *p)
{
	p->v_l = 0;
	p->v_r = 0;
	p->d_l_um = 0;
	p->d_r_um = 0;
	p->stuck_l = FALSE;
	p->stuck_r = FALSE;
}

LOCAL W wheel_ss(W duty)
{
	W	a = (duty < 0) ? -duty : duty;
	W	v = (a > FF_OFFSET_PM) ? (a - FF_OFFSET_PM) * 256 / FF_GAIN_Q8 : 0;

	return (duty < 0) ? -v : v;
}

EXPORT void plant_step(PLANT *p, W duty_l, W duty_r, UW dt_ms)
{
	W	ssl = p->stuck_l ? 0 : wheel_ss(duty_l);
	W	ssr = p->stuck_r ? 0 : wheel_ss(duty_r);

	p->v_l += (ssl - p->v_l) * (W)dt_ms / (60 + (W)dt_ms);
	p->v_r += (ssr - p->v_r) * (W)dt_ms / (60 + (W)dt_ms);
	if (p->stuck_l) {
		p->v_l = 0;
	}
	if (p->stuck_r) {
		p->v_r = 0;
	}
	p->d_l_um += p->v_l * (W)dt_ms;		/* mm/s * ms = um */
	p->d_r_um += p->v_r * (W)dt_ms;
}

EXPORT void plant_input(const PLANT *p, MC_IN *in, UW vmode)
{
	/* measured distance is quantised to whole encoder ticks */
	in->dist_l_um = (p->d_l_um / (W)ENC_UM_PER_TICK) * (W)ENC_UM_PER_TICK;
	in->dist_r_um = (p->d_r_um / (W)ENC_UM_PER_TICK) * (W)ENC_UM_PER_TICK;
	in->speed_l_mms = p->v_l;
	in->speed_r_mms = p->v_r;
	in->enc_valid = TRUE;
	in->vmode = vmode;
	in->armed = TRUE;
	in->steer = 0;
	in->steer_fresh = FALSE;
	in->cap_mms = 0;
	in->line_lost_cap = FALSE;
}

/* Run the controller for up to max_ms; returns elapsed ms when a result
   for 'src' arrives (its word in *res), or max_ms */
EXPORT UW run_until_result(MOTION_CTRL *mc, PLANT *p, UW vmode, UW src,
			   UW max_ms, UW *res)
{
	MC_IN	in;
	MC_OUT	out;
	UW	t;

	*res = 0;
	for (t = 0; t < max_ms; t += MOTION_PERIOD_MS) {
		plant_input(p, &in, vmode);
		mc_out_clear(&out);
		mc_step(mc, &in, MOTION_PERIOD_MS, &out);
		plant_step(p, out.duty_l_pm, out.duty_r_pm, MOTION_PERIOD_MS);
		if ((out.result_mask & (1UL << src)) != 0U) {
			*res = out.result[src];
			return t;
		}
	}
	return max_ms;
}

LOCAL W iabs(W v)
{
	return (v < 0) ? -v : v;
}

/* ------------------------------------------------------------------ */

LOCAL INT test_formats(void)
{
	UW		w, r, s, c;
	MOTION_STATUS	a, b;
	UW		pw[2];

	TEST_INIT();

	w = mcmd_pack(MCMD_MOVE, MSRC_AVOID, 15, 7, 15, -2048);
	TEST_CHECK("S2-U01", mcmd_type(w) == MCMD_MOVE && mcmd_src(w) == MSRC_AVOID
		   && mcmd_seq(w) == 15 && mcmd_epoch(w) == 7 && mcmd_speed(w) == 15
		   && mcmd_arg(w) == -2048 && (w & MCMD_RSV_MASK) == 0,
		   "command word round trip at field limits (arg -2048)");
	w = mcmd_pack(MCMD_TURN, MSRC_BARCODE, 0, 0, 0, 2047);
	TEST_CHECK("S2-U01b", mcmd_arg(w) == 2047 && mcmd_type(w) == MCMD_TURN,
		   "command word arg +2047");

	TEST_CHECK("S2-U02", mcmd_validate(w | 0x10000000UL) == MREJ_FORMAT
		   && mcmd_validate(mcmd_pack(6, MSRC_AVOID, 0, 0, 0, 0)) == MREJ_FORMAT
		   && mcmd_validate(mcmd_pack(MCMD_MOVE, MSRC_LINE, 0, 0, 0, 10)) == MREJ_SOURCE
		   && mcmd_validate(mcmd_pack(MCMD_MOVE, MSRC_SUPER, 0, 0, 0, 10)) == MREJ_SOURCE
		   && mcmd_validate(mcmd_pack(MCMD_SET_SPEED, MSRC_BARCODE, 0, 0, 0, 200)) == MREJ_SOURCE,
		   "reserved bits, unknown type and source rules rejected");
	TEST_CHECK("S2-U03", mcmd_validate(mcmd_pack(MCMD_MOVE, MSRC_AVOID, 0, 0, 0, 0)) == MREJ_RANGE
		   && mcmd_validate(mcmd_pack(MCMD_MOVE, MSRC_AVOID, 0, 0, 0, 2001)) == MREJ_RANGE
		   && mcmd_validate(mcmd_pack(MCMD_MOVE, MSRC_AVOID, 0, 0, 0, -2000)) == MREJ_NONE
		   && mcmd_validate(mcmd_pack(MCMD_TURN, MSRC_AVOID, 0, 0, 0, 361)) == MREJ_RANGE
		   && mcmd_validate(mcmd_pack(MCMD_TURN, MSRC_AVOID, 0, 0, 0, -360)) == MREJ_NONE
		   && mcmd_validate(mcmd_pack(MCMD_SET_SPEED, MSRC_SUPER, 0, 0, 0, 49)) == MREJ_RANGE
		   && mcmd_validate(mcmd_pack(MCMD_SET_SPEED, MSRC_SUPER, 0, 0, 0, 401)) == MREJ_RANGE
		   && mcmd_validate(mcmd_pack(MCMD_SET_SPEED, MSRC_SUPER, 0, 0, 0, 400)) == MREJ_NONE
		   && mcmd_validate(mcmd_pack(MCMD_STOP, MSRC_SUPER, 0, 0, 0, 1)) == MREJ_RANGE,
		   "argument ranges enforced at the boundaries");

	r = mres_pack(mcmd_pack(MCMD_TURN, MSRC_AVOID, 9, 0, 0, 90), MRES_REJECTED, MREJ_QUEUE);
	s = steer_pack(15, -100, 15);
	c = scap_pack(3, 2000);
	TEST_CHECK("S2-U04", mres_seq(r) == 9 && mres_status(r) == MRES_REJECTED
		   && mres_type(r) == MCMD_TURN && mres_reason(r) == MREJ_QUEUE
		   && (r & MRES_VALID) != 0
		   && steer_value(s) == -63 && steer_seq(s) == 15 && steer_speed(s) == 15
		   && steer_value(steer_pack(0, 63, 0)) == 63
		   && scap_value(c) == 1023 && scap_seq(c) == 3,
		   "result, steering (clamped -63) and speed-cap words");

	a.speed_l_mms = -1023;
	a.speed_r_mms = MSTAT_SPEED_INVALID;
	a.mstate = MS_FAULT;
	a.source = MSRC_SUPER;
	a.cmd_active = 1;
	a.stall_l = 1;
	a.stall_r = 0;
	a.enc_ok = 1;
	a.odometer_cm = 65535;
	a.heading_deg = 358;
	a.cruise_mms = 630;
	mstat_pack(&a, pw);
	mstat_unpack(pw, &b);
	TEST_CHECK("S2-U05", b.speed_l_mms == -1023 && b.speed_r_mms == MSTAT_SPEED_INVALID
		   && b.mstate == MS_FAULT && b.source == MSRC_SUPER && b.cmd_active == 1
		   && b.stall_l == 1 && b.stall_r == 0 && b.enc_ok == 1
		   && b.odometer_cm == 65535 && b.heading_deg == 358 && b.cruise_mms == 630,
		   "motion status 2-word pack/unpack at limits");
	TEST_SUMMARY("s2 formats");
	return test_fail_;
}

LOCAL INT test_pid(void)
{
	PID	p;
	W	out, i_before;
	INT	k;

	TEST_INIT();
	pid_init(&p, 512, 64, 0, 2000, -1000, 1000);
	out = pid_update(&p, 100, 100, 320);
	TEST_CHECK("S2-U06", out == 320, "zero error: output = feed-forward");

	pid_reset(&p);
	for (k = 0; k < 200; k++) {
		out = pid_update(&p, 400, 0, 0);	/* saturates high */
	}
	i_before = p.i_acc;
	out = pid_update(&p, 400, 0, 0);
	TEST_CHECK("S2-U07", out == 1000 && p.i_acc == i_before && p.i_acc <= 2000,
		   "saturated output: integrator frozen (anti-windup)");
	TEST_SUMMARY("s2 pid");
	return test_fail_;
}

LOCAL INT test_execution(void)
{
	MOTION_CTRL	mc;
	PLANT		pl;
	MC_OUT		out;
	MC_IN		in;
	UW		res, t, cmd;
	W		arc = mc_turn_arc_um(90);
	W		creep;
	W		min_v;
	UINT		k;

	TEST_INIT();
	TEST_CHECK("S2-U08", iabs(arc - 102101) <= 1 && mc_turn_arc_um(-90) == arc,
		   "turn arc for 90 deg = pi*track/4 (102.1 mm at 130 mm track)");

	/* MOVE 500 mm in Obstacle Handling (AVOID source), at an explicit
	   200 mm/s (level 8) so the check does not depend on the cruise
	   default: 500 mm at the 20 mm/s cruise alone would take 25 s */
	mc_init(&mc);
	plant_init(&pl);
	mc_out_clear(&out);
	cmd = mcmd_pack(MCMD_MOVE, MSRC_AVOID, 1, 2, 8, 500);
	TEST_CHECK("S2-U09a", mc_submit(&mc, cmd, VM_OBSTACLE, 2, &out) == MREJ_NONE,
		   "MOVE accepted in Obstacle Handling");
	t = run_until_result(&mc, &pl, VM_OBSTACLE, MSRC_AVOID, 20000, &res);
	TEST_CHECK("S2-U09", t < 20000 && mres_status(res) == MRES_DONE && mres_seq(res) == 1
		   && iabs((pl.d_l_um + pl.d_r_um) / 2 - 500000) < 25000
		   && iabs(pl.d_l_um - pl.d_r_um) < 15000,
		   "MOVE 500 mm: DONE, within 25 mm, wheels within 15 mm");

	/* Reverse remains globally disabled, but Buddy 5 may make one bounded
	   negative MOVE after entering Obstacle Handling. */
	mc_init(&mc);
	plant_init(&pl);
	mc_out_clear(&out);
	cmd = mcmd_pack(MCMD_MOVE, MSRC_AVOID, 2, 2, 1, -120);
	TEST_CHECK("S2-U09b", mc_submit(&mc, cmd, VM_OBSTACLE, 2, &out) == MREJ_NONE,
		   "bounded AVOID reverse accepted only in Obstacle Handling");
	t = run_until_result(&mc, &pl, VM_OBSTACLE, MSRC_AVOID, 15000, &res);
	TEST_CHECK("S2-U09c", t < 15000 && mres_status(res) == MRES_DONE
		   && (pl.d_l_um + pl.d_r_um) / 2 < -90000,
		   "negative MOVE drives both wheels backward and completes");

	/* TURN +90 */
	plant_init(&pl);
	mc_init(&mc);
	mc_out_clear(&out);
	(void)mc_submit(&mc, mcmd_pack(MCMD_TURN, MSRC_AVOID, 2, 2, 0, 90), VM_OBSTACLE, 2, &out);
	t = run_until_result(&mc, &pl, VM_OBSTACLE, MSRC_AVOID, 20000, &res);
	TEST_CHECK("S2-U10", t < 20000 && mres_status(res) == MRES_DONE
		   && pl.d_l_um > 0 && pl.d_r_um < 0
		   && iabs((pl.d_l_um - pl.d_r_um) / 2 - arc) < 12000,
		   "TURN +90: left forward, right backward, arc within 12 mm");

	/* NFR12: MOVE followed by a queued TURN keeps at least creep speed */
	plant_init(&pl);
	mc_init(&mc);
	mc_out_clear(&out);
	(void)mc_submit(&mc, mcmd_pack(MCMD_MOVE, MSRC_AVOID, 3, 1, 0, 300), VM_OBSTACLE, 1, &out);
	(void)mc_submit(&mc, mcmd_pack(MCMD_TURN, MSRC_AVOID, 4, 1, 0, 90), VM_OBSTACLE, 1, &out);
	creep = mc_creep_speed(&mc);
	min_v = 100000;
	for (k = 0; k < 1000 && mcmd_type(mc.active) == MCMD_MOVE && mc.has_active; k++) {
		plant_input(&pl, &in, VM_OBSTACLE);
		mc_out_clear(&out);
		mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
		plant_step(&pl, out.duty_l_pm, out.duty_r_pm, MOTION_PERIOD_MS);
		if (k > 25 && mc.has_active && mcmd_type(mc.active) == MCMD_MOVE
		    && mc.tgt_l_mms < min_v) {
			min_v = mc.tgt_l_mms;
		}
	}
	TEST_CHECK("S2-U11", mc.has_active && mcmd_type(mc.active) == MCMD_TURN
		   && min_v >= creep - 20,
		   "queued TURN starts straight from creep speed (no full stop)");
	TEST_SUMMARY("s2 execution");
	return test_fail_;
}

LOCAL INT test_arbitration(void)
{
	MOTION_CTRL	mc;
	MC_OUT		out;
	MC_IN		in;
	PLANT		pl;
	UINT		k;

	TEST_INIT();
	mc_init(&mc);
	mc_out_clear(&out);
	TEST_CHECK("S2-U12", mc_submit(&mc, mcmd_pack(MCMD_MOVE, MSRC_AVOID, 1, 0, 0, 100),
				       VM_NORMAL, 0, &out) == MREJ_MODE
		   && mres_status(out.result[MSRC_AVOID]) == MRES_REJECTED,
		   "AVOID command rejected outside Obstacle Handling");
	TEST_CHECK("S2-U13", mc_submit(&mc, mcmd_pack(MCMD_TURN, MSRC_BARCODE, 1, 0, 0, 90),
				       VM_OBSTACLE, 1, &out) != MREJ_NONE,
		   "BARCODE command rejected in Obstacle Handling");
	TEST_CHECK("S2-U14", mc_submit(&mc, mcmd_pack(MCMD_TURN, MSRC_BARCODE, 2, 3, 0, 90),
				       VM_NORMAL, 4, &out) == MREJ_STALE,
		   "command from an older mode epoch rejected as stale");
	TEST_CHECK("S2-U15", mc_submit(&mc, mcmd_pack(MCMD_MOVE, MSRC_BARCODE, 3, 0, 0, 100),
				       VM_INIT, 0, &out) == MREJ_MODE
		   && mc_submit(&mc, mcmd_pack(MCMD_MOVE, MSRC_AVOID, 3, 0, 0, 100),
				VM_SAFE_STOP, 0, &out) == MREJ_MODE,
		   "nothing accepted in INIT or SAFE_STOP");

	/* queue depth: 1 active + MC_QUEUE_DEPTH queued, then QUEUE */
	mc_init(&mc);
	for (k = 0; k <= MC_QUEUE_DEPTH; k++) {
		(void)mc_submit(&mc, mcmd_pack(MCMD_MOVE, MSRC_AVOID, k, 0, 0, 100),
				VM_OBSTACLE, 0, &out);
	}
	TEST_CHECK("S2-U16", mc_submit(&mc, mcmd_pack(MCMD_MOVE, MSRC_AVOID, 9, 0, 0, 100),
				       VM_OBSTACLE, 0, &out) == MREJ_QUEUE,
		   "bounded command queue: overflow rejected (QUEUE)");

	/* supervisory STOP pre-empts an active barcode command */
	mc_init(&mc);
	mc_out_clear(&out);
	(void)mc_submit(&mc, mcmd_pack(MCMD_TURN, MSRC_BARCODE, 5, 0, 0, 90), VM_NORMAL, 0, &out);
	mc_out_clear(&out);
	(void)mc_submit(&mc, mcmd_pack(MCMD_STOP, MSRC_SUPER, 1, 0, 0, 0), VM_NAV_ACTION, 0, &out);
	TEST_CHECK("S2-U17", (out.result_mask & (1UL << MSRC_BARCODE)) != 0
		   && mres_status(out.result[MSRC_BARCODE]) == MRES_ABORTED
		   && out.nav_finished && !mc.has_active,
		   "STOP bypass aborts the running command (ABORTED, nav finished)");
	plant_init(&pl);
	pl.v_l = 200;
	pl.v_r = 200;
	plant_input(&pl, &in, VM_NAV_ACTION);
	mc_out_clear(&out);
	mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
	TEST_CHECK("S2-U18", out.duty_l_pm == 0 && out.duty_r_pm == 0,
		   "stop(): PWM forced to zero in the very next period");
	TEST_SUMMARY("s2 arbitration");
	return test_fail_;
}

LOCAL INT test_faults(void)
{
	MOTION_CTRL	mc;
	MC_OUT		out;
	MC_IN		in;
	PLANT		pl;
	UW		res, t;
	UINT		round;
	BOOL		escalated = FALSE, cleared = FALSE;
	UW		fault_set = 0;

	TEST_INIT();
	mc_init(&mc);
	plant_init(&pl);
	pl.stuck_l = TRUE;
	pl.stuck_r = TRUE;
	mc_out_clear(&out);
	/* explicit 200 mm/s (level 8): at a slow cruise the duty needs
	   longer to pass STALL_DUTY_MIN_PM before the 400 ms window starts */
	(void)mc_submit(&mc, mcmd_pack(MCMD_MOVE, MSRC_AVOID, 1, 0, 8, 500), VM_OBSTACLE, 0, &out);
	t = run_until_result(&mc, &pl, VM_OBSTACLE, MSRC_AVOID, 5000, &res);
	TEST_CHECK("S2-U19", t <= STALL_TIME_MS + 200 && mres_status(res) == MRES_FAULT
		   && mc.state == MS_FAULT && (mc.fault_bits & SYS_FLT_MOTOR_STALL) != 0,
		   "stall (duty, no ticks for 400 ms): FAULT result and MOTOR_STALL");

	/* condition clear for 300 ms -> IDLE, fault bit cleared */
	pl.stuck_l = FALSE;
	pl.stuck_r = FALSE;
	for (t = 0; t < 1000 && mc.state == MS_FAULT; t += MOTION_PERIOD_MS) {
		plant_input(&pl, &in, VM_OBSTACLE);
		mc_out_clear(&out);
		mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
		if ((out.flt_clr & SYS_FLT_MOTOR_STALL) != 0) {
			cleared = TRUE;
		}
		if (out.duty_l_pm != 0 || out.duty_r_pm != 0) {
			break;
		}
	}
	TEST_CHECK("S2-U20", mc.state == MS_IDLE && cleared && t >= FAULT_CLEAR_MS - 40,
		   "FAULT -> IDLE only after 300 ms clear; motors off meanwhile");

	/* repeated stalls escalate instead of cycling Fault <-> Idle */
	mc_init(&mc);
	for (round = 0; round < MAX_FAULT_RETRIES + 1U; round++) {
		plant_init(&pl);
		pl.stuck_l = TRUE;
		pl.stuck_r = TRUE;
		mc_out_clear(&out);
		(void)mc_submit(&mc, mcmd_pack(MCMD_MOVE, MSRC_AVOID, round, 0, 0, 500),
				VM_OBSTACLE, 0, &out);
		for (t = 0; t < 2000; t += MOTION_PERIOD_MS) {
			plant_input(&pl, &in, VM_OBSTACLE);
			mc_out_clear(&out);
			mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
			fault_set |= out.flt_set;
			plant_step(&pl, out.duty_l_pm, out.duty_r_pm, MOTION_PERIOD_MS);
			if (mc.state == MS_IDLE && t > 500) {
				break;
			}
		}
		if ((fault_set & SYS_FLT_MOTION_ESCALATED) != 0) {
			escalated = TRUE;
			break;
		}
	}
	TEST_CHECK("S2-U21", escalated && mc.fault_retries >= MAX_FAULT_RETRIES,
		   "3rd consecutive stall raises MOTION_ESCALATED (bounded recovery)");

	/* one wheel silent while the other really moves: encoder fault */
	mc_init(&mc);
	plant_init(&pl);
	pl.stuck_r = TRUE;
	mc_out_clear(&out);
	(void)mc_submit(&mc, mcmd_pack(MCMD_MOVE, MSRC_AVOID, 1, 0, 0, 800), VM_OBSTACLE, 0, &out);
	fault_set = 0;
	for (t = 0; t < 4000 && mc.state != MS_FAULT; t += MOTION_PERIOD_MS) {
		plant_input(&pl, &in, VM_OBSTACLE);
		mc_out_clear(&out);
		mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
		fault_set |= out.flt_set;
		plant_step(&pl, out.duty_l_pm, out.duty_r_pm, MOTION_PERIOD_MS);
	}
	TEST_CHECK("S2-U22", mc.state == MS_FAULT
		   && (fault_set & (SYS_FLT_ENCODER | SYS_FLT_MOTOR_STALL)) != 0,
		   "one wheel without ticks while driven: motion FAULT raised");
	TEST_SUMMARY("s2 faults");
	return test_fail_;
}

LOCAL INT test_steering(void)
{
	MOTION_CTRL	mc;
	MC_OUT		out;
	MC_IN		in;
	PLANT		pl;
	UINT		k;
	W		capped_max = 0;

	TEST_INIT();
	mc_init(&mc);
	plant_init(&pl);
	for (k = 0; k < 50; k++) {
		plant_input(&pl, &in, VM_NORMAL);
		in.steer = steer_pack(k, 20, 15);	/* turn right */
		in.steer_fresh = TRUE;
		mc_out_clear(&out);
		mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
		plant_step(&pl, out.duty_l_pm, out.duty_r_pm, MOTION_PERIOD_MS);
	}
	TEST_CHECK("S2-U23", mc.tgt_l_mms > mc.tgt_r_mms && pl.v_l > pl.v_r
		   && mc.state == MS_STRAIGHT,
		   "positive steer turns right (left wheel faster)");

	for (k = 0; k < 30; k++) {
		plant_input(&pl, &in, VM_NORMAL);
		in.steer = steer_pack(0, 0, 15);
		in.steer_fresh = FALSE;			/* line task silent */
		mc_out_clear(&out);
		mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
		plant_step(&pl, out.duty_l_pm, out.duty_r_pm, MOTION_PERIOD_MS);
	}
	TEST_CHECK("S2-U24", mc.tgt_l_mms == 0 && mc.tgt_r_mms == 0,
		   "stale steering: the car stops instead of driving blind");

	for (k = 0; k < 60; k++) {
		plant_input(&pl, &in, VM_NORMAL);
		in.steer = steer_pack(k, 0, 15);
		in.steer_fresh = TRUE;
		in.cap_mms = 90;			/* Buddy 5 caution cap */
		mc_out_clear(&out);
		mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
		plant_step(&pl, out.duty_l_pm, out.duty_r_pm, MOTION_PERIOD_MS);
		if (mc.tgt_l_mms > capped_max) {
			capped_max = mc.tgt_l_mms;
		}
	}
	TEST_CHECK("S2-U25", capped_max <= 90 && capped_max > 0,
		   "obstacle speed cap limits line-following speed (FR8)");

	plant_input(&pl, &in, VM_NAV_ACTION);
	in.steer = steer_pack(1, 30, 15);
	in.steer_fresh = TRUE;
	in.cap_mms = 0;
	mc_out_clear(&out);
	mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
	TEST_CHECK("S2-U26", mc.state == MS_IDLE,
		   "steering ignored outside Normal Navigation");

	plant_input(&pl, &in, VM_SAFE_STOP);
	in.steer = steer_pack(2, 30, 15);
	in.steer_fresh = TRUE;
	mc_out_clear(&out);
	mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
	TEST_CHECK("S2-U27", out.duty_l_pm == 0 && out.duty_r_pm == 0,
		   "SAFE_STOP: zero motor output whatever the inputs");

	/* Driving, then an obstacle: no steering, the targets ramp to 0
	   while the wheels still roll.  Without ENC_QUADRATURE the encoder
	   cannot see direction and counts a reversing wheel as forward
	   (the last driven direction), as modelled here. */
	mc_init(&mc);
	plant_init(&pl);
	for (k = 0; k < 50; k++) {
		plant_input(&pl, &in, VM_NORMAL);
		in.steer = steer_pack(k, 0, 15);
		in.steer_fresh = TRUE;
		mc_out_clear(&out);
		mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
		plant_step(&pl, out.duty_l_pm, out.duty_r_pm, MOTION_PERIOD_MS);
	}
	capped_max = 0;				/* here: most reverse duty seen */
	for (k = 0; k < 150; k++) {
		plant_input(&pl, &in, VM_OBSTACLE);
#if !ENC_QUADRATURE
		in.speed_l_mms = iabs(pl.v_l);
		in.speed_r_mms = iabs(pl.v_r);
#endif
		mc_out_clear(&out);
		mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
		plant_step(&pl, out.duty_l_pm, out.duty_r_pm, MOTION_PERIOD_MS);
		if (-out.duty_l_pm > capped_max) {
			capped_max = -out.duty_l_pm;
		}
		if (-out.duty_r_pm > capped_max) {
			capped_max = -out.duty_r_pm;
		}
	}
	TEST_CHECK("S2-U40", (ENC_QUADRATURE || capped_max == 0)
		   && out.duty_l_pm == 0 && out.duty_r_pm == 0
		   && iabs(pl.v_l) < MIN_CTRL_SPEED_MMS && iabs(pl.v_r) < MIN_CTRL_SPEED_MMS,
		   "stop for an obstacle: wheels coast to rest, no reverse duty (no runaway backwards)");
	TEST_SUMMARY("s2 steering");
	return test_fail_;
}

LOCAL void vstep(VFSM *f, UW ev, UW faults, BOOL armed, BOOL ns, BOOL nf,
		 BOOL resume, VFSM_OUT *o)
{
	VFSM_IN	in;

	in.events = ev;
	in.faults = faults;
	in.crit_mask = sys_fault_critical_mask();
	in.armed = armed;
	in.nav_started = ns;
	in.nav_finished = nf;
	in.resume = resume;
	in.dt_ms = MOTION_PERIOD_MS;
	vfsm_step(f, &in, o);
}

LOCAL INT test_vehicle_fsm(void)
{
	VFSM		f;
	VFSM_OUT	o;
	UB		e0;
	UW		t;
	UW		init_flt;
	UW		crit = sys_fault_critical_mask();

	TEST_INIT();
	vfsm_init(&f);
	vstep(&f, SYS_EV_IR_CAL_OK, 0, TRUE, FALSE, FALSE, FALSE, &o);
	TEST_CHECK("S2-U28", f.mode == VM_INIT, "INIT waits for both IR and IMU calibration");
	vstep(&f, SYS_EV_IMU_CAL_OK, 0, FALSE, FALSE, FALSE, FALSE, &o);
	TEST_CHECK("S2-U29", f.mode == VM_INIT, "not armed (test image): stays in INIT");
	vstep(&f, 0, 0, TRUE, FALSE, FALSE, FALSE, &o);
	TEST_CHECK("S2-U30", f.mode == VM_NORMAL && o.changed, "INIT -> NORMAL when calibrated and armed");

	vstep(&f, 0, 0, TRUE, TRUE, FALSE, FALSE, &o);
	TEST_CHECK("S2-U31", f.mode == VM_NAV_ACTION, "barcode command -> NAVIGATION ACTION");
	e0 = f.epoch;
	vstep(&f, SYS_EV_OBSTACLE, 0, TRUE, FALSE, FALSE, FALSE, &o);
	TEST_CHECK("S2-U32", f.mode == VM_OBSTACLE && f.epoch == ((e0 + 1U) & 7U),
		   "obstacle during a barcode action -> OBSTACLE, new epoch");
	vstep(&f, SYS_EV_AVOID_DONE, 0, TRUE, FALSE, FALSE, FALSE, &o);
	TEST_CHECK("S2-U33", f.mode == VM_NORMAL, "avoidance done + line found -> NORMAL");

	vstep(&f, 0, SYS_FLT_COLLISION, TRUE, FALSE, FALSE, FALSE, &o);
	TEST_CHECK("S2-U34", f.mode == VM_SAFE_STOP, "critical fault (collision) -> SAFE_STOP");
	vstep(&f, SYS_EV_AVOID_DONE | SYS_EV_IR_CAL_OK, SYS_FLT_COLLISION, TRUE, FALSE, FALSE,
	      FALSE, &o);
	TEST_CHECK("S2-U35", f.mode == VM_SAFE_STOP, "SAFE_STOP holds; events cannot leave it");
	vstep(&f, 0, 0, TRUE, FALSE, FALSE, TRUE, &o);
	TEST_CHECK("S2-U36", f.mode == VM_INIT && !f.ir_ok && !f.imu_ok,
		   "validated RESUME -> INIT with calibration required again");

	init_flt = 0;
	for (t = 0; t <= INIT_TIMEOUT_MS + 100; t += MOTION_PERIOD_MS) {
		vstep(&f, 0, 0, TRUE, FALSE, FALSE, FALSE, &o);
		init_flt |= o.flt_set;		/* raised once, on the timeout step */
	}
	TEST_CHECK("S2-U37", f.mode == VM_SAFE_STOP && (init_flt & SYS_FLT_INIT) != 0,
		   "initialisation timeout -> SAFE_STOP with SYS_FLT_INIT");

	TEST_CHECK("S2-U38", (crit & SYS_FLT_WIFI_LOSS) == 0 && (crit & SYS_FLT_MQTT_LOSS) == 0
		   && (crit & SYS_FLT_LINE_LOST) == 0 && (crit & SYS_FLT_MOTOR_STALL) == 0
		   && (crit & SYS_FLT_RECOVERY_TIMEOUT) != 0 && (crit & SYS_FLT_OBSTACLE_UNAVOIDABLE) != 0
		   && (crit & SYS_FLT_MOTION_ESCALATED) != 0 && (crit & SYS_FLT_REMOTE_STOP) != 0,
		   "severity: network/line-lost/stall degraded; FR12 and escalation critical");
	TEST_CHECK("S2-U39", (sys_fault_event_mask() & SYS_FLT_US_NO_RESPONSE) == 0
		   && (sys_fault_event_mask() & SYS_FLT_COLLISION) != 0,
		   "condition faults clear themselves; event faults need CLEAR");
	TEST_SUMMARY("s2 vehicle fsm");
	return test_fail_;
}

EXPORT INT s2_run_unit_tests(void)
{
	return test_formats() + test_pid() + test_execution() + test_arbitration()
	     + test_faults() + test_steering() + test_vehicle_fsm();
}

#endif /* APP_TEST || HOST_TEST */
