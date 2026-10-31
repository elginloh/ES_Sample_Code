/*
 *	s2_integration_tests.c
 *	Cross-subsystem integration tests centred on Buddy 2 (vehicle mode,
 *	command arbitration, Safe Stop):
 *	  INT-10  Buddy 5 profile -> plan -> packed AVOID commands -> Buddy 2
 *	          drives the bypass; the line search ends on the line
 *	  INT-11  Buddy 3 barcode read -> navigation command -> Buddy 2
 *	          (NAV_ACTION and back) for all four navigation commands
 *	  INT-12  simultaneous events during a barcode manoeuvre; stale commands
 *	  INT-13  fault propagation from Buddies 2-5, bounded motion-fault
 *	          escalation, validated RESUME / CLEAR
 *	  INT-14  line loss and reacquisition across Buddies 3, 5 and 2
 *	  INT-17  network loss during motion (Buddy 1 -> Buddy 2)
 *	  INT-18  Safe Stop priority over every other event and command
 *	  INT-19  queue-full conditions (motion queue, barcode edge buffer)
 *	  INT-20  remote command validation: MQTT frame -> motion -> ACK
 *	The Motion Control cycle is reproduced with the same pure modules the
 *	task uses (car_cycle() follows control_cycle() in task_motion.c); the
 *	wheels are the plant model of s2_unit_tests.c.  Telemetry integration
 *	(INT-15, INT-16) is in subsystem1/s1_integration_tests.c.
 *	Runs on the host (host_test/) and in the on-target test image.
 */

#include <tk/typedef.h>

#if APP_TEST || HOST_TEST

#include "../test_report.h"
#include "motion_msg.h"
#include "motion_ctrl.h"
#include "motion_cfg.h"
#include "vehicle_fsm.h"
#include "sys_defs.h"
#include "s2_tests.h"
#include "../subsystem1/mqtt_proto.h"
#include "../subsystem1/comm_fsm.h"
#include "../subsystem3/code39.h"
#include "../subsystem3/barcode_dec.h"
#include "../subsystem3/nav_map.h"
#include "../subsystem3/line_follow.h"
#include "../subsystem3/s3_status.h"
#include "../subsystem3/s3_tests.h"
#include "../subsystem5/s5_msg.h"
#include "../subsystem5/obstacle_profile.h"
#include "../subsystem5/avoid_plan.h"
#include "../subsystem5/s5_tests.h"

#define AVOID_SPEED_LVL		6U	/* S5_AVOID_SPEED_LVL (subsystem5.h) */
#define RUN_LIMIT_MS		30000U	/* longest simulated manoeuvre */
#define TURN_TOL_MDEG		10000	/* NFR8: turn within +-10 deg */

/* The vehicle as Buddy 2 sees it: controller, mode machine, shared fault
   word, pending events, published results, wheels and a pose estimate */
typedef struct {
	MOTION_CTRL	mc;
	VFSM		vf;
	PLANT		pl;
	UW		faults;		/* shared fault bitmask */
	UW		events;		/* SYS_EV_* for the next cycle */
	BOOL		nav_started;
	BOOL		nav_finished;
	BOOL		resume;
	UW		res[MSRC_COUNT];	/* newest result word per source */
	UW		res_mask;
	UW		steer;		/* Buddy 3 steering word */
	BOOL		steer_fresh;
	UB		steer_seq;
	UB		seq[MSRC_COUNT];
	W		duty_l;		/* duties actually applied */
	W		duty_r;
	W		last_l_um;
	W		last_r_um;
	W		th_mdeg;	/* heading, + = clockwise from the start */
	D		x_um;		/* + = right of the line */
	D		y_um;		/* along the line */
	D		min_x_um;	/* closest approach beside the obstacle */
} CAR;

LOCAL CAR		car;		/* static: small test task stack */
LOCAL BC_DEC		bc;
LOCAL LINE_FOLLOW	lfol;
LOCAL COMM_FSM		cfsm;

LOCAL W iabs(W v)
{
	return (v < 0) ? -v : v;
}

/* Results, fault bits and navigation flags of one MC_OUT, as
   apply_faults() + post_results() in task_motion.c */
LOCAL void car_take(CAR *c, MC_OUT *o)
{
	UINT	i;

	c->faults &= ~o->flt_clr;
	c->faults |= o->flt_set;
	c->nav_started = c->nav_started || o->nav_started;
	c->nav_finished = c->nav_finished || o->nav_finished;
	for (i = 0; i < MSRC_COUNT; i++) {
		if ((o->result_mask & (1UL << i)) != 0U) {
			c->res[i] = o->result[i];
			c->res_mask |= (1UL << i);
		}
	}
	mc_out_clear(o);
}

LOCAL void car_pose(CAR *c)
{
	W	dl = c->pl.d_l_um - c->last_l_um;
	W	dr = c->pl.d_r_um - c->last_r_um;
	W	ds = (dl + dr) / 2;
	INT	deg;

	c->last_l_um = c->pl.d_l_um;
	c->last_r_um = c->pl.d_r_um;
	/* clockwise turn = (dl - dr) / track radians; 57296 mdeg per radian */
	c->th_mdeg += (W)(((D)(dl - dr) * 57296) / ((D)TRACK_WIDTH_MM * 1000));
	deg = (INT)((c->th_mdeg >= 0) ? (c->th_mdeg + 500) / 1000 : (c->th_mdeg - 500) / 1000);
	c->x_um += ((D)ds * op_sin_q15(deg)) / 32768;
	c->y_um += ((D)ds * op_cos_q15(deg)) / 32768;
	if (c->y_um >= 200000 && c->y_um <= 600000 && c->x_um < c->min_x_um) {
		c->min_x_um = c->x_um;
	}
}

/* One Motion Control period: control with the current mode, then the
   mode machine, then outputs (task_motion.c control_cycle()) */
LOCAL void car_cycle(CAR *c)
{
	MC_IN		in;
	MC_OUT		o;
	VFSM_IN		vin;
	VFSM_OUT	vout;
	W		dl, dr;

	plant_input(&c->pl, &in, c->vf.mode);
	in.steer = c->steer;
	in.steer_fresh = c->steer_fresh;
	in.cap_mms = 0;
	in.line_lost_cap = c->vf.line_lost;
	mc_out_clear(&o);
	mc_step(&c->mc, &in, MOTION_PERIOD_MS, &o);
	dl = o.duty_l_pm;
	dr = o.duty_r_pm;
	car_take(c, &o);

	vin.events = c->events;
	vin.faults = c->faults;
	vin.crit_mask = sys_fault_critical_mask();
	vin.armed = TRUE;
	vin.nav_started = c->nav_started;
	vin.nav_finished = c->nav_finished;
	vin.resume = c->resume;
	vin.dt_ms = MOTION_PERIOD_MS;
	vfsm_step(&c->vf, &vin, &vout);
	c->events = 0;
	c->nav_started = FALSE;
	c->nav_finished = FALSE;
	c->resume = FALSE;
	c->faults |= vout.flt_set;
	if (vout.changed) {
		mc_mode_changed(&c->mc, c->vf.mode, &o);
		car_take(c, &o);
	}
	if (c->vf.mode == VM_INIT || c->vf.mode == VM_SAFE_STOP || vout.changed) {
		dl = 0;				/* motor_off() */
		dr = 0;
	}
	c->duty_l = dl;
	c->duty_r = dr;
	plant_step(&c->pl, dl, dr, MOTION_PERIOD_MS);
	car_pose(c);
}

/* A fresh car, calibrated and armed: INIT -> NORMAL */
LOCAL void car_start(CAR *c)
{
	UINT	i;

	mc_init(&c->mc);
	vfsm_init(&c->vf);
	plant_init(&c->pl);
	c->faults = 0;
	c->nav_started = FALSE;
	c->nav_finished = FALSE;
	c->resume = FALSE;
	for (i = 0; i < MSRC_COUNT; i++) {
		c->res[i] = 0;
		c->seq[i] = 0;
	}
	c->res_mask = 0;
	c->steer = 0;
	c->steer_fresh = FALSE;
	c->steer_seq = 0;
	c->duty_l = 0;
	c->duty_r = 0;
	c->last_l_um = 0;
	c->last_r_um = 0;
	c->th_mdeg = 0;
	c->x_um = 0;
	c->y_um = 0;
	c->min_x_um = 0x7FFFFFFF;
	c->events = SYS_EV_IR_CAL_OK | SYS_EV_IMU_CAL_OK;
	car_cycle(c);
}

/* Line following publishes a new steering word every period */
LOCAL void car_steer(CAR *c, INT steer, UW frac)
{
	c->steer_seq = (UB)((c->steer_seq + 1U) & STEER_SEQ_MASK);
	c->steer = steer_pack(c->steer_seq, steer, frac);
	c->steer_fresh = TRUE;
}

/* motion_make_cmd(): next seq of the source, current epoch */
LOCAL UW car_cmd(CAR *c, UW type, UW src, INT arg, UW spd)
{
	c->seq[src] = (UB)((c->seq[src] + 1U) & MCMD_SEQ_MASK);
	return mcmd_pack(type, src, c->seq[src], c->vf.epoch, spd, arg);
}

/* drain_commands() -> mc_submit() */
LOCAL UW car_submit(CAR *c, UW cmd)
{
	MC_OUT	o;
	UW	r;

	mc_out_clear(&o);
	r = mc_submit(&c->mc, cmd, c->vf.mode, c->vf.epoch, &o);
	car_take(c, &o);
	return r;
}

/* motion_stop(): queue bypass */
LOCAL void car_stop(CAR *c)
{
	MC_OUT	o;

	mc_out_clear(&o);
	mc_force_stop(&c->mc, &o);
	car_take(c, &o);
}

/* Validated supervisory CLEAR / RESUME (supervisory_recovery()) */
LOCAL UW car_recovery(CAR *c, UW type)
{
	UW	clear;
	UW	r = sys_recovery_decide(type, c->vf.mode, c->faults, &clear);

	c->faults &= ~clear;
	if (r == MREJ_NONE && type == MCMD_RESUME) {
		c->resume = TRUE;
	}
	return r;
}

LOCAL BOOL is_result_of(const CAR *c, UW cmd)
{
	UW	src = mcmd_src(cmd);
	UW	r = c->res[src];

	return ((c->res_mask & (1UL << src)) != 0U && (r & MRES_VALID) != 0U
		&& mres_seq(r) == mcmd_seq(cmd) && mres_type(r) == mcmd_type(cmd))
	       ? TRUE : FALSE;
}

/* Run until 'cmd' has a result; returns the result word (0 = none) */
LOCAL UW car_until_result(CAR *c, UW cmd, UW limit_ms)
{
	UW	t;

	for (t = 0; t <= limit_ms; t += MOTION_PERIOD_MS) {
		if (is_result_of(c, cmd)) {
			c->res_mask &= ~(1UL << mcmd_src(cmd));
			return c->res[mcmd_src(cmd)];
		}
		car_cycle(c);
	}
	return 0;
}

LOCAL void car_drive(CAR *c, UINT cycles)
{
	UINT	k;

	for (k = 0; k < cycles; k++) {
		car_steer(c, 0, STEER_SPD_FULL);
		car_cycle(c);
	}
}

/* ------------------------------------------------------------------ */

LOCAL INT int_avoidance(void)
{
	CAR		*c = &car;
	S5_PROFILE	p, q;
	AV_PLAN		plan;
	UW		pw[2];
	UINT		k;
	UW		t;
	INT		rc;
	BOOL		steps_ok = TRUE, packed_ok = TRUE, found = FALSE;
	W		end_th, end_x, closest;

	TEST_INIT();
	car_start(c);
	s5_profile_box(-75, 75, 300, 0, &p);	/* Buddy 5: coarse + fine scan */
	s5_profile_pack(&p, pw);		/* Profile -> Avoidance queue */
	s5_profile_unpack(pw, &q);
	rc = av_plan(&q, &plan);
	c->events = SYS_EV_OBSTACLE;		/* Buddy 5 -> Buddy 2 */
	car_cycle(c);
	TEST_CHECK("INT-10a", rc == AVP_OK && plan.side == AV_SIDE_RIGHT
		   && c->vf.mode == VM_OBSTACLE,
		   "box 150 mm wide at 300 mm: packed profile -> plan; obstacle event -> OBSTACLE");

	for (k = 0; k < plan.n && steps_ok; k++) {
		const AV_STEP	*st = &plan.step[k];
		INT		arg = st->arg;
		UW		type = (st->kind == AVS_TURN) ? MCMD_TURN : MCMD_MOVE;
		UW		spd = (st->kind == AVS_SEARCH) ? AVOID_SPEED_LVL / 2U
						      : AVOID_SPEED_LVL;
		UW		cmd;
		UW		r;

		/* The target implementation terminates PASS when the side sensor
		   crosses the far edge, then advances AV_RETURN_APPROACH_MM. The plant
		   has a 200 mm-deep synthetic box, so model that detected distance
		   instead of executing the 1 m safety bound. */
		if (st->kind == AVS_PASS) {
			arg = q.dist_mm + 200 - plan.offset_mm + AV_RETURN_APPROACH_MM;
		}
		cmd = car_cmd(c, type, MSRC_AVOID, arg, spd);

		if (mcmd_arg(cmd) != arg || mcmd_type(cmd) != type
		    || mcmd_validate(cmd) != MREJ_NONE) {
			packed_ok = FALSE;
		}
		if (car_submit(c, cmd) != MREJ_NONE) {
			steps_ok = FALSE;
			break;
		}
		if (st->kind == AVS_SEARCH) {
			/* Buddy 3 reports the line under the sensors once the car
			   is back over it; Buddy 5 then stops the search move */
			for (t = 0; t < RUN_LIMIT_MS && !is_result_of(c, cmd); t += MOTION_PERIOD_MS) {
				car_steer(c, 40, STEER_SPD_FULL);	/* ignored here */
				car_cycle(c);
				if (c->x_um <= 0) {
					found = TRUE;
					car_stop(c);
					break;
				}
			}
			r = car_until_result(c, cmd, 1000);
			steps_ok = found && mres_status(r) == MRES_ABORTED;
		} else {
			car_steer(c, 40, STEER_SPD_FULL);		/* ignored here */
			r = car_until_result(c, cmd, RUN_LIMIT_MS);
			steps_ok = (r & MRES_VALID) != 0U && mres_status(r) == MRES_DONE;
		}
	}
	end_th = c->th_mdeg;
	end_x = (W)(c->x_um / 1000);
	closest = (W)(c->min_x_um / 1000);
	tm_printf((UB*)"[TEST]   bypass: closest %d mm beside the box, end heading %d mdeg, "
		  "end offset %d mm, travelled %d mm\n", (INT)closest, (INT)end_th,
		  (INT)end_x, (INT)(c->y_um / 1000));
	TEST_CHECK("INT-10b", packed_ok && steps_ok && plan.n == 7U,
		   "every planned step crosses the interface packed and completes; search stops on the line");
	TEST_CHECK("INT-10c", closest >= 75 + AV_ROBOT_WIDTH_MM / 2 && iabs(end_th) <= TURN_TOL_MDEG
		   && iabs(end_x) <= 50,
		   "car centre kept >= 150 mm right of the box edge; back on the line, heading within 10 deg");

	c->events = SYS_EV_AVOID_DONE;		/* Buddy 5: resume mission */
	car_cycle(c);
	car_drive(c, 25);
	TEST_CHECK("INT-10d", c->vf.mode == VM_NORMAL && c->mc.state == MS_STRAIGHT
		   && c->duty_l > 0 && c->duty_r > 0,
		   "AVOID_DONE -> NORMAL: line steering drives the car again");
	TEST_SUMMARY("int avoidance");
	return test_fail_;
}

LOCAL INT int_barcode_nav(void)
{
	static const char	sym[4] = { 'A', 'F', 'Z', 'X' };
	static const UB		navs[4] = { NAV_LEFT, NAV_RIGHT, NAV_UTURN, NAV_STRAIGHT };
	static const W		turn_mdeg[4] = { -90000, 90000, 180000, 0 };
	CAR		*c = &car;
	BC_RESULT	res;
	S3_BC_STATUS	bs, bs2;
	UW		t_us = 1000;
	W		x_um = 0;
	UINT		k;
	UB		nav = 0;
	BOOL		unmapped_no_motion, ok = TRUE, word_ok = TRUE;

	TEST_INIT();
	car_start(c);
	bc_init(&bc);
	nav_map_reset();
	s3_drive_over(&bc, 'W', 200, 50000, 0, &t_us, &x_um, &res);
	unmapped_no_motion = res.result == BCR_VALID && res.n_data == 1U && c39_char(res.data[0]) == 'W'
		     && !nav_map_lookup(res.data[0], &nav);
	car_cycle(c);
	TEST_CHECK("INT-11a", unmapped_no_motion && c->vf.mode == VM_NORMAL && c->res_mask == 0U,
		   "valid unmapped barcode is decoded and reported without motion");

	for (k = 0; k < 4U; k++) {
		W	th0 = c->th_mdeg;
		W	d0 = (c->pl.d_l_um + c->pl.d_r_um) / 2;
		UW	type, cmd, r;
		INT	arg;
		BOOL	nav_mode;
		W	moved;

		/* next barcode beyond the 600 mm duplicate window of the last */
		s3_drive_over(&bc, sym[k], 200, x_um + 700000, (W)k * 7000, &t_us, &x_um, &res);
		if (res.result != BCR_VALID || !nav_map_lookup(res.data[0], &nav) || nav != navs[k]) {
			ok = FALSE;
			continue;
		}
		nav_to_motion(nav, &type, &arg);
		cmd = car_cmd(c, type, MSRC_BARCODE, arg, 0);
		if (car_submit(c, cmd) != MREJ_NONE) {
			ok = FALSE;
			continue;
		}
		car_cycle(c);
		nav_mode = (c->vf.mode == VM_NAV_ACTION);
		r = car_until_result(c, cmd, RUN_LIMIT_MS);
		car_cycle(c);				/* nav finished -> NORMAL */
		moved = (c->pl.d_l_um + c->pl.d_r_um) / 2 - d0;
		ok = ok && nav_mode && mres_status(r) == MRES_DONE && c->vf.mode == VM_NORMAL
		     && iabs(c->th_mdeg - th0 - turn_mdeg[k]) <= TURN_TOL_MDEG
		     && (navs[k] != NAV_STRAIGHT || iabs(moved - S3_STRAIGHT_MM * 1000) <= 10000);
		tm_printf((UB*)"[TEST]   barcode '%c' -> %s: heading change %d mdeg\n",
			  sym[k], nav_name(nav), (INT)(c->th_mdeg - th0));

		/* the barcode status word Buddy 1 reads */
		bs.state = BCS_IDLE;
		bs.symbol = res.data[0];
		bs.result = res.result;
		bs.nav = nav;
		bs.nav_valid = 1;
		bs.reversed = res.reversed ? 1U : 0U;
		bs.n_valid = (UB)(k + 2U);
		bs.n_invalid = 0;
		bs.mres = (UB)mres_status(r);
		bs.executed = 1;
		bs.err_class = S3_BCE_NONE;
		s3_bc_unpack(s3_bc_pack(&bs), &bs2);
		word_ok = word_ok && bs2.symbol == res.data[0] && bs2.nav == nav
			  && bs2.mres == MRES_DONE && bs2.executed == 1U;
	}
	nav_map_reset();
	TEST_CHECK("INT-11b", ok,
		   "A/F/Z/X -> LEFT/RIGHT/U-TURN/STRAIGHT -> NAV_ACTION -> DONE -> NORMAL");
	TEST_CHECK("INT-11c", word_ok, "barcode status word carries symbol, command and motion result");
	TEST_SUMMARY("int barcode nav");
	return test_fail_;
}

LOCAL INT int_simultaneous(void)
{
	CAR	*c = &car;
	UW	turn, stale, spd, r_bc;
	UB	epoch0;
	BOOL	pre;
	UINT	k;

	TEST_INIT();
	car_start(c);
	turn = car_cmd(c, MCMD_TURN, MSRC_BARCODE, 90, 0);
	(void)car_submit(c, turn);
	for (k = 0; k < 10U; k++) {
		car_cycle(c);
	}
	pre = (c->vf.mode == VM_NAV_ACTION) && c->mc.has_active;
	epoch0 = c->vf.epoch;
	stale = car_cmd(c, MCMD_MOVE, MSRC_AVOID, 200, AVOID_SPEED_LVL);	/* old epoch */

	/* one cycle: Buddy 1 speed request, Buddy 5 obstacle, Buddy 3 line lost */
	spd = car_cmd(c, MCMD_SET_SPEED, MSRC_SUPER, 250, 0);
	(void)car_submit(c, spd);
	c->events = SYS_EV_OBSTACLE | SYS_EV_LINE_LOST;
	car_cycle(c);
	r_bc = c->res[MSRC_BARCODE];
	TEST_CHECK("INT-12a", pre && c->vf.mode == VM_OBSTACLE
		   && c->vf.epoch == (UB)((epoch0 + 1U) & 7U)
		   && mres_status(r_bc) == MRES_ABORTED && mres_seq(r_bc) == mcmd_seq(turn)
		   && c->duty_l == 0 && c->duty_r == 0 && c->vf.line_lost,
		   "obstacle during a barcode turn: OBSTACLE wins, turn ABORTED, motors off, line loss kept");
	TEST_CHECK("INT-12b", car_submit(c, stale) == MREJ_STALE
		   && car_submit(c, car_cmd(c, MCMD_TURN, MSRC_BARCODE, 90, 0)) == MREJ_MODE
		   && car_submit(c, car_cmd(c, MCMD_SET_SPEED, MSRC_SUPER, 300, 0)) == MREJ_MODE
		   && car_submit(c, car_cmd(c, MCMD_MOVE, MSRC_AVOID, 100, AVOID_SPEED_LVL)) == MREJ_NONE
		   && c->mc.cruise_mms == 250,
		   "then: stale command, barcode command and speed change refused; Buddy 5's command accepted");
	TEST_SUMMARY("int simultaneous");
	return test_fail_;
}

LOCAL INT int_faults(void)
{
	CAR	*c = &car;
	UW	v1, v2;
	UINT	k, fault_entries = 0;
	UB	prev;
	BOOL	drive_ok, held = TRUE, init_ok, latched;

	TEST_INIT();
	car_start(c);
	car_drive(c, 25);
	drive_ok = c->duty_l > 0 && c->duty_r > 0;
	c->faults |= SYS_FLT_LINE_LOST | SYS_FLT_BARCODE_INVALID | SYS_FLT_US_INVALID
		   | SYS_FLT_SERVO_SUSPECT | SYS_FLT_WIFI_LOSS | SYS_FLT_MQTT_LOSS
		   | SYS_FLT_CMD_INVALID | SYS_FLT_TLM_OVERFLOW | SYS_FLT_CMD_QUEUE;
	car_drive(c, 25);
	TEST_CHECK("INT-13a", drive_ok && c->vf.mode == VM_NORMAL && c->duty_l > 0 && c->duty_r > 0,
		   "degraded faults from Buddies 1, 3, 5 are reported; the car keeps driving");

	c->faults = SYS_FLT_IMU;		/* Buddy 4: IMU silent (condition) */
	car_drive(c, 1);
	held = c->vf.mode == VM_SAFE_STOP && c->duty_l == 0 && c->duty_r == 0;
	for (k = 0; k < 10U; k++) {
		car_drive(c, 1);
		held = held && c->duty_l == 0 && c->duty_r == 0;
	}
	v1 = car_recovery(c, MCMD_RESUME);
	car_cycle(c);
	TEST_CHECK("INT-13b", held && v1 == MREJ_MODE && c->vf.mode == VM_SAFE_STOP,
		   "IMU fault -> SAFE_STOP in one cycle, motors held off; RESUME refused while it persists");

	c->faults &= ~SYS_FLT_IMU;		/* Buddy 4 recovered */
	v2 = car_recovery(c, MCMD_RESUME);
	car_cycle(c);
	init_ok = c->vf.mode == VM_INIT && !c->vf.ir_ok && !c->vf.imu_ok;
	for (k = 0; k < 5U; k++) {
		car_cycle(c);
	}
	init_ok = init_ok && c->vf.mode == VM_INIT;
	c->events = SYS_EV_IR_CAL_OK | SYS_EV_IMU_CAL_OK;
	car_cycle(c);
	TEST_CHECK("INT-13c", v2 == MREJ_NONE && init_ok && c->vf.mode == VM_NORMAL,
		   "condition gone: RESUME -> INIT; both recalibrations required before NORMAL");

	c->faults |= SYS_FLT_COLLISION;		/* Buddy 4: impact (event) */
	c->events = SYS_EV_COLLISION;
	car_cycle(c);
	latched = c->vf.mode == VM_SAFE_STOP;
	for (k = 0; k < 50U; k++) {
		car_cycle(c);
	}
	latched = latched && c->vf.mode == VM_SAFE_STOP && (c->faults & SYS_FLT_COLLISION) != 0U;
	v1 = car_recovery(c, MCMD_CLEAR);
	car_cycle(c);
	latched = latched && v1 == MREJ_NONE && (c->faults & SYS_FLT_COLLISION) == 0U
		  && c->vf.mode == VM_SAFE_STOP;
	v2 = car_recovery(c, MCMD_RESUME);
	car_cycle(c);
	TEST_CHECK("INT-13d", latched && v2 == MREJ_NONE && c->vf.mode == VM_INIT,
		   "collision latched until CLEAR; CLEAR alone never leaves SAFE_STOP, RESUME does");

	car_start(c);				/* both wheels jammed while driving */
	c->pl.stuck_l = TRUE;
	c->pl.stuck_r = TRUE;
	prev = c->mc.state;
	for (k = 0; k < 1000U && c->vf.mode != VM_SAFE_STOP; k++) {
		car_drive(c, 1);
		if (c->mc.state == MS_FAULT && prev != MS_FAULT) {
			fault_entries++;
		}
		prev = c->mc.state;
	}
	TEST_CHECK("INT-13e", c->vf.mode == VM_SAFE_STOP
		   && (c->faults & SYS_FLT_MOTION_ESCALATED) != 0U
		   && fault_entries == MAX_FAULT_RETRIES && c->duty_l == 0 && c->duty_r == 0,
		   "jammed wheels: 3 local motion faults, then escalation to SAFE_STOP (no endless cycling)");
	TEST_SUMMARY("int faults");
	return test_fail_;
}

LOCAL INT int_line_reacquisition(void)
{
	CAR	*c = &car;
	LF_OUT	o;
	UB	cal[10];
	W	odo = 0;
	UINT	k, lost_events = 0;
	BOOL	follow, steered = FALSE, found, gave_up, failed = FALSE;

	TEST_INIT();
	car_start(c);
	lf_init(&lfol);
	for (k = 0; k < 10U; k++) {
		cal[k] = 0x2U;				/* centre over the line */
	}
	(void)lf_calibrate(&lfol, cal, 10);
	lf_step(&lfol, IR_BIT(IR_C), odo, 20, TRUE, &o);
	follow = o.steering && lfol.state == LS_ON_LINE;

	c->events = SYS_EV_OBSTACLE;
	car_cycle(c);
	for (k = 0; k < 30U; k++) {			/* bypass leaves the line */
		odo += 4;
		lf_step(&lfol, 0, odo, 20, c->vf.mode == VM_NORMAL, &o);
		steered = steered || o.steering;
		if ((o.events & LF_EV_LOST) != 0U) {
			lost_events++;
			c->events |= SYS_EV_LINE_LOST;
		}
		car_cycle(c);
	}
	TEST_CHECK("INT-14a", follow && c->vf.mode == VM_OBSTACLE && lost_events == 1U
		   && lfol.state == LS_LOST && !steered && c->vf.line_lost,
		   "during avoidance Buddy 3 reports the loss once and neither steers nor searches");

	lf_step(&lfol, IR_BIT(IR_C) | IR_BIT(IR_L), odo, 20, FALSE, &o);
	found = (o.events & LF_EV_FOUND) != 0U && !o.steering;
	c->events = SYS_EV_LINE_FOUND;		/* also LEV_FOUND for Buddy 5 */
	car_cycle(c);
	c->events = SYS_EV_AVOID_DONE;		/* Buddy 5: aligned, done */
	car_cycle(c);
	TEST_CHECK("INT-14b", found && c->vf.mode == VM_NORMAL && !c->vf.line_lost,
		   "line under the sensors again: FOUND event; AVOID_DONE -> NORMAL");

	lf_reset_tracking(&lfol);		/* task_line.c: fresh NORMAL */
	for (k = 0; k < 20U; k++) {
		lf_step(&lfol, IR_BIT(IR_C), odo, 20, c->vf.mode == VM_NORMAL, &o);
		if (o.steering) {
			car_steer(c, o.steer, o.speed_frac);
		}
		car_cycle(c);
	}
	TEST_CHECK("INT-14c", o.steering && c->mc.state == MS_STRAIGHT
		   && c->duty_l > 0 && c->duty_r > 0,
		   "line steering resumes driving only after the hand-back to NORMAL");

	c->events = SYS_EV_OBSTACLE;		/* search fails: Buddy 5 gives up */
	car_cycle(c);
	c->faults |= SYS_FLT_RECOVERY_TIMEOUT;
	c->events = SYS_EV_RECOVERY_FAILED;
	car_cycle(c);
	gave_up = c->vf.mode == VM_SAFE_STOP;

	car_start(c);				/* line lost in Normal Navigation */
	lf_reset_tracking(&lfol);
	for (k = 0; k < 1000U; k++) {
		odo += 2;
		lf_step(&lfol, 0, odo, 20, c->vf.mode == VM_NORMAL, &o);
		if ((o.events & LF_EV_LOST) != 0U) {
			c->events |= SYS_EV_LINE_LOST;
		}
		if ((o.events & LF_EV_RECOVERY_FAIL) != 0U) {
			failed = TRUE;
			c->faults |= SYS_FLT_LINE_RECOVERY;
		}
		if (o.steering) {
			car_steer(c, o.steer, o.speed_frac);
		}
		car_cycle(c);
	}
	car_cycle(c);
	TEST_CHECK("INT-14d", gave_up && !failed && c->vf.mode == VM_NORMAL
		   && lfol.state == LS_SEARCHING,
		   "line never found: avoidance recovery gives up -> SAFE_STOP; Buddy 3's own search "
		   "keeps turning left in NORMAL (no SAFE_STOP)");
	TEST_SUMMARY("int line reacquisition");
	return test_fail_;
}

/* task_comm.c tick(): connection state -> degraded network fault bits */
LOCAL void net_faults(CAR *c, const COMM_FSM *f)
{
	c->faults = f->wifi_loss ? (c->faults | SYS_FLT_WIFI_LOSS) : (c->faults & ~SYS_FLT_WIFI_LOSS);
	c->faults = f->mqtt_loss ? (c->faults | SYS_FLT_MQTT_LOSS) : (c->faults & ~SYS_FLT_MQTT_LOSS);
}

LOCAL INT int_network_loss(void)
{
	CAR	*c = &car;
	UINT	k;
	BOOL	online, moving = TRUE, normal = TRUE, lost_seen = FALSE;
	UB	critical;

	TEST_INIT();
	car_start(c);
	cfsm_init(&cfsm);
	for (k = 0; k < 10U; k++) {
		cfsm_step(&cfsm, TRUE, CL_UP, CSS_SUBSCRIBED, 100U);
	}
	online = cfsm.state == CS_ONLINE;
	for (k = 0; k < 500U; k++) {		/* 10 s driving, WiFi gone after 1 s */
		if ((k % 5U) == 0U) {		/* communication task: 100 ms */
			cfsm_step(&cfsm, TRUE, (k < 50U) ? CL_UP : CL_FAILED,
				  (k < 50U) ? CSS_SUBSCRIBED : CSS_LOST, 100U);
			net_faults(c, &cfsm);
			lost_seen = lost_seen || (c->faults & SYS_FLT_WIFI_LOSS) != 0U;
		}
		car_drive(c, 1);
		if (k > 25U) {
			moving = moving && c->duty_l > 0 && c->duty_r > 0;
			normal = normal && c->vf.mode == VM_NORMAL;
		}
	}
	critical = ((c->faults & sys_fault_critical_mask()) != 0U) ? 1U : 0U;	/* heartbeat */
	TEST_CHECK("INT-17a", online && lost_seen && (c->faults & SYS_FLT_WIFI_LOSS) != 0U
		   && cfsm_availability(&cfsm) == CAV_RECOVERING && critical == 0U
		   && moving && normal,
		   "WiFi lost for 9 s while driving: WIFI_LOSS + RECOVERING reported, not critical; car keeps going");

	for (k = 0; k < 2000U && cfsm.state != CS_ONLINE; k++) {
		if ((k % 5U) == 0U) {
			cfsm_step(&cfsm, TRUE, CL_UP, CSS_SUBSCRIBED, 100U);
			net_faults(c, &cfsm);
		}
		car_drive(c, 1);
	}
	net_faults(c, &cfsm);
	TEST_CHECK("INT-17b", cfsm.state == CS_ONLINE && cfsm.reconnects == 1U
		   && (c->faults & (SYS_FLT_WIFI_LOSS | SYS_FLT_MQTT_LOSS)) == 0U
		   && c->vf.mode == VM_NORMAL && c->duty_l > 0,
		   "link back: automatic reconnection clears the network faults; driving never interrupted");
	TEST_SUMMARY("int network loss");
	return test_fail_;
}

LOCAL INT int_safe_stop(void)
{
	CAR	*c = &car;
	UW	mv, r;
	UINT	k;
	BOOL	moving, zero;

	TEST_INIT();
	car_start(c);
	c->events = SYS_EV_OBSTACLE;
	car_cycle(c);
	mv = car_cmd(c, MCMD_MOVE, MSRC_AVOID, 500, AVOID_SPEED_LVL);
	(void)car_submit(c, mv);
	for (k = 0; k < 25U; k++) {
		car_cycle(c);
	}
	moving = c->duty_l > 0 && c->duty_r > 0;
	c->events = SYS_EV_AVOID_DONE;		/* same cycle: success report ... */
	c->faults |= SYS_FLT_US_NO_RESPONSE;	/* ... and the sensor dies */
	car_cycle(c);
	r = c->res[MSRC_AVOID];
	TEST_CHECK("INT-18a", moving && c->vf.mode == VM_SAFE_STOP && c->duty_l == 0 && c->duty_r == 0
		   && mres_status(r) == MRES_ABORTED && mres_seq(r) == mcmd_seq(mv),
		   "critical fault in the same cycle as AVOID_DONE: SAFE_STOP wins, manoeuvre ABORTED");

	c->events = SYS_EV_OBSTACLE | SYS_EV_AVOID_DONE | SYS_EV_IR_CAL_OK | SYS_EV_IMU_CAL_OK
		  | SYS_EV_LINE_FOUND;
	car_cycle(c);
	TEST_CHECK("INT-18b", c->vf.mode == VM_SAFE_STOP
		   && car_submit(c, car_cmd(c, MCMD_MOVE, MSRC_AVOID, 100, AVOID_SPEED_LVL)) == MREJ_MODE
		   && car_submit(c, car_cmd(c, MCMD_TURN, MSRC_BARCODE, 90, 0)) == MREJ_MODE
		   && car_submit(c, car_cmd(c, MCMD_STOP, MSRC_SUPER, 0, 0)) == MREJ_NONE,
		   "in SAFE_STOP no event leaves it and only supervisory requests are accepted");

	car_start(c);				/* operator STOP while driving */
	car_drive(c, 25);
	moving = c->duty_l > 0 && c->duty_r > 0;
	c->faults |= SYS_FLT_REMOTE_STOP;	/* handle_command(): SUPER STOP */
	(void)car_submit(c, car_cmd(c, MCMD_STOP, MSRC_SUPER, 0, 0));
	car_drive(c, 1);
	zero = c->duty_l == 0 && c->duty_r == 0;
	r = c->res[MSRC_SUPER];
	TEST_CHECK("INT-18c", moving && zero && mres_status(r) == MRES_DONE
		   && c->vf.mode == VM_SAFE_STOP && car_recovery(c, MCMD_RESUME) == MREJ_NONE,
		   "remote STOP: zero output in the next period, held as SAFE_STOP until RESUME");
	TEST_SUMMARY("int safe stop");
	return test_fail_;
}

LOCAL INT int_queue_full(void)
{
	CAR		*c = &car;
	UW		cmd[MC_QUEUE_DEPTH + 2];
	UW		rej[MC_QUEUE_DEPTH + 2];
	BC_RESULT	res;
	UINT		k, accepted = 0, done = 0;
	BOOL		refused;
	UW		t;

	TEST_INIT();
	car_start(c);
	c->events = SYS_EV_OBSTACLE;
	car_cycle(c);
	for (k = 0; k < MC_QUEUE_DEPTH + 2U; k++) {
		cmd[k] = car_cmd(c, MCMD_MOVE, MSRC_AVOID, 40, AVOID_SPEED_LVL);
		rej[k] = car_submit(c, cmd[k]);
		accepted += (rej[k] == MREJ_NONE) ? 1U : 0U;
	}
	refused = rej[MC_QUEUE_DEPTH + 1U] == MREJ_QUEUE && is_result_of(c, cmd[MC_QUEUE_DEPTH + 1U])
		  && mres_status(c->res[MSRC_AVOID]) == MRES_REJECTED;
	for (k = 0; k <= MC_QUEUE_DEPTH; k++) {
		if (mres_status(car_until_result(c, cmd[k], RUN_LIMIT_MS)) == MRES_DONE) {
			done++;
		}
	}
	TEST_CHECK("INT-19a", accepted == MC_QUEUE_DEPTH + 1U && refused && done == MC_QUEUE_DEPTH + 1U,
		   "motion queue full: the 6th command is refused (QUEUE); the 5 accepted run in order");

	bc_init(&bc);
	t = 100000;
	bc_edge(&bc, t, 1, 200, 100, &res);		/* capture starts */
	for (k = 0; k < BC_MAX_EDGES + 2U && res.result == BCR_NONE; k++) {
		t += 15000;				/* 3 mm at 200 mm/s */
		bc_edge(&bc, t, (UB)(k & 1U), 200, 100, &res);
	}
	TEST_CHECK("INT-19b", res.result == BCR_INVALID && res.err == BC_ERR_OVERFLOW
		   && !bc_capturing(&bc),
		   "barcode edge buffer full: capture rejected as INVALID (never a command), decoder idle");
	TEST_SUMMARY("int queue full");
	return test_fail_;
}

LOCAL INT int_remote_command(void)
{
	CAR		*c = &car;
	CMD_RECORD	cr, cd;
	MC_OUT		o;
	UB		f[MP_MAX_FRAME], detail = 0, ack;
	UW		type = MCMD_COUNT, cmd, mode;
	INT		arg = 0, n;
	BOOL		ok, stop_ok = TRUE;
	W		cruise;

	TEST_INIT();
	car_start(c);
	cr.cmd = CMD_SET_SPEED;
	cr.arg = 300;
	cr.cmd_id = 0x21;
	n = mp_cmd_encode(&cr, 1, f, sizeof(f));
	ok = n > 0 && mp_cmd_decode(f, (UINT)n, &cd) == 1 && mp_cmd_to_motion(&cd, &type, &arg);
	cmd = car_cmd(c, type, MSRC_SUPER, arg, 0);
	ok = ok && car_submit(c, cmd) == MREJ_NONE
	     && mp_ack_of_result(c->res[MSRC_SUPER], &detail) == ACK_OK && c->mc.cruise_mms == 300;
	TEST_CHECK("INT-20a", ok,
		   "MQTT SET_SPEED 300 -> validated -> supervisory request -> applied, ACK OK");

	c->events = SYS_EV_OBSTACLE;
	car_cycle(c);
	cr.arg = 150;
	n = mp_cmd_encode(&cr, 2, f, sizeof(f));
	(void)mp_cmd_decode(f, (UINT)n, &cd);
	(void)mp_cmd_to_motion(&cd, &type, &arg);
	(void)car_submit(c, car_cmd(c, type, MSRC_SUPER, arg, 0));
	ack = mp_ack_of_result(c->res[MSRC_SUPER], &detail);
	TEST_CHECK("INT-20b", ack == ACK_REJ_STATE && detail == MREJ_MODE && c->mc.cruise_mms == 300,
		   "same request during avoidance: refused by Buddy 2, ACK carries the reason (MODE)");

	cruise = c->mc.cruise_mms;
	c->res_mask = 0;
	cr.arg = 401;				/* encoder does not judge ranges */
	n = mp_cmd_encode(&cr, 3, f, sizeof(f));
	ok = mp_cmd_decode(f, (UINT)n, &cd) == MPE_ARG && mp_err_to_ack(MPE_ARG) == ACK_BAD_ARG;
	cr.arg = 200;
	n = mp_cmd_encode(&cr, 4, f, sizeof(f));
	f[5] ^= 0x01U;				/* corrupted in transit */
	ok = ok && mp_cmd_decode(f, (UINT)n, &cd) == MPE_CRC;
	TEST_CHECK("INT-20c", ok && c->res_mask == 0U && c->mc.cruise_mms == cruise,
		   "out-of-range or corrupted MQTT commands are rejected by Buddy 1, never reach Buddy 2");

	for (mode = VM_INIT; mode < VM_COUNT; mode++) {
		mc_out_clear(&o);
		if (mc_submit(&c->mc, mcmd_pack(MCMD_STOP, MSRC_SUPER, 9, 0, 0, 0), mode, 0, &o)
		    != MREJ_NONE) {
			stop_ok = FALSE;
		}
	}
	TEST_CHECK("INT-20d", stop_ok && car_recovery(c, MCMD_RESUME) == MREJ_MODE,
		   "remote STOP accepted in all 5 modes; RESUME refused outside SAFE_STOP");
	TEST_SUMMARY("int remote command");
	return test_fail_;
}

EXPORT INT s2_run_integration_tests(void)
{
	return int_avoidance() + int_barcode_nav() + int_simultaneous() + int_faults()
	     + int_line_reacquisition() + int_network_loss() + int_safe_stop()
	     + int_queue_full() + int_remote_command();
}

#endif /* APP_TEST || HOST_TEST */
