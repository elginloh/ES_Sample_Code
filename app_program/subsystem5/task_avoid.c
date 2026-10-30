/*
 *	task_avoid.c
 *	Avoidance & Recovery task (Week 6: High, event-driven) -- Buddy 5
 *	statechart (6.5):
 *
 *	  IDLE/MONITORING -obstacle detected-> STOP/REDUCE SPEED (event to
 *	  Buddy 2, wait until stopped, hold) -> COARSE SCAN -region found->
 *	  FINE SCAN -> GENERATE PROFILE -> PLAN AVOIDANCE -> EXECUTE AVOIDANCE
 *	  -> SEARCH FOR ORIGINAL LINE -line found-> REACQUIRE LINE (align)
 *	  -> RESUME MISSION (SYS_EV_AVOID_DONE)
 *	  scan failure / invalid profile / no clearance / failed manoeuvre /
 *	  line not found / timeout -> SAFE STOP (critical fault + event)
 *
 *	Buddy 5 never writes the motors: every movement is an AVOID-source
 *	command to Buddy 2, executed one step at a time and checked against
 *	its done/aborted/fault result.  Every wait is bounded, and so is the
 *	whole recovery (S5_RECOVERY_TIMEOUT_MS).
 */

#include "subsystem5.h"
#include "drv_ultrasonic.h"

EXPORT volatile UW	s5_avoid_word;

LOCAL S5_AVOID_STATUS	ast;
LOCAL UB		scan_id;
LOCAL UW		deadline_start;

EXPORT void s5_get_avoid_status(S5_AVOID_STATUS *s)
{
	s5_avoid_status_unpack(app_load(&s5_avoid_word), s);
}

LOCAL void set_state(UB st)
{
	ast.state = st;
	app_store(&s5_avoid_word, s5_avoid_status_pack(&ast));
}

LOCAL BOOL expired(void)
{
	return ((UW)(app_now_ms() - deadline_start) > S5_RECOVERY_TIMEOUT_MS) ? TRUE : FALSE;
}

LOCAL void request_scan(UB type, INT from, INT to, UB step)
{
	S5_SCAN_REQ	rq;
	UW		w;

	rq.type = type;
	rq.scan_id = scan_id;
	rq.start_deg = (B)from;
	rq.end_deg = (B)to;
	rq.step_deg = step;
	w = s5_req_pack(&rq);
	if (tk_snd_mbf(mbf_scan_req, &w, (INT)sizeof(w), TMO_POL) != E_OK) {
		sys_fault_set(SYS_FLT_TASK_COMM);
	}
}

/* Wait for the profile of the current scan id and phase */
LOCAL BOOL wait_profile(UB phase, S5_PROFILE *p)
{
	UW	start = app_now_ms();
	UW	pw[2];

	for (;;) {
		UW	el = app_now_ms() - start;
		INT	n;

		if (el >= S5_SCAN_TIMEOUT_MS) {
			tm_printf((UB*)"avoid: profile timeout, scan %d phase %d after %d ms\n",
				  (INT)scan_id, (INT)phase, (INT)el);
			return FALSE;
		}
		n = tk_rcv_mbf(mbf_profile, pw, (TMO)(S5_SCAN_TIMEOUT_MS - el));
		if (n != (INT)sizeof(pw)) {
			continue;
		}
		s5_profile_unpack(pw, p);
		if (p->scan_id == scan_id && p->phase == phase) {
			return TRUE;		/* stale profiles are dropped */
		}
	}
}

/* One manoeuvre step; returns the MRES_* status */
LOCAL UW move_at_speed(UW type, INT arg, UW speed_lvl)
{
	UW	cmd, res = 0;
	INT	mag = (arg < 0) ? -arg : arg;
	TMO	tmo;

	(void)tk_clr_flg(flg_avoid, ~AVEV_EMERGENCY);
	tmo = (TMO)((type == MCMD_TURN)
		? (UW)mag * 25U + S5_STEP_EXTRA_MS		/* ~40 deg/s worst case */
		: (UW)mag * 1000U / (UW)motion_creep_speed_mms() + S5_STEP_EXTRA_MS);
	if (motion_command(type, MSRC_AVOID, arg, speed_lvl, &cmd) != E_OK) {
		return MRES_REJECTED;
	}
	if (motion_wait_result(cmd, tmo, &res) != E_OK) {
		motion_stop(MSRC_AVOID);
		return MRES_FAULT;
	}
	return mres_status(res);
}

LOCAL UW move(UW type, INT arg)
{
	return move_at_speed(type, arg, S5_AVOID_SPEED_LVL);
}

LOCAL BOOL line_centered(void)
{
	S3_LINE_STATUS	ls;

	s3_get_line_status(&ls);
	return ((ls.ir_black & IR_BIT(IR_C)) != 0U) ? TRUE : FALSE;
}

/* SEARCH: drive until the centre sensor, not merely a side sensor, reaches
   the line.  LEV_FOUND is only a wake-up hint because IR_L can raise it. */
LOCAL BOOL search_line(INT dist)
{
	UW	cmd, res;
	UINT	ptn;

	(void)tk_clr_flg(flg_line_evt, ~LEV_FOUND);
	if (motion_command(MCMD_MOVE, MSRC_AVOID, dist, S5_AVOID_SPEED_LVL / 2U, &cmd) != E_OK) {
		return FALSE;
	}
	for (;;) {
		(void)tk_wai_flg(flg_line_evt, LEV_FOUND, TWF_ORW | TWF_BITCLR,
				 &ptn, S3_PERIOD_MS);

		if (line_centered()) {
			motion_stop(MSRC_AVOID);	/* centre sensor is on the line */
			(void)motion_wait_result(cmd, 1000, &res);
			return TRUE;
		}
		res = motion_last_result(MSRC_AVOID);
		if ((res & MRES_VALID) != 0U && mres_seq(res) == mcmd_seq(cmd)) {
			return line_centered();		/* search distance used up */
		}
		if (expired()) {
			motion_stop(MSRC_AVOID);
			return FALSE;
		}
	}
}

LOCAL void fail(UB reason, UW fault_bits)
{
	motion_stop(MSRC_AVOID);
	ast.fail = reason;
	ast.outcome = AVO_FAILED;
	set_state(AVST_FAILED);
	sys_fault_set(fault_bits);
	sys_post_event(SYS_EV_RECOVERY_FAILED);
	request_scan(SREQ_MONITOR, 0, 0, 0);
	tm_printf((UB*)"avoid: FAILED (reason %d) -> Safe Stop\n", (INT)reason);
}

LOCAL void finish(UB outcome)
{
	ast.outcome = outcome;
	ast.fail = AVF_NONE;
	ast.count = (UB)((ast.count + 1U) & 0xFU);
	set_state(AVST_IDLE);
	request_scan(SREQ_MONITOR, 0, 0, 0);
	sys_post_event(SYS_EV_AVOID_DONE);		/* resume mission (Buddy 2) */
	tm_printf((UB*)"avoid: done (%s)\n",
		  (outcome == AVO_CLEARED) ? "obstacle gone" : "bypassed, line reacquired");
}

LOCAL BOOL wait_stopped(void)
{
	MOTION_STATUS	ms;
	UW		start = app_now_ms();

	while ((UW)(app_now_ms() - start) < S5_STOP_WAIT_MS) {
		motion_get_status(&ms);
		if (ms.mstate == MS_IDLE && ms.speed_l_mms > -20 && ms.speed_l_mms < 20
		    && ms.speed_r_mms > -20 && ms.speed_r_mms < 20) {
			(void)tk_dly_tsk(S5_HOLD_MS);	/* hold position briefly */
			return TRUE;
		}
		(void)tk_dly_tsk(S3_PERIOD_MS);
	}
	return FALSE;
}

/* Coarse then fine scan; returns FALSE after reporting the failure */
LOCAL BOOL profile_obstacle(S5_PROFILE *p, BOOL *cleared)
{
	INT	from, to, clear_from, clear_to;
	W	front_gap;
	W	face;
	W	required = AV_ROBOT_WIDTH_MM + 2 * AV_SIDE_GAP_MM;

	*cleared = FALSE;
	scan_id = (UB)((scan_id + 1U) & 0x7U);
	set_state(AVST_COARSE);
	request_scan(SREQ_COARSE, -S5_COARSE_DEG, S5_COARSE_DEG, S5_COARSE_STEP);
	if (!wait_profile(0, p) || p->status == PRF_INVALID) {
		fail(AVF_PROFILE, SYS_FLT_PROFILE_INVALID);
		return FALSE;
	}
	if (p->status == PRF_NO_OBSTACLE) {
		*cleared = TRUE;
		return FALSE;
	}

	/* fine scan around the coarse edges (Briefing: "fine scan around obstacles") */
	face = (p->dist_mm > 0) ? p->dist_mm : 1;
	from = op_atan_deg(p->left_mm, face) - S5_FINE_MARGIN;
	to = op_atan_deg(p->right_mm, face) + S5_FINE_MARGIN;

	/* The fine sweep must also look far enough beyond each edge to prove
	   that the complete robot corridor is free.  A fixed angular margin can
	   be much less than the required robot corridor at this distance (the hardware
	   run scanned only -33..33 deg and could see just 80/30 mm).  Add one
	   fine step so the stepped sweep reaches or passes the target bearing. */
	clear_from = op_atan_deg(p->left_mm - required, face) - S5_FINE_STEP;
	clear_to = op_atan_deg(p->right_mm + required, face) + S5_FINE_STEP;
	if (clear_from < from) {
		from = clear_from;
	}
	if (clear_to > to) {
		to = clear_to;
	}
	from = (from < -S5_SCAN_LIMIT_DEG) ? -S5_SCAN_LIMIT_DEG : from;
	to = (to > S5_SCAN_LIMIT_DEG) ? S5_SCAN_LIMIT_DEG : to;
	set_state(AVST_FINE);
	request_scan(SREQ_FINE, from, to, S5_FINE_STEP);
	if (!wait_profile(1, p)) {
		fail(AVF_PROFILE, SYS_FLT_PROFILE_INVALID);
		return FALSE;
	}
	if (p->status == PRF_NO_OBSTACLE) {
		*cleared = TRUE;
		return FALSE;
	}
	if (p->status != PRF_VALID) {
		fail(AVF_PROFILE, SYS_FLT_PROFILE_INVALID);
		return FALSE;
	}
	front_gap = p->dist_mm - AV_SENSOR_BEHIND_FRONT_MM;
	if (front_gap < 0) {
		front_gap = 0;
	}
	tm_printf((UB*)"avoid: obstacle sensor %d mm, bumper gap %d mm, width %d mm, "
		  "clear L %d R %d mm\n",
		  (INT)p->dist_mm, (INT)front_gap, (INT)p->width_mm,
		  (INT)p->clear_left_mm, (INT)p->clear_right_mm);
	return TRUE;
}

#define EXEC_DONE	0
#define EXEC_EMERGENCY	1	/* stopped for something new in the way */
#define EXEC_FAILED	2	/* failure already reported           */
#define EXEC_ABANDONED	3	/* mode left Obstacle Handling        */

/* Drive parallel to the obstacle while the ultrasonic looks toward it.
   The obstacle must first be seen reliably; only then can consecutive clear
   readings mark its far edge.  Use the 10 cm lateral gap to begin a shallow
   30-degree return after a bounded 160 mm forward approach, instead of
   waiting for the complete body length to pass while still parallel. */
LOCAL INT pass_obstacle(const AV_PLAN *plan)
{
	S5_SCAN_STATUS	ss;
	UW		cmd, res, last_seq, start;
	UINT		seen_run = 0, clear_run = 0, bad_run = 0, ptn;
	BOOL		seen = FALSE, ready = FALSE;
	INT		look = (plan->side == AV_SIDE_RIGHT) ? -S5_SIDE_LOOK_DEG
							 : S5_SIDE_LOOK_DEG;
	W		odo0 = 0, odo = 0, travelled, depth;

	s5_get_scan_status(&ss);
	last_seq = ss.seq;
	request_scan(SREQ_SIDE, look, 0, 0);
	start = app_now_ms();
	while ((UW)(app_now_ms() - start) < S5_SIDE_READY_MS) {
		(void)tk_dly_tsk(S3_PERIOD_MS);
		s5_get_scan_status(&ss);
		if (ss.seq != last_seq && ss.servo_deg == look) {
			ready = TRUE;
			break;
		}
		last_seq = ss.seq;
	}
	if (!ready) {
		tm_printf((UB*)"avoid: side watch failed to reach %d deg\n", look);
		fail(AVF_SENSOR, SYS_FLT_AVOID_FAILED);
		return EXEC_FAILED;
	}

	tm_printf((UB*)"avoid: passing obstacle, ultrasonic looking %d deg; "
		  "far edge needs %d clear readings\n", look, S5_SIDE_CLEAR_CONFIRM);
	motion_get_odometry(&odo0, NULL);
	if (motion_command(MCMD_MOVE, MSRC_AVOID, AV_SIDE_PASS_MAX_MM,
			   S5_AVOID_SPEED_LVL, &cmd) != E_OK) {
		request_scan(SREQ_MONITOR, 0, 0, 0);
		fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);
		return EXEC_FAILED;
	}
	last_seq = ss.seq;

	for (;;) {
		W	side_mm;

		(void)tk_dly_tsk(S3_PERIOD_MS);
		if (sys_mode() != VM_OBSTACLE) {
			motion_stop(MSRC_AVOID);
			return EXEC_ABANDONED;
		}
		if (expired()) {
			motion_stop(MSRC_AVOID);
			fail(AVF_TIMEOUT, SYS_FLT_RECOVERY_TIMEOUT);
			return EXEC_FAILED;
		}

		motion_get_odometry(&odo, NULL);
		travelled = odo - odo0;
		if (travelled < 0) travelled = -travelled;
		s5_get_scan_status(&ss);
		if (ss.seq != last_seq && ss.servo_deg == look) {
			last_seq = ss.seq;
			side_mm = s5_dist_mm(ss.dist_code);
			if (ss.us_result == US_OK && side_mm > 0
			    && side_mm <= S5_SIDE_PRESENT_MAX_MM) {
				if (seen_run < S5_SIDE_SEEN_CONFIRM) seen_run++;
				if (seen_run >= S5_SIDE_SEEN_CONFIRM) seen = TRUE;
				clear_run = 0;
				bad_run = 0;
			} else if (ss.us_result == US_NO_ECHO
				   || (ss.us_result == US_OK
				       && side_mm > S5_SIDE_PRESENT_MAX_MM)) {
				bad_run = 0;
				if (seen) {
					if (clear_run < S5_SIDE_CLEAR_CONFIRM) clear_run++;
				} else {
					seen_run = 0;
				}
			} else {
				clear_run = 0;
				if (!seen) seen_run = 0;
				if (bad_run < S5_SIDE_BAD_MAX) bad_run++;
			}
		}

		if (bad_run >= S5_SIDE_BAD_MAX) {
			motion_stop(MSRC_AVOID);
			request_scan(SREQ_MONITOR, 0, 0, 0);
			tm_printf((UB*)"avoid: side ultrasonic unusable during pass\n");
			fail(AVF_SENSOR, SYS_FLT_AVOID_FAILED);
			return EXEC_FAILED;
		}
		if (seen && clear_run >= S5_SIDE_CLEAR_CONFIRM
		    && travelled >= S5_SIDE_MIN_TRAVEL_MM) {
			motion_stop(MSRC_AVOID);
			if (motion_wait_result(cmd, 1000, &res) != E_OK
			    || (mres_status(res) != MRES_ABORTED
				&& mres_status(res) != MRES_DONE)) {
				request_scan(SREQ_MONITOR, 0, 0, 0);
				fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);
				return EXEC_FAILED;
			}
			depth = plan->offset_mm + travelled
			      - (plan->front_gap_mm + AV_SENSOR_BEHIND_FRONT_MM);
			if (depth < 0) depth = 0;
			tm_printf((UB*)"avoid: far edge found after %d mm parallel travel; "
				  "estimated obstacle length %d mm; advancing %d mm, "
				  "then returning at %d deg\n",
				  (INT)travelled, (INT)depth, AV_RETURN_APPROACH_MM,
				  AV_RETURN_TURN_DEG);
			request_scan(SREQ_MONITOR, 0, 0, 0);
			(void)tk_dly_tsk(S5_SIDE_CENTER_MS);
			res = move(MCMD_MOVE, AV_RETURN_APPROACH_MM);
			if (res == MRES_DONE) {
				return EXEC_DONE;
			}
			if (tk_wai_flg(flg_avoid, AVEV_EMERGENCY,
				       TWF_ORW | TWF_BITCLR, &ptn, TMO_POL) == E_OK) {
				return EXEC_EMERGENCY;
			}
			fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);
			return EXEC_FAILED;
		}

		res = motion_last_result(MSRC_AVOID);
		if ((res & MRES_VALID) != 0U && mres_seq(res) == mcmd_seq(cmd)) {
			request_scan(SREQ_MONITOR, 0, 0, 0);
			if (mres_status(res) == MRES_DONE) {
				tm_printf((UB*)"avoid: no far edge within %d mm pass limit\n",
					  AV_SIDE_PASS_MAX_MM);
				fail(AVF_CLEARANCE, SYS_FLT_OBSTACLE_UNAVOIDABLE);
			} else {
				fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);
			}
			return EXEC_FAILED;
		}
	}
}

LOCAL INT execute_plan(const AV_PLAN *plan)
{
	UINT	i, ptn;
	INT	s = (plan->side == AV_SIDE_RIGHT) ? 1 : -1;

	ast.side = plan->side;
	set_state(AVST_EXECUTING);
	request_scan(SREQ_MONITOR, 0, 0, 0);		/* emergency watch while moving */

	for (i = 0; i < plan->n; i++) {
		const AV_STEP	*st = &plan->step[i];
		UW		r;

		ast.step = (UB)i;
		if (sys_mode() != VM_OBSTACLE) {
			return EXEC_ABANDONED;		/* Safe Stop or operator stop */
		}
		if (expired()) {
			fail(AVF_TIMEOUT, SYS_FLT_RECOVERY_TIMEOUT);
			return EXEC_FAILED;
		}
		if (st->kind == AVS_SEARCH) {
			BOOL	found;

			set_state(AVST_SEARCHING);
			found = search_line(st->arg);
			if (!found && ast.retries < S5_SEARCH_RETRIES
			    && sys_mode() == VM_OBSTACLE) {
				/* retry once with a steeper approach, shorter run */
				ast.retries++;
				set_state(AVST_SEARCHING);
				if (move(MCMD_TURN, -s * S5_SEARCH_RETRY_DEG) == MRES_DONE) {
					found = search_line(st->arg / 2);
				}
			}
			if (!found) {
				if (sys_mode() != VM_OBSTACLE) {
					return EXEC_ABANDONED;
				}
				fail(AVF_LINE, SYS_FLT_RECOVERY_TIMEOUT);
				return EXEC_FAILED;
			}

			/* The IR board is ahead of the wheel axle.  If the final turn
			   starts when the front sensor first touches the line, the body is
			   still offset.  Advance the calibrated sensor-to-axle distance so
			   the axle reaches the line before aligning with it. */
			tm_printf((UB*)"avoid: centre sensor found line; advancing %d mm "
				  "to place wheel axle on line\n", AV_LINE_SENSOR_AHEAD_MM);
			r = move(MCMD_MOVE, AV_LINE_SENSOR_AHEAD_MM);
			if (r != MRES_DONE) {
				if (sys_mode() != VM_OBSTACLE) {
					return EXEC_ABANDONED;
				}
				fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);
				return EXEC_FAILED;
			}
			set_state(AVST_ALIGNING);	/* REACQUIRE LINE */
			continue;
		}
		if (st->kind == AVS_PASS) {
			INT pass_result = pass_obstacle(plan);

			if (pass_result != EXEC_DONE) {
				return pass_result;
			}
			continue;
		}
		r = move((st->kind == AVS_TURN) ? MCMD_TURN : MCMD_MOVE, st->arg);
		if (r == MRES_DONE) {
			continue;
		}
		if (sys_mode() != VM_OBSTACLE) {
			return EXEC_ABANDONED;
		}
		if (tk_wai_flg(flg_avoid, AVEV_EMERGENCY, TWF_ORW | TWF_BITCLR, &ptn,
			       TMO_POL) == E_OK) {
			return EXEC_EMERGENCY;
		}
		fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);
		return EXEC_FAILED;
	}
	return EXEC_DONE;
}

LOCAL void handle_obstacle(void)
{
	S5_PROFILE	prof;
	AV_PLAN		plan;
	BOOL		cleared;
	UINT		attempt;
	INT		pr;
	W		reverse_mm;
	BOOL		reverse_used = FALSE;
	UW		t0;
	UW		mode = sys_mode();

	if (mode != VM_NORMAL && mode != VM_NAV_ACTION) {
		return;					/* not navigating: ignore */
	}
	deadline_start = app_now_ms();
	ast.retries = 0;
	ast.side = AV_SIDE_NONE;
	ast.step = 0;
	set_state(AVST_STOPPING);
	sys_post_event(SYS_EV_OBSTACLE);		/* Buddy 2: Obstacle Handling + stop */

	t0 = app_now_ms();
	while (sys_mode() != VM_OBSTACLE) {
		if ((UW)(app_now_ms() - t0) > 500U || sys_mode() == VM_SAFE_STOP) {
			set_state(AVST_IDLE);		/* Buddy 2 did not hand over */
			return;
		}
		(void)tk_dly_tsk(S3_PERIOD_MS);
	}

	/* first attempt + bounded re-plans after an emergency stop */
	for (attempt = 0; attempt <= S5_REPLANS; attempt++) {
		if (!wait_stopped()) {
			fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);
			return;
		}
		for (;;) {
			if (!profile_obstacle(&prof, &cleared)) {
				if (cleared) {
					finish(AVO_CLEARED);	/* path clear again: resume */
				}
				return;
			}
			set_state(AVST_PLANNING);
			pr = av_plan(&prof, &plan);
			reverse_mm = av_reverse_needed_mm(&prof);
			if (pr == AVP_NO_CLEARANCE && !reverse_used && reverse_mm > 0) {
				tm_printf((UB*)"avoid: front too close; reversing %d mm at creep, "
					  "then rescanning\n", (INT)reverse_mm);
				if (move_at_speed(MCMD_MOVE, -(INT)reverse_mm,
						  S5_REVERSE_SPEED_LVL) != MRES_DONE) {
					fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);
					return;
				}
				reverse_used = TRUE;
				if (!wait_stopped()) {
					fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);
					return;
				}
				continue;		/* full coarse + fine rescan */
			}
			break;
		}
		if (pr == AVP_NO_CLEARANCE) {
			if (plan.offset_mm > AV_MAX_LATERAL_OFFSET_MM) {
				tm_printf((UB*)"avoid: rejected centre offset %d mm; limit %d mm "
					  "(obstacle width/edge may be noisy)\n",
					  (INT)plan.offset_mm, AV_MAX_LATERAL_OFFSET_MM);
			}
			tm_printf((UB*)"avoid: no safe path (have %d mm bumper gap; "
				  "need %d mm side, %d mm front)\n",
				  (INT)plan.front_gap_mm, (INT)plan.required_mm,
				  AV_MIN_FRONT_CLEARANCE_MM);
			fail(AVF_CLEARANCE, SYS_FLT_OBSTACLE_UNAVOIDABLE);
			return;
		}
		if (pr != AVP_OK) {
			fail(AVF_PROFILE, SYS_FLT_PROFILE_INVALID);
			return;
		}
		tm_printf((UB*)"avoid: plan %s, edge %d + half robot %d + gap %d "
			  "= centre offset %d mm, %d steps\n",
			  (plan.side == AV_SIDE_RIGHT) ? "right" : "left",
			  (INT)((plan.side == AV_SIDE_RIGHT) ? prof.right_mm : -prof.left_mm),
			  AV_ROBOT_WIDTH_MM / 2, (INT)plan.side_gap_mm,
			  (INT)plan.offset_mm, (INT)plan.n);
		switch (execute_plan(&plan)) {
		case EXEC_DONE:
			finish(AVO_SUCCESS);
			return;
		case EXEC_EMERGENCY:
			break;				/* re-scan from where we stand */
		case EXEC_ABANDONED:
			set_state(AVST_IDLE);
			request_scan(SREQ_MONITOR, 0, 0, 0);
			return;
		default:
			return;				/* failure already reported */
		}
	}
	fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);		/* re-plans exhausted */
}

EXPORT void avoid_task(INT stacd, void *exinf)
{
	UINT	ptn;

	(void)stacd;
	(void)exinf;
	ast.state = AVST_IDLE;
	ast.side = AV_SIDE_NONE;
	ast.step = 0;
	ast.retries = 0;
	ast.outcome = AVO_NONE;
	ast.count = 0;
	ast.fail = AVF_NONE;
	set_state(AVST_IDLE);
	request_scan(SREQ_MONITOR, 0, 0, 0);

	for (;;) {
		if (tk_wai_flg(flg_avoid, AVEV_DETECTED, TWF_ORW | TWF_BITCLR, &ptn,
			       TMO_FEVR) < E_OK) {
			(void)tk_dly_tsk(100);
			continue;
		}
		handle_obstacle();
		(void)tk_clr_flg(flg_avoid, 0);	/* drop detections from during handling */
	}
}
