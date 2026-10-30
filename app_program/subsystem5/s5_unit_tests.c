/*
 *	s5_unit_tests.c
 *	Unit tests of subsystem 5's pure logic: distance code, packed
 *	messages, obstacle profiling against ray-cast synthetic scans, and
 *	avoidance planning (side choice, manoeuvre geometry, no invented
 *	bypass).  Runs on the host and in the on-target test image.
 *	Test IDs: S5-Uxx.
 */

#include <tk/typedef.h>

#if APP_TEST || HOST_TEST

#include "../test_report.h"
#include "s5_msg.h"
#include "obstacle_profile.h"
#include "avoid_plan.h"
#include "s5_tests.h"
#include "../subsystem2/motion_msg.h"

LOCAL W iabs(W v)
{
	return (v < 0) ? -v : v;
}

/* Ray cast from the sensor (origin, +y forward, +x right) at 'deg' onto
   an axis-aligned box front face [xl, xr] at y and, optionally, a wall
   parallel to the track at x = wall_x (0 = none).  Distance in mm, -1 =
   no echo within 4 m. */
LOCAL W cast(INT deg, W xl, W xr, W y, W wall_x)
{
	W	c = op_cos_q15(deg);
	W	s = op_sin_q15(deg);
	W	best = -1;

	if (c > 0 && y > 0) {
		W	t = (W)(((D)y * 32768) / c);
		W	x = (W)(((D)t * s) / 32768);

		if (x >= xl && x <= xr) {
			best = t;
		}
	}
	if (wall_x != 0 && ((wall_x > 0 && s > 0) || (wall_x < 0 && s < 0))) {
		W	t = (W)(((D)wall_x * 32768) / s);

		if (t > 0 && (best < 0 || t < best)) {
			best = t;
		}
	}
	return (best > 4000) ? -1 : best;
}

EXPORT UINT s5_synth_scan(OP_SAMPLE *o, W xl, W xr, W y, W wall_x,
			  INT from, INT to, INT step, UINT n)
{
	INT	a;

	for (a = from; a <= to && n < OP_MAX_SAMPLES; a += step) {
		W	d = cast(a, xl, xr, y, wall_x);

		o[n].angle_deg = (B)a;
		o[n].dist_mm = (H)d;
		o[n].status = (d < 0) ? SS_NO_ECHO : SS_OK;
		n++;
	}
	return n;
}

/* Coarse + fine scan of a box, profiled as the Profile task does */
EXPORT void s5_profile_box(W xl, W xr, W y, W wall_x, S5_PROFILE *p)
{
	OP_SAMPLE	o[OP_MAX_SAMPLES];
	UINT		n;

	n = s5_synth_scan(o, xl, xr, y, wall_x, -60, 60, 15, 0);
	n = s5_synth_scan(o, xl, xr, y, wall_x, -45, 45, 3, n);
	op_build(o, n, 60, p);
}

LOCAL INT test_codes(void)
{
	S5_SAMPLE	a, b;
	S5_SCAN_REQ	q, r;
	S5_PROFILE	p, p2;
	S5_SCAN_STATUS	ss, ss2;
	S5_AVOID_STATUS	as, as2;
	UW		w[2];

	TEST_INIT();
	TEST_CHECK("S5-U01", s5_dist_code(0) == 0 && s5_dist_code(2) == 0 && s5_dist_code(3) == 1
		   && s5_dist_code(4000) == 800 && s5_dist_code(4003) == 800 && s5_dist_code(9000) == 800
		   && s5_dist_mm(800) == 4000 && s5_dist_mm(S5_DIST_NO_ECHO) == -1
		   && s5_dist_mm(S5_DIST_INVALID) == -1,
		   "10-bit distance code: 5 mm steps, 0..4 m, 1022/1023 reserved");

	a.angle_deg = -80;
	a.dist_code = S5_DIST_NO_ECHO;
	a.scan_id = 7;
	a.phase = 1;
	a.last = 1;
	a.status = SS_ABORTED;
	s5_sample_unpack(s5_sample_pack(&a), &b);
	TEST_CHECK("S5-U02", b.angle_deg == -80 && b.dist_code == S5_DIST_NO_ECHO && b.scan_id == 7
		   && b.phase == 1 && b.last == 1 && b.status == SS_ABORTED,
		   "scan sample message round trip at limits");
	a.angle_deg = 80;
	a.dist_code = 800;
	s5_sample_unpack(s5_sample_pack(&a), &b);
	TEST_CHECK("S5-U02b", b.angle_deg == 80 && b.dist_code == 800, "scan sample positive limits");

	q.type = SREQ_FINE;
	q.scan_id = 5;
	q.start_deg = -75;
	q.end_deg = 75;
	q.step_deg = 15;
	s5_req_unpack(s5_req_pack(&q), &r);
	TEST_CHECK("S5-U03", r.type == SREQ_FINE && r.scan_id == 5 && r.start_deg == -75
		   && r.end_deg == 75 && r.step_deg == 15,
		   "scan request message round trip");

	p.scan_id = 6;
	p.status = PRF_VALID;
	p.phase = 1;
	p.dist_mm = 4000;
	p.bearing_deg = -127;
	p.width_mm = 1270;
	p.left_mm = -1270;
	p.right_mm = 1270;
	p.clear_left_mm = 9000;
	p.clear_right_mm = -1;
	s5_profile_pack(&p, w);
	s5_profile_unpack(w, &p2);
	TEST_CHECK("S5-U04", p2.scan_id == 6 && p2.status == PRF_VALID && p2.phase == 1
		   && p2.dist_mm == 4000 && p2.bearing_deg == -127 && p2.width_mm == 1270
		   && p2.left_mm == -1270 && p2.right_mm == 1270 && p2.clear_left_mm == 2540
		   && p2.clear_right_mm == -1,
		   "profile message: limits kept, 'unknown' clearance stays unknown");

	ss.dist_code = 1023;
	ss.servo_deg = -75;
	ss.us_result = 3;
	ss.mode = SREQ_IDLE;
	ss.seq = 15;
	ss.cap = 1;
	ss.detected = 1;
	s5_scan_status_unpack(s5_scan_status_pack(&ss), &ss2);
	as.state = AVST_FAILED;
	as.side = AV_SIDE_RIGHT;
	as.step = 7;
	as.retries = 3;
	as.outcome = AVO_CLEARED;
	as.count = 15;
	as.fail = AVF_SENSOR;
	s5_avoid_status_unpack(s5_avoid_status_pack(&as), &as2);
	TEST_CHECK("S5-U05", ss2.dist_code == 1023 && ss2.servo_deg == -75 && ss2.us_result == 3
		   && ss2.mode == SREQ_IDLE && ss2.seq == 15 && ss2.cap == 1 && ss2.detected == 1
		   && as2.state == AVST_FAILED && as2.side == AV_SIDE_RIGHT && as2.step == 7
		   && as2.retries == 3 && as2.outcome == AVO_CLEARED && as2.count == 15
		   && as2.fail == AVF_SENSOR,
		   "scan and avoidance status words round trip");
	TEST_SUMMARY("s5 codes");
	return test_fail_;
}

LOCAL INT test_profile(void)
{
	S5_PROFILE	p;
	OP_SAMPLE	o[OP_MAX_SAMPLES];
	UINT		n, k;
	BOOL		ok = TRUE;
	static const W	cases[4][3] = {	/* xl, xr, y */
		{ -75, 75, 300 }, { -25, 125, 300 }, { -60, 60, 250 }, { -100, 100, 400 }
	};

	TEST_INIT();
	for (k = 0; k < 4U; k++) {
		W	truth = cases[k][1] - cases[k][0];

		s5_profile_box(cases[k][0], cases[k][1], cases[k][2], 0, &p);
		if (p.status != PRF_VALID || iabs(p.width_mm - truth) * 10 > truth
		    || iabs(p.dist_mm - cases[k][2]) > 10) {
			ok = FALSE;
			tm_printf((UB*)"[TEST]   box %d: status %d width %d (truth %d) dist %d\n",
				  (INT)k, (INT)p.status, (INT)p.width_mm, (INT)truth, (INT)p.dist_mm);
		}
	}
	TEST_CHECK("S5-U06", ok, "width within 10 % and distance within 1 cm (4 boxes, coarse+fine)");

	s5_profile_box(-200, -50, 300, 0, &p);
	TEST_CHECK("S5-U07", p.status == PRF_VALID && p.bearing_deg < 0
		   && p.clear_right_mm > p.clear_left_mm,
		   "obstacle left of the path: negative bearing, more room on the right");

	n = s5_synth_scan(o, 0, 0, 0, 0, -60, 60, 15, 0);	/* nothing there */
	op_build(o, n, 60, &p);
	TEST_CHECK("S5-U08", p.status == PRF_NO_OBSTACLE, "no echo anywhere: path clear (NO_OBSTACLE)");
	s5_profile_box(-75, 75, 900, 0, &p);
	TEST_CHECK("S5-U09", p.status == PRF_NO_OBSTACLE, "object beyond 700 mm is not an obstacle to avoid");

	s5_profile_box(-3000, 3000, 300, 0, &p);
	TEST_CHECK("S5-U10", p.status == PRF_INSUFFICIENT && p.clear_left_mm < 0 && p.clear_right_mm < 0,
		   "wall across the whole scan: edges unknown (INSUFFICIENT)");

	for (n = 0; n < 9U; n++) {
		o[n].angle_deg = (B)(-60 + (INT)n * 15);
		o[n].dist_mm = 300;
		o[n].status = SS_OK;
	}
	op_build(o, 9, 60, &p);
	TEST_CHECK("S5-U11", p.status == PRF_INVALID, "identical range at every angle: servo suspect (INVALID)");

	n = s5_synth_scan(o, -75, 75, 300, 0, -15, 15, 15, 0);	/* 3 samples */
	op_build(o, n, 15, &p);
	TEST_CHECK("S5-U12", p.status == PRF_INVALID, "too few valid samples: INVALID");

	s5_profile_box(-75, 75, 300, 260, &p);			/* wall 260 mm right */
	TEST_CHECK("S5-U13", p.status == PRF_VALID && p.clear_right_mm >= 0
		   && p.clear_right_mm < 250 && p.clear_left_mm > 400,
		   "wall beside the obstacle limits that side's clearance");

	/* Hardware regression: one far return at 39 deg split a continuous
	   obstacle, while deeper background echoes made both clear sides look
	   blocked.  One isolated gap must be closed; two misses still mark an
	   edge, and background behind the front-plane band must be ignored. */
	n = 0;
	for (k = 0; k <= 40U; k++) {
		INT	a = -48 + (INT)k * 3;

		o[n].angle_deg = (B)a;
		o[n].status = SS_OK;
		o[n].dist_mm = (H)((a >= -39 && a <= 45 && a != 39) ? 300 : 760);
		n++;
	}
	op_build(o, n, 72, &p);
	TEST_CHECK("S5-U13b", p.status == PRF_VALID
		   && p.clear_left_mm >= AV_ROBOT_WIDTH_MM + 2 * AV_SIDE_GAP_MM
		   && p.clear_right_mm >= AV_ROBOT_WIDTH_MM + 2 * AV_SIDE_GAP_MM
		   && p.right_mm > 200,
		   "single 3-deg dropout stays in one obstacle; open sides ignore deep background");

	/* Hardware regression: extreme-angle returns can have a smaller forward
	   projection than the real obstacle even though their radial range is
	   unrelated.  They must not extend the main face to both scan limits. */
	n = s5_synth_scan(o, -150, 150, 250, 0, -72, 72, 3, 0);
	for (k = 0; k < n; k++) {
		if (iabs((W)o[k].angle_deg) >= 60) {
			o[k].status = SS_OK;
			o[k].dist_mm = 150;
		}
	}
	op_build(o, n, 72, &p);
	TEST_CHECK("S5-U13c", p.status == PRF_VALID
		   && p.clear_left_mm >= AV_ROBOT_WIDTH_MM + 2 * AV_SIDE_GAP_MM
		   && p.clear_right_mm >= AV_ROBOT_WIDTH_MM + 2 * AV_SIDE_GAP_MM,
		   "extreme-angle near echoes stay outside the symmetric front-plane band");

	/* Pico regression, 2026-10-30: phase-1 readings from scan 1.  The
	   Profile task must feed this phase on its own (not append the earlier
	   coarse grid); despite several specular dropouts it has two known
	   edges and open clearance on both sides. */
	{
		static const H hw_dist[] = {
			367, 367, 436, 376, 371, 371, 367, 432, 582, 282,
			642, 296, 296, 294, 267, 270, 261, 264, 255, 255,
			255, 255, 255, 255, 250, 250, 250, 247, 255, 255,
			255, 252, 252, 270, 267, 294, 1089, 282, 296, 296,
			264, 264, 282, 288, 261, 270, 341, 1014, 299
		};

		n = (UINT)(sizeof(hw_dist) / sizeof(hw_dist[0]));
		for (k = 0; k < n; k++) {
			o[k].angle_deg = (B)(-72 + (INT)k * 3);
			o[k].status = SS_OK;
			o[k].dist_mm = (H)s5_dist_mm(s5_dist_code((UW)hw_dist[k]));
		}
		op_build(o, n, 72, &p);
		{
			W raw_width = p.width_mm;
			W raw_left = p.clear_left_mm;
			W raw_right = p.clear_right_mm;

			op_beam_compensate(&p, S5_BEAM_COMP_DEG);
			TEST_CHECK("S5-U13d", p.status == PRF_VALID
				   && p.clear_left_mm >= AV_ROBOT_WIDTH_MM + 2 * AV_SIDE_GAP_MM
				   && p.clear_right_mm >= AV_ROBOT_WIDTH_MM + 2 * AV_SIDE_GAP_MM,
				   "captured production profile proves the 10 cm-gap corridor");
			TEST_CHECK("S5-U13e", p.status == PRF_VALID
				   && p.width_mm < raw_width && p.width_mm <= 300
				   && p.clear_left_mm >= raw_left
				   && p.clear_right_mm >= raw_right,
				   "measured beam compensation removes false width and preserves clearance");
		}
	}
	TEST_SUMMARY("s5 profile");
	return test_fail_;
}

LOCAL INT test_plan(void)
{
	S5_PROFILE	p;
	AV_PLAN		pl;
	UINT		k;
	BOOL		valid = TRUE;
	W		expect_d1;

	TEST_INIT();
	s5_profile_box(-75, 75, 300, 0, &p);
	p.clear_left_mm = 400;
	p.clear_right_mm = 900;
	TEST_CHECK("S5-U14", av_plan(&p, &pl) == AVP_OK && pl.side == AV_SIDE_RIGHT
		   && pl.n == 7 && pl.step[0].kind == AVS_TURN && pl.step[0].arg == 45
		   && pl.step[1].kind == AVS_MOVE && pl.step[2].arg == -45
		   && pl.step[3].kind == AVS_PASS && pl.step[3].arg == AV_SIDE_PASS_MAX_MM
		   && pl.step[4].arg == -30
		   && pl.step[5].kind == AVS_SEARCH && pl.step[6].arg == 30,
		   "both sides free: the side with more clearance; leave, pass, return, search, align");
	expect_d1 = (W)(((D)pl.offset_mm * 32768) / op_sin_q15(45));
	TEST_CHECK("S5-U15", pl.step[1].arg == expect_d1
		   && pl.offset_mm == p.right_mm + AV_ROBOT_WIDTH_MM / 2 + AV_SIDE_GAP_MM
		   && pl.required_mm == AV_ROBOT_WIDTH_MM + 2 * AV_SIDE_GAP_MM
		   && pl.front_gap_mm == p.dist_mm - AV_SENSOR_BEHIND_FRONT_MM
		   && pl.side_gap_mm == AV_SIDE_GAP_MM && AV_RETURN_APPROACH_MM == 160,
		   "lateral leg gives the configured robot-to-obstacle side gap");

	p.clear_left_mm = 900;
	p.clear_right_mm = 400;
	TEST_CHECK("S5-U16", av_plan(&p, &pl) == AVP_OK && pl.side == AV_SIDE_LEFT && pl.step[0].arg == -45,
		   "more clearance on the left: bypass on the left");

	p.left_mm = -400;
	p.right_mm = 400;
	p.width_mm = 800;
	p.clear_left_mm = 900;
	p.clear_right_mm = 900;
	TEST_CHECK("S5-U16b", av_plan(&p, &pl) == AVP_NO_CLEARANCE
		   && pl.offset_mm > AV_MAX_LATERAL_OFFSET_MM && pl.n == 0,
		   "implausibly large lateral offset is rejected instead of leaving the course");

	s5_profile_box(-75, 75, 120, 0, &p);			/* too close to pivot */
	p.clear_left_mm = 900;
	p.clear_right_mm = 900;
	TEST_CHECK("S5-U17", av_plan(&p, &pl) == AVP_NO_CLEARANCE && pl.n == 0,
		   "bumper gap below minimum: no plan; Safe Stop");
	p.dist_mm = 165;					/* sensor is at bumper: 165 mm gap */
	p.clear_left_mm = 400;
	p.clear_right_mm = -1;
	TEST_CHECK("S5-U17b", av_reverse_needed_mm(&p) == 55,
		   "safe side + close front requests bounded reverse to 220 mm gap");
	p.dist_mm = 60;					/* would need 160 mm (> bound) */
	TEST_CHECK("S5-U17c", av_reverse_needed_mm(&p) < 0,
		   "reverse beyond the 150 mm bound is refused");

	p.clear_left_mm = 200;
	p.clear_right_mm = -1;
	TEST_CHECK("S5-U18", av_plan(&p, &pl) == AVP_NO_CLEARANCE && pl.n == 0
		   && pl.side == AV_SIDE_NONE,
		   "no known safe side: no plan (Safe Stop), never an invented bypass");
	p.status = PRF_INSUFFICIENT;
	TEST_CHECK("S5-U19", av_plan(&p, &pl) == AVP_NO_PROFILE, "no valid profile: no plan");

	s5_profile_box(-75, 75, 300, 0, &p);
	p.clear_left_mm = 900;
	p.clear_right_mm = 900;
	(void)av_plan(&p, &pl);
	for (k = 0; k < pl.n; k++) {
		UW	type = (pl.step[k].kind == AVS_TURN) ? MCMD_TURN : MCMD_MOVE;

		if (mcmd_validate(mcmd_pack(type, MSRC_AVOID, k, 0, 6, pl.step[k].arg)) != MREJ_NONE
		    || ((pl.step[k].kind == AVS_MOVE || pl.step[k].kind == AVS_PASS)
			&& pl.step[k].arg <= 0)) {
			valid = FALSE;
		}
	}
	TEST_CHECK("S5-U20", valid,
		   "every planned step is valid and every avoidance MOVE is forward");
	TEST_SUMMARY("s5 plan");
	return test_fail_;
}

EXPORT INT s5_run_unit_tests(void)
{
	return test_codes() + test_profile() + test_plan();
}

#endif /* APP_TEST || HOST_TEST */
