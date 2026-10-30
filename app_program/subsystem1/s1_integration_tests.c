/*
 *	s1_integration_tests.c
 *	Integration tests of the binary interfaces from Buddies 1-5 to the
 *	MQTT server.  Each subsystem's status is packed with its owner's pack
 *	function and unpacked as its getter does, mapped by tlm_build() and
 *	carried through a telemetry frame:
 *	  INT-15  every field at its limit or invalid code arrives unchanged;
 *	          engineering values keep their documented resolution
 *	  INT-16  a hump measured by Buddy 4 reaches the server within +-4 mm
 *	          (Tribunal 1 success criterion), with its event frame; event
 *	          words for mode, barcode, obstacle and collision; a failed
 *	          avoidance is reported although its counter does not move
 *	The expected values are written out by hand from the field
 *	definitions, not computed by the code under test.
 *	Runs on the host (subsystem2/host_test/) and in the on-target test
 *	image.
 */

#include <tk/typedef.h>

#if APP_TEST || HOST_TEST

#include "../test_report.h"
#include "mqtt_proto.h"
#include "comm_fsm.h"
#include "tlm_build.h"
#include "s1_tests.h"
#include "../subsystem2/sys_defs.h"
#include "../subsystem2/motion_msg.h"
#include "../subsystem3/code39.h"
#include "../subsystem3/barcode_dec.h"
#include "../subsystem3/line_follow.h"
#include "../subsystem3/nav_map.h"
#include "../subsystem3/s3_status.h"
#include "../subsystem4/imu_proc.h"
#include "../subsystem4/hump.h"
#include "../subsystem4/s4_status.h"
#include "../subsystem4/s4_tests.h"
#include "../subsystem5/s5_msg.h"
#include "../subsystem5/avoid_plan.h"

#define US_NO_RESPONSE_CODE	2U	/* US_NO_RESPONSE (drv_ultrasonic.h) */

LOCAL TLM_SOURCES	src;		/* static: small test task stack */
LOCAL TLM_RECORD	rec, dec, expect, prev;
LOCAL HUMP		hmp;

LOCAL W iabs(W v)
{
	return (v < 0) ? -v : v;
}

/* Pack with the owner's function, unpack as the owner's getter does */
LOCAL void via_motion(const MOTION_STATUS *m)
{
	UW	w[2];

	mstat_pack(m, w);
	mstat_unpack(w, &src.motion);
}

LOCAL void via_imu(const S4_IMU_STATUS *s)
{
	UW	w[2];

	s4_imu_pack(s, w);
	s4_imu_unpack(w, &src.imu);
}

LOCAL void via_profile(const S5_PROFILE *p)
{
	UW	w[2];

	s5_profile_pack(p, w);
	s5_profile_unpack(w, &src.profile);
}

/* Telemetry frame through the "network": encode, decode */
LOCAL BOOL send_tlm(UB seq)
{
	UB	f[MP_MAX_FRAME];
	INT	n;

	tlm_build(&rec, &src);
	n = mp_tlm_encode(&rec, seq, f, sizeof(f));
	return n == 40 && mp_tlm_decode(f, (UINT)n, &dec) == (INT)seq;
}

LOCAL void sources_at_limits(void)
{
	MOTION_STATUS	m;
	S3_LINE_STATUS	l;
	S3_BC_STATUS	b;
	S4_IMU_STATUS	i;
	S4_HUMP_STATUS	h;
	S5_SCAN_STATUS	sc;
	S5_AVOID_STATUS	av;
	S5_PROFILE	p;
	S1_COMM_STATUS	cs;

	src.uptime_ms = 6553500UL;
	src.vsw = ((UW)VM_SAFE_STOP << VSW_MODE_SHIFT) | (7UL << VSW_EPOCH_SHIFT);
	src.faults = SYS_FLT_ALL | 0xF8000000UL;	/* nothing above bit 26 may leak */

	m.speed_l_mms = MSTAT_SPEED_INVALID;
	m.speed_r_mms = 1023;
	m.mstate = MS_FAULT;
	m.source = MSRC_SUPER;
	m.cmd_active = 1;
	m.stall_l = 1;
	m.stall_r = 1;
	m.enc_ok = 0;
	m.odometer_cm = 65535;
	m.heading_deg = 358;
	m.cruise_mms = 630;
	via_motion(&m);

	l.ir_black = 7;
	l.pos = LP_JUNCTION;
	l.state = LS_UNKNOWN;
	l.seq = 15;
	l.steer = -63;
	l.calibrated = 1;
	l.junctions = 15;
	l.lost_count = 15;
	l.speed_frac = 15;
	s3_line_unpack(s3_line_pack(&l), &src.line);
	b.state = BCS_EXECUTING;
	b.symbol = C39_NONE;
	b.result = BCR_DUPLICATE;
	b.nav = NAV_UTURN;
	b.nav_valid = 1;
	b.reversed = 1;
	b.n_valid = 63;
	b.n_invalid = 63;
	b.mres = MRES_REJECTED;
	b.executed = 1;
	b.err_class = S3_BCE_ABORT;
	s3_bc_unpack(s3_bc_pack(&b), &src.barcode);

	i.pitch_deg = S4_ANGLE_INVALID;
	i.roll_deg = 90;
	i.yaw_rate_dps = S4_RATE_INVALID;
	i.motion_class = MCLS_TURN_RIGHT;
	i.status = IMU_ST_FAULT;
	i.tilt = 1;
	i.collision = 1;
	i.valid = 0;
	i.heading_deg = S4_HEADING_INVALID;
	i.shock_half_g = 31;
	i.seq = 255;
	i.chip = 2;
	i.calibrated = 1;
	via_imu(&i);
	h.last_code = HUMP_CODE_NONE;
	h.max_code = HUMP_CODE_MAX;
	h.count = 15;
	h.active = 1;
	h.last_sat = 0;
	h.max_sat = 1;
	h.last_len_cm = 255;
	s4_hump_unpack(s4_hump_pack(&h), &src.hump);

	sc.dist_code = S5_DIST_NO_ECHO;
	sc.servo_deg = -80;
	sc.us_result = US_NO_RESPONSE_CODE;
	sc.mode = SREQ_IDLE;
	sc.seq = 15;
	sc.cap = 1;
	sc.detected = 1;
	s5_scan_status_unpack(s5_scan_status_pack(&sc), &src.scan);
	av.state = AVST_FAILED;
	av.side = AV_SIDE_RIGHT;
	av.step = 7;
	av.retries = 3;
	av.outcome = AVO_FAILED;
	av.count = 15;
	av.fail = AVF_SENSOR;
	s5_avoid_status_unpack(s5_avoid_status_pack(&av), &src.avoid);
	p.scan_id = 7;
	p.status = PRF_INSUFFICIENT;
	p.phase = 1;
	p.dist_mm = -1;
	p.bearing_deg = -127;
	p.width_mm = -1;
	p.left_mm = -1270;
	p.right_mm = 1270;
	p.clear_left_mm = -1;
	p.clear_right_mm = 3000;		/* beyond the 2.54 m code */
	via_profile(&p);

	cs.state = CS_DISABLED;
	cs.avail = CAV_RECOVERING;
	cs.link = CL_FAILED;
	cs.session = CSS_LOST;
	cs.attempts = 15;
	cs.reconnects = 15;
	cs.tlm_dropped = 0;
	cs.rx_rejected = 0;
	s1_comm_unpack(s1_comm_pack(&cs), &src.comm);
	src.comm.tlm_dropped = 255;		/* separate single-writer words */
	src.comm.rx_rejected = 255;
}

/* What the server must decode for sources_at_limits(), by hand */
LOCAL void expected_at_limits(TLM_RECORD *e)
{
	e->uptime_ds = 65535;	e->mode = 4;		e->epoch = 7;
	e->faults = 0x7FFFFFFUL;
	e->mstate = 3;		e->msource = 3;		e->cmd_active = 1;
	e->speed_l = -1024;	e->speed_r = 1023;	e->odometer_cm = 65535;
	e->heading2 = 179;	e->cruise10 = 63;
	e->ir_black = 7;	e->line_pos = 6;	e->line_state = 3;
	e->steer = -63;		e->bc_state = 3;	e->bc_symbol = 63;
	e->bc_result = 3;	e->nav = 3;		e->nav_valid = 1;
	e->bc_valid = 63;	e->bc_invalid = 63;
	e->imu_status = 3;	e->pitch = -128;	e->roll = 90;
	e->yaw_rate = -256;	e->heading = 511;	e->motion_class = 3;
	e->tilt = 1;		e->collision = 1;	e->shock = 31;
	e->hump_last = 63;	e->hump_max = 50;	e->hump_count = 15;
	e->hump_active = 1;
	e->us_dist = 1022;	e->us_result = 2;	e->avoid_state = 8;
	e->avoid_side = 2;	e->avoid_outcome = 2;	e->obst_width = 255;
	e->obst_bearing = -127;	e->clear_left = 255;	e->clear_right = 254;
	e->comm_state = 7;	e->tlm_dropped = 255;	e->rx_rejected = 255;
}

LOCAL INT int_binary_interfaces(void)
{
	MOTION_STATUS	m;
	S4_IMU_STATUS	i;
	S5_SCAN_STATUS	sc;
	S5_PROFILE	p;
	BOOL		sent, scaled;

	TEST_INIT();
	sources_at_limits();
	expected_at_limits(&expect);
	sent = send_tlm(0x5A);
	TEST_CHECK("INT-15a", sent && s1_tlm_equal(&dec, &expect),
		   "Buddies 1-5 at their limits and invalid codes: all 48 fields reach the server unchanged");

	/* engineering values: the documented resolution is the only loss */
	m = src.motion;
	m.speed_l_mms = 187;
	m.speed_r_mms = -45;
	m.heading_deg = 271;
	m.cruise_mms = 205;
	via_motion(&m);
	i = src.imu;
	i.pitch_deg = s4_deg8(1249);		/* 12.49 deg */
	i.yaw_rate_dps = s4_rate9(45500);	/* 45.5 dps */
	i.heading_deg = 271;
	via_imu(&i);
	sc = src.scan;
	sc.dist_code = s5_dist_code(312);
	s5_scan_status_unpack(s5_scan_status_pack(&sc), &src.scan);
	p.scan_id = 1;
	p.status = PRF_VALID;
	p.phase = 1;
	p.dist_mm = 250;
	p.bearing_deg = -12;
	p.width_mm = 152;
	p.left_mm = -90;
	p.right_mm = 62;
	p.clear_left_mm = 612;
	p.clear_right_mm = 1234;
	via_profile(&p);
	sent = send_tlm(0x5B);
	scaled = dec.speed_l == 187 && dec.speed_r == -45
		 && iabs((W)dec.heading2 * 2 - 271) <= 1 && dec.cruise10 == 20
		 && dec.pitch == 12 && dec.yaw_rate == 46 && dec.heading == 271U
		 && iabs(s5_dist_mm(dec.us_dist) - 312) <= 3
		 && iabs((W)dec.obst_width * 5 - 152) <= 3 && dec.obst_bearing == -12
		 && iabs((W)dec.clear_left * 10 - 612) <= 5
		 && iabs((W)dec.clear_right * 10 - 1234) <= 5;
	TEST_CHECK("INT-15b", sent && scaled,
		   "engineering values arrive within their step: 2 deg, 10 mm/s, 1 deg, 5 mm, 10 mm");
	TEST_SUMMARY("int binary interfaces");
	return test_fail_;
}

LOCAL INT int_hump_telemetry(void)
{
	HUMP_EVT	ev;
	EVT_RECORD	evs[TLM_MAX_EVENTS], e;
	S4_HUMP_STATUS	hs;
	UB		f[MP_MAX_FRAME];
	UINT		ne, k;
	INT		n;
	W		last_um, max_um;
	UB		avoid_count0;
	BOOL		first, three, evt_ok, others, quiet;

	TEST_INIT();
	sources_at_limits();
	src.vsw = (UW)VM_NORMAL << VSW_MODE_SHIFT;
	src.faults = 0;
	src.imu.collision = 0;

	hump_init(&hmp);
	s4_drive_hump(&hmp, 30, 300, &ev);		/* Buddy 4 measures */
	hump_status(&hmp, &hs);
	s4_hump_unpack(s4_hump_pack(&hs), &src.hump);	/* hump status word */
	first = send_tlm(1) && ev.completed;
	last_um = hump_code_to_um(dec.hump_last);
	first = first && iabs(last_um - 30000) <= 4000 && dec.hump_count == 1U;
	TEST_CHECK("INT-16a", first,
		   "30 mm hump measured by Buddy 4 arrives at the server within +-4 mm");

	prev = dec;
	s4_drive_hump(&hmp, 20, 250, &ev);
	s4_drive_hump(&hmp, 45, 300, &ev);
	hump_status(&hmp, &hs);
	s4_hump_unpack(s4_hump_pack(&hs), &src.hump);
	three = send_tlm(2);
	max_um = hump_code_to_um(dec.hump_max);
	last_um = hump_code_to_um(dec.hump_last);
	three = three && dec.hump_count == 3U && iabs(max_um - 45000) <= 4000
		&& iabs(last_um - 45000) <= 4000;
	TEST_CHECK("INT-16b", three, "after humps of 30, 20, 45 mm: count 3, highest 45 mm within +-4 mm");

	ne = tlm_events(&prev, src.avoid.count, &dec, &src, evs);
	evt_ok = ne == 1U && evs[0].code == EVT_HUMP;
	if (evt_ok) {
		n = mp_evt_encode(&evs[0], 7, f, sizeof(f));
		evt_ok = n == 10 && mp_evt_decode(f, (UINT)n, &e) == 7 && e.code == EVT_HUMP
			 && (e.data & 0x3FUL) == dec.hump_last
			 && ((e.data >> 6) & 0x3FUL) == dec.hump_max
			 && ((e.data >> 12) & 0xFUL) == 3UL;
	}
	TEST_CHECK("INT-16c", evt_ok, "one EVT_HUMP frame carries last, highest and count");

	prev = dec;					/* several changes at once */
	avoid_count0 = src.avoid.count;
	src.vsw = ((UW)VM_OBSTACLE << VSW_MODE_SHIFT) | (2UL << VSW_EPOCH_SHIFT);
	src.barcode.n_valid = (UB)((src.barcode.n_valid + 1U) & 0x3FU);
	src.barcode.symbol = (UB)c39_index_of_char('Z');
	src.barcode.result = BCR_VALID;
	src.barcode.nav = NAV_LEFT;
	src.barcode.nav_valid = 1;
	src.imu.collision = 1;
	src.imu.shock_half_g = 5;
	src.avoid.count = (UB)((avoid_count0 + 1U) & 0xFU);
	src.avoid.outcome = AVO_SUCCESS;
	src.avoid.fail = AVF_NONE;
	src.avoid.side = AV_SIDE_LEFT;
	src.profile.dist_mm = 250;
	src.profile.width_mm = 150;
	(void)send_tlm(3);
	ne = tlm_events(&prev, avoid_count0, &dec, &src, evs);
	others = (ne == 4U);
	for (k = 0; others && k < ne; k++) {
		UW	d = evs[k].data;

		switch (evs[k].code) {
		case EVT_MODE:
			others = (d & 7UL) == VM_NORMAL && ((d >> 3) & 7UL) == VM_OBSTACLE
				 && ((d >> 6) & 7UL) == 2UL;
			break;
		case EVT_BARCODE:
			others = (d & 0x3FUL) == (UW)c39_index_of_char('Z')
				 && ((d >> 6) & 3UL) == BCR_VALID && ((d >> 8) & 3UL) == NAV_LEFT
				 && ((d >> 10) & 1UL) == 1UL;
			break;
		case EVT_OBSTACLE:
			others = (d & 3UL) == AVO_SUCCESS && ((d >> 2) & 7UL) == AVF_NONE
				 && ((d >> 5) & 3UL) == AV_SIDE_LEFT && ((d >> 7) & 0xFFUL) == 30UL
				 && ((d >> 15) & 0x3FFUL) == 50UL;
			break;
		case EVT_COLLISION:
			others = d == 5UL;
			break;
		default:
			others = FALSE;
			break;
		}
	}
	prev = dec;
	(void)send_tlm(4);
	quiet = tlm_events(&prev, src.avoid.count, &dec, &src, evs) == 0U;
	TEST_CHECK("INT-16d", others && quiet,
		   "mode, barcode, obstacle and collision events decode to their fields; no change, no event");

	prev = dec;					/* a failure leaves the counter alone */
	src.avoid.state = AVST_EXECUTING;
	(void)send_tlm(5);
	quiet = tlm_events(&prev, src.avoid.count, &dec, &src, evs) == 0U;
	prev = dec;
	src.avoid.state = AVST_FAILED;
	src.avoid.outcome = AVO_FAILED;
	src.avoid.fail = AVF_CLEARANCE;
	(void)send_tlm(6);
	ne = tlm_events(&prev, src.avoid.count, &dec, &src, evs);
	others = ne == 1U && evs[0].code == EVT_OBSTACLE && (evs[0].data & 3UL) == AVO_FAILED
		 && ((evs[0].data >> 2) & 7UL) == AVF_CLEARANCE;
	prev = dec;
	(void)send_tlm(7);
	others = others && tlm_events(&prev, src.avoid.count, &dec, &src, evs) == 0U;
	TEST_CHECK("INT-16e", quiet && others,
		   "a failed avoidance (counter unchanged) sends one OBSTACLE event with its reason");
	TEST_SUMMARY("int hump telemetry");
	return test_fail_;
}

EXPORT INT s1_run_integration_tests(void)
{
	return int_binary_interfaces() + int_hump_telemetry();
}

#endif /* APP_TEST || HOST_TEST */
