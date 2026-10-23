/*
 *	s4_unit_tests.c
 *	Unit tests of subsystem 4's pure logic: integer maths, calibration,
 *	filtering, tilt, collision, motion classification, data validation,
 *	hump detection / height / maximum, quantisation and packing.
 *	Runs on the host (host_test/) and in the on-target test image.
 *	Test IDs: S4-Uxx.
 */

#include <tk/typedef.h>

#if APP_TEST || HOST_TEST

#include "../test_report.h"
#include "imu_proc.h"
#include "hump.h"
#include "s4_status.h"
#include "s4_tests.h"

LOCAL W iabs(W v)
{
	return (v < 0) ? -v : v;
}

LOCAL UH sample_no;

EXPORT void s4_make_in(IMU_IN *s, W ax, W ay, W az, W gx, W gy, W gz)
{
	s->acc_mg[0] = ax;
	s->acc_mg[1] = ay;
	s->acc_mg[2] = az;
	s->gyr_mdps[0] = gx;
	s->gyr_mdps[1] = gy;
	s->gyr_mdps[2] = gz;
	s->saturated = FALSE;
	s->raw_hash = ++sample_no;		/* a live sensor always changes */
}

/* Pitch along a raised-cosine hump of height h_mm and length l_mm at
   distance s_mm into it (point model: pitch = atan(dh/ds)) */
EXPORT W s4_hump_pitch_cdeg(W h_mm, W l_mm, W s_mm)
{
	W	ang = (W)((D)36000 * s_mm / l_mm);			/* 2*pi*s/L */
	W	slope_milli = (W)((D)h_mm * 3142 * imath_sin_q15(ang) / ((D)l_mm * 32768));

	return imath_atan2_cdeg(slope_milli, 1000);
}

/* Drive over one hump at 200 mm/s (2 mm per 10 ms sample) with level
   track before and after; returns the event of the run */
EXPORT void s4_drive_hump(HUMP *h, W h_mm, W l_mm, HUMP_EVT *last)
{
	HUMP_EVT	ev;
	W		s;

	last->completed = FALSE;
	last->rejected = FALSE;
	for (s = -100; s < l_mm + 100; s += 2) {
		W	p = (s < 0 || s > l_mm) ? 0 : s4_hump_pitch_cdeg(h_mm, l_mm, s);

		hump_step(h, p, 2000, TRUE, &ev);
		if (ev.completed || ev.rejected) {
			*last = ev;
		}
	}
}

LOCAL INT test_math(void)
{
	TEST_INIT();
	TEST_CHECK("S4-U01", imath_isqrt(0) == 0 && imath_isqrt(1) == 1 && imath_isqrt(3) == 1
		   && imath_isqrt(4) == 2 && imath_isqrt(65535) == 255 && imath_isqrt(65536) == 256
		   && imath_isqrt(0xFFFFFFFFUL) == 65535,
		   "integer square root exact (floor) incl. 32-bit maximum");
	TEST_CHECK("S4-U02", imath_sin_q15(0) == 0 && iabs(imath_sin_q15(3000) - 16384) < 70
		   && iabs(imath_sin_q15(9000) - 32768) < 70 && iabs(imath_sin_q15(-3000) + 16384) < 70
		   && iabs(imath_sin_q15(18000)) < 70 && iabs(imath_sin_q15(27000) + 32768) < 70,
		   "sine within 0.002 at 0/30/90/-30/180/270 deg");
	TEST_CHECK("S4-U03", imath_atan2_cdeg(0, 1000) == 0 && iabs(imath_atan2_cdeg(1000, 1000) - 4500) < 30
		   && imath_atan2_cdeg(1000, 0) == 9000 && iabs(imath_atan2_cdeg(-1000, -1000) + 13500) < 30
		   && iabs(imath_atan2_cdeg(173648, 984808) - 1000) < 30,
		   "atan2 within 0.3 deg (0, 45, 90, -135, 10 deg)");
	TEST_SUMMARY("s4 math");
	return test_fail_;
}

LOCAL INT test_calibration(void)
{
	IMU_CAL_ACC	c;
	IMU_CAL		cal;
	IMU_IN		s;
	UINT		k;
	INT		r;

	TEST_INIT();
	imu_cal_reset(&c);
	for (k = 0; k < IMU_CAL_SAMPLES; k++) {
		W	n = (k & 1U) ? 5 : -5;

		s4_make_in(&s, 12 + n, -8 - n, 1003 + n, 150 + 40 * n, -220 - 40 * n, 80);
		imu_cal_add(&c, &s);
	}
	r = imu_cal_finish(&c, &cal);
	TEST_CHECK("S4-U04", r == IMU_CAL_OK && cal.valid && iabs(cal.gyro_bias[0] - 150) <= 2
		   && iabs(cal.gyro_bias[1] + 220) <= 2 && iabs(cal.gyro_bias[2] - 80) <= 2
		   && iabs(cal.acc_offset[0] - 12) <= 1 && iabs(cal.acc_offset[1] + 8) <= 1
		   && iabs(cal.acc_offset[2] - 3) <= 1,
		   "stationary calibration recovers gyro bias and level offsets");

	imu_cal_reset(&c);
	for (k = 0; k < IMU_CAL_SAMPLES; k++) {
		s4_make_in(&s, 0, 0, 1000, (k & 1U) ? 3000 : -3000, 0, 0);
		imu_cal_add(&c, &s);
	}
	TEST_CHECK("S4-U05", imu_cal_finish(&c, &cal) == IMU_CAL_NOT_STILL && !cal.valid,
		   "car moving during calibration: rejected (retry)");

	imu_cal_reset(&c);
	for (k = 0; k < IMU_CAL_SAMPLES; k++) {
		s4_make_in(&s, 1000, 0, 0, 0, 0, 0);
		imu_cal_add(&c, &s);
	}
	TEST_CHECK("S4-U06", imu_cal_finish(&c, &cal) == IMU_CAL_ORIENTATION,
		   "gravity not on the configured up axis: calibration failure");

	imu_cal_reset(&c);
	TEST_CHECK("S4-U07", imu_cal_finish(&c, &cal) == IMU_CAL_NO_DATA, "no samples: failure");
	TEST_SUMMARY("s4 calibration");
	return test_fail_;
}

LOCAL void level_cal(IMU_CAL *cal)
{
	UINT	i;

	for (i = 0; i < 3U; i++) {
		cal->gyro_bias[i] = 0;
		cal->acc_offset[i] = 0;
	}
	cal->valid = TRUE;
}

LOCAL INT test_monitoring(void)
{
	IMU_PROC	p;
	IMU_CAL		cal;
	IMU_IN		s;
	UINT		k;
	BOOL		tilt_early, coll1, coll2;
	W		max_dev = 0;

	TEST_INIT();
	level_cal(&cal);

	/* static 10 deg nose-up: gravity (fwd 174 mg, up 985 mg) */
	imu_proc_init(&p, &cal);
	for (k = 0; k < 300U; k++) {
		s4_make_in(&s, 0, 174, 985, 0, 0, 0);
		(void)imu_proc_step(&p, &s, 10);
	}
	TEST_CHECK("S4-U08", iabs(p.pitch_cdeg - 1000) <= 50 && iabs(p.roll_cdeg) <= 50,
		   "static 10 deg pitch measured within 0.5 deg");

	/* one acceleration spike while level must not move the pitch much */
	imu_proc_init(&p, &cal);
	for (k = 0; k < 50U; k++) {
		s4_make_in(&s, 0, (k == 25U) ? 600 : 0, 1000, 0, 0, 0);
		(void)imu_proc_step(&p, &s, 10);
		if (iabs(p.pitch_cdeg) > max_dev) {
			max_dev = iabs(p.pitch_cdeg);
		}
	}
	TEST_CHECK("S4-U09", max_dev <= 100, "short braking/bump spike: pitch error < 1 deg (filter)");

	/* tilt needs > 15 deg for >= 200 ms */
	imu_proc_init(&p, &cal);
	for (k = 0; k < 15U; k++) {
		s4_make_in(&s, -342, 0, 940, 0, 0, 0);		/* 20 deg roll */
		(void)imu_proc_step(&p, &s, 10);
	}
	tilt_early = p.tilt;
	for (k = 0; k < 10U; k++) {
		s4_make_in(&s, -342, 0, 940, 0, 0, 0);
		(void)imu_proc_step(&p, &s, 10);
	}
	TEST_CHECK("S4-U10", !tilt_early && p.tilt && iabs(p.roll_cdeg - 2000) < 100,
		   "tilt event only after 200 ms beyond 15 deg (no false tilt)");

	/* collision: 2.5 g horizontal impact, refractory afterwards */
	imu_proc_init(&p, &cal);
	s4_make_in(&s, 0, 0, 1000, 0, 0, 0);
	(void)imu_proc_step(&p, &s, 10);
	s4_make_in(&s, 0, -2500, 1000, 0, 0, 0);
	(void)imu_proc_step(&p, &s, 10);
	coll1 = p.collision;
	s4_make_in(&s, 0, -2500, 1000, 0, 0, 0);
	(void)imu_proc_step(&p, &s, 10);
	coll2 = p.collision;
	TEST_CHECK("S4-U11", coll1 && !coll2 && p.peak_shock_mg >= 2400,
		   "2.5 g impact detected once (refractory), peak recorded");
	imu_proc_init(&p, &cal);
	s4_make_in(&s, 0, 0, 1000, 0, 0, 0);
	(void)imu_proc_step(&p, &s, 10);
	s4_make_in(&s, 0, 900, 1000, 0, 0, 0);
	(void)imu_proc_step(&p, &s, 10);
	TEST_CHECK("S4-U12", !p.collision, "0.9 g bump below the collision threshold");

	/* motion classification */
	imu_proc_init(&p, &cal);
	for (k = 0; k < 40U; k++) {
		s4_make_in(&s, 0, 0, 1000, 0, 0, 0);
		(void)imu_proc_step(&p, &s, 10);
	}
	TEST_CHECK("S4-U13", p.motion_class == MCLS_STATIONARY, "still for 300 ms: STATIONARY");
	s4_make_in(&s, 0, 0, 1000, 0, 0, 30000);
	(void)imu_proc_step(&p, &s, 10);
	TEST_CHECK("S4-U14", p.motion_class == MCLS_TURN_LEFT && p.yaw_rate_mdps == 30000,
		   "+30 dps yaw: TURN_LEFT, turn rate reported");
	s4_make_in(&s, 0, 0, 1000, 0, 0, -30000);
	(void)imu_proc_step(&p, &s, 10);
	TEST_CHECK("S4-U15", p.motion_class == MCLS_TURN_RIGHT, "-30 dps yaw: TURN_RIGHT");
	s4_make_in(&s, 0, 200, 1000, 4000, 0, 0);
	(void)imu_proc_step(&p, &s, 10);
	TEST_CHECK("S4-U16", p.motion_class == MCLS_MOVING, "vibration/acceleration: MOVING");

	/* data validation: stuck sensor and saturation */
	imu_proc_init(&p, &cal);
	for (k = 0; k < 40U; k++) {
		s4_make_in(&s, 0, 0, 1000, 0, 0, 0);
		s.raw_hash = 1234;			/* bus returns the same bytes */
		(void)imu_proc_step(&p, &s, 10);
	}
	TEST_CHECK("S4-U17", imu_proc_failed(&p), "frozen raw data -> invalid run -> IMU fault");
	imu_proc_init(&p, &cal);
	s4_make_in(&s, 0, 0, 1000, 0, 0, 0);
	s.saturated = TRUE;
	TEST_CHECK("S4-U18", !imu_proc_step(&p, &s, 10) && p.invalid_run == 1,
		   "saturated sample rejected, never used");
	TEST_SUMMARY("s4 monitoring");
	return test_fail_;
}

LOCAL INT test_hump(void)
{
	HUMP		h;
	HUMP_EVT	ev, e2;
	W		s;
	BOOL		any = FALSE, sat;
	UINT		k;

	TEST_INIT();
	hump_init(&h);
	s4_drive_hump(&h, 40, 300, &ev);
	TEST_CHECK("S4-U19", ev.completed && iabs(ev.height_um - 40000) <= 4000 && h.count == 1,
		   "40 mm hump measured within +-4 mm (Tribunal 1 success criterion)");

	hump_init(&h);
	s4_drive_hump(&h, 20, 250, &ev);
	s4_drive_hump(&h, 40, 300, &ev);
	s4_drive_hump(&h, 30, 300, &e2);
	TEST_CHECK("S4-U20", h.count == 3 && iabs(h.max_height_um - 40000) <= 4000
		   && iabs(h.last_height_um - 30000) <= 4000,
		   "highest hump of the run tracked at full resolution");

	hump_init(&h);
	for (s = 0; s < 3000; s += 2) {
		W	noise = ((s / 2) % 7 - 3) * 40;		/* +-1.2 deg vibration */

		hump_step(&h, noise, 2000, TRUE, &ev);
		any = any || ev.completed;
	}
	TEST_CHECK("S4-U21", !any && h.count == 0, "flat track with vibration: no hump");

	hump_init(&h);
	any = FALSE;
	for (s = 0; s < 1600; s += 2) {
		hump_step(&h, 500, 2000, TRUE, &ev);		/* 5 deg ramp */
		any = any || ev.completed;
	}
	TEST_CHECK("S4-U22", !any && h.count == 0, "long ramp (> 1 m) is not reported as a hump");

	hump_init(&h);
	any = FALSE;
	for (s = 0; s < 400; s += 2) {
		W	p = (s > 300) ? 0 : s4_hump_pitch_cdeg(40, 300, s);

		hump_step(&h, p, 2000, (s != 150) ? TRUE : FALSE, &ev);
		any = any || ev.completed;
	}
	TEST_CHECK("S4-U23", !any, "invalid sample mid-hump: measurement discarded");

	TEST_CHECK("S4-U24", hump_code(0, &sat) == 0 && hump_code(999, &sat) == 0
		   && hump_code(1000, &sat) == 1 && hump_code(100000, &sat) == HUMP_CODE_MAX && !sat
		   && hump_code(101000, &sat) == HUMP_CODE_MAX && sat
		   && hump_code(-1, &sat) == HUMP_CODE_NONE
		   && hump_code_to_um(50) == 100000 && hump_code_to_um(HUMP_CODE_NONE) == -1,
		   "6-bit hump code: 2 mm steps, saturation flag, 63 = none");
	sat = FALSE;
	for (k = 0; k <= 100000; k += 250) {
		if (iabs(hump_code_to_um(hump_code((W)k, &sat)) - (W)k) > 1000) {
			sat = TRUE;
			break;
		}
		sat = FALSE;
	}
	TEST_CHECK("S4-U25", !sat, "quantisation error <= 1 mm over 0..100 mm");
	TEST_SUMMARY("s4 hump");
	return test_fail_;
}

LOCAL INT test_packing(void)
{
	S4_IMU_STATUS	a, b;
	S4_HUMP_STATUS	c, d;
	S4_HUMP_SAMPLE	e, f;
	UW		w[2];

	TEST_INIT();
	a.pitch_deg = -90;
	a.roll_deg = S4_ANGLE_INVALID;
	a.yaw_rate_dps = S4_RATE_INVALID;
	a.motion_class = MCLS_TURN_RIGHT;
	a.status = IMU_ST_FAULT;
	a.tilt = 1;
	a.collision = 1;
	a.valid = 0;
	a.heading_deg = 359;
	a.shock_half_g = 31;
	a.seq = 255;
	a.chip = 2;
	a.calibrated = 1;
	s4_imu_pack(&a, w);
	s4_imu_unpack(w, &b);
	TEST_CHECK("S4-U26", b.pitch_deg == -90 && b.roll_deg == S4_ANGLE_INVALID
		   && b.yaw_rate_dps == S4_RATE_INVALID && b.motion_class == MCLS_TURN_RIGHT
		   && b.status == IMU_ST_FAULT && b.tilt == 1 && b.collision == 1 && b.valid == 0
		   && b.heading_deg == 359 && b.shock_half_g == 31 && b.seq == 255
		   && b.chip == 2 && b.calibrated == 1,
		   "IMU status words round trip at limits and invalid codes");
	c.last_code = HUMP_CODE_NONE;
	c.max_code = 50;
	c.count = 200;
	c.active = 1;
	c.last_sat = 0;
	c.max_sat = 1;
	c.last_len_cm = 255;
	s4_hump_unpack(s4_hump_pack(&c), &d);
	TEST_CHECK("S4-U27", d.last_code == HUMP_CODE_NONE && d.max_code == 50 && d.count == 15
		   && d.active == 1 && d.max_sat == 1 && d.last_len_cm == 255,
		   "hump status word (count saturates at 15)");
	e.pitch_ddeg = -2000;
	e.ds_100um = 255;
	e.valid = 1;
	e.seq = 15;
	s4_sample_unpack(s4_sample_pack(&e), &f);
	TEST_CHECK("S4-U28", f.pitch_ddeg == -1023 && f.ds_100um == 255 && f.valid == 1 && f.seq == 15,
		   "IMU->hump queue message: pitch clamped to 11-bit range");
	TEST_CHECK("S4-U29", s4_deg8(-9049) == -90 && s4_deg8(9051) == 90 && s4_deg8(149) == 1
		   && s4_rate9(300000) == 255 && s4_rate9(-1499) == -1 && s4_shock5(20000) == 31
		   && s4_shock5(1250) == 3,
		   "compaction rounds to nearest and saturates");
	TEST_SUMMARY("s4 packing");
	return test_fail_;
}

EXPORT INT s4_run_unit_tests(void)
{
	return test_math() + test_calibration() + test_monitoring() + test_hump()
	     + test_packing();
}

#endif /* APP_TEST || HOST_TEST */
