/*
 *	imu_proc.c
 *	IMU processing (see imu_proc.h).
 */

#include "imu_proc.h"

#define ONE_G_MG	1000

LOCAL W iabs(W v)
{
	return (v < 0) ? -v : v;
}

/* ------------------------------------------------------------------ */
/*  Integer maths                                                      */

EXPORT UW imath_isqrt(UW v)
{
	UW	res = 0, bit = 1UL << 30;

	while (bit > v) {
		bit >>= 2;
	}
	while (bit != 0U) {
		if (v >= res + bit) {
			v -= res + bit;
			res = (res >> 1) + bit;
		} else {
			res >>= 1;
		}
		bit >>= 2;
	}
	return res;
}

/* atan(z), 0 <= z <= 1 in Q15, result in centi-degrees.
   atan(z) ~ (pi/4) z + 0.273 z (1 - z)  [rad], max error ~0.22 deg. */
LOCAL W atan_q15_cdeg(W z)
{
	D	lin = (D)4500 * z;				/* 45 deg * z  */
	D	cor = (D)1564 * z * (32768 - z) / 32768;	/* 15.64 deg * z(1-z) */

	return (W)((lin + cor) / 32768);
}

EXPORT W imath_atan2_cdeg(W y, W x)
{
	W	ax = iabs(x), ay = iabs(y);
	W	a;

	if (ax == 0 && ay == 0) {
		return 0;
	}
	if (ay <= ax) {
		a = atan_q15_cdeg((W)(((D)ay << 15) / ax));
	} else {
		a = 9000 - atan_q15_cdeg((W)(((D)ax << 15) / ay));
	}
	if (x < 0) {
		a = 18000 - a;
	}
	return (y < 0) ? -a : a;
}

/* Bhaskara I approximation on 0..180 deg, error < 0.002 */
EXPORT W imath_sin_q15(W cdeg)
{
	W	x = cdeg % 36000;
	W	sign = 1;
	D	p;

	if (x < 0) {
		x += 36000;
	}
	if (x > 18000) {
		x -= 18000;
		sign = -1;
	}
	p = (D)x * (18000 - x);				/* <= 81e6 */
	return sign * (W)(((D)4 * p * 32768) / ((D)405000000 - p));
}

/* ------------------------------------------------------------------ */
/*  Axis mapping and calibration                                       */

EXPORT void imu_to_body(const IMU_IN *s, W a[3], W g[3])
{
	a[AX_RIGHT] = IMU_SIGN_RIGHT * s->acc_mg[IMU_MAP_RIGHT];
	a[AX_FWD]   = IMU_SIGN_FWD * s->acc_mg[IMU_MAP_FWD];
	a[AX_UP]    = IMU_SIGN_UP * s->acc_mg[IMU_MAP_UP];
	g[AX_RIGHT] = IMU_SIGN_RIGHT * s->gyr_mdps[IMU_MAP_RIGHT];
	g[AX_FWD]   = IMU_SIGN_FWD * s->gyr_mdps[IMU_MAP_FWD];
	g[AX_UP]    = IMU_SIGN_UP * s->gyr_mdps[IMU_MAP_UP];
}

EXPORT void imu_mag_to_body(const IMU_IN *s, W m[3])
{
	static const W	off[3] = { IMU_MAG_OFF_X, IMU_MAG_OFF_Y, IMU_MAG_OFF_Z };

	m[AX_RIGHT] = IMU_SIGN_RIGHT * (s->mag_mgauss[IMU_MAP_RIGHT] - off[IMU_MAP_RIGHT]);
	m[AX_FWD]   = IMU_SIGN_FWD * (s->mag_mgauss[IMU_MAP_FWD] - off[IMU_MAP_FWD]);
	m[AX_UP]    = IMU_SIGN_UP * (s->mag_mgauss[IMU_MAP_UP] - off[IMU_MAP_UP]);
}

EXPORT void imu_cal_reset(IMU_CAL_ACC *c)
{
	UINT	i;

	for (i = 0; i < 3U; i++) {
		c->sum_a[i] = 0;
		c->sum_g[i] = 0;
		c->min_a[i] = 0x7FFFFFFF;
		c->max_a[i] = -0x7FFFFFFF;
		c->min_g[i] = 0x7FFFFFFF;
		c->max_g[i] = -0x7FFFFFFF;
	}
	c->n = 0;
}

EXPORT void imu_cal_add(IMU_CAL_ACC *c, const IMU_IN *s)
{
	W	a[3], g[3];
	UINT	i;

	if (s->saturated) {
		return;
	}
	imu_to_body(s, a, g);
	for (i = 0; i < 3U; i++) {
		c->sum_a[i] += a[i];
		c->sum_g[i] += g[i];
		if (a[i] < c->min_a[i]) c->min_a[i] = a[i];
		if (a[i] > c->max_a[i]) c->max_a[i] = a[i];
		if (g[i] < c->min_g[i]) c->min_g[i] = g[i];
		if (g[i] > c->max_g[i]) c->max_g[i] = g[i];
	}
	c->n++;
}

EXPORT INT imu_cal_finish(const IMU_CAL_ACC *c, IMU_CAL *cal)
{
	W	mean_a[3];
	UINT	i;

	cal->valid = FALSE;
	if (c->n < IMU_CAL_SAMPLES / 2U) {
		return IMU_CAL_NO_DATA;
	}
	for (i = 0; i < 3U; i++) {
		if (c->max_g[i] - c->min_g[i] > IMU_CAL_GYRO_RANGE
		    || c->max_a[i] - c->min_a[i] > IMU_CAL_ACC_RANGE) {
			return IMU_CAL_NOT_STILL;	/* car moved: try again */
		}
		mean_a[i] = c->sum_a[i] / (W)c->n;
		cal->gyro_bias[i] = c->sum_g[i] / (W)c->n;
	}
	if (mean_a[AX_UP] < IMU_CAL_UP_MIN_MG
	    || iabs(mean_a[AX_UP] - ONE_G_MG) > IMU_CAL_1G_TOL_MG) {
		return IMU_CAL_ORIENTATION;		/* axis map wrong / upside down */
	}
	/* Level reference: the car stands on the flat track while it
	   calibrates, so the mean horizontal components are mounting offset */
	cal->acc_offset[AX_RIGHT] = mean_a[AX_RIGHT];
	cal->acc_offset[AX_FWD] = mean_a[AX_FWD];
	cal->acc_offset[AX_UP] = mean_a[AX_UP] - ONE_G_MG;
	cal->valid = TRUE;
	return IMU_CAL_OK;
}

/* ------------------------------------------------------------------ */
/*  Monitoring                                                         */

EXPORT void imu_proc_init(IMU_PROC *p, const IMU_CAL *cal)
{
	UINT	i;

	p->cal = *cal;
	for (i = 0; i < 3U; i++) {
		p->a[i] = 0;
		p->g[i] = 0;
	}
	p->pitch_cdeg = 0;
	p->roll_cdeg = 0;
	p->yaw_cdeg = 0;
	p->yaw_rate_mdps = 0;
	p->yaw_valid = FALSE;
	p->pitch_hist[0] = p->pitch_hist[1] = 0;
	p->roll_hist[0] = p->roll_hist[1] = 0;
	p->pitch_med_cdeg = 0;
	p->roll_med_cdeg = 0;
	p->mag_ref_cdeg = 0;
	p->mag_primed = FALSE;
	for (i = 0; i < IMU_MAG_RATE_WIN; i++) {
		p->yaw_hist[i] = 0;
	}
	p->yaw_idx = 0;
	p->yaw_n = 0;
	p->lin_fwd_mg = 0;
	p->lin_right_mg = 0;
	p->motion_class = MCLS_STATIONARY;
	p->tilt = FALSE;
	p->tilt_ms = 0;
	p->still_ms = 0;
	p->collision = FALSE;
	p->collision_hold_ms = 0;
	p->peak_shock_mg = 0;
	p->last_hash = 0;
	p->same_count = 0;
	p->invalid_run = 0;
	p->primed = FALSE;
	p->samples = 0;
}

LOCAL BOOL sample_valid(IMU_PROC *p, const IMU_IN *s)
{
	if (s->saturated) {
		return FALSE;
	}
	if (p->samples != 0U && s->raw_hash == p->last_hash) {
		if (p->same_count < 255U) {
			p->same_count++;
		}
	} else {
		p->same_count = 0;
	}
	p->last_hash = s->raw_hash;
	return (p->same_count < IMU_STUCK_SAMPLES) ? TRUE : FALSE;
}

EXPORT BOOL imu_proc_failed(const IMU_PROC *p)
{
	return (p->invalid_run >= IMU_INVALID_RUN) ? TRUE : FALSE;
}

LOCAL W med3(W a, W b, W c)
{
	if (a > b) {
		W	t = a;

		a = b;
		b = t;
	}
	if (b > c) {
		b = c;
	}
	return (a > b) ? a : b;
}

LOCAL W wrap180(W cdeg)
{
	while (cdeg > 18000) {
		cdeg -= 36000;
	}
	while (cdeg <= -18000) {
		cdeg += 36000;
	}
	return cdeg;
}

/* No gyro: attitude from the accelerometer alone (see imu_proc.h) */
LOCAL void accel_attitude(IMU_PROC *p, W pitch_acc, W roll_acc, BOOL trust_acc)
{
	W	pm, rm;

	if (!p->primed) {
		p->pitch_hist[0] = p->pitch_hist[1] = pitch_acc;
		p->roll_hist[0] = p->roll_hist[1] = roll_acc;
		p->pitch_cdeg = p->pitch_med_cdeg = pitch_acc;
		p->roll_cdeg = p->roll_med_cdeg = roll_acc;
		p->primed = TRUE;
		return;
	}
	if (!trust_acc) {
		return;				/* bump/brake: hold the angles */
	}
	pm = med3(p->pitch_hist[0], p->pitch_hist[1], pitch_acc);
	rm = med3(p->roll_hist[0], p->roll_hist[1], roll_acc);
	p->pitch_hist[0] = p->pitch_hist[1];
	p->pitch_hist[1] = pitch_acc;
	p->roll_hist[0] = p->roll_hist[1];
	p->roll_hist[1] = roll_acc;
	p->pitch_med_cdeg = pm;
	p->roll_med_cdeg = rm;
	p->pitch_cdeg += (pm - p->pitch_cdeg) * IMU_ACC_LPF_PCT / 100;
	p->roll_cdeg += (rm - p->roll_cdeg) * IMU_ACC_LPF_PCT / 100;
}

/* No gyro: heading from the magnetometer, yaw rate from its change */
LOCAL void mag_heading(IMU_PROC *p, const IMU_IN *s, UW dt_ms)
{
	W	m[3], sp, cp, sr, cr, mf, mr, psi, old;
	UINT	span;

	if (!s->mag_valid) {
		p->yaw_valid = FALSE;
		p->yaw_rate_mdps = 0;
		p->yaw_n = 0;			/* rate restarts from fresh data */
		p->yaw_idx = 0;
		return;
	}
	imu_mag_to_body(s, m);

	/* Horizontal field: undo pitch (about right) and roll (about fwd).
	   The median angles are used, not the low-passed ones: their lag
	   would show as heading error, and as a false turn, on a hump. */
	sp = imath_sin_q15(p->pitch_med_cdeg);
	cp = imath_sin_q15(p->pitch_med_cdeg + 9000);
	sr = imath_sin_q15(p->roll_med_cdeg);
	cr = imath_sin_q15(p->roll_med_cdeg + 9000);
	mf = (m[AX_FWD] * cp - m[AX_UP] * sp) / 32768;
	mr = (m[AX_RIGHT] * cr + m[AX_UP] * sr) / 32768;

	/* North appears to the right after a left (CCW) turn: + = left */
	psi = imath_atan2_cdeg(mr, mf);
	if (!p->mag_primed) {
		p->mag_ref_cdeg = psi;
		p->mag_primed = TRUE;
	}
	p->yaw_cdeg = wrap180(psi - p->mag_ref_cdeg);
	if (p->yaw_cdeg < 0) {
		p->yaw_cdeg += 36000;
	}

	if (p->yaw_n < IMU_MAG_RATE_WIN) {
		old = p->yaw_hist[0];
		span = p->yaw_n;
	} else {
		old = p->yaw_hist[p->yaw_idx];
		span = IMU_MAG_RATE_WIN;
	}
	p->yaw_rate_mdps = (span == 0U) ? 0
			 : wrap180(p->yaw_cdeg - old) * 10000 / (W)(span * dt_ms);
	p->yaw_hist[p->yaw_idx] = p->yaw_cdeg;
	p->yaw_idx = (UB)((p->yaw_idx + 1U) % IMU_MAG_RATE_WIN);
	if (p->yaw_n < IMU_MAG_RATE_WIN) {
		p->yaw_n++;
	}
	p->yaw_valid = TRUE;
}

EXPORT BOOL imu_proc_step(IMU_PROC *p, const IMU_IN *s, UW dt_ms)
{
	W	a[3], g[3];
	W	pitch_acc, roll_acc, amag, dev;
	W	sin_p, sin_r, hmag;
	UINT	i;
	BOOL	trust_acc;

	p->collision = FALSE;
	if (!sample_valid(p, s)) {
		if (p->invalid_run < 255U) {
			p->invalid_run++;
		}
		return FALSE;
	}
	p->invalid_run = 0;
	p->samples++;

	imu_to_body(s, a, g);
	for (i = 0; i < 3U; i++) {
		p->a[i] = a[i] - p->cal.acc_offset[i];
		p->g[i] = g[i] - p->cal.gyro_bias[i];
	}

	/* Complementary filter: gyro integration corrected by the
	   accelerometer's gravity direction when it is trustworthy */
	pitch_acc = imath_atan2_cdeg(p->a[AX_FWD], p->a[AX_UP]);
	roll_acc = imath_atan2_cdeg(-p->a[AX_RIGHT], p->a[AX_UP]);
	amag = (W)imath_isqrt((UW)(p->a[0] * p->a[0] + p->a[1] * p->a[1]
				  + p->a[2] * p->a[2]));
	dev = amag - ONE_G_MG;
	trust_acc = (iabs(dev) <= IMU_ACC_TRUST_MG) ? TRUE : FALSE;
	if (!s->has_gyro) {
		accel_attitude(p, pitch_acc, roll_acc, trust_acc);
		mag_heading(p, s, dt_ms);		/* heading and turn rate (FR19) */
	} else {
		if (!p->primed) {
			p->pitch_cdeg = pitch_acc;
			p->roll_cdeg = roll_acc;
			p->primed = TRUE;
		} else {
			W	pg = p->pitch_cdeg + p->g[AX_RIGHT] * (W)dt_ms / 10000;
			W	rg = p->roll_cdeg + p->g[AX_FWD] * (W)dt_ms / 10000;

			if (trust_acc) {
				p->pitch_cdeg = (IMU_ALPHA_PCT * pg + (100 - IMU_ALPHA_PCT) * pitch_acc) / 100;
				p->roll_cdeg = (IMU_ALPHA_PCT * rg + (100 - IMU_ALPHA_PCT) * roll_acc) / 100;
			} else {
				p->pitch_cdeg = pg;	/* bump/brake: gyro only */
				p->roll_cdeg = rg;
			}
		}

		/* Heading and turn rate (FR19) */
		p->yaw_rate_mdps = p->g[AX_UP];
		p->yaw_cdeg = (p->yaw_cdeg + p->g[AX_UP] * (W)dt_ms / 10000) % 36000;
		if (p->yaw_cdeg < 0) {
			p->yaw_cdeg += 36000;
		}
		p->yaw_valid = TRUE;
	}

	/* Gravity-compensated horizontal acceleration */
	sin_p = imath_sin_q15(p->pitch_cdeg);
	sin_r = imath_sin_q15(p->roll_cdeg);
	p->lin_fwd_mg = p->a[AX_FWD] - (ONE_G_MG * sin_p) / 32768;
	p->lin_right_mg = p->a[AX_RIGHT] + (ONE_G_MG * sin_r) / 32768;
	hmag = (W)imath_isqrt((UW)(p->lin_fwd_mg * p->lin_fwd_mg
				   + p->lin_right_mg * p->lin_right_mg));

	/* Collision / shock (FR18) with a refractory period */
	if (p->collision_hold_ms > dt_ms) {
		p->collision_hold_ms -= dt_ms;
	} else {
		p->collision_hold_ms = 0;
		if (hmag > IMU_COLLISION_MG) {
			p->collision = TRUE;
			p->collision_hold_ms = IMU_COLLISION_HOLD_MS;
		}
	}
	if (hmag > p->peak_shock_mg) {
		p->peak_shock_mg = hmag;
	}

	/* Tilt (FR17): beyond the threshold for a minimum time */
	if (iabs(p->pitch_cdeg) > IMU_TILT_CDEG || iabs(p->roll_cdeg) > IMU_TILT_CDEG) {
		p->tilt_ms += dt_ms;
		if (p->tilt_ms >= IMU_TILT_MIN_MS) {
			p->tilt = TRUE;
		}
	} else if (iabs(p->pitch_cdeg) < IMU_TILT_CDEG - IMU_TILT_HYST_CDEG
		   && iabs(p->roll_cdeg) < IMU_TILT_CDEG - IMU_TILT_HYST_CDEG) {
		p->tilt_ms = 0;
		p->tilt = FALSE;
	}

	/* Motion classification (FR18) */
	if (p->yaw_rate_mdps > IMU_TURN_MDPS) {
		p->motion_class = MCLS_TURN_LEFT;
		p->still_ms = 0;
	} else if (p->yaw_rate_mdps < -IMU_TURN_MDPS) {
		p->motion_class = MCLS_TURN_RIGHT;
		p->still_ms = 0;
	} else if ((s->has_gyro
		    ? (iabs(p->g[0]) < IMU_STILL_GYRO_MDPS && iabs(p->g[1]) < IMU_STILL_GYRO_MDPS
		       && iabs(p->g[2]) < IMU_STILL_GYRO_MDPS)
		    : iabs(p->yaw_rate_mdps) < IMU_STILL_MAG_MDPS)
		   && hmag < IMU_STILL_ACC_MG) {
		p->still_ms += dt_ms;
		p->motion_class = (p->still_ms >= IMU_STILL_MS) ? MCLS_STATIONARY
							       : p->motion_class;
	} else {
		p->still_ms = 0;
		p->motion_class = MCLS_MOVING;
	}
	return TRUE;
}
