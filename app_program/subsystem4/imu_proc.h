/*
 *	imu_proc.h
 *	IMU processing (Buddy 4): calibration, filtering, tilt, turn rate,
 *	motion classification, collision detection, data validation.
 *	Pure logic, integer maths (Cortex-M0+ has no FPU and libm is not
 *	linked), host-testable.
 *
 *	Body frame (Project Briefing p.7): X = right, Y = forward, Z = up,
 *	right-handed.  Pitch is nose-up positive (rotation about X), roll is
 *	right-side-down positive (about Y), yaw rate is counter-clockwise
 *	positive seen from above (about Z): a right turn gives a negative
 *	yaw rate.
 *
 *	Precision (Tribunal 1, Buddy 4): computation stays at full resolution
 *	(mg, mdps, centi-degrees, um); only stored/transmitted values are
 *	compacted (s4_status.h).
 *
 *	Thresholds marked [CAL] are starting points; Tribunal 1 Version B:
 *	"set thresholds from real tests" (needs validation on the car).
 *
 *	Chips without a gyro (LSM303DLHC, the module supplied by the course):
 *	  pitch/roll  accelerometer only: a median of 3 removes single-sample
 *	              spikes, then a low-pass filter.  While |a| is not close
 *	              to 1 g (bump, hard braking) the angles are held.
 *	              Steady acceleration still reads as tilt: 0.05 g of
 *	              speed-up is about 3 deg of nose-up pitch.
 *	  heading     magnetometer, tilt-compensated with pitch/roll, relative
 *	              to the heading at calibration (0 = calibration direction).
 *	  yaw rate    change of that heading over IMU_MAG_RATE_WIN samples.
 *	              Invalid (published as such) while the magnetometer
 *	              overflows; the rest of the sample is still used.
 */

#ifndef IMU_PROC_H
#define IMU_PROC_H

#include <tk/typedef.h>

#define IMU_ST_INIT		0U	/* statechart INITIALIZING */
#define IMU_ST_CALIBRATING	1U
#define IMU_ST_OK		2U	/* MONITORING             */
#define IMU_ST_FAULT		3U	/* FAULT / SAFE STATE     */

#define MCLS_STATIONARY		0U
#define MCLS_MOVING		1U
#define MCLS_TURN_LEFT		2U
#define MCLS_TURN_RIGHT		3U

#define AX_RIGHT		0U
#define AX_FWD			1U
#define AX_UP			2U

/* Chip axis -> body axis mapping [CAL]: which chip axis (0..2) and sign
   feed body X (right), Y (forward), Z (up).  Default: module mounted
   flat with its X to the right and Y to the front. */
#ifndef IMU_MAP_RIGHT
#define IMU_MAP_RIGHT		0
#define IMU_SIGN_RIGHT		1
#define IMU_MAP_FWD		1
#define IMU_SIGN_FWD		1
#define IMU_MAP_UP		2
#define IMU_SIGN_UP		1
#endif

/* Magnetometer hard-iron offsets [CAL], chip axes, mgauss: the field of
   the car's own motors and battery.  Measure by turning the car slowly
   through 360 deg and taking (max + min) / 2 of each axis. */
#ifndef IMU_MAG_OFF_X
#define IMU_MAG_OFF_X		0
#define IMU_MAG_OFF_Y		0
#define IMU_MAG_OFF_Z		0
#endif

/* Filtering / detection thresholds [CAL] */
#define IMU_ALPHA_PCT		98	/* complementary filter gyro weight  */
#define IMU_ACC_TRUST_MG	300	/* |a|-1g beyond: gyro only          */
#define IMU_TILT_CDEG		1500	/* 15 deg pitch or roll ...          */
#define IMU_TILT_HYST_CDEG	300
#define IMU_TILT_MIN_MS		200	/* ... for at least 200 ms           */
#define IMU_TURN_MDPS		15000	/* |yaw rate| above: turning         */
#define IMU_STILL_GYRO_MDPS	3000
#define IMU_STILL_ACC_MG	50
#define IMU_STILL_MS		300
#define IMU_COLLISION_MG	1500	/* horizontal linear acceleration    */
#define IMU_COLLISION_HOLD_MS	500	/* refractory period after an impact */
#define IMU_STUCK_SAMPLES	20	/* identical raw data = sensor stuck */
#define IMU_INVALID_RUN		10	/* 100 ms of bad data = IMU fault    */

/* No-gyro chips [CAL] */
#define IMU_ACC_LPF_PCT		20	/* accel attitude low-pass, ~45 ms   */
#define IMU_MAG_RATE_WIN	20	/* yaw rate over 20 samples (200 ms) */
#define IMU_STILL_MAG_MDPS	8000	/* magnetometer yaw rate is noisier  */

/* Calibration acceptance [CAL] (Tribunal 1: "calibrate while still") */
#define IMU_CAL_SAMPLES		100	/* 1 s at 10 ms                      */
#define IMU_CAL_GYRO_RANGE	4000	/* max-min per gyro axis, mdps       */
#define IMU_CAL_ACC_RANGE	80	/* max-min per accel axis, mg        */
#define IMU_CAL_UP_MIN_MG	800	/* gravity must be on the up axis    */
#define IMU_CAL_1G_TOL_MG	150

#define IMU_CAL_OK		0
#define IMU_CAL_NO_DATA		(-1)
#define IMU_CAL_NOT_STILL	(-2)
#define IMU_CAL_ORIENTATION	(-3)

typedef struct {
	W	acc_mg[3];	/* chip axes */
	W	gyr_mdps[3];
	W	mag_mgauss[3];
	BOOL	has_gyro;	/* FALSE: attitude from accel, heading from mag */
	BOOL	mag_valid;
	BOOL	saturated;
	UH	raw_hash;
} IMU_IN;

typedef struct {
	W	sum_a[3];
	W	sum_g[3];
	W	min_a[3];
	W	max_a[3];
	W	min_g[3];
	W	max_g[3];
	UINT	n;
} IMU_CAL_ACC;

typedef struct {
	W	gyro_bias[3];	/* body axes, mdps */
	W	acc_offset[3];	/* body axes, mg; up axis relative to +1 g */
	BOOL	valid;
} IMU_CAL;

typedef struct {
	IMU_CAL	cal;
	W	a[3];		/* calibrated body acceleration, mg */
	W	g[3];		/* calibrated body rate, mdps */
	W	pitch_cdeg;
	W	roll_cdeg;
	W	yaw_cdeg;	/* heading 0..35999, 0 = direction at start */
	W	yaw_rate_mdps;
	BOOL	yaw_valid;	/* FALSE while a no-gyro chip has no heading */
	W	pitch_hist[2];	/* no gyro: last two accel angles (median) */
	W	roll_hist[2];
	W	pitch_med_cdeg;	/* no gyro: median only, 1 sample behind; */
	W	roll_med_cdeg;	/* levels the magnetometer in step with it */
	W	mag_ref_cdeg;	/* no gyro: magnetic heading at start */
	BOOL	mag_primed;
	W	yaw_hist[IMU_MAG_RATE_WIN];
	UB	yaw_idx;
	UB	yaw_n;
	W	lin_fwd_mg;	/* gravity-compensated */
	W	lin_right_mg;
	UB	motion_class;
	BOOL	tilt;
	UW	tilt_ms;
	UW	still_ms;
	BOOL	collision;		/* impact detected in this sample */
	UW	collision_hold_ms;
	W	peak_shock_mg;		/* largest horizontal impact so far */
	UH	last_hash;
	UB	same_count;
	UB	invalid_run;
	BOOL	primed;
	UW	samples;
} IMU_PROC;

/* Body-frame mapping of one sample */
IMPORT void imu_to_body(const IMU_IN *s, W a[3], W g[3]);
IMPORT void imu_mag_to_body(const IMU_IN *s, W m[3]);	/* hard-iron removed */

IMPORT void imu_cal_reset(IMU_CAL_ACC *c);
IMPORT void imu_cal_add(IMU_CAL_ACC *c, const IMU_IN *s);
IMPORT INT  imu_cal_finish(const IMU_CAL_ACC *c, IMU_CAL *cal);

IMPORT void imu_proc_init(IMU_PROC *p, const IMU_CAL *cal);
/* Returns TRUE when the sample was valid and used */
IMPORT BOOL imu_proc_step(IMU_PROC *p, const IMU_IN *s, UW dt_ms);
IMPORT BOOL imu_proc_failed(const IMU_PROC *p);	/* invalid run reached */

/* Integer maths (exported for the unit tests) */
IMPORT W    imath_atan2_cdeg(W y, W x);		/* -18000..18000 */
IMPORT W    imath_sin_q15(W cdeg);		/* sin * 32768   */
IMPORT UW   imath_isqrt(UW v);

#endif /* IMU_PROC_H */
