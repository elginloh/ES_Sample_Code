/*
 *	task_imu.c
 *	IMU Monitoring task (Week 6: High, 10 ms) -- statechart 6.4:
 *
 *	    INITIALIZING --probe+configure ok--> CALIBRATING (car still, INIT)
 *	    CALIBRATING --calibration_success--> MONITORING
 *	    CALIBRATING --calibration_failed---> FAULT   (SYS_FLT_IMU_CAL)
 *	    MONITORING --collision_detected----> collision latched + event
 *	    MONITORING --sensor_error/invalid--> FAULT   (SYS_FLT_IMU)
 *	    FAULT ------reset / recovery-------> INITIALIZING, at most
 *	                                         S4_RECOVERY_TRIES times
 *	Hump analysis runs in task_hump.c, fed by the bounded queue.
 */

#include "subsystem4.h"
#include "drv_i2c0.h"
#include "drv_imu.h"

/* 1 = print the magnetometer every 0.5 s with the running hard-iron
   estimate (max + min) / 2 per chip axis.  Turn the car slowly through
   360 deg, then copy the last estimate into IMU_MAG_OFF_X/Y/Z
   (imu_proc.h).  Use a TEST=1 image so the motors stay off. */
#ifndef S4_MAG_LOG
#define S4_MAG_LOG		0
#endif

EXPORT TASK_STAT	s4_stat_imu;
EXPORT APP_SEQ		s4_imu_seq;
EXPORT UW		s4_imu_words[2];
EXPORT UW		s4_imu_time;
EXPORT volatile UW	s4_dropped;

LOCAL IMU_PROC		proc;
LOCAL IMU_CAL_ACC	cal_acc;
LOCAL IMU_CAL		cal;
LOCAL UB		state = IMU_ST_INIT;
LOCAL UB		chip;
LOCAL BOOL		collision_latched;
LOCAL UB		sample_seq;
LOCAL UB		hump_seq;
LOCAL W			ds_rem_um;	/* distance not yet sent to the hump task */

EXPORT void imu_cyclic_handler(void *exinf)
{
	(void)exinf;
	(void)tk_set_flg(flg_imu, IMU_EV_TICK);
}

EXPORT void s4_get_imu_status(S4_IMU_STATUS *s)
{
	UW	w[2] = {0, 0};
	UW	t = 0;
	UINT	tries;

	for (tries = 0; tries < APP_SEQ_READ_TRIES; tries++) {
		UW	q = app_seq_read_begin(&s4_imu_seq);

		w[0] = s4_imu_words[0];
		w[1] = s4_imu_words[1];
		t = s4_imu_time;
		if (!app_seq_read_retry(&s4_imu_seq, q)) {
			break;
		}
	}
	s4_imu_unpack(w, s);
	s->time_ms = t;
}

LOCAL void publish(BOOL valid)
{
	S4_IMU_STATUS	s;
	UW		w[2];
	BOOL		ok = valid && (state == IMU_ST_OK);
	BOOL		yaw_ok = ok && proc.yaw_valid;

	s.pitch_deg = ok ? s4_deg8(proc.pitch_cdeg) : (B)S4_ANGLE_INVALID;
	s.roll_deg = ok ? s4_deg8(proc.roll_cdeg) : (B)S4_ANGLE_INVALID;
	s.yaw_rate_dps = yaw_ok ? s4_rate9(proc.yaw_rate_mdps) : (H)S4_RATE_INVALID;
	s.motion_class = proc.motion_class;
	s.status = state;
	s.tilt = (ok && proc.tilt) ? 1U : 0U;
	s.collision = collision_latched ? 1U : 0U;
	s.valid = ok ? 1U : 0U;
	s.heading_deg = yaw_ok ? (UH)(proc.yaw_cdeg / 100) : (UH)S4_HEADING_INVALID;
	s.shock_half_g = s4_shock5(proc.peak_shock_mg);
	s.seq = sample_seq++;
	s.chip = chip;
	s.calibrated = cal.valid ? 1U : 0U;
	s4_imu_pack(&s, w);
	app_seq_write_begin(&s4_imu_seq);
	s4_imu_words[0] = w[0];
	s4_imu_words[1] = w[1];
	s4_imu_time = app_now_ms();
	app_seq_write_end(&s4_imu_seq);
}

LOCAL BOOL read_sample(IMU_IN *in)
{
	IMU_SAMPLE	s;
	UINT		i;

	if (imu_read(&s) < E_OK) {
		return FALSE;
	}
	for (i = 0; i < 3U; i++) {
		in->acc_mg[i] = s.acc_mg[i];
		in->gyr_mdps[i] = s.gyr_mdps[i];
		in->mag_mgauss[i] = s.mag_mgauss[i];
	}
	in->has_gyro = s.has_gyro;
	in->mag_valid = s.mag_valid;
	in->saturated = s.saturated;
	in->raw_hash = s.raw_hash;
	return TRUE;
}

#if S4_MAG_LOG
LOCAL void mag_log(const IMU_IN *in)
{
	static W	lo[3] = { 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF };
	static W	hi[3] = { -0x7FFFFFFF, -0x7FFFFFFF, -0x7FFFFFFF };
	static UINT	n;
	UINT		i;

	if (!in->mag_valid) {
		return;
	}
	for (i = 0; i < 3U; i++) {
		if (in->mag_mgauss[i] < lo[i]) lo[i] = in->mag_mgauss[i];
		if (in->mag_mgauss[i] > hi[i]) hi[i] = in->mag_mgauss[i];
	}
	if (++n % 50U == 0U) {
		tm_printf((UB*)"imu: mag %d %d %d mgauss, offset estimate %d %d %d\n",
			  (INT)in->mag_mgauss[0], (INT)in->mag_mgauss[1], (INT)in->mag_mgauss[2],
			  (INT)((hi[0] + lo[0]) / 2), (INT)((hi[1] + lo[1]) / 2),
			  (INT)((hi[2] + lo[2]) / 2));
	}
}
#endif

/* Feed the Hump & Terrain task: pitch + distance step (0.1 mm units) */
LOCAL void send_hump_sample(BOOL valid)
{
	S4_HUMP_SAMPLE	hs;
	W		spd = 0;
	W		ds;
	UW		w;

	motion_get_odometry(NULL, &spd);
	/* mm/s * 10 ms = um; send whole 0.1 mm steps and carry the rest, so
	   slow driving (0.3 mm per sample at 30 mm/s) loses no distance */
	ds_rem_um += (spd > 0) ? spd * S4_PERIOD_MS : 0;
	ds = ds_rem_um / 100;
	if (ds > 255) {
		ds = 255;
	}
	ds_rem_um -= ds * 100;
	if (ds_rem_um >= 100) {
		ds_rem_um = 0;			/* clamped: do not bank the excess */
	}
	hs.pitch_ddeg = (H)(proc.pitch_cdeg / 10);
	hs.ds_100um = (UB)ds;
	hs.valid = valid ? 1U : 0U;
	hs.seq = hump_seq;
	hump_seq = (UB)((hump_seq + 1U) & 0xFU);
	w = s4_sample_pack(&hs);
	if (tk_snd_mbf(mbf_imu_hump, &w, (INT)sizeof(w), TMO_POL) != E_OK) {
		s4_dropped = s4_dropped + 1U;	/* bounded queue: drop, count */
	}
}

LOCAL BOOL bring_up(void)
{
	UB	addr, who;

	if (imu_probe(&chip, &addr, &who) < E_OK) {
		i2c0_bus_recover();
		tm_printf((UB*)"imu: no supported IMU answered (last id 0x%02x)\n",
			  (INT)who);
		return FALSE;
	}
	if (imu_configure() < E_OK) {
		tm_printf((UB*)"imu: configuration failed\n");
		return FALSE;
	}
	tm_printf((UB*)"imu: %s at 0x%02x (id 0x%02x)\n",
		  (chip == IMU_CHIP_MPU) ? "InvenSense family"
		  : (chip == IMU_CHIP_LSM6) ? "ST LSM6 family"
		  : "ST LSM303DLHC (accel + mag, no gyro)",
		  (INT)addr, (INT)who);
	return TRUE;
}

EXPORT void imu_task(INT stacd, void *exinf)
{
	UW	cal_epoch = 0xFFU;
	UINT	cal_tries = 0, recover_tries = 0;
	UW	wait_ms = 0;
	UINT	ptn;
	IMU_IN	in;

	(void)stacd;
	(void)exinf;
	if (i2c0_init(flg_imu, IMU_EV_I2C) < E_OK) {
		sys_fault_set(SYS_FLT_IMU | SYS_FLT_TASK_COMM);
		state = IMU_ST_FAULT;
	}
	imu_cal_reset(&cal_acc);
	cal.valid = FALSE;

	for (;;) {
		UW	vsw, mode, start;
		BOOL	still, valid;

		(void)tk_wai_flg(flg_imu, IMU_EV_TICK, TWF_ORW | TWF_BITCLR, &ptn,
				 TMO_FEVR);
		start = task_stat_begin(&s4_stat_imu);
		vsw = sys_vsw();
		mode = vsw_mode(vsw);
		/* motors are off in INIT and SAFE_STOP (Tribunal 1: Buddy 2
		   keeps them off while the IMU calibrates) */
		still = (mode == VM_INIT || mode == VM_SAFE_STOP) ? TRUE : FALSE;

		switch (state) {
		case IMU_ST_INIT:
			if (bring_up()) {
				state = IMU_ST_CALIBRATING;
				imu_cal_reset(&cal_acc);
			} else if (++recover_tries >= S4_RECOVERY_TRIES) {
				state = IMU_ST_FAULT;
				sys_fault_set(SYS_FLT_IMU);
				sys_post_event(SYS_EV_IMU_CAL_FAIL);
			} else {
				wait_ms = 0;
				state = IMU_ST_FAULT;	/* retry after the interval */
			}
			break;

		case IMU_ST_CALIBRATING:
			if (!still) {
				imu_cal_reset(&cal_acc);	/* wait for a stationary phase */
				break;
			}
			if (read_sample(&in)) {
				imu_cal_add(&cal_acc, &in);
			}
			if (cal_acc.n >= IMU_CAL_SAMPLES) {
				INT	r = imu_cal_finish(&cal_acc, &cal);

				imu_cal_reset(&cal_acc);
				if (r == IMU_CAL_OK) {
					imu_proc_init(&proc, &cal);
					state = IMU_ST_OK;
					cal_epoch = vsw_epoch(vsw);
					cal_tries = 0;
					recover_tries = 0;
					sys_fault_clear(SYS_FLT_IMU | SYS_FLT_IMU_CAL);
					sys_post_event(SYS_EV_IMU_CAL_OK);
					if (chip == IMU_CHIP_LSM303) {
						tm_printf((UB*)"imu: calibrated (no gyro), level offset %d %d mg\n",
							  (INT)cal.acc_offset[0], (INT)cal.acc_offset[1]);
					} else {
						tm_printf((UB*)"imu: calibrated, gyro bias %d %d %d mdps\n",
							  (INT)cal.gyro_bias[0], (INT)cal.gyro_bias[1],
							  (INT)cal.gyro_bias[2]);
					}
				} else if (r == IMU_CAL_NOT_STILL
					   && (mode == VM_INIT || ++cal_tries < S4_CAL_TRIES)) {
					/* moved during calibration: try again.  In INIT
					   keep trying, as Buddy 3 does for the IR: the car
					   is often still being placed or the switch was
					   just pressed.  A start that never settles still
					   ends in Safe Stop via INIT_TIMEOUT_MS (SYS_FLT_INIT). */
				} else {
					state = IMU_ST_FAULT;
					sys_fault_set(SYS_FLT_IMU_CAL);
					sys_post_event(SYS_EV_IMU_CAL_FAIL);
					tm_printf((UB*)"imu: calibration failed (%d)\n", r);
				}
			}
			break;

		case IMU_ST_OK:
			/* every new INIT epoch re-calibrates (statechart: Initialising
			   -> Calibrate IMU) */
			if (mode == VM_INIT && vsw_epoch(vsw) != cal_epoch) {
				state = IMU_ST_CALIBRATING;
				imu_cal_reset(&cal_acc);
				break;
			}
			if (!read_sample(&in)) {
				/* I2C failure counts as an invalid sample */
				if (proc.invalid_run < 255U) {
					proc.invalid_run++;
				}
				valid = FALSE;
			} else {
				valid = imu_proc_step(&proc, &in, S4_PERIOD_MS);
#if S4_MAG_LOG
				mag_log(&in);
#endif
			}
			if (!valid) {
				if (imu_proc_failed(&proc)) {
					state = IMU_ST_FAULT;	/* sensor_error / invalid_data */
					sys_fault_set(SYS_FLT_IMU);
					tm_printf((UB*)"imu: invalid data, entering FAULT\n");
				}
				send_hump_sample(FALSE);
				publish(FALSE);
				break;
			}
			if (proc.collision) {
				collision_latched = TRUE;
				sys_fault_set(SYS_FLT_COLLISION);
				sys_post_event(SYS_EV_COLLISION);
				tm_printf((UB*)"imu: COLLISION %d mg\n", (INT)proc.peak_shock_mg);
			} else if (collision_latched
				   && (sys_fault_get() & SYS_FLT_COLLISION) == 0U) {
				collision_latched = FALSE;	/* "cleared" by the operator */
			}
			send_hump_sample(TRUE);
			publish(TRUE);
			break;

		default:	/* IMU_ST_FAULT: bounded, timed recovery */
			publish(FALSE);
			wait_ms += S4_PERIOD_MS;
			if (wait_ms >= S4_RECOVERY_MS && still
			    && recover_tries < S4_RECOVERY_TRIES) {
				wait_ms = 0;
				recover_tries++;
				state = IMU_ST_INIT;
			}
			break;
		}
		task_stat_end(&s4_stat_imu, start, (UW)S4_PERIOD_MS * 1000U / 2U);
	}
}
