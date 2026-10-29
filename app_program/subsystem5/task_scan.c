/*
 *	task_scan.c
 *	Obstacle Scan task (Week 6: High; Tribunal 1: the only task that
 *	controls the ultrasonic sensor and the servo).
 *
 *	MONITOR mode: servo centred, one ranging every S5_PING_PERIOD_MS.
 *	  Normal Navigation / Navigation Action: an obstacle closer than
 *	  S5_DETECT_MM on S5_DETECT_CONFIRM readings raises AVEV_DETECTED;
 *	  closer than S5_CAUTION_MM it caps the speed (FR8 "reduce speed").
 *	  Obstacle Handling: an obstacle closer than S5_EMERGENCY_MM while the
 *	  car drives forward stops it at once (stop() bypass) and raises
 *	  AVEV_EMERGENCY.
 *	COARSE / FINE requests: sweep the servo, range at every step, stream
 *	packed samples to the Obstacle Profile task.
 *	Sensor faults: S5_US_FAIL_COUNT silent pings -> SYS_FLT_US_NO_RESPONSE
 *	(cleared after 3 good pings); an implausible reading -> US_INVALID.
 */

#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "drv_servo.h"

EXPORT volatile UW	s5_scan_word;
EXPORT TASK_STAT	s5_stat_scan;

LOCAL S5_SCAN_STATUS	sst;
LOCAL UB		silent_run;
LOCAL UB		good_run;
LOCAL UB		detect_run;

EXPORT void s5_get_scan_status(S5_SCAN_STATUS *s)
{
	s5_scan_status_unpack(app_load(&s5_scan_word), s);
}

LOCAL void publish(void)
{
	sst.servo_deg = (B)servo_angle();
	app_store(&s5_scan_word, s5_scan_status_pack(&sst));
}

/* One ranging with fault bookkeeping; returns US_* and the distance */
LOCAL INT range(UW *mm)
{
	INT	r = us_measure(mm);

	if (r == US_NO_RESPONSE) {
		good_run = 0;
		/* Not while INIT: right after a power-on the sensor (and the
		   servo centring) take a few pings to answer, and the critical
		   fault latched Safe Stop before the car ever drove.  A sensor
		   still silent once driving trips it at the next ping. */
		if (silent_run < 255U && ++silent_run >= S5_US_FAIL_COUNT
		    && sys_mode() != VM_INIT) {
			sys_fault_set(SYS_FLT_US_NO_RESPONSE);
		}
	} else {
		silent_run = 0;
		if (good_run < 255U && ++good_run >= 3U) {
			sys_fault_clear(SYS_FLT_US_NO_RESPONSE);
		}
	}
	if (r == US_INVALID) {
		sys_fault_set(SYS_FLT_US_INVALID);
	} else if (r == US_OK) {
		sys_fault_clear(SYS_FLT_US_INVALID);
	}
	sst.us_result = (UB)r;
	sst.dist_code = (r == US_OK) ? s5_dist_code(*mm)
		      : (r == US_NO_ECHO) ? (UH)S5_DIST_NO_ECHO : (UH)S5_DIST_INVALID;
	sst.seq = (UB)((sst.seq + 1U) & 0xFU);
	return r;
}

LOCAL void monitor_once(void)
{
	UW		mm = 0;
	INT		r;
	UW		mode = sys_mode();
	MOTION_STATUS	ms;

	if (servo_angle() != 0) {
		(void)tk_dly_tsk(servo_set(0));
	}
	r = range(&mm);

	if (mode == VM_NORMAL || mode == VM_NAV_ACTION) {
		if (r == US_OK && mm < S5_DETECT_MM) {
			if (++detect_run >= S5_DETECT_CONFIRM) {
				sst.detected = 1;
				(void)tk_set_flg(flg_avoid, AVEV_DETECTED);
			}
		} else {
			detect_run = 0;
		}
		if (r == US_OK && mm < S5_CAUTION_MM) {
			motion_set_speed_cap(S5_CAUTION_MMS);	/* refreshed every ping */
			sst.cap = 1;
		} else {
			motion_set_speed_cap(0);
			sst.cap = 0;
		}
	} else {
		detect_run = 0;
		sst.detected = 0;
		if (sst.cap != 0U) {
			motion_set_speed_cap(0);
			sst.cap = 0;
		}
		if (mode == VM_OBSTACLE && r == US_OK && mm < S5_EMERGENCY_MM) {
			motion_get_status(&ms);
			if (ms.mstate == MS_STRAIGHT && ms.speed_l_mms > 0 && ms.speed_r_mms > 0) {
				motion_stop(MSRC_AVOID);
				(void)tk_set_flg(flg_avoid, AVEV_EMERGENCY);
			}
		}
	}
	publish();
}

LOCAL void do_scan(const S5_SCAN_REQ *rq)
{
	S5_SAMPLE	smp;
	INT		a = rq->start_deg;
	INT		step = (rq->step_deg == 0U) ? 1 : (INT)rq->step_deg;
	INT		dir = (rq->end_deg >= rq->start_deg) ? 1 : -1;
	UB		phase = (rq->type == SREQ_FINE) ? 1U : 0U;
	UINT		silent = 0, total = 0;

	sst.mode = rq->type;
	for (;;) {
		BOOL	last = ((dir > 0) ? (a + step > rq->end_deg)
					  : (a - step < rq->end_deg)) ? TRUE : FALSE;
		UW	mm = 0;
		INT	r;
		UW	w;

		(void)tk_dly_tsk(servo_set(a));
		r = range(&mm);
		if (r == US_INVALID) {
			(void)tk_dly_tsk(US_MIN_INTERVAL_MS);	/* one retry */
			r = range(&mm);
		}
		total++;
		if (r == US_NO_RESPONSE) {
			silent++;
		}
		smp.angle_deg = (B)servo_angle();
		smp.dist_code = sst.dist_code;
		smp.scan_id = rq->scan_id;
		smp.phase = phase;
		smp.last = last ? 1U : 0U;
		smp.status = (r == US_OK) ? SS_OK : (r == US_NO_ECHO) ? SS_NO_ECHO
			   : (last && silent == total) ? SS_ABORTED : SS_INVALID;
		w = s5_sample_pack(&smp);
		if (tk_snd_mbf(mbf_scan_samples, &w, (INT)sizeof(w), TMO_POL) != E_OK) {
			sys_fault_set(SYS_FLT_TASK_COMM);
		}
		publish();
		if (last) {
			break;
		}
		a += dir * step;
	}
	(void)tk_dly_tsk(servo_set(0));
	sst.mode = SREQ_MONITOR;
	publish();
}

EXPORT void scan_task(INT stacd, void *exinf)
{
	S5_SCAN_REQ	rq;
	UW		w;
	UB		mode = SREQ_MONITOR;

	(void)stacd;
	(void)exinf;
	if (servo_init() < E_OK) {
		sys_fault_set(SYS_FLT_SERVO_SUSPECT);
	}
	sst.dist_code = S5_DIST_INVALID;
	sst.mode = SREQ_MONITOR;
	publish();

	for (;;) {
		UW	start = task_stat_begin(&s5_stat_scan);
		UW	spent;
		TMO	wait;
		INT	n;

		if (mode == SREQ_MONITOR) {
			monitor_once();
		}
		task_stat_end(&s5_stat_scan, start, (UW)S5_PING_PERIOD_MS * 1000U);

		/* wait for a request at most until the next ping is due */
		spent = (hw_time_us() - start) / 1000U;
		wait = (mode != SREQ_MONITOR) ? TMO_FEVR
		     : (TMO)((spent >= S5_PING_PERIOD_MS) ? 0U : (S5_PING_PERIOD_MS - spent));
		n = tk_rcv_mbf(mbf_scan_req, &w, (wait == 0) ? TMO_POL : wait);
		if (n != (INT)sizeof(w)) {
			continue;
		}
		s5_req_unpack(w, &rq);
		if (rq.type == SREQ_COARSE || rq.type == SREQ_FINE) {
			do_scan(&rq);
			mode = SREQ_IDLE;	/* avoidance decides when to monitor again */
		} else {
			mode = rq.type;
		}
		sst.mode = mode;
		publish();
	}
}
