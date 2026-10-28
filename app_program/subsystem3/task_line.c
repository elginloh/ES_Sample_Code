/*
 *	task_line.c
 *	Line Following task (Week 6: High, periodic 20 ms) and its cyclic
 *	handler.  Implements the Line following / Error-recover part of the
 *	Buddy 3 statechart (6.3); the barcode part runs in task_barcode.c.
 *
 *	Per period:
 *	  INIT mode      calibrate the IR polarity once per INIT epoch (car
 *	                 still, on the line) -> SYS_EV_IR_CAL_OK / _FAIL
 *	  NORMAL mode    line position -> steering word to Buddy 2; line
 *	                 loss search; junction detection
 *	  other modes    no steering (Tribunal 1: "Buddy 3 reads the mode to
 *	                 stop steering outside Normal Navigation"), but line
 *	                 found/lost is still reported for Buddy 5's recovery
 */

#include "subsystem3.h"
#include "drv_ir.h"
#include "../subsystem4/subsystem4.h"

EXPORT TASK_STAT	s3_stat_line;
EXPORT volatile UW	s3_line_word;
EXPORT volatile UW	s3_line_time;
EXPORT volatile UW	s3_black_level = 1U;
EXPORT APP_SEQ		s3_line_seq;

LOCAL LINE_FOLLOW	lf;

/* Debug: print the IR sensors whenever they change (1 = on, 0 = off).
   Works in every mode, so the sensors can be checked by hand with the
   car still.  Set to 0 for the final build. */
#define S3_LINE_LOG	1

/* An IMU heading older than this is not used for the search sweeps (the
   IMU task publishes every 10 ms) */
#define S3_IMU_STALE_MS	100

EXPORT void line_cyclic_handler(void *exinf)
{
	(void)exinf;
	(void)tk_wup_tsk(tid_line);
}

EXPORT void s3_get_line_status(S3_LINE_STATUS *s)
{
	UW	w = 0, t = 0;
	UINT	tries;

	for (tries = 0; tries < APP_SEQ_READ_TRIES; tries++) {
		UW	q = app_seq_read_begin(&s3_line_seq);

		w = s3_line_word;
		t = s3_line_time;
		if (!app_seq_read_retry(&s3_line_seq, q)) {
			break;
		}
	}
	s3_line_unpack(w, s);
	s->time_ms = t;
}

/* IMU heading for the line-search sweeps (Buddy 4, IF-I1): degrees,
   + = left, or LF_HEADING_NONE while the IMU is not monitoring, the
   heading is invalid (magnetometer overflow) or the snapshot is stale */
LOCAL INT imu_heading(void)
{
#if LF_SEARCH_USE_IMU
	S4_IMU_STATUS	imu;

	s4_get_imu_status(&imu);
	if (imu.status != IMU_ST_OK || imu.heading_deg == S4_HEADING_INVALID
	    || (UW)(app_now_ms() - imu.time_ms) > S3_IMU_STALE_MS) {
		return LF_HEADING_NONE;
	}
	return (INT)imu.heading_deg;
#else
	return LF_HEADING_NONE;
#endif
}

/* Barcode capture in progress (published by the barcode task) */
LOCAL BOOL barcode_capturing(void)
{
	S3_BC_STATUS	b;

	s3_bc_unpack(app_load(&s3_bc_word), &b);
	return (b.state == BCS_CAPTURING) ? TRUE : FALSE;
}

LOCAL void calibrate_step(UB raw, UW epoch, UW *cal_epoch, UB *buf, UINT *n)
{
	INT	r;

	if (*cal_epoch == epoch) {
		return;				/* already done for this INIT */
	}
	buf[(*n)++] = raw;
	if (*n < S3_CAL_SAMPLES) {
		return;
	}
	r = lf_calibrate(&lf, buf, *n);
	*n = 0;
	if (r == LF_CAL_OK) {
		*cal_epoch = epoch;
		app_store(&s3_black_level, lf.black_level[IR_R]);
		sys_fault_clear(SYS_FLT_IR_CAL);
		sys_post_event(SYS_EV_IR_CAL_OK);
		tm_printf((UB*)"line: IR calibrated, black level = %d\n",
			  (INT)lf.black_level[IR_C]);
	} else {
		/* Retry calibration each 200 ms rather than latching a
		   critical fault when the car starts slightly off the line. */
	}
}

EXPORT void line_task(INT stacd, void *exinf)
{
	S3_LINE_STATUS	st;
	LF_OUT		out;
	UB		cal_buf[S3_CAL_SAMPLES];
	UINT		cal_n = 0;
	UW		cal_epoch = 0xFFU;
	BOOL		was_nav = FALSE;
	UB		seq = 0, junctions = 0, lost_count = 0;
	UB		prev_black = 0xFFU;	/* S3_LINE_LOG */
	UB		prev_state = 0xFFU;	/* S3_LINE_LOG */
	UB		prev_phase = 0xFFU;	/* S3_LINE_LOG */
	W		odo = 0;

	(void)stacd;
	(void)exinf;
	lf_init(&lf);

	for (;;) {
		UW	start, vsw, mode;
		UB	raw, black;
		BOOL	nav;
		INT	steer;

		(void)tk_slp_tsk(TMO_FEVR);
		(void)tk_can_wup(TSK_SELF);	/* never "catch up" on late ticks */
		start = task_stat_begin(&s3_stat_line);

		raw = ir_read_raw();
		vsw = sys_vsw();
		mode = vsw_mode(vsw);
		motion_get_odometry(&odo, NULL);

		if (mode == VM_INIT) {
			calibrate_step(raw, vsw_epoch(vsw), &cal_epoch, cal_buf, &cal_n);
		} else {
			cal_n = 0;
		}

		nav = (mode == VM_NORMAL) ? TRUE : FALSE;
		if (nav && !was_nav) {
			lf_reset_tracking(&lf);	/* fresh start of Normal Navigation */
		}
		was_nav = nav;

		black = lf_black_bits(&lf, raw);
		lf_step(&lf, black, odo, imu_heading(), S3_PERIOD_MS, nav, &out);
		steer = out.steer;
#if S3_LINE_LOG
		if (black != prev_black) {
			tm_printf((UB*)"ir: L%d C%d R%d | err %d state %d steer %d\n",
				  (INT)((black >> IR_L) & 1U),
				  (INT)((black >> IR_C) & 1U),
				  (INT)((black >> IR_R) & 1U),
				  (INT)lf.err, (INT)lf.state, (INT)out.steer);
			prev_black = black;
		}
		/* Each search sweep and the find, with the angle reached from
		   where the line was lost (+ = right): checks LF_SEARCH_DEG */
		if (lf.state == LS_SEARCHING
		    && (lf.state != prev_state || lf.search_phase != prev_phase)) {
			if (lf.search_origin == LF_HEADING_NONE) {
				tm_printf((UB*)"line: sweep %s (timed, no IMU heading)\n",
					  out.reverse ? "back" : (out.steer < 0) ? "left" : "right");
			} else {
				tm_printf((UB*)"line: sweep %s (IMU), at %d deg\n",
					  out.reverse ? "back" : (out.steer < 0) ? "left" : "right",
					  (INT)lf.search_deg);
			}
		}
		if ((out.events & LF_EV_FOUND) != 0U && prev_state == LS_SEARCHING) {
			tm_printf((UB*)"line: found at %d deg\n", (INT)lf.search_deg);
		}
		prev_state = lf.state;
		prev_phase = lf.search_phase;
#endif

		if (nav && out.steering) {
			if (S3_PAUSE_STEER_ON_BARCODE && barcode_capturing()
			    && lf.state == LS_ON_LINE
			    && (black & IR_BIT(IR_C)) != 0U) {
				steer = 0;		/* hold straight over the barcode */
			}
			if (out.reverse) {
				motion_set_steer_reverse(out.speed_frac);
			} else {
				motion_set_steer(steer, out.speed_frac);
			}
		} else {
			motion_clear_steer();
		}

		if ((out.events & LF_EV_LOST) != 0U) {
			lost_count++;
			sys_fault_set(SYS_FLT_LINE_LOST);
			sys_post_event(SYS_EV_LINE_LOST);
			(void)tk_set_flg(flg_line_evt, LEV_LOST);
		}
		if ((out.events & LF_EV_FOUND) != 0U) {
			sys_fault_clear(SYS_FLT_LINE_LOST | SYS_FLT_LINE_RECOVERY);
			sys_post_event(SYS_EV_LINE_FOUND);
			(void)tk_set_flg(flg_line_evt, LEV_FOUND);
		}
		if ((out.events & LF_EV_JUNCTION) != 0U) {
			junctions++;
			(void)tk_set_flg(flg_line_evt, LEV_JUNCTION);
		}
		if ((out.events & LF_EV_RECOVERY_FAIL) != 0U) {
			/* Reserved for compatibility; line recovery is non-latching. */
			sys_fault_clear(SYS_FLT_LINE_RECOVERY);
		}

		st.ir_black = black;
		st.pos = lf.pos;
		st.state = lf.calibrated ? lf.state : LS_UNKNOWN;
		st.seq = seq++;
		st.steer = (B)((nav && out.steering) ? steer : 0);
		st.calibrated = lf.calibrated ? 1U : 0U;
		st.junctions = junctions;
		st.lost_count = lost_count;
		st.speed_frac = out.speed_frac;
		app_seq_write_begin(&s3_line_seq);
		s3_line_word = s3_line_pack(&st);
		s3_line_time = app_now_ms();
		app_seq_write_end(&s3_line_seq);

		task_stat_end(&s3_stat_line, start, (UW)S3_PERIOD_MS * 1000U / 2U);
	}
}
