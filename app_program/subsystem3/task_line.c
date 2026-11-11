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

EXPORT TASK_STAT	s3_stat_line;
EXPORT volatile UW	s3_line_word;
EXPORT volatile UW	s3_line_time;
LOCAL volatile UW	s3_junction_time;
EXPORT volatile UW	s3_black_level = 1U;
EXPORT APP_SEQ		s3_line_seq;

LOCAL LINE_FOLLOW	lf;

/* Debug: print the IR sensors whenever they change (1 = on, 0 = off).
   Works in every mode, so the sensors can be checked by hand with the
   car still.  Set to 0 for the final build. */
#define S3_LINE_LOG	1

EXPORT void line_cyclic_handler(void *exinf)
{
	(void)exinf;
	(void)tk_wup_tsk(tid_line);
}

EXPORT void s3_get_line_status(S3_LINE_STATUS *s)
{
	UW	w = 0, t = 0, jt = 0;
	UINT	tries;

	for (tries = 0; tries < APP_SEQ_READ_TRIES; tries++) {
		UW	q = app_seq_read_begin(&s3_line_seq);

		w = s3_line_word;
		t = s3_line_time;
		jt = s3_junction_time;
		if (!app_seq_read_retry(&s3_line_seq, q)) {
			break;
		}
	}
	s3_line_unpack(w, s);
	s->time_ms = t;
	s->junction_time_ms = jt;
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
		tm_printf((UB*)"line: IR ready from current surface, black level = %d\n",
			  (INT)lf.black_level[IR_C]);
	} else {
		/* Retry each 200 ms if a sensor is flickering during startup. */
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
	BOOL		was_scan_limited = FALSE;
	BOOL		was_executing = FALSE;
	UB		seq = 0, junctions = 0, lost_count = 0;
	UW		junction_time = 0;
	UB		prev_black = 0xFFU;	/* S3_LINE_LOG */
	UB		prev_pos = 0xFFU;	/* S3_LINE_LOG */
	UB		prev_state = 0xFFU;	/* S3_LINE_LOG */
	W		odo = 0;

	(void)stacd;
	(void)exinf;
	lf_init(&lf);

	for (;;) {
		UW	start, vsw, mode;
		UB	raw, black;
		BOOL	nav;
		BOOL	scan_limited = FALSE;
		BOOL	executing;
		S3_BC_STATUS barcode;
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
		s3_get_barcode_status(&barcode);
		executing = (barcode.state == BCS_EXECUTING) ? TRUE : FALSE;
		if (executing) {
			/* Buddy 2 owns the wheels for the junction turn.  What
			   the sensors see while it turns (the centre sweeping
			   over the new line, long all-white spells) must not
			   become a hold, a line loss or a junction afterwards. */
			if (lf.calibrated && lf.state != LS_ON_LINE) {
				sys_fault_clear(SYS_FLT_LINE_LOST | SYS_FLT_LINE_RECOVERY);
				sys_post_event(SYS_EV_LINE_FOUND);
				(void)tk_set_flg(flg_line_evt, LEV_FOUND);
			}
			lf_reset_tracking(&lf);
		} else if (was_executing && nav) {
			if (barcode.mres == MRES_DONE && barcode.nav == NAV_LEFT
			    && (black & (IR_BIT(IR_C) | IR_BIT(IR_L))) == IR_BIT(IR_L)) {
				lf_reset_tracking(&lf);
				tm_printf((UB*)"line: GP2 acquired outgoing branch, resuming steering\n");
			} else if (barcode.mres == MRES_DONE) {
				BOOL centre_acquired = barcode.nav == NAV_RIGHT || barcode.nav == NAV_LEFT;

				lf_turn_done(&lf, centre_acquired);
				tm_printf((UB*)"line: %s\n", centre_acquired
					? "outgoing branch acquired, driving straight"
					: "barcode command done, realigning on the line");
			}
		}
		was_executing = executing;
		lf_step(&lf, black, odo, S3_PERIOD_MS, nav, &out);
		if (nav && out.steering && out.speed_frac > S3_BC_SPEED_FRAC) {
			if (barcode.state == BCS_CAPTURING) {
				/* Scale forward speed and steering together so the
				   correction keeps the same turn radius. */
				out.steer = out.steer * (INT)S3_BC_SPEED_FRAC / (INT)out.speed_frac;
				out.speed_frac = S3_BC_SPEED_FRAC;
				scan_limited = TRUE;
			}
		}
		if (scan_limited != was_scan_limited) {
			tm_printf((UB*)"line: barcode speed cap %s (speed fraction %d/15)\n",
				  scan_limited ? "active" : "released", (INT)out.speed_frac);
			was_scan_limited = scan_limited;
		}
		steer = IR_SIDES_MIRRORED ? -out.steer : out.steer;
#if S3_LINE_LOG
		/* L and R are the car's real sides, steer is what the
		   motors get (+ = clockwise); err is in the line follower's
		   own labels. */
		if (black != prev_black || lf.pos != prev_pos) {
			tm_printf((UB*)"ir: L%d C%d R%d | err %d state %d steer %d\n",
				  (INT)((black >> (IR_SIDES_MIRRORED ? IR_R : IR_L)) & 1U),
				  (INT)((black >> IR_C) & 1U),
				  (INT)((black >> (IR_SIDES_MIRRORED ? IR_L : IR_R)) & 1U),
				  (INT)lf.err, (INT)lf.state, (INT)steer);
			prev_black = black;
			prev_pos = lf.pos;
		}
		if (lf.state == LS_SEARCHING && prev_state != LS_SEARCHING) {
			tm_printf((UB*)"line: lost, turning %s to find it\n",
				  (steer < 0) ? "left" : "right");
		}
		if ((out.events & LF_EV_FOUND) != 0U && prev_state == LS_SEARCHING) {
			tm_printf((UB*)"line: found\n");
		}
		prev_state = lf.state;
#endif

		if (nav && !executing && out.steering) {
			motion_set_steer(steer, out.speed_frac);
		} else {
			motion_clear_steer();
		}
		/* Every mode: Buddy 2 uses these readings to find the outgoing tape. */
		if (lf.calibrated) {
			motion_set_line_seen(
				((black >> (IR_SIDES_MIRRORED ? IR_R : IR_L)) & 1U) != 0U,
				((black >> IR_C) & 1U) != 0U,
				((black >> (IR_SIDES_MIRRORED ? IR_L : IR_R)) & 1U) != 0U);
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
		if (nav && !executing && (out.events & LF_EV_JUNCTION) != 0U) {
			junctions++;
			tm_printf((UB*)"line: junction confirmed (GP6 + GP2 black), count %d\n", (INT)junctions);
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
		if (nav && !executing && lf.pos == LP_JUNCTION
		    && lf.junction_ms >= LF_JUNCTION_CONFIRM_MS) {
			junction_time = s3_line_time;
		}
		s3_junction_time = junction_time;
		app_seq_write_end(&s3_line_seq);

		task_stat_end(&s3_stat_line, start, (UW)S3_PERIOD_MS * 1000U / 2U);
	}
}
