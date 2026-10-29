/*
 *	task_motion.c
 *	Motion Control task (Week 6 task table: High, periodic) and its
 *	cyclic handler.
 *
 *	One loop, woken by flg_motion:
 *	  MF_STOP   stop() bypass: abort everything, motors off at once (NFR6)
 *	  MF_FAULT  new critical fault: motors off at once
 *	  MF_CMD    drain the command queue (arbitration in mc_submit)
 *	  MF_TICK   every 20 ms: measure, control, update the vehicle mode,
 *	            publish status, results and the vehicle status word
 *	The cyclic handler also acts as a motor watchdog: if the loop misses
 *	MOTOR_WDOG_TICKS periods it switches the motors off itself and raises
 *	SYS_FLT_MOTOR_WDOG (the handler is part of Buddy 2, the motor owner).
 */

#include "subsystem2.h"
#include "s2_private.h"
#include "motion_ctrl.h"
#include "vehicle_fsm.h"
#include "motion_cfg.h"
#include "drv_motor.h"
#include "drv_encoder.h"

/* 1 = print each wheel's target/measured speed, motor duty and encoder
   edge count every 500 ms, plus shared GPIO interrupts per second.
   Bench check: with the car stopped, turn a wheel by hand and its edge
   count must rise.  Set back to 0 once the encoders are confirmed. */
#ifndef S2_SPEED_LOG
#define S2_SPEED_LOG		1
#endif

EXPORT TASK_STAT	s2_stat_motion;

LOCAL MOTION_CTRL	mc;
LOCAL VFSM		vf;
LOCAL MC_OUT		mout;

LOCAL volatile UW	wdog_ticks;		/* writer: cyclic handler */
LOCAL volatile UW	wdog_kick;		/* writer: motion task    */
LOCAL volatile UW	wdog_tripped;		/* set by handler, cleared by task */
LOCAL volatile UW	loop_running;		/* writer: motion task    */

LOCAL UW	cycle_ms;			/* local time, +20 ms per tick */
/* Encoder counts of the last SPD_WIN_CYCLES control periods (100 ms) */
#define SPD_WIN_CYCLES		5
typedef struct {
	W	ticks[SPD_WIN_CYCLES];
	UW	t_us[SPD_WIN_CYCLES];
	UINT	n;				/* samples held */
	UINT	i;				/* next slot = oldest once full */
} SPD_WIN;

LOCAL W		spd_l, spd_r;			/* filtered wheel speeds, mm/s */
LOCAL SPD_WIN	win_l, win_r;
LOCAL INT	dir_l = 1, dir_r = 1;
LOCAL UW	odo_um;				/* total travel, um */
LOCAL BOOL	steer_seen;
LOCAL UW	steer_seen_seq, steer_seen_ms;
LOCAL BOOL	cap_seen;
LOCAL UW	cap_seen_seq, cap_seen_ms;
LOCAL BOOL	pend_nav_started, pend_nav_finished, pend_resume;
LOCAL BOOL	enc_valid;
LOCAL W		dist_l_um, dist_r_um;

/* ------------------------------------------------------------------ */

EXPORT void motion_cyclic_handler(void *exinf)
{
	UW	t = wdog_ticks + 1U;

	(void)exinf;
	wdog_ticks = t;
	if (loop_running != 0U && (UW)(t - wdog_kick) >= MOTOR_WDOG_TICKS) {
		motor_off();				/* loop is late: cut power */
		if (wdog_tripped == 0U) {
			wdog_tripped = 1U;
			sys_fault_set(SYS_FLT_MOTOR_WDOG);
		}
	}
	(void)tk_set_flg(flg_motion, MF_TICK);
}

/* ------------------------------------------------------------------ */

/* Speed from the edges counted over the last 100 ms, not from the last
   edge-to-edge period.  At crawl speed the encoder chatters (edge pairs
   1-2 ms apart, then ~50 ms of nothing), which ENC_MIN_EDGE_US does not
   reject: one short period read as a fast wheel (telemetry: 235 mm/s
   reported, 6 mm/s travelled) and the PID cut the power.  A stray edge
   now moves the reading by one tick in the window instead. */
LOCAL W wheel_speed(const ENC_SNAP *s, UW now_us, SPD_WIN *w, INT *dir)
{
	W	dticks = 0;
	UW	dt = 0;

	if (w->n > 0U) {
		UINT	last = (w->i + SPD_WIN_CYCLES - 1U) % SPD_WIN_CYCLES;
		UINT	first = (w->n < SPD_WIN_CYCLES) ? 0U : w->i;
		W	step = s->ticks - w->ticks[last];

		if (step > 0) {
			*dir = 1;
		} else if (step < 0) {
			*dir = -1;
		}
		dticks = s->ticks - w->ticks[first];
		dt = now_us - w->t_us[first];
	}
	w->ticks[w->i] = s->ticks;
	w->t_us[w->i] = now_us;
	w->i = (w->i + 1U) % SPD_WIN_CYCLES;
	if (w->n < SPD_WIN_CYCLES) {
		w->n++;
	}
	if (dticks < 0) {
		dticks = -dticks;
	}
	if (s->edges < 2U || dt == 0U) {
		return 0;
	}
	return (W)(*dir) * (W)((UW)dticks * ENC_UM_PER_TICK * 1000UL / dt);
}

LOCAL void measure(void)
{
	ENC_SNAP	l, r;
	UW		now = hw_time_us();
	W		old_l = dist_l_um, old_r = dist_r_um;
	W		dl, dr;

	if (encoder_snapshot(ENC_LEFT, &l) && encoder_snapshot(ENC_RIGHT, &r)) {
		dist_l_um = l.ticks * (W)ENC_UM_PER_TICK;
		dist_r_um = r.ticks * (W)ENC_UM_PER_TICK;
		spd_l = (spd_l + wheel_speed(&l, now, &win_l, &dir_l)) / 2;
		spd_r = (spd_r + wheel_speed(&r, now, &win_r, &dir_r)) / 2;
		enc_valid = TRUE;
	} else {
		enc_valid = FALSE;	/* keep the previous distances */
	}
	dl = dist_l_um - old_l;
	dr = dist_r_um - old_r;
	odo_um += (UW)(((dl < 0) ? -dl : dl) + ((dr < 0) ? -dr : dr)) / 2U;
}

/* Freshness of a single-writer word whose 4-bit seq must keep changing */
LOCAL BOOL fresh(UW seq, BOOL *seen, UW *seen_seq, UW *seen_ms, UW stale_ms)
{
	if (!*seen || seq != *seen_seq) {
		*seen = TRUE;
		*seen_seq = seq;
		*seen_ms = cycle_ms;
	}
	return ((UW)(cycle_ms - *seen_ms) <= stale_ms) ? TRUE : FALSE;
}

LOCAL void post_results(void)
{
	UINT	i;

	if (mout.result_mask == 0U) {
		return;
	}
	for (i = 0; i < MSRC_COUNT; i++) {
		if ((mout.result_mask & (1UL << i)) != 0U) {
			app_store(&s2_result_word[i], mout.result[i]);
		}
	}
	(void)tk_set_flg(flg_motion_done, (UINT)mout.result_mask);
	mout.result_mask = 0;
}

LOCAL void apply_faults(void)
{
	if (mout.flt_clr != 0U) {
		sys_fault_clear(mout.flt_clr);
		mout.flt_clr = 0;
	}
	if (mout.flt_set != 0U) {
		sys_fault_set(mout.flt_set);
		mout.flt_set = 0;
	}
	pend_nav_started = pend_nav_started || mout.nav_started;
	pend_nav_finished = pend_nav_finished || mout.nav_finished;
	mout.nav_started = FALSE;
	mout.nav_finished = FALSE;
}

/* Supervisory RESUME / CLEAR (Buddy 1, already validated by its decoder;
   validated again here).  The decision is sys_recovery_decide(): RESUME
   only leaves Safe Stop when no critical condition remains, so a remote
   command never overrides safety. */
LOCAL void supervisory_recovery(UW cmd)
{
	UW	reason = mcmd_validate(cmd);
	UW	clear = 0;
	UW	r;

	if (reason == MREJ_NONE) {
		reason = sys_recovery_decide(mcmd_type(cmd), vf.mode, sys_fault_get(),
					     &clear);
		sys_fault_clear(clear);
		if (reason == MREJ_NONE && mcmd_type(cmd) == MCMD_RESUME) {
			pend_resume = TRUE;
		}
	}
	r = mres_pack(cmd, (reason == MREJ_NONE) ? MRES_DONE : MRES_REJECTED, reason);
	mout.result[MSRC_SUPER] = r;
	mout.result_mask |= (1UL << MSRC_SUPER);
}

/* Restart button: Robo Pico GP20 (active low).  In Safe Stop it reboots
   the Pico through the RP2040 watchdog: the same clean start as a power
   cycle or a reflash, with no state carried over.  Needed because the
   robot's power switch does not power the Pico down while the USB cable
   is plugged in, and this build has no MQTT RESUME.  (A supervisory
   RESUME without a reboot was tried first; after it the car did not
   drive properly, while a fresh boot did.)  Ignored outside Safe Stop,
   so it can never cut in while the car is driving. */
#define S2_RESUME_BTN_PIN	20
#define S2_RESUME_BTN_MS	60	/* debounce */
#define WDG_CTRL		0x40058000UL	/* RP2040 WATCHDOG CTRL */
#define WDG_CTRL_TRIGGER	(1UL << 31)

LOCAL UW	btn_ms;
LOCAL BOOL	btn_done;

LOCAL void resume_button(void)
{
	BOOL	pressed = (((in_w(GPIO_IN) >> S2_RESUME_BTN_PIN) & 1UL) == 0UL);

	if (!pressed) {
		btn_ms = 0;
		btn_done = FALSE;
		return;
	}
	if (btn_done) {
		return;			/* one action per press */
	}
	btn_ms += MOTION_PERIOD_MS;
	if (btn_ms < S2_RESUME_BTN_MS) {
		return;
	}
	btn_done = TRUE;
	if (vf.mode != VM_SAFE_STOP) {
		return;
	}
	motor_off();
	tm_printf((UB*)"button: restarting (faults 0x%07x)\n", (INT)sys_fault_get());
	(void)tk_dly_tsk(50);		/* let the console line out */
	out_w(WDG_CTRL, WDG_CTRL_TRIGGER);	/* full chip reset */
}

LOCAL void handle_command(UW cmd)
{
	UW	type = mcmd_type(cmd);
	UW	src = mcmd_src(cmd);

	if (src == MSRC_SUPER && (type == MCMD_RESUME || type == MCMD_CLEAR)) {
		supervisory_recovery(cmd);
		return;
	}
	if (src == MSRC_SUPER && type == MCMD_STOP && mcmd_validate(cmd) == MREJ_NONE) {
		sys_fault_set(SYS_FLT_REMOTE_STOP);	/* operator hold -> Safe Stop */
	}
	if (mc_submit(&mc, cmd, vf.mode, vf.epoch, &mout) == MREJ_NONE
	    && type == MCMD_STOP) {
		motor_off();
	}
}

LOCAL void drain_commands(void)
{
	UW	cmd;
	INT	n;

	while ((n = tk_rcv_mbf(mbf_motion_cmd, &cmd, TMO_POL)) > 0) {
		if (n == (INT)sizeof(cmd)) {
			handle_command(cmd);
		}
	}
}

LOCAL void publish(void)
{
	MOTION_STATUS	st;
	UW		w[2];
	UW		vsw;
	UW		esc = (mc.fault_retries > 3U) ? 3U : mc.fault_retries;
	D		hd;
	W		head;

	vsw = ((UW)vf.mode << VSW_MODE_SHIFT)
	    | ((UW)vf.epoch << VSW_EPOCH_SHIFT)
	    | ((UW)mc.state << VSW_MSTATE_SHIFT)
	    | ((mc.has_active ? mcmd_src(mc.active) : 0U) << VSW_SRC_SHIFT)
	    | (mc.has_active ? VSW_CMD_ACTIVE : 0U)
	    | (vf.ir_ok ? VSW_IR_OK : 0U)
	    | (vf.imu_ok ? VSW_IMU_OK : 0U)
	    | ((app_load(&s2_armed) != 0U) ? VSW_ARMED : 0U)
	    | (vf.line_lost ? VSW_LINE_CAP : 0U)
	    | ((cap_seen && (s2_cap_word & SCAP_VALID) != 0U) ? VSW_OBST_CAP : 0U)
	    | (esc << VSW_ESC_SHIFT);
	sys_vsw_publish(vsw);

	/* encoder heading estimate: (dl - dr) / track, in degrees */
	hd = ((D)(dist_l_um - dist_r_um) * 57296LL) / ((D)TRACK_WIDTH_MM * 1000000LL);
	head = (W)(hd % 360);
	if (head < 0) {
		head += 360;
	}
	st.speed_l_mms = (H)(enc_valid ? spd_l : MSTAT_SPEED_INVALID);
	st.speed_r_mms = (H)(enc_valid ? spd_r : MSTAT_SPEED_INVALID);
	st.mstate = mc.state;
	st.source = (UB)(mc.has_active ? mcmd_src(mc.active) : 0U);
	st.cmd_active = mc.has_active ? 1U : 0U;
	st.stall_l = mc.stall_l ? 1U : 0U;
	st.stall_r = mc.stall_r ? 1U : 0U;
	st.enc_ok = enc_valid ? 1U : 0U;
	st.odometer_cm = (UH)((odo_um / 10000U > 0xFFFFU) ? 0xFFFFU : odo_um / 10000U);
	st.heading_deg = (UH)head;
	st.cruise_mms = (UH)mc.cruise_mms;
	mstat_pack(&st, w);
	app_seq_write_begin(&s2_mstat_seq);
	s2_mstat_words[0] = w[0];
	s2_mstat_words[1] = w[1];
	app_seq_write_end(&s2_mstat_seq);

	app_store(&s2_speed_avg_mms, (UW)((spd_l + spd_r) / 2));
	app_store(&s2_creep_mms, (UW)mc_creep_speed(&mc));
}

#if S2_SPEED_LOG
LOCAL void speed_log(void)
{
	static UW	n, irq0;
	ENC_SNAP	l, r;
	UW		irq;

	if (++n % 25U != 0U
	    || !encoder_snapshot(ENC_LEFT, &l) || !encoder_snapshot(ENC_RIGHT, &r)) {
		return;
	}
	irq = gpio_irq_count();
	tm_printf((UB*)"speed: L tgt %d meas %d duty %d edges %d | "
		  "R tgt %d meas %d duty %d edges %d | irq/s %d\n",
		  (INT)mc.tgt_l_mms, (INT)spd_l, (INT)mout.duty_l_pm, (INT)l.edges,
		  (INT)mc.tgt_r_mms, (INT)spd_r, (INT)mout.duty_r_pm, (INT)r.edges,
		  (INT)((irq - irq0) * 2U));
	irq0 = irq;
}
#endif

LOCAL void control_cycle(void)
{
	MC_IN		in;
	VFSM_IN		vin;
	VFSM_OUT	vout;
	UW		start = task_stat_begin(&s2_stat_motion);
	UW		sw = app_load(&s2_steer_word);
	UW		cw = app_load(&s2_cap_word);
	BOOL		armed = (app_load(&s2_armed) != 0U) ? TRUE : FALSE;

	cycle_ms += MOTION_PERIOD_MS;
	measure();
	resume_button();

	/* 1. control with the current mode */
	in.dist_l_um = dist_l_um;
	in.dist_r_um = dist_r_um;
	in.speed_l_mms = spd_l;
	in.speed_r_mms = spd_r;
	in.enc_valid = enc_valid;
	in.vmode = vf.mode;
	in.armed = armed && (wdog_tripped == 0U);
	in.steer = sw;
	in.steer_fresh = ((sw & STEER_VALID) != 0U)
		? fresh(steer_seq(sw), &steer_seen, &steer_seen_seq, &steer_seen_ms,
			STEER_STALE_MS)
		: FALSE;
	in.cap_mms = (((cw & SCAP_VALID) != 0U)
		&& fresh(scap_seq(cw), &cap_seen, &cap_seen_seq, &cap_seen_ms,
			 SCAP_STALE_MS)) ? scap_value(cw) : 0U;
	in.line_lost_cap = vf.line_lost;
	mc_step(&mc, &in, MOTION_PERIOD_MS, &mout);
	apply_faults();

	/* 2. vehicle mode (Buddy 2 owns it) */
	vin.events = sys_take_events();
	vin.faults = sys_fault_get();
	vin.crit_mask = sys_fault_crit_mask();
	vin.armed = armed;
	vin.nav_started = pend_nav_started;
	vin.nav_finished = pend_nav_finished;
	vin.resume = pend_resume;
	vin.dt_ms = MOTION_PERIOD_MS;
	vfsm_step(&vf, &vin, &vout);
	pend_nav_started = FALSE;
	pend_nav_finished = FALSE;
	pend_resume = FALSE;
	if (vout.flt_set != 0U) {
		sys_fault_set(vout.flt_set);
	}
	if (vout.changed) {
		mc_mode_changed(&mc, vf.mode, &mout);
		apply_faults();
		if (vf.mode == VM_INIT) {
			wdog_tripped = 0U;	/* fault bit cleared by RESUME */
		}
		tm_printf((UB*)"motion: mode %s -> %s (epoch %d, faults 0x%07x)\n",
			  sys_mode_name(vout.old_mode), sys_mode_name(vf.mode),
			  (INT)vf.epoch, (INT)sys_fault_get());
	}

	/* 3. outputs: nothing moves outside the driving modes */
	if (vf.mode == VM_INIT || vf.mode == VM_SAFE_STOP || !armed
	    || wdog_tripped != 0U || vout.changed) {
		motor_off();
	} else {
		motor_set(mout.duty_l_pm, mout.duty_r_pm);
	}
	encoder_set_direction(ENC_LEFT, mout.dir_l);
	encoder_set_direction(ENC_RIGHT, mout.dir_r);

	post_results();
	publish();
#if S2_SPEED_LOG
	speed_log();
#endif
	task_stat_end(&s2_stat_motion, start, (UW)MOTION_PERIOD_MS * 1000U / 2U);
}

EXPORT void motion_task(INT stacd, void *exinf)
{
	UINT	ptn;
	ER	er;

	(void)stacd;
	(void)exinf;

	/* The shared GPIO interrupt is taken on this task's core */
	er = gpio_irq_start();
	if (er < E_OK) {
		CHECK(er, "gpio_irq_start");
		sys_fault_set(SYS_FLT_TASK_COMM);
	}
	if (motor_init() < E_OK) {
		sys_fault_set(SYS_FLT_TASK_COMM);
	}
	gpio_input_config(S2_RESUME_BTN_PIN, GPIO_PULL_UP);
	mc_init(&mc);
	vfsm_init(&vf);
	mc_out_clear(&mout);
	publish();
	wdog_kick = wdog_ticks;
	app_store(&loop_running, 1U);

	for (;;) {
		er = tk_wai_flg(flg_motion, MF_ALL, TWF_ORW | TWF_BITCLR, &ptn, TMO_FEVR);
		if (er < E_OK) {
			CHECK(er, "motion tk_wai_flg");
			motor_off();
			(void)tk_dly_tsk(MOTION_PERIOD_MS);
			continue;
		}
		wdog_kick = wdog_ticks;

		if ((ptn & MF_STOP) != 0U && app_atomic_xchg(&s2_stop_req, 0U) != 0U) {
			mc_force_stop(&mc, &mout);
			motor_off();
		}
		if ((ptn & MF_FAULT) != 0U
		    && (sys_fault_get() & sys_fault_crit_mask()) != 0U) {
			mc_force_stop(&mc, &mout);
			motor_off();		/* Safe Stop starts now */
		}
		if ((ptn & (MF_CMD | MF_TICK)) != 0U) {
			drain_commands();
		}
		if ((ptn & MF_TICK) != 0U) {
			control_cycle();
		} else {
			apply_faults();
			post_results();
		}
	}
}
