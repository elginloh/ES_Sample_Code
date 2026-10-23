/*
 *	motion_api.c
 *	Motion Control interface used by the other subsystems (see
 *	subsystem2.h).  Nothing here touches the motors: every function only
 *	queues a packed command, publishes a packed setpoint word or reads a
 *	snapshot, so Buddy 2 stays the sole owner of motor actuation.
 */

#include "subsystem2.h"
#include "s2_private.h"
#include "motion_cfg.h"
#include "drv_encoder.h"

EXPORT volatile UW	s2_result_word[MSRC_COUNT];
EXPORT volatile UW	s2_steer_word;
EXPORT volatile UW	s2_cap_word;
EXPORT volatile UW	s2_stop_req;
EXPORT volatile UW	s2_armed;
EXPORT volatile UW	s2_speed_avg_mms;
EXPORT volatile UW	s2_creep_mms;
EXPORT APP_SEQ		s2_mstat_seq;
EXPORT UW		s2_mstat_words[2];

/* Per-source sequence counters.  Each source has exactly one producer
   task (barcode task, avoidance task, communication task), so each
   counter has a single writer. */
LOCAL UB		src_seq[MSRC_COUNT];
LOCAL UB		steer_seq_ctr;		/* writer: line task */
LOCAL UB		cap_seq_ctr;		/* writer: scan task */

EXPORT void motion_api_init(void)
{
	UINT	i;

	for (i = 0; i < MSRC_COUNT; i++) {
		s2_result_word[i] = 0;
		src_seq[i] = 0;
	}
	s2_steer_word = 0;
	s2_cap_word = 0;
	s2_stop_req = 0;
#if APP_TEST
	s2_armed = 0;		/* test image: never drive away on its own */
#else
	s2_armed = 1;		/* Week 6 statechart: start after initialisation */
#endif
	s2_speed_avg_mms = 0;
	s2_creep_mms = MIN_CTRL_SPEED_MMS;
	s2_mstat_seq.seq = 0;
	s2_mstat_words[0] = 0;
	s2_mstat_words[1] = 0;
	steer_seq_ctr = 0;
	cap_seq_ctr = 0;
}

EXPORT UW motion_make_cmd(UW type, UW src, INT arg, UW speed_lvl)
{
	UW	s = (UW)(src & MCMD_SRC_MASK);
	UW	seq = ((UW)src_seq[s] + 1U) & MCMD_SEQ_MASK;

	src_seq[s] = (UB)seq;
	return mcmd_pack(type, s, seq, sys_epoch(), speed_lvl, arg);
}

EXPORT ER motion_request(UW cmd)
{
	ER	er = tk_snd_mbf(mbf_motion_cmd, &cmd, (INT)sizeof(cmd), TMO_POL);

	if (er == E_OK) {
		(void)tk_set_flg(flg_motion, MF_CMD);
	} else if (er == E_TMOUT) {
		sys_fault_set(SYS_FLT_CMD_QUEUE);	/* bounded queue overflow */
	} else {
		sys_fault_set(SYS_FLT_TASK_COMM);
	}
	return er;
}

EXPORT ER motion_command(UW type, UW src, INT arg, UW speed_lvl, UW *cmd_out)
{
	UW	cmd = motion_make_cmd(type, src, arg, speed_lvl);

	if (cmd_out != NULL) {
		*cmd_out = cmd;
	}
	return motion_request(cmd);
}

EXPORT UW motion_last_result(UW src)
{
	return app_load(&s2_result_word[src & MCMD_SRC_MASK]);
}

EXPORT ER motion_wait_result(UW cmd, TMO tmo, UW *result)
{
	UW	src = mcmd_src(cmd);
	UW	start = app_now_ms();
	UINT	ptn;
	ER	er;

	for (;;) {
		UW	r = motion_last_result(src);
		TMO	left = TMO_FEVR;

		if ((r & MRES_VALID) != 0U && mres_seq(r) == mcmd_seq(cmd)
		    && mres_type(r) == mcmd_type(cmd)) {
			if (result != NULL) {
				*result = r;
			}
			return E_OK;
		}
		if (tmo != TMO_FEVR) {
			UW	el = app_now_ms() - start;

			if (el >= (UW)tmo) {
				return E_TMOUT;
			}
			left = (TMO)((UW)tmo - el);
		}
		er = tk_wai_flg(flg_motion_done, 1U << src, TWF_ORW | TWF_BITCLR,
				&ptn, left);
		if (er < E_OK && er != E_TMOUT) {
			return er;
		}
	}
}

EXPORT void motion_stop(UW src)
{
	(void)app_atomic_or(&s2_stop_req, 1UL << (src & MCMD_SRC_MASK));
	if (flg_motion > 0) {
		(void)tk_set_flg(flg_motion, MF_STOP);
	}
}

EXPORT void motion_set_steer(INT steer, UW speed_frac)
{
	steer_seq_ctr = (UB)((steer_seq_ctr + 1U) & STEER_SEQ_MASK);
	app_store(&s2_steer_word, steer_pack(steer_seq_ctr, steer, speed_frac));
}

EXPORT void motion_clear_steer(void)
{
	app_store(&s2_steer_word, 0U);
}

EXPORT void motion_set_speed_cap(UW cap_mms)
{
	if (cap_mms == 0U) {
		app_store(&s2_cap_word, 0U);
		return;
	}
	cap_seq_ctr = (UB)((cap_seq_ctr + 1U) & SCAP_SEQ_MASK);
	app_store(&s2_cap_word, scap_pack(cap_seq_ctr, cap_mms));
}

EXPORT void motion_get_status(MOTION_STATUS *st)
{
	UW	w[2] = {0, 0};
	UINT	tries;

	for (tries = 0; tries < APP_SEQ_READ_TRIES; tries++) {
		UW	s = app_seq_read_begin(&s2_mstat_seq);

		w[0] = s2_mstat_words[0];
		w[1] = s2_mstat_words[1];
		if (!app_seq_read_retry(&s2_mstat_seq, s)) {
			break;
		}
	}
	mstat_unpack(w, st);
}

EXPORT void motion_get_odometry(W *dist_mm, W *speed_mms)
{
	ENC_SNAP	l, r;

	if (dist_mm != NULL) {
		if (encoder_snapshot(ENC_LEFT, &l) && encoder_snapshot(ENC_RIGHT, &r)) {
			D	um = ((D)l.ticks + (D)r.ticks) * (D)ENC_UM_PER_TICK / 2;

			*dist_mm = (W)(um / 1000);
		}
	}
	if (speed_mms != NULL) {
		*speed_mms = (W)app_load(&s2_speed_avg_mms);
	}
}

EXPORT W motion_creep_speed_mms(void)
{
	return (W)app_load(&s2_creep_mms);
}

EXPORT void motion_set_armed(BOOL armed)
{
	app_store(&s2_armed, armed ? 1U : 0U);
}
