/*
 *	task_profile.c
 *	Obstacle Profile task (Week 6: Medium, queue-driven).  Collects the
 *	packed samples of one scan phase, builds the profile when the last
 *	sample arrives and sends it to the Avoidance & Recovery task.  Coarse
 *	and fine use the same scan id, so a phase change must also clear the
 *	buffer; otherwise the two angular grids are mixed into one profile.
 *	Samples of another scan id likewise start over, so a late sample can
 *	never mix two obstacles (Tribunal 1: "scan identifiers, validity
 *	checks").
 */

#include "subsystem5.h"

EXPORT APP_SEQ	s5_prof_seq;
EXPORT UW	s5_prof_words[2];

LOCAL OP_SAMPLE	buf[OP_MAX_SAMPLES];

EXPORT void s5_get_profile(S5_PROFILE *p)
{
	UW	w[2] = {0, 0};
	UINT	tries;

	for (tries = 0; tries < APP_SEQ_READ_TRIES; tries++) {
		UW	q = app_seq_read_begin(&s5_prof_seq);

		w[0] = s5_prof_words[0];
		w[1] = s5_prof_words[1];
		if (!app_seq_read_retry(&s5_prof_seq, q)) {
			break;
		}
	}
	s5_profile_unpack(w, p);
}

EXPORT void profile_task(INT stacd, void *exinf)
{
	S5_SAMPLE	smp;
	S5_PROFILE	prof;
	UW		w, pw[2];
	UINT		n = 0;
	UB		cur_id = 0xFFU;
	UB		cur_phase = 0xFFU;
	INT		limit = 0;
	W		raw_width;
	BOOL		aborted = FALSE;

	(void)stacd;
	(void)exinf;
	for (;;) {
		if (tk_rcv_mbf(mbf_scan_samples, &w, TMO_FEVR) != (INT)sizeof(w)) {
			continue;
		}
		s5_sample_unpack(w, &smp);
		if (smp.scan_id != cur_id || smp.phase != cur_phase) {
			cur_id = smp.scan_id;		/* new obstacle or scan phase */
			cur_phase = smp.phase;
			n = 0;
			limit = 0;
			aborted = FALSE;
		}
		if (n < OP_MAX_SAMPLES) {
			buf[n].angle_deg = smp.angle_deg;
			buf[n].dist_mm = (H)((smp.status == SS_OK) ? s5_dist_mm(smp.dist_code) : -1);
			buf[n].status = smp.status;
			n++;
		}
		if (smp.status == SS_ABORTED) {
			aborted = TRUE;
		}
		if (smp.angle_deg > limit) {
			limit = smp.angle_deg;
		} else if (-smp.angle_deg > limit) {
			limit = -smp.angle_deg;
		}
		if (smp.last == 0U) {
			continue;
		}

		op_build(buf, n, limit, &prof);
		raw_width = prof.width_mm;
		if (aborted) {
			prof.status = PRF_INVALID;	/* sensor silent for the whole scan */
		} else {
			op_beam_compensate(&prof, S5_BEAM_COMP_DEG);
		}
		tm_printf((UB*)"profile: scan %d phase %d, samples %d, limit %d deg, "
			  "status %d, sensor %d mm, width %d mm (raw %d), "
			  "clear L %d R %d mm\n",
			  (INT)cur_id, (INT)cur_phase, (INT)n, limit,
			  (INT)prof.status, (INT)prof.dist_mm, (INT)prof.width_mm, (INT)raw_width,
			  (INT)prof.clear_left_mm, (INT)prof.clear_right_mm);
		prof.scan_id = cur_id;
		prof.phase = cur_phase;
		s5_profile_pack(&prof, pw);
		app_seq_write_begin(&s5_prof_seq);
		s5_prof_words[0] = pw[0];
		s5_prof_words[1] = pw[1];
		app_seq_write_end(&s5_prof_seq);
		if (tk_snd_mbf(mbf_profile, pw, (INT)sizeof(pw), TMO_POL) != E_OK) {
			sys_fault_set(SYS_FLT_TASK_COMM);
		}
	}
}
