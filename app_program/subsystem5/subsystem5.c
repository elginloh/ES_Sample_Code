/*
 *	subsystem5.c
 *	Subsystem 5 -- ultrasonic scanning, obstacle profiling, avoidance and
 *	recovery: kernel objects and tasks.
 *
 *	Kernel objects (TEAM.md section 6 budget):
 *	  flg_avoid         event flag   obstacle detected / emergency
 *	  flg_us            event flag   echo complete (driver <-> scan task)
 *	  mbf_scan_req      message buf   4 x 4-byte scan requests
 *	  mbf_scan_samples  message buf  64 x 4-byte packed samples
 *	  mbf_profile       message buf   4 x 8-byte packed profiles
 *	  scan_task 15 (core 1), avoid_task 16, profile_task 19
 *	Pins: GP4 TRIG, GP5 ECHO (shared GPIO IRQ), GP15 servo (PWM slice 7 B)
 */

#include "subsystem5.h"
#include "drv_ultrasonic.h"

EXPORT ID	flg_avoid;
EXPORT ID	flg_us;
EXPORT ID	mbf_scan_req;
EXPORT ID	mbf_scan_samples;
EXPORT ID	mbf_profile;
EXPORT ID	tid_scan;
EXPORT ID	tid_profile;
EXPORT ID	tid_avoid;

LOCAL ID make_mbf(INT n, INT msz)
{
	T_CMBF	cmbf = {0};

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = APP_MBF_BUFSZ(n, msz);
	cmbf.maxmsz = msz;
	return tk_cre_mbf(&cmbf);
}

EXPORT ER subsystem5_init(void)
{
	T_CFLG	cflg = {0};

	s5_prof_seq.seq = 0;
	s5_scan_word = S5_DIST_INVALID;
	s5_avoid_word = 0;

	cflg.flgatr = TA_TFIFO | TA_WSGL;
	flg_avoid = tk_cre_flg(&cflg);
	if (flg_avoid < E_OK) {
		return flg_avoid;
	}
	flg_us = tk_cre_flg(&cflg);
	if (flg_us < E_OK) {
		return flg_us;
	}
	mbf_scan_req = make_mbf(S5_REQ_QUEUE_LEN, (INT)sizeof(UW));
	if (mbf_scan_req < E_OK) {
		return mbf_scan_req;
	}
	mbf_scan_samples = make_mbf(S5_SAMPLE_QUEUE_LEN, (INT)sizeof(UW));
	if (mbf_scan_samples < E_OK) {
		return mbf_scan_samples;
	}
	mbf_profile = make_mbf(S5_PROFILE_QUEUE_LEN, (INT)(2 * sizeof(UW)));
	if (mbf_profile < E_OK) {
		return mbf_profile;
	}
	/* echo pin callback must be registered before the GPIO IRQ starts */
	return us_init(flg_us, USEV_ECHO);
}

EXPORT ER subsystem5_start(void)
{
	tid_scan = app_start_task(scan_task, S5_PRI_SCAN, APP_CORE_CONTROL,
				  "scan_task");
	if (tid_scan < E_OK) {
		return tid_scan;
	}
	tid_avoid = app_start_task(avoid_task, S5_PRI_AVOID, APP_CORE_ANY,
				   "avoid_task");
	if (tid_avoid < E_OK) {
		return tid_avoid;
	}
	tid_profile = app_start_task(profile_task, S5_PRI_PROFILE, APP_CORE_ANY,
				     "profile_task");
	return (tid_profile < E_OK) ? tid_profile : E_OK;
}
