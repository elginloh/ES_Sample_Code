/*
 *	subsystem2.c
 *	Subsystem 2 -- Motion Control: creates its kernel objects and starts
 *	the Motion Control task.
 *
 *	Kernel objects (TEAM.md section 6 budget):
 *	  flg_motion       event flag   wake-up of Motion Control (single waiter)
 *	  flg_motion_done  event flag   IF-M3 completion bits (several waiters)
 *	  mbf_motion_cmd   message buf  IF-M1, 16 x 4-byte command words
 *	  cyc_motion       cyclic       20 ms period + motor watchdog
 *	  motion_task      task         priority 12, core 1
 */

#include "subsystem2.h"
#include "s2_private.h"
#include "motion_cfg.h"
#include "drv_encoder.h"

EXPORT ID	flg_motion;
EXPORT ID	flg_motion_done;
EXPORT ID	mbf_motion_cmd;
EXPORT ID	cyc_motion;
EXPORT ID	tid_motion;

EXPORT ER subsystem2_init(void)
{
	T_CFLG	cflg = {0};
	T_CMBF	cmbf = {0};
	T_CCYC	ccyc = {0};

	sys_state_init();
	gpio_irq_init();
	motion_api_init();

	cflg.flgatr  = TA_TFIFO | TA_WSGL;
	cflg.iflgptn = 0;
	flg_motion = tk_cre_flg(&cflg);
	if (flg_motion < E_OK) {
		return flg_motion;
	}

	cflg.flgatr  = TA_TFIFO | TA_WMUL;	/* one waiter per source */
	flg_motion_done = tk_cre_flg(&cflg);
	if (flg_motion_done < E_OK) {
		return flg_motion_done;
	}

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = APP_MBF_BUFSZ(MOTION_CMD_QUEUE_LEN, sizeof(UW));
	cmbf.maxmsz = sizeof(UW);
	mbf_motion_cmd = tk_cre_mbf(&cmbf);
	if (mbf_motion_cmd < E_OK) {
		return mbf_motion_cmd;
	}

	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)motion_cyclic_handler;
	ccyc.cyctim = MOTION_PERIOD_MS;
	ccyc.cycphs = MOTION_PERIOD_MS;
#if TK_SUPPORT_SMP
	ccyc.cycatr |= TA_ASSPRC;		/* same core as the task it guards */
	ccyc.assprc  = APP_CORE_CONTROL;
#endif
	cyc_motion = tk_cre_cyc(&ccyc);
	if (cyc_motion < E_OK) {
		return cyc_motion;
	}

	/* GPIO interrupt callbacks must be registered before any start */
	return encoder_init();
}

EXPORT ER subsystem2_start(void)
{
	tid_motion = app_start_task(motion_task, S2_PRI_MOTION, APP_CORE_CONTROL,
				    "motion_task");
	if (tid_motion < E_OK) {
		return tid_motion;
	}
	return tk_sta_cyc(cyc_motion);
}
