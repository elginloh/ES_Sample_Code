/*
 *	subsystem4.c
 *	Subsystem 4 -- IMU and terrain monitoring: kernel objects and tasks.
 *
 *	Kernel objects (TEAM.md section 6 budget):
 *	  flg_imu        event flag   10 ms tick + I2C completion (one waiter)
 *	  mbf_imu_hump   message buf  32 x 4-byte packed samples (IF internal)
 *	  cyc_imu        cyclic       10 ms
 *	  imu_task       task         priority 13, core 1
 *	  hump_task      task         priority 18
 *	Peripherals: I2C0 on GP16 (SDA) / GP17 (SCL), IRQ 23.
 */

#include "subsystem4.h"

EXPORT ID	flg_imu;
EXPORT ID	mbf_imu_hump;
EXPORT ID	cyc_imu;
EXPORT ID	tid_imu;
EXPORT ID	tid_hump;

EXPORT ER subsystem4_init(void)
{
	T_CFLG	cflg = {0};
	T_CMBF	cmbf = {0};
	T_CCYC	ccyc = {0};

	s4_imu_seq.seq = 0;
	s4_imu_words[0] = 0;
	s4_imu_words[1] = S4_HEADING_INVALID;
	s4_imu_time = 0;
	s4_dropped = 0;

	cflg.flgatr  = TA_TFIFO | TA_WSGL;
	cflg.iflgptn = 0;
	flg_imu = tk_cre_flg(&cflg);
	if (flg_imu < E_OK) {
		return flg_imu;
	}

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = APP_MBF_BUFSZ(S4_HUMP_QUEUE_LEN, sizeof(UW));
	cmbf.maxmsz = sizeof(UW);
	mbf_imu_hump = tk_cre_mbf(&cmbf);
	if (mbf_imu_hump < E_OK) {
		return mbf_imu_hump;
	}

	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)imu_cyclic_handler;
	ccyc.cyctim = S4_PERIOD_MS;
	ccyc.cycphs = S4_PERIOD_MS;
#if TK_SUPPORT_SMP
	ccyc.cycatr |= TA_ASSPRC;
	ccyc.assprc  = APP_CORE_CONTROL;
#endif
	cyc_imu = tk_cre_cyc(&ccyc);
	return (cyc_imu < E_OK) ? cyc_imu : E_OK;
}

EXPORT ER subsystem4_start(void)
{
	tid_imu = app_start_task(imu_task, S4_PRI_IMU, APP_CORE_CONTROL, "imu_task");
	if (tid_imu < E_OK) {
		return tid_imu;
	}
	tid_hump = app_start_task(hump_task, S4_PRI_HUMP, APP_CORE_ANY, "hump_task");
	if (tid_hump < E_OK) {
		return tid_hump;
	}
	return tk_sta_cyc(cyc_imu);
}
