/*
 *	subsystem3.c
 *	Subsystem 3 -- line following and barcode decoding: kernel objects
 *	and tasks.
 *
 *	Kernel objects (TEAM.md section 6 budget):
 *	  flg_line_evt    event flag  IF-L2 line events (several waiters)
 *	  cyc_line        cyclic      20 ms, wakes the Line Following task
 *	  line_task       task        priority 14, core 1
 *	  barcode_task    task        priority 17, core 1
 *	Pins: GP2 (IR left), GP6 (IR centre), GP27 (IR right / barcode edges)
 */

#include "subsystem3.h"
#include "drv_ir.h"

EXPORT ID	flg_line_evt;
EXPORT ID	cyc_line;
EXPORT ID	tid_line;
EXPORT ID	tid_barcode;

EXPORT ER subsystem3_init(void)
{
	T_CFLG	cflg = {0};
	T_CCYC	ccyc = {0};

	s3_line_seq.seq = 0;

	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_line_evt = tk_cre_flg(&cflg);
	if (flg_line_evt < E_OK) {
		return flg_line_evt;
	}

	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)line_cyclic_handler;
	ccyc.cyctim = S3_PERIOD_MS;
	ccyc.cycphs = S3_PERIOD_MS + 10U;	/* offset from Motion Control */
#if TK_SUPPORT_SMP
	ccyc.cycatr |= TA_ASSPRC;
	ccyc.assprc  = APP_CORE_CONTROL;
#endif
	cyc_line = tk_cre_cyc(&ccyc);
	if (cyc_line < E_OK) {
		return cyc_line;
	}

	/* pins + IR_RIGHT edge callback, before any task starts */
	return ir_init();
}

EXPORT ER subsystem3_start(void)
{
	tid_line = app_start_task(line_task, S3_PRI_LINE, APP_CORE_CONTROL,
				  "line_task");
	if (tid_line < E_OK) {
		return tid_line;
	}
	tid_barcode = app_start_task(barcode_task, S3_PRI_BARCODE, APP_CORE_CONTROL,
				     "barcode_task");
	if (tid_barcode < E_OK) {
		return tid_barcode;
	}
	ir_set_wake_task(tid_barcode);
	return tk_sta_cyc(cyc_line);
}
