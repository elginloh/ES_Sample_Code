/*
 *	app_test.c
 *	On-target test runner support (see app_test.h).  Compiled into the
 *	test image only (make TEST=1); empty otherwise.
 */

#include <tk/typedef.h>

#if APP_TEST

#include "subsystem2.h"
#include "app_test.h"

#if TM_CONSOLE_USB_CDC
/* lib/libtm/sysdepend/pico_rp2040/usb/usb_console.c */
IMPORT UW tm_usb_pending(void);		/* bytes still queued for the host */
IMPORT UW tm_usb_state(void);		/* 3 = enumerated and DTR asserted */
#endif

typedef struct {
	INT		(*body)(void);
	volatile INT	fails;
	volatile UW	done;
	volatile UW	cancelled;	/* waiter gave up: never wake it */
	ID		waiter;
} TEST_JOB;

LOCAL TEST_JOB	job;
LOCAL BOOL	job_hung;		/* a hung body could not be stopped */

LOCAL void test_task(INT stacd, void *exinf)
{
	TEST_JOB	*j = (TEST_JOB *)exinf;

	(void)stacd;
	j->fails = j->body();
	app_store(&j->done, 1U);
	if (app_load(&j->cancelled) == 0U) {
		(void)tk_wup_tsk(j->waiter);
	}
	tk_exd_tsk();				/* exit and free the stack */
}

EXPORT INT app_test_run(INT (*body)(void), const char *name)
{
	T_CTSK	ctsk = {0};
	ID	tid;
	ER	er;
	UW	start;

	if (job_hung) {
		tm_printf((UB*)"[TEST] %s: not run, an earlier test task hung\n", name);
		return 1;
	}
	job.body = body;
	job.fails = 0;
	job.done = 0;
	job.cancelled = 0;
	job.waiter = tk_get_tid();

	ctsk.exinf = (void *)&job;
	ctsk.tskatr = TA_HLNG | TA_RNG3;	/* any core */
	ctsk.task = (FP)test_task;
	ctsk.itskpri = APP_TEST_PRI;
	ctsk.stksz = APP_TEST_STACK;
	tid = tk_cre_tsk(&ctsk);
	if (tid < E_OK) {
		tm_printf((UB*)"[TEST] %s: cannot create the test task (%d)\n", name, (INT)tid);
		return 1;
	}
	er = tk_sta_tsk(tid, 0);
	if (er < E_OK) {
		(void)tk_del_tsk(tid);
		tm_printf((UB*)"[TEST] %s: cannot start the test task (%d)\n", name, (INT)er);
		return 1;
	}

	start = app_now_ms();
	while (app_load(&job.done) == 0U) {
		UW	el = app_now_ms() - start;

		if (el >= APP_TEST_TIMEOUT_MS) {
			app_store(&job.cancelled, 1U);
			if (tk_ter_tsk(tid) == E_OK) {
				(void)tk_del_tsk(tid);
			} else {
				job_hung = TRUE;	/* still running: refuse later runs */
			}
			(void)tk_can_wup(TSK_SELF);
			tm_printf((UB*)"[TEST] %s: test task did not finish in %d ms\n",
				  name, APP_TEST_TIMEOUT_MS);
			return 1;
		}
		(void)tk_slp_tsk((TMO)(APP_TEST_TIMEOUT_MS - el));
	}
	/* The test task may have woken us while we were not asleep.  Drop that
	   wake-up: usermain() ends with tk_slp_tsk(TMO_FEVR), and a stale one
	   would make it return, which shuts the kernel down (app_main.c). */
	(void)tk_can_wup(TSK_SELF);
	(void)tk_dly_tsk(10);			/* let it finish tk_exd_tsk() */
	app_test_drain();
	return job.fails;
}

EXPORT void app_test_drain(void)
{
#if TM_CONSOLE_USB_CDC
	UINT	i;

	for (i = 0; i < 100U; i++) {		/* at most ~1 s */
		if (tm_usb_state() < 3U || tm_usb_pending() == 0U) {
			return;			/* no terminal, or all sent */
		}
		(void)tk_dly_tsk(10);
	}
#endif
	/* UART console: tm_printf() already waits for the transmitter */
}

EXPORT UW app_test_rate(const TASK_STAT *s, UW window_ms)
{
	UW	r0 = s->runs;

	(void)tk_dly_tsk((TMO)window_ms);
	return s->runs - r0;
}

#endif /* APP_TEST */
