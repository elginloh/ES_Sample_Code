/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *
 *	Runs in a test task with its own stack (subsystem2/app_test.h):
 *	  S3-U01..U31  unit tests incl. every supplied barcode sample
 *	               (s3_unit_tests.c)
 *	  S3-U32..U35  checks of the running subsystem on the Pico (below)
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem3/logs/ and record the result in
 *	testing/subsystem3/TEST_LOG.md.  Reading real barcodes and following
 *	a real line are manual tests on the track (VERIFICATION_REPORT.md).
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST

#include "../subsystem2/app_test.h"
#include "s3_tests.h"
#include "drv_ir.h"

/* Only offered to gpio_irq_register() to prove the table is frozen */
LOCAL void refused_callback(UINT pin, UINT events, UW now_us)
{
	(void)pin;
	(void)events;
	(void)now_us;
}

LOCAL INT runtime_checks(void)
{
	T_RCYC		rcyc;
	T_RFLG		rflg;
	S3_LINE_STATUS	a, b;
	S3_BC_STATUS	bs;
	UW		rate, d, dt, mode;
	UINT		k;
	BOOL		ok, decided = TRUE;

	TEST_INIT();
	ok = tk_ref_cyc(cyc_line, &rcyc) == E_OK && (rcyc.cycstat & TCYC_STA) != 0U
	     && tk_ref_flg(flg_line_evt, &rflg) == E_OK;
	rate = app_test_rate(&s3_stat_line, 1000);
	tm_printf((UB*)"[TEST]   line: %d runs/s, worst run %d us\n", (INT)rate,
		  (INT)s3_stat_line.exec_max_us);
	TEST_CHECK("S3-U32", ok && rate >= 48U && rate <= 52U && s3_stat_line.overruns == 0U,
		   "Line Following runs every 20 ms (50 per s) within half its period");

	s3_get_line_status(&a);
	(void)tk_dly_tsk(200);
	s3_get_line_status(&b);
	d = (UW)(b.seq - a.seq) & 0xFU;
	dt = b.time_ms - a.time_ms;
	TEST_CHECK("S3-U33", d >= 9U && d <= 11U && dt >= 180U && dt <= 220U,
		   "line status word republished every period with a fresh timestamp (IF-L1)");

	mode = sys_mode();
	if (mode == VM_INIT) {
		for (k = 0; k < 50U && (sys_vsw() & VSW_IR_OK) == 0U
		     && (sys_fault_get() & SYS_FLT_IR_CAL) == 0U; k++) {
			(void)tk_dly_tsk(20);
		}
		decided = (sys_vsw() & VSW_IR_OK) != 0U || (sys_fault_get() & SYS_FLT_IR_CAL) != 0U;
	}
	tm_printf((UB*)"[TEST]   mode %s, raw IR L/C/R = %d%d%d, IR calibrated %d\n",
		  sys_mode_name(mode), (INT)(ir_read_raw() & 1U), (INT)((ir_read_raw() >> 1) & 1U),
		  (INT)((ir_read_raw() >> 2) & 1U), (INT)((sys_vsw() & VSW_IR_OK) != 0U));
	TEST_CHECK("S3-U34", decided,
		   "IR calibration decided within 1 s of INIT (calibrated, or IR_CAL fault): never stalls");

	s3_get_barcode_status(&bs);
	TEST_CHECK("S3-U35", bs.state == BCS_IDLE && bs.executed == 0U
		   && (sys_fault_get() & SYS_FLT_BARCODE_OVERFLOW) == 0U
		   && gpio_irq_register(IR_RIGHT_PIN, GPIO_IRQ_EDGES, refused_callback) == E_OBJ,
		   "barcode reader idle outside NORMAL, no edge overflow; GPIO callbacks frozen once running");
	TEST_SUMMARY("subsystem3 target");
	return test_fail_;
}

LOCAL INT all_tests(void)
{
	INT	fails = s3_run_unit_tests();

	app_test_drain();
	return fails + runtime_checks();
}

EXPORT void subsystem3_test(void)
{
	INT	fails = app_test_run(all_tests, "subsystem3");

	tm_printf((UB*)"[TEST] subsystem3 TOTAL %d failed check(s)\n", fails);
}

#endif /* APP_TEST */
