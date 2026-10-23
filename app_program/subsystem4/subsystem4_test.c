/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4.  Built only with: make TEST=1
 *
 *	Runs in a test task with its own stack (subsystem2/app_test.h):
 *	  S4-U01..U29  unit tests of the pure logic (s4_unit_tests.c)
 *	  S4-U30..U32  checks of the running subsystem on the Pico (below)
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem4/logs/ and record the result in
 *	testing/subsystem4/TEST_LOG.md.
 *
 *	The checks hold with or without an IMU fitted: without one the task
 *	must report FAULT with invalid codes, never a reading it did not get.
 */

#include "subsystem4.h"
#include "../test_report.h"

#if APP_TEST

#include "../subsystem2/app_test.h"
#include "s4_tests.h"
#include "drv_imu.h"

LOCAL INT runtime_checks(void)
{
	T_RCYC		rcyc;
	S4_IMU_STATUS	a, b;
	S4_HUMP_STATUS	h;
	UW		rate, faults, dseq, dt;
	BOOL		ok;

	TEST_INIT();
	ok = tk_ref_cyc(cyc_imu, &rcyc) == E_OK && (rcyc.cycstat & TCYC_STA) != 0U;
	rate = app_test_rate(&s4_stat_imu, 1000);
	tm_printf((UB*)"[TEST]   imu: %d runs/s, worst run %d us (includes I2C waits)\n",
		  (INT)rate, (INT)s4_stat_imu.exec_max_us);
	TEST_CHECK("S4-U30", ok && rate >= 97U && rate <= 103U,
		   "IMU Monitoring runs every 10 ms (100 per s)");

	s4_get_imu_status(&a);
	(void)tk_dly_tsk(100);
	s4_get_imu_status(&b);
	faults = sys_fault_get();
	dseq = (UW)(b.seq - a.seq) & 0xFFU;
	dt = b.time_ms - a.time_ms;		/* snapshot timestamp advances */
	switch (b.status) {
	case IMU_ST_OK:
		ok = b.valid == 1U && b.chip != IMU_CHIP_NONE && b.calibrated == 1U
		     && dseq >= 8U && dseq <= 12U && dt >= 80U && dt <= 120U
		     && (faults & SYS_FLT_IMU) == 0U;
		break;
	case IMU_ST_FAULT:
		ok = (faults & (SYS_FLT_IMU | SYS_FLT_IMU_CAL)) != 0U && b.valid == 0U
		     && b.pitch_deg == S4_ANGLE_INVALID && b.heading_deg == S4_HEADING_INVALID
		     && dt >= 80U && dt <= 120U;
		break;
	default:			/* initialising / calibrating */
		ok = b.valid == 0U;
		break;
	}
	tm_printf((UB*)"[TEST]   imu status %d, chip %d, valid %d, pitch %d deg\n",
		  (INT)b.status, (INT)b.chip, (INT)b.valid, (INT)b.pitch_deg);
	TEST_CHECK("S4-U31", ok,
		   "IMU snapshot honest and fresh: valid data only from a calibrated chip, else invalid codes + IMU fault");

	s4_get_hump_status(&h);
	TEST_CHECK("S4-U32", h.count == 0U && h.active == 0U && h.last_code == HUMP_CODE_NONE
		   && s4_max_hump_um() < 0 && app_load(&s4_dropped) == 0U,
		   "standing car: no hump reported; IMU -> hump queue never overflowed");
	TEST_SUMMARY("subsystem4 target");
	return test_fail_;
}

LOCAL INT all_tests(void)
{
	INT	fails = s4_run_unit_tests();

	app_test_drain();
	return fails + runtime_checks();
}

EXPORT void subsystem4_test(void)
{
	INT	fails = app_test_run(all_tests, "subsystem4");

	tm_printf((UB*)"[TEST] subsystem4 TOTAL %d failed check(s)\n", fails);
}

#endif /* APP_TEST */
