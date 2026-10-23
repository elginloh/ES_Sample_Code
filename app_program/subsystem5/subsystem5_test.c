/*
 *	subsystem5_test.c
 *	On-target tests for subsystem 5.  Built only with: make TEST=1
 *
 *	Runs in a test task with its own stack (subsystem2/app_test.h):
 *	  S5-U01..U20  unit tests of the pure logic (s5_unit_tests.c)
 *	  S5-U21..U23  checks of the running subsystem on the Pico (below)
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem5/logs/ and record the result in
 *	testing/subsystem5/TEST_LOG.md.
 *
 *	The checks hold with or without the HC-SR04 fitted: a silent sensor
 *	must be reported as such, never as a distance.
 */

#include "subsystem5.h"
#include "../test_report.h"

#if APP_TEST

#include "../subsystem2/app_test.h"
#include "s5_tests.h"
#include "drv_ultrasonic.h"

LOCAL INT runtime_checks(void)
{
	S5_SCAN_STATUS	s;
	S5_AVOID_STATUS	av;
	UW		rate, faults;
	BOOL		ok;

	TEST_INIT();
	rate = app_test_rate(&s5_stat_scan, 1200);
	tm_printf((UB*)"[TEST]   scan: %d rangings in 1.2 s, worst %d us\n", (INT)rate,
		  (INT)s5_stat_scan.exec_max_us);
	TEST_CHECK("S5-U21", rate >= 15U && rate <= 21U,
		   "Obstacle Scan ranges every 60 ms (+ one 10 ms tick) while monitoring");

	s5_get_scan_status(&s);
	faults = sys_fault_get();
	switch (s.us_result) {
	case US_OK:
		ok = s.dist_code <= S5_DIST_MAX_CODE;
		break;
	case US_NO_ECHO:
		ok = s.dist_code == S5_DIST_NO_ECHO;
		break;
	case US_NO_RESPONSE:
		ok = s.dist_code == S5_DIST_INVALID && (faults & SYS_FLT_US_NO_RESPONSE) != 0U;
		break;
	default:
		ok = s.dist_code == S5_DIST_INVALID;
		break;
	}
	tm_printf((UB*)"[TEST]   ultrasonic result %d, distance code %d (5 mm steps)\n",
		  (INT)s.us_result, (INT)s.dist_code);
	TEST_CHECK("S5-U22", ok,
		   "ranging result and distance code agree; a silent sensor raises US_NO_RESPONSE");

	s5_get_avoid_status(&av);
	TEST_CHECK("S5-U23", av.state == AVST_IDLE && s.servo_deg == 0 && s.mode == SREQ_MONITOR,
		   "outside navigation: avoidance idle, servo centred, forward monitoring");
	TEST_SUMMARY("subsystem5 target");
	return test_fail_;
}

LOCAL INT all_tests(void)
{
	INT	fails = s5_run_unit_tests();

	app_test_drain();
	return fails + runtime_checks();
}

EXPORT void subsystem5_test(void)
{
	INT	fails = app_test_run(all_tests, "subsystem5");

	tm_printf((UB*)"[TEST] subsystem5 TOTAL %d failed check(s)\n", fails);
}

#endif /* APP_TEST */
