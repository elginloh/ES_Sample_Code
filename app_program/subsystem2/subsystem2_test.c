/*
 *	subsystem2_test.c
 *	On-target tests for subsystem 2.  Built only with: make TEST=1
 *
 *	Runs in a test task with its own stack (app_test.h):
 *	  S2-U01..U39            unit tests of the pure logic (s2_unit_tests.c)
 *	  INT-10..14, INT-17..20 integration tests (s2_integration_tests.c)
 *	  S2-U40..U47            checks of the running subsystem on the Pico
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem2/logs/ (integration results to
 *	testing/integration/logs/) and record them in the TEST_LOG.md files.
 *
 *	Safe on the bench: the test image never arms the motors
 *	(motion_api.c), so the vehicle stays in INIT, or in SAFE_STOP when a
 *	critical sensor is missing.  Every expectation below holds in both.
 *	S2-U45..U47 act as the BARCODE and SUPER command producers; those
 *	tasks issue nothing while the vehicle is not navigating and no MQTT
 *	command arrives, so the one-producer-per-source rule still holds.
 */

#include "subsystem2.h"
#include "../test_report.h"

#if APP_TEST

#include "s2_private.h"
#include "s2_tests.h"
#include "app_test.h"
#include "motion_cfg.h"
#include "drv_encoder.h"

LOCAL INT runtime_checks(void)
{
	T_RCYC		rcyc;
	T_RMBF		rmbf;
	ENC_SNAP	es;
	UW		t0, dt, rate, vsw, cmd, res = 0, faults, clear, expect;
	BOOL		ok, ok2;

	TEST_INIT();
	ok = tk_ref_cyc(cyc_motion, &rcyc) == E_OK && (rcyc.cycstat & TCYC_STA) != 0U
	     && tk_ref_mbf(mbf_motion_cmd, &rmbf) == E_OK && gpio_irq_running()
	     && encoder_snapshot(ENC_LEFT, &es) && encoder_snapshot(ENC_RIGHT, &es);
	TEST_CHECK("S2-U40", ok,
		   "command queue, 20 ms cyclic handler and shared GPIO interrupt running; encoder records readable");

	t0 = hw_time_us();
	(void)tk_dly_tsk(200);
	dt = hw_elapsed_us(t0);
	tm_printf((UB*)"[TEST]   tk_dly_tsk(200) took %d us on the TIMER\n", (INT)dt);
	TEST_CHECK("S2-U41", dt >= 195000U && dt <= 230000U,
		   "microsecond TIMER runs at 1 MHz against the kernel clock");

	rate = app_test_rate(&s2_stat_motion, 1000);
	tm_printf((UB*)"[TEST]   motion: %d runs/s, worst run %d us, period %d..%d us\n",
		  (INT)rate, (INT)s2_stat_motion.exec_max_us,
		  (INT)s2_stat_motion.period_min_us, (INT)s2_stat_motion.period_max_us);
	TEST_CHECK("S2-U42", rate >= 48U && rate <= 52U && s2_stat_motion.overruns == 0U
		   && s2_stat_motion.exec_max_us < (UW)MOTION_PERIOD_MS * 1000U / 2U,
		   "Motion Control runs every 20 ms and finishes within half its period");

	vsw = sys_vsw();
	tm_printf((UB*)"[TEST]   mode %s, faults 0x%07x\n", sys_mode_name(vsw_mode(vsw)),
		  (INT)sys_fault_get());
	TEST_CHECK("S2-U43", (vsw & VSW_ARMED) == 0U
		   && (vsw_mode(vsw) == VM_INIT || vsw_mode(vsw) == VM_SAFE_STOP)
		   && (sys_fault_get() & SYS_FLT_MOTOR_WDOG) == 0U,
		   "test image disarmed: INIT or SAFE_STOP only; motor watchdog never tripped");

	t0 = hw_time_us();
	motion_stop(MSRC_SUPER);
	while (app_load(&s2_stop_req) != 0U && hw_elapsed_us(t0) < 100000U) {
		/* bounded: Motion Control (higher priority) serves it */
	}
	dt = hw_elapsed_us(t0);
	tm_printf((UB*)"[TEST]   stop() request served after %d us\n", (INT)dt);
	TEST_CHECK("S2-U44", app_load(&s2_stop_req) == 0U && dt < 100000U,
		   "stop() bypass served by Motion Control within 100 ms (NFR6, software path)");

	ok = motion_command(MCMD_MOVE, MSRC_BARCODE, 100, 0, &cmd) == E_OK
	     && motion_wait_result(cmd, 200, &res) == E_OK
	     && mres_status(res) == MRES_REJECTED && mres_reason(res) == MREJ_MODE;
	TEST_CHECK("S2-U45", ok,
		   "IF-M1/M3 on target: a BARCODE MOVE outside navigation is answered REJECTED (MODE)");

	ok = motion_command(MCMD_STOP, MSRC_SUPER, 0, 0, &cmd) == E_OK
	     && motion_wait_result(cmd, 200, &res) == E_OK && mres_status(res) == MRES_DONE;
	(void)tk_dly_tsk(60);
	ok = ok && sys_mode() == VM_SAFE_STOP && (sys_fault_get() & SYS_FLT_REMOTE_STOP) != 0U;
	TEST_CHECK("S2-U46", ok, "remote STOP: DONE, SAFE_STOP and REMOTE_STOP latched within 60 ms");

	faults = sys_fault_get();
	expect = sys_recovery_decide(MCMD_RESUME, VM_SAFE_STOP, faults, &clear);
	ok2 = motion_command(MCMD_RESUME, MSRC_SUPER, 0, 0, &cmd) == E_OK
	      && motion_wait_result(cmd, 200, &res) == E_OK
	      && mres_status(res) == ((expect == MREJ_NONE) ? MRES_DONE : MRES_REJECTED);
	(void)tk_dly_tsk(60);
	tm_printf((UB*)"[TEST]   RESUME with faults 0x%07x -> %s, mode %s\n", (INT)faults,
		  (expect == MREJ_NONE) ? "accepted" : "refused (critical condition)",
		  sys_mode_name(sys_mode()));
	ok2 = ok2 && (expect != MREJ_NONE || sys_mode() == VM_SAFE_STOP)
	      && (expect == MREJ_NONE ? sys_mode() == VM_INIT : TRUE)
	      && (sys_fault_get() & SYS_FLT_REMOTE_STOP) == 0U;
	TEST_CHECK("S2-U47", ok2,
		   "RESUME clears latched events; leaves SAFE_STOP only when no critical condition remains");
	TEST_SUMMARY("subsystem2 target");
	return test_fail_;
}

LOCAL INT all_tests(void)
{
	INT	fails = s2_run_unit_tests();

	app_test_drain();
	fails += s2_run_integration_tests();
	app_test_drain();
	return fails + runtime_checks();
}

EXPORT void subsystem2_test(void)
{
	INT	fails = app_test_run(all_tests, "subsystem2");

	tm_printf((UB*)"[TEST] subsystem2 TOTAL %d failed check(s)\n", fails);
}

#endif /* APP_TEST */
