/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make TEST=1
 *
 *	Runs in a test task with its own stack (subsystem2/app_test.h):
 *	  S1-U01..U20  unit tests of the pure logic       (s1_unit_tests.c)
 *	  INT-15, 16   telemetry integration tests         (s1_integration_tests.c)
 *	  S1-U21..U26  checks of the running subsystem on the Pico (below)
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem1/logs/ and record the result in
 *	testing/subsystem1/TEST_LOG.md.
 *
 *	S1-U24/U25 play the MQTT client's part: they hand command frames to
 *	s1_mqtt_put_rx() and collect the answers with s1_mqtt_take_tx(),
 *	exactly the calls mqtt_utk.c makes once the integration patch
 *	(docs/MQTT_INTEGRATION_PATCH.md) is applied, so the command path is
 *	tested on the Pico without a broker.
 */

#include "subsystem1.h"
#include "../test_report.h"

#if APP_TEST

#include "../subsystem2/app_test.h"
#include "s1_tests.h"

#define TEST_CMD_ID	0xA5U

/* Collect Buddy 1's acknowledgement for command id 'id' (up to 1 s);
   other frames (heartbeats when online) are skipped */
LOCAL BOOL ack_for(UB id, ACK_RECORD *ack)
{
	UB		f[S1_MQTT_FRAME_MAX];
	const char	*topic;
	UINT		tries;

	for (tries = 0; tries < 50U; tries++) {
		int32_t	n = s1_mqtt_take_tx(f, (int32_t)sizeof(f), &topic);

		if (n > 0) {
			if (mp_ack_decode(f, (UINT)n, ack) >= 0 && ack->cmd_id == id) {
				return TRUE;
			}
			continue;
		}
		(void)tk_dly_tsk(20);
	}
	return FALSE;
}

LOCAL INT runtime_checks(void)
{
	T_RCYC		rcyc;
	T_RMBF		rmbf;
	T_RFLG		rflg;
	T_CMBF		cmbf = {0};
	S1_COMM_STATUS	cs;
	CMD_RECORD	c;
	ACK_RECORD	ack;
	UB		f[MP_MAX_FRAME];
	UB		msg[4] = {0, 0, 0, 0};
	UB		out[4] = {0, 0, 0, 0};
	volatile UW	dropped = 0;
	UW		rate, rej0;
	INT		n, k, got1, got2, got3;
	ER		er = E_OK;
	ID		mbf;
	BOOL		ok, ok2;

	TEST_INIT();
	ok = tk_ref_cyc(cyc_comm, &rcyc) == E_OK && (rcyc.cycstat & TCYC_STA) != 0U;
	ok = ok && tk_ref_cyc(cyc_tlm, &rcyc) == E_OK && (rcyc.cycstat & TCYC_STA) != 0U;
	ok = ok && tk_ref_mbf(mbf_telemetry, &rmbf) == E_OK && tk_ref_mbf(mbf_mqtt_tx, &rmbf) == E_OK
	     && tk_ref_mbf(mbf_mqtt_rx, &rmbf) == E_OK && tk_ref_flg(flg_comm, &rflg) == E_OK;
	TEST_CHECK("S1-U21", ok, "message buffers, event flag and both cyclic handlers exist and run");

	rate = app_test_rate(&s1_stat_tlm, 2000);
	tm_printf((UB*)"[TEST]   telemetry: %d runs in 2 s, worst run %d us\n",
		  (INT)rate, (INT)s1_stat_tlm.exec_max_us);
	TEST_CHECK("S1-U22", rate >= 3U && rate <= 5U && s1_stat_tlm.overruns == 0U,
		   "telemetry task: one frame per 500 ms, never over its 50 ms budget");

	s1_get_comm_status(&cs);
	tm_printf((UB*)"[TEST]   comm state %s, link %d, session %d\n",
		  cfsm_name(cs.state), (INT)cs.link, (INT)cs.session);
#if TM_WIFI_CYW43 && TM_NET_LWIP && TM_MQTT
	ok = cs.state != CS_INIT && cs.state != CS_DISABLED;
#else
	ok = cs.state == CS_DISABLED
	     && (sys_fault_get() & (SYS_FLT_WIFI_LOSS | SYS_FLT_MQTT_LOSS)) == 0U;
#endif
	TEST_CHECK("S1-U23", ok,
		   "connection state machine running (DISABLED without NET=lwip MQTT=1, no false fault)");

	c.cmd = CMD_PING;
	c.arg = 0;
	c.cmd_id = TEST_CMD_ID;
	n = mp_cmd_encode(&c, 1, f, sizeof(f));
	ok = s1_mqtt_put_rx("csc14/other", f, n) < 0 && s1_mqtt_put_rx(S1_TOPIC_CMD, f, 0) < 0
	     && s1_mqtt_put_rx(S1_TOPIC_CMD, f, n) == 0
	     && ack_for(TEST_CMD_ID, &ack) && ack.result == ACK_OK && ack.cmd == CMD_PING;
	TEST_CHECK("S1-U24", ok,
		   "MQTT hand-over: wrong topic and empty frame refused; PING answered with ACK OK");

	c.cmd = CMD_SET_SPEED;
	c.arg = 200;				/* the default cruise speed: no side effect */
	c.cmd_id = TEST_CMD_ID + 1U;
	n = mp_cmd_encode(&c, 2, f, sizeof(f));
	ok = s1_mqtt_put_rx(S1_TOPIC_CMD, f, n) == 0 && ack_for(c.cmd_id, &ack)
	     && ack.result == ACK_OK && ack.cmd == CMD_SET_SPEED;
	rej0 = app_load(&s1_rx_rejected);
	f[4] ^= 0x01U;				/* corrupted in transit */
	(void)s1_mqtt_put_rx(S1_TOPIC_CMD, f, n);
	ok2 = !ack_for(c.cmd_id, &ack) && app_load(&s1_rx_rejected) == rej0 + 1U
	      && (sys_fault_get() & SYS_FLT_CMD_INVALID) != 0U;
	sys_fault_clear(SYS_FLT_CMD_INVALID);	/* degraded, induced here: clean up */
	TEST_CHECK("S1-U25", ok && ok2,
		   "SET_SPEED reaches Buddy 2 and is acknowledged; a corrupted frame is counted, flagged, never run");

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz = APP_MBF_BUFSZ(2, 4);
	cmbf.maxmsz = 4;
	mbf = tk_cre_mbf(&cmbf);
	got1 = got2 = got3 = -1;
	if (mbf > 0) {
		for (k = 1; k <= 3; k++) {
			msg[0] = (UB)k;
			if (s1_queue_put_drop_oldest(mbf, msg, 4, &dropped) != E_OK) {
				er = E_QOVR;
			}
		}
		got1 = (tk_rcv_mbf(mbf, out, TMO_POL) == 4) ? out[0] : -1;
		got2 = (tk_rcv_mbf(mbf, out, TMO_POL) == 4) ? out[0] : -1;
		got3 = tk_rcv_mbf(mbf, out, TMO_POL);
		(void)tk_del_mbf(mbf);
	}
	TEST_CHECK("S1-U26", mbf > 0 && er == E_OK && dropped == 1U && got1 == 2 && got2 == 3
		   && got3 == E_TMOUT,
		   "bounded queue full: the OLDEST frame is dropped and counted (overflow policy)");
	TEST_SUMMARY("subsystem1 target");
	return test_fail_;
}

LOCAL INT all_tests(void)
{
	INT	fails = s1_run_unit_tests();

	app_test_drain();
	fails += s1_run_integration_tests();
	app_test_drain();
	return fails + runtime_checks();
}

EXPORT void subsystem1_test(void)
{
	INT	fails = app_test_run(all_tests, "subsystem1");

	tm_printf((UB*)"[TEST] subsystem1 TOTAL %d failed check(s)\n", fails);
}

#endif /* APP_TEST */
