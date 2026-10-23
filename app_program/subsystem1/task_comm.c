/*
 *	task_comm.c
 *	WiFi & MQTT Communication task (Week 6: Medium; statechart 6.1).
 *
 *	Every 100 ms: observe the WiFi link (cyw43_utk_get_status(), the one
 *	radio call TEAM.md allows from any task) and the broker session
 *	(reported through the bridge), run the connection state machine,
 *	raise/clear the DEGRADED network fault bits, send the heartbeat and
 *	move telemetry frames to the MQTT client queue while online.
 *	On inbound frames: validate (version, type, length, CRC, ranges,
 *	duplicates, rate), forward accepted supervisory requests to Buddy 2,
 *	acknowledge.  Nothing here can block local control (NFR9-NFR11).
 */

#include "subsystem1.h"
#if TM_WIFI_CYW43
#include "cyw43_utk.h"
#endif

EXPORT volatile UW	s1_comm_word;
EXPORT volatile UW	s1_rx_rejected;
EXPORT volatile UW	s1_status_now;

LOCAL COMM_FSM		fsm;
LOCAL volatile UW	tx_dropped;		/* writer: comm task */
LOCAL UB		hb_seq;
LOCAL UB		ack_seq;
LOCAL UW		hb_ms;
LOCAL UW		last_tick_ms;
LOCAL UB		dup_ids[S1_DUP_WINDOW];
LOCAL UINT		dup_n;
LOCAL UINT		dup_next;
LOCAL UW		rate_start_ms;
LOCAL UINT		rate_count;

EXPORT void comm_cyclic_handler(void *exinf)
{
	(void)exinf;
	(void)tk_set_flg(flg_comm, COMM_EV_TICK);
}

LOCAL UB sat8(UW v)
{
	return (UB)((v > 255U) ? 255U : v);
}

EXPORT void s1_get_comm_status(S1_COMM_STATUS *s)
{
	s1_comm_unpack(app_load(&s1_comm_word), s);
	s->tlm_dropped = sat8(app_load(&s1_tlm_dropped) + app_load(&tx_dropped));
	s->rx_rejected = sat8(app_load(&s1_rx_rejected));
}

/* Link as seen by the radio service.  Without the network stack and the
   MQTT client in the image (make NET=lwip MQTT=1) there is nothing to
   connect: the state machine reports DISABLED, not a fault. */
LOCAL UB read_link(BOOL *config_ok)
{
#if TM_WIFI_CYW43 && TM_NET_LWIP && TM_MQTT
	T_CYW43_UTK_STATUS	st;

	*config_ok = TRUE;
	cyw43_utk_get_status(&st);
	if (st.init_complete == 0U) {
		return CL_JOINING;
	}
	if (st.ready == 0U || st.link_status < 0) {
		return CL_FAILED;		/* radio dead / join failed */
	}
	if (st.link_status == 3) {		/* CYW43_LINK_UP */
		return CL_UP;
	}
	return (st.link_status > 0) ? CL_JOINING : CL_DOWN;
#else
	*config_ok = FALSE;
	return CL_DOWN;
#endif
}

LOCAL void send_frame(UB kind, const UB *f, INT n)
{
	UB	msg[S1_MSG_MAX];
	INT	i;

	if (n <= 0 || n > S1_MQTT_FRAME_MAX) {
		return;
	}
	msg[0] = kind;
	for (i = 0; i < n; i++) {
		msg[i + 1] = f[i];
	}
	(void)s1_queue_put_drop_oldest(mbf_mqtt_tx, msg, n + 1, &tx_dropped);
}

LOCAL void send_ack(UB cmd_id, UB cmd, UB result, UB detail)
{
	ACK_RECORD	a;
	UB		f[MP_MAX_FRAME];

	a.cmd_id = cmd_id;
	a.cmd = cmd;
	a.result = result;
	a.detail = detail;
	send_frame(MT_ACK, f, mp_ack_encode(&a, ack_seq++, f, sizeof(f)));
}

LOCAL BOOL duplicate(UB id)
{
	UINT	i;

	for (i = 0; i < dup_n; i++) {
		if (dup_ids[i] == id) {
			return TRUE;
		}
	}
	dup_ids[dup_next] = id;
	dup_next = (dup_next + 1U) % S1_DUP_WINDOW;
	if (dup_n < S1_DUP_WINDOW) {
		dup_n++;
	}
	return FALSE;
}

/* Accepted command -> Buddy 2 (supervisory source) -> acknowledgement.
   Buddy 2 decides whether the request is safe in the current mode; a
   remote command can never override a safety action. */
LOCAL void execute(const CMD_RECORD *c)
{
	UW	type, cmd, res = 0;
	INT	arg;
	UB	result = ACK_OK, detail = 0;

	if (c->cmd == CMD_REQ_STATUS) {
		app_store(&s1_status_now, 1U);
		(void)tk_wup_tsk(tid_tlm);
	}
	if (mp_cmd_to_motion(c, &type, &arg)) {
		if (motion_command(type, MSRC_SUPER, arg, 0, &cmd) != E_OK
		    || motion_wait_result(cmd, S1_CMD_WAIT_MS, &res) != E_OK) {
			result = ACK_BUSY;		/* queue full / no answer */
		} else {
			result = mp_ack_of_result(res, &detail);
		}
	}
	send_ack(c->cmd_id, c->cmd, result, detail);
	tm_printf((UB*)"comm: command %d id %d -> ack %d\n", (INT)c->cmd,
		  (INT)c->cmd_id, (INT)result);
}

LOCAL void handle_rx(void)
{
	UB		buf[S1_MQTT_FRAME_MAX];
	CMD_RECORD	c;
	INT		n, r;

	while ((n = tk_rcv_mbf(mbf_mqtt_rx, buf, TMO_POL)) > 0) {
		r = mp_cmd_decode(buf, (UINT)n, &c);
		if (r < 0) {
			/* NFR11: reject without crashing or blocking; acknowledge
			   only when the command id itself is trustworthy */
			s1_rx_rejected = s1_rx_rejected + 1U;
			sys_fault_set(SYS_FLT_CMD_INVALID);
			if (r == MPE_FIELD || r == MPE_ARG) {
				send_ack(c.cmd_id, c.cmd, mp_err_to_ack(r), 0);
			}
			continue;
		}
		if (c.cmd != CMD_STOP) {		/* a STOP is never throttled */
			UW	now = app_now_ms();

			if ((UW)(now - rate_start_ms) >= 1000U) {
				rate_start_ms = now;
				rate_count = 0;
			}
			if (++rate_count > S1_CMD_RATE_MAX) {
				s1_rx_rejected = s1_rx_rejected + 1U;
				send_ack(c.cmd_id, c.cmd, ACK_RATE, 0);
				continue;
			}
		}
		if (duplicate(c.cmd_id)) {
			send_ack(c.cmd_id, c.cmd, ACK_DUPLICATE, 0);
			continue;
		}
		execute(&c);
	}
}

LOCAL void send_heartbeat(void)
{
	HB_RECORD	h;
	UB		f[MP_MAX_FRAME];
	UW		faults = sys_fault_get();
	UINT		i;

	h.uptime_ds = (UH)(app_now_ms() / 100U);
	h.mode = (UB)sys_mode();
	h.comm_state = fsm.state;
	h.critical = ((faults & sys_fault_crit_mask()) != 0U) ? 1U : 0U;
	h.fault_count = 0;
	for (i = 0; i < SYS_FLT_COUNT; i++) {
		h.fault_count = (UB)(h.fault_count + ((faults >> i) & 1U));
	}
	h.reconnects = fsm.reconnects;
	send_frame(MT_HEARTBEAT, f, mp_hb_encode(&h, hb_seq++, f, sizeof(f)));
}

LOCAL void tick(void)
{
	UW		now = app_now_ms();
	UW		dt = now - last_tick_ms;
	UB		old = fsm.state;
	BOOL		cfg;
	UB		link = read_link(&cfg);
	UB		session = (UB)app_load(&s1_session);
	S1_COMM_STATUS	cs;
	UB		msg[S1_MSG_MAX];
	INT		n;

	last_tick_ms = now;
	cfsm_step(&fsm, cfg, link, session, dt);

	if (fsm.wifi_loss) {
		sys_fault_set(SYS_FLT_WIFI_LOSS);	/* degraded only (NFR9) */
	} else {
		sys_fault_clear(SYS_FLT_WIFI_LOSS);
	}
	if (fsm.mqtt_loss) {
		sys_fault_set(SYS_FLT_MQTT_LOSS);
	} else {
		sys_fault_clear(SYS_FLT_MQTT_LOSS);
	}
	if (fsm.reconnect_req) {
		fsm.reconnect_req = FALSE;
		app_store(&s1_reconnect_req, 1U);	/* taken by the MQTT client */
	}
	if (fsm.state != old) {
		tm_printf((UB*)"comm: %s -> %s\n", cfsm_name(old), cfsm_name(fsm.state));
	}

	if (fsm.state == CS_ONLINE) {
		hb_ms += dt;
		if (hb_ms >= S1_HB_PERIOD_MS) {
			hb_ms = 0;
			send_heartbeat();
		}
		/* telemetry waits in mbf_telemetry (drop-oldest) while offline */
		while ((n = tk_rcv_mbf(mbf_telemetry, msg, TMO_POL)) > 0) {
			(void)s1_queue_put_drop_oldest(mbf_mqtt_tx, msg, n, &tx_dropped);
		}
	}

	cs.state = fsm.state;
	cs.avail = cfsm_availability(&fsm);
	cs.link = link;
	cs.session = session;
	cs.attempts = fsm.attempts;
	cs.reconnects = fsm.reconnects;
	app_store(&s1_comm_word, s1_comm_pack(&cs));
}

EXPORT void comm_task(INT stacd, void *exinf)
{
	UINT	ptn;

	(void)stacd;
	(void)exinf;
	cfsm_init(&fsm);
	last_tick_ms = app_now_ms();
	for (;;) {
		if (tk_wai_flg(flg_comm, COMM_EV_TICK | COMM_EV_RX, TWF_ORW | TWF_BITCLR,
			       &ptn, TMO_FEVR) < E_OK) {
			(void)tk_dly_tsk(S1_COMM_PERIOD_MS);
			continue;
		}
		if ((ptn & COMM_EV_RX) != 0U) {
			handle_rx();
		}
		if ((ptn & COMM_EV_TICK) != 0U) {
			tick();
		}
	}
}
