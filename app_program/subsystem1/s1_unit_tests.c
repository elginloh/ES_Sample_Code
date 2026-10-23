/*
 *	s1_unit_tests.c
 *	Unit tests of subsystem 1's pure logic: CRC and bit packing, the
 *	binary MQTT protocol v1 against KNOWN frames, round trips at the field
 *	limits, every rejection path, the command mapping, the WiFi / MQTT
 *	connection state machine (disconnection, capped back-off, recovery,
 *	reconnection) and the communication status word.
 *
 *	The golden frames were generated from the protocol specification by
 *	an independent implementation (Python with its own LSB-first packer
 *	and binascii.crc_hqx for the CRC), not from this C code.  The
 *	generator is listed in docs/MQTT_BINARY_PROTOCOL.md.
 *	Runs on the host (subsystem2/host_test/) and in the on-target test
 *	image.  Test IDs: S1-Uxx.
 */

#include <tk/typedef.h>

#if APP_TEST || HOST_TEST

#include "../test_report.h"
#include "bitpack.h"
#include "mqtt_proto.h"
#include "comm_fsm.h"
#include "s1_tests.h"
#include "../subsystem2/motion_msg.h"

LOCAL const UB gold_hb[9] = {		/* uptime 0x1234, mode 4, comm 5, crit 1,
					   27 faults, 3 reconnects, seq 0x2A */
	0x12, 0x2A, 0x04, 0x34, 0x12, 0xEC, 0x3D, 0x60, 0xE5
};
LOCAL const UB gold_cmd[8] = {		/* SET_SPEED 250 mm/s, id 0x5A, seq 7 */
	0x14, 0x07, 0x03, 0xD4, 0x47, 0x0B, 0x59, 0x72
};
LOCAL const UB gold_ack[8] = {		/* id 0x5A, SET_SPEED, REJ_STATE, MREJ_MODE, seq 3 */
	0x15, 0x03, 0x03, 0x5A, 0x34, 0x02, 0x87, 0x52
};
LOCAL const UB gold_evt[10] = {		/* EVT_HUMP last 20, max 25, count 3, seq 9 */
	0x13, 0x09, 0x05, 0x44, 0x65, 0x03, 0x00, 0x00, 0x9B, 0x88
};
LOCAL const UB gold_tlm[40] = {		/* gold_record(), seq 0x80 */
	0x11, 0x80, 0x23, 0xEF, 0xBE, 0x6B, 0x69, 0x69, 0x69, 0x35,
	0x00, 0xFF, 0x17, 0x43, 0x3D, 0x4B, 0xD5, 0x05, 0x57, 0xFA,
	0x7F, 0x08, 0xA8, 0x05, 0xF0, 0xFF, 0x7D, 0x4A, 0xFE, 0xFE,
	0x87, 0xFA, 0x1F, 0xE8, 0x0F, 0xF0, 0xFF, 0x00, 0x89, 0x17
};

LOCAL char	json[1024];		/* static: keeps the test task stack small */

LOCAL BOOL same(const UB *a, const UB *b, UINT n)
{
	UINT	i;

	for (i = 0; i < n; i++) {
		if (a[i] != b[i]) {
			return FALSE;
		}
	}
	return TRUE;
}

LOCAL void copy(UB *dst, const UB *src, UINT n)
{
	UINT	i;

	for (i = 0; i < n; i++) {
		dst[i] = src[i];
	}
}

/* Recompute the CRC after editing a frame, so the check BEHIND the CRC
   is reached (the CRC itself is verified against S1-U01) */
LOCAL void reframe(UB *f, UINT len)
{
	UH	c = crc16_ccitt(f, len - MP_CRC_LEN);

	f[len - 2U] = (UB)(c & 0xFFU);
	f[len - 1U] = (UB)(c >> 8);
}

/* A command frame with arbitrary field values (bypasses the encoder) */
LOCAL void raw_cmd(UB *f, UW cmd, UW arg, UW id, UW rsv)
{
	BIT_WRITER	w;

	f[0] = (UB)((MP_VERSION << 4) | MT_COMMAND);
	f[1] = 1;
	f[2] = MP_CMD_PAYLOAD;
	bw_init(&w, f + MP_HDR_LEN, MP_CMD_PAYLOAD);
	bw_put(&w, cmd, 3);
	bw_put(&w, arg, 10);
	bw_put(&w, id, 8);
	bw_put(&w, rsv, 3);
	reframe(f, MP_HDR_LEN + MP_CMD_PAYLOAD + MP_CRC_LEN);
}

EXPORT BOOL s1_tlm_equal(const TLM_RECORD *a, const TLM_RECORD *b)
{
#define EQ(f)	(a->f == b->f)
	return EQ(uptime_ds) && EQ(mode) && EQ(epoch) && EQ(faults)
	    && EQ(mstate) && EQ(msource) && EQ(cmd_active) && EQ(speed_l)
	    && EQ(speed_r) && EQ(odometer_cm) && EQ(heading2) && EQ(cruise10)
	    && EQ(ir_black) && EQ(line_pos) && EQ(line_state) && EQ(steer)
	    && EQ(bc_state) && EQ(bc_symbol) && EQ(bc_result) && EQ(nav)
	    && EQ(nav_valid) && EQ(bc_valid) && EQ(bc_invalid) && EQ(imu_status)
	    && EQ(pitch) && EQ(roll) && EQ(yaw_rate) && EQ(heading)
	    && EQ(motion_class) && EQ(tilt) && EQ(collision) && EQ(shock)
	    && EQ(hump_last) && EQ(hump_max) && EQ(hump_count) && EQ(hump_active)
	    && EQ(us_dist) && EQ(us_result) && EQ(avoid_state) && EQ(avoid_side)
	    && EQ(avoid_outcome) && EQ(obst_width) && EQ(obst_bearing)
	    && EQ(clear_left) && EQ(clear_right) && EQ(comm_state)
	    && EQ(tlm_dropped) && EQ(rx_rejected);
#undef EQ
}

/* The record behind gold_tlm: signs, maxima and every invalid code */
LOCAL void gold_record(TLM_RECORD *r)
{
	r->uptime_ds = 0xBEEF;	r->mode = 3;		r->epoch = 5;
	r->faults = 0x5A5A5A5UL;
	r->mstate = 2;		r->msource = 2;		r->cmd_active = 1;
	r->speed_l = -1024;	r->speed_r = 1023;	r->odometer_cm = 54321;
	r->heading2 = 179;	r->cruise10 = 20;
	r->ir_black = 5;	r->line_pos = 6;	r->line_state = 1;
	r->steer = -63;		r->bc_state = 3;	r->bc_symbol = 10;
	r->bc_result = 1;	r->nav = 3;		r->nav_valid = 1;
	r->bc_valid = 63;	r->bc_invalid = 7;
	r->imu_status = 2;	r->pitch = -128;	r->roll = 90;
	r->yaw_rate = -256;	r->heading = 511;	r->motion_class = 3;
	r->tilt = 1;		r->collision = 0;	r->shock = 31;
	r->hump_last = 20;	r->hump_max = 50;	r->hump_count = 15;
	r->hump_active = 1;
	r->us_dist = 1022;	r->us_result = 1;	r->avoid_state = 8;
	r->avoid_side = 2;	r->avoid_outcome = 2;	r->obst_width = 255;
	r->obst_bearing = -127;	r->clear_left = 254;	r->clear_right = 0;
	r->comm_state = 7;	r->tlm_dropped = 255;	r->rx_rejected = 1;
}

/* Every field at its maximum (hi) or minimum (!hi) */
LOCAL void limit_record(TLM_RECORD *r, BOOL hi)
{
#define UMAX(n)	(hi ? ((1UL << (n)) - 1UL) : 0UL)
#define SLIM(n)	(hi ? ((1L << ((n) - 1)) - 1L) : -(1L << ((n) - 1)))
	r->uptime_ds = (UH)UMAX(16);	r->mode = (UB)UMAX(3);	r->epoch = (UB)UMAX(3);
	r->faults = UMAX(27);
	r->mstate = (UB)UMAX(2);	r->msource = (UB)UMAX(2);
	r->cmd_active = (UB)UMAX(1);	r->speed_l = (H)SLIM(11);
	r->speed_r = (H)SLIM(11);	r->odometer_cm = (UH)UMAX(16);
	r->heading2 = (UB)UMAX(8);	r->cruise10 = (UB)UMAX(6);
	r->ir_black = (UB)UMAX(3);	r->line_pos = (UB)UMAX(3);
	r->line_state = (UB)UMAX(2);	r->steer = (B)SLIM(7);
	r->bc_state = (UB)UMAX(2);	r->bc_symbol = (UB)UMAX(6);
	r->bc_result = (UB)UMAX(2);	r->nav = (UB)UMAX(2);
	r->nav_valid = (UB)UMAX(1);	r->bc_valid = (UB)UMAX(6);
	r->bc_invalid = (UB)UMAX(6);	r->imu_status = (UB)UMAX(2);
	r->pitch = (B)SLIM(8);		r->roll = (B)SLIM(8);
	r->yaw_rate = (H)SLIM(9);	r->heading = (UH)UMAX(9);
	r->motion_class = (UB)UMAX(2);	r->tilt = (UB)UMAX(1);
	r->collision = (UB)UMAX(1);	r->shock = (UB)UMAX(5);
	r->hump_last = (UB)UMAX(6);	r->hump_max = (UB)UMAX(6);
	r->hump_count = (UB)UMAX(4);	r->hump_active = (UB)UMAX(1);
	r->us_dist = (UH)UMAX(10);	r->us_result = (UB)UMAX(2);
	r->avoid_state = (UB)UMAX(4);	r->avoid_side = (UB)UMAX(2);
	r->avoid_outcome = (UB)UMAX(2);	r->obst_width = (UB)UMAX(8);
	r->obst_bearing = (B)SLIM(8);	r->clear_left = (UB)UMAX(8);
	r->clear_right = (UB)UMAX(8);	r->comm_state = (UB)UMAX(3);
	r->tlm_dropped = (UB)UMAX(8);	r->rx_rejected = (UB)UMAX(8);
#undef UMAX
#undef SLIM
}

/* ------------------------------------------------------------------ */

LOCAL INT test_bits(void)
{
	BIT_WRITER	w;
	BIT_READER	r;
	UB		buf[8];
	BOOL		ok;

	TEST_INIT();
	TEST_CHECK("S1-U01", crc16_ccitt((const UB *)"123456789", 9) == 0x29B1U
		   && crc16_ccitt((const UB *)"", 0) == 0xFFFFU,
		   "CRC-16/CCITT-FALSE: catalogue check value 0x29B1; empty input 0xFFFF");

	bw_init(&w, buf, 3);
	bw_put(&w, 0x5, 3);
	bw_put(&w, 0x1F, 5);
	bw_put(&w, 0xABC, 12);
	bw_put_signed(&w, -3, 4);
	ok = !w.overflow && bw_bytes(&w) == 3U
	     && buf[0] == 0xFDU && buf[1] == 0xBCU && buf[2] == 0xDAU;
	bw_put(&w, 1, 1);				/* bit 25 does not fit */
	ok = ok && w.overflow;
	br_init(&r, buf, 3);
	ok = ok && br_get(&r, 3) == 5U && br_get(&r, 5) == 0x1FU
	     && br_get(&r, 12) == 0xABCU && br_get_signed(&r, 4) == -3 && !r.underflow;
	(void)br_get(&r, 1);
	ok = ok && r.underflow;
	bw_init(&w, buf, 8);
	bw_put(&w, 0, 0);
	ok = ok && w.overflow;
	bw_init(&w, buf, 8);
	bw_put(&w, 0, 33);
	ok = ok && w.overflow;
	TEST_CHECK("S1-U02", ok,
		   "LSB-first bit order (known bytes FD BC DA), signed fields, overflow/underflow flagged");
	TEST_SUMMARY("s1 bits");
	return test_fail_;
}

LOCAL INT test_frames(void)
{
	HB_RECORD	h, h2;
	CMD_RECORD	c;
	ACK_RECORD	k, k2;
	EVT_RECORD	e, e2;
	TLM_RECORD	a, b;
	UB		f[MP_MAX_FRAME];
	INT		n, seq;
	BOOL		ok;

	TEST_INIT();
	h.uptime_ds = 0x1234;
	h.mode = 4;
	h.comm_state = 5;
	h.critical = 1;
	h.fault_count = 27;
	h.reconnects = 3;
	n = mp_hb_encode(&h, 0x2A, f, sizeof(f));
	ok = (n == 9) && same(f, gold_hb, 9);
	seq = mp_hb_decode(gold_hb, 9, &h2);
	ok = ok && seq == 0x2A && h2.uptime_ds == 0x1234U && h2.mode == 4U
	     && h2.comm_state == 5U && h2.critical == 1U && h2.fault_count == 27U
	     && h2.reconnects == 3U;
	h.reconnects = 200;
	n = mp_hb_encode(&h, 0, f, sizeof(f));
	ok = ok && n == 9 && mp_hb_decode(f, (UINT)n, &h2) == 0 && h2.reconnects == 15U;
	TEST_CHECK("S1-U03", ok,
		   "heartbeat = golden bytes 12 2A 04 34 12 EC 3D 60 E5; decodes; reconnects saturate at 15");

	c.cmd = CMD_SET_SPEED;
	c.arg = 250;
	c.cmd_id = 0x5A;
	n = mp_cmd_encode(&c, 7, f, sizeof(f));
	ok = (n == 8) && same(f, gold_cmd, 8);
	c.cmd = 0;
	c.arg = 0;
	c.cmd_id = 0;
	seq = mp_cmd_decode(gold_cmd, 8, &c);
	ok = ok && seq == 7 && c.cmd == CMD_SET_SPEED && c.arg == 250U && c.cmd_id == 0x5AU;
	TEST_CHECK("S1-U04", ok, "command frame = golden bytes; decoded and validated (SET_SPEED 250, id 0x5A)");

	k.cmd_id = 0x5A;
	k.cmd = CMD_SET_SPEED;
	k.result = ACK_REJ_STATE;
	k.detail = MREJ_MODE;
	n = mp_ack_encode(&k, 3, f, sizeof(f));
	ok = (n == 8) && same(f, gold_ack, 8);
	seq = mp_ack_decode(gold_ack, 8, &k2);
	ok = ok && seq == 3 && k2.cmd_id == 0x5AU && k2.cmd == CMD_SET_SPEED
	     && k2.result == ACK_REJ_STATE && k2.detail == MREJ_MODE;
	e.code = EVT_HUMP;
	e.data = 20UL | (25UL << 6) | (3UL << 12);
	n = mp_evt_encode(&e, 9, f, sizeof(f));
	ok = ok && (n == 10) && same(f, gold_evt, 10);
	seq = mp_evt_decode(gold_evt, 10, &e2);
	ok = ok && seq == 9 && e2.code == EVT_HUMP && e2.data == e.data;
	TEST_CHECK("S1-U05", ok, "acknowledgement and event frames = golden bytes; both decode");

	gold_record(&a);
	n = mp_tlm_encode(&a, 0x80, f, sizeof(f));
	ok = (n == (INT)(MP_HDR_LEN + MP_TLM_PAYLOAD + MP_CRC_LEN)) && n == 40
	     && same(f, gold_tlm, 40);
	seq = mp_tlm_decode(gold_tlm, 40, &b);
	ok = ok && seq == 0x80 && s1_tlm_equal(&a, &b);
	TEST_CHECK("S1-U06", ok,
		   "telemetry: 279 bits -> 40-byte frame = golden frame; decodes field by field");

	limit_record(&a, TRUE);
	n = mp_tlm_encode(&a, 1, f, sizeof(f));
	ok = n == 40 && mp_tlm_decode(f, (UINT)n, &b) == 1 && s1_tlm_equal(&a, &b);
	limit_record(&a, FALSE);
	n = mp_tlm_encode(&a, 2, f, sizeof(f));
	ok = ok && n == 40 && mp_tlm_decode(f, (UINT)n, &b) == 2 && s1_tlm_equal(&a, &b);
	TEST_CHECK("S1-U07", ok, "telemetry round trip with every field at its maximum and at its minimum");
	TEST_SUMMARY("s1 frames");
	return test_fail_;
}

LOCAL INT test_validation(void)
{
	CMD_RECORD	c;
	TLM_RECORD	t;
	MP_HDR		hdr;
	UB		f[MP_MAX_FRAME];
	INT		r[8];
	UW		type;
	INT		arg;
	UB		detail;
	BOOL		ok;

	TEST_INIT();
	copy(f, gold_cmd, 8);
	f[4] ^= 0x10U;				/* one payload bit flipped */
	r[0] = mp_cmd_decode(f, 8, &c);
	r[1] = mp_cmd_decode(gold_cmd, 4, &c);	/* truncated */
	copy(f, gold_cmd, 8);
	f[0] = 0x24U;				/* version 2 */
	reframe(f, 8);
	r[2] = mp_cmd_decode(f, 8, &c);
	copy(f, gold_cmd, 8);
	f[0] = 0x16U;				/* type 6 does not exist */
	reframe(f, 8);
	r[3] = mp_frame_check(f, 8, &hdr);
	copy(f, gold_cmd, 8);
	f[2] = 4U;				/* length byte disagrees */
	reframe(f, 8);
	r[4] = mp_cmd_decode(f, 8, &c);
	r[5] = mp_cmd_decode(gold_hb, 9, &c);	/* heartbeat is no command */
	r[6] = mp_tlm_decode(gold_cmd, 8, &t);
	f[0] = 0x14U;				/* command with a 2-byte payload */
	f[1] = 1U;
	f[2] = 2U;
	f[3] = 0U;
	f[4] = 0U;
	reframe(f, 7);
	r[7] = mp_cmd_decode(f, 7, &c);
	TEST_CHECK("S1-U08", r[0] == MPE_CRC && r[1] == MPE_SHORT && r[2] == MPE_VERSION
		   && r[3] == MPE_TYPE && r[4] == MPE_LENGTH && r[5] == MPE_TYPE
		   && r[6] == MPE_TYPE && r[7] == MPE_LENGTH,
		   "rejected: bad CRC, truncated, version, unknown type, length byte, wrong type, schema size");

	raw_cmd(f, CMD_STOP, 0, 9, 1);
	ok = mp_cmd_decode(f, 8, &c) == MPE_FIELD && c.cmd_id == 9U;
	raw_cmd(f, 6, 0, 9, 0);
	ok = ok && mp_cmd_decode(f, 8, &c) == MPE_FIELD;
	raw_cmd(f, 7, 0, 9, 0);
	ok = ok && mp_cmd_decode(f, 8, &c) == MPE_FIELD;
	raw_cmd(f, CMD_SET_SPEED, 49, 9, 0);
	ok = ok && mp_cmd_decode(f, 8, &c) == MPE_ARG;
	raw_cmd(f, CMD_SET_SPEED, 401, 9, 0);
	ok = ok && mp_cmd_decode(f, 8, &c) == MPE_ARG;
	raw_cmd(f, CMD_SET_SPEED, 1023, 9, 0);
	ok = ok && mp_cmd_decode(f, 8, &c) == MPE_ARG;
	raw_cmd(f, CMD_SET_SPEED, 50, 9, 0);
	ok = ok && mp_cmd_decode(f, 8, &c) == 1 && c.arg == 50U;
	raw_cmd(f, CMD_SET_SPEED, 400, 9, 0);
	ok = ok && mp_cmd_decode(f, 8, &c) == 1 && c.arg == 400U;
	raw_cmd(f, CMD_STOP, 1, 9, 0);
	ok = ok && mp_cmd_decode(f, 8, &c) == MPE_ARG;
	raw_cmd(f, CMD_PING, 0, 9, 0);
	ok = ok && mp_cmd_decode(f, 8, &c) == 1 && c.cmd == CMD_PING;
	TEST_CHECK("S1-U09", ok,
		   "command fields: reserved bits / codes 6-7 rejected; SET_SPEED 50..400 only; no stray args");

	TEST_CHECK("S1-U10", mp_err_to_ack(MPE_VERSION) == ACK_BAD_VERSION
		   && mp_err_to_ack(MPE_TYPE) == ACK_BAD_TYPE
		   && mp_err_to_ack(MPE_SHORT) == ACK_BAD_LENGTH
		   && mp_err_to_ack(MPE_LENGTH) == ACK_BAD_LENGTH
		   && mp_err_to_ack(MPE_FIELD) == ACK_BAD_FIELD
		   && mp_err_to_ack(MPE_ARG) == ACK_BAD_ARG,
		   "decoder errors map to distinct acknowledgement codes");

	c.arg = 0;
	c.cmd = CMD_STOP;
	ok = mp_cmd_to_motion(&c, &type, &arg) && type == MCMD_STOP && arg == 0;
	c.cmd = CMD_RESUME;
	ok = ok && mp_cmd_to_motion(&c, &type, &arg) && type == MCMD_RESUME;
	c.cmd = CMD_CLEAR;
	ok = ok && mp_cmd_to_motion(&c, &type, &arg) && type == MCMD_CLEAR;
	c.cmd = CMD_SET_SPEED;
	c.arg = 275;
	ok = ok && mp_cmd_to_motion(&c, &type, &arg) && type == MCMD_SET_SPEED && arg == 275;
	c.cmd = CMD_PING;
	ok = ok && !mp_cmd_to_motion(&c, &type, &arg);
	c.cmd = CMD_REQ_STATUS;
	ok = ok && !mp_cmd_to_motion(&c, &type, &arg);
	ok = ok && mp_ack_of_result(mres_pack(mcmd_pack(MCMD_STOP, MSRC_SUPER, 1, 0, 0, 0),
					      MRES_DONE, MREJ_NONE), &detail) == ACK_OK
	     && detail == 0U;
	ok = ok && mp_ack_of_result(mres_pack(mcmd_pack(MCMD_RESUME, MSRC_SUPER, 2, 0, 0, 0),
					      MRES_REJECTED, MREJ_MODE), &detail) == ACK_REJ_STATE
	     && detail == MREJ_MODE;
	ok = ok && mp_ack_of_result(0, &detail) == ACK_BUSY;
	TEST_CHECK("S1-U11", ok,
		   "commands map to supervisory requests only; Buddy 2's refusal is acknowledged with its reason");
	TEST_SUMMARY("s1 validation");
	return test_fail_;
}

LOCAL INT test_sizes(void)
{
	TLM_RECORD	a;
	HB_RECORD	h = {0};
	CMD_RECORD	c = {0};
	ACK_RECORD	k = {0};
	EVT_RECORD	e = {0};
	UB		f[MP_MAX_FRAME];
	INT		n, j;

	TEST_INIT();
	gold_record(&a);
	TEST_CHECK("S1-U12", mp_tlm_encode(&a, 0, f, 39) == MPE_SPACE
		   && mp_hb_encode(&h, 0, f, 8) == MPE_SPACE
		   && mp_cmd_encode(&c, 0, f, 7) == MPE_SPACE
		   && mp_ack_encode(&k, 0, f, 7) == MPE_SPACE
		   && mp_evt_encode(&e, 0, f, 9) == MPE_SPACE,
		   "every encoder refuses a buffer one byte too small (no overrun)");

	n = mp_tlm_encode(&a, 0, f, sizeof(f));
	j = mp_tlm_json(&a, json, sizeof(json));
	tm_printf((UB*)"[TEST]   telemetry size: binary frame %d bytes, same fields as JSON %d bytes\n",
		  n, j);
	TEST_CHECK("S1-U13", n == 40 && j >= 10 * n,
		   "binary telemetry frame (40 B) at least 10x smaller than the JSON text form");
	TEST_SUMMARY("s1 sizes");
	return test_fail_;
}

/* Run the connection state machine for 'ms' at the 100 ms task period */
LOCAL void run(COMM_FSM *f, BOOL cfg, UB link, UB session, UW ms)
{
	UW	t;

	for (t = 0; t < ms; t += 100U) {
		cfsm_step(f, cfg, link, session, 100U);
	}
}

LOCAL INT test_comm_fsm(void)
{
	static const UW	expect[6] = { 1000, 2000, 4000, 8000, 16000, 16000 };
	COMM_FSM	f;
	UB		s1, s2, s3;
	UW		waits[6], in_retry = 0, cur = 0, max_mqtt = 0;
	UINT		k, nw = 0;
	BOOL		ok, recovery = FALSE;

	TEST_INIT();
	cfsm_init(&f);
	run(&f, FALSE, CL_DOWN, CSS_NONE, 1000);
	ok = f.state == CS_DISABLED && !f.wifi_loss && !f.mqtt_loss
	     && cfsm_availability(&f) == CAV_DISCONNECTED;
	run(&f, TRUE, CL_JOINING, CSS_NONE, 100);
	TEST_CHECK("S1-U14", ok && f.state == CS_WIFI,
		   "no network stack in the image: DISABLED without faults; starts once configured");

	cfsm_init(&f);
	run(&f, TRUE, CL_JOINING, CSS_NONE, 500);
	s1 = f.state;
	run(&f, TRUE, CL_UP, CSS_NONE, 100);
	s2 = f.state;
	run(&f, TRUE, CL_UP, CSS_CONNECTED, 100);
	s3 = f.state;
	run(&f, TRUE, CL_UP, CSS_SUBSCRIBED, 100);
	TEST_CHECK("S1-U15", s1 == CS_WIFI && s2 == CS_MQTT && s3 == CS_SUBSCRIBE
		   && f.state == CS_ONLINE && cfsm_availability(&f) == CAV_ONLINE
		   && f.attempts == 0U && f.reconnects == 0U && !f.wifi_loss && !f.mqtt_loss,
		   "connect sequence WIFI -> MQTT -> SUBSCRIBE -> ONLINE");

	run(&f, TRUE, CL_FAILED, CSS_LOST, 100);
	TEST_CHECK("S1-U16", f.state == CS_RETRY && f.wifi_loss
		   && cfsm_availability(&f) == CAV_RECOVERING,
		   "WiFi lost while online: WIFI_LOSS (degraded) and RECOVERING reported");

	for (k = 0; k < 2000U && nw < 6U; k++) {
		UB	before = f.state;

		cfsm_step(&f, TRUE, CL_FAILED, CSS_NONE, 100U);
		if (before == CS_RETRY) {
			in_retry += 100U;
			if (f.state != CS_RETRY) {
				waits[nw++] = in_retry;
				in_retry = 0;
			}
		}
		if (f.reconnect_req) {
			recovery = TRUE;		/* RECOVERY asked the client to restart */
			f.reconnect_req = FALSE;
		}
	}
	ok = (nw == 6U) && recovery;
	for (k = 0; ok && k < 6U; k++) {
		ok = (waits[k] == expect[k]);
	}
	TEST_CHECK("S1-U17", ok,
		   "retries wait 1, 2, 4, 8, 16, 16 s (capped, non-blocking); every 3rd restarts the client");

	for (k = 0; k < 300U && f.state != CS_ONLINE; k++) {
		cfsm_step(&f, TRUE, CL_UP, CSS_SUBSCRIBED, 100U);
	}
	TEST_CHECK("S1-U18", f.state == CS_ONLINE && f.reconnects == 1U && f.attempts == 0U
		   && f.backoff_ms == (UW)CFSM_BACKOFF_MIN_MS && !f.wifi_loss && !f.mqtt_loss,
		   "automatic reconnection: ONLINE again, counted, back-off reset, faults cleared");

	run(&f, TRUE, CL_UP, CSS_LOST, 100);
	ok = f.state == CS_RETRY && f.mqtt_loss && !f.wifi_loss;
	for (k = 0; k < 600U; k++) {			/* broker down for 60 s */
		cfsm_step(&f, TRUE, CL_UP, CSS_NONE, 100U);
		cur = (f.state == CS_MQTT) ? cur + 100U : 0U;
		if (cur > max_mqtt) {
			max_mqtt = cur;
		}
	}
	TEST_CHECK("S1-U19", ok && max_mqtt > 0U && max_mqtt <= (UW)CFSM_MQTT_TIMEOUT_MS
		   && f.mqtt_loss && !f.wifi_loss,
		   "broker lost with WiFi up: MQTT_LOSS only; each wait for the broker bounded to 10 s");
	TEST_SUMMARY("s1 comm fsm");
	return test_fail_;
}

LOCAL INT test_status_word(void)
{
	S1_COMM_STATUS	a, b;
	UW		w;

	TEST_INIT();
	a.state = CS_DISABLED;
	a.avail = CAV_RECOVERING;
	a.link = CL_FAILED;
	a.session = CSS_LOST;
	a.attempts = 200;
	a.reconnects = 99;
	a.tlm_dropped = 0;
	a.rx_rejected = 0;
	w = s1_comm_pack(&a);
	s1_comm_unpack(w, &b);
	TEST_CHECK("S1-U20", (w >> 17) == 0U && b.state == CS_DISABLED && b.avail == CAV_RECOVERING
		   && b.link == CL_FAILED && b.session == CSS_LOST && b.attempts == 15U
		   && b.reconnects == 15U,
		   "communication status word: 17 bits, counters saturate at 15");
	TEST_SUMMARY("s1 status word");
	return test_fail_;
}

EXPORT INT s1_run_unit_tests(void)
{
	return test_bits() + test_frames() + test_validation() + test_sizes()
	     + test_comm_fsm() + test_status_word();
}

#endif /* APP_TEST || HOST_TEST */
