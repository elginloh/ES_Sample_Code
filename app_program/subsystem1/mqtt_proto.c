/*
 *	mqtt_proto.c
 *	CSC14 binary MQTT protocol v1 (see mqtt_proto.h and
 *	docs/MQTT_BINARY_PROTOCOL.md).
 */

#include "mqtt_proto.h"
#include "bitpack.h"
#include "../subsystem2/motion_msg.h"

/* ---------------------------------------------------------------------
 *  Telemetry schema v1: ONE table drives encode, decode, the bit count
 *  check and the JSON comparison, so they cannot drift apart.
 *  X(field, bits, signed)
 * --------------------------------------------------------------------- */
#define TLM_FIELDS(X)					\
	X(uptime_ds, 16, 0)	X(mode, 3, 0)		\
	X(epoch, 3, 0)		X(faults, 27, 0)	\
	X(mstate, 2, 0)		X(msource, 2, 0)	\
	X(cmd_active, 1, 0)	X(speed_l, 11, 1)	\
	X(speed_r, 11, 1)	X(odometer_cm, 16, 0)	\
	X(heading2, 8, 0)	X(cruise10, 6, 0)	\
	X(ir_black, 3, 0)	X(line_pos, 3, 0)	\
	X(line_state, 2, 0)	X(steer, 7, 1)		\
	X(bc_state, 2, 0)	X(bc_symbol, 6, 0)	\
	X(bc_result, 2, 0)	X(nav, 2, 0)		\
	X(nav_valid, 1, 0)	X(bc_valid, 6, 0)	\
	X(bc_invalid, 6, 0)	X(imu_status, 2, 0)	\
	X(pitch, 8, 1)		X(roll, 8, 1)		\
	X(yaw_rate, 9, 1)	X(heading, 9, 0)	\
	X(motion_class, 2, 0)	X(tilt, 1, 0)		\
	X(collision, 1, 0)	X(shock, 5, 0)		\
	X(hump_last, 6, 0)	X(hump_max, 6, 0)	\
	X(hump_count, 4, 0)	X(hump_active, 1, 0)	\
	X(us_dist, 10, 0)	X(us_result, 2, 0)	\
	X(avoid_state, 4, 0)	X(avoid_side, 2, 0)	\
	X(avoid_outcome, 2, 0)	X(obst_width, 8, 0)	\
	X(obst_bearing, 8, 1)	X(clear_left, 8, 0)	\
	X(clear_right, 8, 0)	X(comm_state, 3, 0)	\
	X(tlm_dropped, 8, 0)	X(rx_rejected, 8, 0)

#define TLM_SUM(f, n, s)	+ (n)
enum { TLM_SCHEMA_BITS = 0 TLM_FIELDS(TLM_SUM) };
/* compile-time check: the documented size matches the table */
typedef char tlm_size_check[(TLM_SCHEMA_BITS == (INT)MP_TLM_BITS) ? 1 : -1];

/* ------------------------------------------------------------------ */
/*  Framing                                                            */

LOCAL INT frame_finish(UB *out, UINT cap, UB type, UB seq, UINT plen)
{
	UH	crc;

	if (MP_HDR_LEN + plen + MP_CRC_LEN > cap) {
		return MPE_SPACE;
	}
	out[0] = (UB)((MP_VERSION << 4) | (type & 0xFU));
	out[1] = seq;
	out[2] = (UB)plen;
	crc = crc16_ccitt(out, MP_HDR_LEN + plen);
	out[MP_HDR_LEN + plen] = (UB)(crc & 0xFFU);		/* little-endian */
	out[MP_HDR_LEN + plen + 1U] = (UB)(crc >> 8);
	return (INT)(MP_HDR_LEN + plen + MP_CRC_LEN);
}

EXPORT INT mp_frame_check(const UB *frame, UINT len, MP_HDR *hdr)
{
	UH	crc_rx;

	if (frame == NULL || len < MP_HDR_LEN + MP_CRC_LEN) {
		return MPE_SHORT;
	}
	crc_rx = (UH)(frame[len - 2U] | ((UH)frame[len - 1U] << 8));
	if (crc16_ccitt(frame, len - MP_CRC_LEN) != crc_rx) {
		return MPE_CRC;			/* nothing else can be trusted */
	}
	hdr->version = (UB)(frame[0] >> 4);
	hdr->type = (UB)(frame[0] & 0xFU);
	hdr->seq = frame[1];
	hdr->len = frame[2];
	if (hdr->version != MP_VERSION) {
		return MPE_VERSION;
	}
	if (hdr->type < MT_TELEMETRY || hdr->type > MT_ACK) {
		return MPE_TYPE;
	}
	if ((UINT)hdr->len != len - MP_HDR_LEN - MP_CRC_LEN) {
		return MPE_LENGTH;
	}
	return (INT)MP_HDR_LEN;
}

/* Frame check + expected type and payload size */
LOCAL INT open_frame(const UB *frame, UINT len, UB type, UINT plen, BIT_READER *rd)
{
	MP_HDR	h;
	INT	r = mp_frame_check(frame, len, &h);

	if (r < 0) {
		return r;
	}
	if (h.type != type) {
		return MPE_TYPE;
	}
	if ((UINT)h.len != plen) {
		return MPE_LENGTH;
	}
	br_init(rd, frame + MP_HDR_LEN, plen);
	return (INT)h.seq;
}

/* ------------------------------------------------------------------ */
/*  Telemetry                                                          */

EXPORT INT mp_tlm_encode(const TLM_RECORD *r, UB seq, UB *out, UINT cap)
{
	BIT_WRITER	w;

	if (cap < MP_HDR_LEN + MP_TLM_PAYLOAD + MP_CRC_LEN) {
		return MPE_SPACE;
	}
	bw_init(&w, out + MP_HDR_LEN, MP_TLM_PAYLOAD);
#define TLM_ENC(f, n, s)						\
	if (s) { bw_put_signed(&w, (W)r->f, (n)); }			\
	else { bw_put(&w, (UW)r->f, (n)); }
	TLM_FIELDS(TLM_ENC)
#undef TLM_ENC
	if (w.overflow) {
		return MPE_SPACE;
	}
	return frame_finish(out, cap, MT_TELEMETRY, seq, MP_TLM_PAYLOAD);
}

EXPORT INT mp_tlm_decode(const UB *frame, UINT len, TLM_RECORD *r)
{
	BIT_READER	rd;
	INT		seq = open_frame(frame, len, MT_TELEMETRY, MP_TLM_PAYLOAD, &rd);

	if (seq < 0) {
		return seq;
	}
#define TLM_DEC(f, n, s)						\
	if (s) { r->f = br_get_signed(&rd, (n)); }			\
	else { r->f = br_get(&rd, (n)); }
	TLM_FIELDS(TLM_DEC)
#undef TLM_DEC
	return rd.underflow ? MPE_LENGTH : seq;
}

/* ------------------------------------------------------------------ */
/*  Heartbeat                                                          */

EXPORT INT mp_hb_encode(const HB_RECORD *r, UB seq, UB *out, UINT cap)
{
	BIT_WRITER	w;

	if (cap < MP_HDR_LEN + MP_HB_PAYLOAD + MP_CRC_LEN) {
		return MPE_SPACE;
	}
	bw_init(&w, out + MP_HDR_LEN, MP_HB_PAYLOAD);
	bw_put(&w, r->uptime_ds, 16);
	bw_put(&w, r->mode, 3);
	bw_put(&w, r->comm_state, 3);
	bw_put(&w, r->critical, 1);
	bw_put(&w, r->fault_count, 5);
	bw_put(&w, (r->reconnects > 15U) ? 15U : r->reconnects, 4);
	return frame_finish(out, cap, MT_HEARTBEAT, seq, MP_HB_PAYLOAD);
}

EXPORT INT mp_hb_decode(const UB *frame, UINT len, HB_RECORD *r)
{
	BIT_READER	rd;
	INT		seq = open_frame(frame, len, MT_HEARTBEAT, MP_HB_PAYLOAD, &rd);

	if (seq < 0) {
		return seq;
	}
	r->uptime_ds = (UH)br_get(&rd, 16);
	r->mode = (UB)br_get(&rd, 3);
	r->comm_state = (UB)br_get(&rd, 3);
	r->critical = (UB)br_get(&rd, 1);
	r->fault_count = (UB)br_get(&rd, 5);
	r->reconnects = (UB)br_get(&rd, 4);
	return seq;
}

/* ------------------------------------------------------------------ */
/*  Events: 4-bit code + 32-bit code-specific data word                */

EXPORT INT mp_evt_encode(const EVT_RECORD *r, UB seq, UB *out, UINT cap)
{
	BIT_WRITER	w;

	if (cap < MP_HDR_LEN + MP_EVT_PAYLOAD + MP_CRC_LEN) {
		return MPE_SPACE;
	}
	bw_init(&w, out + MP_HDR_LEN, MP_EVT_PAYLOAD);
	bw_put(&w, r->code, 4);
	bw_put(&w, r->data, 32);
	return frame_finish(out, cap, MT_EVENT, seq, MP_EVT_PAYLOAD);
}

EXPORT INT mp_evt_decode(const UB *frame, UINT len, EVT_RECORD *r)
{
	BIT_READER	rd;
	INT		seq = open_frame(frame, len, MT_EVENT, MP_EVT_PAYLOAD, &rd);

	if (seq < 0) {
		return seq;
	}
	r->code = (UB)br_get(&rd, 4);
	r->data = br_get(&rd, 32);
	return seq;
}

/* ------------------------------------------------------------------ */
/*  Commands                                                           */

EXPORT INT mp_cmd_encode(const CMD_RECORD *r, UB seq, UB *out, UINT cap)
{
	BIT_WRITER	w;

	if (cap < MP_HDR_LEN + MP_CMD_PAYLOAD + MP_CRC_LEN) {
		return MPE_SPACE;
	}
	bw_init(&w, out + MP_HDR_LEN, MP_CMD_PAYLOAD);
	bw_put(&w, r->cmd, 3);
	bw_put(&w, r->arg, 10);
	bw_put(&w, r->cmd_id, 8);
	bw_put(&w, 0U, 3);			/* reserved */
	return frame_finish(out, cap, MT_COMMAND, seq, MP_CMD_PAYLOAD);
}

EXPORT INT mp_cmd_decode(const UB *frame, UINT len, CMD_RECORD *r)
{
	BIT_READER	rd;
	INT		seq = open_frame(frame, len, MT_COMMAND, MP_CMD_PAYLOAD, &rd);
	UW		rsv;

	if (seq < 0) {
		return seq;
	}
	r->cmd = (UB)br_get(&rd, 3);
	r->arg = (UH)br_get(&rd, 10);
	r->cmd_id = (UB)br_get(&rd, 8);
	rsv = br_get(&rd, 3);
	if (rsv != 0U || r->cmd >= CMD_COUNT) {
		return MPE_FIELD;
	}
	if (r->cmd == CMD_SET_SPEED) {
		if (r->arg < (UH)MOTION_SPEED_MIN_MMS || r->arg > (UH)MOTION_SPEED_MAX_MMS) {
			return MPE_ARG;
		}
	} else if (r->arg != 0U) {
		return MPE_ARG;
	}
	return seq;
}

EXPORT UB mp_err_to_ack(INT err)
{
	switch (err) {
	case MPE_VERSION:	return ACK_BAD_VERSION;
	case MPE_TYPE:		return ACK_BAD_TYPE;
	case MPE_SHORT:
	case MPE_LENGTH:	return ACK_BAD_LENGTH;
	case MPE_ARG:		return ACK_BAD_ARG;
	default:		return ACK_BAD_FIELD;
	}
}

EXPORT BOOL mp_cmd_to_motion(const CMD_RECORD *c, UW *mcmd_type, INT *arg)
{
	*arg = 0;
	switch (c->cmd) {
	case CMD_STOP:
		*mcmd_type = MCMD_STOP;
		return TRUE;
	case CMD_RESUME:
		*mcmd_type = MCMD_RESUME;
		return TRUE;
	case CMD_CLEAR:
		*mcmd_type = MCMD_CLEAR;
		return TRUE;
	case CMD_SET_SPEED:
		*mcmd_type = MCMD_SET_SPEED;
		*arg = (INT)c->arg;
		return TRUE;
	default:				/* PING, REQ_STATUS */
		*mcmd_type = MCMD_COUNT;
		return FALSE;
	}
}

EXPORT UB mp_ack_of_result(UW mres_word, UB *detail)
{
	*detail = 0;
	if ((mres_word & MRES_VALID) == 0UL) {
		return ACK_BUSY;
	}
	if (mres_status(mres_word) == MRES_DONE) {
		return ACK_OK;
	}
	*detail = (UB)mres_reason(mres_word);	/* why Buddy 2 refused */
	return ACK_REJ_STATE;
}

/* ------------------------------------------------------------------ */
/*  Acknowledgements                                                   */

EXPORT INT mp_ack_encode(const ACK_RECORD *r, UB seq, UB *out, UINT cap)
{
	BIT_WRITER	w;

	if (cap < MP_HDR_LEN + MP_ACK_PAYLOAD + MP_CRC_LEN) {
		return MPE_SPACE;
	}
	bw_init(&w, out + MP_HDR_LEN, MP_ACK_PAYLOAD);
	bw_put(&w, r->cmd_id, 8);
	bw_put(&w, r->cmd, 3);
	bw_put(&w, r->result, 4);
	bw_put(&w, r->detail, 5);
	return frame_finish(out, cap, MT_ACK, seq, MP_ACK_PAYLOAD);
}

EXPORT INT mp_ack_decode(const UB *frame, UINT len, ACK_RECORD *r)
{
	BIT_READER	rd;
	INT		seq = open_frame(frame, len, MT_ACK, MP_ACK_PAYLOAD, &rd);

	if (seq < 0) {
		return seq;
	}
	r->cmd_id = (UB)br_get(&rd, 8);
	r->cmd = (UB)br_get(&rd, 3);
	r->result = (UB)br_get(&rd, 4);
	r->detail = (UB)br_get(&rd, 5);
	return seq;
}

/* ------------------------------------------------------------------ */
/*  JSON form of the same telemetry (measurement only)                 */

LOCAL BOOL jput(char *out, UINT cap, UINT *pos, const char *s)
{
	while (*s != '\0') {
		if (*pos + 1U >= cap) {
			return FALSE;
		}
		out[(*pos)++] = *s++;
	}
	out[*pos] = '\0';
	return TRUE;
}

LOCAL BOOL jint(char *out, UINT cap, UINT *pos, W v)
{
	char	tmp[12];
	UINT	n = 0;
	UW	u = (v < 0) ? (UW)(-v) : (UW)v;

	do {
		tmp[n++] = (char)('0' + (u % 10U));
		u /= 10U;
	} while (u != 0U);
	if (v < 0) {
		tmp[n++] = '-';
	}
	while (n > 0U) {
		if (*pos + 1U >= cap) {
			return FALSE;
		}
		out[(*pos)++] = tmp[--n];
	}
	out[*pos] = '\0';
	return TRUE;
}

EXPORT INT mp_tlm_json(const TLM_RECORD *r, char *out, UINT cap)
{
	UINT	pos = 0;
	BOOL	ok = TRUE;
	BOOL	first = TRUE;

	if (cap == 0U) {
		return MPE_SPACE;
	}
	ok = jput(out, cap, &pos, "{");
#define TLM_JSON(f, n, s)						\
	ok = ok && jput(out, cap, &pos, first ? "\"" #f "\":" : ",\"" #f "\":") \
		&& jint(out, cap, &pos, (W)r->f);			\
	first = FALSE;
	TLM_FIELDS(TLM_JSON)
#undef TLM_JSON
	ok = ok && jput(out, cap, &pos, "}");
	return ok ? (INT)pos : MPE_SPACE;
}
