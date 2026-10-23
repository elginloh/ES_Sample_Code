/*
 *	comm_fsm.c
 *	Connection state machine (see comm_fsm.h).
 */

#include "comm_fsm.h"

EXPORT void cfsm_init(COMM_FSM *f)
{
	f->state = CS_INIT;
	f->in_state_ms = 0;
	f->backoff_ms = CFSM_BACKOFF_MIN_MS;
	f->attempts = 0;
	f->reconnects = 0;
	f->was_online = FALSE;
	f->wifi_loss = FALSE;
	f->mqtt_loss = FALSE;
	f->reconnect_req = FALSE;
}

LOCAL void go(COMM_FSM *f, UB s)
{
	f->state = s;
	f->in_state_ms = 0;
}

/* A connection attempt failed: wait, doubling the delay up to the cap */
LOCAL void retry(COMM_FSM *f)
{
	if (f->attempts < 255U) {
		f->attempts++;
	}
	go(f, CS_RETRY);
}

EXPORT void cfsm_step(COMM_FSM *f, BOOL config_ok, UB link, UB session, UW dt_ms)
{
	f->in_state_ms += dt_ms;

	/* fault conditions follow the observed link/session, in every state */
	if (f->state != CS_INIT && f->state != CS_DISABLED) {
		f->wifi_loss = (link != CL_UP) ? TRUE : FALSE;
		f->mqtt_loss = (link == CL_UP && session != CSS_CONNECTED
				&& session != CSS_SUBSCRIBED
				&& (f->was_online || f->state == CS_RETRY)) ? TRUE : FALSE;
	}

	switch (f->state) {
	case CS_INIT:
		go(f, config_ok ? CS_WIFI : CS_DISABLED);
		break;

	case CS_WIFI:
		if (link == CL_UP) {
			go(f, CS_MQTT);
		} else if (link == CL_FAILED || f->in_state_ms >= CFSM_WIFI_TIMEOUT_MS) {
			retry(f);
		}
		break;

	case CS_MQTT:
		if (link != CL_UP) {
			go(f, CS_WIFI);
		} else if (session == CSS_CONNECTED || session == CSS_SUBSCRIBED) {
			go(f, CS_SUBSCRIBE);
		} else if (f->in_state_ms >= CFSM_MQTT_TIMEOUT_MS) {
			retry(f);			/* broker unavailable */
		}
		break;

	case CS_SUBSCRIBE:
		if (link != CL_UP || session == CSS_LOST || session == CSS_NONE) {
			retry(f);
		} else if (session == CSS_SUBSCRIBED) {
			if (f->was_online && f->reconnects < 255U) {
				f->reconnects++;
			}
			f->was_online = TRUE;
			f->attempts = 0;
			f->backoff_ms = CFSM_BACKOFF_MIN_MS;
			go(f, CS_ONLINE);
		} else if (f->in_state_ms >= CFSM_SUB_TIMEOUT_MS) {
			retry(f);
		}
		break;

	case CS_ONLINE:
		if (link != CL_UP || session == CSS_LOST || session == CSS_NONE) {
			retry(f);			/* connection lost */
		}
		break;

	case CS_RETRY:
		if (f->in_state_ms >= f->backoff_ms) {
			f->backoff_ms = (f->backoff_ms * 2U > CFSM_BACKOFF_MAX_MS)
				      ? CFSM_BACKOFF_MAX_MS : f->backoff_ms * 2U;
			if (f->attempts % CFSM_RECOVERY_EVERY == 0U) {
				go(f, CS_RECOVERY);
			} else {
				go(f, CS_WIFI);
			}
		}
		break;

	case CS_RECOVERY:
		f->reconnect_req = TRUE;		/* consumed by the bridge */
		go(f, CS_WIFI);
		break;

	default:					/* CS_DISABLED */
		if (config_ok) {
			go(f, CS_WIFI);
		}
		break;
	}
}

EXPORT UB cfsm_availability(const COMM_FSM *f)
{
	if (f->state == CS_ONLINE) {
		return CAV_ONLINE;
	}
	return (f->was_online && f->state != CS_DISABLED) ? CAV_RECOVERING
							  : CAV_DISCONNECTED;
}

EXPORT const char *cfsm_name(UB state)
{
	switch (state) {
	case CS_INIT:		return "INIT";
	case CS_WIFI:		return "CONNECTING_WIFI";
	case CS_MQTT:		return "CONNECTING_MQTT";
	case CS_SUBSCRIBE:	return "SUBSCRIBING";
	case CS_ONLINE:		return "ONLINE";
	case CS_RETRY:		return "RETRY_WAIT";
	case CS_RECOVERY:	return "RECOVERY";
	default:		return "DISABLED";
	}
}

EXPORT UW s1_comm_pack(const S1_COMM_STATUS *s)
{
	UW	att = (s->attempts > 15U) ? 15U : s->attempts;
	UW	rec = (s->reconnects > 15U) ? 15U : s->reconnects;

	return ((UW)s->state & 0x7UL) | (((UW)s->avail & 0x3UL) << 3)
	     | (((UW)s->link & 0x3UL) << 5) | (((UW)s->session & 0x3UL) << 7)
	     | (att << 9) | (rec << 13);
}

EXPORT void s1_comm_unpack(UW w, S1_COMM_STATUS *s)
{
	s->state = (UB)(w & 0x7UL);
	s->avail = (UB)((w >> 3) & 0x3UL);
	s->link = (UB)((w >> 5) & 0x3UL);
	s->session = (UB)((w >> 7) & 0x3UL);
	s->attempts = (UB)((w >> 9) & 0xFUL);
	s->reconnects = (UB)((w >> 13) & 0xFUL);
	s->tlm_dropped = 0;
	s->rx_rejected = 0;
}
