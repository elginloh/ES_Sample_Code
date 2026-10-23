/*
 *	mqtt_bridge.c
 *	Non-blocking hand-over to/from the kernel's MQTT client (see
 *	mqtt_bridge.h) and the bounded-queue helper of subsystem 1.
 *
 *	The s1_mqtt_* functions run in cyw43_utk_task once mqtt_utk.c calls
 *	them (docs/MQTT_INTEGRATION_PATCH.md); everything here uses TMO_POL
 *	or single-word atomics, so the network service can never be blocked
 *	by Buddy 1 and Buddy 1 can never be blocked by the network.
 */

#include "subsystem1.h"

EXPORT volatile UW	s1_session = (UW)S1_SESSION_NONE;
EXPORT volatile UW	s1_reconnect_req;

/* bounded queue, overflow policy "drop the oldest" (Tribunal 1: queue
   capacity and overflow handling must be defined) */
EXPORT ER s1_queue_put_drop_oldest(ID mbf, const UB *msg, INT len, volatile UW *dropped)
{
	UB	tmp[S1_MSG_MAX];
	UINT	tries;

	for (tries = 0; tries < 4U; tries++) {
		if (tk_snd_mbf(mbf, msg, len, TMO_POL) == E_OK) {
			return E_OK;
		}
		if (tk_rcv_mbf(mbf, tmp, TMO_POL) > 0 && dropped != NULL) {
			*dropped = *dropped + 1U;	/* single writer: the producer */
		}
	}
	return E_QOVR;
}

LOCAL const char *topic_of(UB kind)
{
	switch (kind) {
	case MT_TELEMETRY:	return S1_TOPIC_TLM;
	case MT_HEARTBEAT:	return S1_TOPIC_HB;
	case MT_EVENT:		return S1_TOPIC_EVT;
	case MT_ACK:		return S1_TOPIC_ACK;
	default:		return NULL;
	}
}

LOCAL BOOL same(const char *a, const char *b)
{
	while (*a != '\0' && *a == *b) {
		a++;
		b++;
	}
	return (*a == *b) ? TRUE : FALSE;
}

int32_t s1_mqtt_take_tx(uint8_t *buf, int32_t cap, const char **topic)
{
	UB	msg[S1_MSG_MAX];
	INT	n, i;

	if (mbf_mqtt_tx <= 0 || buf == NULL || topic == NULL) {
		return 0;
	}
	n = tk_rcv_mbf(mbf_mqtt_tx, msg, TMO_POL);
	if (n <= 1) {
		return 0;				/* nothing queued */
	}
	if (n - 1 > (INT)cap || topic_of(msg[0]) == NULL) {
		return -1;
	}
	for (i = 1; i < n; i++) {
		buf[i - 1] = msg[i];
	}
	*topic = topic_of(msg[0]);
	return (int32_t)(n - 1);
}

int32_t s1_mqtt_put_rx(const char *topic, const uint8_t *data, int32_t len)
{
	if (mbf_mqtt_rx <= 0 || topic == NULL || data == NULL) {
		return -1;
	}
	if (!same(topic, S1_TOPIC_CMD)) {
		return -1;				/* not a command topic */
	}
	if (len <= 0 || len > S1_MQTT_FRAME_MAX) {
		return -2;
	}
	if (tk_snd_mbf(mbf_mqtt_rx, data, (INT)len, TMO_POL) != E_OK) {
		return -3;				/* queue full: dropped */
	}
	(void)tk_set_flg(flg_comm, COMM_EV_RX);
	return 0;
}

void s1_mqtt_session(int32_t state)
{
	app_store(&s1_session, (UW)state & 0x3U);
}

int32_t s1_mqtt_reconnect_requested(void)
{
	return (app_atomic_xchg(&s1_reconnect_req, 0U) != 0U) ? 1 : 0;
}
