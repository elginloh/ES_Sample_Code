/*
 *	mqtt_utk.c
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Runs entirely on the single processor-1-owned task that already owns
 *	the CYW43439 and the rest of lwIP (docs/RELEASE.md safety rule 6):
 *	cyw43_utk_task() (cyw43_utk.c) calls mqtt_utk_start() once, the first
 *	time it observes CYW43_LINK_UP. mqtt.c's own keepalive/ping/retry
 *	timers, and the subsystem 1 bridge below, both ride the same
 *	sys_check_timeouts() polling that task already does for DHCP and TCP --
 *	nothing extra runs on any other task or core.
 *
 *	Subsystem 1 (app_program/subsystem1/mqtt_bridge.h) fills a transmit
 *	queue and drains a receive queue; this file is the only code that
 *	moves those frames to and from the broker (TEAM.md section 3,
 *	app_program/subsystem1/docs/MQTT_INTEGRATION_PATCH.md).
 *
 *	This file does not include tk/tkernel.h or any pico-sdk/cyw43 header
 *	(unlike cyw43_utk.c and sys_arch.c), so it isn't subject to either
 *	file's size_t workaround -- tm/tmonitor.h alone is namespace-safe here.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include "../../../../app_program/subsystem1/mqtt_bridge.h"	/* stdint only */
#include <tm/tmonitor.h>

#define MQTT_UTK_KEEPALIVE_S    60
#define MQTT_UTK_RETRY_MS       3000
#define MQTT_UTK_BRIDGE_MS      50	/* poll subsystem 1's transmit queue */
#define MQTT_UTK_BRIDGE_BURST   4	/* frames published per poll */

/* Inbound message staging. A publish arrives as one topic callback followed by
 * one or more data callbacks, so the topic has to be kept until the fragment
 * carrying MQTT_DATA_FLAG_LAST shows up. Both buffers are sized well past
 * subsystem 1's largest frame (S1_MQTT_FRAME_MAX); anything longer is
 * truncated, flagged and dropped rather than being allowed to run off the end. */
#define MQTT_UTK_RX_TOPIC_MAX   64
#define MQTT_UTK_RX_DATA_MAX    128

static mqtt_client_t	*client;
static ip_addr_t	broker_addr;
static u8_t		connecting;

static char		rx_topic[MQTT_UTK_RX_TOPIC_MAX];
static UB		rx_data[MQTT_UTK_RX_DATA_MAX];
static uint8_t		rx_raw[MQTT_UTK_RX_DATA_MAX];	/* binary, for subsystem 1 */
static u16_t		rx_len;
static u8_t		rx_truncated;

static void bridge_timeout(void *arg);

/* mqtt.c NUL-terminates the topic before handing it over, but it is only
 * valid for the duration of this call -- copy it out. */
static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
	u32_t	i;

	(void)arg;
	(void)tot_len;

	for(i = 0; i < (MQTT_UTK_RX_TOPIC_MAX - 1) && topic[i] != '\0'; i++) {
		rx_topic[i] = topic[i];
	}
	rx_topic[i] = '\0';

	rx_len = 0;
	rx_truncated = 0;
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
	u16_t	i;

	(void)arg;

	for(i = 0; i < len; i++) {
		if(rx_len < (MQTT_UTK_RX_DATA_MAX - 1)) {
			/* rx_raw keeps the real bytes: command frames are binary.
			 * rx_data is a printable copy for debugging on the
			 * console, where raw bytes would scramble the terminal. */
			UB c = data[i];
			rx_raw[rx_len] = c;
			rx_data[rx_len++] = (c >= 0x20 && c < 0x7f)? c: '.';
		} else {
			rx_truncated = 1;
		}
	}

	if((flags & MQTT_DATA_FLAG_LAST) != 0) {
		rx_data[rx_len] = '\0';
		if(!rx_truncated) {
			(void)s1_mqtt_put_rx(rx_topic, rx_raw, (int32_t)rx_len);
		}
	}
}

static void subscribe_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result == ERR_OK) {
		s1_mqtt_session(S1_SESSION_SUBSCRIBED);
		tm_printf((UB*)"mqtt: subscribed to " S1_TOPIC_CMD "\n");
	} else {
		tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
	}
}

static void publish_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result != ERR_OK) {
		tm_printf((UB*)"mqtt: publish failed, err=%d\n", (int)result);
	}
}

/* Publish what subsystem 1 queued (telemetry, heartbeat, events, acks), each
 * on the topic subsystem 1 chose. Never blocks: an empty queue ends the burst,
 * and a full lwIP output ring loses that frame and leaves the rest for the
 * next poll. */
static void bridge_timeout(void *arg)
{
	uint8_t		f[S1_MQTT_FRAME_MAX];
	const char	*topic;
	int32_t		n;
	int		k;

	(void)arg;

	/* Subsystem 1's RECOVERY state asks for a fresh session. mqtt_disconnect()
	 * makes no connection callback, so report the loss here; the poll loop's
	 * mqtt_utk_start() then reconnects and connection_cb() re-arms this. */
	if(s1_mqtt_reconnect_requested() && client != NULL) {
		mqtt_disconnect(client);
		s1_mqtt_session(S1_SESSION_LOST);
		return;
	}

	for(k = 0; k < MQTT_UTK_BRIDGE_BURST && mqtt_client_is_connected(client); k++) {
		n = s1_mqtt_take_tx(f, (int32_t)sizeof(f), &topic);
		if(n <= 0) {
			break;
		}
		if(mqtt_publish(client, topic, f, (u16_t)n, 0 /* qos */,
				0 /* retain */, publish_done_cb, NULL) != ERR_OK) {
			break;
		}
	}

	/* Re-arms itself every call -- mqtt.c's connection callback (below)
	 * only starts this chain once, on the transition into MQTT_CONNECT_
	 * ACCEPTED, so there is exactly one bridge chain alive at a time. */
	sys_timeout(MQTT_UTK_BRIDGE_MS, bridge_timeout, NULL);
}

static void retry_timeout(void *arg)
{
	(void)arg;
	connecting = 0;
	mqtt_utk_start();
}

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
	(void)c;
	(void)arg;

	if(status == MQTT_CONNECT_ACCEPTED) {
		err_t	err;

		/* No longer *connecting* -- clear the guard so that if this
		 * connection later drops, the poll loop's mqtt_utk_start() call
		 * is allowed to reconnect. Leaving it set would wedge the client
		 * permanently after the first disconnect. */
		connecting = 0;
		s1_mqtt_session(S1_SESSION_CONNECTED);

		tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");

		/* Subscribe to subsystem 1's command topic; incoming_data_cb()
		 * hands every command frame to subsystem 1, which validates it
		 * and answers on its acknowledgement topic. Callbacks must be
		 * registered before subscribing, or the first delivery has
		 * nowhere to go. */
		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);

		err = mqtt_subscribe(client, S1_TOPIC_CMD, 0 /* qos */,
				     subscribe_done_cb, NULL);
		if(err != ERR_OK) {
			tm_printf((UB*)"mqtt: mqtt_subscribe() failed, err=%d\n",
				  (int)err);
		}

		/* Drop any chain left over from a previous connection before
		 * arming a new one. Each reconnect would otherwise add another
		 * self-perpetuating bridge poll, and every one of them holds a
		 * MEMP_SYS_TIMEOUT slot until the pool is exhausted. */
		sys_untimeout(bridge_timeout, NULL);
		sys_timeout(MQTT_UTK_BRIDGE_MS, bridge_timeout, NULL);
	} else {
		s1_mqtt_session(S1_SESSION_LOST);
		/* mqtt_client_connect() reconnects on its own for a dropped
		 * keepalive; a rejected/never-established connection does not
		 * retry itself, so do that here -- on a delay, so a broker
		 * that keeps rejecting (e.g. bad client ID) can't turn this
		 * into a tight reconnect loop. */
		tm_printf((UB*)"mqtt: connection status %d, retrying in %ds\n",
			  (int)status, MQTT_UTK_RETRY_MS / 1000);
		sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
	}
}

void mqtt_utk_start(void)
{
	struct mqtt_connect_client_info_t	client_info = {0};
	err_t					err;

	if(connecting || (client != NULL && mqtt_client_is_connected(client))) {
		return;
	}

	if(client == NULL) {
		client = mqtt_client_new();
		if(client == NULL) {
			tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
			return;
		}
	}

	if(!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
		tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP\n");
		return;
	}
	IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

	client_info.client_id = MQTT_CLIENT_ID;
	client_info.keep_alive = MQTT_UTK_KEEPALIVE_S;

	connecting = 1;
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &client_info);
	if(err != ERR_OK) {
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
		connecting = 0;
	}
}
