/*
 *	mqtt_bridge.h
 *	Hand-over points between Buddy 1 and the kernel's MQTT client.
 *
 *	TEAM.md section 3: "No student task may call cyw43_..., lwIP or MQTT
 *	functions ... A 'cloud / reporting' subsystem would add a kernel
 *	object (e.g. a message buffer) that application tasks fill. The MQTT
 *	code then reads it with TMO_POL (never blocking) and publishes it."
 *	These four functions are exactly that: they are meant to be called
 *	ONLY from cyw43_utk_task (lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c)
 *	and never block.  mqtt_utk.c calls them since commit 05a675c (a
 *	change to a protected, team-shared file, recorded in
 *	subsystem1/docs/MQTT_INTEGRATION_PATCH.md).
 *
 *	Plain <stdint.h> types on purpose: mqtt_utk.c lives in the lwIP /
 *	pico-sdk namespace and avoids including tk/tkernel.h.
 */

#ifndef MQTT_BRIDGE_H
#define MQTT_BRIDGE_H

#include <stdint.h>

/* Topics (UTF-8).  Binary payloads, protocol v1 (MQTT_BINARY_PROTOCOL.md) */
#define S1_TOPIC_TLM		"csc14/tlm"	/* robot -> server: telemetry   */
#define S1_TOPIC_HB		"csc14/hb"	/* robot -> server: heartbeat   */
#define S1_TOPIC_EVT		"csc14/evt"	/* robot -> server: events      */
#define S1_TOPIC_ACK		"csc14/ack"	/* robot -> server: command ack */
#define S1_TOPIC_CMD		"csc14/cmd"	/* server -> robot: commands    */

#define S1_MQTT_FRAME_MAX	64

/* Session states reported by the MQTT client */
#define S1_SESSION_NONE		0
#define S1_SESSION_CONNECTED	1
#define S1_SESSION_SUBSCRIBED	2
#define S1_SESSION_LOST		3

/* Next frame to publish: copies it to buf and sets *topic.  Returns the
   frame length, 0 when nothing is queued, <0 on error. */
int32_t s1_mqtt_take_tx(uint8_t *buf, int32_t cap, const char **topic);

/* A message arrived on a subscribed topic.  Returns 0 if accepted, <0 if
   dropped (wrong topic, too long, queue full).  Never blocks. */
int32_t s1_mqtt_put_rx(const char *topic, const uint8_t *data, int32_t len);

/* Broker session state changed (S1_SESSION_*) */
void s1_mqtt_session(int32_t state);

/* 1 once when Buddy 1's recovery state asks for a reconnect, else 0 */
int32_t s1_mqtt_reconnect_requested(void);

#endif /* MQTT_BRIDGE_H */
