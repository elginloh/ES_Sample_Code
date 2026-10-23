/*
 *	comm_fsm.h
 *	WiFi / MQTT connection state machine (Buddy 1, Week 6 statechart 6.1).
 *	Pure logic, host-testable.  It only DECIDES; the radio and the MQTT
 *	client belong to the kernel's cyw43_utk_task (TEAM.md section 3).
 *
 *	    INIT --config valid--> CONNECTING_WIFI --link up--> CONNECTING_MQTT
 *	    CONNECTING_MQTT --broker connected--> SUBSCRIBING --subscribed--> ONLINE
 *	    any connecting state --timeout / failure--> RETRY_WAIT
 *	    ONLINE --link or session lost--> RETRY_WAIT
 *	    RETRY_WAIT --backoff elapsed--> CONNECTING_WIFI, every
 *	        CFSM_RECOVERY_EVERY attempts via RECOVERY (ask the MQTT client
 *	        to restart its connection)
 *	    INIT --no network stack / no MQTT client built--> DISABLED
 *
 *	NFR10: retries are timed and non-blocking with a capped delay
 *	(1, 2, 4, 8, 16 s ...).  NFR9: loss of network is reported with
 *	DEGRADED fault bits only (SYS_FLT_WIFI_LOSS / SYS_FLT_MQTT_LOSS).
 */

#ifndef COMM_FSM_H
#define COMM_FSM_H

#include <tk/typedef.h>

#define CS_INIT			0U
#define CS_WIFI			1U	/* connecting WiFi        */
#define CS_MQTT			2U	/* connecting MQTT broker */
#define CS_SUBSCRIBE		3U	/* subscribing / publishing */
#define CS_ONLINE		4U	/* device communicates    */
#define CS_RETRY		5U	/* timed retry, capped    */
#define CS_RECOVERY		6U	/* restart WiFi / broker  */
#define CS_DISABLED		7U	/* not built / no config  */

/* Link (from cyw43_utk_get_status) */
#define CL_DOWN			0U
#define CL_JOINING		1U
#define CL_UP			2U
#define CL_FAILED		3U

/* Session (from the MQTT client bridge) */
#define CSS_NONE		0U
#define CSS_CONNECTED		1U
#define CSS_SUBSCRIBED		2U
#define CSS_LOST		3U

#define CFSM_WIFI_TIMEOUT_MS	15000
#define CFSM_MQTT_TIMEOUT_MS	10000
#define CFSM_SUB_TIMEOUT_MS	5000
#define CFSM_BACKOFF_MIN_MS	1000
#define CFSM_BACKOFF_MAX_MS	16000
#define CFSM_RECOVERY_EVERY	3

/* Heartbeat-level availability (FR13) */
#define CAV_DISCONNECTED	0U
#define CAV_ONLINE		1U
#define CAV_RECOVERING		2U

typedef struct {
	UB	state;
	UW	in_state_ms;
	UW	backoff_ms;
	UB	attempts;	/* saturating */
	UB	reconnects;	/* successful re-connections, saturating */
	BOOL	was_online;
	BOOL	wifi_loss;	/* fault conditions currently raised */
	BOOL	mqtt_loss;
	BOOL	reconnect_req;	/* one-shot request to the MQTT client */
} COMM_FSM;

/* Communication status word (writer: communication task), 17 bits:
     state 3 | availability 2 | link 2 | session 2 | attempts 4 | reconnects 4
   The two drop/reject counters are separate single-writer words,
   saturated to 8 bits when reported. */
typedef struct {
	UB	state;		/* CS_*  */
	UB	avail;		/* CAV_* */
	UB	link;		/* CL_*  */
	UB	session;	/* CSS_* */
	UB	attempts;	/* 0..15 saturating */
	UB	reconnects;	/* 0..15 saturating */
	UB	tlm_dropped;	/* 0..255 saturating (separate word) */
	UB	rx_rejected;	/* 0..255 saturating (separate word) */
} S1_COMM_STATUS;

IMPORT UW   s1_comm_pack(const S1_COMM_STATUS *s);
IMPORT void s1_comm_unpack(UW w, S1_COMM_STATUS *s);

IMPORT void cfsm_init(COMM_FSM *f);
IMPORT void cfsm_step(COMM_FSM *f, BOOL config_ok, UB link, UB session, UW dt_ms);
IMPORT UB   cfsm_availability(const COMM_FSM *f);
IMPORT const char *cfsm_name(UB state);

#endif /* COMM_FSM_H */
