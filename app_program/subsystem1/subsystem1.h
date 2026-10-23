/*
 *	subsystem1.h
 *	Subsystem 1 -- WiFi / MQTT communication, commands and telemetry
 *	(Buddy 1, student ID 2501573).
 *
 *	Tasks (Week 6 task table; Tribunal 1: keep the two tasks separate):
 *	  WiFi & MQTT Communication  prio 20, core 0, 100 ms + events (Medium)
 *	  Telemetry Collection       prio 21, core 0, 500 ms         (Low)
 *	Both run beside the kernel's network service on core 0 and never
 *	block: local motion and safety never depend on the network (NFR9).
 *
 *	Interfaces:
 *	  mbf_telemetry   Collection -> Communication, encoded frames,
 *	                  S1_TLM_QUEUE_LEN deep, overflow drops the OLDEST
 *	  mbf_mqtt_tx     Communication -> MQTT client (bridge, TMO_POL)
 *	  mbf_mqtt_rx     MQTT client (bridge, TMO_POL) -> Communication
 *	  validated supervisory commands -> Buddy 2 (MSRC_SUPER commands)
 *	  reads: every subsystem's packed status words (telemetry)
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"
#include "../subsystem2/subsystem2.h"
#include "bitpack.h"
#include "mqtt_proto.h"
#include "comm_fsm.h"
#include "mqtt_bridge.h"

#define S1_PRI_COMM		20
#define S1_PRI_TLM		21
#define S1_COMM_PERIOD_MS	100
#define S1_TLM_PERIOD_MS	500	/* Week 6: 500-1000 ms */
#define S1_HB_PERIOD_MS		1000

#define S1_TLM_QUEUE_LEN	4	/* frames buffered while offline     */
#define S1_TX_QUEUE_LEN		8	/* frames waiting for the MQTT client */
#define S1_RX_QUEUE_LEN		4	/* inbound command frames            */
#define S1_MSG_MAX		(1 + S1_MQTT_FRAME_MAX)	/* kind byte + frame */

#define S1_CMD_RATE_MAX		5	/* commands accepted per second      */
#define S1_CMD_WAIT_MS		300	/* wait for Buddy 2's answer         */
#define S1_DUP_WINDOW		4	/* remembered command ids            */

/* Print every Nth telemetry frame as hex on the console (demo without
   the MQTT patch); 0 = off */
#ifndef S1_CONSOLE_TELEMETRY
#define S1_CONSOLE_TELEMETRY	0
#endif

/* flg_comm bits */
#define COMM_EV_TICK		0x01U
#define COMM_EV_RX		0x02U

IMPORT ID	mbf_telemetry;
IMPORT ID	mbf_mqtt_tx;
IMPORT ID	mbf_mqtt_rx;
IMPORT ID	flg_comm;
IMPORT ID	cyc_comm;
IMPORT ID	cyc_tlm;
IMPORT ID	tid_comm;
IMPORT ID	tid_tlm;

/* Communication status (writer: communication task), see comm_fsm.h */
IMPORT void s1_get_comm_status(S1_COMM_STATUS *s);

/* Private to subsystem 1 */
IMPORT volatile UW	s1_comm_word;		/* writer: comm task      */
IMPORT volatile UW	s1_tlm_dropped;		/* writer: telemetry task */
IMPORT volatile UW	s1_rx_rejected;		/* writer: comm task      */
IMPORT volatile UW	s1_session;		/* writer: MQTT client (bridge) */
IMPORT volatile UW	s1_reconnect_req;	/* comm task sets, bridge takes */
IMPORT volatile UW	s1_status_now;		/* comm task sets, telemetry takes */
IMPORT TASK_STAT	s1_stat_tlm;
IMPORT ER   s1_queue_put_drop_oldest(ID mbf, const UB *msg, INT len, volatile UW *dropped);
IMPORT void comm_task(INT stacd, void *exinf);
IMPORT void comm_cyclic_handler(void *exinf);
IMPORT void telemetry_task(INT stacd, void *exinf);
IMPORT void tlm_cyclic_handler(void *exinf);

#endif /* SUBSYSTEM1_H */
