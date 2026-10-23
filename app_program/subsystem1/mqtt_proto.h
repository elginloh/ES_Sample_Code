/*
 *	mqtt_proto.h
 *	CSC14 binary MQTT protocol, version 1 (Buddy 1).  Pure logic.
 *	Full specification: subsystem1/docs/MQTT_BINARY_PROTOCOL.md.
 *
 *	Frame (all MQTT payloads):
 *	    byte 0      version (high nibble, = 1) | message type (low nibble)
 *	    byte 1      sequence number, modulo 256
 *	    byte 2      payload length L in bytes
 *	    bytes 3..   payload: fields packed LSB-first (bitpack.h), in the
 *	                fixed order of the schema for that type and version
 *	    last 2      CRC-16/CCITT-FALSE over bytes 0 .. 2+L, little-endian
 *	Field identifiers are not repeated per field: the (version, type)
 *	pair identifies the schema, which keeps the payload minimal.
 *
 *	Types: 1 telemetry, 2 heartbeat, 3 event (robot -> server);
 *	       4 command (server -> robot); 5 acknowledgement (robot -> server).
 */

#ifndef MQTT_PROTO_H
#define MQTT_PROTO_H

#include <tk/typedef.h>

#define MP_VERSION		1U
#define MP_HDR_LEN		3U
#define MP_CRC_LEN		2U
#define MP_MAX_FRAME		64U

#define MT_TELEMETRY		1U
#define MT_HEARTBEAT		2U
#define MT_EVENT		3U
#define MT_COMMAND		4U
#define MT_ACK			5U

/* Frame / payload errors (negative) */
#define MPE_SHORT		(-1)	/* fewer bytes than header + CRC  */
#define MPE_CRC			(-2)
#define MPE_VERSION		(-3)
#define MPE_TYPE		(-4)
#define MPE_LENGTH		(-5)	/* length byte != actual / schema */
#define MPE_FIELD		(-6)	/* reserved bits / value out of range */
#define MPE_SPACE		(-7)	/* output buffer too small        */
#define MPE_ARG			(-8)	/* command argument out of range  */

/* ---------------------------------------------------------------------
 *  Telemetry record (version 1 schema, 279 bits -> 35-byte payload).
 *  Fields keep their packed widths; the order here IS the wire order.
 * --------------------------------------------------------------------- */
typedef struct {
	/* system (Buddy 2) */
	UH	uptime_ds;	/* 16  0.1 s, wraps                      */
	UB	mode;		/*  3  VM_*                              */
	UB	epoch;		/*  3                                    */
	UW	faults;		/* 27  SYS_FLT_* bitmask                 */
	/* motion (Buddy 2) */
	UB	mstate;		/*  2  MS_*                              */
	UB	msource;	/*  2  MSRC_*                            */
	UB	cmd_active;	/*  1                                    */
	H	speed_l;	/* 11s mm/s, -1024 = invalid             */
	H	speed_r;	/* 11s                                   */
	UH	odometer_cm;	/* 16                                    */
	UB	heading2;	/*  8  encoder heading / 2 deg           */
	UB	cruise10;	/*  6  cruise / 10 mm/s                  */
	/* line + barcode (Buddy 3) */
	UB	ir_black;	/*  3  L,C,R                             */
	UB	line_pos;	/*  3  LP_*                              */
	UB	line_state;	/*  2  LS_*                              */
	B	steer;		/*  7s                                   */
	UB	bc_state;	/*  2  BCS_*                             */
	UB	bc_symbol;	/*  6  Code 39 index, 63 = none          */
	UB	bc_result;	/*  2  BCR_*                             */
	UB	nav;		/*  2  NAV_*                             */
	UB	nav_valid;	/*  1                                    */
	UB	bc_valid;	/*  6  valid reads, modulo 64            */
	UB	bc_invalid;	/*  6  invalid reads, modulo 64          */
	/* IMU + hump (Buddy 4) */
	UB	imu_status;	/*  2  IMU_ST_*                          */
	B	pitch;		/*  8s deg, -128 = invalid               */
	B	roll;		/*  8s deg                               */
	H	yaw_rate;	/*  9s dps, -256 = invalid               */
	UH	heading;	/*  9  deg, 511 = invalid                */
	UB	motion_class;	/*  2  MCLS_*                            */
	UB	tilt;		/*  1                                    */
	UB	collision;	/*  1                                    */
	UB	shock;		/*  5  0.5 g steps                       */
	UB	hump_last;	/*  6  2 mm steps, 63 = none             */
	UB	hump_max;	/*  6  2 mm steps, 63 = none             */
	UB	hump_count;	/*  4                                    */
	UB	hump_active;	/*  1                                    */
	/* obstacle (Buddy 5) */
	UH	us_dist;	/* 10  5 mm steps, 1022 no echo, 1023 invalid */
	UB	us_result;	/*  2  US_*                              */
	UB	avoid_state;	/*  4  AVST_*                            */
	UB	avoid_side;	/*  2  AV_SIDE_*                         */
	UB	avoid_outcome;	/*  2  AVO_*                             */
	UB	obst_width;	/*  8  5 mm steps, 255 unknown           */
	B	obst_bearing;	/*  8s deg                               */
	UB	clear_left;	/*  8  10 mm steps, 255 unknown          */
	UB	clear_right;	/*  8                                    */
	/* communication (Buddy 1) */
	UB	comm_state;	/*  3  CS_*                              */
	UB	tlm_dropped;	/*  8  saturating                        */
	UB	rx_rejected;	/*  8  saturating                        */
} TLM_RECORD;

#define MP_TLM_BITS		279U
#define MP_TLM_PAYLOAD		35U	/* ceil(279 / 8) */

/* Heartbeat (FR13: online / disconnected / recovering), 32 bits */
typedef struct {
	UH	uptime_ds;	/* 16 */
	UB	mode;		/*  3 */
	UB	comm_state;	/*  3 */
	UB	critical;	/*  1  any critical fault bit set */
	UB	fault_count;	/*  5  number of fault bits set (0..27) */
	UB	reconnects;	/*  4  saturating */
} HB_RECORD;
#define MP_HB_PAYLOAD		4U

/* Events (robot -> server): 4-bit code + one 32-bit data word whose
   layout depends on the code (LSB-first, see the protocol document):
     EVT_MODE       old mode 3 | new mode 3 | epoch 3
     EVT_FAULTS     fault mask 27
     EVT_BARCODE    symbol 6 | result 2 | nav 2 | nav valid 1
     EVT_HUMP       height code 6 | highest code 6 | count 4
     EVT_OBSTACLE   outcome 2 | fail 3 | side 2 | width 8 | distance 10
     EVT_COLLISION  shock 5 (0.5 g steps) */
#define EVT_MODE		1U
#define EVT_FAULTS		2U
#define EVT_BARCODE		3U
#define EVT_HUMP		4U
#define EVT_OBSTACLE		5U
#define EVT_COLLISION		6U
typedef struct {
	UB	code;		/*  4 */
	UW	data;		/* 32 */
} EVT_RECORD;
#define MP_EVT_PAYLOAD		5U	/* fixed 36 bits -> 5 bytes */

/* Commands (server -> robot), 24 bits */
#define CMD_PING		0U
#define CMD_STOP		1U
#define CMD_RESUME		2U
#define CMD_CLEAR		3U
#define CMD_SET_SPEED		4U	/* arg = cruise speed, mm/s */
#define CMD_REQ_STATUS		5U	/* send telemetry now       */
#define CMD_COUNT		6U
typedef struct {
	UB	cmd;		/*  3 */
	UH	arg;		/* 10 */
	UB	cmd_id;		/*  8 */
} CMD_RECORD;
#define MP_CMD_PAYLOAD		3U

/* Acknowledgement result codes (4 bits) */
#define ACK_OK			0U
#define ACK_BAD_VERSION		1U
#define ACK_BAD_TYPE		2U
#define ACK_BAD_LENGTH		3U
#define ACK_BAD_FIELD		4U	/* reserved bits / unknown command */
#define ACK_BAD_ARG		5U
#define ACK_REJ_STATE		6U	/* not allowed in this vehicle mode */
#define ACK_DUPLICATE		7U
#define ACK_RATE		8U	/* too many commands */
#define ACK_BUSY		9U	/* queue full / no answer */
typedef struct {
	UB	cmd_id;		/*  8 */
	UB	cmd;		/*  3 */
	UB	result;		/*  4 */
	UB	detail;		/*  5  e.g. MREJ_* from Buddy 2 */
} ACK_RECORD;
#define MP_ACK_PAYLOAD		3U

typedef struct {
	UB	version;
	UB	type;
	UB	seq;
	UB	len;		/* payload bytes */
} MP_HDR;

/* Check header, length and CRC; returns the payload offset (3) or MPE_* */
IMPORT INT mp_frame_check(const UB *frame, UINT len, MP_HDR *hdr);

IMPORT INT mp_tlm_encode(const TLM_RECORD *r, UB seq, UB *out, UINT cap);
IMPORT INT mp_tlm_decode(const UB *frame, UINT len, TLM_RECORD *r);
IMPORT INT mp_hb_encode(const HB_RECORD *r, UB seq, UB *out, UINT cap);
IMPORT INT mp_hb_decode(const UB *frame, UINT len, HB_RECORD *r);
IMPORT INT mp_evt_encode(const EVT_RECORD *r, UB seq, UB *out, UINT cap);
IMPORT INT mp_evt_decode(const UB *frame, UINT len, EVT_RECORD *r);
IMPORT INT mp_cmd_encode(const CMD_RECORD *r, UB seq, UB *out, UINT cap);
/* Decode AND validate a command frame (NFR11, Tribunal 1 R1) */
IMPORT INT mp_cmd_decode(const UB *frame, UINT len, CMD_RECORD *r);
IMPORT INT mp_ack_encode(const ACK_RECORD *r, UB seq, UB *out, UINT cap);
IMPORT INT mp_ack_decode(const UB *frame, UINT len, ACK_RECORD *r);
IMPORT UB  mp_err_to_ack(INT err);

/* A validated command as Buddy 2's supervisory request (MCMD_* type and
   argument).  FALSE for the commands Buddy 1 answers itself (PING,
   REQ_STATUS). */
IMPORT BOOL mp_cmd_to_motion(const CMD_RECORD *c, UW *mcmd_type, INT *arg);
/* Acknowledgement for Buddy 2's result word: ACK_OK when done, else
   ACK_REJ_STATE with the MREJ_* reason in *detail (ACK_BUSY if the word
   is not a valid result). */
IMPORT UB  mp_ack_of_result(UW mres_word, UB *detail);

/* The same telemetry as the Week 6 JSON design would have sent it; used
   only to MEASURE the message-size reduction (Week 13 evidence). */
IMPORT INT mp_tlm_json(const TLM_RECORD *r, char *out, UINT cap);

#endif /* MQTT_PROTO_H */
