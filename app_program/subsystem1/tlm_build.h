/*
 *	tlm_build.h
 *	Telemetry record and event words built from every subsystem's status
 *	(Buddy 1).  Pure logic, host-testable.
 *
 *	The Telemetry Collection task reads each status through its owner's
 *	interface (motion_get_status(), s3_get_line_status(), ...) into a
 *	TLM_SOURCES snapshot and calls tlm_build() / tlm_events().  The same
 *	functions run in the integration tests, so the mapping of every
 *	Buddy 2-5 field onto the binary protocol is tested end to end,
 *	including the invalid / unknown codes.
 */

#ifndef TLM_BUILD_H
#define TLM_BUILD_H

#include <tk/typedef.h>
#include "mqtt_proto.h"
#include "comm_fsm.h"
#include "../subsystem2/sys_defs.h"
#include "../subsystem2/motion_msg.h"
#include "../subsystem3/s3_status.h"
#include "../subsystem4/s4_status.h"
#include "../subsystem5/s5_msg.h"

/* One consistent-enough snapshot of the other subsystems (each part is
   read atomically by its own getter) */
typedef struct {
	UW		uptime_ms;
	UW		vsw;		/* Buddy 2 vehicle status word */
	UW		faults;		/* shared fault bitmask        */
	MOTION_STATUS	motion;		/* Buddy 2 */
	S3_LINE_STATUS	line;		/* Buddy 3 */
	S3_BC_STATUS	barcode;
	S4_IMU_STATUS	imu;		/* Buddy 4 */
	S4_HUMP_STATUS	hump;
	S5_SCAN_STATUS	scan;		/* Buddy 5 */
	S5_AVOID_STATUS	avoid;
	S5_PROFILE	profile;
	S1_COMM_STATUS	comm;		/* Buddy 1 */
} TLM_SOURCES;

#define TLM_MAX_EVENTS		6	/* one per EVT_* code */

/* obst_width (5 mm steps) and clear_left/right (10 mm steps): 254 =
   at least 254 steps, 255 = unknown (never treated as free) */
#define TLM_UNKNOWN8		255U

IMPORT void tlm_build(TLM_RECORD *r, const TLM_SOURCES *s);

/* Events for what changed since the previous sample: mode, faults,
   barcode read, hump completed, avoidance finished, collision.  Event
   data word layouts: mqtt_proto.h.  Returns the number of events
   written to ev[] (0..TLM_MAX_EVENTS). */
IMPORT UINT tlm_events(const TLM_RECORD *prev, UB prev_avoid_count,
		       const TLM_RECORD *cur, const TLM_SOURCES *s,
		       EVT_RECORD ev[TLM_MAX_EVENTS]);

#endif /* TLM_BUILD_H */
