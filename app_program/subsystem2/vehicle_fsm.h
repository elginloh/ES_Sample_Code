/*
 *	vehicle_fsm.h
 *	Vehicle operating-mode state machine (Week 6 system-level statechart,
 *	section 5), owned by Buddy 2 (Tribunal 1, Version B).  Runs inside the
 *	Motion Control task; there is deliberately no separate Vehicle
 *	Controller task (Week 6 section 4).  Pure logic.
 *
 *	    INIT ------ IR + IMU calibrated --------------> NORMAL
 *	    INIT ------ init timeout / calibration fail ---> SAFE_STOP
 *	    NORMAL ---- barcode command starts ------------> NAV_ACTION
 *	    NAV_ACTION  command done/aborted --------------> NORMAL
 *	    NORMAL/NAV  obstacle detected (Buddy 5) -------> OBSTACLE
 *	    OBSTACLE -- avoidance done + line reacquired --> NORMAL
 *	    OBSTACLE -- recovery failed -------------------> SAFE_STOP
 *	    any ------- critical fault bit ----------------> SAFE_STOP
 *	    SAFE_STOP - validated supervisory RESUME ------> INIT
 *
 *	The epoch (3 bits) increments on entering INIT, OBSTACLE and
 *	SAFE_STOP; commands built under an older epoch are rejected as stale.
 */

#ifndef VEHICLE_FSM_H
#define VEHICLE_FSM_H

#include <tk/typedef.h>
#include "sys_defs.h"

typedef struct {
	UB	mode;		/* VM_* */
	UB	epoch;		/* 0..7 */
	BOOL	ir_ok;		/* IR calibrated in this INIT epoch */
	BOOL	imu_ok;		/* IMU calibrated in this INIT epoch */
	BOOL	line_lost;	/* last line event was LOST */
	UW	in_mode_ms;	/* time spent in the current mode */
} VFSM;

typedef struct {
	UW	events;		/* SYS_EV_* consumed this cycle */
	UW	faults;		/* shared fault bitmask */
	UW	crit_mask;	/* critical fault bits */
	BOOL	armed;		/* motors allowed (FALSE in test builds) */
	BOOL	nav_started;	/* barcode command began */
	BOOL	nav_finished;	/* barcode command ended */
	BOOL	resume;		/* validated RESUME accepted */
	UW	dt_ms;
} VFSM_IN;

typedef struct {
	BOOL	changed;
	UB	old_mode;
	UW	flt_set;	/* faults raised by the FSM itself */
} VFSM_OUT;

IMPORT void vfsm_init(VFSM *f);
IMPORT void vfsm_step(VFSM *f, const VFSM_IN *in, VFSM_OUT *out);

#endif /* VEHICLE_FSM_H */
