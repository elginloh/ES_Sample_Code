/*
 *	line_follow.h
 *	IR line following (Buddy 3).  Pure logic, host-testable.
 *
 *	Hardware (Week 6 pin table, Project Briefing p.6): three DIGITAL IR
 *	sensors (D0 outputs; A0 unused): IR_L GP2, IR_C GP6 and IR_R GP27.
 *	Only IR_C and IR_L steer.  IR_R reads the barcodes beside the line
 *	and is ignored here, except that a crossing line covering all three
 *	sensors is a junction (the barcode task counts them).  With
 *	IR_SIDES_MIRRORED (drv_ir.h) IR_R is on the car's LEFT and IR_L on
 *	its right; these labels and the steering sign are the line
 *	follower's own, task_line.c mirrors them for the motors.
 *
 *	Line position (black = 1, IR_R not used):
 *	    C=1 L=0      centred                 error  0
 *	    C=1 L=1      line drifting to IR_L   error -1
 *	    C=0 L=1      line under IR_L only    error -2: pivot towards IR_L
 *	                 until the centre has it (held through all-white)
 *	    C=0 L=0      line lost: after LF_LOST_WAIT_MS straight (a flicker
 *	                 or a gap), turn towards IR_R (the car's left) until C
 *	                 or L sees black again.  First an arc while moving on
 *	                 (a curve), after LF_LOST_ARC_MM / _MS a pivot on the
 *	                 spot (a sharp corner).  Once found, keep turning that
 *	                 way gently for LF_REALIGN_MS: the curve goes on.
 *	    L=C=R=1      junction (crossing line)
 *	Steering = PD on the error, + = turn right (motion_msg.h convention).
 *
 *	Week 6 statechart 6.3 "Error / recover": the turn towards IR_R starts
 *	after the wait; after LF_LOST_CONFIRM_MM / _MS without the line LF_EV_LOST
 *	is raised and the state becomes LS_SEARCHING, still turning the same
 *	way.  The search never latches a safe stop (LF_EV_RECOVERY_FAIL is
 *	kept for compatibility but no longer raised).
 */

#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <tk/typedef.h>

#define IR_L		0U	/* bit index in a 3-bit IR state */
#define IR_C		1U
#define IR_R		2U
#define IR_BIT(i)	(1U << (i))

/* Line position (3 bits) */
#define LP_CENTER	0U
#define LP_LEFT_NEAR	1U
#define LP_LEFT_FAR	2U
#define LP_RIGHT	3U	/* no longer produced (IR_R does not steer) */
#define LP_LOST_LEFT	4U	/* no longer produced */
#define LP_LOST_RIGHT	5U	/* line lost: turning towards IR_R */
#define LP_JUNCTION	6U
#define LP_RIGHT_NEAR	7U	/* no longer produced */

/* Line state (2 bits) */
#define LS_ON_LINE	0U
#define LS_SEARCHING	1U	/* Error/recover: lost-line turn running */
#define LS_LOST		2U	/* lost outside Normal Navigation       */
#define LS_UNKNOWN	3U	/* not calibrated yet                   */

/* Events returned by lf_step() */
#define LF_EV_LOST		0x01U
#define LF_EV_FOUND		0x02U
#define LF_EV_JUNCTION		0x04U
#define LF_EV_RECOVERY_FAIL	0x08U

/* Calibration results */
#define LF_CAL_OK		0
#define LF_CAL_AMBIGUOUS	(-1)	/* L and C read the same level      */
#define LF_CAL_UNSTABLE		(-2)	/* readings flickered while still   */

/* Tuning [CAL] */
#define LF_KP			18	/* steer units per error step (was 12:
					   too gentle for the ~20 cm curves) */
#define LF_KD			6	/* (was 8: smaller kick at turn start) */
#define LF_LOST_CONFIRM_MM	40	/* lost after this much travel ... (was 25) */
#define LF_LOST_CONFIRM_MS	250	/* ... or this long without line (was 300, 400) */
#define LF_JUNCTION_MM		8	/* all-black run to call a junction */
#define LF_LOST_DIR		1	/* centre and IR_L white: turn towards
					   IR_R (+1), the car's left with
					   IR_SIDES_MIRRORED */
#define LF_LOST_WAIT_MS		200	/* ... first holding straight this
					   long ... */
#define LF_LOST_WAIT_SPEED	8	/* ... at 8/15 of cruise (0 = stand
					   still while waiting), then ... */
#define LF_LOST_ARC_STEER	20	/* ... arcing at this steer ... */
#define LF_LOST_ARC_SPEED	8	/* ... and 8/15 of cruise (inner wheel
					   ~13 %, outer ~93 % of cruise) ... */
#define LF_LOST_ARC_MM		40	/* ... for this much travel ... */
#define LF_LOST_ARC_MS		400	/* ... or this long after the wait,
					   then ... */
#define LF_LOST_PIVOT_STEER	20	/* ... pivoting on the spot at this
					   steer (as LF_SIDE_PIVOT_STEER) */
#define LF_REALIGN_MS		400	/* found by that turn: keep turning the
					   same way this long ... */
#define LF_REALIGN_STEER	14	/* ... this hard, while the centre alone
					   has the line ... */
#define LF_REALIGN_SPEED	8	/* ... at 8/15 of cruise */
#define LF_SIDE_PIVOT_STEER	20	/* only IR_L on the line: stop and pivot
					   towards it at this steer (~47 mm/s
					   per wheel at the 117 mm/s cruise)
					   until the centre is back on it, so the
					   car never drives off the path.  The
					   pivot is held even when IR_L goes white
					   again (the line passes between the
					   sensors) instead of turning back */
#define LF_SIDE_PIVOT_MAX_MS	9000	/* ~a full turn at ~40 deg/s: the pivot
					   always finishes first; only if the
					   centre never finds the line does the
					   lost-line turn take over (was 1500) */
#define LF_SETTLE_MS		300	/* after a side pivot: drive on ...   */
#define LF_SETTLE_SPEED		8	/* ... at 8/15 of cruise this long    */

typedef struct {
	UB	black_level[3];	/* raw level meaning "black", per sensor */
	BOOL	calibrated;
	UB	state;		/* LS_* */
	UB	pos;		/* LP_* */
	INT	err;		/* last error, -2..3 */
	INT	prev_err;
	UW	lost_ms;	/* time without the line */
	W	lost_start_mm;
	BOOL	lost_pending;
	UW	settle_ms;	/* half-speed time left after a pivot */
	BOOL	turning;	/* lost-line turn under way ... */
	UW	turn_ms;	/* ... this long, wait included ... */
	W	turn_start_mm;	/* ... arcing since this odometer reading */
	UW	realign_ms;	/* turn-on time left after it found the line */
	INT	pivot_dir;	/* side pivot under way: -1 towards IR_L */
	UW	pivot_ms;	/* time since the centre last saw the line */
	BOOL	in_junction;
	W	junction_start_mm;
} LINE_FOLLOW;

typedef struct {
	INT	steer;		/* -63..63 */
	UB	speed_frac;	/* 0..15 of cruise */
	UB	events;		/* LF_EV_* */
	BOOL	steering;	/* TRUE: publish steer */
} LF_OUT;

IMPORT void lf_init(LINE_FOLLOW *lf);
IMPORT void lf_reset_tracking(LINE_FOLLOW *lf);

/* Calibration with the car on the line: CENTRE over black, LEFT and
   RIGHT over white.  raw[i] holds 3-bit raw samples. */
IMPORT INT  lf_calibrate(LINE_FOLLOW *lf, const UB *raw, UINT n);

/* Raw 3-bit levels -> black bits (1 = black) */
IMPORT UB   lf_black_bits(const LINE_FOLLOW *lf, UB raw);

/* One period.  'navigating' = Normal Navigation (steering allowed and
   the search is ours); otherwise only tracking/events are produced. */
IMPORT void lf_step(LINE_FOLLOW *lf, UB black, W odo_mm, UW dt_ms,
		    BOOL navigating, LF_OUT *out);

#endif /* LINE_FOLLOW_H */
