/*
 *	line_follow.h
 *	IR line following (Buddy 3).  Pure logic, host-testable.
 *
 *	Hardware (Week 6 pin table, Project Briefing p.6): three DIGITAL IR
 *	sensors (D0 outputs; A0 unused): LEFT GP2 and CENTRE GP6 track the
 *	18 mm line, RIGHT GP27 reads the barcodes 20 mm to the right of the
 *	line.  IR_RIGHT is not used for steering (Tribunal 1 shared-resource
 *	row "decide IR_RIGHT's role"): it only contributes to junction
 *	detection, because a crossing line covers all three sensors while a
 *	barcode bar covers only the right one.
 *
 *	Line position from LEFT/CENTRE (black = 1):
 *	    C=1 L=0  centred                error  0
 *	    C=1 L=1  line drifting left     error -1
 *	    C=0 L=1  line under left only   error -2
 *	    C=0 L=0  line lost; side inferred from history (+2 right of centre,
 *	             -3 beyond the left sensor, +3 beyond the right)
 *	    L=C=R=1  junction (crossing line)
 *	Steering = PD on the error, + = turn right (motion_msg.h convention).
 *
 *	Week 6 statechart 6.3 "Error / recover": when the line is lost the
 *	car pivots on the spot (no reversing) to LF_SEARCH_DEG towards the
 *	side it was last seen, then to LF_SEARCH_DEG on the other side, and
 *	keeps sweeping edge to edge until the line is found.  The angle is
 *	measured with the IMU heading when one is given, else estimated from
 *	time.  The search never latches a safe stop (LF_EV_RECOVERY_FAIL is
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
#define LP_RIGHT	3U	/* inferred: line moved right of centre */
#define LP_LOST_LEFT	4U
#define LP_LOST_RIGHT	5U
#define LP_JUNCTION	6U

/* Line state (2 bits) */
#define LS_ON_LINE	0U
#define LS_SEARCHING	1U	/* Error/recover: pivot search running */
#define LS_LOST		2U	/* search failed (Stop / safe state)    */
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
#define LF_KP_PIVOT		12	/* |err| 3 spot pivot keeps the old
					   strength (stronger spun past the line) */
#define LF_KD			6	/* (was 8: smaller kick at turn start) */
#define LF_LOST_CONFIRM_MM	40	/* lost after this much travel ... (was 25) */
#define LF_LOST_CONFIRM_MS	250	/* ... or this long without line (was 300, 400) */
#define LF_GAP_HOLD_MS		150	/* centre lost, nothing on L or R:
					   keep straight this long before
					   assuming the line went right */
#define LF_JUNCTION_MM		8	/* all-black run to call a junction */
#define LF_SEARCH_PIVOT		63	/* steer while pivot-searching (was 40;
					   STEER_MAX, so the pivot still reaches
					   creep speed at a 20 mm/s cruise) */
#define LF_SEARCH_DEG		90	/* sweep this far each side of the
					   heading where the line was lost */
#define LF_SEARCH_USE_IMU	1	/* 1 = measure the sweep with the IMU
					   heading (Buddy 4, s4_get_imu_status);
					   0 = time only (LF_SEARCH_T1_MS).  The
					   LSM303DLHC heading is magnetic: set 0
					   if the motors bend it (sweeps far off
					   90 deg in the "line: sweep" log) */
#define LF_HEADING_NONE		(-1)	/* lf_step(): no usable heading */
#define LF_SEARCH_T1_MS		4600	/* first sweep, towards last side, when
					   timed: ~90 deg.  The pivot runs at
					   creep speed (22 mm/s per wheel, TRACK
					   130 mm -> 19 deg/s); later sweeps take
					   2 x T1 for the full 180 deg.  With a
					   heading, 1.5 x the timed length is the
					   safety limit of a sweep (was 3100 for
					   60 deg) */
#define LF_REALIGN_MS		500	/* after a sweep finds the line: keep */
#define LF_REALIGN_STEER	14	/* arcing the sweep way this hard ... */
#define LF_REALIGN_SPEED	8	/* ... at 8/15 of cruise, to line up  */
#define LF_TURN_MIN_STEER	10	/* |steer| that counts as "turning"   */
#define LF_TURN_MEMORY_MS	400	/* remember the last turn this long:
					   steering right when the centre lost
					   the line -> look LEFT first, and
					   vice versa */
#define LF_BACKUP_MM		0	/* line lost: first reverse straight back
					   this far before sweeping; 0 = never
					   reverse, sweep at once (was 100: the
					   odometer counts down while reversing,
					   and the old check never saw 100 mm, so
					   the car reversed until it crossed the
					   line again) */
#define LF_SEARCH_RETRIES	2	/* sweep pairs before giving up     */

typedef struct {
	UB	black_level[3];	/* raw level meaning "black", per sensor */
	BOOL	calibrated;
	UB	state;		/* LS_* */
	UB	pos;		/* LP_* */
	INT	err;		/* last error, -3..3 */
	INT	prev_err;
	INT	last_side;	/* -1 left, +1 right, 0 centre */
	UW	lost_ms;	/* time without the line */
	W	lost_start_mm;
	BOOL	lost_pending;
	UW	gap_ms;		/* straight hold after losing the centre */
	UW	realign_ms;	/* turn-in time left after a sweep found it */
	INT	realign_dir;	/* -1 left, +1 right */
	INT	turn_dir;	/* recent steering on the line, -1/0/+1 */
	BOOL	backing;	/* search phase 0: reversing */
	W	backup_start_mm;
	UW	turn_ms;	/* time since that steering */
	UW	search_ms;	/* time in the current sweep */
	INT	search_origin;	/* heading when the search began, deg, or
				   LF_HEADING_NONE: sweeps are timed */
	INT	search_deg;	/* angle from there, + = right (measured) */
	UB	search_phase;	/* 0 = towards last side, 1 = other side */
	UB	retries;
	BOOL	in_junction;
	W	junction_start_mm;
} LINE_FOLLOW;

typedef struct {
	INT	steer;		/* -63..63 */
	UB	speed_frac;	/* 0..15 of cruise */
	UB	events;		/* LF_EV_* */
	BOOL	steering;	/* TRUE: publish steer */
	BOOL	reverse;	/* TRUE: drive straight back (speed_frac) */
} LF_OUT;

IMPORT void lf_init(LINE_FOLLOW *lf);
IMPORT void lf_reset_tracking(LINE_FOLLOW *lf);

/* Calibration with the car on the line: CENTRE over black, LEFT and
   RIGHT over white.  raw[i] holds 3-bit raw samples. */
IMPORT INT  lf_calibrate(LINE_FOLLOW *lf, const UB *raw, UINT n);

/* Raw 3-bit levels -> black bits (1 = black) */
IMPORT UB   lf_black_bits(const LINE_FOLLOW *lf, UB raw);

/* One period.  'navigating' = Normal Navigation (steering allowed and
   the search is ours); otherwise only tracking/events are produced.
   'heading_deg' = IMU heading 0..359, + = left (Buddy 4 convention), or
   LF_HEADING_NONE; it only measures the search sweeps. */
IMPORT void lf_step(LINE_FOLLOW *lf, UB black, W odo_mm, INT heading_deg,
		    UW dt_ms, BOOL navigating, LF_OUT *out);

#endif /* LINE_FOLLOW_H */
