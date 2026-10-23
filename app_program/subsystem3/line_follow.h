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
 *	car searches (pivot towards the side it was last seen, then the other
 *	side, bounded retries).  When the retries are exhausted it reports
 *	LF_EV_RECOVERY_FAIL ("Stop / safe state").
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
#define LF_KP			12	/* steer units per error step      */
#define LF_KD			8
#define LF_LOST_CONFIRM_MM	25	/* lost after this much travel ... */
#define LF_LOST_CONFIRM_MS	300	/* ... or this long without line   */
#define LF_JUNCTION_MM		8	/* all-black run to call a junction */
#define LF_SEARCH_PIVOT		40	/* steer while pivot-searching      */
#define LF_SEARCH_T1_MS		900	/* first sweep, towards last side   */
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
	UW	search_ms;	/* time in the current sweep */
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
