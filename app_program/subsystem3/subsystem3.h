/*
 *	subsystem3.h
 *	Subsystem 3 -- IR line following and barcode decoding (Buddy 3,
 *	student ID 2501720).
 *
 *	Tasks (Week 6 task table):
 *	  Line Following   prio 14, core 1, 20 ms cyclic   (High)
 *	  Barcode Decoding prio 17, edge events            (Medium)
 *
 *	Interfaces provided:
 *	  IF-L1  line status word + timestamp   s3_get_line_status()
 *	  IF-L2  line events for Buddy 5        flg_line_evt (LEV_* bits; the
 *	         consumer clears the bits it waits for with TWF_BITCLR)
 *	  IF-L3  barcode status word            s3_get_barcode_status()
 *	Interfaces used:
 *	  Buddy 2 steering word (Normal Navigation only), BARCODE-source
 *	  motion commands, sys_post_event(IR_CAL_*, LINE_LOST/FOUND), faults.
 *	Word layouts: s3_status.h.
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include "../subsystem2/subsystem2.h"
#include "line_follow.h"
#include "barcode_dec.h"
#include "nav_map.h"
#include "s3_status.h"

#define S3_PRI_LINE		14
#define S3_PRI_BARCODE		17
#define S3_PERIOD_MS		20	/* Week 6: 20-50 ms; one fixed value */
#define S3_CAL_SAMPLES		10	/* 200 ms of samples while still     */
#define S3_BC_POLL_MS		20	/* completion check while capturing  */
#define S3_NAV_WAIT_MS		15000	/* longest barcode manoeuvre          */

/* Statechart 6.3 "Barcode / junction detected: pause line following":
   1 = hold straight while a barcode is being captured, but only while
   the centre sensor still sees the line (otherwise steer normally). */
#ifndef S3_PAUSE_STEER_ON_BARCODE
#define S3_PAUSE_STEER_ON_BARCODE	1
#endif

/* TEAM DECISION: 0 = execute the decoded command at once (Week 6
   statechart order decode -> execute); 1 = keep it until the next
   junction (LF_EV_JUNCTION), discarded after S3_NAV_PENDING_MM. */
#ifndef S3_NAV_AT_JUNCTION
#define S3_NAV_AT_JUNCTION	0
#endif
#define S3_NAV_PENDING_MM	800

/* IF-L2: line event flag bits (owned by subsystem 3) */
#define LEV_FOUND	0x01U	/* line reacquired (LOST -> ON_LINE) */
#define LEV_LOST	0x02U
#define LEV_JUNCTION	0x04U
IMPORT ID	flg_line_evt;

IMPORT void s3_get_line_status(S3_LINE_STATUS *s);
IMPORT void s3_get_barcode_status(S3_BC_STATUS *s);

/* Private to subsystem 3 */
IMPORT ID		tid_line;
IMPORT ID		tid_barcode;
IMPORT ID		cyc_line;
IMPORT TASK_STAT	s3_stat_line;
IMPORT volatile UW	s3_line_word;		/* writer: line task    */
IMPORT volatile UW	s3_line_time;		/* writer: line task    */
IMPORT volatile UW	s3_bc_word;		/* writer: barcode task */
IMPORT volatile UW	s3_black_level;		/* writer: line task    */
IMPORT APP_SEQ		s3_line_seq;		/* writer: line task    */
IMPORT void line_task(INT stacd, void *exinf);
IMPORT void line_cyclic_handler(void *exinf);
IMPORT void barcode_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM3_H */
