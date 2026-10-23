/*
 *	s5_msg.h
 *	Packed inter-task messages and status words of subsystem 5 (pure,
 *	host-testable).  Tribunal 1, Version B: minimum-bit messages between
 *	the three Buddy 5 tasks, verified by pack/unpack round trips, while
 *	keeping "enough distance resolution for safe obstacle avoidance".
 *
 *	Distance code (10 bits) used everywhere in subsystem 5 and in the
 *	telemetry: 5 mm steps, 0..4000 mm -> codes 0..800 (HC-SR04 resolution
 *	is ~3 mm, its accuracy ~1 cm, so 5 mm keeps the measurement);
 *	1022 = no echo (nothing within 4 m), 1023 = invalid / no reading.
 *
 *	Scan request (Avoidance -> Scan), 25 bits
 *	    bits  0..1   type    SREQ_*            (2)
 *	    bits  2..4   scan id                   (3)
 *	    bits  5..12  start angle, deg signed   (8)
 *	    bits 13..20  end angle, deg signed     (8)
 *	    bits 21..24  step, deg 1..15           (4)
 *	Scan sample (Scan -> Profile), 25 bits
 *	    bits  0..7   servo angle, deg signed   (8)  + = right
 *	    bits  8..17  distance code             (10)
 *	    bits 18..20  scan id                   (3)
 *	    bit  21      phase  0 coarse / 1 fine  (1)
 *	    bit  22      last sample of the scan   (1)
 *	    bits 23..24  status SS_*               (2)
 *	Obstacle profile (Profile -> Avoidance), 2 words
 *	    w0 bits  0..2   scan id                (3)
 *	       bits  3..4   status PRF_*           (2)
 *	       bits  5..14  forward distance code  (10)  nearest face
 *	       bits 15..22  bearing, deg signed    (8)
 *	       bits 23..30  width, 5 mm            (8)  255 = unknown
 *	       bit  31      phase                  (1)
 *	    w1 bits  0..7   left edge, 10 mm signed  (8) lateral, + = right
 *	       bits  8..15  right edge, 10 mm signed (8)
 *	       bits 16..23  left clearance, 10 mm  (8)  254 = >= 2.54 m, 255 = unknown
 *	                                                (unknown is never treated as free)
 *	       bits 24..31  right clearance, 10 mm (8)
 */

#ifndef S5_MSG_H
#define S5_MSG_H

#include <tk/typedef.h>

#define S5_DIST_STEP_MM		5
#define S5_DIST_MAX_CODE	800
#define S5_DIST_NO_ECHO		1022U
#define S5_DIST_INVALID		1023U

#define SREQ_COARSE		0U
#define SREQ_FINE		1U
#define SREQ_MONITOR		2U	/* forward ranging, servo centred */
#define SREQ_IDLE		3U	/* stop ranging, servo centred    */

#define SS_OK			0U
#define SS_NO_ECHO		1U
#define SS_INVALID		2U
#define SS_ABORTED		3U

#define PRF_VALID		0U
#define PRF_NO_OBSTACLE		1U
#define PRF_INVALID		2U	/* bad data / servo suspect   */
#define PRF_INSUFFICIENT	3U	/* obstacle wider than the scan */

#define S5_WIDTH_UNKNOWN	255U
#define S5_CLEAR_UNKNOWN	255U

typedef struct {
	UB	type;
	UB	scan_id;
	B	start_deg;
	B	end_deg;
	UB	step_deg;
} S5_SCAN_REQ;

typedef struct {
	B	angle_deg;
	UH	dist_code;
	UB	scan_id;
	UB	phase;
	UB	last;
	UB	status;
} S5_SAMPLE;

/* Unpacked profile in engineering units (mm, deg) */
typedef struct {
	UB	scan_id;
	UB	status;
	UB	phase;
	W	dist_mm;	/* forward distance to the nearest face, -1 unknown */
	W	bearing_deg;
	W	width_mm;	/* -1 unknown */
	W	left_mm;	/* lateral position of the left edge  */
	W	right_mm;	/* lateral position of the right edge */
	W	clear_left_mm;	/* -1 unknown */
	W	clear_right_mm;
} S5_PROFILE;

IMPORT UH   s5_dist_code(UW mm);
IMPORT W    s5_dist_mm(UH code);		/* -1 for the special codes */

IMPORT UW   s5_req_pack(const S5_SCAN_REQ *r);
IMPORT void s5_req_unpack(UW w, S5_SCAN_REQ *r);
IMPORT UW   s5_sample_pack(const S5_SAMPLE *s);
IMPORT void s5_sample_unpack(UW w, S5_SAMPLE *s);
IMPORT void s5_profile_pack(const S5_PROFILE *p, UW w[2]);
IMPORT void s5_profile_unpack(const UW w[2], S5_PROFILE *p);

/* ---------------------------------------------------------------------
 *  Status words published by subsystem 5 (one writer each)
 *
 *  Scan status (writer: Obstacle Scan task), 28 bits
 *      bits  0..9   forward distance code   (10)
 *      bits 10..17  servo angle, deg signed (8)
 *      bits 18..19  last ranging result     (2)  US_* (drv_ultrasonic.h)
 *      bits 20..21  scan mode               (2)  SREQ_*
 *      bits 22..25  sequence                (4)  +1 per ranging
 *      bit  26      speed cap active        (1)
 *      bit  27      obstacle detected       (1)
 *  Avoidance status (writer: Avoidance & Recovery task), 20 bits
 *      bits  0..3   state                   (4)  AVST_*
 *      bits  4..5   side                    (2)  AV_SIDE_*
 *      bits  6..8   plan step               (3)
 *      bits  9..10  retries                 (2)
 *      bits 11..12  last outcome            (2)  AVO_*
 *      bits 13..16  avoidance count         (4)  modulo 16
 *      bits 17..19  fail reason             (3)  AVF_*
 * --------------------------------------------------------------------- */
#define AVST_IDLE	0U	/* Idle / Monitoring              */
#define AVST_STOPPING	1U	/* Stop / reduce speed            */
#define AVST_COARSE	2U
#define AVST_FINE	3U
#define AVST_PLANNING	4U	/* profile + plan avoidance       */
#define AVST_EXECUTING	5U
#define AVST_SEARCHING	6U	/* search for the original line   */
#define AVST_ALIGNING	7U	/* reacquire line                 */
#define AVST_FAILED	8U	/* Safe Stop / Fault              */

#define AVO_NONE	0U
#define AVO_SUCCESS	1U
#define AVO_FAILED	2U
#define AVO_CLEARED	3U	/* obstacle disappeared */

#define AVF_NONE	0U
#define AVF_PROFILE	1U	/* scan / profile failure  */
#define AVF_CLEARANCE	2U	/* no safe side            */
#define AVF_MOTION	3U	/* manoeuvre failed        */
#define AVF_LINE	4U	/* line not reacquired     */
#define AVF_TIMEOUT	5U	/* recovery time exceeded  */
#define AVF_SENSOR	6U	/* ultrasonic not usable   */

typedef struct {
	UH	dist_code;
	B	servo_deg;
	UB	us_result;
	UB	mode;
	UB	seq;
	UB	cap;
	UB	detected;
} S5_SCAN_STATUS;

typedef struct {
	UB	state;
	UB	side;
	UB	step;
	UB	retries;
	UB	outcome;
	UB	count;
	UB	fail;
} S5_AVOID_STATUS;

IMPORT UW   s5_scan_status_pack(const S5_SCAN_STATUS *s);
IMPORT void s5_scan_status_unpack(UW w, S5_SCAN_STATUS *s);
IMPORT UW   s5_avoid_status_pack(const S5_AVOID_STATUS *s);
IMPORT void s5_avoid_status_unpack(UW w, S5_AVOID_STATUS *s);

#endif /* S5_MSG_H */
