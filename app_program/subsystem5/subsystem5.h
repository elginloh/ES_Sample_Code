/*
 *	subsystem5.h
 *	Subsystem 5 -- Adaptive ultrasonic scanning, obstacle profiling,
 *	avoidance and line recovery (Buddy 5, student ID 2502075).
 *
 *	Three tasks, kept separate (Tribunal 1, Version B: "No change"):
 *	  Obstacle Scan          prio 15, core 1  owns HC-SR04 (GP4/GP5) + servo (GP15)
 *	  Avoidance & Recovery   prio 16          plans/executes, never drives motors
 *	  Obstacle Profile       prio 19          scan samples -> obstacle profile
 *	Data flow (Week 6 task interaction diagram):
 *	  Scan --mbf_scan_samples--> Profile --mbf_profile--> Avoidance
 *	  Avoidance --mbf_scan_req--> Scan;  Scan --flg_avoid--> Avoidance
 *	  Avoidance --AVOID commands / SYS_EV_*--> Buddy 2
 *	  Buddy 3 line status / flg_line_evt --> Avoidance (line reacquisition)
 *
 *	Interfaces provided: s5_get_scan_status(), s5_get_avoid_status(),
 *	s5_get_profile() (packed words, for telemetry).
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "s5_msg.h"
#include "obstacle_profile.h"
#include "avoid_plan.h"

#define S5_PRI_SCAN		15
#define S5_PRI_AVOID		16
#define S5_PRI_PROFILE		19

/* Detection and speed (FR8) [CAL] */
#define S5_PING_PERIOD_MS	60	/* HC-SR04 measurement cycle        */
#define S5_STOPPING_ALLOWANCE_MM 60	/* at 100 mm/s caution speed: scan closer */
#define S5_DETECT_MM		(AV_SENSOR_BEHIND_FRONT_MM \
				 + AV_MIN_FRONT_CLEARANCE_MM + S5_STOPPING_ALLOWANCE_MM)
#define S5_DETECT_CONFIRM	2	/* consecutive readings             */
#define S5_CAUTION_MM		(S5_DETECT_MM + 150)	/* slow before detect range */
#define S5_CAUTION_MMS		100	/* ... to this cap                  */
#define S5_EMERGENCY_MM		(AV_SENSOR_BEHIND_FRONT_MM \
				 + AV_EMERGENCY_FRONT_CLEARANCE_MM)
#define S5_US_FAIL_COUNT	5	/* silent pings before a fault      */

/* Scanning */
#define S5_COARSE_DEG		60	/* coarse sweep -60..+60 ...        */
#define S5_COARSE_STEP		15	/* ... every 15 deg                 */
#define S5_FINE_STEP		3	/* fine sweep every 3 deg ...       */
#define S5_FINE_MARGIN		12	/* ... 12 deg beyond the coarse edges */
#define S5_SCAN_LIMIT_DEG	75
#define S5_SCAN_PINGS		3	/* majority + median at every angle */
#define S5_SCAN_AGREE_MM	80	/* two-return maximum disagreement */
#define S5_SIDE_LOOK_DEG	75	/* near-perpendicular view of obstacle side */
#define S5_SIDE_PRESENT_MAX_MM 230	/* 176 mm nominal at 10 cm edge gap + tolerance */
#define S5_SIDE_SEEN_CONFIRM	2	/* establish the obstacle before looking for edge */
#define S5_SIDE_CLEAR_CONFIRM	5	/* consecutive clear side readings */
#define S5_SIDE_BAD_MAX	10	/* unusable side readings before failure */
#define S5_SIDE_READY_MS	2000	/* servo reach + first side sample */
#define S5_SIDE_MIN_TRAVEL_MM	60	/* reject an immediate false edge */
#define S5_SIDE_CENTER_MS	700	/* centre servo before final forward clearance */

/* Avoidance and recovery (bounded) [CAL] */
#define S5_HOLD_MS		300	/* "hold position briefly"          */
#define S5_STOP_WAIT_MS		1500
#define S5_SCAN_TIMEOUT_MS	15000	/* three pings/angle fine scan is ~10 s;
					   leave margin for serial/Wi-Fi scheduling */
#define S5_AVOID_SPEED_LVL	2	/* 2 * 25 = 50 mm/s (was 6 = 150, then
					   4 = 100: turns spun the car ~90 deg/s.
					   The ~1.8 m plan must still finish inside
					   S5_RECOVERY_TIMEOUT_MS) */
#define S5_STEP_EXTRA_MS	3000
#define S5_RECOVERY_TIMEOUT_MS	90000	/* whole avoidance + reacquisition;
					   includes an optional reverse + rescan and
					   the ~1.8 m plan at 50 mm/s */
#define S5_REVERSE_SPEED_LVL	1	/* bounded recovery reverse at 25 mm/s */
#define S5_SEARCH_RETRIES	1
#define S5_SEARCH_RETRY_DEG	20
#define S5_REPLANS		1	/* after an emergency stop          */

/* flg_avoid bits */
#define AVEV_DETECTED		0x01U
#define AVEV_EMERGENCY		0x02U
/* flg_us bits */
#define USEV_ECHO		0x01U

#define S5_REQ_QUEUE_LEN	4
#define S5_SAMPLE_QUEUE_LEN	64
#define S5_PROFILE_QUEUE_LEN	4

IMPORT ID	flg_avoid;
IMPORT ID	flg_us;
IMPORT ID	mbf_scan_req;
IMPORT ID	mbf_scan_samples;
IMPORT ID	mbf_profile;
IMPORT ID	tid_scan;
IMPORT ID	tid_profile;
IMPORT ID	tid_avoid;

IMPORT void s5_get_scan_status(S5_SCAN_STATUS *s);
IMPORT void s5_get_avoid_status(S5_AVOID_STATUS *s);
IMPORT void s5_get_profile(S5_PROFILE *p);

/* Private to subsystem 5 */
IMPORT volatile UW	s5_scan_word;		/* writer: scan task    */
IMPORT volatile UW	s5_avoid_word;		/* writer: avoid task   */
IMPORT APP_SEQ		s5_prof_seq;		/* writer: profile task */
IMPORT UW		s5_prof_words[2];
IMPORT TASK_STAT	s5_stat_scan;
IMPORT void scan_task(INT stacd, void *exinf);
IMPORT void profile_task(INT stacd, void *exinf);
IMPORT void avoid_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM5_H */
