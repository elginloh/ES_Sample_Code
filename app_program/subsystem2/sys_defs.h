/*
 *	sys_defs.h
 *	System-wide definitions owned by subsystem 2 (Buddy 2): vehicle
 *	operating mode, the shared fault bitmask and the events other
 *	subsystems send to Buddy 2.
 *
 *	Tribunal 1, Version B: "Ownership of car's operating mode -> Buddy 2.
 *	Buddy 3, 4 and 5 send events to Buddy 2 (barcode command, line lost,
 *	collision, obstacle detected, avoidance complete, recovery failed).
 *	Buddy 2 updates the mode."
 *	Change Requirement 1: faults are individual bits of ONE shared
 *	bitmask; enumerations use the minimum number of bits.
 *
 *	Pure definitions only (no kernel calls), so host tests can include it.
 */

#ifndef SYS_DEFS_H
#define SYS_DEFS_H

#include <tk/typedef.h>

/* ---------------------------------------------------------------------
 *  Vehicle operating mode (Week 6 system-level statechart, section 5).
 *  5 states -> 3 bits.  Codes 5..7 are invalid.
 * --------------------------------------------------------------------- */
#define VM_INIT		0U	/* Initialising: calibrate, motors off */
#define VM_NORMAL	1U	/* Normal Navigation: line following */
#define VM_NAV_ACTION	2U	/* Navigation Action: barcode command */
#define VM_OBSTACLE	3U	/* Obstacle Handling: Buddy 5 in control */
#define VM_SAFE_STOP	4U	/* Safe Stop / Fault: motors disabled */
#define VM_COUNT	5U
#define VM_BITS		3U

/* ---------------------------------------------------------------------
 *  Shared fault bitmask.  One bit per independent fault, one owner per
 *  bit.  27 bits are defined -> transmitted as a 27-bit field.
 *
 *  Severity (sys_fault_severity()):
 *    FSEV_DEGRADED   reported only; the vehicle keeps running
 *    FSEV_CRIT_COND  critical CONDITION: forces Safe Stop; the owner
 *                    clears the bit when the condition has gone
 *    FSEV_CRIT_EVENT critical EVENT: forces Safe Stop; latched until a
 *                    validated supervisory CLEAR/RESUME (Week 6 system
 *                    statechart: "wait for recovery / reset condition")
 *  The policy table is in sys_fault_severity() (sys_state_logic.c) so
 *  the team can change one row instead of hunting through tasks.
 * --------------------------------------------------------------------- */
#define SYS_FLT_MOTOR_STALL		(1UL << 0)	/* B2: PWM, no ticks   */
#define SYS_FLT_ENCODER			(1UL << 1)	/* B2: one wheel silent */
#define SYS_FLT_MOTION_ESCALATED	(1UL << 2)	/* B2: retry limit hit  */
#define SYS_FLT_MOTOR_WDOG		(1UL << 3)	/* B2: control loop late */
#define SYS_FLT_CMD_QUEUE		(1UL << 4)	/* B2: command rejected, queue full */
#define SYS_FLT_IR_CAL			(1UL << 5)	/* B3: IR calibration failed */
#define SYS_FLT_LINE_LOST		(1UL << 6)	/* B3: line lost, searching */
#define SYS_FLT_LINE_RECOVERY		(1UL << 7)	/* B3: line search exhausted */
#define SYS_FLT_BARCODE_INVALID		(1UL << 8)	/* B3: last barcode invalid */
#define SYS_FLT_BARCODE_OVERFLOW	(1UL << 9)	/* B3: edge buffer overflow */
#define SYS_FLT_IMU			(1UL << 10)	/* B4: no response / invalid data */
#define SYS_FLT_IMU_CAL			(1UL << 11)	/* B4: calibration failed */
#define SYS_FLT_COLLISION		(1UL << 12)	/* B4: impact detected */
#define SYS_FLT_US_NO_RESPONSE		(1UL << 13)	/* B5: ultrasonic silent */
#define SYS_FLT_US_INVALID		(1UL << 14)	/* B5: invalid distance */
#define SYS_FLT_SERVO_SUSPECT		(1UL << 15)	/* B5: scan looks frozen */
#define SYS_FLT_OBSTACLE_UNAVOIDABLE	(1UL << 16)	/* B5: no safe clearance */
#define SYS_FLT_PROFILE_INVALID		(1UL << 17)	/* B5: profiling failed */
#define SYS_FLT_AVOID_FAILED		(1UL << 18)	/* B5: manoeuvre failed */
#define SYS_FLT_RECOVERY_TIMEOUT	(1UL << 19)	/* B5: line not reacquired */
#define SYS_FLT_WIFI_LOSS		(1UL << 20)	/* B1: WiFi link lost */
#define SYS_FLT_MQTT_LOSS		(1UL << 21)	/* B1: broker session lost */
#define SYS_FLT_CMD_INVALID		(1UL << 22)	/* B1: last remote command rejected */
#define SYS_FLT_TLM_OVERFLOW		(1UL << 23)	/* B1: telemetry dropped */
#define SYS_FLT_INIT			(1UL << 24)	/* B2: initialisation timeout */
#define SYS_FLT_TASK_COMM		(1UL << 25)	/* any: kernel object/queue failure on a safety path */
#define SYS_FLT_REMOTE_STOP		(1UL << 26)	/* B1->B2: operator STOP held */
#define SYS_FLT_COUNT			27U
#define SYS_FLT_ALL			((1UL << SYS_FLT_COUNT) - 1UL)

#define FSEV_DEGRADED		0U
#define FSEV_CRIT_COND		1U
#define FSEV_CRIT_EVENT		2U

/* Degraded bits that record a past event rather than a present
   condition; a supervisory CLEAR / RESUME clears them as well. */
#define SYS_FLT_TRANSIENT	(SYS_FLT_CMD_QUEUE | SYS_FLT_BARCODE_INVALID \
				 | SYS_FLT_BARCODE_OVERFLOW | SYS_FLT_CMD_INVALID \
				 | SYS_FLT_TLM_OVERFLOW | SYS_FLT_US_INVALID)

/* ---------------------------------------------------------------------
 *  Events sent to Buddy 2 (sys_post_event).  One bit each so several can
 *  be pending at once without a queue; Buddy 2 consumes them every cycle.
 * --------------------------------------------------------------------- */
#define SYS_EV_IR_CAL_OK	(1UL << 0)	/* B3 */
#define SYS_EV_IR_CAL_FAIL	(1UL << 1)	/* B3 */
#define SYS_EV_IMU_CAL_OK	(1UL << 2)	/* B4 */
#define SYS_EV_IMU_CAL_FAIL	(1UL << 3)	/* B4 */
#define SYS_EV_LINE_LOST	(1UL << 4)	/* B3 */
#define SYS_EV_LINE_FOUND	(1UL << 5)	/* B3 */
#define SYS_EV_OBSTACLE		(1UL << 6)	/* B5: obstacle detected */
#define SYS_EV_AVOID_DONE	(1UL << 7)	/* B5: avoided + line reacquired */
#define SYS_EV_RECOVERY_FAILED	(1UL << 8)	/* B5 */
#define SYS_EV_COLLISION	(1UL << 9)	/* B4 */
#define SYS_EV_COUNT		10U
#define SYS_EV_ALL		((1UL << SYS_EV_COUNT) - 1UL)

/* ---------------------------------------------------------------------
 *  Vehicle status word (VSW): one 32-bit word, written ONLY by the
 *  Motion Control task, read by everyone (aligned word = atomic load).
 *
 *    bits  0..2   mode            (3)  VM_*
 *    bits  3..5   epoch           (3)  bumps on entering INIT/OBSTACLE/
 *                                      SAFE_STOP; stale-command guard
 *    bits  6..7   motion state    (2)  MS_* (motion_msg.h)
 *    bits  8..9   active source   (2)  MSRC_* of the running command
 *    bit   10     command active  (1)
 *    bit   11     IR calibrated   (1)  for the current INIT epoch
 *    bit   12     IMU calibrated  (1)
 *    bit   13     armed           (1)  motors allowed to move
 *    bit   14     line-lost cap   (1)  speed capped to creep
 *    bit   15     obstacle cap    (1)  Buddy 5 speed cap active
 *    bits 16..17  escalation cnt  (2)  motion fault retries (saturating)
 *    bits 18..31  reserved (0)
 * --------------------------------------------------------------------- */
#define VSW_MODE_SHIFT		0U
#define VSW_MODE_MASK		0x7UL
#define VSW_EPOCH_SHIFT		3U
#define VSW_EPOCH_MASK		0x7UL
#define VSW_MSTATE_SHIFT	6U
#define VSW_MSTATE_MASK		0x3UL
#define VSW_SRC_SHIFT		8U
#define VSW_SRC_MASK		0x3UL
#define VSW_CMD_ACTIVE		(1UL << 10)
#define VSW_IR_OK		(1UL << 11)
#define VSW_IMU_OK		(1UL << 12)
#define VSW_ARMED		(1UL << 13)
#define VSW_LINE_CAP		(1UL << 14)
#define VSW_OBST_CAP		(1UL << 15)
#define VSW_ESC_SHIFT		16U
#define VSW_ESC_MASK		0x3UL

static inline UW vsw_field(UW vsw, UW shift, UW mask)
{
	return (vsw >> shift) & mask;
}

static inline UW vsw_mode(UW vsw)	{ return vsw_field(vsw, VSW_MODE_SHIFT, VSW_MODE_MASK); }
static inline UW vsw_epoch(UW vsw)	{ return vsw_field(vsw, VSW_EPOCH_SHIFT, VSW_EPOCH_MASK); }
static inline UW vsw_mstate(UW vsw)	{ return vsw_field(vsw, VSW_MSTATE_SHIFT, VSW_MSTATE_MASK); }
static inline UW vsw_source(UW vsw)	{ return vsw_field(vsw, VSW_SRC_SHIFT, VSW_SRC_MASK); }
static inline UW vsw_escalation(UW vsw)	{ return vsw_field(vsw, VSW_ESC_SHIFT, VSW_ESC_MASK); }

/* Fault policy (sys_state_logic.c, pure) */
IMPORT UW   sys_fault_severity(UW bit_index);
IMPORT UW   sys_fault_critical_mask(void);	/* CRIT_COND | CRIT_EVENT bits */
IMPORT UW   sys_fault_event_mask(void);		/* CRIT_EVENT bits only */
IMPORT const char *sys_mode_name(UW mode);

/* Validated supervisory CLEAR / RESUME (MCMD_* type): the fault bits to
   clear (*clear_bits) and the MREJ_* verdict.  RESUME is accepted only
   in Safe Stop and only when no critical CONDITION remains after the
   clear, so a remote command never overrides a safety action. */
IMPORT UW   sys_recovery_decide(UW type, UW mode, UW faults, UW *clear_bits);

#endif /* SYS_DEFS_H */
