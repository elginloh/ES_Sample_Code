/*
 *	sys_state_logic.c
 *	Fault severity policy and mode names (pure logic, host-testable).
 *
 *	Severity follows the Week 6 system-level statechart: "critical sensor
 *	fault / collision / invalid data", "scan failure / invalid profile /
 *	avoidance failure / line recovery timeout" and "initialisation or
 *	calibration failure" lead to Safe Stop; network loss never does
 *	(NFR9); motor stall / encoder faults are first handled locally by
 *	Motion Control and only escalate after the retry limit (Tribunal 1).
 *
 *	TEAM DECISION FLAGS (Tribunal 1 readiness table: "collision response,
 *	shared fault-mask handling, fault clearing ... not finalised"):
 *	  SYS_COLLISION_STOPS  1 = collision forces Safe Stop (Week 6 statechart)
 *	  SYS_IMU_REQUIRED     1 = IMU faults are critical (statechart: critical
 *	                           sensor fault / invalid data -> Safe Stop)
 *	  SYS_US_REQUIRED      1 = a silent ultrasonic sensor is critical,
 *	                           because driving without obstacle detection
 *	                           is unsafe (FR15)
 */

#include "sys_defs.h"
#include "motion_msg.h"

#ifndef SYS_COLLISION_STOPS
#define SYS_COLLISION_STOPS	1
#endif
#ifndef SYS_IMU_REQUIRED
#define SYS_IMU_REQUIRED	1
#endif
#ifndef SYS_US_REQUIRED
#define SYS_US_REQUIRED		1
#endif

#if SYS_IMU_REQUIRED
#define SEV_IMU_COND	FSEV_CRIT_COND
#define SEV_IMU_CAL	FSEV_CRIT_EVENT
#else
#define SEV_IMU_COND	FSEV_DEGRADED
#define SEV_IMU_CAL	FSEV_DEGRADED
#endif

#if SYS_COLLISION_STOPS
#define SEV_COLLISION	FSEV_CRIT_EVENT
#else
#define SEV_COLLISION	FSEV_DEGRADED
#endif

#if SYS_US_REQUIRED
#define SEV_US_SILENT	FSEV_CRIT_COND
#else
#define SEV_US_SILENT	FSEV_DEGRADED
#endif

/* One row per SYS_FLT_* bit, in bit order (see sys_defs.h) */
LOCAL const UB fault_severity[SYS_FLT_COUNT] = {
	FSEV_DEGRADED,		/*  0 MOTOR_STALL: local FAULT + retry   */
	FSEV_DEGRADED,		/*  1 ENCODER: local FAULT + retry       */
	FSEV_CRIT_EVENT,	/*  2 MOTION_ESCALATED                   */
	FSEV_CRIT_EVENT,	/*  3 MOTOR_WDOG                         */
	FSEV_DEGRADED,		/*  4 CMD_QUEUE                          */
	FSEV_CRIT_EVENT,	/*  5 IR_CAL (init calibration failure)  */
	FSEV_DEGRADED,		/*  6 LINE_LOST (Buddy 3 searching)      */
	FSEV_CRIT_EVENT,	/*  7 LINE_RECOVERY (retries exhausted)  */
	FSEV_DEGRADED,		/*  8 BARCODE_INVALID                    */
	FSEV_DEGRADED,		/*  9 BARCODE_OVERFLOW                   */
	SEV_IMU_COND,		/* 10 IMU                                */
	SEV_IMU_CAL,		/* 11 IMU_CAL                            */
	SEV_COLLISION,		/* 12 COLLISION                          */
	SEV_US_SILENT,		/* 13 US_NO_RESPONSE                     */
	FSEV_DEGRADED,		/* 14 US_INVALID (single reading)        */
	FSEV_DEGRADED,		/* 15 SERVO_SUSPECT (profile then fails) */
	FSEV_CRIT_EVENT,	/* 16 OBSTACLE_UNAVOIDABLE (FR12)        */
	FSEV_CRIT_EVENT,	/* 17 PROFILE_INVALID (FR12)             */
	FSEV_CRIT_EVENT,	/* 18 AVOID_FAILED (FR12)                */
	FSEV_CRIT_EVENT,	/* 19 RECOVERY_TIMEOUT (FR12)            */
	FSEV_DEGRADED,		/* 20 WIFI_LOSS (NFR9)                   */
	FSEV_DEGRADED,		/* 21 MQTT_LOSS (NFR9)                   */
	FSEV_DEGRADED,		/* 22 CMD_INVALID                        */
	FSEV_DEGRADED,		/* 23 TLM_OVERFLOW                       */
	FSEV_CRIT_EVENT,	/* 24 INIT                               */
	FSEV_CRIT_EVENT,	/* 25 TASK_COMM                          */
	FSEV_CRIT_EVENT		/* 26 REMOTE_STOP (operator hold)        */
};

EXPORT UW sys_fault_severity(UW bit_index)
{
	if (bit_index >= SYS_FLT_COUNT) {
		return FSEV_DEGRADED;
	}
	return fault_severity[bit_index];
}

LOCAL UW mask_of(UW sev_a, UW sev_b)
{
	UW	i, mask = 0;

	for (i = 0; i < SYS_FLT_COUNT; i++) {
		if (fault_severity[i] == sev_a || fault_severity[i] == sev_b) {
			mask |= (1UL << i);
		}
	}
	return mask;
}

EXPORT UW sys_fault_critical_mask(void)
{
	return mask_of(FSEV_CRIT_COND, FSEV_CRIT_EVENT);
}

EXPORT UW sys_fault_event_mask(void)
{
	return mask_of(FSEV_CRIT_EVENT, FSEV_CRIT_EVENT);
}

EXPORT UW sys_recovery_decide(UW type, UW mode, UW faults, UW *clear_bits)
{
	UW	clear = sys_fault_event_mask() | SYS_FLT_TRANSIENT;

	*clear_bits = 0;
	if (type == MCMD_CLEAR) {
		*clear_bits = clear;		/* latched events only; mode unchanged */
		return MREJ_NONE;
	}
	if (type != MCMD_RESUME || mode != VM_SAFE_STOP) {
		return MREJ_MODE;
	}
	*clear_bits = clear;
	return ((faults & ~clear & sys_fault_critical_mask()) != 0UL)
	       ? MREJ_MODE : MREJ_NONE;	/* a critical condition is still active */
}

EXPORT const char *sys_mode_name(UW mode)
{
	switch (mode) {
	case VM_INIT:		return "INIT";
	case VM_NORMAL:		return "NORMAL";
	case VM_NAV_ACTION:	return "NAV_ACTION";
	case VM_OBSTACLE:	return "OBSTACLE";
	case VM_SAFE_STOP:	return "SAFE_STOP";
	default:		return "?";
	}
}
