/*
 *	sys_state.c
 *	Shared fault bitmask, events to Buddy 2 and the vehicle status word.
 *
 *	Change Requirement 1: "Fault conditions ... use individual bits within
 *	a shared fault bitmask" and "read-modify-write operations on shared
 *	packed data must not introduce race conditions".  Several tasks and
 *	ISRs set/clear fault bits, so every update is an atomic OR / AND-NOT
 *	(app_atomic_*).  The vehicle status word has one writer (Motion
 *	Control) and is read with a single aligned load.
 */

#include "subsystem2.h"
#include "s2_private.h"

LOCAL volatile UW	sys_fault_word;		/* many writers: atomics only */
LOCAL volatile UW	sys_fault_seen_word;	/* every bit set since boot */
LOCAL volatile UW	sys_event_word;		/* many writers: atomics only */
LOCAL volatile UW	sys_vsw_word;		/* writer: Motion Control */
LOCAL UW		sys_crit_mask;		/* constant after init */

EXPORT void sys_state_init(void)
{
	sys_fault_word = 0;
	sys_fault_seen_word = 0;
	sys_event_word = 0;
	sys_vsw_word = (UW)VM_INIT << VSW_MODE_SHIFT;
	sys_crit_mask = sys_fault_critical_mask();
}

EXPORT UW sys_fault_get(void)
{
	return app_load(&sys_fault_word);
}

EXPORT void sys_fault_set(UW bits)
{
	UW	old;

	bits &= SYS_FLT_ALL;
	if (bits == 0U) {
		return;
	}
	old = app_atomic_or(&sys_fault_word, bits);
	(void)app_atomic_or(&sys_fault_seen_word, bits);
	/* A NEW critical bit wakes Motion Control at once, so motor output
	   is cut without waiting for the next control period. */
	if ((~old & bits & sys_crit_mask) != 0U && flg_motion > 0) {
		(void)tk_set_flg(flg_motion, MF_FAULT);
	}
}

EXPORT void sys_fault_clear(UW bits)
{
	(void)app_atomic_clear(&sys_fault_word, bits & SYS_FLT_ALL);
}

/* Every fault bit raised since boot, cleared or not.  A critical fault
   that clears itself (IMU, ultrasonic silence) still leaves Safe Stop
   latched; this is how the cause is found afterwards. */
EXPORT UW sys_fault_seen(void)
{
	return app_load(&sys_fault_seen_word);
}

EXPORT UW sys_fault_crit_mask(void)
{
	return sys_crit_mask;
}

EXPORT void sys_post_event(UW ev)
{
	ev &= SYS_EV_ALL;
	if (ev == 0U) {
		return;
	}
	(void)app_atomic_or(&sys_event_word, ev);
	if (flg_motion > 0) {
		(void)tk_set_flg(flg_motion, MF_EVENT);
	}
}

EXPORT UW sys_take_events(void)
{
	return app_atomic_xchg(&sys_event_word, 0U);
}

EXPORT UW sys_vsw(void)
{
	return app_load(&sys_vsw_word);
}

EXPORT void sys_vsw_publish(UW vsw)
{
	app_store(&sys_vsw_word, vsw);
}

EXPORT UW sys_mode(void)
{
	return vsw_mode(sys_vsw());
}

EXPORT UW sys_epoch(void)
{
	return vsw_epoch(sys_vsw());
}
