/*
 *	drv_hwtime.h
 *	Microsecond time base and core id (RP2040).
 *
 *	The kernel tick is 10 ms (config.h CNF_TIMER_PERIOD), too coarse for
 *	encoder periods, barcode edge widths or ultrasonic echo timing.  The
 *	RP2040 TIMER block counts microseconds; the professor's own USB and
 *	CYW43 code read TIMERAWL the same way (usb_tinyusb_glue.c,
 *	cyw43_utk_compat.c).  The 32-bit value wraps every ~71.6 minutes, so
 *	always compare times by unsigned subtraction.
 *
 *	The TIMER rate is checked against the kernel clock by the on-target
 *	self test S2-U41 in subsystem2_test.c (expected 1000 us per ms).
 */

#ifndef DRV_HWTIME_H
#define DRV_HWTIME_H

#include <tk/tkernel.h>

/* Current time in microseconds (free running, wraps) */
Inline UW hw_time_us(void)
{
	return in_w(TIMER_TIMERAWL);
}

/* Microseconds elapsed since 'since' (wrap-safe) */
Inline UW hw_elapsed_us(UW since)
{
	return hw_time_us() - since;
}

/* Busy-wait for a few microseconds (trigger pulses only).  Bounded by an
   iteration cap so a stopped TIMER can never hang the caller; returns
   FALSE if the cap was hit. */
Inline BOOL hw_delay_us(UW us)
{
	UW	start = hw_time_us();
	UW	cap = (us + 1U) * 64U;		/* far above 125 MHz loop rate */

	while (hw_elapsed_us(start) < us) {
		if (cap-- == 0U) {
			return FALSE;
		}
	}
	return TRUE;
}

/* 0 on core 0, 1 on core 1 (SIO CPUID, core-local, no lock needed) */
Inline UINT hw_core_id(void)
{
	return (UINT)in_w(SIO_CPUID_ADDR);
}

#endif /* DRV_HWTIME_H */
