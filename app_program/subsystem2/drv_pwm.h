/*
 *	drv_pwm.h
 *	RP2040 PWM slice helpers shared by the motor driver (Buddy 2, slices
 *	4 and 5) and the scanning servo (Buddy 5, slice 7).  Each slice has
 *	exactly one owning task; these helpers never touch another slice.
 *
 *	The kernel only releases the PWM block from reset when USE_PTMR is 1
 *	(hw_setting.c), which this project does not use, so the first owner
 *	to initialise releases it.  Releasing is idempotent and the reset is
 *	never re-asserted (that would stop the other owner's slice).
 *
 *	The compare register holds BOTH channels (A = bits 0..15, B = bits
 *	16..31).  pwm_slice_write_cc() writes the whole register in one store,
 *	so there is no read-modify-write window (libbsp's pwm_set_cc() does a
 *	read-modify-write, which is unsafe if a handler also writes it).
 */

#ifndef DRV_PWM_H
#define DRV_PWM_H

#include <tk/tkernel.h>

#define PWM_CLK_HZ		(TMCLK_MHz * 1000000UL)	/* clk_sys, 125 MHz */
#define PWM_SLICE_REG(s, off)	(PWM_BASE + (UW)(s) * 0x14UL + (off))
#define PWM_RESET_WAIT_MAX	10000U

/* Release the PWM block from reset (idempotent).  FALSE if it never
   reported done. */
Inline BOOL pwm_block_enable(void)
{
	UW	n = PWM_RESET_WAIT_MAX;

	clr_w(RESETS_RESET, RESETS_RESET_PWM);
	while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0U) {
		if (n-- == 0U) {
			return FALSE;
		}
	}
	return TRUE;
}

/* Configure a slice: integer divider 1..255, wrap (TOP), both outputs 0,
   free-running, enabled. */
Inline void pwm_slice_setup(UW slice, UW div_int, UW top)
{
	out_w(PWM_SLICE_REG(slice, PWM_CHx_CSR), 0);		/* disable */
	out_w(PWM_SLICE_REG(slice, PWM_CHx_DIV), (div_int & 0xFFU) << 4);
	out_w(PWM_SLICE_REG(slice, PWM_CHx_TOP), top & 0xFFFFU);
	out_w(PWM_SLICE_REG(slice, PWM_CHx_CC), 0);
	out_w(PWM_SLICE_REG(slice, PWM_CHx_CTR), 0);
	out_w(PWM_SLICE_REG(slice, PWM_CHx_CSR), PWM_CH_CSR_EN);	/* enable */
}

/* Write both channel compare values in one store */
Inline void pwm_slice_write_cc(UW slice, UW cc_a, UW cc_b)
{
	out_w(PWM_SLICE_REG(slice, PWM_CHx_CC),
	      (cc_a & 0xFFFFU) | ((cc_b & 0xFFFFU) << 16));
}

#endif /* DRV_PWM_H */
