/*
 *	drv_servo.c
 *	SG90 servo PWM (see drv_servo.h).
 */

#include "subsystem5.h"
#include "drv_servo.h"
#include "../subsystem2/drv_pwm.h"

LOCAL INT	cur_deg;
LOCAL BOOL	servo_ready;		/* PWM slice configured */

LOCAL UW deg_to_us(INT deg)
{
	W	us = SERVO_CENTER_US + SERVO_DIR * (W)deg * SERVO_US_PER_DEG_X10 / 10;

	if (us < SERVO_MIN_US) {
		us = SERVO_MIN_US;
	} else if (us > SERVO_MAX_US) {
		us = SERVO_MAX_US;
	}
	return (UW)us;
}

EXPORT ER servo_init(void)
{
	if (!pwm_block_enable()) {
		return E_IO;
	}
	/* 125 MHz / 125 = 1 MHz count, wrap at 20000 -> 50 Hz */
	pwm_slice_setup(SERVO_SLICE, 125U, SERVO_FRAME_US - 1U);
	(void)pwm_set_pin(SERVO_PIN);			/* libbsp: FUNCSEL = PWM */
	cur_deg = 0;
	pwm_slice_write_cc(SERVO_SLICE, 0, deg_to_us(0));	/* channel B */
	servo_ready = TRUE;
#if SERVO_DEBUG
	tm_printf((UB*)"servo: init pin %d, centre 0 deg, pulse %d us, 50 Hz\n",
		  SERVO_PIN, (INT)deg_to_us(0));
#endif
	return E_OK;
}

EXPORT UW servo_set(INT deg)
{
	INT	next;

	if (deg > SERVO_LIMIT_DEG) {
		deg = SERVO_LIMIT_DEG;
	} else if (deg < -SERVO_LIMIT_DEG) {
		deg = -SERVO_LIMIT_DEG;
	}
#if SERVO_DEBUG
	INT	from = cur_deg;
	INT	delta = (deg >= from) ? deg - from : from - deg;

	if (delta != 0) {
		tm_printf((UB*)"servo: cmd %d -> %d deg, pulse %d us, ramp %d steps\n",
			  from, deg, (INT)deg_to_us(deg),
			  (delta + SERVO_SLEW_STEP_DEG - 1) / SERVO_SLEW_STEP_DEG);
	}
#endif
	if (!servo_ready) {
		cur_deg = deg;
		return SERVO_SETTLE_MS;
	}

	/*
	 * An SG90 always moves to a newly commanded position at its own maximum
	 * speed.  Ramp the command in small increments so a coarse 15-degree scan
	 * step does not become one fast mechanical jump.
	 */
	while (cur_deg != deg) {
		if (deg - cur_deg > SERVO_SLEW_STEP_DEG) {
			next = cur_deg + SERVO_SLEW_STEP_DEG;
		} else if (cur_deg - deg > SERVO_SLEW_STEP_DEG) {
			next = cur_deg - SERVO_SLEW_STEP_DEG;
		} else {
			next = deg;
		}
		pwm_slice_write_cc(SERVO_SLICE, 0, deg_to_us(next));
		cur_deg = next;
		if (cur_deg != deg) {
			(void)tk_dly_tsk(SERVO_SLEW_STEP_MS);
		}
	}
	return SERVO_SETTLE_MS;
}

EXPORT INT servo_angle(void)
{
	return cur_deg;
}
