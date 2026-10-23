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
	return E_OK;
}

EXPORT UW servo_set(INT deg)
{
	INT	d;
	UW	settle;

	if (deg > SERVO_LIMIT_DEG) {
		deg = SERVO_LIMIT_DEG;
	} else if (deg < -SERVO_LIMIT_DEG) {
		deg = -SERVO_LIMIT_DEG;
	}
	d = (deg > cur_deg) ? (deg - cur_deg) : (cur_deg - deg);
	if (servo_ready) {
		pwm_slice_write_cc(SERVO_SLICE, 0, deg_to_us(deg));
	}
	cur_deg = deg;
	settle = (UW)d * SERVO_MS_PER_DEG;
	return (settle < SERVO_SETTLE_MIN_MS) ? SERVO_SETTLE_MIN_MS : settle;
}

EXPORT INT servo_angle(void)
{
	return cur_deg;
}
