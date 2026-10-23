/*
 *	drv_motor.c
 *	Motor PWM output (see drv_motor.h).
 */

#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_pwm.h"
#include "motion_cfg.h"

#define MOTOR_PWM_TOP	(PWM_CLK_HZ / MOTOR_PWM_HZ - 1UL)	/* 12499 at 10 kHz */

LOCAL volatile UW	motor_is_ready;

EXPORT ER motor_init(void)
{
	if (!pwm_block_enable()) {
		return E_IO;
	}
	/* both channels start at 0, so the pins switch to PWM with the
	   motors off */
	pwm_slice_setup(MOTOR_L_SLICE, 1, MOTOR_PWM_TOP);
	pwm_slice_setup(MOTOR_R_SLICE, 1, MOTOR_PWM_TOP);
	(void)pwm_set_pin(MOTOR_L_A_PIN);		/* libbsp: FUNCSEL = PWM */
	(void)pwm_set_pin(MOTOR_L_B_PIN);
	(void)pwm_set_pin(MOTOR_R_A_PIN);
	(void)pwm_set_pin(MOTOR_R_B_PIN);
	app_store(&motor_is_ready, 1U);
	return E_OK;
}

EXPORT BOOL motor_ready(void)
{
	return (app_load(&motor_is_ready) != 0U) ? TRUE : FALSE;
}

LOCAL void write_motor(UW slice, W duty, BOOL invert)
{
	UW	cc;

	if (invert) {
		duty = -duty;
	}
	if (duty > 1000) {
		duty = 1000;
	} else if (duty < -1000) {
		duty = -1000;
	}
	cc = (UW)((duty < 0) ? -duty : duty) * (MOTOR_PWM_TOP + 1UL) / 1000UL;
	if (duty >= 0) {
		pwm_slice_write_cc(slice, cc, 0);	/* forward: A = PWM, B = 0 */
	} else {
		pwm_slice_write_cc(slice, 0, cc);	/* reverse: A = 0, B = PWM */
	}
}

EXPORT void motor_set(W duty_l_pm, W duty_r_pm)
{
	if (app_load(&motor_is_ready) == 0U) {
		return;
	}
	write_motor(MOTOR_L_SLICE, duty_l_pm, MOTOR_L_INVERT != 0);
	write_motor(MOTOR_R_SLICE, duty_r_pm, MOTOR_R_INVERT != 0);
}

EXPORT void motor_off(void)
{
	if (app_load(&motor_is_ready) == 0U) {
		return;		/* PWM block / pins not set up: nothing is driven */
	}
	pwm_slice_write_cc(MOTOR_L_SLICE, 0, 0);
	pwm_slice_write_cc(MOTOR_R_SLICE, 0, 0);
}
