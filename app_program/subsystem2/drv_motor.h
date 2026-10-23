/*
 *	drv_motor.h
 *	Robo Pico on-board motor driver (Buddy 2 is the ONLY writer).
 *
 *	Pins (Week 6 hardware table): left motor M1A/M1B = GP8/GP9 (PWM slice
 *	4 A/B), right motor M2A/M2B = GP10/GP11 (PWM slice 5 A/B).
 *	Drive scheme for a two-input H-bridge: forward = PWM on A with B low,
 *	reverse = PWM on B with A low, off = both low (outputs disabled).
 *	Tribunal 1 interface map: "PWM is forced to zero in Fault and on
 *	stop()".
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <tk/tkernel.h>

#define MOTOR_L_A_PIN	8
#define MOTOR_L_B_PIN	9
#define MOTOR_R_A_PIN	10
#define MOTOR_R_B_PIN	11
#define MOTOR_L_SLICE	4
#define MOTOR_R_SLICE	5

IMPORT ER   motor_init(void);
/* duty in permille, -1000..1000; sign = direction */
IMPORT void motor_set(W duty_l_pm, W duty_r_pm);
/* Both motors off in one store per slice; safe from handlers */
IMPORT void motor_off(void);
IMPORT BOOL motor_ready(void);

#endif /* DRV_MOTOR_H */
