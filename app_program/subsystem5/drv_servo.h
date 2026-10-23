/*
 *	drv_servo.h
 *	SG90 scanning servo on GP15 (Robo Pico servo port 4) = PWM slice 7,
 *	channel B.  50 Hz frame; the slice counts 1 us per step (clk_sys
 *	125 MHz / 125) so the compare value IS the pulse width in us.
 *	Owned by the Obstacle Scan task (Tribunal 1: "only Obstacle Scan
 *	controls scanning hardware").
 *
 *	Angle convention: 0 = straight ahead, positive = to the RIGHT (same
 *	sign as motion turns).  An SG90 gives no position feedback, so a
 *	failed servo can only be detected indirectly (obstacle_profile.c:
 *	a sweep that returns identical distances at every angle).
 */

#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include <tk/tkernel.h>

#define SERVO_PIN		15
#define SERVO_SLICE		7
#define SERVO_FRAME_US		20000

#define SERVO_CENTER_US		1500	/* [CAL] pulse for straight ahead     */
#define SERVO_US_PER_DEG_X10	111	/* [CAL] ~(2500-500)/180 us per deg  */
#define SERVO_DIR		(-1)	/* [CAL] -1: larger pulse turns left  */
#define SERVO_MIN_US		500
#define SERVO_MAX_US		2500
#define SERVO_LIMIT_DEG		80	/* never drive into the end stops     */
#define SERVO_MS_PER_DEG	2	/* [CAL] SG90 ~0.1 s / 60 deg + margin */
#define SERVO_SETTLE_MIN_MS	30

IMPORT ER   servo_init(void);
/* Move; returns the settle time in ms the caller must wait before ranging */
IMPORT UW   servo_set(INT deg);
IMPORT INT  servo_angle(void);

#endif /* DRV_SERVO_H */
