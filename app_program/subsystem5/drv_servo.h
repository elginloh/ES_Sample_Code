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
#define SERVO_SLEW_STEP_DEG	3	/* smooth commanded motion             */
#define SERVO_SLEW_STEP_MS	20	/* 3 deg / 20 ms = 150 deg/s [CAL]     */
#define SERVO_SETTLE_MS		50	/* wait after reaching each scan angle */
#define SERVO_DEBUG		0	/* set to 1 for command/sample diagnostics */

IMPORT ER   servo_init(void);
/*
 * Move through bounded intermediate angles.  The function performs the
 * inter-step delays and returns the final settle time which the caller must
 * wait before taking an ultrasonic reading.
 */
IMPORT UW   servo_set(INT deg);
IMPORT INT  servo_angle(void);

#endif /* DRV_SERVO_H */
