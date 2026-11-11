/*
 *	motion_cfg.h
 *	Motion Control configuration (Buddy 2).
 *
 *	!!! CALIBRATION REQUIRED !!!  Values marked [CAL] are engineering
 *	starting points, NOT measurements.  None of the design documents gives
 *	the wheel size, encoder resolution, track width or motor response;
 *	Week 6 assumptions A2/A3 and Tribunal 1 say they must be measured on
 *	the bench.  Record the measured values here (and in the PID tuning
 *	report) before relying on distance/turn accuracy (NFR7, NFR8).
 */

#ifndef MOTION_CFG_H
#define MOTION_CFG_H

/* ---- Control timing (Tribunal 1: ONE fixed period, not a 20-50 ms range) */
#define MOTION_PERIOD_MS	20

/* ---- Geometry [CAL] ----------------------------------------------- */
#define WHEEL_DIAMETER_MM	65	/* [CAL] measure the wheel           */
#define ENC_PULSES_PER_REV	628	/* [CAL] counted edges per revolution.
					   Measured: one turn of the left wheel =
					   1256 edges on both edges of A, so 628 on
					   rising edges only (motor-shaft encoder
					   behind the gearbox, ~0.33 mm per edge) */
#define TRACK_WIDTH_MM		130	/* [CAL] wheel centre to wheel centre */

/* Wheel travel per counted encoder edge, micrometres (pi*D/N) */
#define ENC_UM_PER_TICK		((WHEEL_DIAMETER_MM * 3141593UL / 1000UL) \
				 / ENC_PULSES_PER_REV)

/* ---- Speeds (mm/s) [CAL] ------------------------------------------- */
#define CRUISE_DEFAULT_MMS	158	/* [CAL] normal line-following speed (was 200, 20, 30, 45, 90, 135,
					   117 = 135 / 1.15, 176 = 1.5 x 117, 160;
					   158 = 160 - 1 %, rounded to whole mm/s).
					   Floor: a barcode MOVE/TURN must finish
					   inside S3_NAV_WAIT_MS (15 s) */
#define TURN_SPEED_MMS		135	/* [CAL] wheel speed while pivoting (was 120, 20, 30, 45, 90) */
#define MIN_CTRL_SPEED_MMS	22	/* [CAL] A2: minimum controllable speed (was 40,
					   then 15).  Also the creep floor: line
					   search pivot speed */
#define MOTION_ALLOW_REVERSE	0	/* 0 = never drive backwards: a MOVE
					   with a negative distance is rejected
					   (MREJ_RANGE) and a reverse steer
					   stands still.  Pivots still turn one
					   wheel backwards. */
#define MOTION_ALLOW_AVOID_REVERSE 1	/* bounded negative MOVE from Buddy 5
					   only while in VM_OBSTACLE */
#define CREEP_PCT		15	/* NFR12 / Tribunal 1: creep = max(15 % cruise, min) */
#define ACCEL_MMS2		800	/* [CAL] ramp limit                  */
#define DECEL_GAIN		2	/* approach: allowed mm/s per mm left */
#define STEER_UNIT_DIV		80	/* (was 50, then 75 = steering 1.5x slower
					   after the 135 mm/s cruise; back to 50 =
					   1.5x faster once wheel speed was measured
					   over a window instead of one edge period;
					   100 = steering 2x slower after the
					   176 mm/s cruise; 80 = the 40 mm/s
					   pivot at the 160 mm/s cruise)
					   one steer unit = cruise / DIV mm/s of
					   wheel speed difference: 4 mm/s at the
					   designed 200 mm/s cruise (the old fixed
					   STEER_UNIT_MMS 4).  Scaling with cruise
					   keeps the path curvature per steer unit,
					   so the line-follow gains suit any speed.
					   A fixed 1 mm/s at 20 mm/s cruise was
					   2.5x too strong: the car over-steered
					   off the line and kept searching */
#define HEADING_GAIN		3	/* mm/s correction per mm L/R mismatch */

/* ---- Barcode junction turns (90 deg) [CAL] ----------------------------
 *  At the junction the IR sensors are on the crossing line, ~80 mm ahead
 *  of the wheels.  Turning there would leave the crossing line beside
 *  the car, not under its sensors.  So a 90 deg barcode turn:
 *    1. drives on until the RIGHT IR sensor leaves the crossing line
 *       (reference point), then TURN_ADVANCE_x_MM - half a line width
 *       more, so the turn point is where the turn must happen;
 *    2. turns: outer wheel forward, inner wheel slowly backward, about a
 *       point TURN_PIVOT_OFFSET_MM from the axle centre;
 *    3. ends when the centre sensor meets the outgoing tape on a right
 *       turn. A left turn settles near 90 deg, then seeks the tape while
 *       moving forward until the centre sensor finds it.
 *  Advance to align with the outgoing line:
 *    right turn: IR_AHEAD - PIVOT_OFFSET + IR_GAP
 *    left turn:  IR_AHEAD - PIVOT_OFFSET + left course adjustment
 *  U-turns turn on the spot with no advance.
 * --------------------------------------------------------------------- */
#define TURN_IR_AHEAD_MM	80	/* [CAL] centre IR sensor ahead of the
					   wheel axle (= AV_LINE_SENSOR_AHEAD_MM) */
#define TURN_IR_GAP_MM		20	/* [CAL] centre to right IR sensor,
					   measured sideways                  */
#define TURN_LINE_WIDTH_MM	18	/* [CAL] black tape width             */
#define TURN_PIVOT_OFFSET_MM	25	/* turn point this far towards the inner
					   wheel: inner wheel backward at
					   (65-25)/(65+25) = 44 % of the outer */
#define TURN_ADVANCE_TRIM_MM	0	/* [CAL] + if the line ends up nearer the
					   centre sensor, - if nearer the right */
#define TURN_ADVANCE_RIGHT_MM	(TURN_IR_AHEAD_MM - TURN_PIVOT_OFFSET_MM \
				 + TURN_IR_GAP_MM + TURN_ADVANCE_TRIM_MM)
#define TURN_ADVANCE_LEFT_EXTRA_MM 50	/* [CAL] extra approach on the physical course */
#define TURN_ADVANCE_LEFT_MM	(TURN_IR_AHEAD_MM - TURN_PIVOT_OFFSET_MM \
				 + TURN_ADVANCE_TRIM_MM + TURN_ADVANCE_LEFT_EXTRA_MM)
#define TURN_LEFT_COAST_DEG	3	/* release drive before 90 deg */
#define TURN_LEFT_SEARCH_MM	60	/* bounded forward search after pivot */
#define TURN_LEFT_STRAIGHT_MM	20	/* then gently arc back toward the junction */
#define TURN_LINE_SEEK		1	/* 1 = use Buddy 3's line-seen word for
					   the reference point and the end of
					   the turn; 0 = distances/encoders only */
#define TURN_SEEK_EARLY_DEG	20	/* sensors end the turn only from
					   90 - 20 = 70 deg ...               */
#define TURN_SEEK_EXTRA_DEG	15	/* ... and at the latest at 105 deg   */
#define TURN_SEEK_SETTLE_MS	120	/* PWM off before forward search    */
#define LSEEN_STALE_MS		100	/* line-seen word freshness           */
#define TURN_JUNCTION_EDGE_MS	200	/* keep a recently cleared crossing for command handoff */

/* ---- Speed PID + feed-forward [CAL] (duty in permille, -1000..1000) - */
#define PID_KP_Q8		512	/* 2.0  permille per mm/s            */
#define PID_KI_Q8		64	/* 0.25 permille per mm/s*cycle      */
#define PID_KD_Q8		0
#define PID_I_LIMIT		2000	/* (mm/s)*cycles                     */
#define FF_OFFSET_PM		120	/* [CAL] duty to overcome friction   */
#define FF_GAIN_Q8		512	/* [CAL] 2.0 permille per mm/s       */
#define DUTY_MAX_PM		1000

/* ---- Fault detection (Tribunal 1: numeric stall criterion) ---------- */
#define STALL_DUTY_MIN_PM	300	/* driven at least this hard ...      */
#define STALL_TIME_MS		400	/* ... with no encoder edge this long */
#define ENC_FAULT_MIN_MM	60	/* other wheel moved >= this meanwhile */
#define FAULT_CLEAR_MS		300	/* Week 6: clear-for-300 ms debounce  */
#define MAX_FAULT_RETRIES	3	/* Tribunal 1: then escalate to Safe Stop */
#define HEALTHY_RESET_MS	5000	/* fault-free driving resets the count */
#define CMD_TIMEOUT_EXTRA_MS	2000	/* slack added to the expected time   */
#define DONE_TOL_UM		2000	/* target reached within 2 mm         */

/* ---- Freshness of continuously updated inputs ---------------------- */
#define STEER_STALE_MS		100	/* 5 line-following periods           */
#define SCAP_STALE_MS		300	/* obstacle speed cap                 */

/* ---- Safety net ---------------------------------------------------- */
#define MOTOR_WDOG_TICKS	5	/* 5 missed periods = 100 ms (NFR6)   */
#define INIT_TIMEOUT_MS		8000	/* calibration must finish by then    */

/* ---- Motor driver (Robo Pico M1A/M1B = GP8/GP9, M2A/M2B = GP10/GP11) - */
#define MOTOR_PWM_HZ		10000	/* [CAL] confirm against the driver   */
#define MOTOR_L_INVERT		0	/* [CAL] set 1 if the left wheel runs backwards */
#define MOTOR_R_INVERT		0	/* [CAL] set 1 if the right wheel runs backwards */

/* ---- Encoders (left A/B = GP0/GP1, right A/B = GP7/GP28) ------------ */
#define ENC_QUADRATURE		0	/* Tribunal 1 A3.  1 = direction from
					   channel B at each A edge; the B signs in
					   drv_encoder.c are [CAL] and unverified
					   (a wrong sign spun the car: one wheel
					   ran away to full duty).  0 = direction
					   from the sign of the applied duty
					   (motion_ctrl.c), no calibration needed.
					   Before setting 1: turning a wheel
					   forward by hand must read meas > 0 */
#define ENC_SWAP_LR		1	/* 1 = the GP0/GP1 encoder plug is on the
					   RIGHT wheel and GP7/GP28 on the left.
					   Found on the car: the "R" motor drove a
					   wheel only the "L" encoder saw, so the
					   car rocked on one wheel */
#define ENC_COUNT_BOTH_EDGES	0	/* 1 = count rise+fall of A (x2);
					   keep ENC_PULSES_PER_REV in step.  Not
					   needed at 628 edges per turn, and both
					   edges would crowd ENC_MIN_EDGE_US at speed */
#define ENC_MIN_EDGE_US		300	/* glitch filter: reject faster edges */
#define ENC_SPEED_TIMEOUT_MS	300	/* no edge for this long = stopped   */

#endif /* MOTION_CFG_H */
