/*
 *	drv_encoder.h
 *	Wheel encoders: Encoder ISR / tick counter (Week 6 task interaction
 *	diagram).  Left A/B = GP0/GP1, right A/B = GP7/GP28.
 *
 *	The ISR (a callback of the shared GPIO handler) only counts and
 *	timestamps (Tribunal 1 interface map).  It is the single writer of
 *	each wheel's record; readers take a consistent copy through a
 *	sequence count, so no lock is ever held in the ISR and the counters
 *	are read "atomically" as Tribunal 1 requires.
 *
 *	Tribunal 1 A3: quadrature is not confirmed, so by default
 *	(ENC_QUADRATURE 0) channel A is counted and the sign comes from the
 *	commanded motor direction set by Motion Control.
 *	Note: GP0/GP1 double as the UART0 console when the image is built
 *	with CRASH_REPORT=1 or CONSOLE=uart (BOARD_UART0_FREE = 0).  Such a
 *	build leaves GP0/GP1 to the console and has no left encoder, so the
 *	car cannot drive (README section 3: unplug the encoder first).
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <tk/tkernel.h>

#define ENC_L_A_PIN	0
#define ENC_L_B_PIN	1
#define ENC_R_A_PIN	7
#define ENC_R_B_PIN	28
#define ENC_LEFT	0U
#define ENC_RIGHT	1U

typedef struct {
	W	ticks;		/* signed cumulative count */
	UW	edges;		/* counted edges (unsigned) */
	UW	last_us;	/* time of the last counted edge */
	UW	period_us;	/* time between the last two counted edges */
	UW	glitches;	/* edges rejected by the glitch filter */
} ENC_SNAP;

IMPORT ER   encoder_init(void);			/* pins + IRQ registration */
IMPORT BOOL encoder_snapshot(UINT wheel, ENC_SNAP *out);
IMPORT void encoder_set_direction(UINT wheel, INT dir);	/* +1 / -1 */

#endif /* DRV_ENCODER_H */
