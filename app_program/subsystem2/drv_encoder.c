/*
 *	drv_encoder.c
 *	Encoder ISR / tick counter (see drv_encoder.h).
 */

#include "subsystem2.h"
#include "drv_encoder.h"
#include "motion_cfg.h"

typedef struct {
	APP_SEQ	seq;		/* odd while the ISR is updating */
	W	ticks;
	UW	edges;
	UW	last_us;
	UW	period_us;
	UW	glitches;
} ENC_REC;

LOCAL ENC_REC		enc_rec[2];		/* single writer: the ISR */
LOCAL volatile UW	enc_reverse[2];		/* single writer: Motion Control */

/* Shared-GPIO-handler callback: count + timestamp, nothing else */
LOCAL void encoder_edge(UINT pin, UINT events, UW now_us)
{
	UINT	w = (pin == ENC_L_A_PIN) ? ENC_LEFT : ENC_RIGHT;
	ENC_REC	*r = &enc_rec[w];
	W	step;

#if !ENC_COUNT_BOTH_EDGES
	if ((events & GPIO_IRQ_EDGE_RISE) == 0U) {
		return;
	}
#else
	(void)events;
#endif
	/* Glitch filter: an edge closer than ENC_MIN_EDGE_US to the last
	   counted one is comparator chatter, not a slot. */
	if (r->edges != 0U && (UW)(now_us - r->last_us) < ENC_MIN_EDGE_US) {
		r->glitches = r->glitches + 1U;
		return;
	}
#if ENC_QUADRATURE
	{
		UW	b_pin = (w == ENC_LEFT) ? ENC_L_B_PIN : ENC_R_B_PIN;
		W	b = (W)((in_w(GPIO_IN) >> b_pin) & 1UL);

		step = (b != 0) ? -1 : 1;
		if (w == ENC_RIGHT) {
			step = -step;		/* [CAL] mirrored mounting */
		}
	}
#else
	step = (enc_reverse[w] != 0U) ? -1 : 1;
#endif
	app_seq_write_begin(&r->seq);
	r->ticks += step;
	r->period_us = (r->edges == 0U) ? 0U : (now_us - r->last_us);
	r->last_us = now_us;
	r->edges = r->edges + 1U;
	app_seq_write_end(&r->seq);
}

EXPORT ER encoder_init(void)
{
	UINT	ev = GPIO_IRQ_EDGE_RISE;
	ER	er = E_OK;

#if ENC_COUNT_BOTH_EDGES
	ev = GPIO_IRQ_EDGES;
#endif
	enc_reverse[ENC_LEFT] = 0;
	enc_reverse[ENC_RIGHT] = 0;
	gpio_input_config(ENC_R_A_PIN, GPIO_PULL_UP);
	gpio_input_config(ENC_R_B_PIN, GPIO_PULL_UP);
#if BOARD_UART0_FREE
	gpio_input_config(ENC_L_A_PIN, GPIO_PULL_UP);
	gpio_input_config(ENC_L_B_PIN, GPIO_PULL_UP);
	er = gpio_irq_register(ENC_L_A_PIN, ev, encoder_edge);
#else
	/* CRASH_REPORT=1 / CONSOLE=uart: GP0/GP1 carry the UART0 console
	   (README section 3: unplug the encoder for such a build).  The
	   pins are left alone; the left wheel reports no ticks, so Motion
	   Control's stall / encoder detection stops any drive attempt and
	   escalates to Safe Stop. */
	tm_printf((UB*)"encoder: GP0/GP1 belong to the UART0 console in this build; "
		  "left encoder disabled\n");
#endif
	return (er < E_OK) ? er : gpio_irq_register(ENC_R_A_PIN, ev, encoder_edge);
}

EXPORT BOOL encoder_snapshot(UINT wheel, ENC_SNAP *out)
{
	const ENC_REC	*r = &enc_rec[wheel & 1U];
	UINT		tries;

	for (tries = 0; tries < APP_SEQ_READ_TRIES; tries++) {
		UW	s = app_seq_read_begin(&r->seq);

		out->ticks = r->ticks;
		out->edges = r->edges;
		out->last_us = r->last_us;
		out->period_us = r->period_us;
		out->glitches = r->glitches;
		if (!app_seq_read_retry(&r->seq, s)) {
			return TRUE;
		}
	}
	return FALSE;			/* ISR kept updating: caller keeps old data */
}

EXPORT void encoder_set_direction(UINT wheel, INT dir)
{
	if (dir > 0) {
		app_store(&enc_reverse[wheel & 1U], 0U);
	} else if (dir < 0) {
		app_store(&enc_reverse[wheel & 1U], 1U);
	}
	/* dir == 0: keep the last direction (a coasting wheel) */
}
