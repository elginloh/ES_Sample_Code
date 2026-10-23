/*
 *	drv_ir.c
 *	IR sensor inputs and barcode edge capture (see drv_ir.h).
 */

#include "subsystem3.h"
#include "drv_ir.h"

/* Ring: head written only by the ISR, tail only by the consumer task */
LOCAL UW		edge_ring[IR_EDGE_RING];
LOCAL volatile UW	edge_head;
LOCAL volatile UW	edge_tail;
LOCAL volatile UW	edge_overflow;		/* ISR sets, task takes */
LOCAL volatile UW	edge_total;		/* writer: ISR */
LOCAL ID		edge_wake_tid;

LOCAL void push_edge(UW now_us, UW level)
{
	UW	head = edge_head;
	UW	next = (head + 1U) & (IR_EDGE_RING - 1U);

	if (next == app_load(&edge_tail)) {
		edge_overflow = 1U;		/* consumer too slow: flag it */
	} else {
		edge_ring[head] = (now_us & ~1UL) | (level & 1U);
		app_store(&edge_head, next);	/* publish after the data */
		edge_total = edge_total + 1U;
	}
	if (edge_wake_tid > 0) {
		(void)tk_wup_tsk(edge_wake_tid);
	}
}

#if !S3_CAPTURE_POLL_1MS
/* Shared-GPIO-handler callback: timestamp only (Tribunal 1) */
LOCAL void ir_right_edge(UINT pin, UINT events, UW now_us)
{
	(void)pin;
	if (events == GPIO_IRQ_EDGES) {
		/* both edges latched before we got here: a pulse shorter than
		   the interrupt latency; record it in level order so the
		   decoder can merge it away */
		UW	lvl = (in_w(GPIO_IN) >> IR_RIGHT_PIN) & 1UL;

		push_edge(now_us, lvl ^ 1U);
		push_edge(now_us, lvl);
		return;
	}
	push_edge(now_us, (events & GPIO_IRQ_EDGE_RISE) ? 1U : 0U);
}
#else
/* RP2040 TIMER alarm 3 (IRQ 3), re-armed every 1 ms without drift */
#define POLL_ALARM_INTNO	3
#define POLL_ALARM_BIT		(1UL << 3)
#define POLL_PERIOD_US		1000UL
LOCAL UW	poll_next_us;
LOCAL UW	poll_level;

LOCAL void ir_poll_handler(UINT intno)
{
	UW	now = hw_time_us();
	UW	lvl = (in_w(GPIO_IN) >> IR_RIGHT_PIN) & 1UL;

	(void)intno;
	out_w(TIMER_INTR, POLL_ALARM_BIT);		/* acknowledge */
	poll_next_us += POLL_PERIOD_US;
	if ((W)(poll_next_us - now) <= 0) {
		poll_next_us = now + POLL_PERIOD_US;	/* we were late: resync */
	}
	out_w(TIMER_ALARM3, poll_next_us);		/* writing arms it */
	if (lvl != poll_level) {
		poll_level = lvl;
		push_edge(now, lvl);
	}
}
#endif

EXPORT void ir_set_wake_task(ID tid)
{
	edge_wake_tid = tid;		/* one aligned store */
}

EXPORT ER ir_init(void)
{
	edge_head = 0;
	edge_tail = 0;
	edge_overflow = 0;
	edge_total = 0;
	edge_wake_tid = 0;
	gpio_input_config(IR_LEFT_PIN, GPIO_PULL_UP);
	gpio_input_config(IR_CENTER_PIN, GPIO_PULL_UP);
	gpio_input_config(IR_RIGHT_PIN, GPIO_PULL_UP);
#if !S3_CAPTURE_POLL_1MS
	return gpio_irq_register(IR_RIGHT_PIN, GPIO_IRQ_EDGES, ir_right_edge);
#else
	return E_OK;
#endif
}

EXPORT ER ir_capture_start(void)
{
#if S3_CAPTURE_POLL_1MS
	T_DINT	dint;
	ER	er;

	dint.intatr = TA_HLNG;
	dint.inthdr = (FP)ir_poll_handler;
	er = tk_def_int(POLL_ALARM_INTNO, &dint);
	if (er < E_OK) {
		return er;
	}
	poll_level = (in_w(GPIO_IN) >> IR_RIGHT_PIN) & 1UL;
	poll_next_us = hw_time_us() + POLL_PERIOD_US;
	out_w(TIMER_INTR, POLL_ALARM_BIT);
	set_w(TIMER_INTE, POLL_ALARM_BIT);
	out_w(TIMER_ALARM3, poll_next_us);
	ClearInt(POLL_ALARM_INTNO);
	EnableInt(POLL_ALARM_INTNO, GPIO_IRQ_LEVEL);
#endif
	return E_OK;
}

EXPORT UB ir_read_raw(void)
{
	UW	in = in_w(GPIO_IN);		/* one read: consistent snapshot */

	return (UB)(((in >> IR_LEFT_PIN) & 1UL)
		  | (((in >> IR_CENTER_PIN) & 1UL) << 1)
		  | (((in >> IR_RIGHT_PIN) & 1UL) << 2));
}

EXPORT BOOL ir_edge_get(UW *rec)
{
	UW	tail = edge_tail;

	if (tail == app_load(&edge_head)) {
		return FALSE;
	}
	*rec = edge_ring[tail];
	app_store(&edge_tail, (tail + 1U) & (IR_EDGE_RING - 1U));
	return TRUE;
}

EXPORT BOOL ir_edge_overflow_take(void)
{
	return (app_atomic_xchg(&edge_overflow, 0U) != 0U) ? TRUE : FALSE;
}

EXPORT void ir_edge_flush(void)
{
	UW	rec;

	while (ir_edge_get(&rec)) {
	}
}

EXPORT UW ir_edge_count(void)
{
	return app_load(&edge_total);
}
