/*
 *	drv_gpio_irq.h
 *	The ONE shared GPIO interrupt handler (IO_IRQ_BANK0) and GPIO input
 *	pad configuration.
 *
 *	Tribunal 1 shared-resource matrix: "GPIO interrupt (shared by encoders
 *	and barcode) ... Keep every handler to counting or timestamping only".
 *	On RP2040 all GPIO edges arrive on one vector per core, so the
 *	encoders (Buddy 2, GP0/GP7), the barcode sensor (Buddy 3, GP27) and
 *	the ultrasonic echo (Buddy 5, GP5) register small callbacks here
 *	instead of each defining their own handler.
 *
 *	Rules for callbacks: run in interrupt context, a few microseconds,
 *	no tm_printf, no waiting, only ISR-safe kernel calls (tk_set_flg,
 *	tk_wup_tsk, tk_sig_sem).  'now_us' is the TIMER value read once on
 *	entry, so every pin serviced in one interrupt gets the same time.
 *
 *	Usage: gpio_irq_register() from subsystemN_init() only; the Motion
 *	Control task calls gpio_irq_start() once, so the interrupt is taken
 *	on that task's core (core 1 on the SMP build).
 */

#ifndef DRV_GPIO_IRQ_H
#define DRV_GPIO_IRQ_H

#include <tk/tkernel.h>

#define GPIO_IRQ_EDGE_FALL	0x4U	/* IO_BANK0 INTR "EDGE_LOW"  */
#define GPIO_IRQ_EDGE_RISE	0x8U	/* IO_BANK0 INTR "EDGE_HIGH" */
#define GPIO_IRQ_EDGES		(GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE)

#define GPIO_IRQ_MAX_PINS	8
#define GPIO_IRQ_INTNO		13	/* IO_IRQ_BANK0 (RP2040 datasheet) */
#define GPIO_IRQ_LEVEL		1	/* highest external priority level  */

#define GPIO_PULL_NONE		0U
#define GPIO_PULL_UP		1U
#define GPIO_PULL_DOWN		2U

typedef void (*GPIO_IRQ_FN)(UINT pin, UINT events, UW now_us);

IMPORT void gpio_irq_init(void);
IMPORT ER   gpio_irq_register(UINT pin, UINT events, GPIO_IRQ_FN fn);
IMPORT ER   gpio_irq_start(void);
IMPORT BOOL gpio_irq_running(void);
IMPORT void gpio_irq_enable_pin(UINT pin, BOOL on);
IMPORT UW   gpio_irq_count(void);	/* interrupts taken (diagnostics) */

/* SIO input with Schmitt trigger and the given pull */
IMPORT void gpio_input_config(UINT pin, UINT pull);

#endif /* DRV_GPIO_IRQ_H */
