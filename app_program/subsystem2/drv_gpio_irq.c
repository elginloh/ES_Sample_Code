/*
 *	drv_gpio_irq.c
 *	Shared IO_IRQ_BANK0 dispatcher (see drv_gpio_irq.h).
 *
 *	IO_BANK0 interrupt registers (RP2040 datasheet 2.19.6, offsets checked
 *	against pico-sdk hardware/regs/io_bank0.h): four bits per GPIO,
 *	eight GPIOs per 32-bit register.
 *	    INTR0..3        0x0F0  raw status, edge bits write-1-to-clear
 *	    PROC0_INTE0..3  0x100  enable for core 0
 *	    PROC0_INTS0..3  0x120  masked status for core 0
 *	    PROC1_INTE0..3  0x130  enable for core 1
 *	    PROC1_INTS0..3  0x150  masked status for core 1
 */

#include "subsystem2.h"

#define IOB_INTR(n)		(IO_BANK0_BASE + 0x0F0UL + 4UL * (n))
#define IOB_INTE(core, n)	(IO_BANK0_BASE + ((core) ? 0x130UL : 0x100UL) + 4UL * (n))
#define IOB_INTS(core, n)	(IO_BANK0_BASE + ((core) ? 0x150UL : 0x120UL) + 4UL * (n))
#define IOB_REG_OF(pin)		((UW)(pin) >> 3)
#define IOB_SHIFT_OF(pin)	(((UW)(pin) & 7UL) * 4UL)

typedef struct {
	UB		pin;
	UB		events;
	GPIO_IRQ_FN	fn;
} GPIO_IRQ_ENTRY;

LOCAL GPIO_IRQ_ENTRY	irq_tab[GPIO_IRQ_MAX_PINS];
LOCAL UINT		irq_n;			/* entries used (init phase only) */
LOCAL UINT		irq_core;		/* core that takes the interrupt */
LOCAL volatile UW	irq_started;
LOCAL volatile UW	irq_count;		/* written by the handler only */

EXPORT void gpio_irq_init(void)
{
	irq_n = 0;
	irq_started = 0;
	irq_count = 0;
}

EXPORT void gpio_input_config(UINT pin, UINT pull)
{
	UW	pad = GPIO_IE | GPIO_OD | GPIO_SHEMITT | GPIO_DRIVE_4MA;

	if (pin >= GPIO_NUM) {
		return;
	}
	if (pull == GPIO_PULL_UP) {
		pad |= GPIO_PUE;
	} else if (pull == GPIO_PULL_DOWN) {
		pad |= GPIO_PDE;
	}
	out_w(GPIO_OE_CLR, 1UL << pin);			/* never drive it */
	out_w(GPIO(pin), pad);
	out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_SIO);
}

EXPORT ER gpio_irq_register(UINT pin, UINT events, GPIO_IRQ_FN fn)
{
	UINT	i;

	if (pin >= GPIO_NUM || fn == NULL || (events & GPIO_IRQ_EDGES) == 0U) {
		return E_PAR;
	}
	if (irq_started != 0U) {
		return E_OBJ;			/* table is frozen once running */
	}
	for (i = 0; i < irq_n; i++) {
		if (irq_tab[i].pin == pin) {
			return E_OBJ;		/* one owner per pin (TEAM.md) */
		}
	}
	if (irq_n >= GPIO_IRQ_MAX_PINS) {
		return E_LIMIT;
	}
	irq_tab[irq_n].pin = (UB)pin;
	irq_tab[irq_n].events = (UB)(events & GPIO_IRQ_EDGES);
	irq_tab[irq_n].fn = fn;
	irq_n++;
	return E_OK;
}

/* The shared handler: read the time once, acknowledge each pending edge
   before calling its owner (so an edge during the callback is kept). */
LOCAL void gpio_irq_handler(UINT intno)
{
	UW	now = hw_time_us();
	UW	ints[4];
	UINT	i;

	(void)intno;
	for (i = 0; i < 4U; i++) {
		ints[i] = in_w(IOB_INTS(irq_core, i));
	}
	for (i = 0; i < irq_n; i++) {
		UW	pin = irq_tab[i].pin;
		UW	sh = IOB_SHIFT_OF(pin);
		UW	ev = (ints[IOB_REG_OF(pin)] >> sh) & irq_tab[i].events;

		if (ev != 0U) {
			out_w(IOB_INTR(IOB_REG_OF(pin)), ev << sh);	/* W1C */
			irq_tab[i].fn((UINT)pin, (UINT)ev, now);
		}
	}
	irq_count = irq_count + 1U;
}

EXPORT ER gpio_irq_start(void)
{
	T_DINT	dint;
	UINT	i;
	ER	er;

	if (irq_started != 0U) {
		return E_OK;
	}
	irq_core = hw_core_id();

	dint.intatr = TA_HLNG;
	dint.inthdr = (FP)gpio_irq_handler;
	er = tk_def_int(GPIO_IRQ_INTNO, &dint);
	if (er < E_OK) {
		return er;
	}
	for (i = 0; i < irq_n; i++) {
		UW	pin = irq_tab[i].pin;
		UW	bits = (UW)irq_tab[i].events << IOB_SHIFT_OF(pin);

		out_w(IOB_INTR(IOB_REG_OF(pin)), bits);		/* drop stale edges */
		set_w(IOB_INTE(irq_core, IOB_REG_OF(pin)), bits);
	}
	ClearInt(GPIO_IRQ_INTNO);
	EnableInt(GPIO_IRQ_INTNO, GPIO_IRQ_LEVEL);
	app_store(&irq_started, 1U);
	return E_OK;
}

EXPORT BOOL gpio_irq_running(void)
{
	return (app_load(&irq_started) != 0U) ? TRUE : FALSE;
}

/* Mask/unmask one registered pin at run time (atomic set/clear aliases,
   safe from any core). */
EXPORT void gpio_irq_enable_pin(UINT pin, BOOL on)
{
	UINT	i;

	if (!gpio_irq_running()) {
		return;			/* gpio_irq_start() enables every pin */
	}
	for (i = 0; i < irq_n; i++) {
		if (irq_tab[i].pin == pin) {
			UW	bits = (UW)irq_tab[i].events << IOB_SHIFT_OF(pin);

			if (on) {
				out_w(IOB_INTR(IOB_REG_OF(pin)), bits);
				set_w(IOB_INTE(irq_core, IOB_REG_OF(pin)), bits);
			} else {
				clr_w(IOB_INTE(irq_core, IOB_REG_OF(pin)), bits);
			}
			return;
		}
	}
}

EXPORT UW gpio_irq_count(void)
{
	return app_load(&irq_count);
}
