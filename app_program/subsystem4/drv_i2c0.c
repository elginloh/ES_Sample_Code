/*
 *	drv_i2c0.c
 *	I2C0 master (see drv_i2c0.h).  Register map: RP2040 datasheet 4.3
 *	(DW_apb_i2c); offsets match device/i2c/sysdepend/rp2040/i2c_sysdep.h
 *	and pico-sdk hardware/regs/i2c.h.  Timing as in pico-sdk
 *	i2c_set_baudrate(): fast-mode counts, low = 3/5 of the period.
 */

#include "subsystem4.h"
#include "drv_i2c0.h"

#define I2C0_BASE		0x40044000UL
#define I2C_REG(off)		(I2C0_BASE + (off))
#define IC_CON			I2C_REG(0x00)
#define IC_TAR			I2C_REG(0x04)
#define IC_DATA_CMD		I2C_REG(0x10)
#define IC_FS_SCL_HCNT		I2C_REG(0x1C)
#define IC_FS_SCL_LCNT		I2C_REG(0x20)
#define IC_INTR_STAT		I2C_REG(0x2C)
#define IC_INTR_MASK		I2C_REG(0x30)
#define IC_RX_TL		I2C_REG(0x38)
#define IC_TX_TL		I2C_REG(0x3C)
#define IC_CLR_INTR		I2C_REG(0x40)
#define IC_CLR_TX_ABRT		I2C_REG(0x54)
#define IC_CLR_STOP_DET		I2C_REG(0x60)
#define IC_ENABLE		I2C_REG(0x6C)
#define IC_STATUS		I2C_REG(0x70)
#define IC_RXFLR		I2C_REG(0x78)
#define IC_SDA_HOLD		I2C_REG(0x7C)
#define IC_TX_ABRT_SOURCE	I2C_REG(0x80)
#define IC_ENABLE_STATUS	I2C_REG(0x9C)
#define IC_FS_SPKLEN		I2C_REG(0xA0)

#define CON_MASTER		(1UL << 0)
#define CON_SPEED_FAST		(2UL << 1)
#define CON_RESTART_EN		(1UL << 5)
#define CON_SLAVE_DISABLE	(1UL << 6)
#define CON_TX_EMPTY_CTRL	(1UL << 8)
#define CMD_READ		(1UL << 8)
#define CMD_STOP		(1UL << 9)
#define CMD_RESTART		(1UL << 10)
#define INT_RX_FULL		(1UL << 2)
#define INT_TX_ABRT		(1UL << 6)
#define INT_STOP_DET		(1UL << 9)
#define STATUS_ACTIVITY		(1UL << 0)
#define FUNCSEL_I2C		3UL
#define SPIN_MAX		20000U

LOCAL ID		done_flgid;
LOCAL UINT		done_ptn;
LOCAL volatile UW	irq_status;	/* writer: handler; reader: task (after wake) */
LOCAL volatile UW	last_abort;

LOCAL void i2c0_handler(UINT intno)
{
	UW	st = in_w(IC_INTR_STAT);

	(void)intno;
	if ((st & INT_TX_ABRT) != 0U) {
		last_abort = in_w(IC_TX_ABRT_SOURCE);
		(void)in_w(IC_CLR_TX_ABRT);
	}
	if ((st & INT_STOP_DET) != 0U) {
		(void)in_w(IC_CLR_STOP_DET);
	}
	out_w(IC_INTR_MASK, 0);		/* one completion per transfer */
	irq_status = st;
	(void)tk_set_flg(done_flgid, done_ptn);
}

LOCAL BOOL wait_disabled(void)
{
	UINT	n = SPIN_MAX;

	out_w(IC_ENABLE, 0);
	while ((in_w(IC_ENABLE_STATUS) & 1UL) != 0U) {
		if (n-- == 0U) {
			return FALSE;
		}
	}
	return TRUE;
}

LOCAL BOOL wait_idle(void)
{
	UINT	n = SPIN_MAX;

	while ((in_w(IC_STATUS) & STATUS_ACTIVITY) != 0U) {
		if (n-- == 0U) {
			return FALSE;
		}
	}
	return TRUE;
}

EXPORT ER i2c0_init(ID done_flg, UINT done_bit)
{
	UW	period = (TMCLK_MHz * 1000000UL + I2C0_BAUD_HZ / 2U) / I2C0_BAUD_HZ;
	UW	lcnt = period * 3U / 5U;
	UW	hcnt = period - lcnt;
	UW	n = SPIN_MAX;
	T_DINT	dint;
	ER	er;

	done_flgid = done_flg;
	done_ptn = done_bit;

	/* I2C0 is released from reset at boot (hw_setting.c, USE_SDEV_DRV);
	   make sure without ever asserting a reset someone else relies on */
	clr_w(RESETS_RESET, RESETS_RESET_I2C0);
	while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_I2C0) == 0U) {
		if (n-- == 0U) {
			return E_IO;
		}
	}
	if (!wait_disabled()) {
		return E_IO;
	}
	out_w(IC_CON, CON_SLAVE_DISABLE | CON_MASTER | CON_RESTART_EN
		    | CON_TX_EMPTY_CTRL | CON_SPEED_FAST);
	out_w(IC_FS_SCL_HCNT, hcnt);
	out_w(IC_FS_SCL_LCNT, lcnt);
	out_w(IC_FS_SPKLEN, (lcnt < 16U) ? 1U : lcnt / 16U);
	out_w(IC_SDA_HOLD, (TMCLK_MHz * 1000000UL * 3U) / 10000000UL + 1U);
	out_w(IC_TX_TL, 0);
	out_w(IC_INTR_MASK, 0);
	(void)in_w(IC_CLR_INTR);

	/* pads: input enable, pull-up, Schmitt; function = I2C */
	out_w(GPIO(I2C0_SDA_PIN), GPIO_IE | GPIO_PUE | GPIO_SHEMITT | GPIO_DRIVE_4MA);
	out_w(GPIO(I2C0_SCL_PIN), GPIO_IE | GPIO_PUE | GPIO_SHEMITT | GPIO_DRIVE_4MA);
	out_w(GPIO_CTRL(I2C0_SDA_PIN), FUNCSEL_I2C);
	out_w(GPIO_CTRL(I2C0_SCL_PIN), FUNCSEL_I2C);

	dint.intatr = TA_HLNG;
	dint.inthdr = (FP)i2c0_handler;
	er = tk_def_int(I2C0_INTNO, &dint);
	if (er < E_OK) {
		return er;
	}
	ClearInt(I2C0_INTNO);
	EnableInt(I2C0_INTNO, 2);
	return E_OK;
}

/* Common part: address the slave, queue the commands, sleep until the
   handler reports, check for an abort. */
LOCAL ER transfer(UB addr, const UW *cmds, UINT ncmd, UW wait_mask, UINT rx_n)
{
	UINT	i, ptn;
	ER	er;

	if (!wait_idle() || !wait_disabled()) {
		return E_IO;
	}
	out_w(IC_TAR, addr & 0x7FU);
	out_w(IC_RX_TL, (rx_n > 0U) ? (rx_n - 1U) : 0U);
	out_w(IC_ENABLE, 1);
	(void)in_w(IC_CLR_INTR);
	(void)tk_clr_flg(done_flgid, ~done_ptn);
	irq_status = 0;
	out_w(IC_INTR_MASK, wait_mask | INT_TX_ABRT);
	for (i = 0; i < ncmd; i++) {
		out_w(IC_DATA_CMD, cmds[i]);
	}
	er = tk_wai_flg(done_flgid, done_ptn, TWF_ORW | TWF_BITCLR, &ptn,
			I2C0_TIMEOUT_MS);
	out_w(IC_INTR_MASK, 0);
	if (er < E_OK) {
		(void)wait_disabled();		/* stop whatever is stuck */
		return E_TMOUT;
	}
	if ((irq_status & INT_TX_ABRT) != 0U) {
		return E_IO;			/* NACK / arbitration lost */
	}
	return E_OK;
}

EXPORT ER i2c0_write_reg(UB addr, UB reg, UB val)
{
	UW	cmds[2];

	cmds[0] = reg;
	cmds[1] = (UW)val | CMD_STOP;
	return transfer(addr, cmds, 2U, INT_STOP_DET, 0U);
}

EXPORT ER i2c0_read_regs(UB addr, UB reg, UB *buf, UINT n)
{
	UW	cmds[I2C0_MAX_READ + 1U];
	UINT	i;
	ER	er;

	if (n == 0U || n > I2C0_MAX_READ) {
		return E_PAR;
	}
	cmds[0] = reg;
	for (i = 0; i < n; i++) {
		cmds[i + 1U] = CMD_READ | ((i == 0U) ? CMD_RESTART : 0U)
			     | ((i + 1U == n) ? CMD_STOP : 0U);
	}
	er = transfer(addr, cmds, n + 1U, INT_RX_FULL, n);
	if (er < E_OK) {
		return er;
	}
	if (in_w(IC_RXFLR) < n) {
		return E_IO;
	}
	for (i = 0; i < n; i++) {
		buf[i] = (UB)(in_w(IC_DATA_CMD) & 0xFFU);
	}
	return E_OK;
}

/* Free a slave that holds SDA low (e.g. after a reset mid-transfer):
   clock SCL nine times with SDA released, then generate a STOP. */
EXPORT void i2c0_bus_recover(void)
{
	UINT	i;

	(void)wait_disabled();
	out_w(GPIO_OE_CLR, (1UL << I2C0_SDA_PIN) | (1UL << I2C0_SCL_PIN));
	out_w(GPIO_OUT_CLR, (1UL << I2C0_SDA_PIN) | (1UL << I2C0_SCL_PIN));
	out_w(GPIO_CTRL(I2C0_SDA_PIN), GPIO_CTRL_FUNCSEL_SIO);
	out_w(GPIO_CTRL(I2C0_SCL_PIN), GPIO_CTRL_FUNCSEL_SIO);
	for (i = 0; i < 9U; i++) {		/* open-drain: OE=1 pulls low */
		out_w(GPIO_OE_SET, 1UL << I2C0_SCL_PIN);
		(void)hw_delay_us(5);
		out_w(GPIO_OE_CLR, 1UL << I2C0_SCL_PIN);
		(void)hw_delay_us(5);
	}
	out_w(GPIO_OE_SET, 1UL << I2C0_SDA_PIN);	/* STOP: SDA low -> high */
	(void)hw_delay_us(5);
	out_w(GPIO_OE_CLR, 1UL << I2C0_SDA_PIN);
	(void)hw_delay_us(5);
	out_w(GPIO_CTRL(I2C0_SDA_PIN), FUNCSEL_I2C);
	out_w(GPIO_CTRL(I2C0_SCL_PIN), FUNCSEL_I2C);
}

EXPORT UW i2c0_last_abort(void)
{
	return last_abort;
}
