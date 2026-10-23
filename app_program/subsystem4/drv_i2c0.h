/*
 *	drv_i2c0.h
 *	Register-level, interrupt-driven I2C0 master for the IMU (Buddy 4).
 *	SDA = GP16, SCL = GP17 (Week 6 pin table, Grove 4).
 *
 *	Why our own driver: the kernel's I2C device driver is disabled in
 *	config/config_device.h (DEVCNF_USE_IIC 0) and enabling it changes a
 *	protected, team-shared file.  The README says subsystem drivers live
 *	in the subsystem folder as drv_<name>.c.  Tribunal 1: "IMU Monitoring
 *	runs every 10 ms and is the only task that uses the IMU on I2C0", so
 *	these functions are called from the IMU Monitoring task only.
 *
 *	A register read pre-loads every command into the 16-entry TX FIFO and
 *	the caller SLEEPS on its event flag until RX_FULL / TX_ABRT, so a
 *	14-byte read costs ~0.4 ms of bus time but almost no CPU (NFR13: no
 *	blocking of other tasks).  Every wait is bounded.
 */

#ifndef DRV_I2C0_H
#define DRV_I2C0_H

#include <tk/tkernel.h>

#define I2C0_SDA_PIN		16
#define I2C0_SCL_PIN		17
#define I2C0_INTNO		23	/* I2C0_IRQ */
#define I2C0_BAUD_HZ		400000UL
#define I2C0_TIMEOUT_MS		20
#define I2C0_MAX_READ		15	/* 16-entry TX FIFO minus the register byte */

/* done_flg/done_bit: the owner task's event flag and the bit the
   interrupt handler sets when a transfer ends */
IMPORT ER   i2c0_init(ID done_flg, UINT done_bit);
IMPORT ER   i2c0_write_reg(UB addr, UB reg, UB val);
IMPORT ER   i2c0_read_regs(UB addr, UB reg, UB *buf, UINT n);
IMPORT void i2c0_bus_recover(void);	/* 9 SCL pulses + STOP */
IMPORT UW   i2c0_last_abort(void);	/* TX_ABRT_SOURCE of the last failure */

#endif /* DRV_I2C0_H */
