/*
 *	drv_imu.c
 *	IMU probing, configuration and reading (see drv_imu.h).
 *
 *	InvenSense map (MPU-6050 register map rev 4.2 and successors):
 *	  0x19 SMPLRT_DIV = 9      -> 100 Hz with the DLPF on
 *	  0x1A CONFIG     = 0x03   -> DLPF ~44 Hz (vibration rejection, NFR14)
 *	  0x1B GYRO_CFG   = 0x08   -> +-500 dps, 65.5 LSB/dps
 *	  0x1C ACCEL_CFG  = 0x10   -> +-8 g, 4096 LSB/g
 *	  0x6B PWR_MGMT_1 = 0x80 reset, then 0x01 (PLL on gyro X)
 *	  0x3B..0x48 ACCEL_XOUT_H..GYRO_ZOUT_L, big-endian
 *	ST map (LSM6DS3 datasheet):
 *	  0x12 CTRL3_C    = 0x01 reset, then 0x44 (BDU, auto-increment)
 *	  0x10 CTRL1_XL   = 0x4C   -> 104 Hz, +-8 g, 0.244 mg/LSB
 *	  0x11 CTRL2_G    = 0x44   -> 104 Hz, 500 dps, 17.5 mdps/LSB
 *	  0x22..0x2D OUTX_L_G..OUTZ_H_XL, little-endian
 *	ST LSM303DLHC (datasheet DocID018771), accelerometer at 0x19:
 *	  0x24 CTRL_REG5_A = 0x80 reboot memory content
 *	  0x20 CTRL_REG1_A = 0x57  -> 100 Hz, normal mode, X/Y/Z on
 *	  0x23 CTRL_REG4_A = 0xA8  -> BDU, +-8 g, high resolution:
 *	                              12-bit left-justified, 4 mg/LSB
 *	  0x28..0x2D OUT_X_L_A..OUT_Z_H_A, little-endian; a multi-byte
 *	                              read needs bit 7 of the register set
 *	magnetometer at 0x1E:
 *	  0x00 CRA_REG_M   = 0x1C  -> 220 Hz, so every 10 ms read is fresh
 *	  0x01 CRB_REG_M   = 0x40  -> +-1.9 gauss: X/Y 855, Z 760 LSB/gauss
 *	  0x02 MR_REG_M    = 0x00  -> continuous conversion
 *	  0x03..0x08 OUT_X_H_M, X_L, Z_H, Z_L, Y_H, Y_L: big-endian and in
 *	                              the order X, Z, Y; -4096 = overflow
 */

#include "subsystem4.h"
#include "drv_imu.h"
#include "drv_i2c0.h"

#define MPU_REG_SMPLRT_DIV	0x19U
#define MPU_REG_CONFIG		0x1AU
#define MPU_REG_GYRO_CFG	0x1BU
#define MPU_REG_ACCEL_CFG	0x1CU
#define MPU_REG_DATA		0x3BU
#define MPU_REG_PWR_MGMT_1	0x6BU
#define MPU_REG_WHO_AM_I	0x75U

#define LSM_REG_WHO_AM_I	0x0FU
#define LSM_REG_CTRL1_XL	0x10U
#define LSM_REG_CTRL2_G		0x11U
#define LSM_REG_CTRL3_C		0x12U
#define LSM_REG_DATA		0x22U

#define L303_ACC_ADDR		0x19U
#define L303_MAG_ADDR		0x1EU
#define L303_REG_CTRL1_A	0x20U
#define L303_REG_CTRL4_A	0x23U
#define L303_REG_CTRL5_A	0x24U
#define L303_REG_OUT_A		0x28U
#define L303_AUTO_INC		0x80U	/* accelerometer register auto-increment */
#define L303_REG_CRA_M		0x00U
#define L303_REG_CRB_M		0x01U
#define L303_REG_MR_M		0x02U
#define L303_REG_OUT_M		0x03U
#define L303_REG_IRA_M		0x0AU
#define L303_MAG_XY_LSB		855	/* LSB/gauss at +-1.9 gauss */
#define L303_MAG_Z_LSB		760
#define L303_MAG_OVERFLOW	(-4096)

LOCAL UB	chip = IMU_CHIP_NONE;
LOCAL UB	chip_addr;

LOCAL const UB mpu_ids[] = { 0x68, 0x70, 0x71, 0x73, 0x11, 0x12 };
LOCAL const UB lsm_ids[] = { 0x69, 0x6A, 0x6C };

LOCAL BOOL id_in(UB v, const UB *list, UINT n)
{
	UINT	i;

	for (i = 0; i < n; i++) {
		if (list[i] == v) {
			return TRUE;
		}
	}
	return FALSE;
}

EXPORT UB imu_chip(void)
{
	return chip;
}

EXPORT ER imu_probe(UB *out_chip, UB *out_addr, UB *out_who)
{
	static const UB	mpu_addr[2] = { 0x68, 0x69 };
	static const UB	lsm_addr[2] = { 0x6A, 0x6B };
	UB		who = 0;
	UINT		i;

	chip = IMU_CHIP_NONE;
	for (i = 0; i < 2U; i++) {
		if (i2c0_read_regs(mpu_addr[i], MPU_REG_WHO_AM_I, &who, 1U) == E_OK
		    && id_in(who, mpu_ids, sizeof(mpu_ids))) {
			chip = IMU_CHIP_MPU;
			chip_addr = mpu_addr[i];
			break;
		}
		if (i2c0_read_regs(lsm_addr[i], LSM_REG_WHO_AM_I, &who, 1U) == E_OK
		    && id_in(who, lsm_ids, sizeof(lsm_ids))) {
			chip = IMU_CHIP_LSM6;
			chip_addr = lsm_addr[i];
			break;
		}
	}
	if (chip == IMU_CHIP_NONE) {
		UB	id[3] = {0, 0, 0};

		/* LSM303DLHC: identified by the magnetometer's 'H','4','3';
		   the accelerometer must answer too */
		if (i2c0_read_regs(L303_MAG_ADDR, L303_REG_IRA_M, id, 3U) == E_OK
		    && id[0] == 'H' && id[1] == '4' && id[2] == '3'
		    && i2c0_read_regs(L303_ACC_ADDR, L303_REG_CTRL1_A, &who, 1U) == E_OK) {
			chip = IMU_CHIP_LSM303;
			chip_addr = L303_ACC_ADDR;
		}
		who = id[0];
	}
	*out_chip = chip;
	*out_addr = (chip != IMU_CHIP_NONE) ? chip_addr : 0U;
	*out_who = who;
	return (chip != IMU_CHIP_NONE) ? E_OK : E_IO;
}

LOCAL ER wr(UB reg, UB val)
{
	return i2c0_write_reg(chip_addr, reg, val);
}

EXPORT ER imu_configure(void)
{
	ER	er = E_IO;

	if (chip == IMU_CHIP_MPU) {
		er = wr(MPU_REG_PWR_MGMT_1, 0x80);		/* device reset */
		(void)tk_dly_tsk(100);
		if (er == E_OK) er = wr(MPU_REG_PWR_MGMT_1, 0x01);
		if (er == E_OK) er = wr(MPU_REG_SMPLRT_DIV, 9);
		if (er == E_OK) er = wr(MPU_REG_CONFIG, 0x03);
		if (er == E_OK) er = wr(MPU_REG_GYRO_CFG, 0x08);
		if (er == E_OK) er = wr(MPU_REG_ACCEL_CFG, 0x10);
	} else if (chip == IMU_CHIP_LSM6) {
		er = wr(LSM_REG_CTRL3_C, 0x01);			/* software reset */
		(void)tk_dly_tsk(20);
		if (er == E_OK) er = wr(LSM_REG_CTRL3_C, 0x44);
		if (er == E_OK) er = wr(LSM_REG_CTRL1_XL, 0x4C);
		if (er == E_OK) er = wr(LSM_REG_CTRL2_G, 0x44);
	} else if (chip == IMU_CHIP_LSM303) {
		er = wr(L303_REG_CTRL5_A, 0x80);		/* reboot memory content */
		(void)tk_dly_tsk(20);
		if (er == E_OK) er = wr(L303_REG_CTRL1_A, 0x57);
		if (er == E_OK) er = wr(L303_REG_CTRL4_A, 0xA8);
		if (er == E_OK) er = i2c0_write_reg(L303_MAG_ADDR, L303_REG_CRA_M, 0x1C);
		if (er == E_OK) er = i2c0_write_reg(L303_MAG_ADDR, L303_REG_CRB_M, 0x40);
		if (er == E_OK) er = i2c0_write_reg(L303_MAG_ADDR, L303_REG_MR_M, 0x00);
	}
	(void)tk_dly_tsk(50);		/* first samples after power-up are noise */
	return er;
}

LOCAL H be16(const UB *p)
{
	return (H)(((UH)p[0] << 8) | p[1]);
}

LOCAL H le16(const UB *p)
{
	return (H)(((UH)p[1] << 8) | p[0]);
}

LOCAL BOOL at_full_scale(H v)
{
	return (v == 32767 || v == -32768) ? TRUE : FALSE;
}

/* LSM303DLHC: accelerometer and magnetometer read back to back */
LOCAL ER read_lsm303(IMU_SAMPLE *s)
{
	UB	b[6], m[6];
	H	a[3], mx, my, mz;
	UINT	i;
	UH	hash = 0;
	ER	er;

	er = i2c0_read_regs(L303_ACC_ADDR, L303_REG_OUT_A | L303_AUTO_INC, b, 6U);
	if (er < E_OK) {
		return er;
	}
	er = i2c0_read_regs(L303_MAG_ADDR, L303_REG_OUT_M, m, 6U);
	if (er < E_OK) {
		return er;
	}
	s->saturated = FALSE;
	for (i = 0; i < 3U; i++) {
		a[i] = (H)(le16(&b[2U * i]) >> 4);	/* 12-bit, left-justified */
		s->acc_mg[i] = (W)a[i] * 4;
		s->gyr_mdps[i] = 0;
		if (a[i] == 2047 || a[i] == -2048) {
			s->saturated = TRUE;
		}
	}
	mx = be16(&m[0]);
	mz = be16(&m[2]);
	my = be16(&m[4]);
	s->mag_mgauss[0] = (W)mx * 1000 / L303_MAG_XY_LSB;
	s->mag_mgauss[1] = (W)my * 1000 / L303_MAG_XY_LSB;
	s->mag_mgauss[2] = (W)mz * 1000 / L303_MAG_Z_LSB;
	s->mag_valid = (mx != L303_MAG_OVERFLOW && my != L303_MAG_OVERFLOW
			&& mz != L303_MAG_OVERFLOW) ? TRUE : FALSE;
	s->has_gyro = FALSE;
	for (i = 0; i < 6U; i++) {
		hash = (UH)((hash * 31U) ^ b[i] ^ ((UH)m[i] << 3));
	}
	s->raw_hash = hash;
	return E_OK;
}

EXPORT ER imu_read(IMU_SAMPLE *s)
{
	UB	b[14];
	H	a[3], g[3];
	UINT	i;
	UH	hash = 0;
	ER	er;

	if (chip == IMU_CHIP_LSM303) {
		return read_lsm303(s);
	}
	if (chip == IMU_CHIP_MPU) {
		er = i2c0_read_regs(chip_addr, MPU_REG_DATA, b, 14U);
		if (er < E_OK) {
			return er;
		}
		for (i = 0; i < 3U; i++) {
			a[i] = be16(&b[2U * i]);
			g[i] = be16(&b[8U + 2U * i]);	/* skip temperature */
		}
		for (i = 0; i < 3U; i++) {
			s->acc_mg[i] = (W)a[i] * 1000 / 4096;
			s->gyr_mdps[i] = (W)g[i] * 10000 / 655;
		}
	} else if (chip == IMU_CHIP_LSM6) {
		er = i2c0_read_regs(chip_addr, LSM_REG_DATA, b, 12U);
		if (er < E_OK) {
			return er;
		}
		for (i = 0; i < 3U; i++) {
			g[i] = le16(&b[2U * i]);
			a[i] = le16(&b[6U + 2U * i]);
		}
		for (i = 0; i < 3U; i++) {
			s->acc_mg[i] = (W)a[i] * 244 / 1000;
			s->gyr_mdps[i] = (W)g[i] * 175 / 10;
		}
	} else {
		return E_OBJ;
	}
	for (i = 0; i < 3U; i++) {
		s->mag_mgauss[i] = 0;
	}
	s->has_gyro = TRUE;
	s->mag_valid = FALSE;
	s->saturated = FALSE;
	for (i = 0; i < 3U; i++) {
		if (at_full_scale(a[i]) || at_full_scale(g[i])) {
			s->saturated = TRUE;
		}
		hash = (UH)((hash * 31U) ^ (UH)a[i] ^ ((UH)g[i] << 1));
	}
	s->raw_hash = hash;
	return E_OK;
}
