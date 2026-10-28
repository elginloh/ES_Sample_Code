/*
 *	drv_imu.h
 *	IMU on I2C0 (Buddy 4).
 *
 *	The module supplied by the course is a GY-511 board with an ST
 *	LSM303DLHC: 3-axis accelerometer + 3-axis magnetometer, NO gyroscope.
 *	The driver probes for it and for the two common 6-DoF families, and
 *	uses the register map of the chip that answers:
 *	  InvenSense MPU-6050/6500/9250/9255, ICM-20600/20602
 *	      address 0x68/0x69, WHO_AM_I @0x75
 *	  ST LSM6DS3 / LSM6DS3TR-C / LSM6DSL / LSM6DSO
 *	      address 0x6A/0x6B, WHO_AM_I @0x0F
 *	  ST LSM303DLHC (GY-511)
 *	      accelerometer 0x19, magnetometer 0x1E, IRA/IRB/IRC_REG_M
 *	      @0x0A..0x0C = 'H' '4' '3' (the accelerometer has no WHO_AM_I)
 *	If none answers, imu_probe() fails and Buddy 4 reports an IMU fault;
 *	it never invents data.  Verify the register values used in drv_imu.c
 *	against the datasheet of the module actually fitted.
 *
 *	Output is in the CHIP's axes, physical units: mg, mdps and mgauss.
 *	A chip without a gyro reports has_gyro = FALSE and gyro values of 0;
 *	imu_proc.c then takes attitude from the accelerometer alone and
 *	heading from the magnetometer.
 */

#ifndef DRV_IMU_H
#define DRV_IMU_H

#include <tk/tkernel.h>

#define IMU_CHIP_NONE		0U
#define IMU_CHIP_MPU		1U	/* InvenSense family */
#define IMU_CHIP_LSM6		2U	/* ST family         */
#define IMU_CHIP_LSM303		3U	/* ST LSM303DLHC, accel + mag, no gyro */

#define IMU_ACC_FS_MG		8000	/* +-8 g configured      */
#define IMU_GYR_FS_MDPS		500000	/* +-500 dps configured  */

typedef struct {
	W	acc_mg[3];	/* chip X, Y, Z */
	W	gyr_mdps[3];	/* 0 when has_gyro is FALSE */
	W	mag_mgauss[3];	/* chip X, Y, Z; meaningful when mag_valid */
	BOOL	has_gyro;
	BOOL	mag_valid;	/* chip has a magnetometer and it did not overflow */
	BOOL	saturated;	/* a raw value hit full scale */
	UH	raw_hash;	/* changes when any raw byte changes (stuck check) */
} IMU_SAMPLE;

IMPORT ER   imu_probe(UB *chip, UB *addr, UB *whoami);
IMPORT ER   imu_configure(void);
IMPORT ER   imu_read(IMU_SAMPLE *s);
IMPORT UB   imu_chip(void);

#endif /* DRV_IMU_H */
