/*
 *	drv_imu.h
 *	6-DoF IMU on I2C0 (Buddy 4).
 *
 *	!!! HARDWARE CONFIRMATION REQUIRED !!!  No supplied document names the
 *	IMU chip (Week 6: "IMU (6-DoF)" on Grove 4; Tribunal 1 Buddy 4: "I
 *	still need to confirm which sensors the IMU module provides").  The
 *	driver therefore does NOT assume one: it probes the WHO_AM_I register
 *	of the two common 6-DoF families sold on Grove/breakout boards and
 *	uses the register map of the family that answers:
 *	  InvenSense MPU-6050/6500/9250/9255, ICM-20600/20602
 *	      address 0x68/0x69, WHO_AM_I @0x75
 *	  ST LSM6DS3 / LSM6DS3TR-C / LSM6DSL / LSM6DSO
 *	      address 0x6A/0x6B, WHO_AM_I @0x0F
 *	If neither answers, imu_probe() fails and Buddy 4 reports an IMU
 *	fault; it never invents data.  Verify the register values used in
 *	drv_imu.c against the datasheet of the module actually fitted.
 *
 *	Output is in the CHIP's axes, physical units: mg and mdps.
 */

#ifndef DRV_IMU_H
#define DRV_IMU_H

#include <tk/tkernel.h>

#define IMU_CHIP_NONE		0U
#define IMU_CHIP_MPU		1U	/* InvenSense family */
#define IMU_CHIP_LSM6		2U	/* ST family         */

#define IMU_ACC_FS_MG		8000	/* +-8 g configured      */
#define IMU_GYR_FS_MDPS		500000	/* +-500 dps configured  */

typedef struct {
	W	acc_mg[3];	/* chip X, Y, Z */
	W	gyr_mdps[3];
	BOOL	saturated;	/* a raw value hit full scale */
	UH	raw_hash;	/* changes when any raw byte changes (stuck check) */
} IMU_SAMPLE;

IMPORT ER   imu_probe(UB *chip, UB *addr, UB *whoami);
IMPORT ER   imu_configure(void);
IMPORT ER   imu_read(IMU_SAMPLE *s);
IMPORT UB   imu_chip(void);

#endif /* DRV_IMU_H */
