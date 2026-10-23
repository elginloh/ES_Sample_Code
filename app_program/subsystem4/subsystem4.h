/*
 *	subsystem4.h
 *	Subsystem 4 -- IMU-based motion and terrain monitoring (Buddy 4,
 *	student ID 2500320).
 *
 *	Tasks (Week 6 task table, Tribunal 1 interface map):
 *	  IMU Monitoring          prio 13, core 1, 10 ms cyclic (High);
 *	                          the ONLY user of I2C0 / the IMU
 *	  Hump & Terrain Analysis prio 18, queue-driven (Medium); no hardware
 *	Neither task ever sends motor commands; Buddy 4 only reports.
 *
 *	Interfaces provided:
 *	  IF-I1  IMU status snapshot (2 packed words + seq)  s4_get_imu_status()
 *	  IF-I2  hump status word                            s4_get_hump_status()
 *	  IF-I3  events to Buddy 2: SYS_EV_IMU_CAL_OK/FAIL, SYS_EV_COLLISION;
 *	         faults SYS_FLT_IMU, SYS_FLT_IMU_CAL, SYS_FLT_COLLISION
 *	Interfaces used: motion_get_odometry() (encoder speed, Buddy 2),
 *	sys_vsw() (vehicle mode: calibrate only while stationary in INIT).
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "../subsystem2/subsystem2.h"
#include "imu_proc.h"
#include "hump.h"
#include "s4_status.h"

#define S4_PRI_IMU		13
#define S4_PRI_HUMP		18
#define S4_PERIOD_MS		10	/* Week 6: IMU Monitoring every 10 ms */
#define S4_HUMP_QUEUE_LEN	32	/* samples (320 ms) IMU -> Hump task  */
#define S4_RECOVERY_MS		1000	/* retry interval in FAULT            */
#define S4_RECOVERY_TRIES	3	/* bounded recovery                   */
#define S4_CAL_TRIES		3	/* "car moved" retries                */

/* flg_imu bits (private to subsystem 4) */
#define IMU_EV_TICK		0x01U
#define IMU_EV_I2C		0x02U

IMPORT ID	flg_imu;
IMPORT ID	mbf_imu_hump;
IMPORT ID	cyc_imu;
IMPORT ID	tid_imu;
IMPORT ID	tid_hump;

IMPORT void s4_get_imu_status(S4_IMU_STATUS *s);
IMPORT void s4_get_hump_status(S4_HUMP_STATUS *s);
IMPORT W    s4_max_hump_um(void);		/* full resolution (FR6) */

/* Private to subsystem 4 */
IMPORT TASK_STAT	s4_stat_imu;
IMPORT APP_SEQ		s4_imu_seq;
IMPORT UW		s4_imu_words[2];	/* writer: IMU task  */
IMPORT UW		s4_imu_time;		/* writer: IMU task  */
IMPORT volatile UW	s4_hump_word;		/* writer: hump task */
IMPORT volatile UW	s4_max_hump;		/* writer: hump task */
IMPORT volatile UW	s4_dropped;		/* writer: IMU task  */
IMPORT void imu_task(INT stacd, void *exinf);
IMPORT void imu_cyclic_handler(void *exinf);
IMPORT void hump_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM4_H */
