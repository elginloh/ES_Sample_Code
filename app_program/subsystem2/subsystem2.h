/*
 *	subsystem2.h
 *	Subsystem 2 -- Motion Control (Buddy 2, student ID 2502591).
 *	Motors, wheel encoders, PID speed control, encoder-based turning;
 *	owner of the vehicle operating mode, the shared fault bitmask and the
 *	shared GPIO interrupt (Tribunal 1, Version B).
 *
 *	Public interface: other subsystems include "../subsystem2/subsystem2.h".
 *	Buddy 2 is the SOLE owner of motor PWM/direction (GP8..GP11).  Other
 *	subsystems move the car only through this interface:
 *	  IF-M1  discrete commands      motion_command()/motion_request()
 *	                                -> mbf_motion_cmd (32-bit packed words)
 *	  IF-M2  line steering          motion_set_steer()  (Buddy 3 only)
 *	  IF-M3  completion results     motion_wait_result() <- flg_motion_done
 *	  IF-M4  stop() bypass          motion_stop()        (any task / ISR)
 *	  IF-M5  obstacle speed cap     motion_set_speed_cap() (Buddy 5 only)
 *	  IF-S1  events to Buddy 2      sys_post_event()
 *	  IF-S2  shared fault bitmask   sys_fault_set()/clear()/get()
 *	  IF-S3  vehicle status word    sys_vsw() / sys_mode() / sys_epoch()
 *	  IF-S4  status for telemetry   motion_get_status(), motion_get_odometry()
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include "../app.h"
#include "sys_defs.h"
#include "motion_msg.h"
#include "app_sync.h"
#include "drv_hwtime.h"
#include "drv_gpio_irq.h"
#include "task_stat.h"

/* ---------------------------------------------------------------------
 *  Task placement (Week 6 task table, Tribunal 1: distinct priorities,
 *  fixed periods).  High-class tasks run on core 1, away from the
 *  kernel's USB (prio 3) and CYW43 (prio 4) service tasks on core 0.
 *  app_start_task() drops the affinity on the single-core build.
 * --------------------------------------------------------------------- */
#define S2_PRI_MOTION		12
#define APP_CORE_CONTROL	TP_PRC2
#define APP_CORE_COMMS		TP_PRC1
#define APP_CORE_ANY		0

/* Message buffer size for n messages of sz bytes (kernel stores a 4-byte
   header and rounds each message up to 4 bytes, messagebuf.c). */
#define APP_MBF_BUFSZ(n, sz)	((n) * (4 + (((sz) + 3) & ~3)))

/* Milliseconds since boot (10 ms kernel resolution) */
Inline UW app_now_ms(void)
{
	SYSTIM	t;

	(void)tk_get_otm(&t);
	return t.lo;
}

/* ---------------------------------------------------------------------
 *  Kernel objects owned by subsystem 2
 * --------------------------------------------------------------------- */
#define MF_TICK		0x01U	/* 20 ms control period            */
#define MF_STOP		0x02U	/* motion_stop() request           */
#define MF_EVENT	0x04U	/* sys_post_event()                */
#define MF_CMD		0x08U	/* command queued                  */
#define MF_FAULT	0x10U	/* new critical fault bit          */
#define MF_ALL		0x1FU

#define MOTION_CMD_QUEUE_LEN	16	/* command words buffered (IF-M1) */

IMPORT ID	flg_motion;		/* private: Motion Control wake-up  */
IMPORT ID	flg_motion_done;	/* IF-M3: bit (1 << MSRC_x)         */
IMPORT ID	mbf_motion_cmd;		/* IF-M1: 4-byte command words      */
IMPORT ID	cyc_motion;
IMPORT ID	tid_motion;

/* ---------------------------------------------------------------------
 *  Motion API (motion_api.c)
 * --------------------------------------------------------------------- */
/* Build a command word (fills seq + current epoch).  ONE producer task
   per source: barcode task, avoidance task, communication task. */
IMPORT UW   motion_make_cmd(UW type, UW src, INT arg, UW speed_lvl);
/* Queue a command without blocking: E_OK, or E_TMOUT if the queue is full */
IMPORT ER   motion_request(UW cmd);
IMPORT ER   motion_command(UW type, UW src, INT arg, UW speed_lvl, UW *cmd_out);
/* Wait for the result of 'cmd' (done/aborted/fault/rejected); tmo in ms */
IMPORT ER   motion_wait_result(UW cmd, TMO tmo, UW *result);
IMPORT UW   motion_last_result(UW src);
/* stop(): bypasses the queue, flushes it, output zero within one
   wake-up of the Motion Control task (NFR6: < 100 ms).  ISR-safe. */
IMPORT void motion_stop(UW src);
/* Line steering, -63..63 (+ = right), speed fraction 0..15 of cruise */
IMPORT void motion_set_steer(INT steer, UW speed_frac);
IMPORT void motion_clear_steer(void);
/* Obstacle caution speed cap, mm/s; 0 removes it */
IMPORT void motion_set_speed_cap(UW cap_mms);
IMPORT void motion_get_status(MOTION_STATUS *st);
/* Average wheel travel since boot (mm, signed) and speed (mm/s) */
IMPORT void motion_get_odometry(W *dist_mm, W *speed_mms);
IMPORT W    motion_creep_speed_mms(void);
IMPORT void motion_set_armed(BOOL armed);	/* bench / test use */

/* ---------------------------------------------------------------------
 *  System state API (sys_state.c)
 * --------------------------------------------------------------------- */
IMPORT UW   sys_vsw(void);
IMPORT UW   sys_mode(void);
IMPORT UW   sys_epoch(void);
IMPORT UW   sys_fault_get(void);
IMPORT void sys_fault_set(UW bits);		/* ISR-safe, atomic */
IMPORT void sys_fault_clear(UW bits);		/* ISR-safe, atomic */
IMPORT UW   sys_fault_crit_mask(void);
IMPORT void sys_post_event(UW ev);		/* ISR-safe */

/* Diagnostics (task_motion.c) */
IMPORT TASK_STAT	s2_stat_motion;

/* Tasks and handlers */
IMPORT void motion_task(INT stacd, void *exinf);
IMPORT void motion_cyclic_handler(void *exinf);

#endif /* SUBSYSTEM2_H */
