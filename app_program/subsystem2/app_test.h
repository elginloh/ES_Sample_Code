/*
 *	app_test.h
 *	On-target test support shared by the five subsystemN_test.c files
 *	(test image only: make TEST=1).  Owned by subsystem 2.
 *
 *	Why a separate test task: usermain() calls subsystemN_test() in the
 *	kernel's initial task, whose stack is only INITTASK_STKSZ = 1 KB
 *	(include/sys/inittask.h).  The unit and integration suites need more
 *	(barcode decoder buffers, obstacle scans, simulated wheels), so
 *	app_test_run() executes a whole test body in a temporary task with
 *	APP_TEST_STACK bytes of stack, below every application task, and
 *	returns its number of failed checks.  The initial task just sleeps
 *	meanwhile.
 */

#ifndef APP_TEST_H
#define APP_TEST_H

#include <tk/tkernel.h>
#include "task_stat.h"

#define APP_TEST_STACK		8192	/* bytes, from the kernel heap */
#define APP_TEST_PRI		22	/* lowest: never delays the application */
#define APP_TEST_TIMEOUT_MS	120000	/* a hung test body is reported, not waited for */

/* Run body() in the test task; returns its failed-check count (or 1 if
   the task could not be created or did not finish in time) */
IMPORT INT  app_test_run(INT (*body)(void), const char *name);

/* Let the USB console ring (4 KB) drain before printing more */
IMPORT void app_test_drain(void);

/* Activations of a periodic task during window_ms (its TASK_STAT) */
IMPORT UW   app_test_rate(const TASK_STAT *s, UW window_ms);

#endif /* APP_TEST_H */
