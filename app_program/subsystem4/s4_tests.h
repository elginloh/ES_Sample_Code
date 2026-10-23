/*
 *	s4_tests.h
 *	Test helpers of subsystem 4 shared with the integration tests (test
 *	builds only).
 */

#ifndef S4_TESTS_H
#define S4_TESTS_H

#include <tk/typedef.h>
#include "imu_proc.h"
#include "hump.h"

IMPORT void s4_make_in(IMU_IN *s, W ax, W ay, W az, W gx, W gy, W gz);
IMPORT W    s4_hump_pitch_cdeg(W h_mm, W l_mm, W s_mm);
IMPORT void s4_drive_hump(HUMP *h, W h_mm, W l_mm, HUMP_EVT *last);
IMPORT INT  s4_run_unit_tests(void);

#endif /* S4_TESTS_H */
