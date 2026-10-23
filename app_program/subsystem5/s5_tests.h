/*
 *	s5_tests.h
 *	Test helpers of subsystem 5 shared with the integration tests (test
 *	builds only).
 */

#ifndef S5_TESTS_H
#define S5_TESTS_H

#include <tk/typedef.h>
#include "obstacle_profile.h"

IMPORT UINT s5_synth_scan(OP_SAMPLE *o, W xl, W xr, W y, W wall_x,
			  INT from, INT to, INT step, UINT n);
IMPORT void s5_profile_box(W xl, W xr, W y, W wall_x, S5_PROFILE *p);
IMPORT INT  s5_run_unit_tests(void);

#endif /* S5_TESTS_H */
