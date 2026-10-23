/*
 *	s2_tests.h
 *	Test helpers shared by subsystem 2's unit tests and the cross-
 *	subsystem integration tests (test builds only).
 */

#ifndef S2_TESTS_H
#define S2_TESTS_H

#include <tk/typedef.h>
#include "motion_ctrl.h"

typedef struct {
	W	v_l;		/* wheel speeds, mm/s */
	W	v_r;
	W	d_l_um;		/* travel, um */
	W	d_r_um;
	BOOL	stuck_l;
	BOOL	stuck_r;
} PLANT;

IMPORT void plant_init(PLANT *p);
IMPORT void plant_step(PLANT *p, W duty_l, W duty_r, UW dt_ms);
IMPORT void plant_input(const PLANT *p, MC_IN *in, UW vmode);
IMPORT UW   run_until_result(MOTION_CTRL *mc, PLANT *p, UW vmode, UW src,
			     UW max_ms, UW *res);

IMPORT INT  s2_run_unit_tests(void);
IMPORT INT  s2_run_integration_tests(void);

#endif /* S2_TESTS_H */
