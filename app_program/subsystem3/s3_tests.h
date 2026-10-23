/*
 *	s3_tests.h
 *	Test helpers of subsystem 3 shared with the integration tests (test
 *	builds only).
 */

#ifndef S3_TESTS_H
#define S3_TESTS_H

#include <tk/typedef.h>
#include "barcode_dec.h"

/* Drive the simulated car over "*<c>*" printed at track position
   start_um (3 mm narrow / 9 mm wide, 0.6 mm optical blur, +-15 % speed
   ripple of phase phase_um, encoder speed reading 5 % high) and feed the
   decoder exactly as the Barcode Decoding task does; *t_us and *x_um
   carry time and position from one call to the next. */
IMPORT void s3_drive_over(BC_DEC *d, char c, W v0_mms, W start_um, W phase_um,
			  UW *t_us, W *x_um, BC_RESULT *res);
IMPORT INT  s3_run_unit_tests(void);

#endif /* S3_TESTS_H */
