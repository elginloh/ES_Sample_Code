/*
 *	s1_tests.h
 *	Test helpers of subsystem 1 shared with the integration tests (test
 *	builds only).
 */

#ifndef S1_TESTS_H
#define S1_TESTS_H

#include <tk/typedef.h>
#include "mqtt_proto.h"

/* Field-by-field comparison of two telemetry records (never memcmp:
   structure padding is undefined) */
IMPORT BOOL s1_tlm_equal(const TLM_RECORD *a, const TLM_RECORD *b);

IMPORT INT  s1_run_unit_tests(void);
IMPORT INT  s1_run_integration_tests(void);

#endif /* S1_TESTS_H */
