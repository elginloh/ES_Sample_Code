/*
 *	host_main.c
 *	Runs the subsystem unit tests and the cross-subsystem integration
 *	tests on the host.  Exit status = number of failed checks (0 = all
 *	passed), so the result can be scripted.
 */

#include <stdio.h>
#include <string.h>
#include <tk/typedef.h>

IMPORT INT s1_run_unit_tests(void);
IMPORT INT s2_run_unit_tests(void);
IMPORT INT s3_run_unit_tests(void);
IMPORT INT s4_run_unit_tests(void);
IMPORT INT s5_run_unit_tests(void);
IMPORT INT s1_run_integration_tests(void);
IMPORT INT s2_run_integration_tests(void);

typedef struct {
	const char	*name;
	INT		(*run)(void);
} SUITE;

static const SUITE suites[] = {
	{ "s1", s1_run_unit_tests },
	{ "s2", s2_run_unit_tests },
	{ "s3", s3_run_unit_tests },
	{ "s4", s4_run_unit_tests },
	{ "s5", s5_run_unit_tests },
	{ "int", s2_run_integration_tests },
	{ "int", s1_run_integration_tests },
};

static int selected(const char *name, int argc, char **argv)
{
	int	i;

	if (argc <= 1) {
		return 1;
	}
	for (i = 1; i < argc; i++) {
		if (strcmp(argv[i], name) == 0) {
			return 1;
		}
	}
	return 0;
}

int main(int argc, char **argv)
{
	unsigned	i;
	int		fails = 0;

	for (i = 0; i < sizeof(suites) / sizeof(suites[0]); i++) {
		if (selected(suites[i].name, argc, argv)) {
			fails += suites[i].run();
		}
	}
	printf("[TEST] HOST TOTAL: %d failed check(s)\n", fails);
	return (fails > 255) ? 255 : fails;
}
