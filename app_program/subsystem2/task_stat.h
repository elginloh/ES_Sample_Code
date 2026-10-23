/*
 *	task_stat.h
 *	Execution-time and period instrumentation for periodic tasks.
 *
 *	Tribunal 1 (CPU row, Version C row 5): "measure worst-case run times",
 *	"Measure task execution time, jitter and queue latency with packing
 *	enabled".  Each periodic task owns one TASK_STAT (single writer) and
 *	brackets its work with task_stat_begin()/task_stat_end().  Readers
 *	(tests, telemetry debug) read individual words; a slightly torn view
 *	of these diagnostics is harmless.  All times are microseconds.
 */

#ifndef TASK_STAT_H
#define TASK_STAT_H

#include "drv_hwtime.h"

typedef struct {
	volatile UW	runs;		/* completed activations           */
	volatile UW	last_start_us;	/* start of the latest activation  */
	volatile UW	exec_max_us;	/* worst execution time            */
	volatile UW	exec_last_us;
	volatile UW	period_min_us;	/* shortest start-to-start gap     */
	volatile UW	period_max_us;	/* longest start-to-start gap      */
	volatile UW	overruns;	/* exec longer than the budget     */
} TASK_STAT;

Inline UW task_stat_begin(TASK_STAT *s)
{
	UW	now = hw_time_us();

	if (s->runs != 0U) {
		UW	period = now - s->last_start_us;

		if (period > s->period_max_us) {
			s->period_max_us = period;
		}
		if (s->period_min_us == 0U || period < s->period_min_us) {
			s->period_min_us = period;
		}
	}
	s->last_start_us = now;
	return now;
}

Inline void task_stat_end(TASK_STAT *s, UW start, UW budget_us)
{
	UW	exec = hw_time_us() - start;

	s->exec_last_us = exec;
	if (exec > s->exec_max_us) {
		s->exec_max_us = exec;
	}
	if (exec > budget_us) {
		s->overruns = s->overruns + 1U;
	}
	s->runs = s->runs + 1U;
}

#endif /* TASK_STAT_H */
