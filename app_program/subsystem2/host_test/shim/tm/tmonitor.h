/*
 *	Host-test shim for <tm/tmonitor.h>: tm_printf() prints to stdout so
 *	app_program/test_report.h (TEST_CHECK/TEST_SUMMARY) works unchanged.
 */

#ifndef __TM_TMONITOR_H__
#define __TM_TMONITOR_H__

#include <stdio.h>
#include <stdarg.h>
#include <tk/typedef.h>

static inline INT tm_printf(const UB *fmt, ...)
{
	va_list	ap;
	INT	n;

	va_start(ap, fmt);
	n = vprintf((const char *)fmt, ap);
	va_end(ap);
	return n;
}

#endif /* __TM_TMONITOR_H__ */
