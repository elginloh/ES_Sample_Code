/*
 *	drv_ultrasonic.c
 *	HC-SR04 driver (see drv_ultrasonic.h).
 */

#include "subsystem5.h"
#include "drv_ultrasonic.h"

#define EDGE_RISE	0x1U
#define EDGE_FALL	0x2U

/* Written by the echo callback (one interrupt source), reset by the
   scan task only while no measurement is in flight. */
LOCAL volatile UW	echo_rise_us;
LOCAL volatile UW	echo_fall_us;
LOCAL volatile UW	echo_seen;
LOCAL ID		done_flgid;
LOCAL UINT		done_ptn;
LOCAL UW		us_seq;
#if US_DEBUG
LOCAL BOOL		us_debug_near;
LOCAL INT		us_debug_last_result;
LOCAL UW		us_debug_last_mm;
#endif

LOCAL INT report_measurement(INT result, UW seen, UW rise_delay, UW pulse,
			     UW distance, UINT echo_level)
{
#if US_DEBUG
	BOOL	near;
	BOOL	fault = (result == US_NO_RESPONSE || result == US_INVALID) ? TRUE : FALSE;
	BOOL	was_fault = (us_debug_last_result == US_NO_RESPONSE
			     || us_debug_last_result == US_INVALID) ? TRUE : FALSE;
	UW	delta = (distance > us_debug_last_mm)
		      ? distance - us_debug_last_mm : us_debug_last_mm - distance;
	UB	*event = NULL;

	us_seq = (us_seq + 1U) & 0xFFFFU;
	/* Enter at the real obstacle-trigger threshold.  Once inside, retain a
	   small hysteresis band so normal ultrasonic jitter does not print an
	   obstacle/clear pair every 60 ms. */
	near = (result == US_OK
		&& distance <= S5_DETECT_MM + (us_debug_near ? US_DEBUG_HYST_MM : 0U))
	       ? TRUE : FALSE;
	if (near && (!us_debug_near
		     || (distance <= S5_DETECT_MM && delta >= US_DEBUG_CHANGE_MM))) {
		event = (UB*)"obstacle";
		us_debug_last_mm = distance;
	} else if (!near && us_debug_near) {
		event = fault ? (UB*)"fault" : (UB*)"clear";
	} else if (fault && result != us_debug_last_result) {
		event = (UB*)"fault";
	} else if (!fault && was_fault) {
		event = (UB*)"recovered";
	}
	if (event != NULL) {
		tm_printf((UB*)"us: %s seq %d result %d echo %d edges %d rise %d us "
			  "pulse %d us distance %d mm (threshold %d mm)\n",
			  event, (INT)us_seq, result, (INT)echo_level, (INT)seen,
			  (INT)rise_delay, (INT)pulse, (INT)distance, S5_DETECT_MM);
	}
	us_debug_near = near;
	us_debug_last_result = result;
#else
	(void)seen;
	(void)rise_delay;
	(void)pulse;
	(void)distance;
	(void)echo_level;
#endif
	return result;
}

/* Shared-GPIO-handler callback: timestamp only */
LOCAL void echo_edge(UINT pin, UINT events, UW now_us)
{
	(void)pin;
	if ((events & GPIO_IRQ_EDGE_RISE) != 0U && (echo_seen & EDGE_RISE) == 0U) {
		echo_rise_us = now_us;
		echo_seen = echo_seen | EDGE_RISE;
	}
	if ((events & GPIO_IRQ_EDGE_FALL) != 0U && (echo_seen & EDGE_RISE) != 0U) {
		echo_fall_us = now_us;
		echo_seen = echo_seen | EDGE_FALL;
		(void)tk_set_flg(done_flgid, done_ptn);
	}
}

EXPORT UW us_pulse_to_mm(UW pulse_us)
{
	return (pulse_us * US_MM_PER_US_X10000 + 5000U) / 10000U;
}

EXPORT ER us_init(ID done_flg, UINT done_bit)
{
	ER	er;

	done_flgid = done_flg;
	done_ptn = done_bit;
	echo_seen = 0;
	us_seq = 0;
#if US_DEBUG
	us_debug_near = FALSE;
	us_debug_last_result = US_OK;
	us_debug_last_mm = 0;
#endif
	(void)gpio_set_pin(US_TRIG_PIN, GPIO_MODE_OUT);		/* libbsp */
	(void)gpio_set_val(US_TRIG_PIN, 0);
	gpio_input_config(US_ECHO_PIN, GPIO_PULL_DOWN);
	er = gpio_irq_register(US_ECHO_PIN, GPIO_IRQ_EDGES, echo_edge);
#if US_DEBUG
	tm_printf((UB*)"us: init trig GP%d echo GP%d, wait %d ms, irq status %d\n",
		  US_TRIG_PIN, US_ECHO_PIN, US_WAIT_MS, (INT)er);
#endif
	return er;
}

EXPORT INT us_measure(UW *dist_mm)
{
	UW	t_trig, seen, pulse, rise_delay;
	UINT	ptn;

	*dist_mm = 0;
	if (!gpio_irq_running()) {
		return report_measurement(US_NO_RESPONSE, 0, 0, 0, 0,
					  gpio_get_val(US_ECHO_PIN));
	}
	if (gpio_get_val(US_ECHO_PIN) != 0U) {
		(void)tk_dly_tsk(US_WAIT_MS);	/* previous echo still running */
		if (gpio_get_val(US_ECHO_PIN) != 0U) {
			return report_measurement(US_INVALID, 0, 0, 0, 0, 1);
		}
	}
	(void)tk_clr_flg(done_flgid, ~done_ptn);
	echo_seen = 0;
	APP_DMB();

	(void)gpio_set_val(US_TRIG_PIN, 1);		/* >= 10 us trigger pulse */
	(void)hw_delay_us(12);
	(void)gpio_set_val(US_TRIG_PIN, 0);
	t_trig = hw_time_us();

	(void)tk_wai_flg(done_flgid, done_ptn, TWF_ORW | TWF_BITCLR, &ptn, US_WAIT_MS);
	seen = app_load(&echo_seen);
	rise_delay = ((seen & EDGE_RISE) != 0U) ? echo_rise_us - t_trig : 0U;

	if ((seen & EDGE_RISE) == 0U || rise_delay > US_RISE_TIMEOUT_US + 1000U) {
		return report_measurement(US_NO_RESPONSE, seen, rise_delay, 0, 0,
					  gpio_get_val(US_ECHO_PIN));
	}
	if ((seen & EDGE_FALL) == 0U) {
		return report_measurement(US_NO_ECHO, seen, rise_delay, 0, 0,
					  gpio_get_val(US_ECHO_PIN));
	}
	pulse = echo_fall_us - echo_rise_us;
	*dist_mm = us_pulse_to_mm(pulse);
	if (*dist_mm > US_MAX_RANGE_MM) {
		return report_measurement(US_NO_ECHO, seen, rise_delay, pulse, *dist_mm,
					  gpio_get_val(US_ECHO_PIN));
	}
	if (*dist_mm < US_MIN_RANGE_MM) {
		return report_measurement(US_INVALID, seen, rise_delay, pulse, *dist_mm,
					  gpio_get_val(US_ECHO_PIN));
	}
	return report_measurement(US_OK, seen, rise_delay, pulse, *dist_mm,
				  gpio_get_val(US_ECHO_PIN));
}
