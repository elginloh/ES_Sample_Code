/*
 *	drv_ultrasonic.h
 *	HC-SR04 ultrasonic ranger (Buddy 5).  TRIG = GP4 (output), ECHO = GP5
 *	(input, both edges timestamped by the shared GPIO interrupt).  Week 6
 *	hardware notes: the module is powered at 3.3 V (3.3 V-compatible
 *	variant), so ECHO needs no level shifting.
 *
 *	Results distinguish the cases the brief asks for:
 *	  US_OK           valid distance
 *	  US_NO_ECHO      echo pulse longer than the range: nothing within
 *	                  4 m (a valid "clear" reading, not a fault)
 *	  US_NO_RESPONSE  ECHO never rose: sensor missing or dead (timeout)
 *	  US_INVALID      pulse impossible (shorter than the minimum range,
 *	                  ECHO stuck high before the trigger, torn edges)
 *	Called only from the Obstacle Scan task (the owner of the sensor).
 */

#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include <tk/tkernel.h>

#define US_TRIG_PIN		4
#define US_ECHO_PIN		5

#define US_OK			0
#define US_NO_ECHO		1
#define US_NO_RESPONSE		2
#define US_INVALID		3

#define US_MIN_RANGE_MM		20	/* HC-SR04 datasheet: 2 cm  */
#define US_MAX_RANGE_MM		4000	/* HC-SR04 datasheet: 4 m   */
#define US_RISE_TIMEOUT_US	10000	/* echo starts ~0.5 ms after trigger */
#define US_WAIT_MS		40	/* covers the 38 ms "no echo" pulse */
#define US_MIN_INTERVAL_MS	60	/* datasheet measurement cycle */
#define US_DEBUG		1	/* raw echo timing diagnostics      */

/* Speed of sound [CAL]: 343.2 m/s at 20 C -> 0.1716 mm/us round trip/2 */
#define US_MM_PER_US_X10000	1716

IMPORT ER   us_init(ID done_flg, UINT done_bit);
IMPORT INT  us_measure(UW *dist_mm);
IMPORT UW   us_pulse_to_mm(UW pulse_us);

#endif /* DRV_ULTRASONIC_H */
