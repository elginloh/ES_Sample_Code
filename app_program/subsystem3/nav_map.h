/*
 *	nav_map.h
 *	Decoded barcode symbol -> navigation command (Buddy 3).  Pure logic.
 *
 *	Navigation commands use 2 bits (Tribunal 1, curveball response for
 *	Buddy 3: "2-bit codes for up to 4 navigation commands"); validity is
 *	a separate bit, so all four codes stay usable.
 *
 *	Current course mapping: A = RIGHT, F and Z = LEFT, U = U-TURN,
 *	X = STRAIGHT. Other valid but unmapped barcode symbols are reported
 *	without producing motion.
 */

#ifndef NAV_MAP_H
#define NAV_MAP_H

#include <tk/typedef.h>
#include <tk/errno.h>

#define NAV_LEFT	0U
#define NAV_RIGHT	1U
#define NAV_STRAIGHT	2U
#define NAV_UTURN	3U
#define NAV_BITS	2U

#define NAV_MAP_SIZE	8	/* entries in the runtime table */

IMPORT void nav_map_reset(void);			/* load the default table */
IMPORT ER   nav_map_set(UB symbol, UB nav);		/* add/replace one entry  */
IMPORT void nav_map_clear(void);
IMPORT BOOL nav_map_lookup(UB symbol, UB *nav);		/* FALSE = no meaning     */
IMPORT const char *nav_name(UB nav);

/* Motion translation of a navigation command (FR3):
   LEFT = TURN -90, RIGHT = TURN +90, U-TURN = TURN +180,
   STRAIGHT = MOVE forward S3_STRAIGHT_MM (cross the junction).
   Buddy 2 runs a 90 deg barcode turn in two parts (motion_cfg.h): drive
   on to the turn point, measured from where the right IR sensor leaves
   the crossing line, then turn (outer wheel forward, inner wheel slowly
   backward) until the crossing line lies between the centre and right
   sensors, at about 90 deg.  U-turns turn on the spot. */
#define S3_TURN_DEG		90
#define S3_UTURN_DEG		180
#define S3_STRAIGHT_MM		120	/* [CAL] distance to clear a junction */
#define S3_NAV_SPEED_LEVEL	2U	/* 2 x 25 = 50 mm/s for barcode
					   moves and turns; 0 used the faster
					   global defaults */

IMPORT void nav_to_motion(UB nav, UW *mcmd_type, INT *arg);

#endif /* NAV_MAP_H */
