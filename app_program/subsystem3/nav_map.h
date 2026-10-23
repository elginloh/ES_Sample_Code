/*
 *	nav_map.h
 *	Decoded barcode symbol -> navigation command (Buddy 3).  Pure logic.
 *
 *	Navigation commands use 2 bits (Tribunal 1, curveball response for
 *	Buddy 3: "2-bit codes for up to 4 navigation commands"); validity is
 *	a separate bit, so all four codes stay usable.
 *
 *	!!! TEAM DECISION REQUIRED !!!  None of the supplied documents says
 *	which barcode character means Turn Left / Turn Right / Go Straight /
 *	U-Turn (FR3).  The samples only establish the symbology (Code 39) and
 *	the characters A, F, W, X, Y, Z.  The default table is therefore
 *	EMPTY: a valid barcode is decoded and reported (telemetry, status
 *	word) but produces no motion.  Fill S3_NAV_MAP_DEFAULT in nav_map.c
 *	once the track's meaning is known, or call nav_map_set() at start-up.
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

/* Motion translation of a navigation command (encoder-based, FR3):
   LEFT = TURN -90, RIGHT = TURN +90, U-TURN = TURN +180,
   STRAIGHT = MOVE forward S3_STRAIGHT_MM (cross the junction). */
#define S3_TURN_DEG		90
#define S3_UTURN_DEG		180
#define S3_STRAIGHT_MM		120	/* [CAL] distance to clear a junction */

IMPORT void nav_to_motion(UB nav, UW *mcmd_type, INT *arg);

#endif /* NAV_MAP_H */
