/*
 *	nav_map.c
 *	Barcode symbol -> navigation command table (see nav_map.h).
 */

#include "nav_map.h"
#include "../subsystem2/motion_msg.h"

typedef struct {
	UB	symbol;		/* Code 39 index (code39.h) */
	UB	nav;		/* NAV_* */
} NAV_ENTRY;

/* !!! TEAM DECISION REQUIRED (see nav_map.h) !!!
   Example of a filled table, kept commented out on purpose:
	{ 10, NAV_LEFT },	'A'
	{ 35, NAV_RIGHT },	'Z'                                         */
LOCAL const NAV_ENTRY S3_NAV_MAP_DEFAULT[] = {
	{ 0xFF, 0 }		/* terminator: no default meanings */
};

LOCAL NAV_ENTRY	nav_tab[NAV_MAP_SIZE];
LOCAL UINT	nav_n;

EXPORT void nav_map_clear(void)
{
	nav_n = 0;
}

EXPORT void nav_map_reset(void)
{
	UINT	i;

	nav_map_clear();
	for (i = 0; S3_NAV_MAP_DEFAULT[i].symbol != 0xFFU; i++) {
		(void)nav_map_set(S3_NAV_MAP_DEFAULT[i].symbol,
				  S3_NAV_MAP_DEFAULT[i].nav);
	}
}

EXPORT ER nav_map_set(UB symbol, UB nav)
{
	UINT	i;

	if (nav > NAV_UTURN || symbol >= 43U) {	/* '*' is never data */
		return E_PAR;
	}
	for (i = 0; i < nav_n; i++) {
		if (nav_tab[i].symbol == symbol) {
			nav_tab[i].nav = nav;
			return E_OK;
		}
	}
	if (nav_n >= NAV_MAP_SIZE) {
		return E_LIMIT;
	}
	nav_tab[nav_n].symbol = symbol;
	nav_tab[nav_n].nav = nav;
	nav_n++;
	return E_OK;
}

EXPORT BOOL nav_map_lookup(UB symbol, UB *nav)
{
	UINT	i;

	for (i = 0; i < nav_n; i++) {
		if (nav_tab[i].symbol == symbol) {
			*nav = nav_tab[i].nav;
			return TRUE;
		}
	}
	return FALSE;
}

EXPORT const char *nav_name(UB nav)
{
	switch (nav) {
	case NAV_LEFT:		return "LEFT";
	case NAV_RIGHT:		return "RIGHT";
	case NAV_STRAIGHT:	return "STRAIGHT";
	case NAV_UTURN:		return "U-TURN";
	default:		return "?";
	}
}

EXPORT void nav_to_motion(UB nav, UW *mcmd_type, INT *arg)
{
	switch (nav) {
	case NAV_LEFT:
		*mcmd_type = MCMD_TURN;
		*arg = -S3_TURN_DEG;
		break;
	case NAV_RIGHT:
		*mcmd_type = MCMD_TURN;
		*arg = S3_TURN_DEG;
		break;
	case NAV_UTURN:
		*mcmd_type = MCMD_TURN;
		*arg = S3_UTURN_DEG;
		break;
	default:
		*mcmd_type = MCMD_MOVE;
		*arg = S3_STRAIGHT_MM;
		break;
	}
}
