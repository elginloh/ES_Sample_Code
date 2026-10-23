/*
 *	barcode_dec.h
 *	Barcode capture and decoding state machine (Buddy 3), fed by the edge
 *	timestamps recorded by the IR_RIGHT (GP27) interrupt (Tribunal 1,
 *	Version B: interrupt edge capture, asynchronous decoding).  Pure
 *	logic, host-testable.
 *
 *	Element widths: the time between two edges multiplied by the wheel
 *	speed from the encoders (Tribunal 1: "consider vehicle speed and
 *	encoder distance").  Encoder ticks (~10 mm) are coarser than a 3 mm
 *	narrow bar, so distance alone cannot measure a bar; time x speed can.
 *	Classification is by ratio within each symbol, so a slow drift of
 *	speed or a different print scale does not matter.
 *
 *	    IDLE --(white->black after a quiet zone)--> CAPTURING
 *	    CAPTURING --(white for > end quiet zone)--> decode -> IDLE
 *	    CAPTURING --(element too long / too many / window or time out)
 *	              --> INVALID -> IDLE
 *	The decode window is a travelled DISTANCE (BC_MAX_LENGTH_UM), so the
 *	time limit automatically adapts to the speed (Week 6 statechart:
 *	"timeout adapts to speed").
 */

#ifndef BARCODE_DEC_H
#define BARCODE_DEC_H

#include <tk/typedef.h>
#include "code39.h"

#define BC_MAX_EDGES		64	/* 4 data symbols: 6*10-1 elements */
#define BC_MAX_DATA		4

/* Geometry limits, micrometres.  The specification gives a 141 mm
   barcode = 47 narrow units, i.e. narrow ~3 mm, wide ~9 mm.  Limits are
   generous so a bigger print (the sample PDF printed full page has
   ~7 mm narrow bars) still decodes. */
#define BC_MIN_ELEMENT_UM	800	/* shorter = noise, merged away   */
#define BC_MAX_ELEMENT_UM	40000	/* longer = not a barcode (line)  */
#define BC_QUIET_MIN_UM		12000	/* white before the first bar     */
#define BC_END_QUIET_UM		20000	/* white after the last bar ...   */
#define BC_END_QUIET_X10	25	/* ... and >= 2.5 x widest element */
#define BC_MAX_LENGTH_UM	450000	/* capture window (distance)      */
#define BC_IDLE_TIMEOUT_US	3000000	/* no edge for 3 s: car stopped   */
#define BC_LOCKOUT_MM		150	/* ignore starts after a decode   */
#define BC_DUP_WINDOW_MM	600	/* same symbol again = duplicate  */
#define BC_FALLBACK_MMS		100	/* speed if the encoders give none */

/* States (2 bits in the barcode status word) */
#define BCS_IDLE		0U
#define BCS_CAPTURING		1U
#define BCS_DECODING		2U	/* used by the task while executing */
#define BCS_EXECUTING		3U

/* Results (2 bits) */
#define BCR_NONE		0U
#define BCR_VALID		1U
#define BCR_INVALID		2U
#define BCR_DUPLICATE		3U

/* Capture failures beyond the C39_ERR_* codes */
#define BC_ERR_LONG		(-10)	/* element longer than a barcode bar */
#define BC_ERR_OVERFLOW		(-11)	/* too many edges                    */
#define BC_ERR_WINDOW		(-12)	/* barcode longer than the window    */
#define BC_ERR_STALLED		(-13)	/* no edge for too long              */
#define BC_ERR_ABORTED		(-14)	/* aborted by the caller             */

typedef struct {
	UB	result;			/* BCR_* */
	UB	n_data;
	UB	data[BC_MAX_DATA];	/* Code 39 indices */
	BOOL	reversed;
	INT	err;			/* when INVALID */
	UB	n_elements;		/* after glitch merging */
} BC_RESULT;

typedef struct {
	UB	state;
	UB	level;			/* 1 = black under the sensor */
	BOOL	level_known;
	UW	t_last_us;		/* last edge */
	UW	edge_t[BC_MAX_EDGES];
	UH	edge_v[BC_MAX_EDGES];	/* speed at each edge, mm/s */
	UINT	n_edges;
	W	start_odo_mm;
	BOOL	lockout;
	W	lockout_end_mm;
	BOOL	have_last;
	UB	last_sym;
	W	last_sym_mm;
	UW	widest_um;
	UH	last_speed;		/* last positive speed seen */
} BC_DEC;

IMPORT void bc_init(BC_DEC *d);
IMPORT void bc_abort(BC_DEC *d, BC_RESULT *res);
IMPORT BOOL bc_capturing(const BC_DEC *d);

/* One edge: 'black' is the level AFTER the edge (1 = black) */
IMPORT void bc_edge(BC_DEC *d, UW t_us, UB black, W speed_mms, W odo_mm,
		    BC_RESULT *res);

/* Periodic check without an edge: completion, window, stall */
IMPORT void bc_poll(BC_DEC *d, UW now_us, W speed_mms, W odo_mm,
		    BC_RESULT *res);

#endif /* BARCODE_DEC_H */
