/*
 *	s3_unit_tests.c
 *	Unit tests of subsystem 3's pure logic: Code 39 decoding against
 *	EVERY supplied barcode sample, the capture state machine at several
 *	speeds, invalid-pattern rejection, duplicate suppression, the
 *	navigation map, line-following decisions and status-word packing.
 *	Runs on the host (host_test/) and in the on-target test image.
 *	Test IDs: S3-Uxx.
 */

#include <tk/typedef.h>

#if APP_TEST || HOST_TEST

#include "../test_report.h"
#include "code39.h"
#include "barcode_dec.h"
#include "nav_map.h"
#include "line_follow.h"
#include "s3_status.h"
#include "s3_tests.h"
#include "../subsystem2/motion_msg.h"

/* Element widths (bar, space, ... bar) MEASURED from the supplied files:
   Barcode Sample.pdf pages 1-4 (rendered), BARCODE.pptx EMF/PNG geometry,
   and the photograph in Barcode & Line Specification.pdf (real print).
   Extraction method: subsystem3/docs/BARCODE_ANALYSIS.md. */
typedef struct {
	const char	*name;
	char		expect;
	UW		el[29];
} BC_VECTOR;

LOCAL const BC_VECTOR bc_vectors[] = {
	{ "PDF_A_P1", 'A', { 39, 119, 40, 40, 118, 40, 119, 40, 39, 40, 119, 39, 40, 40, 39, 119, 40, 39, 119, 40, 39, 119, 40, 40, 118, 40, 119, 40, 39 } },
	{ "PDF_A_P2", 'A', { 20, 60, 20, 20, 60, 19, 60, 20, 20, 20, 59, 20, 20, 20, 20, 60, 20, 20, 59, 20, 20, 60, 20, 19, 60, 20, 60, 20, 20 } },
	{ "PDF_Z_P3", 'Z', { 39, 119, 40, 40, 118, 40, 119, 40, 39, 40, 39, 119, 119, 40, 119, 39, 40, 40, 39, 40, 39, 119, 40, 40, 118, 40, 119, 40, 39 } },
	{ "PDF_Z_P4", 'Z', { 20, 60, 20, 20, 59, 20, 60, 20, 20, 20, 20, 59, 60, 20, 60, 19, 20, 20, 20, 20, 20, 60, 19, 20, 60, 20, 60, 19, 20 } },
	{ "PPT_A", 'A', { 117, 351, 117, 117, 351, 117, 351, 117, 117, 117, 351, 117, 117, 117, 117, 351, 117, 117, 351, 117, 117, 351, 117, 117, 351, 117, 351, 117, 117 } },
	{ "PPT_F", 'F', { 117, 351, 117, 117, 351, 117, 351, 117, 117, 117, 117, 117, 351, 117, 351, 351, 117, 117, 117, 117, 117, 351, 117, 117, 351, 117, 351, 117, 117 } },
	{ "PPT_Z", 'Z', { 117, 351, 117, 117, 351, 117, 351, 117, 117, 117, 117, 351, 351, 117, 351, 117, 117, 117, 117, 117, 117, 351, 117, 117, 351, 117, 351, 117, 117 } },
	{ "PPT_W", 'W', { 26, 78, 26, 26, 78, 26, 78, 26, 26, 26, 78, 78, 78, 26, 26, 26, 26, 26, 26, 26, 26, 78, 26, 26, 78, 26, 78, 26, 26 } },
	{ "PPT_X", 'X', { 22, 66, 22, 22, 66, 22, 66, 22, 22, 22, 22, 66, 22, 22, 66, 22, 22, 22, 66, 22, 22, 66, 22, 22, 66, 22, 66, 22, 22 } },
	{ "PPT_Y", 'Y', { 9, 27, 9, 9, 27, 9, 27, 9, 9, 9, 27, 27, 9, 9, 27, 9, 9, 9, 9, 9, 9, 27, 9, 9, 27, 9, 27, 9, 9 } },
	{ "PHOTO_A", 'A', { 19, 64, 18, 22, 60, 22, 59, 22, 19, 23, 58, 22, 19, 22, 18, 63, 18, 23, 58, 22, 19, 63, 19, 21, 59, 23, 59, 22, 19 } },
};
#define N_VECTORS	(sizeof(bc_vectors) / sizeof(bc_vectors[0]))

/* ------------------------------------------------------------------ */

LOCAL INT test_code39(void)
{
	UB	data[4];
	BOOL	rev;
	UW	el[29];
	UINT	i, k, ok = 0, bad_table = 0;
	INT	n;

	TEST_INIT();
	for (i = 0; i < C39_NSYM; i++) {
		UH	p = c39_pattern(i);
		UINT	wide = 0;

		for (k = 0; k < 9U; k++) {
			wide += (p >> k) & 1U;
		}
		if (c39_lookup(p) != (INT)i || wide != 3U) {
			bad_table++;
		}
	}
	TEST_CHECK("S3-U01", bad_table == 0 && c39_pattern(C39_STAR) == 0x094
		   && c39_pattern((UINT)c39_index_of_char('A')) == 0x109
		   && c39_pattern((UINT)c39_index_of_char('Z')) == 0x0D0,
		   "Code 39 table: 44 unique patterns, 3 wide each; * A Z as measured");

	for (i = 0; i < N_VECTORS; i++) {
		n = c39_decode(bc_vectors[i].el, 29, data, 4, &rev);
		if (n == 1 && c39_char(data[0]) == bc_vectors[i].expect && !rev) {
			ok++;
		} else {
			tm_printf((UB*)"[TEST]   sample %s decoded n=%d\n", bc_vectors[i].name, n);
		}
	}
	TEST_CHECK("S3-U02", ok == N_VECTORS,
		   "every supplied sample decodes (PDF A/Z, PPTX A/F/Z/W/X/Y, photo A)");

	for (k = 0; k < 29U; k++) {
		el[k] = bc_vectors[4].el[28U - k];		/* PPT_A backwards */
	}
	n = c39_decode(el, 29, data, 4, &rev);
	TEST_CHECK("S3-U03", n == 1 && c39_char(data[0]) == 'A' && rev,
		   "barcode read in the opposite direction decodes (reversed flag)");

	for (k = 0; k < 29U; k++) {
		el[k] = bc_vectors[0].el[k] * 7U / 2U;		/* 3.5x bigger print */
	}
	n = c39_decode(el, 29, data, 4, &rev);
	TEST_CHECK("S3-U04", n == 1 && c39_char(data[0]) == 'A',
		   "print-scale independent (ratio classification)");

	for (k = 0; k < 29U; k++) {
		el[k] = bc_vectors[0].el[k];
	}
	TEST_CHECK("S3-U05", c39_decode(el, 28, data, 4, &rev) == C39_ERR_COUNT,
		   "invalid: element count not 10k-1");
	for (k = 0; k < 29U; k++) {
		el[k] = 40;
	}
	TEST_CHECK("S3-U06", c39_decode(el, 29, data, 4, &rev) == C39_ERR_WIDTH,
		   "invalid: no wide/narrow contrast");
	for (k = 0; k < 29U; k++) {
		el[k] = bc_vectors[0].el[k];
	}
	el[1] = 40;				/* '*' loses its wide space */
	el[3] = 119;
	TEST_CHECK("S3-U07", c39_decode(el, 29, data, 4, &rev) < 0,
		   "invalid: corrupted start symbol rejected (no command)");
	for (k = 0; k < 29U; k++) {
		el[k] = bc_vectors[0].el[k];
	}
	el[9] = 140;				/* inter-character gap far too wide */
	TEST_CHECK("S3-U08", c39_decode(el, 29, data, 4, &rev) == C39_ERR_GAP,
		   "invalid: wide inter-character gap rejected");
	for (k = 0; k < 29U; k++) {
		el[k] = bc_vectors[0].el[k];
	}
	for (k = 0; k < 9U; k++) {
		el[k] = bc_vectors[0].el[10U + k];	/* 'A' where '*' must be */
	}
	TEST_CHECK("S3-U09", c39_decode(el, 29, data, 4, &rev) == C39_ERR_STARTSTOP,
		   "invalid: missing start/stop character");
	TEST_SUMMARY("s3 code39");
	return test_fail_;
}

/* ------------------------------------------------------------------ */
/*  Physical simulation: barcode on the track, car driving over it.    */

#define SIM_NARROW_UM	3000	/* 141 mm / 47 units (specification) */
#define SIM_WIDE_UM	9000

/* Element widths of "*<c>*" in um, with an optical bias that makes bars
   look wider and spaces narrower (IR spot / threshold) */
LOCAL UINT build_barcode(char c, W bias_um, W *el)
{
	UB	sym[3];
	UINT	s, k, n = 0;

	sym[0] = C39_STAR;
	sym[1] = (UB)c39_index_of_char(c);
	sym[2] = C39_STAR;
	for (s = 0; s < 3U; s++) {
		UH	p = c39_pattern(sym[s]);

		for (k = 0; k < 9U; k++) {
			W	w = ((p >> (8U - k)) & 1U) ? SIM_WIDE_UM : SIM_NARROW_UM;

			el[n++] = ((k & 1U) == 0U) ? w + bias_um : w - bias_um;
		}
		if (s < 2U) {
			el[n++] = SIM_NARROW_UM - bias_um;	/* gap */
		}
	}
	return n;
}

/* Integer "sine" for the speed ripple: triangle wave, -1000..1000 */
LOCAL W tri(W x_um, W period_um)
{
	W	p = ((x_um % period_um) + period_um) % period_um;
	W	q = p * 4000 / period_um;		/* 0..4000 */

	return (q < 2000) ? (q - 1000) : (3000 - q);
}

/* Drive over the barcode starting at track position start_um (see
   s3_tests.h): speed ripples +-15 % around v0; the reported (encoder)
   speed reads 5 % high. */
EXPORT void s3_drive_over(BC_DEC *d, char c, W v0_mms, W start_um, W phase_um,
			  UW *t_us, W *x_um, BC_RESULT *res)
{
	W	el[30];
	UINT	n = build_barcode(c, 600, el);
	UINT	i;
	W	pos = start_um;
	UB	level = 1;

	res->result = BCR_NONE;
	/* travel from the current position to the barcode start (white) */
	for (i = 0; i <= n; i++) {
		W	target = (i == 0U) ? start_um : pos + el[i - 1U];

		while (*x_um < target) {
			W	v = v0_mms + v0_mms * 15 * tri(*x_um + phase_um, 60000) / 100000;

			*x_um += 100;				/* 0.1 mm steps */
			*t_us += (UW)(100 * 1000 / v);		/* um / (mm/s) -> us */
		}
		if (i > 0U) {
			pos = target;
		}
		bc_edge(d, *t_us, level, v0_mms * 105 / 100, *x_um / 1000, res);
		if (res->result != BCR_NONE) {
			return;
		}
		level ^= 1U;
	}
	/* white beyond the last bar: poll until completion */
	for (i = 0; i < 400U && res->result == BCR_NONE; i++) {
		*x_um += 1000;
		*t_us += (UW)(1000 * 1000 / v0_mms);
		bc_poll(d, *t_us, v0_mms, *x_um / 1000, res);
	}
}

LOCAL INT test_capture(void)
{
	static const W	speeds[3] = { 100, 200, 300 };
	BC_DEC		d;
	BC_RESULT	res;
	UW		t;
	W		x;
	UINT		s, trial, good[3] = {0, 0, 0};

	TEST_INIT();
	for (s = 0; s < 3U; s++) {
		for (trial = 0; trial < 15U; trial++) {
			bc_init(&d);
			t = 1000;
			x = 0;
			s3_drive_over(&d, 'A', speeds[s], 50000, (W)trial * 4000, &t, &x, &res);
			if (res.result == BCR_VALID && res.n_data == 1
			    && c39_char(res.data[0]) == 'A') {
				good[s]++;
			}
		}
	}
	TEST_CHECK("S3-U10", good[0] == 15 && good[1] == 15 && good[2] == 15,
		   "15/15 reads at 100, 200 and 300 mm/s (+-15 % ripple, 5 % speed bias, 0.6 mm blur)");

	bc_init(&d);
	t = 1000;
	x = 0;
	s3_drive_over(&d, 'Z', 200, 50000, 0, &t, &x, &res);
	TEST_CHECK("S3-U11a", res.result == BCR_VALID && c39_char(res.data[0]) == 'Z',
		   "first pass over Z: VALID");
	/* past the 150 mm lockout, inside the 600 mm duplicate window */
	s3_drive_over(&d, 'Z', 200, x + 200000, 0, &t, &x, &res);
	TEST_CHECK("S3-U11", res.result == BCR_DUPLICATE,
		   "same barcode again within 600 mm: DUPLICATE (no second command)");

	/* the line itself under the sensor: one 60 mm black run */
	bc_init(&d);
	bc_edge(&d, 100000, 1, 200, 100, &res);
	bc_edge(&d, 400000, 0, 200, 160, &res);
	TEST_CHECK("S3-U12", res.result == BCR_INVALID && res.err == BC_ERR_LONG && !bc_capturing(&d),
		   "60 mm black run rejected as not-a-barcode at once");

	/* car stops in the middle of a barcode */
	bc_init(&d);
	bc_edge(&d, 100000, 1, 200, 100, &res);
	bc_edge(&d, 115000, 0, 200, 103, &res);
	bc_poll(&d, 115000 + BC_IDLE_TIMEOUT_US + 1000, 0, 103, &res);
	TEST_CHECK("S3-U13", res.result == BCR_INVALID && res.err == BC_ERR_STALLED,
		   "no edge for 3 s while capturing: aborted (bounded)");

	/* window exceeded (speed adaptive: distance, not time) */
	bc_init(&d);
	bc_edge(&d, 100000, 1, 200, 100, &res);
	bc_edge(&d, 115000, 0, 200, 103, &res);
	bc_poll(&d, 120000, 200, 103 + BC_MAX_LENGTH_UM / 1000 + 5, &res);
	TEST_CHECK("S3-U14", res.result == BCR_INVALID && res.err == BC_ERR_WINDOW,
		   "capture longer than the 450 mm window: aborted");
	TEST_SUMMARY("s3 capture");
	return test_fail_;
}

LOCAL INT test_nav_map(void)
{
	UB	nav;
	UW	type;
	INT	arg;
	BOOL	t_ok;

	TEST_INIT();
	nav_map_reset();
	TEST_CHECK("S3-U15", !nav_map_lookup((UB)c39_index_of_char('A'), &nav)
		   && !nav_map_lookup((UB)c39_index_of_char('Z'), &nav),
		   "default map is EMPTY: no invented barcode meanings");
	TEST_CHECK("S3-U16", nav_map_set((UB)c39_index_of_char('A'), NAV_LEFT) == E_OK
		   && nav_map_lookup((UB)c39_index_of_char('A'), &nav) && nav == NAV_LEFT
		   && nav_map_set(C39_STAR, NAV_LEFT) == E_PAR
		   && nav_map_set(1, 4) == E_PAR,
		   "configured mapping works; '*' and nav > 3 rejected");
	nav_to_motion(NAV_LEFT, &type, &arg);
	t_ok = (type == MCMD_TURN && arg == -90);
	nav_to_motion(NAV_RIGHT, &type, &arg);
	t_ok = t_ok && (type == MCMD_TURN && arg == 90);
	nav_to_motion(NAV_UTURN, &type, &arg);
	t_ok = t_ok && (type == MCMD_TURN && arg == 180);
	nav_to_motion(NAV_STRAIGHT, &type, &arg);
	t_ok = t_ok && (type == MCMD_MOVE && arg == S3_STRAIGHT_MM);
	TEST_CHECK("S3-U17", t_ok, "LEFT/RIGHT/U-TURN/STRAIGHT -> encoder TURN/MOVE commands");
	nav_map_reset();
	TEST_SUMMARY("s3 nav map");
	return test_fail_;
}

LOCAL INT test_line(void)
{
	LINE_FOLLOW	lf;
	LF_OUT		o;
	UB		cal[10];
	UINT		k;
	UW		t;
	W		odo = 0;
	BOOL		lost = FALSE, found = FALSE, fail = FALSE, junction = FALSE;
	BOOL		bar_ok = TRUE, held = TRUE, ok1;
	UINT		junction_events = 0, waits, arcs, pivots;

	TEST_INIT();
	lf_init(&lf);
	for (k = 0; k < 10U; k++) {
		cal[k] = 0x2U;			/* raw: C high (black), L/R low */
	}
	TEST_CHECK("S3-U18", lf_calibrate(&lf, cal, 10) == LF_CAL_OK && lf.black_level[IR_C] == 1,
		   "calibration on the line: black = high");
	for (k = 0; k < 10U; k++) {
		cal[k] = 0x5U;			/* inverted modules: C low = black */
	}
	TEST_CHECK("S3-U19", lf_calibrate(&lf, cal, 10) == LF_CAL_OK && lf.black_level[IR_C] == 0
		   && lf_black_bits(&lf, 0x5U) == IR_BIT(IR_C),
		   "inverted sensor polarity is learnt, not assumed");
	for (k = 0; k < 10U; k++) {
		cal[k] = 0x3U;			/* L and C equal */
	}
	TEST_CHECK("S3-U20", lf_calibrate(&lf, cal, 10) == LF_CAL_AMBIGUOUS,
		   "ambiguous start position -> calibration failure");
	for (k = 0; k < 10U; k++) {
		cal[k] = (k & 1U) ? 0x2U : 0x0U;
	}
	TEST_CHECK("S3-U21", lf_calibrate(&lf, cal, 10) == LF_CAL_UNSTABLE,
		   "flickering sensor while still -> calibration failure");

	lf_init(&lf);
	for (k = 0; k < 10U; k++) {
		cal[k] = 0x2U;
	}
	(void)lf_calibrate(&lf, cal, 10);
	lf_step(&lf, IR_BIT(IR_C), 0, 20, TRUE, &o);
	TEST_CHECK("S3-U22", o.steering && o.steer == 0 && o.speed_frac == 15 && lf.pos == LP_CENTER,
		   "centred: straight at full cruise fraction");
	lf_step(&lf, IR_BIT(IR_C) | IR_BIT(IR_L), 2, 20, TRUE, &o);
	TEST_CHECK("S3-U23", o.steer < 0 && lf.pos == LP_LEFT_NEAR, "line drifting to IR_L: steer towards it");
	lf_step(&lf, IR_BIT(IR_L), 4, 20, TRUE, &o);
	TEST_CHECK("S3-U24", o.steer == -LF_SIDE_PIVOT_STEER && o.speed_frac == 0
		   && lf.pos == LP_LEFT_FAR,
		   "line under IR_L only: stop and pivot towards it");

	/* the side pivot holds while all three read white: not lost, and
	   no lost-line turn the other way */
	for (t = 0; t < 400U; t += 20U) {
		odo += 4;
		lf_step(&lf, 0, odo, 20, TRUE, &o);
		lost = lost || ((o.events & LF_EV_LOST) != 0U);
		held = held && o.steer == -LF_SIDE_PIVOT_STEER && o.speed_frac == 0;
	}
	lf_step(&lf, IR_BIT(IR_C), odo, 20, TRUE, &o);
	TEST_CHECK("S3-U52", held && !lost && lf.state == LS_ON_LINE && lf.pivot_dir == 0
		   && o.steer == 0 && o.speed_frac == LF_SETTLE_SPEED,
		   "side pivot held through all-white until the centre has the line; then half speed, no kick back");

	/* centre and IR_L white: straight for LF_LOST_WAIT_MS, then arc
	   towards IR_R while moving on, then pivot on the spot after
	   LF_LOST_ARC_MS; LOST meanwhile */
	lf_reset_tracking(&lf);
	lf_step(&lf, IR_BIT(IR_C), odo, 20, TRUE, &o);
	waits = 0;
	arcs = 0;
	pivots = 0;
	held = TRUE;
	lost = FALSE;
	for (t = 0; t < 1000U; t += 20U) {
		odo += 1;				/* slow: the time limit ends the arc */
		lf_step(&lf, 0, odo, 20, TRUE, &o);
		lost = lost || ((o.events & LF_EV_LOST) != 0U);
		if (o.steer == 0 && o.speed_frac == LF_LOST_WAIT_SPEED) {
			held = held && arcs == 0U && pivots == 0U;	/* wait first */
			waits++;
		} else if (o.steer == LF_LOST_DIR * LF_LOST_ARC_STEER
			   && o.speed_frac == LF_LOST_ARC_SPEED) {
			held = held && pivots == 0U;	/* arc only before the pivot */
			arcs++;
		} else if (o.steer == LF_LOST_DIR * LF_LOST_PIVOT_STEER && o.speed_frac == 0) {
			pivots++;
		} else {
			held = FALSE;
		}
	}
	TEST_CHECK("S3-U25", held && lost && lf.state == LS_SEARCHING && lf.pos == LP_LOST_RIGHT
		   && waits == LF_LOST_WAIT_MS / 20U && arcs == LF_LOST_ARC_MS / 20U && pivots > 0U,
		   "centre and IR_L white: straight for LF_LOST_WAIT_MS, then arc towards IR_R (car's left), then pivot; LOST");
	lf_step(&lf, IR_BIT(IR_C), odo, 20, TRUE, &o);
	found = (o.events & LF_EV_FOUND) != 0U;
	ok1 = o.steer == LF_LOST_DIR * LF_REALIGN_STEER && o.speed_frac == LF_REALIGN_SPEED;
	for (t = 20; t < LF_REALIGN_MS; t += 20U) {
		lf_step(&lf, IR_BIT(IR_C), odo, 20, TRUE, &o);
		ok1 = ok1 && o.steer == LF_LOST_DIR * LF_REALIGN_STEER && o.speed_frac == LF_REALIGN_SPEED;
	}
	lf_step(&lf, IR_BIT(IR_C), odo, 20, TRUE, &o);
	TEST_CHECK("S3-U26", found && ok1 && lf.state == LS_ON_LINE && o.steer == 0
		   && o.speed_frac == 15,
		   "line reacquired -> FOUND; keeps turning gently towards IR_R for LF_REALIGN_MS, then straight");

	/* faster travel: the distance limit ends the arc */
	lf_reset_tracking(&lf);
	lf_step(&lf, IR_BIT(IR_C), odo, 20, TRUE, &o);
	arcs = 0;
	for (k = 0; k < 20U; k++) {
		odo += 10;
		lf_step(&lf, 0, odo, 20, TRUE, &o);
		if (o.speed_frac == LF_LOST_ARC_SPEED) {
			arcs++;			/* (wait, then arc: same speed) */
		}
	}
	TEST_CHECK("S3-U55", arcs == LF_LOST_WAIT_MS / 20U + (UINT)(LF_LOST_ARC_MM / 10)
		   && o.speed_frac == 0
		   && o.steer == LF_LOST_DIR * LF_LOST_PIVOT_STEER,
		   "lost-line arc ends after LF_LOST_ARC_MM of travel, then pivot");

	/* found by the turn, then the line drifts to IR_L: normal steering
	   towards IR_L wins over the gentle turn-on */
	lf_step(&lf, IR_BIT(IR_C), odo, 20, TRUE, &o);
	lf_step(&lf, IR_BIT(IR_C) | IR_BIT(IR_L), odo, 20, TRUE, &o);
	TEST_CHECK("S3-U56", o.steer < 0 && lf.pos == LP_LEFT_NEAR,
		   "turning on after a find, line drifts to IR_L: steer towards IR_L");

	/* never found: arcs, then keeps pivoting the same way, never gives up */
	held = TRUE;
	for (t = 0; t < 20000U; t += 20U) {
		lf_step(&lf, 0, odo, 20, TRUE, &o);
		fail = fail || (o.events & LF_EV_RECOVERY_FAIL) != 0U;
		if (t > LF_LOST_WAIT_MS + LF_LOST_ARC_MS) {
			held = held && o.steer == LF_LOST_DIR * LF_LOST_PIVOT_STEER && o.speed_frac == 0;
		}
	}
	TEST_CHECK("S3-U27", !fail && held && lf.state == LS_SEARCHING,
		   "line never found: keeps pivoting towards IR_R; no RECOVERY_FAIL");

	lf_reset_tracking(&lf);
	for (k = 0; k < 10U; k++) {
		odo += 2;
		lf_step(&lf, IR_BIT(IR_L) | IR_BIT(IR_C) | IR_BIT(IR_R), odo, 20, TRUE, &o);
		if ((o.events & LF_EV_JUNCTION) != 0U) {
			junction = TRUE;
			junction_events++;
		}
	}
	TEST_CHECK("S3-U28", junction && junction_events == 1 && lf.pos == LP_JUNCTION,
		   "all three sensors black for >= 8 mm: one JUNCTION event");

	/* IR_R black beside the centre, however long: never steers */
	lf_reset_tracking(&lf);
	for (k = 0; k <= 100U; k += 2U) {
		lf_step(&lf, IR_BIT(IR_C) | IR_BIT(IR_R), odo + (W)k, 20, TRUE, &o);
		bar_ok = bar_ok && lf.pos == LP_CENTER && o.steer == 0 && o.speed_frac == 15;
	}
	TEST_CHECK("S3-U29", bar_ok,
		   "black under the barcode sensor (IR_R) beside the line does not disturb steering");

	/* IR_R alone is not the line: the lost-line turn runs and the
	   search does not end */
	lf_reset_tracking(&lf);
	held = TRUE;
	lost = FALSE;
	found = FALSE;
	lf_step(&lf, IR_BIT(IR_C), odo, 20, TRUE, &o);
	for (k = 0; k < 30U; k++) {
		lf_step(&lf, (k & 1U) ? IR_BIT(IR_R) : 0U, odo, 20, TRUE, &o);
		lost = lost || (o.events & LF_EV_LOST) != 0U;
		found = found || (o.events & LF_EV_FOUND) != 0U;
		held = held && (k < LF_LOST_WAIT_MS / 20U ? o.steer == 0 : o.steer * LF_LOST_DIR > 0);
	}
	TEST_CHECK("S3-U53", held && lost && !found && lf.state == LS_SEARCHING,
		   "black under IR_R alone is not the line: lost, turning towards IR_R, search not ended");

	/* turning towards IR_R, then IR_L sees the line: pivot towards IR_L */
	lf_step(&lf, IR_BIT(IR_L), odo, 20, TRUE, &o);
	found = (o.events & LF_EV_FOUND) != 0U;
	TEST_CHECK("S3-U54", found && lf.pivot_dir == -1 && o.steer == -LF_SIDE_PIVOT_STEER
		   && o.speed_frac == 0,
		   "lost-line turn, then IR_L has the line: FOUND, pivot towards IR_L");

	/* centre flickers white for less than the wait: no turn at all,
	   and no turn-on afterwards */
	lf_reset_tracking(&lf);
	lf_step(&lf, IR_BIT(IR_C), odo, 20, TRUE, &o);
	held = TRUE;
	for (k = 0; k < LF_LOST_WAIT_MS / 20U; k++) {
		odo += 2;
		lf_step(&lf, 0, odo, 20, TRUE, &o);
		held = held && o.steer == 0 && o.speed_frac == LF_LOST_WAIT_SPEED
		       && (o.events & LF_EV_LOST) == 0U;
	}
	lf_step(&lf, IR_BIT(IR_C), odo, 20, TRUE, &o);
	TEST_CHECK("S3-U57", held && lf.state == LS_ON_LINE && o.steer == 0 && o.speed_frac == 15,
		   "line back within LF_LOST_WAIT_MS: straight on, no left turn, no turn-on, no LOST");
	TEST_SUMMARY("s3 line");
	return test_fail_;
}

LOCAL INT test_status_words(void)
{
	S3_LINE_STATUS	a, b;
	S3_BC_STATUS	c, d;

	TEST_INIT();
	a.ir_black = 7;
	a.pos = LP_JUNCTION;
	a.state = LS_UNKNOWN;
	a.seq = 15;
	a.steer = -63;
	a.calibrated = 1;
	a.junctions = 15;
	a.lost_count = 15;
	a.speed_frac = 15;
	s3_line_unpack(s3_line_pack(&a), &b);
	TEST_CHECK("S3-U30", b.ir_black == 7 && b.pos == LP_JUNCTION && b.state == LS_UNKNOWN
		   && b.seq == 15 && b.steer == -63 && b.calibrated == 1 && b.junctions == 15
		   && b.lost_count == 15 && b.speed_frac == 15,
		   "line status word round trip at limits (steer -63)");
	c.state = BCS_EXECUTING;
	c.symbol = C39_NONE;
	c.result = BCR_DUPLICATE;
	c.nav = NAV_UTURN;
	c.nav_valid = 1;
	c.reversed = 1;
	c.n_valid = 63;
	c.n_invalid = 63;
	c.mres = MRES_REJECTED;
	c.executed = 1;
	c.err_class = S3_BCE_ABORT;
	s3_bc_unpack(s3_bc_pack(&c), &d);
	TEST_CHECK("S3-U31", d.state == BCS_EXECUTING && d.symbol == C39_NONE && d.result == BCR_DUPLICATE
		   && d.nav == NAV_UTURN && d.nav_valid == 1 && d.reversed == 1 && d.n_valid == 63
		   && d.n_invalid == 63 && d.mres == MRES_REJECTED && d.executed == 1
		   && d.err_class == S3_BCE_ABORT,
		   "barcode status word round trip at limits");
	TEST_SUMMARY("s3 status words");
	return test_fail_;
}

EXPORT INT s3_run_unit_tests(void)
{
	return test_code39() + test_capture() + test_nav_map() + test_line()
	     + test_status_words();
}

#endif /* APP_TEST || HOST_TEST */
