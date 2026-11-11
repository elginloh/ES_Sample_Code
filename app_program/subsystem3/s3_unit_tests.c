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
	{
		UW recorded[29] = {
			75000,129000,307000,48000,343004,84996,64000,344000,73000,119000,
			287000,140000,84000,318000,73000,140000,72000,145000,295000,149000,
			72002,145998,293000,158000,275000,615000,111000,124000,241000
		};
		n = c39_decode(recorded, 29, data, 4, &rev);
		TEST_CHECK("S3-U61", n < 0 && rev,
			   "recorded reverse A with damaged final delimiter rejected as reverse");
		/* Replace only the damaged final delimiter with a valid reversed
		   star; preserve the measured first delimiter, A and gaps. */
		for (k = 0; k < 9U; k++) {
			recorded[20U + k] = (c39_pattern(C39_STAR) & (1U << k)) ? 300000U : 100000U;
		}
		n = c39_decode(recorded, 29, data, 4, &rev);
		TEST_CHECK("S3-U62", n == 1 && c39_char(data[0]) == 'A' && rev,
			   "measured reverse A decodes when final delimiter is valid");
	}

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
	{
		/* In an ideal 3:1 scan, *A* and *B* yield identical timings
		   when elements 2–4 and 11–13 each merge into a single run. */
		static const UW ambiguous[25] = {
			1, 5, 3, 1, 3, 1, 1, 1, 5, 1, 1, 3, 1,
			1, 3, 1, 1, 3, 1, 1, 3, 1, 3, 1, 1
		};
		TEST_CHECK("S3-U79", c39_decode(ambiguous, 25U, data, 4U, &rev) == C39_ERR_COUNT,
			   "25-element scan shared by A and B cannot queue a turn");
	}
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
	{
		UW recorded[29] = {
			183000,128000,469000,125000,555000,71000,269000,582000,100000,262008,
			109994,250004,153994,579000,402000,264000,436000,195002,122998,298002,
			79998,171000,618004,82996,787004,286996,256000,640000,98000
		};
		n = c39_decode(recorded, 29U, data, 4U, &rev);
		TEST_CHECK("S3-U72", n == 1 && rev && c39_char(data[0]) == 'F',
			   "recorded reverse F survives one short narrow-space measurement");
		recorded[19] = 600000U;
		TEST_CHECK("S3-U73", c39_decode(recorded, 29U, data, 4U, &rev) == C39_ERR_GAP && rev,
			   "reverse F still rejects a genuinely oversized gap");
	}
	{
		/* Independent Code 39 encodings from ZXing Code39Reader's A–Z table. */
		static const UH alphabet[] = {
			0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00D, 0x10C, 0x04C, 0x01C,
			0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106, 0x046, 0x016,
			0x181, 0x0C1, 0x1C0, 0x091, 0x190, 0x0D0
		};
		UINT forward_ok = 0, reverse_ok = 0;
		static const UINT label_mm[] = {170U, 250U, 339U};
		UINT scale_ok = 0;
		UINT recoverable_directions = 0, rejected_merges = 0, wrong_merges = 0;

		for (i = 0; i < sizeof(alphabet) / sizeof(alphabet[0]); i++) {
			char expected = (char)('A' + i);

			/* Use the measured delimiters and gaps from the printed A. */
			for (k = 0; k < 29U; k++) {
				el[k] = bc_vectors[0].el[k];
			}
			for (k = 0; k < C39_ELEMENTS; k++) {
				el[C39_STRIDE + k] = (alphabet[i] & (1U << (8U - k))) ? 120U : 40U;
			}
			for (UINT size = 0; size < sizeof(label_mm) / sizeof(label_mm[0]); size++) {
				/* Approximate the deck sizes at a steady 32 mm/s.
				   A 3:1 single-letter Code 39 label spans 47 units. */
				UW narrow_us = label_mm[size] * 1000000UL / (47U * 32U);

				for (UINT direction = 0; direction < 2U; direction++) {
					BC_DEC capture;
					BC_RESULT result;
					UW now = 100000U, widest = 0U;
					UB level = 1U;
					BOOL intact = TRUE;

					bc_init(&capture);
					bc_edge(&capture, now, level, 32, 0, &result);
					for (k = 0; k < 29U; k++) {
						UINT pos = direction ? 28U - k : k;
						UW duration = el[pos] * narrow_us / 40U;

						for (UW tick = 20000U; tick < duration; tick += 20000U) {
							bc_poll(&capture, now + tick, 32, 0, &result);
							intact = intact && result.result == BCR_NONE;
						}
						if (duration > widest) {
							widest = duration;
						}
						now += duration;
						level ^= 1U;
						bc_edge(&capture, now, level, 32, 0, &result);
						intact = intact && result.result == BCR_NONE;
					}
					bc_poll(&capture, now + widest * 25U / 10U + 20000U, 32, 0, &result);
					if (intact && result.result == BCR_VALID && result.n_data == 1U
					    && result.reversed == (direction != 0U)
					    && c39_char(result.data[0]) == expected) {
						scale_ok++;
					} else {
						tm_printf((UB*)"[TEST] %c size %d mm reverse %d failed, err %d\n",
							  expected, (INT)label_mm[size], (INT)direction, result.err);
					}
				}
			}
			n = c39_decode(el, 29U, data, 4U, &rev);
			if (n == 1 && !rev && c39_char(data[0]) == expected) {
				forward_ok++;
			} else {
				tm_printf((UB*)"[TEST] forward %c failed, result %d\n", expected, n);
			}
			for (UINT direction = 0; direction < 2U; direction++) {
				BOOL recovered = FALSE;

				for (UINT merge = 0; merge < C39_MERGED_ELEMENTS; merge++) {
					UW merged[C39_MERGED_ELEMENTS];
					UINT pos = 0;

					for (k = 0; k < C39_SINGLE_ELEMENTS; k++) {
						UW width = el[direction ? C39_SINGLE_ELEMENTS - 1U - k : k];
						if (k == merge + 1U || k == merge + 2U) {
							merged[pos - 1U] += width;
						} else {
							merged[pos++] = width;
						}
					}
					n = c39_decode(merged, C39_MERGED_ELEMENTS, data, 4U, &rev);
					if (n == 1 && c39_char(data[0]) == expected && rev == (direction != 0U)) {
						recovered = TRUE;
					} else if (n < 0) {
						rejected_merges++;
					} else {
						wrong_merges++;
					}
				}
				if (recovered) {
					recoverable_directions++;
				}
			}
			for (k = 0; k < 14U; k++) {
				UW width = el[k];
				el[k] = el[28U - k];
				el[28U - k] = width;
			}
			n = c39_decode(el, 29U, data, 4U, &rev);
			if (n == 1 && rev && c39_char(data[0]) == expected) {
				reverse_ok++;
			} else {
				tm_printf((UB*)"[TEST] reverse %c failed, result %d\n", expected, n);
			}
		}
		TEST_CHECK("S3-U74", forward_ok == 26U, "all A–Z decode in forward direction");
		TEST_CHECK("S3-U75", reverse_ok == 26U, "all A–Z decode in reverse direction");
		TEST_CHECK("S3-U78", scale_ok == 156U,
			   "A-Z capture and decode both ways at 170, 250 and 339 mm (156 cases)");
		TEST_CHECK("S3-U80", recoverable_directions == 52U && wrong_merges == 0U,
			   "27-element recovery covers A-Z both ways with no wrong letter across all merge positions");
		TEST_CHECK("S3-U81", rejected_merges > 0U,
			   "ambiguous 27-element captures are rejected");
	}
	{
		static const UW reverse_f[29] = {
			160000,111000,485000,186000,582000,48000,312000,557000,127002,201998,
			225000,89000,279000,623000,378000,57000,601000,90000,277000,104000,
			364000,97000,585000,42000,523000,76000,299000,487000,92010
		};
		static const UW ambiguous_f[29] = {
			274000,187000,103000,64000,327000,95000,332000,116000,137004,112996,
			102000,79000,341000,83000,395000,424000,130000,39000,155000,125000,
			249000,439000,132000,76000,381000,69000,558004,98996,141004
		};
		n = c39_decode(reverse_f, 29U, data, 4U, &rev);
		TEST_CHECK("S3-U76", n == 1 && rev && c39_char(data[0]) == 'F',
			   "recorded reverse F with 1.35x bar separation decodes");
		n = c39_decode(ambiguous_f, 29U, data, 4U, &rev);
		TEST_CHECK("S3-U77", n == C39_ERR_WIDTH,
			   "recorded forward scan with 1.19x bar separation is rejected");
	}
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
	TEST_CHECK("S3-U11a", res.result == BCR_VALID && c39_char(res.data[0]) == 'Z'
		   && res.n_diag == 29U && res.element_um[0] > 0U,
		   "first pass over Z: VALID, with 29 diagnostic element widths");
	/* past the 150 mm lockout, inside the 600 mm duplicate window */
	s3_drive_over(&d, 'Z', 200, x + 200000, 0, &t, &x, &res);
	TEST_CHECK("S3-U11", res.result == BCR_DUPLICATE,
		   "same barcode again within 600 mm: DUPLICATE (no second command)");

	/* the line itself under the sensor: one 60 mm black run */
	bc_init(&d);
	bc_edge(&d, 100000, 1, 200, 100, &res);
	bc_edge(&d, 400000, 0, 200, 160, &res);
	bc_poll(&d, 400000 + BC_IDLE_TIMEOUT_US + 1000U, 0, 160, &res);
	TEST_CHECK("S3-U12", res.result == BCR_INVALID && !bc_capturing(&d),
		   "isolated black run times out without a complete symbol");

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
	TEST_CHECK("S3-U14", res.result == BCR_NONE && bc_capturing(&d),
		   "wheel odometry jump does not affect sensor capture");
	{
		UINT symbol, element;
		UW now = 100000U;
		UB level = 1U;

		bc_init(&d);
		bc_edge(&d, now, level, 0, 0, &res);
		for (symbol = 0; symbol < 3U; symbol++) {
			UH pattern = c39_pattern(symbol == 1U ? 10U : C39_STAR);

			for (element = 0; element < 9U; element++) {
				now += (pattern & (1U << (8U - element))) ? 90000U : 30000U;
				level ^= 1U;
				bc_edge(&d, now, level, 0, 0, &res);
			}
			if (symbol < 2U) {
				now += 30000U;
				level ^= 1U;
				bc_edge(&d, now, level, 0, 0, &res);
			}
		}
		bc_poll(&d, now + 300000U, 0, 0, &res);
		TEST_CHECK("S3-U60", res.result == BCR_VALID && res.n_diag == 29U
			   && c39_char(res.data[0]) == 'A',
			   "stationary wheels: digital transition timing decodes A");
	}
	{
		UW widths[29] = {
			71000,362000,76000,148000,293000,132000,313000,157000,51000,162000,
			284000,150000,62000,117012,167988,337000,70000,158000,282000,151000,
			70000,401000,34000,193000,250000,204000,278000,148000,76002
		};
		UW now = 100000U;
		UINT k;
		UB level = 1U;
		BOOL intact = TRUE;

		bc_init(&d);
		bc_edge(&d, now, level, 0, 0, &res);
		for (k = 0; k < 29U; k++) {
			UW elapsed;
			for (elapsed = 20000U; elapsed < widths[k]; elapsed += 20000U) {
				bc_poll(&d, now + elapsed, 0, 0, &res);
				if (res.result != BCR_NONE) intact = FALSE;
			}
			now += widths[k];
			level ^= 1U;
			bc_edge(&d, now, level, 0, 0, &res);
		}
		bc_poll(&d, now + 1100000U, 0, 0, &res);
		TEST_CHECK("S3-U63", intact && res.result == BCR_VALID
			   && c39_char(res.data[0]) == 'A' && !res.reversed,
			   "exact recorded A: wide first white space preserved, biased gaps accepted");
	}
	{
		W widths[C39_SINGLE_ELEMENTS];
		UW narrow = 339000000UL / (47U * 32U);
		UW now = 100000U;
		UB level = 1U;
		UINT k;

		(void)build_barcode('W', 0, widths);
		bc_init(&d);
		bc_edge(&d, now, level, 32, 0, &res);
		for (k = 0; k < C39_SINGLE_ELEMENTS; k++) {
			W width = widths[k];
			if (k == C39_STRIDE) {
				/* First three elements of W are all wide. */
				width += widths[k + 1U] + widths[k + 2U];
				k += 2U;
			}
			now += (UW)(width / SIM_NARROW_UM) * narrow;
			level ^= 1U;
			bc_edge(&d, now, level, 32, 0, &res);
		}
		bc_poll(&d, now + 1800000U, 32, 0, &res);
		TEST_CHECK("S3-U84", res.result == BCR_VALID && res.n_elements == C39_MERGED_ELEMENTS
			   && c39_char(res.data[0]) == 'W',
			   "339 mm W with 2 s merged run finishes capture before the stall timeout");
	}
	TEST_SUMMARY("s3 capture");
	return test_fail_;
}

LOCAL INT test_nav_map(void)
{
	UB	nav;
	UW	type;
	INT	arg;
	BOOL	t_ok, map_ok;

	TEST_INIT();
	nav_map_reset();
	map_ok = nav_map_lookup((UB)c39_index_of_char('A'), &nav) && nav == NAV_RIGHT
		 && nav_map_lookup((UB)c39_index_of_char('F'), &nav) && nav == NAV_LEFT
		 && nav_map_lookup((UB)c39_index_of_char('Z'), &nav) && nav == NAV_LEFT
		 && nav_map_lookup((UB)c39_index_of_char('X'), &nav) && nav == NAV_STRAIGHT
		 && nav_map_lookup((UB)c39_index_of_char('U'), &nav) && nav == NAV_UTURN
		 && !nav_map_lookup((UB)c39_index_of_char('W'), &nav);
	TEST_CHECK("S3-U15", map_ok,
		   "default barcode mapping: A=RIGHT, F=LEFT, Z=LEFT, X=STRAIGHT, U=U-TURN");
	t_ok = nav_map_set((UB)c39_index_of_char('A'), NAV_LEFT) == E_OK
		   && nav_map_lookup((UB)c39_index_of_char('A'), &nav) && nav == NAV_LEFT
		   && nav_map_set(C39_STAR, NAV_LEFT) == E_PAR
		   && nav_map_set(1, 4) == E_PAR;
	nav_map_reset();
	t_ok = t_ok && nav_map_lookup((UB)c39_index_of_char('A'), &nav) && nav == NAV_RIGHT;
	TEST_CHECK("S3-U16", t_ok,
		   "mapping overrides work; '*' and nav > 3 rejected; reset restores defaults");
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
	UINT		junction_events = 0, arcs, pivots, grace;

	TEST_INIT();
	lf_init(&lf);
	for (k = 0; k < 10U; k++) {
		cal[k] = 0x2U;			/* raw: C high (black), L/R low */
	}
	TEST_CHECK("S3-U18", lf_calibrate(&lf, cal, 10) == LF_CAL_OK && lf.black_level[IR_C] == 1,
		   "stable centre-on-black start: configured black-high polarity");
	for (k = 0; k < 10U; k++) {
		cal[k] = 0x0U;			/* all sensors over white */
	}
	TEST_CHECK("S3-U19", lf_calibrate(&lf, cal, 10) == LF_CAL_OK
		   && lf.black_level[IR_C] == LF_CONFIGURED_BLACK_LEVEL
		   && lf_black_bits(&lf, 0x0U) == 0U,
		   "all-white start is stable and immediately qualifies IR startup");
	for (k = 0; k < 10U; k++) {
		cal[k] = 0x7U;			/* all sensors over black */
	}
	TEST_CHECK("S3-U20", lf_calibrate(&lf, cal, 10) == LF_CAL_OK
		   && lf_black_bits(&lf, 0x7U) == 0x7U,
		   "all-black start is stable and immediately qualifies IR startup");
	for (k = 0; k < 10U; k++) {
		cal[k] = (UB)IR_BIT(IR_L);	/* physical-right GP2 on black */
	}
	TEST_CHECK("S3-U59", lf_calibrate(&lf, cal, 10) == LF_CAL_OK
		   && lf_black_bits(&lf, IR_BIT(IR_L)) == IR_BIT(IR_L),
		   "physical-right-only black start qualifies and preserves its position");
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
	TEST_CHECK("S3-U22", o.steering && o.steer == LF_EDGE_STEER_MIN
		   && o.speed_frac == LF_NEAR_SPEED && lf.pos == LP_RIGHT_NEAR,
		   "centre black: gradual physical-left correction through mirrored steering");
	lf_step(&lf, IR_BIT(IR_C) | IR_BIT(IR_L), 2, 20, TRUE, &o);
	TEST_CHECK("S3-U23", o.steer == 0 && o.speed_frac == LF_CRUISE_SPEED
		   && lf.pos == LP_JUNCTION,
		   "wide line filling the C-to-IR_L corridor: drive straight");
	lf_step(&lf, IR_BIT(IR_L), 4, 20, TRUE, &o);
	TEST_CHECK("S3-U24", o.steer == -LF_EDGE_STEER_MIN
		   && o.speed_frac == LF_NEAR_SPEED
		   && lf.pos == LP_LEFT_NEAR,
		   "line on outer boundary: smooth moving correction, never a pivot");

	/* Both steering sensors white is the desired corridor, not an
	   immediate loss or stop-and-pivot. */
	for (t = 0; t < 400U; t += 20U) {
		odo += 4;
		lf_step(&lf, 0, odo, 20, TRUE, &o);
		lost = lost || ((o.events & LF_EV_LOST) != 0U);
		held = held && o.steer == 0
		       && o.speed_frac == LF_NEAR_SPEED;
	}
	lf_step(&lf, IR_BIT(IR_C), odo, 20, TRUE, &o);
	TEST_CHECK("S3-U52", held && !lost && lf.state == LS_ON_LINE
		   && lf.pivot_dir == 0 && o.steer == LF_EDGE_STEER_MIN
		   && o.speed_frac == LF_NEAR_SPEED,
		   "white corridor straight; centre black ramps physical-left correction");

	/* Centre and IR_L white: suppress two isolated samples, then arc
	   gently towards IR_R while moving on, and finally pivot. */
	lf_reset_tracking(&lf);
	lf_step(&lf, IR_BIT(IR_C), odo, 20, TRUE, &o);
	/* Explicit known-loss setup: normal both-white no longer means loss. */
	lf.turning = TRUE;
	lf.turn_start_mm = odo;
	lf.turn_ms = 0;
	arcs = 0;
	pivots = 0;
	grace = 0;
	held = TRUE;
	lost = FALSE;
	for (t = 0; t < 1400U; t += 20U) {
		odo += 1;				/* slow: the time limit ends the arc */
		lf_step(&lf, 0, odo, 20, TRUE, &o);
		lost = lost || ((o.events & LF_EV_LOST) != 0U);
		if (o.steer == 0
		    && o.speed_frac == LF_NEAR_SPEED && lf.state == LS_ON_LINE) {
			grace++;
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
	TEST_CHECK("S3-U25", held && lost && lf.state == LS_SEARCHING && lf.pos == LP_LOST_LEFT
		   && grace == 0U
		   && arcs == LF_LOST_ARC_MS / 20U && pivots > 0U,
		   "explicit known loss uses an arc, then pivot; normal white corridor does not trigger it");
	lf_step(&lf, IR_BIT(IR_C), odo, 20, TRUE, &o);
	found = (o.events & LF_EV_FOUND) != 0U;
	ok1 = o.steer == LF_EDGE_STEER_MIN && o.speed_frac == LF_NEAR_SPEED;
	TEST_CHECK("S3-U26", found && ok1 && lf.state == LS_ON_LINE,
		   "line reacquired on centre -> FOUND and gradual physical-left correction");

	/* One or two white samples between centred samples must not alter the
	   motor command or create a LOST/FOUND event pair. */
	lf_reset_tracking(&lf);
	lf_step(&lf, IR_BIT(IR_C), odo, 20, TRUE, &o);
	lf_step(&lf, 0, odo + 2, 20, TRUE, &o);
	held = o.steer == 0
	       && o.speed_frac == LF_NEAR_SPEED && o.events == 0;
	lf_step(&lf, 0, odo + 4, 20, TRUE, &o);
	held = held && o.steer == 0
	       && o.speed_frac == LF_NEAR_SPEED && o.events == 0;
	lf_step(&lf, IR_BIT(IR_C), odo + 6, 20, TRUE, &o);
	TEST_CHECK("S3-U58", held && o.steer == LF_EDGE_STEER_MIN
		   && o.speed_frac == LF_NEAR_SPEED
		   && o.events == 0 && lf.state == LS_ON_LINE,
		   "white corridor gradually returns to straight with no loss event");

	/* a short slip off a straight: the arc finds it within
	   LF_REALIGN_MIN_MS, then straight on (turning on weaved) */
	lf_reset_tracking(&lf);
	lf_step(&lf, IR_BIT(IR_C), odo, 20, TRUE, &o);
	for (t = 20; t < LF_REALIGN_MIN_MS; t += 20U) {
		odo += 2;
		lf_step(&lf, 0, odo, 20, TRUE, &o);
	}
	ok1 = o.steer == 0;
	lf_step(&lf, IR_BIT(IR_C), odo, 20, TRUE, &o);
	TEST_CHECK("S3-U57", ok1 && lf.realign_ms == 0U
		   && o.steer == LF_EDGE_STEER_MIN
		   && o.speed_frac == LF_NEAR_SPEED,
		   "short white corridor remains straight and ramps smoothly at the centre edge");

	/* faster travel: the distance limit ends the arc */
	lf_reset_tracking(&lf);
	lf_step(&lf, IR_BIT(IR_C), odo, 20, TRUE, &o);
	/* Explicit known-loss setup: normal both-white no longer means loss. */
	lf.turning = TRUE;
	lf.turn_start_mm = odo;
	lf.turn_ms = 0;
	arcs = 0;
	for (k = 0; k < 20U; k++) {
		odo += 10;
		lf_step(&lf, 0, odo, 20, TRUE, &o);
		if (o.speed_frac == LF_LOST_ARC_SPEED) {
			arcs++;
		}
	}
	TEST_CHECK("S3-U55", arcs == (UINT)(LF_LOST_ARC_MM / 10 - 1) && o.speed_frac == 0
		   && o.steer == LF_LOST_DIR * LF_LOST_PIVOT_STEER,
		   "lost-line arc ends after LF_LOST_ARC_MM of travel, then pivot");

	/* found by the turn, then the line drifts to IR_L: normal steering
	   towards IR_L wins over the gentle turn-on */
	lf_step(&lf, IR_BIT(IR_C), odo, 20, TRUE, &o);
	lf_step(&lf, IR_BIT(IR_C) | IR_BIT(IR_L), odo, 20, TRUE, &o);
	TEST_CHECK("S3-U56", o.steer == 0 && lf.pos == LP_JUNCTION,
		   "line spanning both corridor edges after a find: drive straight");

	/* Explicit known loss: continue search without latching safe stop. */
	lf.turning = TRUE;
	lf.turn_start_mm = odo;
	lf.turn_ms = 0;
	held = TRUE;
	for (t = 0; t < 20000U; t += 20U) {
		lf_step(&lf, 0, odo, 20, TRUE, &o);
		fail = fail || (o.events & LF_EV_RECOVERY_FAIL) != 0U;
		if (t > LF_CENTER_GAP_MS + LF_LOST_ARC_MS) {
			held = held && o.steer == LF_LOST_DIR * LF_LOST_PIVOT_STEER && o.speed_frac == 0;
		}
	}
	TEST_CHECK("S3-U27", !fail && held && lf.state == LS_SEARCHING,
		   "line never found: keeps pivoting towards IR_R; no RECOVERY_FAIL");

	lf_reset_tracking(&lf);
	for (k = 0; k < 10U; k++) {
		odo += 2;
		lf_step(&lf, IR_BIT(IR_L) | IR_BIT(IR_C), odo, 20, TRUE, &o);
		if ((o.events & LF_EV_JUNCTION) != 0U) {
			junction = TRUE;
			junction_events++;
		}
	}
	TEST_CHECK("S3-U28", junction && junction_events == 1 && lf.pos == LP_JUNCTION,
		   "GP6 + GP2 black for 60 ms, barcode white: one JUNCTION event");

	/* IR_R black beside the centre, however long: never steers */
	lf_reset_tracking(&lf);
	for (k = 0; k <= 100U; k += 2U) {
		lf_step(&lf, IR_BIT(IR_C) | IR_BIT(IR_R), odo + (W)k, 20, TRUE, &o);
		bar_ok = bar_ok && lf.pos == LP_RIGHT_NEAR && o.steer > 0
			 && o.steer <= LF_EDGE_STEER_MAX
			 && o.speed_frac == LF_NEAR_SPEED;
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
	/* Explicit known-loss setup: normal both-white no longer means loss. */
	lf.turning = TRUE;
	lf.turn_start_mm = odo;
	lf.turn_ms = 0;
	for (k = 0; k < 60U; k++) {
		lf_step(&lf, (k & 1U) ? IR_BIT(IR_R) : 0U, odo, 20, TRUE, &o);
		lost = lost || (o.events & LF_EV_LOST) != 0U;
		found = found || (o.events & LF_EV_FOUND) != 0U;
		held = held && o.steer * LF_LOST_DIR > 0;
	}
	TEST_CHECK("S3-U53", held && lost && !found && lf.state == LS_SEARCHING,
		   "black under IR_R alone is not the line: lost, turning towards IR_R, search not ended");

	/* Lost-line search, then IR_L sees the line: resume a moving edge correction. */
	lf_step(&lf, IR_BIT(IR_L), odo, 20, TRUE, &o);
	found = (o.events & LF_EV_FOUND) != 0U;
	TEST_CHECK("S3-U54", found && lf.pivot_dir == 0
		   && o.steer == -LF_EDGE_STEER_MIN
		   && o.speed_frac == LF_NEAR_SPEED,
		   "lost-line turn, then IR_L has the line: FOUND, smooth moving correction");
	{
		LINE_FOLLOW track_a, track_b;
		LF_OUT a, b;
		UB samples[10];
		BOOL independent = TRUE;
		UINT n;

		for (n = 0; n < 10U; n++) {
			samples[n] = (UB)(IR_BIT(IR_C) | ((n & 1U) ? IR_BIT(IR_R) : 0U));
		}
		lf_init(&track_a);
		lf_init(&track_b);
		TEST_CHECK("S3-U64", lf_calibrate(&track_a, samples, 10U) == LF_CAL_OK
			   && lf_calibrate(&track_b, samples, 10U) == LF_CAL_OK,
			   "barcode toggles on GP26 do not block stable GP6/GP2 startup");
		/* Exercise all tracking combinations, both vehicle-ownership
		   modes and loss/recovery, while the second barcode input toggles. */
		for (n = 0; n < 400U; n++) {
			UB tracking = (UB)((n / 50U) & 3U);
			BOOL navigating = n < 200U;
			lf_step(&track_a, tracking, (W)n, 20U, navigating, &a);
			lf_step(&track_b, (UB)(tracking | ((n & 1U) ? IR_BIT(IR_R) : 0U)),
				(W)n, 20U, navigating, &b);
			independent = independent && a.steer == b.steer
				&& a.speed_frac == b.speed_frac && a.events == b.events
				&& a.steering == b.steering && track_a.state == track_b.state;
		}
		TEST_CHECK("S3-U65", independent,
			   "barcode input cannot change line steering, speed, junction or recovery events");
	}
	{
		BOOL corridor_ok = TRUE;
		lf_reset_tracking(&lf);
		for (k = 0; k < 1500U; k++) {
			lf_step(&lf, (k & 1U) ? IR_BIT(IR_R) : 0U, odo + (W)k, 20U, TRUE, &o);
			corridor_ok = corridor_ok && o.steer == 0 && o.speed_frac == LF_CRUISE_SPEED
				&& o.events == 0U && lf.state == LS_ON_LINE;
		}
		TEST_CHECK("S3-U66", corridor_ok,
			   "30 s both-white corridor remains straight; barcode toggling never triggers search");
	}
	{
		BOOL smooth = TRUE;
		INT previous = 0;
		lf_reset_tracking(&lf);
		for (k = 0; k < 40U; k++) {
			UB bits = k < 10U ? IR_BIT(IR_C) : k < 20U ? IR_BIT(IR_L) : 0U;
			INT delta;
			lf_step(&lf, bits, odo + (W)k, 20U, TRUE, &o);
			delta = o.steer - previous;
			smooth = smooth && delta >= -LF_STEER_SLEW_STEP
				&& delta <= LF_STEER_SLEW_STEP && o.speed_frac == LF_CRUISE_SPEED;
			/* 10 samples on one sensor: grown to MIN + 180 ms / GROW */
			if (k == 9U) smooth = smooth
				&& o.steer == LF_EDGE_STEER_MIN + 180 / LF_EDGE_GROW_MS; /* mirrored: LEFT */
			if (k == 19U) smooth = smooth
				&& o.steer == -(LF_EDGE_STEER_MIN + 180 / LF_EDGE_GROW_MS); /* mirrored: RIGHT */
			/* 200 ms contact >= LF_HOLD_MIN_MS: the turn is held */
			if (k >= 20U && k < 20U + LF_HOLD_MS / 20U) smooth = smooth
				&& o.steer == -LF_HOLD_STEER;
			previous = o.steer;
		}
		TEST_CHECK("S3-U67", smooth && o.steer == 0,
			   "centre LEFT -> right sensor RIGHT -> white: hold, then STRAIGHT; fixed speed");
	}
	{
		BOOL responsive;
		lf_reset_tracking(&lf);
		lf_step(&lf, IR_BIT(IR_C), odo, 20U, TRUE, &o);
		responsive = o.steer == LF_EDGE_STEER_MIN && o.speed_frac == LF_CRUISE_SPEED;
		lf_step(&lf, IR_BIT(IR_C), odo + 1, 20U, TRUE, &o);
		for (k = 0; k < 2U; k++) {
			lf_step(&lf, IR_BIT(IR_L), odo + 2 + (W)k, 20U, TRUE, &o);
		}
		TEST_CHECK("S3-U68", responsive && o.steer == -LF_EDGE_STEER_MIN
			   && o.speed_frac == LF_CRUISE_SPEED,
			   "edge correction in the first 20 ms; reverse correction within 40 ms, wheels keep moving");
	}
	{
		BOOL capped = TRUE;
		INT peak = 0;
		lf_reset_tracking(&lf);
		for (k = 0; k < 50U; k++) {
			lf_step(&lf, IR_BIT(IR_C), odo + (W)k, 20U, TRUE, &o);
			capped = capped && o.steer >= LF_EDGE_STEER_MIN
				 && o.steer <= LF_EDGE_STEER_MAX && o.steer >= peak;
			peak = o.steer;
		}
		TEST_CHECK("S3-U69", capped && peak == LF_EDGE_STEER_MAX
			   && o.speed_frac == LF_CRUISE_SPEED,
			   "centre stays black (curve): correction grows to LF_EDGE_STEER_MAX, never beyond");
	}
	{
		BOOL ok_short, ok_hold = TRUE;
		/* short drift on a straight: straight again at once */
		lf_reset_tracking(&lf);
		for (k = 0; k < LF_HOLD_MIN_MS / 20U - 1U; k++) {
			lf_step(&lf, IR_BIT(IR_C), odo, 20U, TRUE, &o);
		}
		lf_step(&lf, 0, odo, 20U, TRUE, &o);
		ok_short = o.steer >= 0 && o.steer < LF_HOLD_STEER;	/* slewing to 0 */
		lf_step(&lf, 0, odo, 20U, TRUE, &o);
		ok_short = ok_short && o.steer == 0;
		/* sustained contact: hold the turn for LF_HOLD_MS, then straight */
		lf_reset_tracking(&lf);
		for (k = 0; k < LF_HOLD_MIN_MS / 20U; k++) {
			lf_step(&lf, IR_BIT(IR_C), odo, 20U, TRUE, &o);
		}
		for (k = 0; k < LF_HOLD_MS / 20U; k++) {
			lf_step(&lf, 0, odo, 20U, TRUE, &o);
			ok_hold = ok_hold && o.steer == LF_HOLD_STEER && o.events == 0U;
		}
		lf_step(&lf, 0, odo, 20U, TRUE, &o);
		TEST_CHECK("S3-U70", ok_short && ok_hold && o.steer == 0
			   && lf.state == LS_ON_LINE,
			   "short contact -> straight; sustained contact -> turn held LF_HOLD_MS, then straight");
	}
	{
		BOOL ok_opp, ok_jn;
		lf_reset_tracking(&lf);
		for (k = 0; k < 10U; k++) {
			lf_step(&lf, IR_BIT(IR_C), odo, 20U, TRUE, &o);
		}
		lf_step(&lf, 0, odo, 20U, TRUE, &o);
		lf_step(&lf, IR_BIT(IR_L), odo, 20U, TRUE, &o);
		lf_step(&lf, IR_BIT(IR_L), odo, 20U, TRUE, &o);
		ok_opp = o.steer == -LF_EDGE_STEER_MIN;
		lf_reset_tracking(&lf);
		for (k = 0; k < 10U; k++) {
			lf_step(&lf, IR_BIT(IR_C), odo, 20U, TRUE, &o);
		}
		lf_step(&lf, 0, odo, 20U, TRUE, &o);
		lf_step(&lf, IR_BIT(IR_C) | IR_BIT(IR_L), odo, 20U, TRUE, &o);
		lf_step(&lf, 0, odo, 20U, TRUE, &o);
		ok_jn = o.steer == 0;
		TEST_CHECK("S3-U71", ok_opp && ok_jn,
			   "held turn ends at once when the other sensor sees black or at a junction");
	}
	{
		BOOL ok_pivot = TRUE, ok_aligned = TRUE, ok_quiet = TRUE;
		/* Left barcode turn: while Buddy 2 turns, the centre sweeps
		   over the new line (long contact) and both sensors are white
		   for a long time.  The task resets tracking every period. */
		for (k = 0; k < 100U; k++) {
			lf_reset_tracking(&lf);
			lf_step(&lf, (k >= 40U && k < 55U) ? IR_BIT(IR_C) : 0U,
				odo, 20U, TRUE, &o);
			ok_quiet = ok_quiet && o.events == 0U;
		}
		/* Overshot: GP2 (IR_L) on the line when the turn ends.
		   Pivot on the spot towards it (right) until it is white. */
		lf_turn_done(&lf, FALSE);
		for (k = 0; k < 5U; k++) {
			lf_step(&lf, IR_BIT(IR_L), odo, 20U, TRUE, &o);
			ok_pivot = ok_pivot && o.speed_frac == 0
				   && o.steer == -LF_POST_TURN_PIVOT_STEER;
		}
		/* line now between the sensors: straight, no held turn */
		for (k = 0; k < 30U; k++) {
			lf_step(&lf, 0, odo + (W)k, 20U, TRUE, &o);
			ok_pivot = ok_pivot && o.steer == 0 && o.speed_frac > 0
				   && o.events == 0U;
		}
		ok_pivot = ok_pivot && !lf.post_turn && lf.state == LS_ON_LINE;
		/* Turn ended with the line already between the sensors:
		   straight on, and later contacts use normal edge steering. */
		lf_turn_done(&lf, FALSE);
		for (k = 0; k < LF_POST_TURN_MS / 20U; k++) {
			lf_step(&lf, 0, odo, 20U, TRUE, &o);
			ok_aligned = ok_aligned && o.steer == 0
				     && o.speed_frac == LF_CRUISE_SPEED;
		}
		lf_step(&lf, 0, odo, 20U, TRUE, &o);
		lf_step(&lf, IR_BIT(IR_L), odo, 20U, TRUE, &o);
		ok_aligned = ok_aligned && !lf.post_turn
			     && o.steer == -LF_EDGE_STEER_MIN && o.speed_frac > 0;
		TEST_CHECK("S3-U82", ok_quiet && ok_pivot && ok_aligned,
			   "after a barcode turn: GP2 on the line -> pivot right until white, then straight");
		lf_turn_done(&lf, TRUE);
		ok_aligned = TRUE;
		for (k = 0; k < LF_RIGHT_TURN_STRAIGHT_MS; k += 20U) {
			lf_step(&lf, IR_BIT(IR_C), odo, 20U, TRUE, &o);
			ok_aligned = ok_aligned && o.steer == 0 && o.speed_frac == LF_SETTLE_SPEED;
		}
		lf_step(&lf, IR_BIT(IR_C), odo, 20U, TRUE, &o);
		TEST_CHECK("S3-U85", ok_aligned && o.steer > 0,
			   "right branch under GP6: drive straight briefly then resume normal steering");
	}
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
