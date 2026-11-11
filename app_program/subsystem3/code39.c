/*
 *	code39.c
 *	Code 39 pattern table, width classification and run decoding.
 *
 *	Code 39 rule: each symbol is 5 bars and 4 spaces.  43 of the 44
 *	symbols have 2 wide bars and 1 wide space; the bar pair is one of ten
 *	"digit" patterns and the wide space selects one of four groups
 *	(1-9,0 / A-J / K-T / U-Z - . space *).  '$' '/' '+' '%' have no wide
 *	bar and 3 wide spaces.  Verified against the supplied samples:
 *	  '*' nWnnWnWnn   'A' WnnnnWnnW   'F' nnWnWWnnn   'W' WWWnnnnnn
 *	  'X' nWnnWnnnW   'Y' WWnnWnnnn   'Z' nWWnWnnnn
 */

#include "code39.h"

LOCAL const char c39_chars[C39_NSYM + 1] =
	"0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*";

/* Bar patterns b1..b5 (b1 = bit 4) for "digit" 1,2,...,9,0 */
LOCAL const UB c39_bars[10] = {
	0x11, 0x09, 0x18, 0x05, 0x14, 0x0C, 0x03, 0x12, 0x0A, 0x06
};

/* Wide-space masks s1..s4 (s1 = bit 3) */
#define SP_DIGITS	0x4U	/* 1..9, 0         */
#define SP_A_J		0x2U
#define SP_K_T		0x1U
#define SP_U_STAR	0x8U	/* U..Z - . space * */

LOCAL const UB c39_special_spaces[4] = { 0xE, 0xD, 0xB, 0x7 };	/* $ / + % */

/* Interleave 5 bar bits and 4 space bits: b1 s1 b2 s2 b3 s3 b4 s4 b5 */
LOCAL UH interleave(UW bars, UW spaces)
{
	UH	p = 0;
	UINT	i;

	for (i = 0; i < 5U; i++) {
		p = (UH)((p << 1) | ((bars >> (4U - i)) & 1U));
		if (i < 4U) {
			p = (UH)((p << 1) | ((spaces >> (3U - i)) & 1U));
		}
	}
	return p;
}

EXPORT UH c39_pattern(UINT index)
{
	if (index <= 9U) {			/* '0'..'9' */
		return interleave(c39_bars[(index + 9U) % 10U], SP_DIGITS);
	}
	if (index <= 19U) {			/* 'A'..'J' */
		return interleave(c39_bars[index - 10U], SP_A_J);
	}
	if (index <= 29U) {			/* 'K'..'T' */
		return interleave(c39_bars[index - 20U], SP_K_T);
	}
	if (index <= 38U) {			/* 'U'..'Z' '-' '.' ' ' */
		return interleave(c39_bars[index - 30U], SP_U_STAR);
	}
	if (index <= 42U) {			/* '$' '/' '+' '%' */
		return interleave(0U, c39_special_spaces[index - 39U]);
	}
	if (index == C39_STAR) {		/* '*' uses the "0" bar pattern */
		return interleave(c39_bars[9], SP_U_STAR);
	}
	return 0x1FF;
}

EXPORT char c39_char(UINT index)
{
	return (index < C39_NSYM) ? c39_chars[index] : '?';
}

EXPORT INT c39_index_of_char(char c)
{
	INT	i;

	for (i = 0; i < C39_NSYM; i++) {
		if (c39_chars[i] == c) {
			return i;
		}
	}
	return -1;
}

EXPORT INT c39_lookup(UH pattern)
{
	INT	i;

	for (i = 0; i < C39_NSYM; i++) {
		if (c39_pattern((UINT)i) == pattern) {
			return i;
		}
	}
	return -1;
}

/* a >= ratio * b, with the ratio specified in tenths. */
LOCAL BOOL ratio_ok(UW a, UW b, UW ratio_x10)
{
	return ((UD)a * 10U >= (UD)b * ratio_x10) ? TRUE : FALSE;
}

/* Sort indices of v[] by descending width (n <= 5, insertion sort) */
LOCAL void order_desc(const UW *v, UINT n, UB *idx)
{
	UINT	i, j;

	for (i = 0; i < n; i++) {
		idx[i] = (UB)i;
	}
	for (i = 1; i < n; i++) {
		UB	t = idx[i];

		for (j = i; j > 0U && v[idx[j - 1U]] < v[t]; j--) {
			idx[j] = idx[j - 1U];
		}
		idx[j] = t;
	}
}

EXPORT INT c39_classify(const UW w[C39_ELEMENTS])
{
	UW	bar[5], sp[4];
	UB	bo[5], so[4];
	UW	wide_bars = 0, wide_sp = 0;
	UINT	i;

	for (i = 0; i < 5U; i++) {
		bar[i] = w[2U * i];
	}
	for (i = 0; i < 4U; i++) {
		sp[i] = w[2U * i + 1U];
	}
	order_desc(bar, 5U, bo);
	order_desc(sp, 4U, so);

	/* Bars: 2 wide (2nd widest clearly wider than 3rd) or none wide */
	if (ratio_ok(bar[bo[1]], bar[bo[2]], C39_MIN_BAR_RATIO_X10)) {
		wide_bars = (1UL << (4U - bo[0])) | (1UL << (4U - bo[1]));
	} else if (!ratio_ok(bar[bo[0]], bar[bo[4]], C39_MIN_RATIO_X10)) {
		wide_bars = 0;
	} else {
		return C39_ERR_WIDTH;
	}

	/* Spaces: 1 wide, or 3 wide (only valid with no wide bar) */
	if (ratio_ok(sp[so[0]], sp[so[1]], C39_MIN_RATIO_X10)) {
		wide_sp = 1UL << (3U - so[0]);
	} else if (ratio_ok(sp[so[2]], sp[so[3]], C39_MIN_RATIO_X10)) {
		wide_sp = (1UL << (3U - so[0])) | (1UL << (3U - so[1]))
			| (1UL << (3U - so[2]));
	} else {
		return C39_ERR_WIDTH;
	}
	return (INT)interleave(wide_bars, wide_sp);
}

EXPORT UW c39_gap_limit(const UW s[C39_ELEMENTS], UH pattern)
{
	UW narrow[4];
	UB order[4];
	UINT i, count = 0;
	UD limit;

	for (i = 1U; i < C39_ELEMENTS; i += 2U) {
		if ((pattern & (1U << (C39_ELEMENTS - 1U - i))) == 0U) {
			narrow[count++] = s[i];
		}
	}
	if (count == 0U) {
		return 0U;
	}
	/* Standard symbols have three narrow spaces, special symbols one. */
	order_desc(narrow, count, order);
	limit = (UD)narrow[order[count / 2U]] * 25U / 10U;
	return limit > 0xFFFFFFFFUL ? 0xFFFFFFFFUL : (UW)limit;
}

LOCAL INT decode_dir(const UW *el, UINT n, BOOL rev, UB *data, UINT max_data)
{
	UW	sym[C39_ELEMENTS];
	UINT	k = (n + 1U) / C39_STRIDE;	/* symbols incl. start/stop */
	UINT	s, i, nd = 0;

	for (s = 0; s < k; s++) {
		INT	pat, idx;

		for (i = 0; i < C39_ELEMENTS; i++) {
			UINT	pos = s * C39_STRIDE + i;

			sym[i] = rev ? el[n - 1U - pos] : el[pos];
		}
		pat = c39_classify(sym);
		if (pat < 0) {
			return pat;
		}
		idx = c39_lookup((UH)pat);
		if (idx < 0) {
			return C39_ERR_PATTERN;
		}
		if ((s == 0U || s == k - 1U) != (idx == C39_STAR)) {
			return C39_ERR_STARTSTOP;
		}
		if (s + 1U < k) {
			/* inter-character gap must be narrow: < 2.5 narrow units */
			UINT	gpos = s * C39_STRIDE + C39_ELEMENTS;
			UW	gap = rev ? el[n - 1U - gpos] : el[gpos];

			if (gap > c39_gap_limit(sym, (UH)pat)) {
				return C39_ERR_GAP;
			}
		}
		if (idx != C39_STAR) {
			if (nd >= max_data) {
				return C39_ERR_SPACE;
			}
			data[nd++] = (UB)idx;
		}
	}
	return (INT)nd;
}

/* Recover one missed transition pair in *<letter>*. A 3:1 template has
   47 narrow units, including both gaps. Merging a run preserves its
   total duration, so the complete capture sets the timing scale. */
LOCAL INT decode_merged_letter(const UW *el, UB *data, UINT max_data, BOOL *reversed)
{
	UD measured[C39_MERGED_ELEMENTS];
	UD total = 0;
	INT match = -1;
	BOOL match_reversed = FALSE;
	UINT i, symbol, direction, merge;

	for (i = 0; i < C39_MERGED_ELEMENTS; i++) {
		if (el[i] == 0U) {
			return C39_ERR_WIDTH;
		}
		total += el[i];
		measured[i] = (UD)el[i] * 47U * 5U;
	}
	for (symbol = 10U; symbol < 36U; symbol++) {
		UB widths[C39_SINGLE_ELEMENTS];
		UINT group, k;

		for (group = 0; group < 3U; group++) {
			UH pattern = c39_pattern(group == 1U ? symbol : C39_STAR);

			for (k = 0; k < C39_ELEMENTS; k++) {
				widths[group * C39_STRIDE + k] =
					(pattern & (1U << (C39_ELEMENTS - 1U - k))) ? 3U : 1U;
			}
		}
		widths[9] = widths[19] = 1U;

		for (direction = 0; direction < 2U; direction++) {
			for (merge = 0; merge < C39_MERGED_ELEMENTS; merge++) {
				UINT pos = 0;

				for (i = 0; i < C39_MERGED_ELEMENTS; i++) {
					UINT units = widths[pos++];
					UINT observed = direction ? C39_MERGED_ELEMENTS - 1U - i : i;
					UD expected;

					if (i == merge) {
						units += widths[pos] + widths[pos + 1U];
						pos += 2U;
					}
					/* Each run must fit the template within 20%. */
					expected = total * units;
					if (measured[observed] < expected * 4U
					    || measured[observed] > expected * 6U) {
						break;
					}
				}
				if (i == C39_MERGED_ELEMENTS) {
					if (match >= 0 && match != (INT)symbol) {
						return C39_ERR_WIDTH;
					}
					if (match < 0) {
						match = (INT)symbol;
						match_reversed = direction != 0U;
					}
					break;
				}
			}
		}
	}
	if (match < 0) {
		return C39_ERR_WIDTH;
	}
	if (max_data == 0U) {
		return C39_ERR_SPACE;
	}
	data[0] = (UB)match;
	*reversed = match_reversed;
	return 1;
}

EXPORT INT c39_decode(const UW *el, UINT n, UB *data, UINT max_data,
		      BOOL *reversed)
{
	INT	r, rr;
	UW first_rev[C39_ELEMENTS];
	UINT i;

	*reversed = FALSE;
	if (n == C39_MERGED_ELEMENTS) {
		return decode_merged_letter(el, data, max_data, reversed);
	}
	if (n < 3U * C39_STRIDE - 1U || (n + 1U) % C39_STRIDE != 0U) {
		return C39_ERR_COUNT;
	}
	r = decode_dir(el, n, FALSE, data, max_data);
	if (r >= 0) {
		return r;
	}
	/* A '*' read backwards is not '*', so a forward decode of a reversed
	   run fails at the first symbol; then try the other direction. */
	rr = decode_dir(el, n, TRUE, data, max_data);
	if (rr >= 0) {
		*reversed = TRUE;
		return rr;
	}
	/* Infer orientation from the first observed delimiter, even when
	   the final delimiter is damaged. Report that direction's error. */
	for (i = 0; i < C39_ELEMENTS; i++) {
		first_rev[i] = el[C39_ELEMENTS - 1U - i];
	}
	if (c39_classify(first_rev) == (INT)c39_pattern(C39_STAR)) {
		*reversed = TRUE;
		return rr;
	}
	return r;
}
