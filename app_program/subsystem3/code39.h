/*
 *	code39.h
 *	Code 39 symbol decoding (Buddy 3).  Pure logic, host-testable.
 *
 *	Evidence (subsystem3/docs/BARCODE_ANALYSIS.md): every barcode supplied
 *	with the brief -- Barcode Sample.pdf ("A", "Z"), BARCODE.pptx ("A",
 *	"F", "Z", "W", "X", "Y") and the photo in Barcode & Line
 *	Specification.pdf -- measures as 3 symbols of 9 elements (5 bars, 4
 *	spaces) with exactly 3 wide elements, wide:narrow = 3:1, a narrow
 *	inter-character gap, and decodes as '*' + one character + '*'.  That
 *	is the Code 39 structure; the table below is generated from the
 *	Code 39 rule (bar pattern x wide-space group) rather than typed in,
 *	and the unit tests check it against the measured sample widths.
 *
 *	Symbol index (6 bits, 0..43): '0'..'9' = 0..9, 'A'..'Z' = 10..35,
 *	'-' 36, '.' 37, ' ' 38, '$' 39, '/' 40, '+' 41, '%' 42, '*' 43.
 *	C39_NONE (63) = no symbol.
 *
 *	Pattern: 9 bits, element 1 (first bar) = bit 8 ... element 9 = bit 0,
 *	1 = wide.
 */

#ifndef CODE39_H
#define CODE39_H

#include <tk/typedef.h>

#define C39_NSYM		44
#define C39_STAR		43
#define C39_NONE		63
#define C39_ELEMENTS		9	/* per symbol: b s b s b s b s b */
#define C39_STRIDE		10	/* symbol + inter-character gap  */

/* Decoder errors (negative) */
#define C39_ERR_COUNT		(-1)	/* element count is not 10k-1, k >= 3 */
#define C39_ERR_WIDTH		(-2)	/* wide/narrow ambiguous              */
#define C39_ERR_PATTERN		(-3)	/* not a Code 39 pattern              */
#define C39_ERR_STARTSTOP	(-4)	/* no '*' at both ends / '*' inside    */
#define C39_ERR_GAP		(-5)	/* inter-character gap too wide       */
#define C39_ERR_SPACE		(-6)	/* output buffer too small            */

/* Minimum wide/narrow ratio accepted, x10 (nominal print ratio is 3:1) */
#define C39_MIN_RATIO_X10	15

IMPORT UH   c39_pattern(UINT index);		/* 0x1FF on bad index */
IMPORT char c39_char(UINT index);		/* '?' on bad index   */
IMPORT INT  c39_index_of_char(char c);		/* -1 if not Code 39  */
IMPORT INT  c39_lookup(UH pattern);		/* index or -1        */

/* Classify 9 element widths (any linear unit) into a 9-bit pattern.
   Bars are compared with bars and spaces with spaces, so a constant
   black/white bias (ink spread, IR spot blur, threshold offset) cancels.
   Returns the pattern, or C39_ERR_WIDTH. */
IMPORT INT  c39_classify(const UW w[C39_ELEMENTS]);

/* Decode a whole element run (bar, space, bar, ... bar).  Tries the
   forward order, then the reversed order (car driving the other way).
   data[] receives the data symbols between the '*' delimiters.
   Returns the number of data symbols (>= 1) or a C39_ERR_* code. */
IMPORT INT  c39_decode(const UW *el, UINT n, UB *data, UINT max_data,
		       BOOL *reversed);

#endif /* CODE39_H */
