/*
 *	bitpack.c
 *	LSB-first bit writer/reader and CRC-16/CCITT-FALSE (see bitpack.h).
 */

#include "bitpack.h"

EXPORT void bw_init(BIT_WRITER *w, UB *buf, UINT cap)
{
	UINT	i;

	w->buf = buf;
	w->cap = cap;
	w->bitpos = 0;
	w->overflow = FALSE;
	for (i = 0; i < cap; i++) {
		buf[i] = 0;
	}
}

EXPORT void bw_put(BIT_WRITER *w, UW value, UINT nbits)
{
	UINT	i;

	if (nbits == 0U || nbits > 32U || w->bitpos + nbits > w->cap * 8U) {
		w->overflow = TRUE;
		return;
	}
	for (i = 0; i < nbits; i++) {
		if (((value >> i) & 1UL) != 0U) {
			w->buf[w->bitpos >> 3] |= (UB)(1U << (w->bitpos & 7U));
		}
		w->bitpos++;
	}
}

EXPORT void bw_put_signed(BIT_WRITER *w, W value, UINT nbits)
{
	bw_put(w, (UW)value, nbits);		/* low nbits of two's complement */
}

EXPORT UINT bw_bytes(const BIT_WRITER *w)
{
	return (w->bitpos + 7U) / 8U;
}

EXPORT void br_init(BIT_READER *r, const UB *buf, UINT len)
{
	r->buf = buf;
	r->len = len;
	r->bitpos = 0;
	r->underflow = FALSE;
}

EXPORT UW br_get(BIT_READER *r, UINT nbits)
{
	UW	v = 0;
	UINT	i;

	if (nbits == 0U || nbits > 32U || r->bitpos + nbits > r->len * 8U) {
		r->underflow = TRUE;
		return 0;
	}
	for (i = 0; i < nbits; i++) {
		if ((r->buf[r->bitpos >> 3] & (1U << (r->bitpos & 7U))) != 0U) {
			v |= (1UL << i);
		}
		r->bitpos++;
	}
	return v;
}

EXPORT W br_get_signed(BIT_READER *r, UINT nbits)
{
	UW	v = br_get(r, nbits);
	UW	sign;

	if (nbits == 0U || nbits >= 32U) {
		return (W)v;
	}
	sign = 1UL << (nbits - 1U);
	return (W)((v ^ sign) - sign);
}

EXPORT UH crc16_ccitt(const UB *data, UINT len)
{
	UH	crc = 0xFFFFU;
	UINT	i, b;

	for (i = 0; i < len; i++) {
		crc = (UH)(crc ^ ((UH)data[i] << 8));
		for (b = 0; b < 8U; b++) {
			UW	shifted = (UW)crc << 1;

			crc = (UH)(((crc & 0x8000U) != 0U) ? (shifted ^ 0x1021U) : shifted);
		}
	}
	return crc;
}
