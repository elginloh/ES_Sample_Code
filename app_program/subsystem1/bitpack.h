/*
 *	bitpack.h
 *	Portable bit-stream packing for the binary MQTT protocol (Buddy 1).
 *	Pure logic, host-testable.
 *
 *	Change Requirement 1: "compact binary fields rather than ... text";
 *	Tribunal 1: "use portable masks and shifts ... rather than relying on
 *	compiler-dependent C bit-field layout".  Bit order is fixed and
 *	documented: fields are written LSB-first; stream bit k is bit (k % 8)
 *	of byte (k / 8).  Signed fields are two's complement of the stated
 *	width.  CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF): check value
 *	0x29B1 for the ASCII string "123456789".
 */

#ifndef BITPACK_H
#define BITPACK_H

#include <tk/typedef.h>

typedef struct {
	UB	*buf;
	UINT	cap;		/* bytes */
	UINT	bitpos;
	BOOL	overflow;	/* a write did not fit */
} BIT_WRITER;

typedef struct {
	const UB *buf;
	UINT	len;		/* bytes */
	UINT	bitpos;
	BOOL	underflow;	/* a read ran past the end */
} BIT_READER;

IMPORT void bw_init(BIT_WRITER *w, UB *buf, UINT cap);
IMPORT void bw_put(BIT_WRITER *w, UW value, UINT nbits);	/* nbits 1..32 */
IMPORT void bw_put_signed(BIT_WRITER *w, W value, UINT nbits);
IMPORT UINT bw_bytes(const BIT_WRITER *w);

IMPORT void br_init(BIT_READER *r, const UB *buf, UINT len);
IMPORT UW   br_get(BIT_READER *r, UINT nbits);
IMPORT W    br_get_signed(BIT_READER *r, UINT nbits);

IMPORT UH   crc16_ccitt(const UB *data, UINT len);

#endif /* BITPACK_H */
