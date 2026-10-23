/*
 *	Host-test shim for <tk/typedef.h>.
 *
 *	The real header maps W/UW to 'long', which is 64-bit on a 64-bit host.
 *	The pure-logic modules depend on the TARGET widths (32-bit W/UW), so
 *	host tests use exact-width types instead.  Used only by the host test
 *	makefiles under app_program/subsystemN/host_test/ (never by the
 *	firmware build, which does not compile files below one folder level).
 */

#ifndef __TK_TYPEDEF_H__
#define __TK_TYPEDEF_H__

#include <stdint.h>
#include <stddef.h>

typedef int8_t		B;
typedef int16_t		H;
typedef int32_t		W;
typedef int64_t		D;
typedef uint8_t		UB;
typedef uint16_t	UH;
typedef uint32_t	UW;
typedef uint64_t	UD;
typedef volatile B	_B;
typedef volatile H	_H;
typedef volatile W	_W;
typedef volatile D	_D;
typedef volatile UB	_UB;
typedef volatile UH	_UH;
typedef volatile UW	_UW;
typedef volatile UD	_UD;

typedef int		INT;
typedef unsigned int	UINT;
typedef W		SZ;
typedef INT		ID;
typedef W		MSEC;
typedef void		(*FP)();
typedef INT		(*FUNCP)();

#define LOCAL		static
#define EXPORT
#define IMPORT		extern
#define CONST		const

typedef UINT		BOOL;
#define TRUE		1
#define FALSE		0

typedef INT		ER;
typedef INT		PRI;
typedef W		TMO;
typedef UW		RELTIM;
typedef UW		ATR;

#define TMO_POL		0
#define TMO_FEVR	(-1)

#define E_OK		(0)
#define E_PAR		(-17)
#define E_LIMIT		(-34)
#define E_OBJ		(-41)
#define E_QOVR		(-43)
#define E_TMOUT		(-50)
#define E_IO		(-57)

#endif /* __TK_TYPEDEF_H__ */
