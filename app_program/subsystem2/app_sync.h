/*
 *	app_sync.h
 *	Safe access to application data shared between tasks, or between a
 *	task and an interrupt handler, on either core.
 *
 *	Owned by subsystem 2, used by every subsystem.  Change Requirement 1:
 *	read-modify-write of packed shared data must not race, so
 *	  - words with several writers use app_atomic_or/clear/xchg();
 *	  - words with ONE writer use app_store()/app_load() (an aligned
 *	    32-bit access is indivisible; the barrier orders it across cores);
 *	  - multi-word records with one writer use an APP_SEQ sequence count;
 *	  - anything else uses an APP_LOCK for a few instructions.
 *
 *	Why a wrapper: the kernel's SMP API (tk/smp.h: ISpinLock, atomic_*)
 *	is only compiled when TK_SUPPORT_SMP is 1.  On the single-core build
 *	(make SMP=0) masking interrupts is enough because nothing else runs.
 *	Never call a kernel function or block while holding an APP_LOCK.
 */

#ifndef APP_SYNC_H
#define APP_SYNC_H

#include <tk/tkernel.h>
#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif

/* Full memory barrier (Cortex-M0+ DMB) */
#define APP_DMB()	Asm("dmb" ::: "memory")

/* ---------------------------------------------------------------------
 *  Short critical section
 * --------------------------------------------------------------------- */
typedef struct {
#if TK_SUPPORT_SMP
	T_SPLOCK	word;		/* tk/smp.h software spinlock */
#else
	UW		unused;		/* single core: interrupt mask only */
#endif
} APP_LOCK;

Inline void app_lock_init(APP_LOCK *lk)
{
#if TK_SUPPORT_SMP
	(void)InitSpinLock(&lk->word);
#else
	lk->unused = 0;
#endif
}

/* Masks interrupts on this core and, with SMP, excludes the other core.
   Returns the state for app_lock_exit().  Usable from handlers. */
Inline UINT app_lock_enter(APP_LOCK *lk)
{
	UINT	st;
#if TK_SUPPORT_SMP
	(void)ISpinLock(&lk->word, &st);
#else
	(void)lk;
	DI(st);
#endif
	return st;
}

Inline void app_lock_exit(APP_LOCK *lk, UINT st)
{
#if TK_SUPPORT_SMP
	(void)ISpinUnlock(&lk->word, st);
#else
	(void)lk;
	EI(st);
#endif
}

/* ---------------------------------------------------------------------
 *  Atomic read-modify-write of a shared 32-bit word (returns old value)
 * --------------------------------------------------------------------- */
Inline UW app_atomic_or(volatile UW *p, UW bits)
{
#if TK_SUPPORT_SMP
	return atomic_bitset((UW *)p, bits);
#else
	UINT	st;
	UW	old;

	DI(st);
	old = *p;
	*p = old | bits;
	EI(st);
	return old;
#endif
}

Inline UW app_atomic_clear(volatile UW *p, UW bits)
{
#if TK_SUPPORT_SMP
	return atomic_bitclr((UW *)p, bits);
#else
	UINT	st;
	UW	old;

	DI(st);
	old = *p;
	*p = old & ~bits;
	EI(st);
	return old;
#endif
}

Inline UW app_atomic_xchg(volatile UW *p, UW val)
{
#if TK_SUPPORT_SMP
	return atomic_xchg((UW *)p, val);
#else
	UINT	st;
	UW	old;

	DI(st);
	old = *p;
	*p = val;
	EI(st);
	return old;
#endif
}

/* ---------------------------------------------------------------------
 *  Single-writer word publication
 * --------------------------------------------------------------------- */
Inline void app_store(volatile UW *p, UW v)
{
	APP_DMB();
	*p = v;
	APP_DMB();
}

Inline UW app_load(const volatile UW *p)
{
	UW	v = *p;

	APP_DMB();
	return v;
}

/* ---------------------------------------------------------------------
 *  Sequence count for a multi-word record with ONE writer (task or ISR)
 *  and any number of readers.  Odd = write in progress.  Readers retry a
 *  bounded number of times and report failure instead of spinning.
 * --------------------------------------------------------------------- */
typedef struct {
	volatile UW	seq;
} APP_SEQ;

#define APP_SEQ_READ_TRIES	8

Inline void app_seq_write_begin(APP_SEQ *s)
{
	s->seq = s->seq + 1U;		/* only the single writer touches it */
	APP_DMB();
}

Inline void app_seq_write_end(APP_SEQ *s)
{
	APP_DMB();
	s->seq = s->seq + 1U;
}

Inline UW app_seq_read_begin(const APP_SEQ *s)
{
	UW	v = s->seq;

	APP_DMB();
	return v;
}

/* TRUE if the copy taken since app_seq_read_begin() is torn */
Inline BOOL app_seq_read_retry(const APP_SEQ *s, UW start)
{
	APP_DMB();
	return ((start & 1U) != 0U) || (s->seq != start);
}

#endif /* APP_SYNC_H */
