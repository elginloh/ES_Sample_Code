/*
 *	task_hump.c
 *	Hump & Terrain Analysis task (Week 6: Medium, event-driven by the
 *	bounded queue mbf_imu_hump).  It never touches the IMU or the encoder
 *	hardware and never affects motion (Tribunal 1 interface map).
 *
 *	A gap in the sample sequence (queue overflow) or an invalid sample
 *	discards the hump in progress rather than measuring across missing
 *	data (Tribunal 1: "consumers ignore invalid or stale samples").
 */

#include "subsystem4.h"

EXPORT volatile UW	s4_hump_word = (UW)HUMP_CODE_NONE | ((UW)HUMP_CODE_NONE << 6);
EXPORT volatile UW	s4_max_hump = (UW)-1;

LOCAL HUMP	hump;

EXPORT void s4_get_hump_status(S4_HUMP_STATUS *s)
{
	s4_hump_unpack(app_load(&s4_hump_word), s);
}

EXPORT W s4_max_hump_um(void)
{
	return (W)app_load(&s4_max_hump);
}

LOCAL void publish(void)
{
	S4_HUMP_STATUS	s;

	hump_status(&hump, &s);
	app_store(&s4_hump_word, s4_hump_pack(&s));
	app_store(&s4_max_hump, (UW)hump.max_height_um);
}

EXPORT void hump_task(INT stacd, void *exinf)
{
	S4_HUMP_SAMPLE	s;
	HUMP_EVT	ev;
	UW		w;
	UB		expect = 0;
	BOOL		first = TRUE;
	BOOL		was_active = FALSE;

	(void)stacd;
	(void)exinf;
	hump_init(&hump);
	publish();

	for (;;) {
		INT	n = tk_rcv_mbf(mbf_imu_hump, &w, TMO_FEVR);
		BOOL	valid;

		if (n != (INT)sizeof(w)) {
			continue;
		}
		s4_sample_unpack(w, &s);
		valid = (s.valid != 0U) ? TRUE : FALSE;
		if (!first && s.seq != expect) {
			valid = FALSE;			/* samples were dropped */
		}
		first = FALSE;
		expect = (UB)((s.seq + 1U) & 0xFU);

		hump_step(&hump, (W)s.pitch_ddeg * 10, (W)s.ds_100um * 100, valid, &ev);
		if (ev.completed) {
			tm_printf((UB*)"hump: height %d.%d mm, length %d mm, highest %d.%d mm\n",
				  (INT)(ev.height_um / 1000), (INT)((ev.height_um % 1000) / 100),
				  (INT)ev.length_mm,
				  (INT)(hump.max_height_um / 1000),
				  (INT)((hump.max_height_um % 1000) / 100));
		}
		if (ev.completed || ev.rejected || hump_active(&hump) != was_active) {
			was_active = hump_active(&hump);
			publish();
		}
	}
}
