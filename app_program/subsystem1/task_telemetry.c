/*
 *	task_telemetry.c
 *	Telemetry Collection task (Week 6: Low, periodic 500 ms).
 *
 *	Reads every subsystem's packed status through its own interface
 *	(never their internals), builds one TLM_RECORD (tlm_build.c), encodes
 *	the binary frame (mqtt_proto.c) and queues it for the Communication
 *	task.  It also derives event frames from changes (mode, faults,
 *	barcode, hump, obstacle outcome, collision) so short-lived events are
 *	not lost between telemetry samples.  Queue overflow drops the OLDEST
 *	frame and is counted (Tribunal 1: capacity and overflow policy
 *	defined).
 */

#include "subsystem1.h"
#include "tlm_build.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"

EXPORT volatile UW	s1_tlm_dropped;
EXPORT TASK_STAT	s1_stat_tlm;

LOCAL TLM_SOURCES	src;		/* used by this task only */
LOCAL TLM_RECORD	cur;
LOCAL TLM_RECORD	prev;
LOCAL BOOL		have_prev;
LOCAL UB		prev_avoid_count;
LOCAL UB		tlm_seq;
LOCAL UB		evt_seq;
LOCAL BOOL		boot_report_sent;

EXPORT void tlm_cyclic_handler(void *exinf)
{
	(void)exinf;
	(void)tk_wup_tsk(tid_tlm);
}

LOCAL void collect(TLM_SOURCES *s)
{
	s->uptime_ms = app_now_ms();
	s->vsw = sys_vsw();
	s->faults = sys_fault_get();
	motion_get_status(&s->motion);
	s3_get_line_status(&s->line);
	s3_get_barcode_status(&s->barcode);
	s4_get_imu_status(&s->imu);
	s4_get_hump_status(&s->hump);
	s5_get_scan_status(&s->scan);
	s5_get_avoid_status(&s->avoid);
	s5_get_profile(&s->profile);
	s1_get_comm_status(&s->comm);
}

LOCAL void queue_frame(UB kind, const UB *frame, INT len)
{
	UB	msg[S1_MSG_MAX];
	INT	i;
	UW	before = app_load(&s1_tlm_dropped);

	if (len <= 0 || len > S1_MQTT_FRAME_MAX) {
		return;
	}
	msg[0] = kind;
	for (i = 0; i < len; i++) {
		msg[i + 1] = frame[i];
	}
	if (s1_queue_put_drop_oldest(mbf_telemetry, msg, len + 1, &s1_tlm_dropped) != E_OK
	    || app_load(&s1_tlm_dropped) != before) {
		sys_fault_set(SYS_FLT_TLM_OVERFLOW);	/* degraded: a frame was lost */
	}
	(void)tk_set_flg(flg_comm, COMM_EV_TICK);
}

LOCAL void send_event(const EVT_RECORD *e)
{
	UB	f[MP_MAX_FRAME];

	queue_frame(MT_EVENT, f, mp_evt_encode(e, evt_seq++, f, sizeof(f)));
}

#if S1_CONSOLE_TELEMETRY
LOCAL void print_hex(const UB *f, INT n)
{
	INT	i;

	tm_printf((UB*)"tlm:");
	for (i = 0; i < n; i++) {
		tm_printf((UB*)" %02x", (INT)f[i]);
	}
	tm_printf((UB*)"\n");
}
#endif

EXPORT void telemetry_task(INT stacd, void *exinf)
{
	EVT_RECORD	ev[TLM_MAX_EVENTS];
	UB		f[MP_MAX_FRAME];

	(void)stacd;
	(void)exinf;
	for (;;) {
		UW	start;
		UINT	i, ne;
		INT	n;

		(void)tk_slp_tsk(TMO_FEVR);
		(void)tk_can_wup(TSK_SELF);
		(void)app_atomic_xchg(&s1_status_now, 0U);	/* served by this run */
		start = task_stat_begin(&s1_stat_tlm);

		collect(&src);
		tlm_build(&cur, &src);
		n = mp_tlm_encode(&cur, tlm_seq++, f, sizeof(f));
		if (n > 0) {
			queue_frame(MT_TELEMETRY, f, n);
#if S1_CONSOLE_TELEMETRY
			if (tlm_seq % S1_CONSOLE_TELEMETRY == 0U) {
				print_hex(f, n);
			}
#endif
		}
		if (have_prev) {
			ne = tlm_events(&prev, prev_avoid_count, &cur, &src, ev);
			for (i = 0; i < ne; i++) {
				send_event(&ev[i]);
			}
		}
		/* Events raised before the first connection are dropped from the
		   full queue, and a fault that clears within a 500 ms sample is
		   never seen at all.  Once online, report every fault since boot
		   so a Safe Stop at power-on still shows its cause. */
		if (!boot_report_sent && src.comm.state == CS_ONLINE) {
			EVT_RECORD	e;

			e.code = EVT_FAULTS;
			e.data = sys_fault_seen() & SYS_FLT_ALL;
			send_event(&e);
			tm_printf((UB*)"tlm: faults since boot 0x%07x\n", (INT)e.data);
			boot_report_sent = TRUE;
		}
		prev = cur;
		prev_avoid_count = src.avoid.count;
		have_prev = TRUE;
		task_stat_end(&s1_stat_tlm, start, 50000U);
	}
}
