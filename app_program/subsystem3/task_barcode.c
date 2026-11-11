/*
 *	task_barcode.c
 *	Barcode Decoding task (Week 6: Medium, event-driven).  Implements the
 *	"Barcode detected -> Decode barcode -> Execute command" part of the
 *	Buddy 3 statechart (6.3).
 *
 *	Woken by the IR_RIGHT edge ISR (tk_wup_tsk), and every S3_BC_POLL_MS
 *	while a capture is open so that completion (trailing quiet zone) and
 *	the speed-adaptive window are detected without an edge.  It does not
 *	read the sensor itself and never touches the motors (Tribunal 1
 *	interface map): an invalid barcode produces no command, a valid one
 *	is sent to Buddy 2 as a BARCODE-source command and the task waits for
 *	the done/aborted/fault result.
 */

#include "subsystem3.h"
#include "drv_ir.h"

#if S3_BARCODE_DIAG
#define BC_DIAG_IDLE_POLL_MS	250U
#define BC_DIAG_HEARTBEAT_US	2000000UL
#endif

EXPORT volatile UW	s3_bc_word = (UW)C39_NONE << 2;

LOCAL BC_DEC		dec;
LOCAL S3_BC_STATUS	bst;
LOCAL BOOL		pending_valid;
LOCAL UB		pending_nav;
LOCAL UB		pending_junctions;
LOCAL UB		capture_junctions;

EXPORT void s3_get_barcode_status(S3_BC_STATUS *s)
{
	s3_bc_unpack(app_load(&s3_bc_word), s);
}

LOCAL void publish(void)
{
	app_store(&s3_bc_word, s3_bc_pack(&bst));
}

LOCAL UB junction_count(void)
{
	S3_LINE_STATUS	ls;

	s3_get_line_status(&ls);
	return ls.junctions;
}

#if S3_BARCODE_DIAG
/* Print the merged sensor durations once per completed capture.
   Units are microseconds. Each Code 39 symbol has nine elements followed by
   one narrow inter-character gap, except the final symbol. */
LOCAL void print_widths(const BC_RESULT *res)
{
	UINT	i = 0, symbol = 0;

	if (res->n_diag == 0U) {
		return;
	}
	tm_printf((UB*)"barcode: durations us (%d elements)\n",
		  (INT)res->n_diag);
	while (i < res->n_diag) {
		UINT	j, count = res->n_diag - i;
		UW sym[C39_ELEMENTS];

		if (count > C39_ELEMENTS) {
			count = C39_ELEMENTS;
		}
		tm_printf((UB*)"  s%d:", (INT)symbol);
		for (j = 0; j < count; j++) {
			UW	w = res->element_um[i + j];
			sym[j] = w;

			tm_printf((UB*)" %d", (INT)w);
		}
		if (count == C39_ELEMENTS) {
			INT forward = c39_classify(sym), backward;
			UW reversed[C39_ELEMENTS];
			for (j = 0; j < C39_ELEMENTS; j++) {
				reversed[j] = sym[C39_ELEMENTS - 1U - j];
			}
			backward = c39_classify(reversed);
			tm_printf((UB*)" | symbol forward=%c reverse=%c",
				  forward < 0 ? '?' : c39_char(c39_lookup((UH)forward)),
				  backward < 0 ? '?' : c39_char(c39_lookup((UH)backward)));
		}
		i += count;
		if (count == C39_ELEMENTS && i < res->n_diag) {
			UW	gap = res->element_um[i++];
			INT pattern;

			if (res->reversed && i + C39_ELEMENTS <= res->n_diag) {
				for (j = 0; j < C39_ELEMENTS; j++) {
					sym[j] = res->element_um[i + C39_ELEMENTS - 1U - j];
				}
			}
			pattern = c39_classify(sym);
			tm_printf((UB*)" | gap %d", (INT)gap);
			if (pattern >= 0) {
				tm_printf((UB*)" (max %d)", (INT)c39_gap_limit(sym, (UH)pattern));
			}
		}
		tm_printf((UB*)"\n");
		symbol++;
	}
}
#endif

/* Statechart "Execute command": send to Motion Control, wait for result */
LOCAL void execute(UB nav)
{
	UW	type, cmd, res = 0;
	INT	arg;
	ER	er;

	nav_to_motion(nav, &type, &arg);
	tm_printf((UB*)"barcode: junction command %s, arg %d, waiting for motion completion\n",
		  nav_name(nav), arg);
	bst.state = BCS_EXECUTING;
	bst.executed = 1;
	publish();
	er = motion_command(type, MSRC_BARCODE, arg, S3_NAV_SPEED_LEVEL, &cmd);
	if (er == E_OK) {
		er = motion_wait_result(cmd, S3_NAV_WAIT_MS, &res);
	}
	if (er == E_OK) {
		bst.mres = (UB)mres_status(res);
	} else {
		bst.mres = MRES_FAULT;		/* not queued / no answer */
		motion_stop(MSRC_BARCODE);
	}
	tm_printf((UB*)"barcode: %s -> motion result %d\n", nav_name(nav),
		  (INT)bst.mres);
	if (nav == NAV_LEFT || nav == NAV_RIGHT) {
		if (bst.mres == MRES_DONE) {
			tm_printf((UB*)"barcode: %s junction turn complete\n", nav_name(nav));
		} else {
			tm_printf((UB*)"barcode: %s junction turn incomplete\n", nav_name(nav));
		}
	}
	/* edges seen while manoeuvring belong to no barcode */
	ir_edge_flush();
	bc_init(&dec);
	bst.state = BCS_IDLE;
	publish();
}

LOCAL void handle(const BC_RESULT *res)
{
	UB	nav;

	if (res->result == BCR_NONE) {
		return;
	}
#if S3_BARCODE_DIAG
	print_widths(res);
#endif
	bst.result = res->result;
	bst.reversed = res->reversed ? 1U : 0U;
	bst.err_class = s3_err_class(res->err);

	if (res->result == BCR_INVALID) {
		bst.n_invalid = (UB)((bst.n_invalid + 1U) & 0x3FU);
		bst.nav_valid = 0;
		bst.state = BCS_IDLE;
		publish();
		/* "invalid barcode -> Error/recover: discard"; never a command */
		sys_fault_set(SYS_FLT_BARCODE_INVALID);
		tm_printf((UB*)"barcode: invalid (err %d, %d elements, direction %s)\n",
			  res->err, (INT)res->n_elements,
			  res->reversed ? "reverse" : "forward/unknown");
		return;
	}

	bst.symbol = res->data[0];
	sys_fault_clear(SYS_FLT_BARCODE_INVALID);
	if (res->result == BCR_DUPLICATE) {
		publish();
		return;				/* duplicate-reading prevention */
	}
	bst.n_valid = (UB)((bst.n_valid + 1U) & 0x3FU);
	tm_printf((UB*)"barcode: '%c'%s%s\n", c39_char(res->data[0]),
		  res->reversed ? " (read reversed)" : "",
		  res->n_elements == C39_MERGED_ELEMENTS ? " (recovered 27-element scan)" : "");

	if (sys_mode() != VM_NORMAL || !nav_map_lookup(res->data[0], &nav)) {
		bst.nav_valid = 0;		/* no meaning assigned (nav_map.h) */
		bst.state = BCS_IDLE;
		publish();
		return;
	}
	bst.nav = nav;
	bst.nav_valid = 1;
	if (S3_NAV_AT_JUNCTION) {
		pending_valid = TRUE;
		pending_nav = nav;
		pending_junctions = capture_junctions;
		bst.state = BCS_DECODING;
		publish();
		tm_printf((UB*)"barcode: queued %s; following line until GP6 + GP2 junction\n",
			  nav_name(nav));
	} else {
		execute(nav);
	}
}

LOCAL void check_pending(UW mode)
{
	S3_LINE_STATUS line;

	if (!pending_valid) {
		return;
	}
	s3_get_line_status(&line);
	if (mode == VM_INIT || mode == VM_SAFE_STOP) {
		pending_valid = FALSE;
		bst.state = BCS_IDLE;
		bst.nav_valid = 0;
		bc_init(&dec);
		publish();
		tm_printf((UB*)"barcode: pending turn cleared on init or safe stop\n");
	} else if (mode != VM_NORMAL) {
		pending_junctions = line.junctions;
	} else if (line.junctions != pending_junctions) {
		pending_junctions = line.junctions;
		if ((UW)(app_now_ms() - line.junction_time_ms) > S3_JUNCTION_HANDOFF_MS) {
			tm_printf((UB*)"barcode: junction passed before handoff; %s still queued\n",
				  nav_name(pending_nav));
			return;
		}
		pending_valid = FALSE;
		tm_printf((UB*)"barcode: confirmed junction; executing %s\n", nav_name(pending_nav));
		execute(pending_nav);
	}
}

EXPORT void barcode_task(INT stacd, void *exinf)
{
	BC_RESULT	res;
	UW		rec;
	W		odo = 0, spd = 0;
#if S3_BARCODE_DIAG
	UW		diag_next_us = 0;
	UW		diag_edges = 0;
	UW		diag_last_edge_us = 0;
#endif

	(void)stacd;
	(void)exinf;
	bc_init(&dec);
	nav_map_reset();
	bst.state = BCS_IDLE;
	bst.symbol = C39_NONE;
	bst.result = BCR_NONE;
	bst.nav = 0;
	bst.nav_valid = 0;
	bst.reversed = 0;
	bst.n_valid = 0;
	bst.n_invalid = 0;
	bst.mres = 0;
	bst.executed = 0;
	bst.err_class = S3_BCE_NONE;
	publish();
	if (ir_capture_start() < E_OK) {
		sys_fault_set(SYS_FLT_TASK_COMM);
	}
	ir_edge_flush();
#if S3_BARCODE_DIAG
#if S3_BARCODE_ANALOG
	tm_printf((UB*)"barcode: GP26 ADC0, threshold=%d +/- %d, black-high=%d\n",
		  (INT)S3_BARCODE_ADC_THRESHOLD, (INT)S3_BARCODE_ADC_HYSTERESIS,
		  (INT)S3_BARCODE_ADC_BLACK_HIGH);
#else
	tm_printf((UB*)"barcode: debug enabled (GP26 digital input, 1 ms sampling, 3 ms stable filter)\n");
#endif
#endif

	for (;;) {
		UW	mode;
		UB	black_level;
		TMO	wait;

		wait = (bc_capturing(&dec) || pending_valid)
			 ? (TMO)S3_BC_POLL_MS : TMO_FEVR;
#if S3_BARCODE_DIAG
		if (wait == TMO_FEVR) {
			wait = (TMO)BC_DIAG_IDLE_POLL_MS;
		}
#endif
		(void)tk_slp_tsk(wait);
		mode = sys_mode();
		motion_get_odometry(&odo, &spd);
		black_level = (UB)app_load(&s3_black_level);
		/* Keep the decoded command intact until its junction. Later
		   barcode/line edges must not overwrite it or issue another turn. */
		if (pending_valid) {
			ir_edge_flush();
			(void)ir_edge_overflow_take();
			check_pending(mode);
			continue;
		}
#if S3_BARCODE_DIAG
		{
			UW now = hw_time_us();

			if (diag_next_us == 0U || (W)(now - diag_next_us) >= 0) {
				UB raw = (UB)((ir_read_raw() >> 2) & 1U);

#if S3_BARCODE_ANALOG
				tm_printf((UB*)"barcode: GP26 adc=%d black=%d mode=%d edges=%d\n",
					  (INT)ir_barcode_adc(), (INT)(raw == black_level),
					  (INT)mode, (INT)diag_edges);
#else
				tm_printf((UB*)"barcode: debug alive GP26 raw=%d black=%d mode=%d speed=%d mm/s edges=%d\n",
					  (INT)raw, (INT)(raw == black_level),
					  (INT)mode, (INT)spd, (INT)diag_edges);
#endif
				diag_next_us = now + BC_DIAG_HEARTBEAT_US;
			}
		}
#endif

		if (ir_edge_overflow_take()) {
			sys_fault_set(SYS_FLT_BARCODE_OVERFLOW);
			bc_abort(&dec, &res);
			handle(&res);
			ir_edge_flush();
			continue;
		}

		while (ir_edge_get(&rec)) {
			UB	black = (IR_EDGE_LEVEL(rec) == black_level) ? 1U : 0U;
			BOOL was_capturing = bc_capturing(&dec);

#if S3_BARCODE_DIAG
			diag_edges++;
			tm_printf((UB*)"barcode: edge %d raw=%d black=%d dt=%d us\n",
				  (INT)diag_edges, (INT)IR_EDGE_LEVEL(rec), (INT)black,
				  (INT)(diag_last_edge_us ? IR_EDGE_TIME(rec) - diag_last_edge_us : 0U));
			diag_last_edge_us = IR_EDGE_TIME(rec);
#endif
			bc_edge(&dec, IR_EDGE_TIME(rec), black, spd, odo, &res);
			if (!was_capturing && bc_capturing(&dec)) {
				capture_junctions = junction_count();
			}
			handle(&res);
			if (pending_valid) {
				break;
			}
		}
		bc_poll(&dec, hw_time_us(), spd, odo, &res);
		handle(&res);
		check_pending(mode);

		if (bc_capturing(&dec) != (bst.state == BCS_CAPTURING)
		    && bst.state != BCS_DECODING) {
			if (bc_capturing(&dec)) {
#if S3_BARCODE_DIAG
				tm_printf((UB*)"barcode: capture started (speed %d mm/s)\n",
					  (INT)spd);
#endif
			}
			bst.state = bc_capturing(&dec) ? BCS_CAPTURING : BCS_IDLE;
			publish();
		}
	}
}
