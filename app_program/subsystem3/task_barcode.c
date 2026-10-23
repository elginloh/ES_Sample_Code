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

EXPORT volatile UW	s3_bc_word = (UW)C39_NONE << 2;

LOCAL BC_DEC		dec;
LOCAL S3_BC_STATUS	bst;
LOCAL BOOL		pending_valid;
LOCAL UB		pending_nav;
LOCAL W			pending_odo;
LOCAL UB		pending_junctions;

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

/* Statechart "Execute command": send to Motion Control, wait for result */
LOCAL void execute(UB nav)
{
	UW	type, cmd, res = 0;
	INT	arg;
	ER	er;

	nav_to_motion(nav, &type, &arg);
	bst.state = BCS_EXECUTING;
	bst.executed = 1;
	publish();
	er = motion_command(type, MSRC_BARCODE, arg, 0, &cmd);
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
	/* edges seen while manoeuvring belong to no barcode */
	ir_edge_flush();
	bst.state = BCS_IDLE;
	publish();
}

LOCAL void handle(const BC_RESULT *res, W odo)
{
	UB	nav;

	if (res->result == BCR_NONE) {
		return;
	}
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
		tm_printf((UB*)"barcode: invalid (err %d, %d elements)\n",
			  res->err, (INT)res->n_elements);
		return;
	}

	bst.symbol = res->data[0];
	sys_fault_clear(SYS_FLT_BARCODE_INVALID);
	if (res->result == BCR_DUPLICATE) {
		publish();
		return;				/* duplicate-reading prevention */
	}
	bst.n_valid = (UB)((bst.n_valid + 1U) & 0x3FU);
	tm_printf((UB*)"barcode: '%c'%s\n", c39_char(res->data[0]),
		  res->reversed ? " (read reversed)" : "");

	if (!nav_map_lookup(res->data[0], &nav)) {
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
		pending_odo = odo;
		pending_junctions = junction_count();
		bst.state = BCS_DECODING;
		publish();
	} else {
		execute(nav);
	}
}

LOCAL void check_pending(UW mode, W odo)
{
	if (!pending_valid) {
		return;
	}
	if (mode != VM_NORMAL || odo - pending_odo > S3_NAV_PENDING_MM) {
		pending_valid = FALSE;		/* no junction in time: discard */
		bst.state = BCS_IDLE;
		publish();
	} else if (junction_count() != pending_junctions) {
		pending_valid = FALSE;
		execute(pending_nav);
	}
}

EXPORT void barcode_task(INT stacd, void *exinf)
{
	BC_RESULT	res;
	UW		rec;
	W		odo = 0, spd = 0;
	UW		prev_mode = VM_INIT;

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

	for (;;) {
		UW	mode;
		UB	black_level;

		(void)tk_slp_tsk((bc_capturing(&dec) || pending_valid)
				 ? (TMO)S3_BC_POLL_MS : TMO_FEVR);
		mode = sys_mode();
		motion_get_odometry(&odo, &spd);
		black_level = (UB)app_load(&s3_black_level);

		if (ir_edge_overflow_take()) {
			sys_fault_set(SYS_FLT_BARCODE_OVERFLOW);
			bc_abort(&dec, &res);
			handle(&res, odo);
			ir_edge_flush();
			continue;
		}
		/* Barcodes are read only in Normal Navigation */
		if (mode != VM_NORMAL) {
			if (prev_mode == VM_NORMAL) {
				bc_abort(&dec, &res);
				handle(&res, odo);
			}
			ir_edge_flush();
			prev_mode = mode;
			check_pending(mode, odo);
			continue;
		}
		prev_mode = mode;

		while (ir_edge_get(&rec)) {
			UB	black = (IR_EDGE_LEVEL(rec) == black_level) ? 1U : 0U;

			bc_edge(&dec, IR_EDGE_TIME(rec), black, spd, odo, &res);
			handle(&res, odo);
		}
		bc_poll(&dec, hw_time_us(), spd, odo, &res);
		handle(&res, odo);
		check_pending(mode, odo);

		if (bc_capturing(&dec) != (bst.state == BCS_CAPTURING)
		    && bst.state != BCS_DECODING) {
			bst.state = bc_capturing(&dec) ? BCS_CAPTURING : BCS_IDLE;
			publish();
		}
	}
}
