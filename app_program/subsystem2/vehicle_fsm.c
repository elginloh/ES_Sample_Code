/*
 *	vehicle_fsm.c
 *	Vehicle operating-mode state machine (pure logic).
 */

#include "vehicle_fsm.h"
#include "motion_cfg.h"

EXPORT void vfsm_init(VFSM *f)
{
	f->mode = VM_INIT;
	f->epoch = 0;
	f->ir_ok = FALSE;
	f->imu_ok = FALSE;
	f->line_lost = FALSE;
	f->in_mode_ms = 0;
}

LOCAL void enter(VFSM *f, UB mode, VFSM_OUT *out)
{
	if (mode == VM_INIT || mode == VM_OBSTACLE || mode == VM_SAFE_STOP) {
		f->epoch = (UB)((f->epoch + 1U) & 0x7U);
	}
	if (mode == VM_INIT) {
		f->ir_ok = FALSE;		/* calibrate again for this epoch */
		f->imu_ok = FALSE;
	}
	if (mode == VM_INIT || mode == VM_NORMAL) {
		f->line_lost = FALSE;
	}
	f->mode = mode;
	f->in_mode_ms = 0;
	out->changed = TRUE;
}

EXPORT void vfsm_step(VFSM *f, const VFSM_IN *in, VFSM_OUT *out)
{
	UW	ev = in->events;

	out->changed = FALSE;
	out->old_mode = f->mode;
	out->flt_set = 0;
	f->in_mode_ms += in->dt_ms;

	if ((ev & SYS_EV_LINE_LOST) != 0UL) {
		f->line_lost = TRUE;
	}
	if ((ev & SYS_EV_LINE_FOUND) != 0UL) {
		f->line_lost = FALSE;
	}

	/* Safe Stop has the highest priority, from every other mode */
	if (f->mode != VM_SAFE_STOP && (in->faults & in->crit_mask) != 0UL) {
		enter(f, VM_SAFE_STOP, out);
		return;
	}

	switch (f->mode) {
	case VM_INIT:
		if ((ev & SYS_EV_IR_CAL_OK) != 0UL) {
			f->ir_ok = TRUE;
		}
		if ((ev & SYS_EV_IMU_CAL_OK) != 0UL) {
			f->imu_ok = TRUE;
		}
		if (f->ir_ok && f->imu_ok && in->armed) {
			enter(f, VM_NORMAL, out);
		} else if (in->armed && f->in_mode_ms > INIT_TIMEOUT_MS) {
			out->flt_set |= SYS_FLT_INIT;
			enter(f, VM_SAFE_STOP, out);
		}
		break;

	case VM_NORMAL:
		if ((ev & SYS_EV_OBSTACLE) != 0UL) {
			enter(f, VM_OBSTACLE, out);
		} else if (in->nav_started && !in->nav_finished) {
			enter(f, VM_NAV_ACTION, out);
		}
		break;

	case VM_NAV_ACTION:
		if ((ev & SYS_EV_OBSTACLE) != 0UL) {
			enter(f, VM_OBSTACLE, out);	/* avoidance > barcode */
		} else if (in->nav_finished) {
			enter(f, VM_NORMAL, out);
		}
		break;

	case VM_OBSTACLE:
		if ((ev & SYS_EV_RECOVERY_FAILED) != 0UL) {
			enter(f, VM_SAFE_STOP, out);
		} else if ((ev & SYS_EV_AVOID_DONE) != 0UL) {
			enter(f, VM_NORMAL, out);
		}
		break;

	case VM_SAFE_STOP:
		if (in->resume) {
			enter(f, VM_INIT, out);
		}
		break;

	default:			/* corrupted mode: fail safe */
		enter(f, VM_SAFE_STOP, out);
		break;
	}
}
