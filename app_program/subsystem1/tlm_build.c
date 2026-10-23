/*
 *	tlm_build.c
 *	Telemetry record and events from the subsystem snapshots (see
 *	tlm_build.h).  Every value keeps its source's scale except the three
 *	obstacle-profile fields, which are compacted here (field table:
 *	mqtt_proto.h).
 */

#include "tlm_build.h"

LOCAL W clampw(W v, W lo, W hi)
{
	return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* mm -> steps of 'step' mm, rounded; 254 = at least 254 steps */
LOCAL UB steps8(W mm, W step)
{
	if (mm < 0) {
		return TLM_UNKNOWN8;
	}
	return (UB)clampw((mm + step / 2) / step, 0, 254);
}

EXPORT void tlm_build(TLM_RECORD *r, const TLM_SOURCES *s)
{
	/* system (Buddy 2) */
	r->uptime_ds = (UH)(s->uptime_ms / 100U);
	r->mode = (UB)vsw_mode(s->vsw);
	r->epoch = (UB)vsw_epoch(s->vsw);
	r->faults = s->faults & SYS_FLT_ALL;
	/* motion (Buddy 2) */
	r->mstate = s->motion.mstate;
	r->msource = s->motion.source;
	r->cmd_active = s->motion.cmd_active;
	r->speed_l = s->motion.speed_l_mms;
	r->speed_r = s->motion.speed_r_mms;
	r->odometer_cm = s->motion.odometer_cm;
	r->heading2 = (UB)(s->motion.heading_deg / 2U);
	r->cruise10 = (UB)(s->motion.cruise_mms / 10U);
	/* line + barcode (Buddy 3) */
	r->ir_black = s->line.ir_black;
	r->line_pos = s->line.pos;
	r->line_state = s->line.state;
	r->steer = s->line.steer;
	r->bc_state = s->barcode.state;
	r->bc_symbol = s->barcode.symbol;
	r->bc_result = s->barcode.result;
	r->nav = s->barcode.nav;
	r->nav_valid = s->barcode.nav_valid;
	r->bc_valid = s->barcode.n_valid;
	r->bc_invalid = s->barcode.n_invalid;
	/* IMU + hump (Buddy 4) */
	r->imu_status = s->imu.status;
	r->pitch = s->imu.pitch_deg;
	r->roll = s->imu.roll_deg;
	r->yaw_rate = s->imu.yaw_rate_dps;
	r->heading = s->imu.heading_deg;
	r->motion_class = s->imu.motion_class;
	r->tilt = s->imu.tilt;
	r->collision = s->imu.collision;
	r->shock = s->imu.shock_half_g;
	r->hump_last = s->hump.last_code;
	r->hump_max = s->hump.max_code;
	r->hump_count = s->hump.count;
	r->hump_active = s->hump.active;
	/* obstacle (Buddy 5) */
	r->us_dist = s->scan.dist_code;
	r->us_result = s->scan.us_result;
	r->avoid_state = s->avoid.state;
	r->avoid_side = s->avoid.side;
	r->avoid_outcome = s->avoid.outcome;
	r->obst_width = steps8(s->profile.width_mm, 5);
	r->obst_bearing = (B)clampw(s->profile.bearing_deg, -127, 127);
	r->clear_left = steps8(s->profile.clear_left_mm, 10);
	r->clear_right = steps8(s->profile.clear_right_mm, 10);
	/* communication (Buddy 1) */
	r->comm_state = s->comm.state;
	r->tlm_dropped = s->comm.tlm_dropped;
	r->rx_rejected = s->comm.rx_rejected;
}

LOCAL void add(EVT_RECORD *ev, UINT *n, UB code, UW data)
{
	if (*n < TLM_MAX_EVENTS) {
		ev[*n].code = code;
		ev[*n].data = data;
		(*n)++;
	}
}

EXPORT UINT tlm_events(const TLM_RECORD *prev, UB prev_avoid_count,
		       const TLM_RECORD *cur, const TLM_SOURCES *s,
		       EVT_RECORD ev[TLM_MAX_EVENTS])
{
	UINT	n = 0;

	if (cur->mode != prev->mode) {
		add(ev, &n, EVT_MODE, (UW)prev->mode | ((UW)cur->mode << 3)
				      | ((UW)cur->epoch << 6));
	}
	if (cur->faults != prev->faults) {
		add(ev, &n, EVT_FAULTS, cur->faults);
	}
	if (cur->bc_valid != prev->bc_valid || cur->bc_invalid != prev->bc_invalid) {
		add(ev, &n, EVT_BARCODE, (UW)cur->bc_symbol | ((UW)cur->bc_result << 6)
					 | ((UW)cur->nav << 8) | ((UW)cur->nav_valid << 10));
	}
	if (cur->hump_count != prev->hump_count) {
		add(ev, &n, EVT_HUMP, (UW)cur->hump_last | ((UW)cur->hump_max << 6)
				      | ((UW)cur->hump_count << 12));
	}
	if (s->avoid.count != prev_avoid_count) {
		UH	d = (s->profile.dist_mm < 0) ? (UH)S5_DIST_INVALID
			  : s5_dist_code((UW)s->profile.dist_mm);

		add(ev, &n, EVT_OBSTACLE, (UW)s->avoid.outcome | ((UW)s->avoid.fail << 2)
					  | ((UW)s->avoid.side << 5)
					  | ((UW)cur->obst_width << 7) | ((UW)d << 15));
	}
	if (cur->collision != 0U && prev->collision == 0U) {
		add(ev, &n, EVT_COLLISION, cur->shock);
	}
	return n;
}
