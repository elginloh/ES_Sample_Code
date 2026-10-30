/*
 * protocol.js
 * The backend's view of the CSC14 binary protocol.  The wire layout, the
 * enumerations and the fault policy come from the ONE JavaScript codec,
 * app_program/subsystem1/docs/nodered_decoder.js (which mirrors
 * mqtt_proto.c); this file only maps decoded frames onto database columns.
 */
'use strict';

const path = require('path');
const { REPO } = require('./config');

const codec = require(path.join(REPO, 'app_program', 'subsystem1', 'docs', 'nodered_decoder.js'));

const E = codec.ENUMS;
const code = (list, label) => {
	const i = list.indexOf(label);
	return (i < 0) ? null : i;
};

/* Typed telemetry columns (units in the names, NULL = not available).
   Enumerations are stored as their firmware codes; csc14.enum_label (seeded
   from codec.ENUMS by the migration tool) names them. */
function telemetryColumns(d) {
	const r = d.raw;
	const ob = d.obstacle;

	return {
		mode: r.mode,
		epoch: r.epoch,
		fault_mask: r.faults,
		critical: d.critical,
		motion_state: r.mstate,
		motion_source: r.msource,
		cmd_active: d.motion.cmd_active,
		speed_l_mms: d.motion.speed_l_mms,
		speed_r_mms: d.motion.speed_r_mms,
		odometer_cm: r.odometer_cm,
		heading_enc_cw_deg: d.motion.heading_enc_cw_deg,
		cruise_set_mms: d.motion.cruise_set_mms,
		ir_raw: r.ir_black,
		ir_left: d.line.ir.left === 1,
		ir_centre: d.line.ir.centre === 1,
		ir_right: d.line.ir.right === 1,
		line_pos: r.line_pos,
		line_pos_phys: d.line.pos_phys_code,
		line_state: r.line_state,
		steer: r.steer,
		bc_state: r.bc_state,
		bc_symbol: d.barcode.symbol,
		bc_result: r.bc_result,
		nav: d.barcode.nav_valid ? r.nav : null,
		nav_valid: d.barcode.nav_valid,
		bc_valid_mod64: r.bc_valid,
		bc_invalid_mod64: r.bc_invalid,
		imu_status: r.imu_status,
		pitch_deg: d.imu.pitch_deg,
		roll_deg: d.imu.roll_deg,
		yaw_rate_ccw_dps: d.imu.yaw_rate_ccw_dps,
		heading_imu_ccw_deg: d.imu.heading_ccw_deg,
		motion_class: r.motion_class,
		tilt: d.imu.tilt,
		collision: d.imu.collision,
		shock_g: d.imu.shock_g,
		hump_last_mm: d.hump.last_mm,
		hump_max_boot_mm: d.hump.max_boot_mm,
		hump_count: r.hump_count,
		hump_active: d.hump.active,
		us_dist_mm: ob.distance_mm,
		us_no_echo: ob.no_echo,
		us_result: r.us_result,
		us_context: ob.us_context,
		us_forward_mm: ob.forward_mm,
		avoid_state: r.avoid_state,
		avoid_side: r.avoid_side,
		avoid_outcome: r.avoid_outcome,
		profile_reported: ob.profile === 'reported',
		obst_width_mm: ob.width_mm,
		obst_bearing_deg: ob.bearing_deg,
		clear_left_mm: ob.clear_left_mm,
		clear_left_at_least: ob.clear_left_at_least,
		clear_right_mm: ob.clear_right_mm,
		clear_right_at_least: ob.clear_right_at_least,
		comm_state: r.comm_state,
		tlm_dropped: r.tlm_dropped,
		rx_rejected: r.rx_rejected
	};
}

/* Rows for csc14.enum_label and csc14.fault_bit (seeded, never typed in
   SQL by hand) */
function enumLabelRows() {
	const rows = [];
	for (const [enumName, list] of Object.entries(E)) {
		list.forEach((label, i) => {
			if (label !== null) { rows.push([enumName, i, label]); }
		});
	}
	return rows;
}

function faultBitRows(policy) {
	return codec.FAULT_TABLE.map(([faultName, owner], bit) =>
		[bit, faultName, owner, codec.faultSeverity(bit, policy)]);
}

module.exports = { codec, E, code, telemetryColumns, enumLabelRows, faultBitRows };
