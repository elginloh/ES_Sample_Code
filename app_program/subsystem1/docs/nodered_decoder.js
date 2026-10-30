/*
 * nodered_decoder.js
 * CSC14 binary MQTT protocol v1 -- server-side codec (decoder, validator,
 * frame encoder).  Specification: MQTT_BINARY_PROTOCOL.md (same folder).
 * Mirrors app_program/subsystem1/mqtt_proto.c field for field: the field
 * tables below are the ONLY JavaScript copy of the wire layout, and the
 * telemetry backend (telemetry/lib) loads this file instead of keeping its
 * own.
 *
 * Three ways to use this one file:
 *   1. Node.js: const csc14 = require('./nodered_decoder.js');
 *      csc14.decode(buffer), csc14.encodeCommand('SET_SPEED', 250, id, seq),
 *      csc14.encodeFrame(csc14.TYPE.TELEMETRY, seq, rawFields)
 *   2. Node-RED: telemetry/nodered loads it through functionGlobalContext.
 *      Pasting the whole file into a Function node still works: valid
 *      frames leave with msg.payload = decoded object, invalid ones are
 *      dropped with a warning.
 *   3. Self test against the golden frames: node nodered_decoder.js
 *
 * The firmware is the reference: if this file and mqtt_proto.c ever
 * disagree, the golden frames in MQTT_BINARY_PROTOCOL.md decide.
 *
 * Interpretation that depends on how the firmware was BUILT (not on the
 * frame) is passed in opts and defaults to the firmware defaults:
 *   irSidesMirrored  drv_ir.h IR_SIDES_MIRRORED (1): IR_R/GP27 is on the
 *                    car's LEFT, IR_L/GP2 on its right
 *   policy           sys_state_logic.c SYS_COLLISION_STOPS,
 *                    SYS_IMU_REQUIRED, SYS_US_REQUIRED (all 1)
 */
'use strict';

const VERSION = 1;
const HDR_LEN = 3;
const CRC_LEN = 2;
const TYPE = { TELEMETRY: 1, HEARTBEAT: 2, EVENT: 3, COMMAND: 4, ACK: 5 };
const TYPE_NAMES = { 1: 'telemetry', 2: 'heartbeat', 3: 'event', 4: 'command', 5: 'ack' };
const PAYLOAD_LEN = { 1: 35, 2: 4, 3: 5, 4: 3, 5: 3 };

/* ------------------------------------------------------------------ */
/*  Enumerations (names in code order, from the firmware headers)      */

const ENUMS = {
	mode: ['INIT', 'NORMAL', 'NAV_ACTION', 'OBSTACLE', 'SAFE_STOP'],	/* sys_defs.h VM_* */
	motion_state: ['IDLE', 'STRAIGHT', 'TURNING', 'FAULT'],			/* motion_msg.h MS_* */
	motion_source: ['LINE', 'BARCODE', 'AVOID', 'SUPER'],			/* MSRC_* */
	/* line_follow.h LP_*, in the line follower's OWN labels (IR_L / IR_R) */
	line_pos: ['CENTER', 'LEFT_NEAR', 'LEFT_FAR', 'RIGHT', 'LOST_LEFT', 'LOST_RIGHT',
		'JUNCTION', 'RIGHT_NEAR'],
	/* the same positions named after the car's real sides (see linePosPhys) */
	line_pos_phys: ['CENTRE', 'LEFT_NEAR', 'LEFT_FAR', 'RIGHT_NEAR', 'RIGHT_FAR',
		'LOST_LEFT', 'LOST_RIGHT', 'JUNCTION'],
	line_state: ['ON_LINE', 'SEARCHING', 'LOST', 'UNKNOWN'],		/* LS_* */
	bc_state: ['IDLE', 'CAPTURING', 'DECODING', 'EXECUTING'],		/* BCS_* */
	bc_result: ['NONE', 'VALID', 'INVALID', 'DUPLICATE'],			/* BCR_* */
	nav: ['LEFT', 'RIGHT', 'STRAIGHT', 'U_TURN'],				/* NAV_* */
	imu_status: ['INIT', 'CALIBRATING', 'OK', 'FAULT'],			/* IMU_ST_* */
	motion_class: ['STATIONARY', 'MOVING', 'TURN_LEFT', 'TURN_RIGHT'],	/* MCLS_* */
	us_result: ['OK', 'NO_ECHO', 'NO_RESPONSE', 'INVALID'],			/* US_* */
	avoid_state: ['IDLE', 'STOPPING', 'COARSE', 'FINE', 'PLANNING', 'EXECUTING',
		'SEARCHING', 'ALIGNING', 'FAILED'],				/* AVST_* */
	avoid_side: ['NONE', 'LEFT', 'RIGHT'],					/* AV_SIDE_* */
	avoid_outcome: ['NONE', 'SUCCESS', 'FAILED', 'CLEARED'],		/* AVO_* */
	avoid_fail: ['NONE', 'PROFILE', 'CLEARANCE', 'MOTION', 'LINE', 'TIMEOUT', 'SENSOR'], /* AVF_* */
	comm_state: ['INIT', 'CONNECTING_WIFI', 'CONNECTING_MQTT', 'SUBSCRIBING',
		'ONLINE', 'RETRY_WAIT', 'RECOVERY', 'DISABLED'],		/* comm_fsm.h CS_* */
	event: [null, 'MODE', 'FAULTS', 'BARCODE', 'HUMP', 'OBSTACLE', 'COLLISION'], /* EVT_* */
	command: ['PING', 'STOP', 'RESUME', 'CLEAR', 'SET_SPEED', 'REQ_STATUS'],	/* CMD_* */
	ack_result: ['OK', 'BAD_VERSION', 'BAD_TYPE', 'BAD_LENGTH', 'BAD_FIELD', 'BAD_ARG',
		'REJ_STATE', 'DUPLICATE', 'RATE', 'BUSY'],			/* ACK_* */
	reject_reason: ['NONE', 'FORMAT', 'RANGE', 'SOURCE', 'MODE', 'STALE', 'PRIORITY', 'QUEUE'] /* MREJ_* */
};
const CODE39 = '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*';
const SYMBOL_NONE = 63;

/* ------------------------------------------------------------------ */
/*  Fault bitmask (sys_defs.h) and severity policy (sys_state_logic.c) */

const SEV = { DEGRADED: 'DEGRADED', CRIT_COND: 'CRITICAL_CONDITION', CRIT_EVENT: 'CRITICAL_EVENT' };
/* [name, owner, severity] in bit order; 'IMU', 'IMU_CAL', 'COLLISION' and
   'US' rows follow the team decision flags, everything else is fixed */
const FAULT_TABLE = [
	['MOTOR_STALL', 'B2', SEV.DEGRADED], ['ENCODER', 'B2', SEV.DEGRADED],
	['MOTION_ESCALATED', 'B2', SEV.CRIT_EVENT], ['MOTOR_WDOG', 'B2', SEV.CRIT_EVENT],
	['CMD_QUEUE', 'B2', SEV.DEGRADED], ['IR_CAL', 'B3', SEV.CRIT_EVENT],
	['LINE_LOST', 'B3', SEV.DEGRADED], ['LINE_RECOVERY', 'B3', SEV.DEGRADED],
	['BARCODE_INVALID', 'B3', SEV.DEGRADED], ['BARCODE_OVERFLOW', 'B3', SEV.DEGRADED],
	['IMU', 'B4', 'IMU'], ['IMU_CAL', 'B4', 'IMU_CAL'],
	['COLLISION', 'B4', 'COLLISION'], ['US_NO_RESPONSE', 'B5', 'US'],
	['US_INVALID', 'B5', SEV.DEGRADED], ['SERVO_SUSPECT', 'B5', SEV.DEGRADED],
	['OBSTACLE_UNAVOIDABLE', 'B5', SEV.CRIT_EVENT], ['PROFILE_INVALID', 'B5', SEV.CRIT_EVENT],
	['AVOID_FAILED', 'B5', SEV.CRIT_EVENT], ['RECOVERY_TIMEOUT', 'B5', SEV.CRIT_EVENT],
	['WIFI_LOSS', 'B1', SEV.DEGRADED], ['MQTT_LOSS', 'B1', SEV.DEGRADED],
	['CMD_INVALID', 'B1', SEV.DEGRADED], ['TLM_OVERFLOW', 'B1', SEV.DEGRADED],
	['INIT', 'B2', SEV.CRIT_EVENT], ['TASK_COMM', 'any', SEV.CRIT_EVENT],
	['REMOTE_STOP', 'B1->B2', SEV.CRIT_EVENT]
];
const FAULTS = FAULT_TABLE.map((f) => f[0]);
const FAULT_MASK_ALL = 0x7FFFFFF;
const DEFAULT_POLICY = { collisionStops: true, imuRequired: true, usRequired: true };

function faultSeverity(bit, policy) {
	const p = Object.assign({}, DEFAULT_POLICY, policy || {});
	const row = FAULT_TABLE[bit];

	if (!row) {
		return SEV.DEGRADED;
	}
	switch (row[2]) {
	case 'IMU': return p.imuRequired ? SEV.CRIT_COND : SEV.DEGRADED;
	case 'IMU_CAL': return p.imuRequired ? SEV.CRIT_EVENT : SEV.DEGRADED;
	case 'COLLISION': return p.collisionStops ? SEV.CRIT_EVENT : SEV.DEGRADED;
	case 'US': return p.usRequired ? SEV.CRIT_COND : SEV.DEGRADED;
	default: return row[2];
	}
}

function criticalMask(policy) {
	let mask = 0;
	for (let i = 0; i < FAULTS.length; i++) {
		if (faultSeverity(i, policy) !== SEV.DEGRADED) { mask += 2 ** i; }
	}
	return mask;
}

function faultList(mask) {
	const names = [];
	for (let i = 0; i < FAULTS.length; i++) {
		if (Math.floor(mask / 2 ** i) % 2) { names.push(FAULTS[i]); }
	}
	return names;
}

/* ------------------------------------------------------------------ */
/*  Schemas, wire order: [name, bits, signed].  Keep in step with      */
/*  TLM_FIELDS (mqtt_proto.c) and MQTT_BINARY_PROTOCOL.md section 5.   */

const TLM_FIELDS = [
	['uptime_ds', 16, 0], ['mode', 3, 0], ['epoch', 3, 0], ['faults', 27, 0],
	['mstate', 2, 0], ['msource', 2, 0], ['cmd_active', 1, 0], ['speed_l', 11, 1],
	['speed_r', 11, 1], ['odometer_cm', 16, 0], ['heading2', 8, 0], ['cruise10', 6, 0],
	['ir_black', 3, 0], ['line_pos', 3, 0], ['line_state', 2, 0], ['steer', 7, 1],
	['bc_state', 2, 0], ['bc_symbol', 6, 0], ['bc_result', 2, 0], ['nav', 2, 0],
	['nav_valid', 1, 0], ['bc_valid', 6, 0], ['bc_invalid', 6, 0], ['imu_status', 2, 0],
	['pitch', 8, 1], ['roll', 8, 1], ['yaw_rate', 9, 1], ['heading', 9, 0],
	['motion_class', 2, 0], ['tilt', 1, 0], ['collision', 1, 0], ['shock', 5, 0],
	['hump_last', 6, 0], ['hump_max', 6, 0], ['hump_count', 4, 0], ['hump_active', 1, 0],
	['us_dist', 10, 0], ['us_result', 2, 0], ['avoid_state', 4, 0], ['avoid_side', 2, 0],
	['avoid_outcome', 2, 0], ['obst_width', 8, 0], ['obst_bearing', 8, 1], ['clear_left', 8, 0],
	['clear_right', 8, 0], ['comm_state', 3, 0], ['tlm_dropped', 8, 0], ['rx_rejected', 8, 0]
];
const HB_FIELDS = [['uptime_ds', 16, 0], ['mode', 3, 0], ['comm_state', 3, 0],
	['critical', 1, 0], ['fault_count', 5, 0], ['reconnects', 4, 0]];
const EVT_FIELDS = [['code', 4, 0], ['data', 32, 0]];
const CMD_FIELDS = [['cmd', 3, 0], ['arg', 10, 0], ['cmd_id', 8, 0], ['reserved', 3, 0]];
const ACK_FIELDS = [['cmd_id', 8, 0], ['cmd', 3, 0], ['result', 4, 0], ['detail', 5, 0]];
const FIELDS = { 1: TLM_FIELDS, 2: HB_FIELDS, 3: EVT_FIELDS, 4: CMD_FIELDS, 5: ACK_FIELDS };

/* Event data word layouts (LSB first), tlm_build.c tlm_events() */
const EVT_DATA = {
	1: [['old_mode', 3], ['new_mode', 3], ['epoch', 3]],
	2: [['fault_mask', 27]],
	3: [['symbol', 6], ['result', 2], ['nav', 2], ['nav_valid', 1]],
	4: [['last', 6], ['max', 6], ['count', 4]],
	5: [['outcome', 2], ['fail', 3], ['side', 2], ['width', 8], ['distance', 10]],
	6: [['shock', 5]]
};

/* Values a field can never take in a v1 frame.  Fields not listed use
   their whole bit range (all codes valid, some of them special). */
const angleOk = (v) => (v >= -90 && v <= 90) || v === -128;
const humpOk = (v) => v <= 50 || v === 63;
const distOk = (v) => v <= 800 || v === 1022 || v === 1023;
const TLM_RANGE = {
	mode: (v) => v <= 4,
	heading2: (v) => v <= 179,
	steer: (v) => v >= -63,
	bc_symbol: (v) => v <= 43 || v === SYMBOL_NONE,
	pitch: angleOk,
	roll: angleOk,
	heading: (v) => v <= 359 || v === 511,
	hump_last: humpOk,
	hump_max: humpOk,
	us_dist: distOk,
	avoid_state: (v) => v <= 8,
	avoid_side: (v) => v <= 2,
	obst_bearing: (v) => v >= -127
};
const HB_RANGE = { mode: (v) => v <= 4, fault_count: (v) => v <= 27 };
const EVT_RANGE = {
	1: { old_mode: (v) => v <= 4, new_mode: (v) => v <= 4 },
	3: { symbol: (v) => v <= 43 || v === SYMBOL_NONE },
	4: { last: humpOk, max: humpOk },
	5: { fail: (v) => v <= 6, side: (v) => v <= 2, distance: distOk }
};
const ACK_RANGE = { cmd: (v) => v <= 5, result: (v) => v <= 9 };

/* ------------------------------------------------------------------ */
/*  CRC and bit packing                                                */

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final XOR */
function crc16(buf, len) {
	let crc = 0xFFFF;
	for (let i = 0; i < len; i++) {
		crc ^= buf[i] << 8;
		for (let b = 0; b < 8; b++) {
			crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) & 0xFFFF : (crc << 1) & 0xFFFF;
		}
	}
	return crc;
}

/* LSB-first bit reader: stream bit k = bit (k % 8) of byte (k / 8) */
function bitReader(buf, start, len) {
	let pos = 0;
	return {
		get(n) {
			let v = 0;
			for (let i = 0; i < n; i++, pos++) {
				if (pos >= len * 8) { throw new Error('LENGTH'); }
				if (buf[start + (pos >> 3)] & (1 << (pos & 7))) { v += 2 ** i; }
			}
			return v;
		},
		getSigned(n) {
			const v = this.get(n);
			return (v >= 2 ** (n - 1)) ? v - 2 ** n : v;
		}
	};
}

function bitWriter(len) {
	const out = Buffer.alloc(len);
	let pos = 0;
	return {
		put(v, n) {
			for (let i = 0; i < n; i++, pos++) {
				if (Math.floor(v / 2 ** i) % 2) { out[pos >> 3] |= 1 << (pos & 7); }
			}
		},
		putSigned(v, n) {
			this.put((v < 0) ? v + 2 ** n : v, n);
		},
		buffer: out
	};
}

function readFields(buf, start, len, fields) {
	const rd = bitReader(buf, start, len);
	const raw = {};
	for (const [name, bits, signed] of fields) {
		raw[name] = signed ? rd.getSigned(bits) : rd.get(bits);
	}
	return raw;
}

/* Sub-fields of a 32-bit event data word */
function splitWord(word, layout) {
	const out = {};
	let off = 0;
	for (const [name, bits] of layout) {
		out[name] = Math.floor(word / 2 ** off) % 2 ** bits;
		off += bits;
	}
	return out;
}

function rangeIssues(raw, checks) {
	const issues = [];
	for (const name of Object.keys(checks || {})) {
		if (name in raw && !checks[name](raw[name])) {
			issues.push(name + '=' + raw[name]);
		}
	}
	return issues;
}

/* ------------------------------------------------------------------ */
/*  Frame check (firmware order: length, CRC, version, type, length)   */

function checkFrame(buf) {
	if (!Buffer.isBuffer(buf) || buf.length < HDR_LEN + CRC_LEN) { return { error: 'SHORT' }; }
	const crcRx = buf[buf.length - 2] | (buf[buf.length - 1] << 8);
	if (crc16(buf, buf.length - CRC_LEN) !== crcRx) { return { error: 'CRC' }; }
	const version = buf[0] >> 4;
	const type = buf[0] & 0x0F;
	const len = buf[2];
	if (version !== VERSION) { return { error: 'VERSION', version }; }
	if (!(type in PAYLOAD_LEN)) { return { error: 'TYPE', version, type }; }
	if (len !== buf.length - HDR_LEN - CRC_LEN || len !== PAYLOAD_LEN[type]) {
		return { error: 'LENGTH', version, type };
	}
	return { version, type, seq: buf[1], len };
}

/* ------------------------------------------------------------------ */
/*  Interpretation                                                     */

const name = (list, code) => (list[code] !== undefined ? list[code] : 'CODE_' + code);
const orNull = (v, invalid) => ((v === invalid) ? null : v);
const symbolOf = (code) => ((code < CODE39.length) ? CODE39[code] : null);
const humpMm = (code) => ((code === 63) ? null : code * 2);
const distMm = (code) => ((code <= 800) ? code * 5 : null);

/* Line position in the car's real sides.  The line follower only names
   its own sensors; with IR_SIDES_MIRRORED its IR_L (GP2) is on the car's
   right and IR_R (GP27) on the car's left. */
const POS_PHYS_MIRRORED = [0, 3, 4, 2, 6, 5, 7, 1];	/* line_pos -> line_pos_phys */
const POS_PHYS_DIRECT = [0, 1, 2, 4, 5, 6, 7, 3];
function linePosPhys(pos, mirrored) {
	return (mirrored ? POS_PHYS_MIRRORED : POS_PHYS_DIRECT)[pos];
}

/* What the ultrasonic distance means at this moment (task_scan.c,
   task_avoid.c): forward range while monitoring with the servo centred,
   one sample of a servo sweep (angle not transmitted) during COARSE/FINE,
   and the last sweep sample, held, while PLANNING (ranging is idle). */
function usContext(avoidState) {
	if (avoidState === 2 || avoidState === 3) { return 'scan_sweep'; }
	if (avoidState === 4) { return 'held'; }
	return 'forward';
}

/* The profile words start at zero (task_profile.c), which decodes as a
   0 mm wide obstacle straight ahead.  A real profile always has a width
   or a clearance; all four zero means "no profile since boot". */
function profileState(r) {
	return (r.obst_width === 0 && r.clear_left === 0 && r.clear_right === 0
		&& r.obst_bearing === 0) ? 'none_yet' : 'reported';
}

function interpretTelemetry(raw, opts) {
	const mirrored = opts.irSidesMirrored;
	const policy = opts.policy;
	const bit = (i) => (raw.ir_black >> i) & 1;
	const crit = criticalMask(policy);
	const critFaults = faultList(raw.faults).filter((n) => crit & 2 ** FAULTS.indexOf(n));
	const prof = profileState(raw);
	const ctx = usContext(raw.avoid_state);
	const profOk = prof === 'reported';
	const clear = (c) => ((c === 255 || !profOk) ? null : c * 10);

	return {
		uptime_s: raw.uptime_ds / 10,
		mode: name(ENUMS.mode, raw.mode),
		epoch: raw.epoch,
		fault_mask: raw.faults,
		faults: faultList(raw.faults),
		critical: critFaults.length > 0,
		critical_faults: critFaults,
		motion: {
			state: name(ENUMS.motion_state, raw.mstate),
			source: name(ENUMS.motion_source, raw.msource),
			cmd_active: raw.cmd_active === 1,
			speed_l_mms: orNull(raw.speed_l, -1024),
			speed_r_mms: orNull(raw.speed_r, -1024),
			odometer_cm: raw.odometer_cm,
			odometer_saturated: raw.odometer_cm === 65535,
			heading_enc_cw_deg: raw.heading2 * 2,		/* + = clockwise from start */
			cruise_set_mms: raw.cruise10 * 10		/* setting, 10 mm/s steps */
		},
		line: {
			ir: {						/* car's real sides, 1 = black */
				left: mirrored ? bit(2) : bit(0),
				centre: bit(1),
				right: mirrored ? bit(0) : bit(2)
			},
			ir_code: { l: bit(0), c: bit(1), r: bit(2) },	/* firmware labels */
			ir_mirrored: mirrored,
			pos: name(ENUMS.line_pos, raw.line_pos),
			pos_phys: name(ENUMS.line_pos_phys, linePosPhys(raw.line_pos, mirrored)),
			pos_phys_code: linePosPhys(raw.line_pos, mirrored),
			state: name(ENUMS.line_state, raw.line_state),
			steer: raw.steer				/* + = turn right (clockwise) */
		},
		barcode: {
			state: name(ENUMS.bc_state, raw.bc_state),
			symbol: symbolOf(raw.bc_symbol),
			result: name(ENUMS.bc_result, raw.bc_result),
			nav_valid: raw.nav_valid === 1,
			nav: raw.nav_valid ? name(ENUMS.nav, raw.nav) : null,	/* mapping, not execution */
			valid_reads_mod64: raw.bc_valid,
			invalid_reads_mod64: raw.bc_invalid
		},
		imu: {
			status: name(ENUMS.imu_status, raw.imu_status),
			pitch_deg: orNull(raw.pitch, -128),		/* nose up + */
			roll_deg: orNull(raw.roll, -128),		/* right side down + */
			yaw_rate_ccw_dps: orNull(raw.yaw_rate, -256),	/* left / CCW + */
			heading_ccw_deg: orNull(raw.heading, 511),	/* CCW + from calibration */
			motion_class: name(ENUMS.motion_class, raw.motion_class),
			tilt: raw.tilt === 1,
			collision: raw.collision === 1,			/* latched until CLEAR/RESUME */
			shock_g: raw.shock / 2
		},
		hump: {
			last_mm: humpMm(raw.hump_last),
			max_boot_mm: humpMm(raw.hump_max),		/* highest since robot boot */
			count: raw.hump_count,
			count_saturated: raw.hump_count === 15,
			active: raw.hump_active === 1
		},
		obstacle: {
			us_result: name(ENUMS.us_result, raw.us_result),
			distance_mm: distMm(raw.us_dist),
			no_echo: raw.us_dist === 1022,
			us_context: ctx,
			forward_mm: (ctx === 'forward') ? distMm(raw.us_dist) : null,
			avoid_state: name(ENUMS.avoid_state, raw.avoid_state),
			avoid_side: name(ENUMS.avoid_side, raw.avoid_side),
			avoid_outcome: name(ENUMS.avoid_outcome, raw.avoid_outcome),
			profile: prof,
			width_mm: (raw.obst_width === 255 || !profOk) ? null : raw.obst_width * 5,
			bearing_deg: profOk ? raw.obst_bearing : null,	/* + = right */
			clear_left_mm: clear(raw.clear_left),
			clear_left_at_least: raw.clear_left === 254,
			clear_right_mm: clear(raw.clear_right),
			clear_right_at_least: raw.clear_right === 254
		},
		comm: {
			state: name(ENUMS.comm_state, raw.comm_state),
			tlm_dropped: raw.tlm_dropped,
			tlm_dropped_saturated: raw.tlm_dropped === 255,
			rx_rejected: raw.rx_rejected,
			rx_rejected_saturated: raw.rx_rejected === 255
		}
	};
}

/* Heartbeat availability (FR13).  The robot only sends heartbeats while
   ONLINE, so a live heartbeat normally says ONLINE; telemetry queued
   during an outage carries the other states. */
function availability(commState) {
	if (commState === 4) { return 'ONLINE'; }
	if (commState === 7) { return 'DISABLED'; }
	return (commState >= 5) ? 'RECOVERING' : 'CONNECTING';
}

function interpretEvent(code, data) {
	const f = splitWord(data, EVT_DATA[code] || []);
	switch (code) {
	case 1: return { old_mode: name(ENUMS.mode, f.old_mode), new_mode: name(ENUMS.mode, f.new_mode),
			 epoch: f.epoch };
	case 2: return { fault_mask: f.fault_mask, faults: faultList(f.fault_mask) };
	case 3: return { symbol: symbolOf(f.symbol), result: name(ENUMS.bc_result, f.result),
			 nav_valid: f.nav_valid === 1, nav: f.nav_valid ? name(ENUMS.nav, f.nav) : null };
	case 4: return { last_mm: humpMm(f.last), max_boot_mm: humpMm(f.max), count: f.count,
			 count_saturated: f.count === 15 };
	case 5: return { outcome: name(ENUMS.avoid_outcome, f.outcome), fail: name(ENUMS.avoid_fail, f.fail),
			 side: name(ENUMS.avoid_side, f.side),
			 width_mm: (f.width === 255) ? null : f.width * 5, distance_mm: distMm(f.distance) };
	case 6: return { shock_g: f.shock / 2 };
	default: return {};
	}
}

/* ------------------------------------------------------------------ */
/*  Decode                                                             */

function options(opts) {
	const o = opts || {};
	return {
		irSidesMirrored: (o.irSidesMirrored === undefined) ? true : !!o.irSidesMirrored,
		policy: Object.assign({}, DEFAULT_POLICY, o.policy || {})
	};
}

/* Decode any frame.  { error } when the frame fails a structural check
   (nothing in it can be trusted); otherwise the decoded frame, whose
   'issues' list names fields holding values v1 never produces. */
function decode(buf, opts) {
	const o = options(opts);
	const h = checkFrame(buf);
	if (h.error) { return h; }
	const raw = readFields(buf, HDR_LEN, h.len, FIELDS[h.type]);
	const base = { version: h.version, type: h.type, kind: TYPE_NAMES[h.type], seq: h.seq, raw };

	switch (h.type) {
	case TYPE.TELEMETRY:
		return Object.assign(base, { issues: rangeIssues(raw, TLM_RANGE) }, interpretTelemetry(raw, o));
	case TYPE.HEARTBEAT:
		return Object.assign(base, {
			issues: rangeIssues(raw, HB_RANGE),
			uptime_s: raw.uptime_ds / 10, mode: name(ENUMS.mode, raw.mode),
			comm_state: name(ENUMS.comm_state, raw.comm_state),
			availability: availability(raw.comm_state), critical: raw.critical === 1,
			fault_count: raw.fault_count, reconnects: raw.reconnects,
			reconnects_saturated: raw.reconnects === 15
		});
	case TYPE.EVENT: {
		const known = raw.code in EVT_DATA;
		const fields = splitWord(raw.data, EVT_DATA[raw.code] || []);
		const issues = known ? rangeIssues(fields, EVT_RANGE[raw.code]) : ['code=' + raw.code];
		return Object.assign(base, { issues, event: known ? ENUMS.event[raw.code] : 'CODE_' + raw.code,
					      code: raw.code, data: raw.data, fields },
				     interpretEvent(raw.code, raw.data));
	}
	case TYPE.ACK:
		return Object.assign(base, {
			issues: rangeIssues(raw, ACK_RANGE), cmd_id: raw.cmd_id,
			cmd: name(ENUMS.command, raw.cmd), result: name(ENUMS.ack_result, raw.result),
			detail: (raw.result === 6) ? name(ENUMS.reject_reason, raw.detail) : raw.detail
		});
	default: {				/* a command: validate like the robot */
		const issues = [];
		if (raw.reserved !== 0 || raw.cmd >= ENUMS.command.length) { issues.push('FIELD'); }
		if (raw.cmd === 4 ? (raw.arg < 50 || raw.arg > 400) : raw.arg !== 0) { issues.push('ARG'); }
		return Object.assign(base, { issues, cmd: name(ENUMS.command, raw.cmd), arg: raw.arg,
					      cmd_id: raw.cmd_id });
	}
	}
}

/* ------------------------------------------------------------------ */
/*  Encode                                                             */

/* Build a frame from raw field values (the 'raw' object decode()
   returns).  Used by simulators and tests; the robot validates commands
   again. */
function encodeFrame(type, seq, raw) {
	const fields = FIELDS[type];
	if (!fields) { throw new Error('unknown frame type ' + type); }
	const w = bitWriter(PAYLOAD_LEN[type]);
	for (const [fname, bits, signed] of fields) {
		const v = (raw[fname] === undefined) ? 0 : raw[fname];
		const lo = signed ? -(2 ** (bits - 1)) : 0;
		const hi = signed ? 2 ** (bits - 1) - 1 : 2 ** bits - 1;
		if (!Number.isInteger(v) || v < lo || v > hi) {
			throw new Error(fname + '=' + v + ' does not fit ' + bits + ' bits');
		}
		if (signed) { w.putSigned(v, bits); } else { w.put(v, bits); }
	}
	const f = Buffer.concat([Buffer.from([(VERSION << 4) | type, seq & 0xFF, PAYLOAD_LEN[type]]),
				 w.buffer, Buffer.alloc(CRC_LEN)]);
	const c = crc16(f, f.length - CRC_LEN);
	f[f.length - 2] = c & 0xFF;
	f[f.length - 1] = c >> 8;
	return f;
}

/* Event frame from its code and named data-word fields (EVT_DATA) */
function encodeEvent(code, fields, seq) {
	let data = 0;
	let off = 0;
	for (const [fname, bits] of EVT_DATA[code] || []) {
		data += ((fields[fname] || 0) % 2 ** bits) * 2 ** off;
		off += bits;
	}
	return encodeFrame(TYPE.EVENT, seq, { code, data });
}

/* Build a command frame for csc14/cmd.  The robot validates it again. */
function encodeCommand(cmdName, arg, cmdId, seq) {
	const cmd = ENUMS.command.indexOf(cmdName);
	if (cmd < 0) { throw new Error('unknown command ' + cmdName); }
	if (cmd === 4 ? (arg < 50 || arg > 400) : arg !== 0) {
		throw new Error('argument out of range');
	}
	return encodeFrame(TYPE.COMMAND, seq, { cmd, arg, cmd_id: cmdId & 0xFF, reserved: 0 });
}

/* ------------------------------------------------------------------ */
/*  Golden frames from MQTT_BINARY_PROTOCOL.md section 7               */

const GOLDEN = {
	heartbeat: '12 2A 04 34 12 EC 3D 60 E5',
	command: '14 07 03 D4 47 0B 59 72',
	ack: '15 03 03 5A 34 02 87 52',
	event: '13 09 05 44 65 03 00 00 9B 88',
	telemetry: '11 80 23 EF BE 6B 69 69 69 35 00 FF 17 43 3D 4B D5 05 57 FA'
		+ ' 7F 08 A8 05 F0 FF 7D 4A FE FE 87 FA 1F E8 0F F0 FF 00 89 17'
};
const hex = (s) => Buffer.from(s.replace(/\s/g, ''), 'hex');

function selfTest() {
	const checks = [];
	const hb = decode(hex(GOLDEN.heartbeat));
	checks.push(['heartbeat', hb.uptime_s === 466 && hb.mode === 'SAFE_STOP' &&
		hb.comm_state === 'RETRY_WAIT' && hb.critical && hb.fault_count === 27 &&
		hb.reconnects === 3 && hb.issues.length === 0]);
	checks.push(['command encode', encodeCommand('SET_SPEED', 250, 0x5A, 7).equals(hex(GOLDEN.command))]);
	const ack = decode(hex(GOLDEN.ack));
	checks.push(['ack', ack.cmd_id === 0x5A && ack.cmd === 'SET_SPEED' &&
		ack.result === 'REJ_STATE' && ack.detail === 'MODE']);
	const ev = decode(hex(GOLDEN.event));
	checks.push(['event', ev.event === 'HUMP' && ev.last_mm === 40 && ev.max_boot_mm === 50 && ev.count === 3]);
	const t = decode(hex(GOLDEN.telemetry));
	checks.push(['telemetry', !t.error && t.issues.length === 0 && t.raw.uptime_ds === 0xBEEF &&
		t.fault_mask === 0x5A5A5A5 && t.motion.speed_l_mms === null && t.motion.speed_r_mms === 1023 &&
		t.motion.odometer_cm === 54321 && t.motion.heading_enc_cw_deg === 358 &&
		t.line.steer === -63 && t.barcode.symbol === 'A' && t.barcode.nav === 'U_TURN' &&
		t.imu.pitch_deg === null && t.imu.roll_deg === 90 && t.imu.yaw_rate_ccw_dps === null &&
		t.imu.heading_ccw_deg === null && t.hump.last_mm === 40 && t.hump.max_boot_mm === 100 &&
		t.obstacle.no_echo && t.obstacle.distance_mm === null && t.obstacle.width_mm === null &&
		t.obstacle.bearing_deg === -127 && t.obstacle.clear_left_mm === 2540 &&
		t.obstacle.clear_left_at_least && t.comm.state === 'DISABLED' && t.comm.tlm_dropped === 255]);
	/* the JavaScript encoder reproduces every golden frame byte for byte */
	checks.push(['encode golden telemetry', encodeFrame(TYPE.TELEMETRY, 0x80, t.raw).equals(hex(GOLDEN.telemetry))]);
	checks.push(['encode golden heartbeat', encodeFrame(TYPE.HEARTBEAT, 0x2A, hb.raw).equals(hex(GOLDEN.heartbeat))]);
	checks.push(['encode golden ack', encodeFrame(TYPE.ACK, 3, ack.raw).equals(hex(GOLDEN.ack))]);
	checks.push(['encode golden event', encodeEvent(4, { last: 20, max: 25, count: 3 }, 9).equals(hex(GOLDEN.event))]);
	/* severity follows sys_state_logic.c: LINE_RECOVERY (bit 7) is degraded */
	checks.push(['line recovery degraded', !(criticalMask() & (1 << 7)) && (criticalMask() & (1 << 5)) !== 0 &&
		(criticalMask({ collisionStops: false }) & (1 << 12)) === 0]);
	/* IR bits: firmware bit 0 (IR_L, GP2) is the car's RIGHT when mirrored */
	const ir = decode(encodeFrame(TYPE.TELEMETRY, 1, Object.assign({}, t.raw, { ir_black: 1, line_pos: 2 })));
	checks.push(['ir physical sides', ir.line.ir.right === 1 && ir.line.ir.left === 0 &&
		ir.line.pos_phys === 'RIGHT_FAR' &&
		decode(encodeFrame(TYPE.TELEMETRY, 1, Object.assign({}, t.raw, { ir_black: 1 })),
		       { irSidesMirrored: false }).line.ir.left === 1]);
	const bad = hex(GOLDEN.command);
	bad[4] ^= 0x10;
	checks.push(['crc rejected', decode(bad).error === 'CRC']);
	checks.push(['short rejected', decode(hex('14 07')).error === 'SHORT']);
	checks.push(['range issue reported', decode(encodeFrame(TYPE.TELEMETRY, 2,
		Object.assign({}, t.raw, { mode: 6 }))).issues[0] === 'mode=6']);
	let fails = 0;
	for (const [checkName, ok] of checks) {
		console.log((ok ? 'PASS ' : 'FAIL ') + checkName);
		fails += ok ? 0 : 1;
	}
	return fails;
}

const csc14 = {
	VERSION, HDR_LEN, CRC_LEN, TYPE, TYPE_NAMES, PAYLOAD_LEN, ENUMS, CODE39, FAULTS, FAULT_TABLE,
	FAULT_MASK_ALL, SEV, DEFAULT_POLICY, TLM_FIELDS, HB_FIELDS, EVT_FIELDS, CMD_FIELDS, ACK_FIELDS,
	EVT_DATA, GOLDEN, crc16, checkFrame, decode, encodeFrame, encodeEvent, encodeCommand,
	faultSeverity, criticalMask, faultList, linePosPhys, usContext, profileState, availability, selfTest
};

if (typeof module !== 'undefined' && module.exports) {
	module.exports = csc14;
	if (require.main === module) {
		process.exitCode = selfTest();
	}
} else if (typeof msg !== 'undefined' && typeof node !== 'undefined') {
	/* Node-RED Function node body */
	const out = decode(msg.payload);
	if (out.error || out.issues.length > 0) {
		node.warn('csc14: rejected frame on ' + msg.topic + ': ' + (out.error || out.issues.join(',')));
		return null;
	}
	msg.payload = out;
	return msg;
}
