/*
 * nodered_decoder.js
 * CSC14 binary MQTT protocol v1 -- server-side decoder and command encoder.
 * Specification: MQTT_BINARY_PROTOCOL.md (same folder).  Mirrors
 * app_program/subsystem1/mqtt_proto.c field for field.
 *
 * Three ways to use this one file:
 *   1. Node-RED: paste the whole file into a Function node fed by an
 *      "mqtt in" node subscribed to csc14/tlm, csc14/hb, csc14/evt and
 *      csc14/ack with output "a Buffer".  Valid frames leave the node with
 *      msg.payload = decoded object; invalid frames are dropped with a
 *      warning (NFR11 on the server side, FR14 validation).
 *   2. Node.js: const csc14 = require('./nodered_decoder.js');
 *      csc14.decode(buffer), csc14.encodeCommand('SET_SPEED', 250, id, seq)
 *   3. Self test against the golden frames: node nodered_decoder.js
 *
 * The firmware is the reference: if this file and mqtt_proto.c ever
 * disagree, the golden frames in MQTT_BINARY_PROTOCOL.md decide.
 */
'use strict';

const VERSION = 1;
const HDR_LEN = 3;
const CRC_LEN = 2;
const TYPE = { TELEMETRY: 1, HEARTBEAT: 2, EVENT: 3, COMMAND: 4, ACK: 5 };
const PAYLOAD_LEN = { 1: 35, 2: 4, 3: 5, 4: 3, 5: 3 };

const MODES = ['INIT', 'NORMAL', 'NAV_ACTION', 'OBSTACLE', 'SAFE_STOP'];
const FAULTS = [
	'MOTOR_STALL', 'ENCODER', 'MOTION_ESCALATED', 'MOTOR_WDOG', 'CMD_QUEUE',
	'IR_CAL', 'LINE_LOST', 'LINE_RECOVERY', 'BARCODE_INVALID', 'BARCODE_OVERFLOW',
	'IMU', 'IMU_CAL', 'COLLISION', 'US_NO_RESPONSE', 'US_INVALID',
	'SERVO_SUSPECT', 'OBSTACLE_UNAVOIDABLE', 'PROFILE_INVALID', 'AVOID_FAILED',
	'RECOVERY_TIMEOUT', 'WIFI_LOSS', 'MQTT_LOSS', 'CMD_INVALID', 'TLM_OVERFLOW',
	'INIT', 'TASK_COMM', 'REMOTE_STOP'
];
const CRITICAL = new Set([2, 3, 5, 7, 10, 11, 12, 13, 16, 17, 18, 19, 24, 25, 26]);
const COMM_STATES = ['INIT', 'CONNECTING_WIFI', 'CONNECTING_MQTT', 'SUBSCRIBING',
	'ONLINE', 'RETRY_WAIT', 'RECOVERY', 'DISABLED'];
const CODE39 = '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*';
const NAV = ['LEFT', 'RIGHT', 'STRAIGHT', 'U_TURN'];
const COMMANDS = { PING: 0, STOP: 1, RESUME: 2, CLEAR: 3, SET_SPEED: 4, REQ_STATUS: 5 };
const COMMAND_NAMES = ['PING', 'STOP', 'RESUME', 'CLEAR', 'SET_SPEED', 'REQ_STATUS'];
const ACK_RESULTS = ['OK', 'BAD_VERSION', 'BAD_TYPE', 'BAD_LENGTH', 'BAD_FIELD',
	'BAD_ARG', 'REJ_STATE', 'DUPLICATE', 'RATE', 'BUSY'];
const MREJ = ['NONE', 'FORMAT', 'RANGE', 'SOURCE', 'MODE', 'STALE', 'PRIORITY', 'QUEUE'];
const EVENTS = { 1: 'MODE', 2: 'FAULTS', 3: 'BARCODE', 4: 'HUMP', 5: 'OBSTACLE', 6: 'COLLISION' };

/* Telemetry schema v1, wire order (name, bits, signed) */
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
		buffer: out
	};
}

/* Header, length and CRC checks, in the firmware's order */
function checkFrame(buf) {
	if (!Buffer.isBuffer(buf) || buf.length < HDR_LEN + CRC_LEN) { return { error: 'SHORT' }; }
	const crcRx = buf[buf.length - 2] | (buf[buf.length - 1] << 8);
	if (crc16(buf, buf.length - CRC_LEN) !== crcRx) { return { error: 'CRC' }; }
	const version = buf[0] >> 4;
	const type = buf[0] & 0x0F;
	const len = buf[2];
	if (version !== VERSION) { return { error: 'VERSION' }; }
	if (!(type in PAYLOAD_LEN)) { return { error: 'TYPE' }; }
	if (len !== buf.length - HDR_LEN - CRC_LEN || len !== PAYLOAD_LEN[type]) { return { error: 'LENGTH' }; }
	return { version, type, seq: buf[1], len };
}

function orNull(v, invalid) { return (v === invalid) ? null : v; }

function faultList(mask) {
	const names = [];
	for (let i = 0; i < FAULTS.length; i++) { if (mask & (1 << i)) { names.push(FAULTS[i]); } }
	return names;
}

function decodeTelemetry(buf) {
	const rd = bitReader(buf, HDR_LEN, PAYLOAD_LEN[TYPE.TELEMETRY]);
	const raw = {};
	for (const [name, bits, signed] of TLM_FIELDS) {
		raw[name] = signed ? rd.getSigned(bits) : rd.get(bits);
	}
	return {
		raw,
		uptime_s: raw.uptime_ds / 10,
		mode: MODES[raw.mode] || 'INVALID',
		epoch: raw.epoch,
		faults: faultList(raw.faults),
		critical: faultList(raw.faults).some((n) => CRITICAL.has(FAULTS.indexOf(n))),
		speed_l_mms: orNull(raw.speed_l, -1024),
		speed_r_mms: orNull(raw.speed_r, -1024),
		odometer_m: raw.odometer_cm / 100,
		encoder_heading_deg: raw.heading2 * 2,
		cruise_mms: raw.cruise10 * 10,
		ir: { left: raw.ir_black & 1, centre: (raw.ir_black >> 1) & 1, right: (raw.ir_black >> 2) & 1 },
		line_pos: raw.line_pos,
		line_state: raw.line_state,
		steer: raw.steer,
		barcode: {
			state: raw.bc_state,
			symbol: (raw.bc_symbol < CODE39.length) ? CODE39[raw.bc_symbol] : null,
			result: raw.bc_result,
			nav: raw.nav_valid ? NAV[raw.nav] : null,
			valid_reads: raw.bc_valid,
			invalid_reads: raw.bc_invalid
		},
		imu: {
			status: raw.imu_status,
			pitch_deg: orNull(raw.pitch, -128),
			roll_deg: orNull(raw.roll, -128),
			yaw_rate_dps: orNull(raw.yaw_rate, -256),
			heading_deg: orNull(raw.heading, 511),
			motion_class: raw.motion_class,
			tilt: raw.tilt,
			collision: raw.collision,
			shock_g: raw.shock / 2
		},
		hump: {
			last_mm: (raw.hump_last === 63) ? null : raw.hump_last * 2,
			max_mm: (raw.hump_max === 63) ? null : raw.hump_max * 2,
			count: raw.hump_count,
			active: raw.hump_active
		},
		obstacle: {
			distance_mm: (raw.us_dist <= 800) ? raw.us_dist * 5 : null,
			no_echo: raw.us_dist === 1022,
			us_result: raw.us_result,
			avoid_state: raw.avoid_state,
			avoid_side: raw.avoid_side,
			avoid_outcome: raw.avoid_outcome,
			width_mm: (raw.obst_width === 255) ? null : raw.obst_width * 5,
			bearing_deg: raw.obst_bearing,
			clear_left_mm: (raw.clear_left === 255) ? null : raw.clear_left * 10,
			clear_right_mm: (raw.clear_right === 255) ? null : raw.clear_right * 10
		},
		comm_state: COMM_STATES[raw.comm_state],
		tlm_dropped: raw.tlm_dropped,
		rx_rejected: raw.rx_rejected
	};
}

function decodeEvent(code, d) {
	switch (code) {
	case 1: return { old_mode: MODES[d & 7], new_mode: MODES[(d >> 3) & 7], epoch: (d >> 6) & 7 };
	case 2: return { faults: faultList(d & 0x7FFFFFF) };
	case 3: return { symbol: CODE39[d & 0x3F] || null, result: (d >> 6) & 3,
			 nav: ((d >> 10) & 1) ? NAV[(d >> 8) & 3] : null };
	case 4: return { last_mm: ((d & 0x3F) === 63) ? null : (d & 0x3F) * 2,
			 max_mm: (((d >> 6) & 0x3F) === 63) ? null : ((d >> 6) & 0x3F) * 2,
			 count: (d >> 12) & 0xF };
	case 5: return { outcome: d & 3, fail: (d >> 2) & 7, side: (d >> 5) & 3,
			 width_mm: (((d >> 7) & 0xFF) === 255) ? null : ((d >> 7) & 0xFF) * 5,
			 distance_mm: (((d >>> 15) & 0x3FF) <= 800) ? ((d >>> 15) & 0x3FF) * 5 : null };
	case 6: return { shock_g: (d & 0x1F) / 2 };
	default: return {};
	}
}

/* Decode any frame; { error } for anything that fails validation */
function decode(buf) {
	const h = checkFrame(buf);
	if (h.error) { return h; }
	const rd = bitReader(buf, HDR_LEN, h.len);
	const base = { type: h.type, seq: h.seq };
	switch (h.type) {
	case TYPE.TELEMETRY:
		return Object.assign(base, { kind: 'telemetry' }, decodeTelemetry(buf));
	case TYPE.HEARTBEAT:
		return Object.assign(base, {
			kind: 'heartbeat', uptime_s: rd.get(16) / 10, mode: MODES[rd.get(3)] || 'INVALID',
			comm_state: COMM_STATES[rd.get(3)], critical: rd.get(1) === 1,
			fault_count: rd.get(5), reconnects: rd.get(4)
		});
	case TYPE.EVENT: {
		const code = rd.get(4);
		const data = rd.get(32);
		return Object.assign(base, { kind: 'event', event: EVENTS[code] || code, data },
				     decodeEvent(code, data));
	}
	case TYPE.ACK:
		return Object.assign(base, {
			kind: 'ack', cmd_id: rd.get(8), cmd: COMMAND_NAMES[rd.get(3)],
			result: ACK_RESULTS[rd.get(4)], detail: MREJ[rd.get(5)]
		});
	default: {				/* a command echoed back: validate like the robot */
		const cmd = rd.get(3);
		const arg = rd.get(10);
		const id = rd.get(8);
		if (rd.get(3) !== 0 || cmd >= COMMAND_NAMES.length) { return { error: 'FIELD' }; }
		return Object.assign(base, { kind: 'command', cmd: COMMAND_NAMES[cmd], arg, cmd_id: id });
	}
	}
}

/* Build a command frame for csc14/cmd.  The robot validates it again. */
function encodeCommand(name, arg, cmdId, seq) {
	const cmd = COMMANDS[name];
	if (cmd === undefined) { throw new Error('unknown command ' + name); }
	if (cmd === COMMANDS.SET_SPEED ? (arg < 50 || arg > 400) : arg !== 0) {
		throw new Error('argument out of range');
	}
	const w = bitWriter(PAYLOAD_LEN[TYPE.COMMAND]);
	w.put(cmd, 3);
	w.put(arg, 10);
	w.put(cmdId & 0xFF, 8);
	w.put(0, 3);
	const f = Buffer.concat([Buffer.from([(VERSION << 4) | TYPE.COMMAND, seq & 0xFF,
					      PAYLOAD_LEN[TYPE.COMMAND]]), w.buffer, Buffer.alloc(2)]);
	const c = crc16(f, f.length - CRC_LEN);
	f[f.length - 2] = c & 0xFF;
	f[f.length - 1] = c >> 8;
	return f;
}

/* Golden frames from MQTT_BINARY_PROTOCOL.md section 7 */
function selfTest() {
	const hex = (s) => Buffer.from(s.replace(/\s/g, ''), 'hex');
	const checks = [];
	const hb = decode(hex('12 2A 04 34 12 EC 3D 60 E5'));
	checks.push(['heartbeat', hb.uptime_s === 466 && hb.mode === 'SAFE_STOP' &&
		hb.comm_state === 'RETRY_WAIT' && hb.critical && hb.fault_count === 27 && hb.reconnects === 3]);
	checks.push(['command encode', encodeCommand('SET_SPEED', 250, 0x5A, 7)
		.equals(hex('14 07 03 D4 47 0B 59 72'))]);
	const ack = decode(hex('15 03 03 5A 34 02 87 52'));
	checks.push(['ack', ack.cmd_id === 0x5A && ack.cmd === 'SET_SPEED' &&
		ack.result === 'REJ_STATE' && ack.detail === 'MODE']);
	const ev = decode(hex('13 09 05 44 65 03 00 00 9B 88'));
	checks.push(['event', ev.event === 'HUMP' && ev.last_mm === 40 && ev.max_mm === 50 && ev.count === 3]);
	const t = decode(hex('11 80 23 EF BE 6B 69 69 69 35 00 FF 17 43 3D 4B D5 05 57 FA' +
			     '7F 08 A8 05 F0 FF 7D 4A FE FE 87 FA 1F E8 0F F0 FF 00 89 17'));
	checks.push(['telemetry', !t.error && t.raw.uptime_ds === 0xBEEF && t.raw.faults === 0x5A5A5A5 &&
		t.speed_l_mms === null && t.speed_r_mms === 1023 && t.raw.odometer_cm === 54321 &&
		t.raw.steer === -63 && t.barcode.symbol === 'A' && t.imu.pitch_deg === null &&
		t.imu.roll_deg === 90 && t.imu.yaw_rate_dps === null && t.imu.heading_deg === null &&
		t.hump.last_mm === 40 && t.hump.max_mm === 100 && t.obstacle.no_echo &&
		t.obstacle.width_mm === null && t.raw.obst_bearing === -127 &&
		t.obstacle.clear_left_mm === 2540 && t.comm_state === 'DISABLED' && t.tlm_dropped === 255]);
	const bad = hex('14 07 03 D4 47 0B 59 72');
	bad[4] ^= 0x10;
	checks.push(['crc rejected', decode(bad).error === 'CRC']);
	checks.push(['short rejected', decode(hex('14 07')).error === 'SHORT']);
	let fails = 0;
	for (const [name, ok] of checks) {
		console.log((ok ? 'PASS ' : 'FAIL ') + name);
		fails += ok ? 0 : 1;
	}
	return fails;
}

const csc14 = { VERSION, TYPE, crc16, checkFrame, decode, encodeCommand, selfTest };

if (typeof module !== 'undefined' && module.exports) {
	module.exports = csc14;
	if (require.main === module) {
		process.exitCode = selfTest();
	}
} else if (typeof msg !== 'undefined' && typeof node !== 'undefined') {
	/* Node-RED Function node body */
	const out = decode(msg.payload);
	if (out.error) {
		node.warn('csc14: rejected frame on ' + msg.topic + ': ' + out.error);
		return null;
	}
	msg.payload = out;
	return msg;
}
