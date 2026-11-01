/*
 * Protocol codec (app_program/subsystem1/docs/nodered_decoder.js) as the
 * backend uses it: golden frames, signed/scaled values, invalid codes,
 * structural rejection, field ranges, severity policy, physical sides.
 */
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const { codec, telemetryColumns } = require('../../lib/protocol');

const T = codec.TYPE;
const hex = (s) => Buffer.from(s.replace(/\s/g, ''), 'hex');
const golden = codec.decode(hex(codec.GOLDEN.telemetry));
const tlm = (over, opts) => codec.decode(codec.encodeFrame(T.TELEMETRY, 1, Object.assign({}, golden.raw, over)), opts);

test('golden frames decode to the documented values and re-encode byte for byte', () => {
	assert.equal(golden.issues.length, 0);
	assert.equal(golden.raw.uptime_ds, 0xBEEF);
	assert.equal(golden.mode, 'OBSTACLE');
	assert.equal(golden.epoch, 5);
	assert.equal(golden.fault_mask, 0x5A5A5A5);
	assert.equal(golden.motion.speed_l_mms, null);			/* -1024 = no encoder data */
	assert.equal(golden.motion.speed_r_mms, 1023);
	assert.equal(golden.motion.heading_enc_cw_deg, 358);
	assert.equal(golden.motion.cruise_set_mms, 200);
	assert.equal(golden.line.steer, -63);
	assert.equal(golden.barcode.symbol, 'A');
	assert.equal(golden.barcode.valid_reads_mod64, 63);
	assert.equal(golden.imu.pitch_deg, null);
	assert.equal(golden.imu.roll_deg, 90);
	assert.equal(golden.imu.yaw_rate_ccw_dps, null);
	assert.equal(golden.imu.heading_ccw_deg, null);
	assert.equal(golden.imu.shock_g, 15.5);
	assert.equal(golden.hump.last_mm, 40);
	assert.equal(golden.hump.max_boot_mm, 100);
	assert.ok(golden.hump.count_saturated);
	assert.equal(golden.obstacle.no_echo, true);
	assert.equal(golden.obstacle.distance_mm, null);
	assert.equal(golden.obstacle.width_mm, null);
	assert.equal(golden.obstacle.bearing_deg, -127);
	assert.equal(golden.obstacle.clear_left_mm, 2540);
	assert.equal(golden.obstacle.clear_left_at_least, true);
	assert.equal(golden.obstacle.clear_right_mm, 0);
	assert.equal(golden.comm.state, 'DISABLED');
	assert.ok(golden.comm.tlm_dropped_saturated);
	for (const k of ['heartbeat', 'ack', 'event', 'telemetry']) {
		const d = codec.decode(hex(codec.GOLDEN[k]));
		assert.ok(codec.encodeFrame(d.type, d.seq, d.raw).equals(hex(codec.GOLDEN[k])), k);
	}
	assert.ok(codec.encodeCommand('SET_SPEED', 250, 0x5A, 7).equals(hex(codec.GOLDEN.command)));
});

test('random records survive encode -> decode with every field intact (signed fields included)', () => {
	let seed = 12345;
	const rnd = () => { seed = (seed * 1103515245 + 12345) % 2147483648; return seed / 2147483648; };
	for (let n = 0; n < 500; n++) {
		const raw = {};
		for (const [name, bits, signed] of codec.TLM_FIELDS) {
			const span = 2 ** bits;
			let v = Math.floor(rnd() * span);
			if (signed) { v -= span / 2; }
			raw[name] = v;
		}
		const d = codec.decode(codec.encodeFrame(T.TELEMETRY, n % 256, raw));
		assert.equal(d.error, undefined);
		assert.deepEqual(d.raw, raw);
	}
});

test('scaling and units', () => {
	const d = tlm({ speed_l: -500, speed_r: 321, heading2: 45, cruise10: 16, pitch: -30, roll: 12, yaw_rate: -255,
		heading: 359, shock: 3, hump_last: 21, hump_max: 50, us_dist: 800, us_result: 0, obst_width: 30,
		obst_bearing: 15, clear_left: 40, clear_right: 254, avoid_state: 0 });
	assert.equal(d.motion.speed_l_mms, -500);
	assert.equal(d.motion.heading_enc_cw_deg, 90);
	assert.equal(d.motion.cruise_set_mms, 160);
	assert.equal(d.imu.pitch_deg, -30);
	assert.equal(d.imu.yaw_rate_ccw_dps, -255);
	assert.equal(d.imu.heading_ccw_deg, 359);
	assert.equal(d.imu.shock_g, 1.5);
	assert.equal(d.hump.last_mm, 42);
	assert.equal(d.hump.max_boot_mm, 100);
	assert.equal(d.obstacle.distance_mm, 4000);
	assert.equal(d.obstacle.forward_mm, 4000);
	assert.equal(d.obstacle.width_mm, 150);
	assert.equal(d.obstacle.clear_left_mm, 400);
	assert.equal(d.obstacle.clear_right_mm, 2540);
	assert.equal(d.obstacle.clear_right_at_least, true);
});

test('invalid codes become NULL in the typed columns, with their status kept', () => {
	const d = tlm({ speed_l: -1024, speed_r: -1024, pitch: -128, roll: -128, yaw_rate: -256, heading: 511,
		hump_last: 63, hump_max: 63, us_dist: 1023, us_result: 3, obst_width: 255, clear_left: 255,
		clear_right: 255, imu_status: 3 });
	const c = telemetryColumns(d);
	for (const k of ['speed_l_mms', 'speed_r_mms', 'pitch_deg', 'roll_deg', 'yaw_rate_ccw_dps',
		'heading_imu_ccw_deg', 'hump_last_mm', 'hump_max_boot_mm', 'us_dist_mm', 'us_forward_mm',
		'obst_width_mm', 'clear_left_mm', 'clear_right_mm']) {
		assert.equal(c[k], null, k);
	}
	assert.equal(c.us_result, 3);		/* INVALID kept as status */
	assert.equal(c.imu_status, 3);
	assert.equal(c.us_no_echo, false);
	assert.equal(tlm({ us_dist: 1022 }).obstacle.no_echo, true);
});

test('structural errors reject the whole frame', () => {
	const good = hex(codec.GOLDEN.heartbeat);
	const crc = Buffer.from(good); crc[4] ^= 1;
	assert.equal(codec.decode(crc).error, 'CRC');
	assert.equal(codec.decode(good.subarray(0, 4)).error, 'SHORT');
	assert.equal(codec.decode(Buffer.alloc(0)).error, 'SHORT');
	assert.equal(codec.decode('not a buffer').error, 'SHORT');
	const reframe = (bytes) => {
		const f = Buffer.from(bytes);
		const c = codec.crc16(f, f.length - 2);
		f[f.length - 2] = c & 0xFF; f[f.length - 1] = c >> 8;
		return f;
	};
	const v2 = Buffer.from(good); v2[0] = 0x22;
	assert.equal(codec.decode(reframe(v2)).error, 'VERSION');
	for (const type of [0, 6, 15]) {
		const t = Buffer.from(good); t[0] = 0x10 | type;
		assert.equal(codec.decode(reframe(t)).error, 'TYPE', 'type ' + type);
	}
	const len = Buffer.from(good); len[2] = 5;
	assert.equal(codec.decode(reframe(len)).error, 'LENGTH');
	/* a heartbeat payload with a telemetry header: length != schema */
	const wrongType = Buffer.from(good); wrongType[0] = 0x11;
	assert.equal(codec.decode(reframe(wrongType)).error, 'LENGTH');
	const extra = reframe(Buffer.concat([good.subarray(0, 7), Buffer.from([0]), good.subarray(7)]));
	assert.equal(codec.decode(extra).error, 'LENGTH');
});

test('values v1 never produces are reported as issues', () => {
	const cases = [['mode', 5], ['heading2', 180], ['steer', -64], ['bc_symbol', 50], ['pitch', 100],
		['roll', -91], ['heading', 400], ['hump_last', 55], ['hump_max', 51], ['us_dist', 900],
		['avoid_state', 9], ['avoid_side', 3], ['obst_bearing', -128]];
	for (const [k, v] of cases) {
		assert.deepEqual(tlm({ [k]: v }).issues, [k + '=' + v]);
	}
	const hb = codec.decode(codec.encodeFrame(T.HEARTBEAT, 1, { uptime_ds: 1, mode: 7, comm_state: 4,
		critical: 0, fault_count: 28, reconnects: 0 }));
	assert.deepEqual(hb.issues, ['mode=7', 'fault_count=28']);
	assert.deepEqual(codec.decode(codec.encodeFrame(T.EVENT, 1, { code: 0, data: 0 })).issues, ['code=0']);
	assert.deepEqual(codec.decode(codec.encodeFrame(T.EVENT, 1, { code: 9, data: 0 })).issues, ['code=9']);
	assert.deepEqual(codec.decode(codec.encodeEvent(1, { old_mode: 1, new_mode: 6 }, 1)).issues, ['new_mode=6']);
	assert.deepEqual(codec.decode(codec.encodeEvent(5, { fail: 7, distance: 900 }, 1)).issues, ['fail=7', 'distance=900']);
	assert.deepEqual(codec.decode(codec.encodeFrame(T.ACK, 1, { cmd_id: 1, cmd: 6, result: 12, detail: 0 })).issues,
		['cmd=6', 'result=12']);
});

test('fault severity follows sys_state_logic.c and its policy flags', () => {
	const crit = (p) => codec.FAULTS.filter((n, i) => codec.criticalMask(p) & (2 ** i));
	assert.deepEqual(crit(), ['MOTION_ESCALATED', 'MOTOR_WDOG', 'IR_CAL', 'IMU', 'IMU_CAL', 'COLLISION',
		'US_NO_RESPONSE', 'OBSTACLE_UNAVOIDABLE', 'PROFILE_INVALID', 'AVOID_FAILED', 'RECOVERY_TIMEOUT',
		'INIT', 'TASK_COMM', 'REMOTE_STOP']);
	assert.equal(codec.faultSeverity(7), codec.SEV.DEGRADED);		/* LINE_RECOVERY */
	assert.equal(codec.faultSeverity(10), codec.SEV.CRIT_COND);
	assert.equal(codec.faultSeverity(10, { imuRequired: false }), codec.SEV.DEGRADED);
	assert.equal(codec.faultSeverity(11, { imuRequired: false }), codec.SEV.DEGRADED);
	assert.equal(codec.faultSeverity(12, { collisionStops: false }), codec.SEV.DEGRADED);
	assert.equal(codec.faultSeverity(13, { usRequired: false }), codec.SEV.DEGRADED);
	for (const bit of [20, 21, 22, 23]) {
		assert.equal(codec.faultSeverity(bit), codec.SEV.DEGRADED, 'network faults are never critical');
	}
	assert.equal(tlm({ faults: 2 ** 7 }).critical, false);
	assert.equal(tlm({ faults: 2 ** 12 }).critical, true);
	assert.equal(tlm({ faults: 2 ** 12 }, { policy: { collisionStops: false } }).critical, false);
});

test('IR sensors and line position use the car\'s real sides', () => {
	/* firmware bit 0 = IR_L (GP2), bit 2 = IR_R (GP27, barcode sensor) */
	const m = tlm({ ir_black: 0b100 });
	assert.deepEqual(m.line.ir, { left: 1, centre: 0, right: 0 });
	assert.deepEqual(tlm({ ir_black: 0b100 }, { irSidesMirrored: false }).line.ir, { left: 0, centre: 0, right: 1 });
	const mirrored = ['CENTRE', 'RIGHT_NEAR', 'RIGHT_FAR', 'LEFT_FAR', 'LOST_RIGHT', 'LOST_LEFT', 'JUNCTION', 'LEFT_NEAR'];
	const direct = ['CENTRE', 'LEFT_NEAR', 'LEFT_FAR', 'RIGHT_FAR', 'LOST_LEFT', 'LOST_RIGHT', 'JUNCTION', 'RIGHT_NEAR'];
	for (let p = 0; p < 8; p++) {
		assert.equal(tlm({ line_pos: p }).line.pos_phys, mirrored[p], 'mirrored ' + p);
		assert.equal(tlm({ line_pos: p }, { irSidesMirrored: false }).line.pos_phys, direct[p], 'direct ' + p);
	}
	/* steer is already in the motor convention: never mirrored again */
	assert.equal(tlm({ steer: 20 }).line.steer, 20);
});

test('ultrasonic meaning depends on the avoidance phase; zero profile words mean no profile', () => {
	const ctx = (s) => tlm({ avoid_state: s, us_dist: 100 }).obstacle;
	for (const s of [0, 1, 5, 6, 7, 8]) { assert.equal(ctx(s).us_context, 'forward'); assert.equal(ctx(s).forward_mm, 500); }
	for (const s of [2, 3]) { assert.equal(ctx(s).us_context, 'scan_sweep'); assert.equal(ctx(s).forward_mm, null); }
	assert.equal(ctx(4).us_context, 'held');
	const none = tlm({ obst_width: 0, obst_bearing: 0, clear_left: 0, clear_right: 0 }).obstacle;
	assert.equal(none.profile, 'none_yet');
	assert.equal(none.width_mm, null);
	assert.equal(none.clear_left_mm, null);
	assert.equal(none.bearing_deg, null);
	assert.equal(tlm({ obst_width: 30, obst_bearing: 0, clear_left: 0, clear_right: 0 }).obstacle.profile, 'reported');
});

test('event and heartbeat interpretation', () => {
	const e = (code, f) => codec.decode(codec.encodeEvent(code, f, 3));
	assert.deepEqual([e(1, { old_mode: 1, new_mode: 4, epoch: 2 }).new_mode, e(1, { old_mode: 1, new_mode: 4 }).old_mode],
		['SAFE_STOP', 'NORMAL']);
	assert.deepEqual(e(2, { fault_mask: 2 ** 6 + 2 ** 20 }).faults, ['LINE_LOST', 'WIFI_LOSS']);
	const b = e(3, { symbol: 35, result: 1, nav: 1, nav_valid: 1 });
	assert.deepEqual([b.symbol, b.result, b.nav], ['Z', 'VALID', 'RIGHT']);
	assert.equal(e(3, { symbol: 63, result: 2, nav: 1, nav_valid: 0 }).nav, null);
	const h = e(4, { last: 15, max: 63, count: 15 });
	assert.deepEqual([h.last_mm, h.max_boot_mm, h.count_saturated], [30, null, true]);
	const o = e(5, { outcome: 2, fail: 5, side: 1, width: 255, distance: 1023 });
	assert.deepEqual([o.outcome, o.fail, o.side, o.width_mm, o.distance_mm], ['FAILED', 'TIMEOUT', 'LEFT', null, null]);
	assert.equal(e(6, { shock: 7 }).shock_g, 3.5);
	const hb = codec.decode(codec.encodeFrame(T.HEARTBEAT, 1, { uptime_ds: 100, mode: 1, comm_state: 6,
		critical: 0, fault_count: 1, reconnects: 15 }));
	assert.deepEqual([hb.availability, hb.reconnects_saturated], ['RECOVERING', true]);
});
