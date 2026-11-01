/*
 * sim.js
 * A virtual robot that produces the same binary frames as the firmware
 * (encoded with the shared codec) for testing without the Pico.  It copies
 * the firmware's reporting behaviour, including its limits:
 *   - events are derived from consecutive 500 ms snapshots (tlm_build.c);
 *   - the first FAULTS event after going online carries every fault since
 *     boot (task_telemetry.c);
 *   - no OBSTACLE event for a failed avoidance unless failEvents is set
 *     (the patched tlm_events());
 *   - the 4-bit hump count saturates at 15;
 *   - while offline only the newest 4 frames are kept (mbf_telemetry,
 *     drop-oldest) and sent after reconnection.
 * Time is passed in, so tests are deterministic.
 */
'use strict';

const { codec } = require('./protocol');

const T = codec.TYPE;
const CODE39 = codec.CODE39;
const FLT = Object.fromEntries(codec.FAULTS.map((n, i) => [n, 2 ** i]));

function baseRecord() {
	return {
		uptime_ds: 0, mode: 0, epoch: 0, faults: 0, mstate: 0, msource: 0, cmd_active: 0,
		speed_l: 0, speed_r: 0, odometer_cm: 0, heading2: 0, cruise10: 16, ir_black: 2, line_pos: 0,
		line_state: 3, steer: 0, bc_state: 0, bc_symbol: 63, bc_result: 0, nav: 0, nav_valid: 0,
		bc_valid: 0, bc_invalid: 0, imu_status: 1, pitch: 0, roll: 0, yaw_rate: 0, heading: 0,
		motion_class: 0, tilt: 0, collision: 0, shock: 0, hump_last: 63, hump_max: 63, hump_count: 0,
		hump_active: 0, us_dist: 1022, us_result: 1, avoid_state: 0, avoid_side: 0, avoid_outcome: 0,
		obst_width: 0, obst_bearing: 0, clear_left: 0, clear_right: 0, comm_state: 4, tlm_dropped: 0,
		rx_rejected: 0
	};
}

class VirtualRobot {
	/* opts: bootMs, failEvents, seed */
	constructor(opts) {
		const o = opts || {};
		this.bootMs = o.bootMs || Date.now();
		this.failEvents = !!o.failEvents;
		this.rnd = mulberry32(o.seed || 1);
		this.seq = { 1: 0, 2: 0, 3: 0, 5: 0 };
		this.r = baseRecord();
		this.prev = null;
		this.prevAvoidCount = 0;
		this.avoidCount = 0;
		this.humpHeights = [];
		this.faultsSeen = 0;
		this.bootReportSent = false;
		this.online = true;
		this.reconnects = 0;
		this.queue = [];		/* offline frames, newest 4 kept */
		this.tlmDropped = 0;
		this.odoUm = 0;
		this.headUm = 0;		/* (dl - dr) accumulated, um */
	}

	uptimeDs(nowMs) {
		return Math.floor((nowMs - this.bootMs) / 100) % 65536;
	}

	setFaults(mask) {
		this.r.faults = mask >>> 0;
		this.faultsSeen = (this.faultsSeen | mask) >>> 0;
	}

	/* ------------------------------------------------------------------ */
	/*  Scenario: robot state at t seconds after boot (120 s cycle)        */

	scenario(t) {
		const r = this.r;
		const noise = (a) => Math.round((this.rnd() * 2 - 1) * a);
		const c = t % 120;
		let faults = r.faults & ~(FLT.LINE_LOST | FLT.BARCODE_INVALID | FLT.WIFI_LOSS | FLT.MQTT_LOSS);
		const driving = (v) => {
			r.mstate = 1; r.msource = 0; r.cmd_active = 0;
			r.speed_l = v + noise(8); r.speed_r = v + noise(8);
		};

		r.hump_active = 0; r.pitch = noise(1); r.roll = noise(1); r.tilt = 0;
		r.ir_black = 2; r.line_pos = 0; r.steer = noise(6); r.us_dist = 1022; r.us_result = 1;
		if (c < 3) {
			r.mode = 0; r.imu_status = (c < 2) ? 1 : 2; r.line_state = (c < 2) ? 3 : 0;
			r.speed_l = 0; r.speed_r = 0; r.mstate = 0; r.steer = 0;
			if (c < 0.5 && t > 1) { r.epoch = (r.epoch + 1) % 8; }
		} else if (c >= 100) {
			/* collision -> Safe Stop, latched until the next cycle */
			r.mode = 4; r.speed_l = 0; r.speed_r = 0; r.mstate = 0; r.collision = 1; r.shock = 5;
			faults |= FLT.COLLISION; r.motion_class = 0;
		} else {
			r.mode = 1; r.imu_status = 2; r.line_state = 0; r.collision = 0;
			faults &= ~FLT.COLLISION;
			driving(160);
			r.motion_class = 1;
			if (c >= 12 && c < 12.5) {				/* barcode 'A', valid, unmapped */
				r.bc_state = 1;
			} else if (c >= 12.5 && c < 13) {
				if (r.bc_state === 1) { r.bc_valid = (r.bc_valid + 1) % 64; }
				r.bc_state = 0; r.bc_symbol = CODE39.indexOf('A'); r.bc_result = 1; r.nav_valid = 0;
			}
			if (c >= 65 && c < 65.5) {				/* invalid barcode */
				if (r.bc_result !== 2) { r.bc_invalid = (r.bc_invalid + 1) % 64; }
				r.bc_result = 2; r.bc_symbol = 63; faults |= FLT.BARCODE_INVALID;
			}
			for (const [start, mm] of [[20, 42], [40, 56], [75, 30]]) {
				if (c >= start && c < start + 1.5) {
					r.hump_active = 1; r.pitch = (c < start + 0.7) ? 8 : -8;
				} else if (c >= start + 1.5 && c < start + 2 && !this.humpHeights.includes(start + Math.floor(t / 120) * 120)) {
					this.humpHeights.push(start + Math.floor(t / 120) * 120);
					const code = mm / 2;
					r.hump_last = code;
					r.hump_max = (r.hump_max === 63) ? code : Math.max(r.hump_max, code);
					r.hump_count = Math.min(15, r.hump_count + 1);
				}
			}
			if (c >= 30 && c < 31.5) {				/* line lost briefly */
				r.ir_black = 0; r.line_pos = 5; r.line_state = 1; r.steer = 20; faults |= FLT.LINE_LOST;
			} else if (c >= 33 && c < 34) {
				r.ir_black = 3; r.line_pos = 1; r.steer = -18;	/* line under IR_L = car's right */
			}
			if (c >= 50 && c < 62) {				/* obstacle avoided on the left */
				r.mode = 3;
				const phases = [[50.5, 1], [52.5, 2], [54, 3], [54.5, 4], [57.5, 5], [58.5, 6], [62, 7]];
				r.avoid_state = phases.find(([end]) => c < end)[1];
				r.us_dist = (r.avoid_state === 2 || r.avoid_state === 3) ? 60 + noise(40) + 40 : 70 + noise(2);
				r.us_result = 0;
				if (r.avoid_state >= 4) {
					r.obst_width = 30; r.obst_bearing = -5; r.clear_left = 40; r.clear_right = 12; r.avoid_side = 1;
				}
				if (r.avoid_state === 1 || r.avoid_state === 4) { r.speed_l = 0; r.speed_r = 0; r.mstate = 0; }
				if (r.avoid_state === 5) { r.msource = 2; r.cmd_active = 1; r.mstate = 2; }
			} else if (c >= 62 && r.avoid_state !== 0) {
				r.avoid_state = 0; r.avoid_outcome = 1; this.avoidCount = (this.avoidCount + 1) % 16;
			} else if (c < 50) {
				r.us_dist = 300 + noise(20); r.us_result = 0;
			}
		}
		this.setFaults(faults);
		/* odometer and encoder heading follow the wheel speeds */
		const dl = Math.max(0, r.speed_l) * 50;		/* um per 50 ms step */
		const dr = Math.max(0, r.speed_r) * 50;
		this.odoUm += 10 * (dl + dr) / 2;
		this.headUm += 10 * (dl - dr);
		r.odometer_cm = Math.min(65535, Math.floor(this.odoUm / 10000));
		const enc = ((Math.round(this.headUm * 57.296 / 100000 / 2) % 180) + 180) % 180;
		r.heading2 = enc;
		r.heading = (r.imu_status === 2) ? (360 - enc * 2 + 360) % 360 : 511;
		r.yaw_rate = (r.imu_status === 2) ? noise(3) : -256;
	}

	/* ------------------------------------------------------------------ */
	/*  Frames for one 500 ms telemetry cycle (+ heartbeat each second)    */

	frame(type, raw) {
		const f = codec.encodeFrame(type, this.seq[type], raw);
		this.seq[type] = (this.seq[type] + 1) % 256;
		return f;
	}

	evtFrame(code, fields) {
		const f = codec.encodeEvent(code, fields, this.seq[3]);
		this.seq[3] = (this.seq[3] + 1) % 256;
		return f;
	}

	/* tlm_events(): one event per change, from two snapshots */
	events(prev, cur) {
		const ev = [];
		if (cur.mode !== prev.mode) { ev.push([1, { old_mode: prev.mode, new_mode: cur.mode, epoch: cur.epoch }]); }
		if (cur.faults !== prev.faults) { ev.push([2, { fault_mask: cur.faults }]); }
		if (cur.bc_valid !== prev.bc_valid || cur.bc_invalid !== prev.bc_invalid) {
			ev.push([3, { symbol: cur.bc_symbol, result: cur.bc_result, nav: cur.nav, nav_valid: cur.nav_valid }]);
		}
		if (cur.hump_count !== prev.hump_count) {
			ev.push([4, { last: cur.hump_last, max: cur.hump_max, count: cur.hump_count }]);
		}
		const failed = this.failEvents && cur.avoid_state === 8 && prev.avoid_state !== 8;
		if (this.avoidCount !== this.prevAvoidCount || failed) {
			ev.push([5, { outcome: cur.avoid_outcome, fail: failed ? 2 : 0, side: cur.avoid_side,
				width: cur.obst_width, distance: 140 }]);
		}
		if (cur.collision && !prev.collision) { ev.push([6, { shock: cur.shock }]); }
		return ev;
	}

	/* Returns [{channel, frame}] to publish now (empty while offline) */
	cycle(nowMs, opts) {
		const o = opts || {};
		const out = [];
		const t = (nowMs - this.bootMs) / 1000;

		if (!o.frozen) { this.scenario(t); }
		this.r.uptime_ds = this.uptimeDs(nowMs);
		this.r.comm_state = this.online ? 4 : 5;
		this.r.faults = this.online ? (this.r.faults & ~(FLT.WIFI_LOSS)) >>> 0
			: (this.r.faults | FLT.WIFI_LOSS) >>> 0;
		this.faultsSeen = (this.faultsSeen | this.r.faults) >>> 0;
		this.r.tlm_dropped = Math.min(255, this.tlmDropped);
		const cur = Object.assign({}, this.r, o.override || {});
		const frames = [{ channel: 'tlm', frame: this.frame(T.TELEMETRY, cur) }];
		if (this.prev) {
			for (const [code, fields] of this.events(this.prev, cur)) {
				frames.push({ channel: 'evt', frame: this.evtFrame(code, fields) });
			}
		}
		this.prev = cur;
		this.prevAvoidCount = this.avoidCount;

		if (!this.online) {
			for (const f of frames) {
				this.queue.push(f);
				if (this.queue.length > 4) {
					this.queue.shift();
					this.tlmDropped++;
				}
			}
			return out;
		}
		if (!this.bootReportSent) {
			frames.push({ channel: 'evt', frame: this.evtFrame(2, { fault_mask: this.faultsSeen }) });
			this.bootReportSent = true;
		}
		if (this.queue.length) {
			out.push(...this.queue);
			this.queue = [];
		}
		if (o.heartbeat) {
			out.push({ channel: 'hb', frame: this.heartbeat(nowMs) });
		}
		out.push(...frames);
		return out;
	}

	heartbeat(nowMs) {
		const crit = codec.criticalMask();
		let n = 0;
		for (let i = 0; i < 27; i++) { n += (this.r.faults >>> i) & 1; }
		return this.frame(T.HEARTBEAT, { uptime_ds: this.uptimeDs(nowMs), mode: this.r.mode, comm_state: 4,
			critical: (this.r.faults & crit) ? 1 : 0, fault_count: n, reconnects: Math.min(15, this.reconnects) });
	}

	setOnline(on) {
		if (on && !this.online) { this.reconnects++; }
		this.online = on;
	}

	ack(cmdId, cmd, result, detail) {
		return this.frame(T.ACK, { cmd_id: cmdId, cmd, result, detail: detail || 0 });
	}
}

/* Small deterministic PRNG */
function mulberry32(a) {
	return function rnd() {
		a = (a + 0x6D2B79F5) | 0;
		let t = Math.imul(a ^ (a >>> 15), 1 | a);
		t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
		return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
	};
}

module.exports = { VirtualRobot, baseRecord, FLT };
