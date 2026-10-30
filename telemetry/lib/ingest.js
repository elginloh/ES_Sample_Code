/*
 * ingest.js
 * Validation, interpretation and bookkeeping of robot frames (pure logic:
 * no network, no database, time is passed in).  handle() turns one MQTT
 * message into database operations; tick() turns the passage of time into
 * link-state operations.  Node-RED calls both (nodered-api.js); the unit
 * tests call them directly.
 *
 * What the robot does NOT send, and how it is handled here:
 *   - no boot id: a "boot session" is inferred from uptime continuity.
 *     uptime_ds is 16 bits of 0.1 s and wraps every 6553.6 s; frames of the
 *     same session keep uptime and receive time moving together.
 *   - no event time: events carry only a sequence number.  received_at is
 *     the time the backend got the frame, never presented as event time.
 *   - per-type sequence numbers are 8 bits; a gap is only computed while
 *     it is unambiguous (fewer than 128 periods since the previous frame).
 *   - telemetry is a 500 ms snapshot: changes shorter than that, and
 *     changes during a network outage, may never be seen.
 */
'use strict';

const crypto = require('crypto');
const { codec, telemetryColumns } = require('./protocol');

const E = codec.ENUMS;
const UPTIME_MOD = 65536;		/* 16-bit uptime, 0.1 s units */
const LATE_TOL_DS = 100;		/* a frame may be up to 10 s older than expected (queued on the robot) */
const EARLY_TOL_DS = 50;		/* or 5 s newer (the reference frame itself was late) */
const FRESH_DS = 15;			/* within 1.5 s: timely, becomes the new reference */
const MAX_CONTINUITY_MS = 24 * 3600 * 1000;
const LATE_MS = 1500;
const PERIOD_MS = { 1: 500, 2: 1000 };	/* telemetry, heartbeat */
const SEQ_TIME_LIMIT = 120;		/* periods after which an 8-bit gap is ambiguous */
const REJECT_STORE_PER_MIN = 30;
const ROW_REFRESH_MS = 5000;

const op = (table, row, priority, key) => ({ table, row, priority: !!priority, key });

function severityOfFault(bit, policy) {
	return (codec.faultSeverity(bit, policy) === codec.SEV.DEGRADED) ? 'warning' : 'critical';
}

class Ingestor {
	constructor(cfg) {
		this.robotId = cfg.robotId;
		this.topics = cfg.topics;
		this.decodeOpts = cfg.decode;
		this.link = cfg.link;
		this.storeJson = cfg.store ? cfg.store.storeDecodedJson !== false : true;
		this.streams = new Map();
		this.brokerConnected = null;
		this.stats = {
			frames: 0, ignored: 0, duplicates: 0, rejected: 0, rejectedNotStored: 0,
			byKind: { telemetry: 0, heartbeat: 0, event: 0, ack: 0, command: 0 },
			rejectReasons: {}, sessions: 0, seqGapFrames: 0
		};
	}

	/* "csc14/tlm" -> { source: 'live', channel: 'tlm' } */
	classify(topic) {
		const t = String(topic || '');
		for (const [source, prefix] of [['simulated', this.topics.sim], ['live', this.topics.live]]) {
			if (prefix && t.startsWith(prefix + '/')) {
				const channel = t.slice(prefix.length + 1);
				if (channel.length > 0 && !channel.includes('/')) {
					return { source, channel };
				}
			}
		}
		return null;
	}

	stream(source) {
		const key = this.robotId + '|' + source;
		let s = this.streams.get(key);

		if (!s) {
			s = {
				robotId: this.robotId, source, boot: null, ref: null, minOffsetMs: null,
				seq: {}, lastRx: {}, prev: null, hb: null,
				link: { state: 'NEVER_SEEN', since: null }, lastLinkWriteMs: 0,
				lastBootWriteMs: 0, rejectWindow: { start: 0, n: 0 }
			};
			this.streams.set(key, s);
		}
		return s;
	}

	base(s, topic, rxMs) {
		return { robot_id: s.robotId, source: s.source, received_at: new Date(rxMs), topic };
	}

	/* ------------------------------------------------------------------ */
	/*  One MQTT message                                                   */

	handle(topic, payload, rxMs) {
		const ops = [];
		const where = this.classify(topic);

		this.stats.frames++;
		if (!where) {
			this.stats.ignored++;
			return { kind: 'ignored', ops };
		}
		const s = this.stream(where.source);
		const buf = Buffer.isBuffer(payload) ? payload : Buffer.from(payload === undefined ? [] : payload);
		const d = codec.decode(buf, this.decodeOpts);
		const base = this.base(s, topic, rxMs);

		if (d.error) {
			return this.reject(s, base, buf, d.error, null, rxMs, ops);
		}
		if (d.issues.length > 0) {
			return this.reject(s, base, buf, 'FIELD_RANGE', d.issues.join(','), rxMs, ops);
		}
		this.stats.byKind[d.kind]++;
		switch (d.kind) {
		case 'telemetry':
			this.onTelemetry(s, d, buf, base, rxMs, ops);
			break;
		case 'heartbeat':
			this.onHeartbeat(s, d, buf, base, rxMs, ops);
			break;
		case 'event':
			this.onEvent(s, d, buf, base, rxMs, ops);
			break;
		case 'ack':
			this.onAck(s, d, buf, base, rxMs, ops);
			break;
		default:			/* a command frame seen on an observed topic */
			ops.push(this.event(s, base, 'bus', 'COMMAND_SEEN', 'info',
				`Command ${d.cmd} (id ${d.cmd_id}${d.cmd === 'SET_SPEED' ? ', ' + d.arg + ' mm/s' : ''}) published on ${base.topic}`,
				{ cmd: d.cmd, arg: d.arg, cmd_id: d.cmd_id }, { seq: d.seq, frame: buf }));
		}
		this.touchBoot(s, rxMs, ops);
		return { kind: d.kind, decoded: d, ops };
	}

	reject(s, base, buf, reason, detail, rxMs, ops) {
		const w = s.rejectWindow;

		this.stats.rejected++;
		this.stats.rejectReasons[reason] = (this.stats.rejectReasons[reason] || 0) + 1;
		if (rxMs - w.start >= 60000) {
			w.start = rxMs;
			w.n = 0;
		}
		if (w.n < REJECT_STORE_PER_MIN) {
			w.n++;
			ops.push(op('rejected_frame', Object.assign({}, base, {
				reason, detail, length: buf.length, frame: buf.subarray(0, 128)
			}), true));
		} else {
			this.stats.rejectedNotStored++;
		}
		return { kind: 'rejected', reason, detail, ops };
	}

	event(s, base, origin, kind, severity, message, data, extra) {
		return op('event', Object.assign({}, base, {
			boot_id: s.boot ? s.boot.id : null, origin, kind, severity, message,
			data: data || {}, seq: null, frame: null
		}, extra || {}), true);
	}

	/* ------------------------------------------------------------------ */
	/*  Boot sessions from uptime continuity                               */

	newBoot(s, reason, uptimeDs, rxMs, base, ops) {
		const prev = s.boot;

		s.boot = { id: crypto.randomUUID(), firstRxMs: rxMs, firstUptimeDs: uptimeDs, reason,
			   faultsReported: false, lastUptimeS: uptimeDs / 10 };
		s.ref = null;
		s.minOffsetMs = null;
		s.seq = {};
		s.prev = null;
		s.hb = null;
		this.stats.sessions++;
		ops.push(this.bootRow(s, rxMs));
		const up = (uptimeDs / 10).toFixed(1);
		if (reason === 'first_seen') {
			ops.push(this.event(s, base, 'server', 'SESSION_START', 'info',
				`Backend started following the robot (robot uptime ${up} s; earlier history of this boot not observed)`,
				{ uptime_s: uptimeDs / 10, reason }));
		} else {
			ops.push(this.event(s, base, 'server', 'BOOT_DETECTED', 'warning',
				(reason === 'robot_restart')
					? `Robot restarted (inferred from uptime ${up} s; previous session ${prev.id.slice(0, 8)})`
					: `Robot uptime does not continue the previous session (${reason}); new session from uptime ${up} s`,
				{ uptime_s: uptimeDs / 10, reason, previous_boot_id: prev ? prev.id : null }));
		}
	}

	/* Place an uptime-carrying frame in a boot session.  Returns its
	   unwrapped uptime (0.1 s) and an estimate of how late it arrived. */
	locate(s, uptimeDs, rxMs, base, ops) {
		const r = s.ref;
		let U = uptimeDs;
		let diff = 0;

		if (!s.boot) {
			this.newBoot(s, 'first_seen', uptimeDs, rxMs, base, ops);
		} else if (!r || rxMs - r.t > MAX_CONTINUITY_MS) {
			this.newBoot(s, 'continuity_unknown', uptimeDs, rxMs, base, ops);
		} else {
			const expected = r.U + (rxMs - r.t) / 100;
			U = uptimeDs + UPTIME_MOD * Math.round((expected - uptimeDs) / UPTIME_MOD);
			diff = U - expected;
			if (diff < -LATE_TOL_DS || diff > EARLY_TOL_DS) {
				/* booted after we last heard from it: uptime is shorter than
				   the time since then */
				const restart = uptimeDs < (rxMs - r.t) / 100 + LATE_TOL_DS;
				this.newBoot(s, restart ? 'robot_restart' : 'uptime_discontinuity', uptimeDs, rxMs, base, ops);
				U = uptimeDs;
				diff = 0;
			}
		}
		if (!s.ref || diff >= -FRESH_DS) {
			s.ref = { t: rxMs, U };
		}
		const offset = rxMs - U * 100;
		if (s.minOffsetMs === null || offset < s.minOffsetMs) {
			s.minOffsetMs = offset;
		}
		const delayMs = Math.round(offset - s.minOffsetMs);
		s.boot.lastUptimeS = U / 10;
		return { U, delayMs, late: delayMs > LATE_MS };
	}

	bootRow(s, rxMs) {
		const b = s.boot;
		s.lastBootWriteMs = rxMs;
		return op('boot_session', {
			boot_id: b.id, robot_id: s.robotId, source: s.source,
			first_rx_at: new Date(b.firstRxMs), last_rx_at: new Date(rxMs),
			first_uptime_s: b.firstUptimeDs / 10, last_uptime_s: b.lastUptimeS,
			start_reason: b.reason, ir_sides_mirrored: this.decodeOpts.irSidesMirrored,
			policy: this.decodeOpts.policy
		}, true, 'boot_session:' + b.id);
	}

	touchBoot(s, rxMs, ops) {
		if (s.boot && rxMs - s.lastBootWriteMs >= ROW_REFRESH_MS) {
			ops.push(this.bootRow(s, rxMs));
		}
	}

	/* Frames missing before this one, per message type.  null = unknown. */
	seqGap(s, type, seq, rxMs, buf) {
		const last = s.seq[type];
		const crc = buf.readUInt16LE(buf.length - 2);

		s.seq[type] = { seq, t: rxMs, crc };
		if (!last) {
			return { gap: null };
		}
		const delta = (seq - last.seq + 256) % 256;
		if (delta === 0) {
			return (crc === last.crc) ? { duplicate: true } : { gap: null };
		}
		const period = PERIOD_MS[type];
		if (period && rxMs - last.t > SEQ_TIME_LIMIT * period) {
			return { gap: null, estimate: Math.max(0, Math.round((rxMs - last.t) / period) - 1) };
		}
		if (delta > 128) {
			return { gap: null };		/* out of order */
		}
		this.stats.seqGapFrames += delta - 1;
		return { gap: delta - 1 };
	}

	/* ------------------------------------------------------------------ */
	/*  Telemetry                                                          */

	onTelemetry(s, d, buf, base, rxMs, ops) {
		const loc = this.locate(s, d.raw.uptime_ds, rxMs, base, ops);
		const gap = this.seqGap(s, 1, d.seq, rxMs, buf);

		if (gap.duplicate) {
			this.stats.duplicates++;
			return;
		}
		const decoded = Object.assign({}, d);
		delete decoded.raw;
		ops.push(op('telemetry', Object.assign({}, base, {
			boot_id: s.boot.id, proto_version: d.version, seq: d.seq, seq_gap: gap.gap,
			rx_delay_ms: loc.delayMs, late: loc.late, uptime_ds: d.raw.uptime_ds,
			uptime_s: loc.U / 10
		}, telemetryColumns(d), {
			frame: buf, decoded: this.storeJson ? decoded : null
		}), false));
		if (gap.gap > 0 || gap.estimate > 0) {
			const n = (gap.gap !== null && gap.gap !== undefined) ? gap.gap : gap.estimate;
			ops.push(this.event(s, base, 'server', 'TELEMETRY_GAP', 'warning',
				`${n} telemetry frame(s) not received before seq ${d.seq}` +
				((gap.gap === null) ? ' (estimated from time: sequence numbers wrapped)' : ''),
				{ missing: n, exact: gap.gap !== null, seq: d.seq }));
		}
		if (s.prev) {
			this.derive(s, s.prev, d, base, gap, ops);
		}
		s.prev = { raw: d.raw, rxMs };
		s.lastRx.tlm = rxMs;
	}

	/* Events the robot does not send, from two consecutive snapshots */
	derive(s, p, d, base, gap, ops) {
		const r = d.raw;
		const pol = this.decodeOpts.policy;
		const after = (gap.gap > 0) ? ` (after ${gap.gap} missing frame(s))` : '';
		const changed = (p.raw.faults ^ r.faults) >>> 0;

		for (let bit = 0; bit < codec.FAULTS.length; bit++) {
			if (!((changed >>> bit) & 1)) { continue; }
			const set = ((r.faults >>> bit) & 1) === 1;
			const sev = codec.faultSeverity(bit, pol);
			ops.push(this.event(s, base, 'server', set ? 'FAULT_SET' : 'FAULT_CLEARED',
				set ? severityOfFault(bit, pol) : 'info',
				`Fault ${set ? 'set' : 'cleared'}: ${codec.FAULTS[bit]} (${sev.toLowerCase().replace('_', ' ')})${after}`,
				{ bit, fault: codec.FAULTS[bit], severity: sev }));
		}
		const pr = p.raw;
		if (pr.tilt !== r.tilt) {
			ops.push(this.event(s, base, 'server', r.tilt ? 'TILT_ON' : 'TILT_OFF', r.tilt ? 'warning' : 'info',
				`Tilt ${r.tilt ? 'detected' : 'ended'} (pitch ${d.imu.pitch_deg} deg, roll ${d.imu.roll_deg} deg)${after}`,
				{ pitch_deg: d.imu.pitch_deg, roll_deg: d.imu.roll_deg }));
		}
		if (pr.collision === 1 && r.collision === 0) {
			ops.push(this.event(s, base, 'server', 'COLLISION_CLEARED', 'info',
				`Collision latch cleared${after}`, {}));
		}
		if (pr.line_state !== r.line_state) {
			const to = E.line_state[r.line_state];
			ops.push(this.event(s, base, 'server', 'LINE_STATE',
				(to === 'SEARCHING' || to === 'LOST') ? 'warning' : 'info',
				`Line ${E.line_state[pr.line_state]} -> ${to}${after}`,
				{ from: E.line_state[pr.line_state], to }));
		}
		if (pr.comm_state !== r.comm_state) {
			ops.push(this.event(s, base, 'server', 'ROBOT_COMM_STATE', 'info',
				`Robot-reported link ${E.comm_state[pr.comm_state]} -> ${E.comm_state[r.comm_state]}${after}`,
				{ from: E.comm_state[pr.comm_state], to: E.comm_state[r.comm_state] }));
		}
		if (r.avoid_state === 8 && pr.avoid_state !== 8) {
			/* entering FAILED, once per failure (the outcome stays FAILED
			   until the next success).  Unpatched v1 firmware sends no
			   OBSTACLE event for a failure; the patched tlm_events() does,
			   with the reason.  Failures are counted from this event. */
			ops.push(this.event(s, base, 'server', 'AVOID_FAILED', 'critical',
				`Obstacle avoidance FAILED (state ${d.obstacle.avoid_state}, side ${d.obstacle.avoid_side}); ` +
				'failure reason only available from a robot OBSTACLE event' + after,
				{ avoid_state: d.obstacle.avoid_state, side: d.obstacle.avoid_side }));
		}
		this.deriveHumps(s, pr, d, base, ops);
	}

	humpOp(s, base, f) {
		return op('hump', Object.assign({}, base, {
			boot_id: s.boot.id, height_mm: (f.code === null || f.code === 63) ? null : f.code * 2,
			height_code: f.code, count_after: f.count, max_boot_mm: (f.max === 63) ? null : f.max * 2,
			detected_by: f.by
		}), true);
	}

	/* Humps completed since the previous snapshot.  The first snapshot of
	   a session is only a baseline: humps before it are not "new". */
	deriveHumps(s, pr, d, base, ops) {
		const r = d.raw;

		if (r.hump_count > pr.hump_count) {
			for (let c = pr.hump_count + 1; c < r.hump_count; c++) {
				ops.push(this.humpOp(s, base, { code: null, count: c, max: r.hump_max, by: 'telemetry' }));
			}
			ops.push(this.humpOp(s, base, { code: r.hump_last, count: r.hump_count, max: r.hump_max, by: 'telemetry' }));
			if (r.hump_count - pr.hump_count > 1) {
				ops.push(this.event(s, base, 'server', 'HUMP_HEIGHT_UNKNOWN', 'warning',
					`${r.hump_count - pr.hump_count - 1} hump(s) completed between snapshots; ` +
					'their heights are only known if the robot HUMP events arrived', {}));
			}
		} else if (r.hump_count === 15 && pr.hump_count === 15) {
			/* the robot's 4-bit count is saturated and it sends no more HUMP
			   events: a new highest value, or a new last value, still proves
			   a new hump; an equal last height is invisible */
			if (r.hump_max !== pr.hump_max && r.hump_max !== 63) {
				ops.push(this.humpOp(s, base, { code: r.hump_max, count: null, max: r.hump_max, by: 'max_increase' }));
			} else if (r.hump_last !== pr.hump_last && r.hump_last !== 63) {
				ops.push(this.humpOp(s, base, { code: r.hump_last, count: null, max: r.hump_max, by: 'last_change' }));
			}
		} else if (r.hump_count < pr.hump_count) {
			ops.push(this.event(s, base, 'server', 'HUMP_COUNTER_RESET', 'warning',
				`Robot hump count went from ${pr.hump_count} to ${r.hump_count} without a detected restart`, {}));
		}
	}

	/* ------------------------------------------------------------------ */
	/*  Heartbeat, events, acknowledgements                                */

	onHeartbeat(s, d, buf, base, rxMs, ops) {
		const loc = this.locate(s, d.raw.uptime_ds, rxMs, base, ops);
		const gap = this.seqGap(s, 2, d.seq, rxMs, buf);

		if (gap.duplicate) {
			this.stats.duplicates++;
			return;
		}
		ops.push(op('heartbeat', Object.assign({}, base, {
			boot_id: s.boot.id, seq: d.seq, seq_gap: gap.gap, uptime_ds: d.raw.uptime_ds,
			uptime_s: loc.U / 10, mode: d.raw.mode, comm_state: d.raw.comm_state,
			critical: d.critical, fault_count: d.fault_count, reconnects: d.reconnects, frame: buf
		}), false));
		if (s.hb && d.reconnects > s.hb.reconnects) {
			ops.push(this.event(s, base, 'robot', 'ROBOT_RECONNECTED', 'info',
				`Robot reports reconnection #${d.reconnects}${d.reconnects_saturated ? '+' : ''} ` +
				'(its Wi-Fi/MQTT session was re-established)', { reconnects: d.reconnects }));
		}
		s.hb = { reconnects: d.reconnects, rxMs, comm_state: d.raw.comm_state, mode: d.raw.mode };
		s.lastRx.hb = rxMs;
	}

	onEvent(s, d, buf, base, rxMs, ops) {
		const gap = this.seqGap(s, 3, d.seq, rxMs, buf);
		let sev = 'info';
		let msg;

		if (gap.duplicate) {
			this.stats.duplicates++;
			return;
		}
		const data = Object.assign({ data_word: d.data }, d.fields);
		switch (d.event) {
		case 'MODE':
			msg = `Mode ${d.old_mode} -> ${d.new_mode} (epoch ${d.epoch})`;
			sev = (d.new_mode === 'SAFE_STOP') ? 'critical' : 'info';
			break;
		case 'FAULTS': {
			const crit = codec.criticalMask(this.decodeOpts.policy);
			const first = s.boot && !s.boot.faultsReported;
			data.faults = d.faults;
			data.since_boot_union = !!first;
			if (s.boot) { s.boot.faultsReported = true; }
			msg = (first ? 'Faults raised since boot (first report after connecting, not necessarily active): '
				     : 'Fault mask now: ') + (d.faults.length ? d.faults.join(', ') : 'none');
			sev = (d.fault_mask & crit) ? 'critical' : (d.fault_mask ? 'warning' : 'info');
			break;
		}
		case 'BARCODE':
			msg = `Barcode ${d.symbol === null ? '(no symbol)' : "'" + d.symbol + "'"} ${d.result}` +
				(d.nav ? `, mapped to ${d.nav} (mapping, not confirmed execution)` : ', no navigation mapping');
			sev = (d.result === 'INVALID') ? 'warning' : 'info';
			break;
		case 'HUMP':
			msg = `Hump measured: ${d.last_mm === null ? 'n/a' : d.last_mm + ' mm'} ` +
				`(highest since boot ${d.max_boot_mm === null ? 'n/a' : d.max_boot_mm + ' mm'}, ` +
				`robot count ${d.count}${d.count_saturated ? ', saturated' : ''})`;
			if (s.boot) {
				ops.push(this.humpOp(s, base, { code: d.fields.last, count: d.count, max: d.fields.max,
					by: 'robot_event' }));
			}
			break;
		case 'OBSTACLE':
			msg = `Avoidance ${d.outcome}` + ((d.fail !== 'NONE') ? ` (reason ${d.fail})` : '') +
				`, side ${d.side}, width ${d.width_mm === null ? 'unknown' : d.width_mm + ' mm'}` +
				`, distance ${d.distance_mm === null ? 'unknown' : d.distance_mm + ' mm'}`;
			sev = (d.outcome === 'FAILED') ? 'critical' : 'info';
			break;
		case 'COLLISION':
			msg = `Collision detected, peak shock ${d.shock_g} g`;
			sev = 'critical';
			break;
		default:
			msg = `Unknown event code ${d.code}`;
			sev = 'warning';
		}
		ops.push(this.event(s, base, 'robot', d.event, sev, msg, data, { seq: d.seq, frame: buf }));
		if (gap.gap > 0) {
			ops.push(this.event(s, base, 'server', 'EVENTS_MISSED', 'warning',
				`${gap.gap} robot event frame(s) not received before seq ${d.seq}`, { missing: gap.gap }));
		}
		s.lastRx.evt = rxMs;
	}

	onAck(s, d, buf, base, rxMs, ops) {
		const gap = this.seqGap(s, 5, d.seq, rxMs, buf);

		if (gap.duplicate) {
			this.stats.duplicates++;
			return;
		}
		ops.push(op('ack', Object.assign({}, base, {
			boot_id: s.boot ? s.boot.id : null, seq: d.seq, cmd_id: d.cmd_id, cmd: d.raw.cmd,
			result: d.raw.result, detail: d.raw.detail, frame: buf
		}), true));
		ops.push(this.event(s, base, 'robot', 'ACK', (d.result === 'OK') ? 'info' : 'warning',
			`ACK for ${d.cmd} id ${d.cmd_id}: ${d.result}` +
			((d.result === 'REJ_STATE') ? ` (${d.detail})` : '') + ' (accepted is not the same as done)',
			{ cmd: d.cmd, cmd_id: d.cmd_id, result: d.result, detail: d.detail }, { seq: d.seq }));
		s.lastRx.ack = rxMs;
	}

	/* ------------------------------------------------------------------ */
	/*  Time: online / stale / offline                                     */

	linkRow(s, nowMs) {
		const d = (ms) => (ms ? new Date(ms) : null);
		s.lastLinkWriteMs = nowMs;
		return op('robot_link', {
			robot_id: s.robotId, source: s.source, state: s.link.state,
			state_since: d(s.link.since), last_rx_at: d(Math.max(s.lastRx.hb || 0, s.lastRx.tlm || 0,
				s.lastRx.evt || 0, s.lastRx.ack || 0)),
			last_hb_at: d(s.lastRx.hb), last_tlm_at: d(s.lastRx.tlm),
			last_reconnects: s.hb ? s.hb.reconnects : null,
			last_comm_state: s.hb ? s.hb.comm_state : null,
			boot_id: s.boot ? s.boot.id : null, updated_at: new Date(nowMs)
		}, true, 'robot_link:' + s.robotId + '|' + s.source);
	}

	tick(nowMs) {
		const ops = [];

		for (const s of this.streams.values()) {
			const last = Math.max(s.lastRx.hb || 0, s.lastRx.tlm || 0);
			let state;

			if (!last) { continue; }
			const age = nowMs - last;
			if (this.brokerConnected === false) {
				state = 'UNKNOWN';
			} else {
				state = (age <= this.link.staleAfterMs) ? 'ONLINE'
					: (age <= this.link.offlineAfterMs) ? 'STALE' : 'OFFLINE';
			}
			if (state !== s.link.state) {
				const base = this.base(s, null, nowMs);
				const was = s.link.state;
				const away = s.link.since ? ((nowMs - s.link.since) / 1000).toFixed(1) : null;
				let msg;
				let sev = 'warning';
				if (state === 'ONLINE') {
					sev = 'info';
					msg = (was === 'NEVER_SEEN') ? 'Robot link ONLINE (first frames received)'
						: `Robot link ONLINE again (was ${was} for ${away} s)`;
				} else if (state === 'UNKNOWN') {
					msg = 'Backend is not connected to the broker: robot state unknown (backend problem, not a robot fault)';
				} else {
					msg = `Robot link ${state}: no telemetry or heartbeat for ${(age / 1000).toFixed(1)} s`;
				}
				s.link = { state, since: nowMs };
				ops.push(this.event(s, base, 'server', 'LINK_' + state, sev, msg,
					{ from: was, age_s: age / 1000 }));
				ops.push(this.linkRow(s, nowMs));
			} else if (nowMs - s.lastLinkWriteMs >= ROW_REFRESH_MS) {
				ops.push(this.linkRow(s, nowMs));
			}
		}
		return ops;
	}

	setBrokerConnected(connected, nowMs) {
		const ops = [];
		const was = this.brokerConnected;

		this.brokerConnected = !!connected;
		if (was !== null && was !== this.brokerConnected) {
			const s = this.stream('live');
			ops.push(this.event(s, this.base(s, null, nowMs), 'server',
				connected ? 'BACKEND_BROKER_CONNECTED' : 'BACKEND_BROKER_DISCONNECTED',
				connected ? 'info' : 'warning',
				connected ? 'Backend reconnected to the MQTT broker'
					  : 'Backend lost its MQTT broker connection (frames published meanwhile are not received)', {}));
		}
		return ops;
	}

	/* ------------------------------------------------------------------ */
	/*  Persistence across backend restarts                                */

	exportState() {
		const streams = [];
		for (const s of this.streams.values()) {
			streams.push({
				robotId: s.robotId, source: s.source, boot: s.boot, ref: s.ref,
				minOffsetMs: s.minOffsetMs, seq: s.seq, lastRx: s.lastRx,
				prev: s.prev, hb: s.hb, link: s.link
			});
		}
		return { version: 1, savedAt: Date.now(), streams };
	}

	importState(state) {
		if (!state || state.version !== 1 || !Array.isArray(state.streams)) { return 0; }
		for (const saved of state.streams) {
			if (saved.robotId !== this.robotId) { continue; }
			const s = this.stream(saved.source);
			Object.assign(s, {
				boot: saved.boot, ref: saved.ref, minOffsetMs: saved.minOffsetMs, seq: saved.seq || {},
				lastRx: saved.lastRx || {}, prev: saved.prev, hb: saved.hb,
				link: saved.link || { state: 'NEVER_SEEN', since: null }
			});
		}
		return state.streams.length;
	}
}

module.exports = { Ingestor, LATE_MS, LATE_TOL_DS, EARLY_TOL_DS };
