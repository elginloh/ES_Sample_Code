/*
 * Ingest logic (lib/ingest.js): sessions, wrap, gaps, saturation, derived
 * events, link states, persistence, separation of simulated data.
 */
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const { Ingestor } = require('../../lib/ingest');
const { codec } = require('../../lib/protocol');
const { VirtualRobot, baseRecord } = require('../../lib/sim');

const T = codec.TYPE;
const CFG = {
	robotId: 'car', topics: { live: 'csc14', sim: 'sim/csc14' },
	decode: { irSidesMirrored: true, policy: { collisionStops: true, imuRequired: true, usRequired: true } },
	link: { staleAfterMs: 3000, offlineAfterMs: 10000 }, store: { storeDecodedJson: true }
};
const T0 = Date.UTC(2026, 9, 30, 4, 0, 0);

/* A robot that booted at bootMs; tlm(t) sends a telemetry frame at t ms */
function robot(ing, bootMs, prefix) {
	const r = baseRecord();
	const seq = { 1: 0, 2: 0, 3: 0, 5: 0 };
	const topic = (ch) => (prefix || 'csc14') + '/' + ch;
	return {
		r,
		seq,
		tlm(t, over, opts) {
			const raw = Object.assign({}, r, { uptime_ds: Math.floor((t - bootMs) / 100) % 65536 }, over || {});
			const f = codec.encodeFrame(T.TELEMETRY, (opts && opts.seq !== undefined) ? opts.seq : seq[1], raw);
			seq[1] = ((opts && opts.seq !== undefined) ? opts.seq : seq[1]) + 1 & 0xFF;
			return ing.handle(topic('tlm'), f, (opts && opts.rx) || t);
		},
		hb(t, over) {
			const f = codec.encodeFrame(T.HEARTBEAT, seq[2]++ & 0xFF, Object.assign({ uptime_ds:
				Math.floor((t - bootMs) / 100) % 65536, mode: 1, comm_state: 4, critical: 0, fault_count: 0,
				reconnects: 0 }, over || {}));
			return ing.handle(topic('hb'), f, t);
		},
		evt(t, code, fields) {
			return ing.handle(topic('evt'), codec.encodeEvent(code, fields, seq[3]++ & 0xFF), t);
		}
	};
}
const ofTable = (res, table) => res.ops.filter((o) => o.table === table).map((o) => o.row);
const kinds = (res) => ofTable(res, 'event').map((e) => e.kind);

test('topics: live, simulated and foreign topics', () => {
	const ing = new Ingestor(CFG);
	assert.deepEqual(ing.classify('csc14/tlm'), { source: 'live', channel: 'tlm' });
	assert.deepEqual(ing.classify('sim/csc14/hb'), { source: 'simulated', channel: 'hb' });
	assert.equal(ing.classify('other/tlm'), null);
	assert.equal(ing.classify('csc14/a/b'), null);
	assert.equal(ing.handle('other/tlm', Buffer.alloc(3), T0).kind, 'ignored');
	const sim = robot(ing, T0, 'sim/csc14').tlm(T0 + 1000);
	assert.equal(ofTable(sim, 'telemetry')[0].source, 'simulated');
});

test('first frame opens a session; telemetry rows carry columns, frame and decoded record', () => {
	const ing = new Ingestor(CFG);
	const b = robot(ing, T0 - 12345 * 100);
	const res = b.tlm(T0, { speed_l: -1024 });
	assert.deepEqual(kinds(res), ['SESSION_START']);
	const row = ofTable(res, 'telemetry')[0];
	assert.equal(row.uptime_ds, 12345);
	assert.equal(row.uptime_s, 1234.5);
	assert.equal(row.speed_l_mms, null);
	assert.equal(row.seq_gap, null);			/* unknown for the first frame */
	assert.equal(row.frame.length, 40);
	assert.equal(row.decoded.motion.speed_l_mms, null);
	assert.ok(row.boot_id);
	assert.equal(ofTable(res, 'boot_session')[0].start_reason, 'first_seen');
	const noJson = new Ingestor(Object.assign({}, CFG, { store: { storeDecodedJson: false } }));
	assert.equal(ofTable(robot(noJson, T0).tlm(T0 + 500), 'telemetry')[0].decoded, null);
});

test('uptime wrap (16-bit, 6553.6 s) stays in one session and is unwrapped', () => {
	const ing = new Ingestor(CFG);
	const boot = T0 - 6550 * 1000;			/* 3.6 s before the wrap */
	const b = robot(ing, boot);
	const ids = new Set();
	let last;
	for (let t = T0; t < T0 + 10000; t += 500) {
		last = ofTable(b.tlm(t), 'telemetry')[0];
		ids.add(last.boot_id);
	}
	assert.equal(ids.size, 1);
	assert.ok(last.uptime_ds < 100);			/* raw value wrapped */
	assert.ok(last.uptime_s > 6555 && last.uptime_s < 6560);
});

test('a reboot (uptime restarts) opens a new session; a backend restart with saved state does not', () => {
	const ing = new Ingestor(CFG);
	const b = robot(ing, T0 - 600000);
	const first = ofTable(b.tlm(T0), 'telemetry')[0].boot_id;
	b.tlm(T0 + 500);
	/* the robot reboots and is back 12 s later with uptime 9 s */
	const b2 = robot(ing, T0 + 3000);
	const res = b2.tlm(T0 + 12000);
	assert.ok(kinds(res).includes('BOOT_DETECTED'));
	assert.equal(ofTable(res, 'boot_session')[0].start_reason, 'robot_restart');
	const second = ofTable(res, 'telemetry')[0].boot_id;
	assert.notEqual(second, first);
	/* persistence: a fresh Ingestor with the saved state continues the session */
	const again = new Ingestor(CFG);
	again.importState(JSON.parse(JSON.stringify(ing.exportState())));
	b2.seq[1] = ing.streams.get('car|live').seq[1].seq + 1;
	const cont = robot(again, T0 + 3000);
	cont.seq[1] = b2.seq[1];
	const r3 = cont.tlm(T0 + 30000);
	assert.deepEqual(kinds(r3).filter((k) => k === 'BOOT_DETECTED' || k === 'SESSION_START'), []);
	assert.equal(ofTable(r3, 'telemetry')[0].boot_id, second);
	assert.equal(ofTable(r3, 'telemetry')[0].seq_gap, 0);	/* sequence continues across the restart */
});

test('frames queued during an outage arrive late without starting a new session', () => {
	const ing = new Ingestor(CFG);
	const b = robot(ing, T0);
	for (let t = T0 + 1000; t <= T0 + 5000; t += 500) { b.tlm(t); }
	/* 20 s outage; the robot keeps its newest 4 frames (made in the last 2 s)
	   and sends them right after a fresh heartbeat */
	b.hb(T0 + 25200);
	const late = [];
	for (let k = 0; k < 4; k++) {
		const made = T0 + 23500 + k * 500;
		late.push(ofTable(b.tlm(made, {}, { rx: T0 + 25250 + k * 5, seq: (b.seq[1] + 36) & 0xFF }), 'telemetry')[0]);
		b.seq[1] = (b.seq[1] - 36) & 0xFF;
	}
	assert.equal(new Set(late.map((r) => r.boot_id)).size, 1);
	assert.equal(late[0].late, true);
	assert.ok(late[0].rx_delay_ms > 1500);
	assert.equal(late[3].late, false);
});

test('sequence numbers: gaps, wrap, duplicates and ambiguous long gaps', () => {
	const ing = new Ingestor(CFG);
	const b = robot(ing, T0);
	b.tlm(T0 + 500, {}, { seq: 253 });
	assert.equal(ofTable(b.tlm(T0 + 1000, {}, { seq: 254 }), 'telemetry')[0].seq_gap, 0);
	assert.equal(ofTable(b.tlm(T0 + 1500, {}, { seq: 255 }), 'telemetry')[0].seq_gap, 0);
	assert.equal(ofTable(b.tlm(T0 + 2000, {}, { seq: 0 }), 'telemetry')[0].seq_gap, 0);	/* 255 -> 0 */
	const gap = b.tlm(T0 + 4000, {}, { seq: 4 });
	assert.equal(ofTable(gap, 'telemetry')[0].seq_gap, 3);
	assert.ok(kinds(gap).includes('TELEMETRY_GAP'));
	/* identical frame again (same seq and CRC) is a duplicate, not stored */
	const raw = Object.assign({}, b.r, { uptime_ds: 40 });
	const f = codec.encodeFrame(T.TELEMETRY, 9, raw);
	ing.handle('csc14/tlm', f, T0 + 4500);
	const dup = ing.handle('csc14/tlm', f, T0 + 4510);
	assert.equal(ofTable(dup, 'telemetry').length, 0);
	assert.equal(ing.stats.duplicates, 1);
	/* 2 minutes later: more than 128 frames could have been missed */
	const far = b.tlm(T0 + 125000, {}, { seq: 20 });
	assert.equal(ofTable(far, 'telemetry')[0].seq_gap, null);
	const e = ofTable(far, 'event').find((x) => x.kind === 'TELEMETRY_GAP');
	assert.equal(e.data.exact, false);
	assert.ok(e.data.missing >= 238);
});

test('humps: baseline at first sight, increments, several between snapshots, saturation', () => {
	const ing = new Ingestor(CFG);
	const b = robot(ing, T0);
	/* first snapshot already shows 3 humps: they are history, not new */
	assert.equal(ofTable(b.tlm(T0 + 500, { hump_count: 3, hump_last: 10, hump_max: 20 }), 'hump').length, 0);
	const one = ofTable(b.tlm(T0 + 1000, { hump_count: 4, hump_last: 15, hump_max: 20 }), 'hump');
	assert.deepEqual(one.map((h) => [h.height_mm, h.count_after, h.detected_by]), [[30, 4, 'telemetry']]);
	const two = b.tlm(T0 + 1500, { hump_count: 6, hump_last: 12, hump_max: 20 });
	assert.deepEqual(ofTable(two, 'hump').map((h) => [h.height_mm, h.count_after]), [[null, 5], [24, 6]]);
	assert.ok(kinds(two).includes('HUMP_HEIGHT_UNKNOWN'));
	/* robot event for the same hump becomes the same row (boot, count) */
	const ev = b.evt(T0 + 1510, 4, { last: 12, max: 20, count: 6 });
	assert.deepEqual(ofTable(ev, 'hump').map((h) => [h.count_after, h.detected_by]), [[6, 'robot_event']]);
	/* saturated counter: only a new maximum or a new last value is visible */
	b.tlm(T0 + 2000, { hump_count: 15, hump_last: 12, hump_max: 20 });
	assert.equal(ofTable(b.tlm(T0 + 2500, { hump_count: 15, hump_last: 12, hump_max: 20 }), 'hump').length, 0);
	assert.deepEqual(ofTable(b.tlm(T0 + 3000, { hump_count: 15, hump_last: 25, hump_max: 25 }), 'hump')
		.map((h) => [h.height_mm, h.count_after, h.detected_by]), [[50, null, 'max_increase']]);
	assert.deepEqual(ofTable(b.tlm(T0 + 3500, { hump_count: 15, hump_last: 9, hump_max: 25 }), 'hump')
		.map((h) => [h.height_mm, h.detected_by]), [[18, 'last_change']]);
});

test('derived events: faults per bit, tilt, line state, avoidance failure, comm state', () => {
	const ing = new Ingestor(CFG);
	const b = robot(ing, T0);
	b.tlm(T0 + 500, { line_state: 0 });
	const r = b.tlm(T0 + 1000, { faults: 2 ** 6 + 2 ** 12, tilt: 1, line_state: 1, avoid_outcome: 2,
		avoid_state: 8, comm_state: 5 });
	const ev = ofTable(r, 'event');
	const set = ev.filter((e) => e.kind === 'FAULT_SET');
	assert.deepEqual(set.map((e) => [e.data.fault, e.severity]), [['LINE_LOST', 'warning'], ['COLLISION', 'critical']]);
	assert.ok(ev.some((e) => e.kind === 'TILT_ON'));
	assert.ok(ev.some((e) => e.kind === 'LINE_STATE' && e.data.to === 'SEARCHING'));
	assert.ok(ev.some((e) => e.kind === 'AVOID_FAILED' && e.severity === 'critical'));
	assert.ok(ev.some((e) => e.kind === 'ROBOT_COMM_STATE' && e.data.to === 'RETRY_WAIT'));
	const clr = ofTable(b.tlm(T0 + 1500, { faults: 2 ** 12, line_state: 0 }), 'event');
	assert.deepEqual(clr.filter((e) => e.kind === 'FAULT_CLEARED').map((e) => e.data.fault), ['LINE_LOST']);
});

test('the first FAULTS event of a boot is labelled as the since-boot union', () => {
	const ing = new Ingestor(CFG);
	const b = robot(ing, T0);
	b.tlm(T0 + 500);
	const first = ofTable(b.evt(T0 + 510, 2, { fault_mask: 2 ** 20 }), 'event')[0];
	assert.equal(first.data.since_boot_union, true);
	assert.match(first.message, /since boot/);
	const next = ofTable(b.evt(T0 + 1000, 2, { fault_mask: 0 }), 'event')[0];
	assert.equal(next.data.since_boot_union, false);
});

test('link: online -> stale -> offline -> online, and UNKNOWN when the broker is lost', () => {
	const ing = new Ingestor(CFG);
	const b = robot(ing, T0);
	b.hb(T0 + 1000);
	const k = (t) => ing.tick(t).filter((o) => o.table === 'event').map((o) => o.row.kind);
	assert.deepEqual(k(T0 + 1500), ['LINK_ONLINE']);
	assert.deepEqual(k(T0 + 3500), []);
	assert.deepEqual(k(T0 + 4500), ['LINK_STALE']);
	assert.deepEqual(k(T0 + 9000), []);
	assert.deepEqual(k(T0 + 11500), ['LINK_OFFLINE']);
	b.hb(T0 + 20000);
	const back = ing.tick(T0 + 20100).find((o) => o.table === 'event').row;
	assert.equal(back.kind, 'LINK_ONLINE');
	assert.match(back.message, /OFFLINE for/);
	assert.equal(ing.tick(T0 + 20200).length, 0);
	ing.setBrokerConnected(true, T0 + 20300);
	const lost = ing.setBrokerConnected(false, T0 + 20400);
	assert.equal(lost[0].row.kind, 'BACKEND_BROKER_DISCONNECTED');
	assert.deepEqual(k(T0 + 20500), ['LINK_UNKNOWN']);
	assert.equal(ing.tick(T0 + 26000).filter((o) => o.table === 'robot_link').length, 1);	/* 5 s refresh */
});

test('malformed frames are rejected (and their storage rate-limited); nothing reaches telemetry', () => {
	const ing = new Ingestor(CFG);
	const good = codec.encodeFrame(T.TELEMETRY, 1, baseRecord());
	const bad = Buffer.from(good); bad[10] ^= 0xFF;
	const r = ing.handle('csc14/tlm', bad, T0);
	assert.equal(r.kind, 'rejected');
	assert.equal(r.reason, 'CRC');
	assert.deepEqual(r.ops.map((o) => o.table), ['rejected_frame']);
	const range = ing.handle('csc14/tlm', codec.encodeFrame(T.TELEMETRY, 2, Object.assign(baseRecord(), { mode: 6 })), T0);
	assert.equal(range.reason, 'FIELD_RANGE');
	assert.equal(range.ops[0].row.detail, 'mode=6');
	for (let i = 0; i < 40; i++) { ing.handle('csc14/tlm', Buffer.from('garbage'), T0 + i); }
	assert.equal(ing.stats.rejected, 42);
	assert.equal(ing.stats.rejectedNotStored, 12);		/* 30 stored per minute */
	assert.equal(ing.streams.get('car|live').prev, null);
});

test('a command frame seen on an observed topic is logged as a bus event; ACKs are routed', () => {
	const ing = new Ingestor(CFG);
	const c = ing.handle('csc14/cmd', codec.encodeCommand('PING', 0, 7, 1), T0);
	assert.equal(c.kind, 'command');
	assert.equal(ofTable(c, 'event')[0].origin, 'bus');
	const a = ing.handle('csc14/ack', codec.encodeFrame(T.ACK, 1, { cmd_id: 7, cmd: 0, result: 0, detail: 0 }), T0 + 50);
	assert.deepEqual(ofTable(a, 'ack').map((x) => [x.cmd_id, x.result]), [[7, 0]]);
	assert.match(ofTable(a, 'event')[0].message, /not the same as done/);
});

test('virtual robot scenario end to end through the ingestor (130 s, outage, collision)', () => {
	const ing = new Ingestor(CFG);
	const vr = new VirtualRobot({ bootMs: T0, seed: 3 });
	const all = [];
	for (let i = 0; i < 260; i++) {
		const t = T0 + 500 + i * 500;
		vr.setOnline(!(t >= T0 + 85000 && t < T0 + 93000));
		for (const f of vr.cycle(t, { heartbeat: i % 2 === 0 })) {
			all.push(...ing.handle('sim/csc14/' + f.channel, f.frame, t + 20).ops);
		}
		all.push(...ing.tick(t + 30));
	}
	const ev = all.filter((o) => o.table === 'event').map((o) => o.row);
	const count = (k) => ev.filter((e) => e.kind === k).length;
	assert.equal(count('SESSION_START'), 1);
	assert.equal(count('BOOT_DETECTED'), 0);
	assert.equal(count('HUMP'), 3);			/* 42, 56, 30 mm (the 2nd cycle starts at 120 s) */
	assert.equal(count('OBSTACLE'), 1);
	assert.equal(count('COLLISION'), 1);
	assert.equal(count('ROBOT_RECONNECTED'), 1);
	assert.ok(count('LINK_STALE') >= 1);
	const humps = all.filter((o) => o.table === 'hump').map((o) => o.row.height_mm);
	assert.deepEqual([...new Set(humps)].sort(), [30, 42, 56]);
	const rows = all.filter((o) => o.table === 'telemetry').map((o) => o.row);
	assert.ok(rows.some((r) => r.late), 'frames queued during the outage arrive late');
	assert.ok(rows.some((r) => r.comm_state === 5), 'queued frames show RETRY_WAIT');
	assert.equal(ing.stats.rejected, 0);
});
