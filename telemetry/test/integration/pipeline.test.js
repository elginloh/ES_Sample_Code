/*
 * Synthetic MQTT -> the real Node-RED runtime (flows.json) -> PostgreSQL
 * -> Grafana, on isolated services (test/helpers/harness.js).  Topics use
 * the prefix it/csc14 on a private broker; the database is created and
 * dropped by the test.  Nothing is ever published on a command topic.
 */
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('fs');
const path = require('path');
const mqtt = require('mqtt');
const h = require('../helpers/harness');
const { codec } = require('../../lib/protocol');
const grafana = require('../helpers/grafana');
const dashboard = require('../../grafana/dashboards/csc14-robot.json');

const drained = (x) => x && x.buffer.bulk + x.buffer.priority + x.buffer.inflight === 0;
const hex = (s) => Buffer.from(s.replace(/\s/g, ''), 'hex');

test('pipeline on isolated services', { timeout: 900000 }, async (t) => {
	const dir = h.tmpDir();
	const broker = await h.startBroker(dir);
	const db = await h.createDatabase();
	const proxy = new h.Proxy({ host: db.cfg.pg.host, port: db.cfg.pg.port });
	await proxy.enable();
	const env = Object.assign({}, db.env, {
		CSC14_RUNTIME_DIR: path.join(dir, 'runtime'), CSC14_ROBOT_ID: 'it-car',
		CSC14_MQTT_HOST: '127.0.0.1', CSC14_MQTT_PORT: String(broker.port), CSC14_MQTT_CLIENT_ID: 'csc14-it-nodered',
		CSC14_TOPIC_LIVE: 'it/csc14', CSC14_TOPIC_SIM: 'it/sim/csc14',
		CSC14_PG_HOST: '127.0.0.1', CSC14_PG_PORT: String(proxy.port),
		CSC14_NODERED_HOST: '127.0.0.1', CSC14_NODERED_PORT: String(await h.freePort()),
		CSC14_NODERED_CREDENTIAL_SECRET: 'it', CSC14_STALE_AFTER_S: '2', CSC14_OFFLINE_AFTER_S: '4',
		CSC14_BUFFER_BULK_ROWS: '60', CSC14_BUFFER_PRIORITY_ROWS: '300', CSC14_STORE_RETRY_MAX_MS: '1000',
		CSC14_IR_SIDES_MIRRORED: '1'
	});
	fs.mkdirSync(path.join(env.CSC14_RUNTIME_DIR, 'nodered'), { recursive: true });
	const nr = new h.NodeRed(env);
	await nr.start();
	const client = mqtt.connect({ host: '127.0.0.1', port: broker.port, clientId: 'csc14-it-pub' });
	await new Promise((r) => client.on('connect', r));
	const live = new h.Sender(client, 'it/csc14', 120000);
	const q = async (sql, args) => (await db.admin.query(sql, args)).rows;
	const count = async (table, where) => Number((await q(`SELECT count(*) AS n FROM csc14.${table} ${where || ''}`))[0].n);

	try {
		await t.test('golden frames are routed to their tables; unavailable values are NULL', async () => {
			const pub = (ch, s) => new Promise((r) => client.publish('it/sim/csc14/' + ch, hex(s), {}, r));
			await pub('tlm', codec.GOLDEN.telemetry);
			await pub('hb', codec.GOLDEN.heartbeat);
			await pub('evt', codec.GOLDEN.event);
			await pub('ack', codec.GOLDEN.ack);
			const row = await h.eventually(async () => (await q(`SELECT * FROM csc14.telemetry WHERE source = 'simulated'`))[0], 8000, 'golden telemetry');
			assert.equal(row.speed_l_mms, null);
			assert.equal(row.speed_r_mms, 1023);
			assert.equal(row.pitch_deg, null);
			assert.equal(row.heading_imu_ccw_deg, null);
			assert.equal(row.us_dist_mm, null);
			assert.equal(row.us_no_echo, true);
			assert.equal(row.hump_last_mm, 40);
			assert.equal(row.clear_left_mm, 2540);
			assert.equal(row.fault_mask, 0x5A5A5A5);
			assert.equal(row.frame.toString('hex'), hex(codec.GOLDEN.telemetry).toString('hex'));
			assert.equal(row.decoded.barcode.symbol, 'A');
			await h.eventually(async () => await count('heartbeat', "WHERE source = 'simulated'") === 1, 5000, 'heartbeat');
			await h.eventually(async () => await count('event', "WHERE kind = 'HUMP'") === 1, 5000, 'HUMP event');
			const ack = (await q('SELECT cmd_id, cmd, result, detail FROM csc14.ack'))[0];
			assert.deepEqual([ack.cmd_id, ack.cmd, ack.result, ack.detail], [0x5A, 4, 6, 4]);
		});

		await t.test('malformed frames are rejected and never reach the measurement tables', async () => {
			const good = codec.encodeFrame(codec.TYPE.TELEMETRY, 1, live.r);
			const crc = Buffer.from(good); crc[9] ^= 0x40;
			const v2 = Buffer.from(good); v2[0] = 0x21;
			const range = codec.encodeFrame(codec.TYPE.TELEMETRY, 2, Object.assign({}, live.r, { avoid_state: 12 }));
			for (const f of [crc, good.subarray(0, 3), v2, range]) {
				await new Promise((r) => client.publish('it/csc14/tlm', f, {}, r));
			}
			const reasons = await h.eventually(async () => {
				const r = await q('SELECT reason FROM csc14.rejected_frame ORDER BY id');
				return r.length === 4 ? r.map((x) => x.reason) : null;
			}, 8000, 'rejected rows');
			assert.deepEqual(reasons, ['CRC', 'SHORT', 'CRC', 'FIELD_RANGE']);	/* v2 header breaks the CRC first */
			assert.equal(await count('telemetry', "WHERE source = 'live'"), 0);
		});

		let runId;
		await t.test('run boundaries: rows join the run active when they were received', async () => {
			for (let i = 0; i < 4; i++) { await live.tlm(); }
			await live.hb();
			await h.eventually(async () => await count('telemetry', "WHERE source = 'live'") === 4, 8000, 'pre-run rows');
			runId = (await q("SELECT csc14.start_run('it-car', 'live', 'it run', NULL, NULL, '{}', 'test') AS id"))[0].id;
			for (let i = 0; i < 6; i++) { await live.tlm({ bc_valid: 1 }); }
			await live.evt(3, { symbol: 10, result: 1, nav: 0, nav_valid: 0 });
			await h.eventually(async () => await count('telemetry', `WHERE run_id = '${runId}'`) === 6, 8000, 'run rows');
			await h.eventually(async () => await count('event', `WHERE run_id = '${runId}' AND kind = 'BARCODE'`) === 1, 5000);
			await q("SELECT csc14.end_run('it-car', 'live', 'done')");
			for (let i = 0; i < 3; i++) { await live.tlm(); }
			await h.eventually(async () => await count('telemetry', "WHERE source = 'live' AND run_id IS NULL") === 7, 8000, 'post-run rows');
			const s = (await q('SELECT * FROM csc14.run_summary($1)', [runId]))[0];
			assert.equal(Number(s.tlm_frames), 6);
			assert.equal(Number(s.barcode_reads_valid), 1);
		});

		await t.test('a Node-RED restart keeps the boot session and the active run; lost frames show as a gap', async () => {
			const run2 = (await q("SELECT csc14.start_run('it-car', 'live', 'restart run', NULL, NULL, '{}', 'test') AS id"))[0].id;
			for (let i = 0; i < 4; i++) { await live.tlm(); }
			await h.eventually(async () => await count('telemetry', `WHERE run_id = '${run2}'`) === 4, 8000);
			const boot = (await q(`SELECT DISTINCT boot_id FROM csc14.telemetry WHERE run_id = '${run2}'`))[0].boot_id;
			await nr.stop();
			await live.tlm(); await live.tlm();		/* nobody subscribed: lost (QoS 0) */
			await nr.start();
			const sessionsBefore = await count('event', "WHERE kind IN ('SESSION_START', 'BOOT_DETECTED') AND source = 'live'");
			for (let i = 0; i < 4; i++) { await live.tlm(); }
			await h.eventually(async () => await count('telemetry', `WHERE run_id = '${run2}'`) === 8, 8000, 'rows after restart');
			const after = await q(`SELECT boot_id, seq_gap FROM csc14.telemetry WHERE run_id = '${run2}' ORDER BY id`);
			assert.ok(after.every((r) => r.boot_id === boot), 'same boot session after the restart');
			assert.equal(after[4].seq_gap, 2, 'the two frames sent while Node-RED was down are counted as missing');
			assert.equal(await count('event', "WHERE kind IN ('SESSION_START', 'BOOT_DETECTED') AND source = 'live'"), sessionsBefore);
			await q("SELECT csc14.end_run('it-car', 'live', NULL)");
		});

		await t.test('a robot reboot opens a new boot session', async () => {
			live.reboot();
			await live.tlm();
			await h.eventually(async () => await count('event', "WHERE kind = 'BOOT_DETECTED' AND source = 'live'") === 1, 8000);
			assert.equal(await count('boot_session', "WHERE source = 'live' AND start_reason = 'robot_restart'"), 1);
		});

		await t.test('stale and offline are detected from missing frames, online again on recovery', async () => {
			await live.hb();
			await h.eventually(async () => (await q("SELECT state FROM csc14.robot_link WHERE source = 'live'"))[0]?.state === 'ONLINE', 8000, 'ONLINE');
			await h.eventually(async () => (await q("SELECT state FROM csc14.robot_link WHERE source = 'live'"))[0]?.state === 'OFFLINE', 12000, 'OFFLINE');
			const k = (await q("SELECT kind FROM csc14.event WHERE source = 'live' AND kind LIKE 'LINK_%' ORDER BY id")).map((r) => r.kind);
			assert.deepEqual(k.slice(-2), ['LINK_STALE', 'LINK_OFFLINE']);
			await live.hb();
			await live.tlm();
			await h.eventually(async () => (await q("SELECT state FROM csc14.robot_link WHERE source = 'live'"))[0]?.state === 'ONLINE', 8000, 'ONLINE again');
		});

		await t.test('database outage: rows are buffered and all written after recovery', async () => {
			const before = await count('telemetry', "WHERE source = 'live'");
			await proxy.disable();
			const t0 = Date.now();
			for (let i = 0; i < 20; i++) { await live.tlm(); if (i % 2 === 0) { await live.hb(); } await h.sleep(50); }
			const hd = await h.eventually(async () => {
				const x = await nr.health();
				return x && x.database.connected === false && x.buffer.bulk >= 20 ? x : null;
			}, 15000, 'buffering');
			assert.equal(hd.ok, false);
			assert.equal(await count('telemetry', "WHERE source = 'live'"), before, 'nothing written during the outage');
			await h.sleep(1500);
			await proxy.enable();
			await h.eventually(async () => await count('telemetry', "WHERE source = 'live'") === before + 20, 20000, 'rows after recovery');
			const lat = (await q(`SELECT max(extract(epoch FROM stored_at - received_at)) AS s FROM csc14.telemetry
				WHERE source = 'live' AND received_at >= to_timestamp($1)`, [t0 / 1000]))[0].s;
			assert.ok(Number(lat) >= 1.5, 'buffered rows were stored late');
			const hh = await nr.health();
			assert.equal(hh.store.droppedBulk, 0);
			t.diagnostic(`outage recovery: max received->stored ${Number(lat).toFixed(2)} s, buffered ${hd.buffer.bulk} rows`);
		});

		await t.test('buffer overflow drops the oldest rows and reports the loss', async () => {
			const before = await count('telemetry', "WHERE source = 'live'");
			await proxy.disable();
			for (let i = 0; i < 100; i++) { await live.tlm(); }
			const hd = await h.eventually(async () => {
				const x = await nr.health();
				return x && x.store.droppedBulk >= 40 ? x : null;
			}, 15000, 'drops');
			await proxy.enable();
			const ev = await h.eventually(async () => (await q("SELECT data FROM csc14.event WHERE kind = 'BACKEND_BUFFER_OVERFLOW'"))[0], 20000, 'overflow event');
			const added = await h.eventually(async () => {
				const n = await count('telemetry', "WHERE source = 'live'") - before;
				return (n > 0 && drained(await nr.health())) ? n : null;
			}, 20000);
			assert.ok(added <= 60 && added >= 40, `kept ${added} of 100 (buffer 60)`);
			assert.ok(ev.data.rows >= 40);
			t.diagnostic(`overflow: ${hd.store.droppedBulk} dropped, ${added} telemetry rows kept, reported ${ev.data.rows}`);
		});

		await t.test('an ACK updates the matching pending command', async () => {
			await q(`INSERT INTO csc14.command (robot_id, source, topic, cmd, arg, cmd_id, seq, frame)
				VALUES ('it-car', 'live', 'it/csc14/cmd', 'PING', 0, 77, 1, '\\x00')`);
			await live.ack(77, 0, 0);
			const c = await h.eventually(async () => (await q("SELECT status, ack_result FROM csc14.command WHERE cmd_id = 77 AND status <> 'pending'"))[0], 8000, 'ack applied');
			assert.deepEqual([c.status, c.ack_result], ['accepted', 'OK']);
		});

		await t.test('command tool: PING reaches a (fake) robot, its ACK marks the command accepted; STOP is refused', async () => {
			const { execFile } = require('child_process');
			const run = (args, extra) => new Promise((resolve) => execFile(process.execPath,
				[path.join(__dirname, '..', '..', 'tools', 'command.js')].concat(args),
				{ env: Object.assign({}, env, extra || {}) }, (err, stdout, stderr) =>
					resolve({ code: err ? err.code : 0, out: stdout + stderr })));
			/* fake robot on the PRIVATE broker: answers every command with ACK OK */
			const robot = mqtt.connect({ host: '127.0.0.1', port: broker.port, clientId: 'csc14-it-fake-robot' });
			await new Promise((r) => robot.on('connect', r));
			await new Promise((r) => robot.subscribe('it/csc14/cmd', r));
			robot.on('message', (topic, payload) => {
				const c = codec.decode(payload);
				if (!c.error && c.issues.length === 0) {
					robot.publish('it/csc14/ack', codec.encodeFrame(codec.TYPE.ACK, 1,
						{ cmd_id: c.cmd_id, cmd: c.raw.cmd, result: 0, detail: 0 }));
				}
			});
			try {
				const ping = await run(['ping']);
				assert.match(ping.out, /ACCEPTED: OK/, ping.out);
				assert.match(ping.out, /not the same as done/);
				const stop = await run(['stop'], { CSC14_ALLOW_MOTION_COMMANDS: '' });
				assert.equal(stop.code, 2);
				assert.match(stop.out, /disabled/);
				const rows = await q("SELECT cmd, status FROM csc14.command WHERE topic = 'it/csc14/cmd' ORDER BY id");
				assert.deepEqual(rows.map((r) => [r.cmd, r.status]).slice(-1), [['PING', 'accepted']]);
				assert.ok(!rows.some((r) => r.cmd === 'STOP'), 'a refused command is not recorded or sent');
			} finally {
				robot.end(true);
			}
		});

		await t.test('write latency in normal operation', async () => {
			const t0 = Date.now();
			for (let i = 0; i < 40; i++) { await live.tlm(); await h.sleep(25); }
			await h.eventually(async () => drained(await nr.health()), 8000);
			const r = (await q(`SELECT count(*) n, percentile_cont(0.5) WITHIN GROUP (ORDER BY extract(epoch FROM stored_at - received_at)) * 1000 AS p50,
				percentile_cont(0.95) WITHIN GROUP (ORDER BY extract(epoch FROM stored_at - received_at)) * 1000 AS p95,
				max(extract(epoch FROM stored_at - received_at)) * 1000 AS mx
				FROM csc14.telemetry WHERE source = 'live' AND received_at >= to_timestamp($1)`, [t0 / 1000]))[0];
			t.diagnostic(`received->committed over ${r.n} rows: p50 ${Number(r.p50).toFixed(1)} ms, p95 ${Number(r.p95).toFixed(1)} ms, max ${Number(r.mx).toFixed(1)} ms`);
			assert.ok(Number(r.p95) < 1000);
		});

		await t.test('Grafana: provisioned data source and dashboard; every panel query runs', async (tt) => {
			const g = await h.startGrafana(dir, db);
			if (!g) {
				tt.skip('Grafana is not installed (bin/csc14 setup)');
				return;
			}
			try {
				const ds = await (await fetch(g.url + '/api/datasources/uid/csc14-pg/health', { headers: { authorization: g.auth } })).json();
				assert.equal(ds.status, 'OK', JSON.stringify(ds));
				const d = await (await fetch(g.url + '/api/dashboards/uid/csc14-robot', { headers: { authorization: g.auth } })).json();
				assert.equal(d.dashboard.uid, 'csc14-robot');
				assert.equal(d.meta.provisioned, true);
				const res = await grafana.queryAll(g.url, g.auth, dashboard, { robot: 'it-car', source: 'live', intervalMs: 1000 },
					Date.now() - 30 * 60000, Date.now());
				const failed = res.filter((r) => r.status !== 200 || r.error);
				assert.deepEqual(failed, []);
				const runRes = await grafana.queryAll(g.url, g.auth, dashboard, { robot: 'it-car', source: 'live', run: runId,
					intervalMs: 1000 }, Date.now() - 30 * 60000, Date.now());
				assert.deepEqual(runRes.filter((r) => r.status !== 200 || r.error), []);
				const ms = res.map((r) => r.ms).sort((a, b) => a - b);
				tt.diagnostic(`${res.length} panel queries OK; median ${ms[ms.length >> 1].toFixed(0)} ms, max ${ms[ms.length - 1].toFixed(0)} ms`);
			} finally {
				await g.stop();
			}
		});
	} finally {
		client.end(true);
		await nr.stop();
		broker.stop();
		await proxy.disable().catch(() => {});
		await db.drop();
		fs.rmSync(dir, { recursive: true, force: true });
	}
});
