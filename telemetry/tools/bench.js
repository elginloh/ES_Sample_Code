#!/usr/bin/env node
/*
 * bench.js -- reproducible backend measurements on ISOLATED services (own
 * broker, database, Node-RED and Grafana; the live ones are not touched).
 *
 *   bin/csc14 bench [--normal-s 120] [--out FILE]
 *
 * Phases: idle, normal (virtual robot at the real 2 Hz), dashboard polling
 * (every panel query every 5 s), stress (20 Hz telemetry), database outage,
 * broker outage.  Reports latency (publish -> received by Node-RED,
 * received -> row committed), CPU and memory of each service, storage per
 * row and per hour, dashboard query times.  All numbers are MEASURED on
 * this computer with simulated frames; they say nothing about the Pico's
 * Wi-Fi or its control loops.
 */
'use strict';

const { execFileSync } = require('child_process');
const fs = require('fs');
const os = require('os');
const path = require('path');
const mqtt = require('mqtt');
const h = require('../test/helpers/harness');
const grafana = require('../test/helpers/grafana');
const dashboard = require('../grafana/dashboards/csc14-robot.json');
const { VirtualRobot } = require('../lib/sim');
const { codec } = require('../lib/protocol');

function arg(name, dflt) {
	const i = process.argv.indexOf('--' + name);
	return (i < 0) ? dflt : process.argv[i + 1];
}
const NORMAL_S = Number(arg('normal-s', 120));
const OUT = arg('out', path.join(__dirname, '..', '..', 'testing', 'system', 'logs',
	new Date().toISOString().slice(0, 10) + '_telemetry_backend_bench.md'));

/* cumulative CPU seconds and RSS (MB) of a set of pids */
function psSample(pids) {
	if (!pids.length) { return { cpu: 0, rss: 0 }; }
	let out = '';
	try { out = execFileSync('ps', ['-o', 'time=,rss=', '-p', pids.join(',')], { encoding: 'utf8' }); } catch (e) { out = e.stdout || ''; }
	let cpu = 0;
	let rss = 0;
	for (const line of out.trim().split('\n').filter(Boolean)) {
		const [t, r] = line.trim().split(/\s+/);	/* [days-][hh:]mm:ss.ss */
		const [d, hms] = t.includes('-') ? t.split('-') : ['0', t];
		let s = 0;
		for (const p of hms.split(':').map(Number)) { s = s * 60 + p; }
		cpu += Number(d) * 86400 + s;
		rss += Number(r) / 1024;
	}
	return { cpu, rss };
}
/* postmaster and its children: on macOS the children rename themselves
   ("postgres: walwriter", backends), so pgrep -x only finds the postmaster */
const pgPids = () => {
	const list = (a) => {
		try { return execFileSync('pgrep', a, { encoding: 'utf8' }).trim().split('\n').filter(Boolean); }
		catch (e) { return []; }
	};
	const parents = list(['-x', 'postgres']);
	return [...new Set(parents.concat(...parents.map((p) => list(['-P', p]))))];
};

class Meter {
	constructor(groups) { this.groups = groups; }
	start() {
		this.t0 = Date.now();
		this.a = {};
		this.peak = {};
		for (const [k, f] of Object.entries(this.groups)) { this.a[k] = psSample(f()); this.peak[k] = this.a[k].rss; }
		this.timer = setInterval(() => {
			for (const [k, f] of Object.entries(this.groups)) { this.peak[k] = Math.max(this.peak[k], psSample(f()).rss); }
		}, 1000);
	}
	stop() {
		clearInterval(this.timer);
		const wall = (Date.now() - this.t0) / 1000;
		const res = {};
		for (const [k, f] of Object.entries(this.groups)) {
			const b = psSample(f());
			res[k] = { cpu_pct: Math.max(0, 100 * (b.cpu - this.a[k].cpu) / wall), rss_mb: b.rss, peak_rss_mb: this.peak[k] };
		}
		return res;
	}
}

const pct = (arr, p) => {
	if (!arr.length) { return null; }
	const s = arr.slice().sort((x, y) => x - y);
	return s[Math.min(s.length - 1, Math.floor(p * (s.length - 1) + 0.5))];
};
const drained = (x) => x && x.buffer.bulk + x.buffer.priority + x.buffer.inflight === 0;
const f1 = (v) => (v === null || v === undefined) ? '-' : Number(v).toFixed(1);

async function main() {
	const dir = h.tmpDir();
	const broker = await h.startBroker(dir);
	const db = await h.createDatabase();
	const proxy = new h.Proxy({ host: db.cfg.pg.host, port: db.cfg.pg.port });
	await proxy.enable();
	const env = Object.assign({}, db.env, {
		CSC14_RUNTIME_DIR: path.join(dir, 'runtime'), CSC14_ROBOT_ID: 'bench-car',
		CSC14_MQTT_HOST: '127.0.0.1', CSC14_MQTT_PORT: String(broker.port), CSC14_MQTT_CLIENT_ID: 'csc14-bench-nodered',
		CSC14_TOPIC_LIVE: 'bench/csc14', CSC14_TOPIC_SIM: 'bench/sim/csc14',
		CSC14_PG_HOST: '127.0.0.1', CSC14_PG_PORT: String(proxy.port),
		CSC14_NODERED_HOST: '127.0.0.1', CSC14_NODERED_PORT: String(await h.freePort()),
		CSC14_NODERED_CREDENTIAL_SECRET: 'bench'
	});
	fs.mkdirSync(path.join(env.CSC14_RUNTIME_DIR, 'nodered'), { recursive: true });
	const nr = new h.NodeRed(env);
	await nr.start();
	const g = await h.startGrafana(dir, db);
	const report = { when: new Date().toISOString(), host: `${os.type()} ${os.release()} ${os.arch()}, ` +
		`${os.cpus().length} CPUs, ${(os.totalmem() / 2 ** 30).toFixed(0)} GB`, node: process.version, phases: {} };
	const meter = new Meter({
		'Node-RED': () => [String(nr.proc.pid)],
		PostgreSQL: pgPids,
		Mosquitto: () => [String(broker.proc.pid)],
		Grafana: () => (g ? [String(g.proc.pid)] : [])
	});
	/* like the Pico: QoS 0 and nothing queued while disconnected */
	const client = mqtt.connect({ host: '127.0.0.1', port: broker.port, clientId: 'csc14-bench-pub',
		queueQoSZero: false, reconnectPeriod: 500 });
	await new Promise((r) => client.on('connect', r));
	const q = async (sql, a) => (await db.admin.query(sql, a)).rows;
	const pub = new Map();				/* "seq:uptime" -> publish time */
	let robot = new VirtualRobot({ bootMs: Date.now() - 5000, seed: 11 });
	let cycle = 0;
	let published = 0;
	let bytes = 0;

	function sendCycle(extraTlm) {
		const now = Date.now();
		const frames = robot.cycle(now, { heartbeat: cycle % 2 === 0 });
		cycle++;
		for (const f of frames) {
			if (f.channel === 'tlm') {
				const d = codec.decode(f.frame);
				pub.set(d.seq + ':' + d.raw.uptime_ds, now);
			}
			client.publish('bench/sim/csc14/' + f.channel, f.frame, { qos: 0 });
			published++;
			bytes += f.frame.length;
		}
		for (let i = 0; i < (extraTlm || 0); i++) {
			const f = robot.frame(codec.TYPE.TELEMETRY, Object.assign({}, robot.r, { uptime_ds: robot.uptimeDs(Date.now()) }));
			const d = codec.decode(f);
			pub.set(d.seq + ':' + d.raw.uptime_ds, Date.now());
			client.publish('bench/sim/csc14/tlm', f, { qos: 0 });
			published++;
			bytes += f.length;
		}
	}

	async function runFor(seconds, periodMs, extraTlm, during) {
		const until = Date.now() + seconds * 1000;
		while (Date.now() < until) {
			const t = Date.now();
			sendCycle(extraTlm);
			if (during) { await during(); }
			await h.sleep(Math.max(0, periodMs - (Date.now() - t)));
		}
	}

	async function latency(fromMs) {
		await h.eventually(async () => drained(await nr.health()), 30000, 'drain');
		await h.sleep(500);
		const rows = await q(`SELECT seq, uptime_ds, extract(epoch FROM received_at) * 1000 AS rx,
			extract(epoch FROM stored_at - received_at) * 1000 AS commit_ms
			FROM csc14.telemetry WHERE source = 'simulated' AND received_at >= to_timestamp($1)`, [fromMs / 1000]);
		const rxLat = [];
		for (const r of rows) {
			const p = pub.get(r.seq + ':' + r.uptime_ds);
			if (p) { rxLat.push(Number(r.rx) - p); }
		}
		const c = rows.map((r) => Number(r.commit_ms));
		return { rows: rows.length, pub_to_rx_ms: { p50: pct(rxLat, 0.5), p95: pct(rxLat, 0.95), max: pct(rxLat, 1) },
			 rx_to_commit_ms: { p50: pct(c, 0.5), p95: pct(c, 0.95), max: pct(c, 1) } };
	}

	try {
		/* idle */
		process.stdout.write('idle 20 s ... ');
		meter.start();
		await h.sleep(20000);
		report.phases.idle = { resources: meter.stop() };
		console.log('done');

		/* normal 2 Hz */
		process.stdout.write(`normal 2 Hz for ${NORMAL_S} s ... `);
		const size0 = (await q("SELECT pg_total_relation_size('csc14.telemetry') + pg_total_relation_size('csc14.heartbeat') + pg_total_relation_size('csc14.event') AS b"))[0].b;
		const t0 = Date.now();
		const p0 = published;
		const b0 = bytes;
		meter.start();
		await runFor(NORMAL_S, 500, 0);
		const res = meter.stop();
		const lat = await latency(t0);
		const counts = (await q(`SELECT (SELECT count(*) FROM csc14.telemetry WHERE received_at >= to_timestamp($1)) t,
			(SELECT count(*) FROM csc14.heartbeat WHERE received_at >= to_timestamp($1)) hb,
			(SELECT count(*) FROM csc14.event WHERE received_at >= to_timestamp($1)) ev`, [t0 / 1000]))[0];
		const size1 = (await q("SELECT pg_total_relation_size('csc14.telemetry') + pg_total_relation_size('csc14.heartbeat') + pg_total_relation_size('csc14.event') AS b"))[0].b;
		const rowSize = (await q(`SELECT avg(pg_column_size(t.*)) AS row_b, avg(pg_column_size(t.frame)) AS frame_b,
			avg(coalesce(pg_column_size(t.decoded), 0)) AS json_b FROM csc14.telemetry t`))[0];
		report.phases.normal = { seconds: NORMAL_S, frames_published: published - p0, payload_bytes: bytes - b0,
			rows: counts, latency: lat, resources: res,
			storage: { bytes_added: Number(size1) - Number(size0), row_bytes: Number(rowSize.row_b),
				frame_bytes: Number(rowSize.frame_b), decoded_json_bytes: Number(rowSize.json_b) } };
		console.log('done');

		/* dashboard polling: every panel query every 5 s */
		if (g) {
			process.stdout.write('dashboard polling 60 s (+ robot at 2 Hz) ... ');
			const times = [];
			let failed = 0;
			meter.start();
			const until = Date.now() + 60000;
			let next = Date.now();
			await runFor(60, 500, 0, async () => {
				if (Date.now() >= next && Date.now() < until) {
					next += 5000;
					const r = await grafana.queryAll(g.url, g.auth, dashboard, { robot: 'bench-car', source: 'simulated',
						intervalMs: 1000 }, Date.now() - 15 * 60000, Date.now());
					for (const x of r) { times.push(x.ms); if (x.status !== 200 || x.error) { failed++; } }
				}
			});
			const wide = await grafana.queryAll(g.url, g.auth, dashboard, { robot: 'bench-car', source: 'simulated',
				intervalMs: 20000 }, Date.now() - 6 * 3600000, Date.now());
			report.phases.dashboard = { resources: meter.stop(), queries: times.length, failed,
				query_ms: { p50: pct(times, 0.5), p95: pct(times, 0.95), max: pct(times, 1) },
				six_hour_range_ms: { p50: pct(wide.map((x) => x.ms), 0.5), max: pct(wide.map((x) => x.ms), 1),
					failed: wide.filter((x) => x.status !== 200 || x.error).length } };
			console.log('done');
		}

		/* stress: 20 telemetry frames per second (10x) */
		process.stdout.write('stress 20 Hz telemetry for 30 s ... ');
		const s0 = Date.now();
		const sp = published;
		meter.start();
		await runFor(30, 500, 9);
		const sres = meter.stop();
		report.phases.stress = { telemetry_per_s: 20, frames_published: published - sp, latency: await latency(s0), resources: sres };
		console.log('done');

		/* database outage at 2 Hz */
		process.stdout.write('database outage 30 s ... ');
		await proxy.disable();
		const o0 = Date.now();
		await runFor(30, 500, 0);
		const peak = (await nr.health()).buffer;
		await proxy.enable();
		const back = Date.now();
		await h.eventually(async () => drained(await nr.health()), 60000, 'drain');
		const drainMs = Date.now() - back;
		const lost = (await nr.health()).store;
		const stored = (await q("SELECT count(*) n FROM csc14.telemetry WHERE received_at >= to_timestamp($1) AND received_at < to_timestamp($2)",
			[o0 / 1000, back / 1000]))[0].n;
		report.phases.db_outage = { seconds: 30, buffered_peak: peak, drain_ms: drainMs,
			telemetry_rows_kept: Number(stored), dropped_bulk: lost.droppedBulk, dropped_priority: lost.droppedPriority };
		console.log('done');

		/* broker outage at 2 Hz: frames published while it is down are lost */
		process.stdout.write('broker outage 15 s ... ');
		const before = Number((await q("SELECT count(*) n FROM csc14.telemetry"))[0].n);
		const sent0 = published;
		broker.proc.kill('SIGTERM');
		const k0 = Date.now();
		let detectMs = null;
		await runFor(15, 500, 0, async () => {
			if (detectMs === null) {
				const x = await nr.health();
				if (x && x.broker.connected === false) { detectMs = Date.now() - k0; }
			}
		});
		await broker.restart();
		const r0 = Date.now();
		await h.eventually(async () => (await nr.health()).broker.connected === true, 30000, 'Node-RED reconnect');
		const reconnectMs = Date.now() - r0;
		await new Promise((r) => { if (client.connected) { r(); } else { client.once('connect', r); } });
		await runFor(5, 500, 0);
		await h.sleep(1500);
		const after = Number((await q("SELECT count(*) n FROM csc14.telemetry"))[0].n);
		const unk = Number((await q("SELECT count(*) n FROM csc14.event WHERE kind = 'LINK_UNKNOWN'"))[0].n);
		report.phases.broker_outage = { seconds: 15, nodered_detect_ms: detectMs, nodered_reconnect_ms: reconnectMs,
			frames_published: published - sent0, telemetry_rows_added: after - before, link_unknown_events: unk };
		console.log('done');
	} finally {
		client.end(true);
		await nr.stop();
		if (g) { await g.stop(); }
		broker.stop();
		await proxy.disable().catch(() => {});
		await db.drop();
		fs.rmSync(dir, { recursive: true, force: true });
	}

	fs.mkdirSync(path.dirname(OUT), { recursive: true });
	fs.writeFileSync(OUT.replace(/\.md$/, '.json'), JSON.stringify(report, null, 2));
	fs.writeFileSync(OUT, markdown(report));
	console.log('\n' + markdown(report));
	console.log('written ' + OUT);
}

function markdown(r) {
	const L = [];
	const res = (x) => Object.entries(x).map(([k, v]) => `| ${k} | ${f1(v.cpu_pct)} % | ${f1(v.rss_mb)} MB | ${f1(v.peak_rss_mb)} MB |`);
	L.push(`# Telemetry backend measurements (${r.when})`, '',
		`Host: ${r.host}, Node.js ${r.node}. Isolated services, simulated frames (lib/sim.js).`,
		'MEASURED on this computer; not evidence about the Pico, its Wi-Fi or its control loops.', '');
	for (const [name, p] of Object.entries(r.phases)) {
		L.push(`## ${name}`, '');
		if (p.latency) {
			L.push(`Rows: ${p.latency.rows}. Publish -> received by Node-RED: p50 ${f1(p.latency.pub_to_rx_ms.p50)} ms, ` +
				`p95 ${f1(p.latency.pub_to_rx_ms.p95)} ms, max ${f1(p.latency.pub_to_rx_ms.max)} ms. ` +
				`Received -> row committed: p50 ${f1(p.latency.rx_to_commit_ms.p50)} ms, p95 ${f1(p.latency.rx_to_commit_ms.p95)} ms, ` +
				`max ${f1(p.latency.rx_to_commit_ms.max)} ms.`, '');
		}
		if (p.storage) {
			const perHour = p.storage.bytes_added / p.seconds * 3600;
			L.push(`Frames published ${p.frames_published} (${p.payload_bytes} payload bytes, ` +
				`${f1(p.payload_bytes / p.seconds)} B/s). Rows: telemetry ${p.rows.t}, heartbeat ${p.rows.hb}, events ${p.rows.ev}.`,
			`Storage: ${(p.storage.bytes_added / 1024).toFixed(0)} KiB added in ${p.seconds} s (telemetry+heartbeat+event incl. indexes) ` +
				`= ${(perHour / 2 ** 20).toFixed(1)} MiB per hour of robot operation. Average telemetry row ${f1(p.storage.row_bytes)} B ` +
				`(frame ${f1(p.storage.frame_bytes)} B, decoded JSONB ${f1(p.storage.decoded_json_bytes)} B).`, '');
		}
		if (p.query_ms) {
			L.push(`${p.queries} panel queries (${p.failed} failed), every panel every 5 s: p50 ${f1(p.query_ms.p50)} ms, ` +
				`p95 ${f1(p.query_ms.p95)} ms, max ${f1(p.query_ms.max)} ms. 6-hour range: p50 ${f1(p.six_hour_range_ms.p50)} ms, ` +
				`max ${f1(p.six_hour_range_ms.max)} ms (${p.six_hour_range_ms.failed} failed).`, '');
		}
		if (p.drain_ms !== undefined) {
			L.push(`Outage ${p.seconds} s: peak buffer ${p.buffered_peak.bulk} bulk + ${p.buffered_peak.priority} priority rows; ` +
				`drained ${p.drain_ms} ms after the database returned; telemetry rows kept ${p.telemetry_rows_kept}; ` +
				`dropped ${p.dropped_bulk} bulk / ${p.dropped_priority} priority.`, '');
		}
		if (p.nodered_reconnect_ms !== undefined) {
			L.push(`Broker down ${p.seconds} s: Node-RED noticed after ${p.nodered_detect_ms} ms, reconnected ` +
				`${p.nodered_reconnect_ms} ms after the broker returned. Frames published in the phase ${p.frames_published}, ` +
				`telemetry rows added ${p.telemetry_rows_added} (frames sent while the broker was down are lost: QoS 0, ` +
				`no queue, as on the Pico). LINK_UNKNOWN events: ${p.link_unknown_events}.`, '');
		}
		if (p.resources) {
			L.push('| service | CPU (avg) | RSS (end) | RSS (peak) |', '|---|---|---|---|', ...res(p.resources), '');
		}
	}
	return L.join('\n') + '\n';
}

main().catch((e) => {
	console.error('bench failed: ' + (e.stack || e.message));
	process.exitCode = 1;
});
