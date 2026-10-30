/*
 * harness.js -- isolated services for the integration tests:
 *   - a private Mosquitto on a random port (127.0.0.1 only)
 *   - a throw-away database csc14_it_<pid> with its own roles
 *   - a TCP proxy in front of PostgreSQL to simulate a database outage
 *   - Node-RED (the real runtime and the real flows.json) and Grafana
 * Nothing here touches the live broker, topics, database or roles.
 */
'use strict';

const { spawn, execFileSync } = require('child_process');
const crypto = require('crypto');
const fs = require('fs');
const net = require('net');
const os = require('os');
const path = require('path');
const { Client } = require('pg');
const config = require('../../lib/config');
const { migrate } = require('../../lib/migrate');
const { codec } = require('../../lib/protocol');
const { baseRecord } = require('../../lib/sim');

const ROOT = config.ROOT;
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

function freePort() {
	return new Promise((resolve, reject) => {
		const s = net.createServer();
		s.unref();
		s.on('error', reject);
		s.listen(0, '127.0.0.1', () => {
			const { port } = s.address();
			s.close(() => resolve(port));
		});
	});
}

async function waitPort(port, ms) {
	const until = Date.now() + ms;
	while (Date.now() < until) {
		const ok = await new Promise((r) => {
			const c = net.connect(port, '127.0.0.1');
			c.on('connect', () => { c.destroy(); r(true); });
			c.on('error', () => r(false));
		});
		if (ok) { return; }
		await sleep(100);
	}
	throw new Error('port ' + port + ' did not open');
}

async function eventually(fn, ms, what) {
	const until = Date.now() + (ms || 10000);
	let last;
	while (Date.now() < until) {
		try {
			const v = await fn();
			if (v) { return v; }
		} catch (e) { last = e; }
		await sleep(150);
	}
	throw new Error('timed out waiting for ' + (what || 'condition') + (last ? ': ' + last.message : ''));
}

function mosquittoBin() {
	for (const p of ['/opt/homebrew/sbin/mosquitto', '/usr/local/sbin/mosquitto', '/usr/sbin/mosquitto']) {
		if (fs.existsSync(p)) { return p; }
	}
	return 'mosquitto';
}

async function startBroker(dir) {
	const port = await freePort();
	const conf = path.join(dir, 'mosquitto.conf');
	fs.writeFileSync(conf, `listener ${port} 127.0.0.1\nallow_anonymous true\npersistence false\n`);
	const b = { port, proc: spawn(mosquittoBin(), ['-c', conf], { stdio: 'ignore' }) };
	b.stop = () => b.proc.kill('SIGTERM');
	b.restart = async () => {			/* same port, e.g. after an outage */
		b.proc = spawn(mosquittoBin(), ['-c', conf], { stdio: 'ignore' });
		await waitPort(port, 5000);
	};
	await waitPort(port, 5000);
	return b;
}

/* Forwards 127.0.0.1:<port> to PostgreSQL; disable() cuts every connection */
class Proxy {
	constructor(target) {
		this.target = target;
		this.sockets = new Set();
		this.server = null;
		this.port = null;
	}
	async enable() {
		this.server = net.createServer((c) => {
			const u = net.connect(this.target.port, this.target.host);
			this.sockets.add(c); this.sockets.add(u);
			c.pipe(u); u.pipe(c);
			const end = () => { c.destroy(); u.destroy(); this.sockets.delete(c); this.sockets.delete(u); };
			c.on('error', end); u.on('error', end); c.on('close', end); u.on('close', end);
		});
		if (!this.port) { this.port = await freePort(); }
		await new Promise((r) => this.server.listen(this.port, '127.0.0.1', r));
	}
	async disable() {
		for (const s of this.sockets) { s.destroy(); }
		this.sockets.clear();
		await new Promise((r) => this.server.close(r));
	}
}

async function createDatabase() {
	const tag = 'csc14_it_' + process.pid;
	const env = Object.assign({}, process.env, {
		CSC14_PG_DATABASE: tag, CSC14_PG_WRITER_USER: tag + '_w', CSC14_PG_READER_USER: tag + '_r',
		CSC14_PG_WRITER_PASSWORD: crypto.randomBytes(12).toString('hex'),
		CSC14_PG_READER_PASSWORD: crypto.randomBytes(12).toString('hex')
	});
	const cfg = config.load(env);
	await migrate(cfg, { log: () => {} });
	const admin = new Client({ host: cfg.pg.host, port: cfg.pg.port, database: tag, user: cfg.pgAdmin.user,
		password: cfg.pgAdmin.password });
	await admin.connect();
	return {
		cfg, admin, env,
		async drop() {
			await admin.end();
			const c = new Client({ host: cfg.pg.host, port: cfg.pg.port, database: 'postgres',
				user: cfg.pgAdmin.user, password: cfg.pgAdmin.password });
			await c.connect();
			await c.query(`DROP DATABASE IF EXISTS ${tag} WITH (FORCE)`);
			for (const r of [cfg.pg.user, cfg.pgReader.user]) {
				await c.query(`DROP ROLE IF EXISTS ${r}`);
			}
			await c.end();
		}
	};
}

class NodeRed {
	constructor(env) {
		this.env = env;
		this.proc = null;
		this.log = '';
	}
	async start() {
		this.proc = spawn(process.execPath, [path.join(ROOT, 'node_modules', 'node-red', 'red.js'),
			'--settings', path.join(ROOT, 'nodered', 'settings.js'), '--userDir', this.env.CSC14_RUNTIME_DIR + '/nodered'],
		{ env: this.env, stdio: ['ignore', 'pipe', 'pipe'] });
		this.proc.stdout.on('data', (d) => { this.log += d; });
		this.proc.stderr.on('data', (d) => { this.log += d; });
		await eventually(async () => (await this.health()) !== null, 30000, 'Node-RED /csc14/health');
		await eventually(async () => {
			const h = await this.health();
			return h && h.broker.connected;
		}, 15000, 'Node-RED broker connection');
	}
	async health() {
		try {
			const r = await fetch(`http://127.0.0.1:${this.env.CSC14_NODERED_PORT}/csc14/health`);
			return await r.json();
		} catch (e) { return null; }
	}
	async stop() {
		if (!this.proc) { return; }
		const p = this.proc;
		this.proc = null;
		const done = new Promise((r) => p.on('exit', r));
		p.kill('SIGTERM');
		await Promise.race([done, sleep(15000).then(() => p.kill('SIGKILL'))]);
	}
}

/* A robot that booted bootMs ago and keeps uptime with the wall clock */
class Sender {
	constructor(client, prefix, bootAgoMs) {
		this.client = client;
		this.prefix = prefix;
		this.boot = Date.now() - (bootAgoMs || 100000);
		this.seq = { 1: 0, 2: 0, 3: 0, 5: 0 };
		this.r = baseRecord();
		this.r.mode = 1; this.r.line_state = 0; this.r.imu_status = 2;
	}
	uptime() { return Math.floor((Date.now() - this.boot) / 100) % 65536; }
	pub(channel, frame) {
		return new Promise((r) => this.client.publish(this.prefix + '/' + channel, frame, { qos: 0 }, r));
	}
	tlm(over, skip) {
		if (skip) { this.seq[1] = (this.seq[1] + skip) & 0xFF; }
		const f = codec.encodeFrame(codec.TYPE.TELEMETRY, this.seq[1], Object.assign({}, this.r,
			{ uptime_ds: this.uptime() }, over || {}));
		this.seq[1] = (this.seq[1] + 1) & 0xFF;
		return this.pub('tlm', f);
	}
	hb(over) {
		const f = codec.encodeFrame(codec.TYPE.HEARTBEAT, this.seq[2], Object.assign({ uptime_ds: this.uptime(),
			mode: 1, comm_state: 4, critical: 0, fault_count: 0, reconnects: 0 }, over || {}));
		this.seq[2] = (this.seq[2] + 1) & 0xFF;
		return this.pub('hb', f);
	}
	evt(code, fields) {
		const f = codec.encodeEvent(code, fields, this.seq[3]);
		this.seq[3] = (this.seq[3] + 1) & 0xFF;
		return this.pub('evt', f);
	}
	ack(cmdId, cmd, result, detail) {
		const f = codec.encodeFrame(codec.TYPE.ACK, this.seq[5], { cmd_id: cmdId, cmd, result, detail: detail || 0 });
		this.seq[5] = (this.seq[5] + 1) & 0xFF;
		return this.pub('ack', f);
	}
	reboot() {
		this.boot = Date.now() - 8000;
		this.seq = { 1: 0, 2: 0, 3: 0, 5: 0 };
	}
}

function grafanaHome() {
	const dir = path.join(ROOT, '.runtime');
	const g = fs.existsSync(dir) && fs.readdirSync(dir).find((d) => /^grafana-\d+\.\d+\.\d+$/.test(d));
	return g ? path.join(dir, g) : null;
}

async function startGrafana(dir, db) {
	const home = grafanaHome();
	if (!home) { return null; }
	const port = await freePort();
	const env = Object.assign({}, process.env, {
		CSC14_TELEMETRY_DIR: ROOT, CSC14_PG_HOST: db.cfg.pg.host, CSC14_PG_PORT: String(db.cfg.pg.port),
		CSC14_PG_DATABASE: db.cfg.pg.database, CSC14_PG_READER_USER: db.cfg.pgReader.user,
		CSC14_PG_READER_PASSWORD: db.cfg.pgReader.password,
		GF_PATHS_DATA: path.join(dir, 'grafana-data'), GF_PATHS_LOGS: path.join(dir, 'grafana-logs'),
		GF_PATHS_PLUGINS: path.join(dir, 'grafana-data', 'plugins'),
		GF_PATHS_PROVISIONING: path.join(ROOT, 'grafana', 'provisioning'),
		GF_SERVER_HTTP_ADDR: '127.0.0.1', GF_SERVER_HTTP_PORT: String(port),
		GF_SECURITY_ADMIN_PASSWORD: 'it-' + crypto.randomBytes(8).toString('hex'), GF_AUTH_ANONYMOUS_ENABLED: 'false'
	});
	const proc = spawn(path.join(home, 'bin', 'grafana'), ['server', '--homepath', home,
		'--config', path.join(ROOT, 'grafana', 'grafana.ini')], { env, stdio: 'ignore' });
	const url = 'http://127.0.0.1:' + port;
	await eventually(async () => (await fetch(url + '/api/health')).ok, 120000, 'Grafana');
	return {
		url, proc, auth: 'Basic ' + Buffer.from('admin:' + env.GF_SECURITY_ADMIN_PASSWORD).toString('base64'),
		stop: () => new Promise((r) => { proc.on('exit', r); proc.kill('SIGTERM'); })
	};
}

function tmpDir() {
	return fs.mkdtempSync(path.join(os.tmpdir(), 'csc14-it-'));
}

module.exports = { freePort, waitPort, eventually, startBroker, Proxy, createDatabase, NodeRed, Sender,
	startGrafana, tmpDir, sleep, execFileSync };
