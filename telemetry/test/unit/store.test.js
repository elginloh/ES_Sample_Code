/*
 * Store (lib/store.js) with a fake PostgreSQL pool: bounded buffering,
 * overflow policy, retry and recovery, poison rows, coalescing, spill.
 */
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('fs');
const os = require('os');
const path = require('path');
const { Store, Deque } = require('../../lib/store');

/* A pool whose availability the test controls */
function fakePool() {
	const p = {
		up: true,
		failCode: null,			/* data error code for rows matching badMarker */
		badMarker: null,
		inserts: [],			/* committed { table, rows } */
		on() {},
		async end() {},
		async connect() {
			if (!p.up) { const e = new Error('connect ECONNREFUSED 127.0.0.1:5432'); e.code = 'ECONNREFUSED'; throw e; }
			let tx = [];
			return {
				async query(q) {
					const text = typeof q === 'string' ? q : q.text;
					if (text === 'BEGIN') { tx = []; return {}; }
					if (text === 'ROLLBACK') { tx = []; return {}; }
					if (text === 'COMMIT') { p.inserts.push(...tx); tx = []; return {}; }
					if (!p.up) { const e = new Error('Connection terminated unexpectedly'); throw e; }
					if (p.badMarker !== null && q.values.includes(p.badMarker)) {
						const e = new Error('invalid input syntax'); e.code = p.failCode; throw e;
					}
					const table = /INSERT INTO csc14\.(\w+)/.exec(text)[1];
					const cols = /\(([^)]+)\) VALUES/.exec(text)[1].split(',');
					const rows = [];
					for (let i = 0; i < q.values.length; i += cols.length) {
						rows.push(Object.fromEntries(cols.map((c, j) => [c, q.values[i + j]])));
					}
					tx.push({ table, rows, text });
					return { rowCount: rows.length };
				},
				release() {}
			};
		}
	};
	return p;
}

const quiet = { info() {}, warn() {}, error() {} };
const tlm = (n) => ({ table: 'telemetry', priority: false, row: { robot_id: 'car', source: 'live',
	received_at: new Date(1e12 + n * 500), seq: n % 256, decoded: { n } } });
const evt = (n) => ({ table: 'event', priority: true, row: { robot_id: 'car', source: 'live',
	received_at: new Date(1e12 + n * 500), origin: 'server', kind: 'K' + n, severity: 'info', message: 'm', data: { n } } });
const rows = (pool, table) => pool.inserts.filter((x) => x.table === table).flatMap((x) => x.rows);
const settle = (store, ms) => new Promise((resolve) => {
	const until = Date.now() + (ms || 3000);
	const tick = () => ((store.queued() === 0 && store.dbUp) || Date.now() > until) ? resolve() : setTimeout(tick, 10);
	tick();
});

function makeStore(pool, opts) {
	return new Store({}, Object.assign({ poolFactory: () => pool, retryMinMs: 20, retryMaxMs: 40,
		maxBulkRows: 5, maxPriorityRows: 3, robotId: 'car' }, opts || {}), quiet);
}

test('rows are written in arrival order with parameterized multi-row INSERTs and JSONB casts', async () => {
	const pool = fakePool();
	const s = makeStore(pool, { maxBulkRows: 100 });
	for (let i = 0; i < 4; i++) { s.enqueue(tlm(i)); }
	s.enqueue(evt(4));
	s.start();
	await settle(s);
	await s.stop(0);
	assert.deepEqual(rows(pool, 'telemetry').map((r) => r.seq), [0, 1, 2, 3]);
	assert.equal(rows(pool, 'event')[0].kind, 'K4');
	const text = pool.inserts.find((x) => x.table === 'telemetry').text;
	assert.match(text, /\$\d+::jsonb/);
	assert.doesNotMatch(text, /K4|1000000000000/);			/* values never inlined */
	assert.equal(typeof rows(pool, 'telemetry')[0].decoded, 'string');
});

test('database outage: rows are buffered, retried, and all written after recovery', async () => {
	const pool = fakePool();
	pool.up = false;
	const s = makeStore(pool, { maxBulkRows: 100 });
	s.start();
	for (let i = 0; i < 20; i++) { s.enqueue(tlm(i)); }
	await new Promise((r) => setTimeout(r, 150));
	assert.equal(s.dbUp, false);
	assert.equal(s.queued(), 20);
	assert.ok(s.health().counters.outages >= 1);
	pool.up = true;
	await settle(s);
	await s.stop(0);
	assert.deepEqual(rows(pool, 'telemetry').map((r) => r.seq), [...Array(20).keys()]);
	assert.equal(s.health().counters.droppedBulk, 0);
});

test('a full buffer drops the oldest bulk rows, keeps priority rows, and reports the loss', async () => {
	const pool = fakePool();
	pool.up = false;
	const s = makeStore(pool);			/* 5 bulk, 3 priority */
	s.start();
	for (let i = 0; i < 8; i++) { s.enqueue(tlm(i)); }
	s.enqueue(evt(100));
	s.enqueue(evt(101));
	assert.equal(s.health().counters.droppedBulk, 3);
	assert.equal(s.health().counters.droppedPriority, 0);
	assert.equal(s.health().loss_pending, 3);
	pool.up = true;
	await settle(s);
	await settle(s);
	await s.stop(0);
	assert.deepEqual(rows(pool, 'telemetry').map((r) => r.seq), [3, 4, 5, 6, 7]);
	const loss = rows(pool, 'event').find((e) => e.kind === 'BACKEND_BUFFER_OVERFLOW');
	assert.ok(loss, 'overflow is reported once the database is back');
	assert.equal(JSON.parse(loss.data).rows, 3);
	assert.deepEqual(rows(pool, 'event').filter((e) => e.kind !== 'BACKEND_BUFFER_OVERFLOW').map((e) => e.kind),
		['K100', 'K101']);
});

test('a row the database refuses as bad data is isolated and dropped; the rest is written', async () => {
	const pool = fakePool();
	pool.failCode = '22P02';
	pool.badMarker = 3;				/* seq value 3 */
	const s = makeStore(pool, { maxBulkRows: 100 });
	for (let i = 0; i < 6; i++) { s.enqueue(tlm(i)); }
	s.start();
	await settle(s);
	await s.stop(0);
	assert.deepEqual(rows(pool, 'telemetry').map((r) => r.seq), [0, 1, 2, 4, 5]);
	assert.equal(s.health().counters.poisonRows, 1);
});

test('missing table or permission is not "bad data": rows are kept and retried', async () => {
	const pool = fakePool();
	pool.failCode = '42P01';			/* undefined_table: migration not applied */
	pool.badMarker = 0;
	const s = makeStore(pool, { maxBulkRows: 100 });
	s.enqueue(tlm(0));
	s.start();
	await new Promise((r) => setTimeout(r, 100));
	assert.equal(s.queued(), 1);
	assert.equal(s.health().counters.poisonRows, 0);
	pool.badMarker = null;
	await settle(s);
	await s.stop(0);
	assert.equal(rows(pool, 'telemetry').length, 1);
});

test('keyed upserts (link state) coalesce while queued', async () => {
	const pool = fakePool();
	pool.up = false;
	const s = makeStore(pool);
	for (let i = 0; i < 10; i++) {
		s.enqueue({ table: 'robot_link', priority: true, key: 'robot_link:car|live',
			row: { robot_id: 'car', source: 'live', state: 'S' + i, updated_at: new Date() } });
	}
	assert.equal(s.queued(), 1);
	s.start();
	pool.up = true;
	await settle(s);
	await s.stop(0);
	assert.deepEqual(rows(pool, 'robot_link').map((r) => r.state), ['S9']);
	assert.match(pool.inserts.find((x) => x.table === 'robot_link').text, /ON CONFLICT \(robot_id, source\)/);
});

test('rows left at shutdown are spilled to disk and queued again on the next start', async () => {
	const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'csc14-spill-'));
	const spillFile = path.join(dir, 'unwritten.jsonl');
	const down = fakePool();
	down.up = false;
	const s1 = makeStore(down, { spillFile, maxBulkRows: 100 });
	s1.start();
	for (let i = 0; i < 3; i++) { s1.enqueue(tlm(i)); }
	s1.enqueue(evt(9));
	await s1.stop(50);
	assert.ok(fs.existsSync(spillFile));
	const up = fakePool();
	const s2 = makeStore(up, { spillFile, maxBulkRows: 100 });
	s2.start();
	await settle(s2);
	await s2.stop(0);
	assert.equal(fs.existsSync(spillFile), false);
	const t = rows(up, 'telemetry');
	assert.deepEqual(t.map((r) => r.seq), [0, 1, 2]);
	assert.ok(t[0].received_at instanceof Date);
	assert.equal(rows(up, 'event')[0].kind, 'K9');
	fs.rmSync(dir, { recursive: true, force: true });
});

test('deque keeps order through shift and unshiftAll', () => {
	const d = new Deque();
	for (let i = 0; i < 3000; i++) { d.push(i); }
	for (let i = 0; i < 2000; i++) { assert.equal(d.shift(), i); }
	d.unshiftAll([-2, -1]);
	assert.equal(d.length, 1002);
	assert.equal(d.shift(), -2);
	assert.equal(d.peek(), -1);
});
