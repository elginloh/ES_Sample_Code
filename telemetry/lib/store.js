/*
 * store.js
 * PostgreSQL writer for the ingestion operations (ingest.js).
 *
 * Buffering policy (docs/DESIGN.md, "Database outage"):
 *   - two bounded in-memory queues: PRIORITY (events, humps, ACKs,
 *     rejected frames, link and session rows) and BULK (telemetry,
 *     heartbeats, health snapshots).
 *   - a full queue drops its OLDEST row; every drop is counted and, once
 *     the database accepts writes again, summarised in one
 *     BACKEND_BUFFER_OVERFLOW event.  Memory therefore stays bounded.
 *   - rows are written in arrival order; whatever is queued (up to
 *     batchRows) goes in one transaction, so a backlog drains quickly
 *     without delaying single rows to fill a batch.
 *   - connection or schema problems: the batch goes back to the front of
 *     the queue and is retried with exponential back-off.  A row that the
 *     database rejects as bad data is isolated and dropped (counted), so it
 *     cannot block the queue.
 *   - on shutdown the remaining rows are spilled to a file and loaded
 *     again on the next start.
 */
'use strict';

const fs = require('fs');
const path = require('path');
const { Pool } = require('pg');

const BASE = ['robot_id', 'source', 'received_at'];
const TABLES = {
	telemetry: {
		bulk: true,
		cols: BASE.concat(['topic', 'boot_id', 'proto_version', 'seq', 'seq_gap', 'rx_delay_ms', 'late',
			'uptime_ds', 'uptime_s', 'mode', 'epoch', 'fault_mask', 'critical', 'motion_state',
			'motion_source', 'cmd_active', 'speed_l_mms', 'speed_r_mms', 'odometer_cm',
			'heading_enc_cw_deg', 'cruise_set_mms', 'ir_raw', 'ir_left', 'ir_centre', 'ir_right',
			'line_pos', 'line_pos_phys', 'line_state', 'steer', 'bc_state', 'bc_symbol', 'bc_result',
			'nav', 'nav_valid', 'bc_valid_mod64', 'bc_invalid_mod64', 'imu_status', 'pitch_deg',
			'roll_deg', 'yaw_rate_ccw_dps', 'heading_imu_ccw_deg', 'motion_class', 'tilt', 'collision',
			'shock_g', 'hump_last_mm', 'hump_max_boot_mm', 'hump_count', 'hump_active', 'us_dist_mm',
			'us_no_echo', 'us_result', 'us_context', 'us_forward_mm', 'avoid_state', 'avoid_side',
			'avoid_outcome', 'profile_reported', 'obst_width_mm', 'obst_bearing_deg', 'clear_left_mm',
			'clear_left_at_least', 'clear_right_mm', 'clear_right_at_least', 'comm_state',
			'tlm_dropped', 'rx_rejected', 'frame', 'decoded']),
		json: ['decoded']
	},
	heartbeat: {
		bulk: true,
		cols: BASE.concat(['topic', 'boot_id', 'seq', 'seq_gap', 'uptime_ds', 'uptime_s', 'mode',
			'comm_state', 'critical', 'fault_count', 'reconnects', 'frame'])
	},
	event: {
		cols: BASE.concat(['topic', 'boot_id', 'origin', 'kind', 'severity', 'seq', 'message', 'data', 'frame']),
		json: ['data']
	},
	ack: {
		cols: BASE.concat(['topic', 'boot_id', 'seq', 'cmd_id', 'cmd', 'result', 'detail', 'frame'])
	},
	rejected_frame: {
		cols: BASE.concat(['topic', 'reason', 'detail', 'length', 'frame'])
	},
	hump: {
		single: true,
		cols: BASE.concat(['topic', 'boot_id', 'height_mm', 'height_code', 'count_after', 'max_boot_mm',
			'detected_by']),
		conflict: 'ON CONFLICT (boot_id, count_after) DO UPDATE SET ' +
			'height_mm = COALESCE(csc14.hump.height_mm, EXCLUDED.height_mm), ' +
			'height_code = COALESCE(csc14.hump.height_code, EXCLUDED.height_code), ' +
			'max_boot_mm = COALESCE(csc14.hump.max_boot_mm, EXCLUDED.max_boot_mm), ' +
			'detected_by = CASE WHEN strpos(csc14.hump.detected_by, EXCLUDED.detected_by) > 0 ' +
			'THEN csc14.hump.detected_by ELSE csc14.hump.detected_by || \'+\' || EXCLUDED.detected_by END'
	},
	boot_session: {
		single: true,
		cols: ['boot_id', 'robot_id', 'source', 'first_rx_at', 'last_rx_at', 'first_uptime_s',
			'last_uptime_s', 'start_reason', 'ir_sides_mirrored', 'policy'],
		json: ['policy'],
		conflict: 'ON CONFLICT (boot_id) DO UPDATE SET ' +
			'last_rx_at = GREATEST(csc14.boot_session.last_rx_at, EXCLUDED.last_rx_at), ' +
			'last_uptime_s = EXCLUDED.last_uptime_s'
	},
	robot_link: {
		single: true,
		cols: ['robot_id', 'source', 'state', 'state_since', 'last_rx_at', 'last_hb_at', 'last_tlm_at',
			'last_reconnects', 'last_comm_state', 'boot_id', 'updated_at'],
		conflict: 'ON CONFLICT (robot_id, source) DO UPDATE SET state = EXCLUDED.state, ' +
			'state_since = EXCLUDED.state_since, last_rx_at = EXCLUDED.last_rx_at, ' +
			'last_hb_at = EXCLUDED.last_hb_at, last_tlm_at = EXCLUDED.last_tlm_at, ' +
			'last_reconnects = EXCLUDED.last_reconnects, last_comm_state = EXCLUDED.last_comm_state, ' +
			'boot_id = EXCLUDED.boot_id, updated_at = EXCLUDED.updated_at'
	},
	ingest_health: {
		bulk: true,
		cols: ['at', 'instance', 'broker_connected', 'db_connected', 'queue_bulk', 'queue_priority',
			'dropped_bulk_total', 'dropped_priority_total', 'written_total', 'write_errors_total',
			'latency_p50_ms', 'latency_p95_ms', 'latency_max_ms', 'frames_total', 'rejected_total',
			'rss_mb', 'heap_mb', 'cpu_pct', 'uptime_s', 'details'],
		json: ['details']
	}
};

/* Connection-level failures and schema/permission problems: keep the rows
   and retry.  Everything else from the server is a problem with the row. */
const RETRY_CODES = new Set(['ECONNREFUSED', 'ECONNRESET', 'ETIMEDOUT', 'EPIPE', 'ENOTFOUND',
	'EHOSTUNREACH', 'ENETUNREACH', 'EAI_AGAIN', '57P01', '57P02', '57P03', '53300', '42P01',
	'42703', '42501', '3D000', '28P01', '28000', '25P02', '40001', '40P01']);
function retryable(err) {
	const c = err && err.code;
	if (c && RETRY_CODES.has(c)) { return true; }
	if (c && /^08/.test(c)) { return true; }
	return !c || /terminat|timeout|connect|ECONN|closed/i.test(String(err && err.message));
}

/* Double-ended queue (array with a moving head) */
class Deque {
	constructor() { this.a = []; this.h = 0; }
	get length() { return this.a.length - this.h; }
	push(x) { this.a.push(x); }
	shift() {
		if (this.h >= this.a.length) { return undefined; }
		const x = this.a[this.h];
		this.a[this.h++] = undefined;
		if (this.h > 1024 && this.h * 2 > this.a.length) {
			this.a = this.a.slice(this.h);
			this.h = 0;
		}
		return x;
	}
	unshiftAll(items) {
		this.a = items.concat(this.a.slice(this.h));
		this.h = 0;
	}
	peek() { return this.a[this.h]; }
	toArray() { return this.a.slice(this.h); }
}

function percentile(sorted, p) {
	if (sorted.length === 0) { return null; }
	return sorted[Math.min(sorted.length - 1, Math.floor(p * (sorted.length - 1) + 0.5))];
}

/* Spill format: Buffers and Dates survive JSON */
function encodeValue(k, v) {
	if (v && v.type === 'Buffer' && Array.isArray(v.data)) { return { $b64: Buffer.from(v.data).toString('base64') }; }
	return v;
}
function reviveRow(row) {
	for (const [k, v] of Object.entries(row)) {
		if (v && typeof v === 'object' && v.$b64 !== undefined) { row[k] = Buffer.from(v.$b64, 'base64'); }
		else if (v && typeof v === 'object' && v.$date !== undefined) { row[k] = new Date(v.$date); }
	}
	return row;
}

class Store {
	constructor(pgConfig, opts, log) {
		this.pgConfig = pgConfig;
		this.opts = Object.assign({ maxBulkRows: 10800, maxPriorityRows: 5000, batchRows: 500,
			retryMinMs: 500, retryMaxMs: 10000, spillFile: null, instance: 'nodered' }, opts || {});
		this.log = log || { info: console.log, warn: console.warn, error: console.error };
		this.bulk = new Deque();
		this.prio = new Deque();
		this.keyed = new Map();
		this.pool = null;
		this.running = false;
		this.wake = null;
		this.dbUp = null;
		this.dbDownSince = null;
		this.backoff = this.opts.retryMinMs;
		this.inflight = 0;			/* rows taken from the queue, not yet committed */
		this.loss = null;			/* rows dropped since the last report */
		this.latencies = [];
		this.s = { enqueued: 0, written: 0, writtenByTable: {}, droppedBulk: 0, droppedPriority: 0,
			   poisonRows: 0, writeErrors: 0, outages: 0, batches: 0, maxBatch: 0,
			   lastError: null, lastWriteAt: null, spilled: 0, restored: 0 };
	}

	/* ------------------------------------------------------------------ */
	enqueue(o) {
		const def = TABLES[o.table];
		if (!def) {
			throw new Error('unknown table ' + o.table);
		}
		if (o.key && this.keyed.has(o.key)) {
			Object.assign(this.keyed.get(o.key).row, o.row);	/* coalesce an upsert */
			return;
		}
		const item = { table: o.table, row: o.row, key: o.key, t: Date.now(),
			       bulk: !!def.bulk && !o.priority };
		for (const j of def.json || []) {
			if (item.row[j] !== null && item.row[j] !== undefined && typeof item.row[j] !== 'string') {
				item.row = Object.assign({}, item.row, { [j]: JSON.stringify(item.row[j]) });
			}
		}
		const q = item.bulk ? this.bulk : this.prio;
		const max = item.bulk ? this.opts.maxBulkRows : this.opts.maxPriorityRows;
		while (q.length >= max) {
			this.drop(q.shift(), item.bulk);
		}
		q.push(item);
		if (item.key) { this.keyed.set(item.key, item); }
		this.s.enqueued++;
		if (this.wake) { this.wake(); }
	}

	enqueueAll(ops) {
		for (const o of ops || []) { this.enqueue(o); }
	}

	drop(item, bulk) {
		if (!item) { return; }
		if (item.key) { this.keyed.delete(item.key); }
		if (bulk) { this.s.droppedBulk++; } else { this.s.droppedPriority++; }
		const at = item.row.received_at || item.row.at || null;
		if (!this.loss) {
			this.loss = { rows: 0, byTable: {}, first: at, last: at, since: Date.now() };
		}
		this.loss.rows++;
		this.loss.byTable[item.table] = (this.loss.byTable[item.table] || 0) + 1;
		this.loss.last = at || this.loss.last;
	}

	queued() { return this.bulk.length + this.prio.length + this.inflight; }

	/* ------------------------------------------------------------------ */
	start() {
		if (this.running) { return; }
		this.running = true;
		const pc = Object.assign({ max: 2, connectionTimeoutMillis: 3000, idleTimeoutMillis: 30000,
			application_name: 'csc14-' + this.opts.instance, keepAlive: true,
			statement_timeout: 15000 }, this.pgConfig);
		this.pool = this.opts.poolFactory ? this.opts.poolFactory(pc) : new Pool(pc);	/* tests inject a fake */
		this.pool.on('error', (err) => { this.noteError(err); });
		this.restoreSpill();
		this.loopDone = this.loop();
	}

	async stop(flushMs) {
		const until = Date.now() + (flushMs === undefined ? 3000 : flushMs);
		while (this.queued() > 0 && this.dbUp && Date.now() < until) {
			await new Promise((r) => setTimeout(r, 50));
		}
		this.running = false;
		if (this.wake) { this.wake(); }
		try { await this.loopDone; } catch (e) { /* loop ended */ }
		this.spill();
		if (this.pool) {
			await this.pool.end().catch(() => {});
			this.pool = null;
		}
	}

	noteError(err) {
		this.s.lastError = (err && (err.code ? err.code + ' ' : '') + (err.message || String(err))).slice(0, 300);
	}

	async loop() {
		while (this.running) {
			if (this.queued() === 0) {
				await new Promise((r) => { this.wake = r; setTimeout(r, 1000); });
				this.wake = null;
				continue;
			}
			const batch = this.take();
			this.inflight = batch.length;
			const ok = await this.write(batch);
			this.inflight = 0;
			if (!ok) {
				await new Promise((r) => setTimeout(r, this.backoff));
				this.backoff = Math.min(this.backoff * 2, this.opts.retryMaxMs);
			}
		}
	}

	take() {
		const batch = [];
		while (batch.length < this.opts.batchRows && this.queued() > 0) {
			/* oldest first across both queues, so order is kept; priority
			   rows only differ in how they are dropped */
			const p = this.prio.peek();
			const b = this.bulk.peek();
			const item = (!b || (p && p.t <= b.t)) ? this.prio.shift() : this.bulk.shift();
			if (item.key) { this.keyed.delete(item.key); }
			batch.push(item);
		}
		return batch;
	}

	requeue(batch) {
		const prio = batch.filter((i) => !i.bulk);
		const bulk = batch.filter((i) => i.bulk);
		for (const i of prio) {
			if (i.key) {
				if (this.keyed.has(i.key)) {		/* a newer value arrived meanwhile */
					const newer = this.keyed.get(i.key);
					newer.row = Object.assign({}, i.row, newer.row);
					continue;
				}
				this.keyed.set(i.key, i);
			}
		}
		this.prio.unshiftAll(prio.filter((i) => !i.key || this.keyed.get(i.key) === i));
		this.bulk.unshiftAll(bulk);
		for (const [q, max, isBulk] of [[this.prio, this.opts.maxPriorityRows, false],
			[this.bulk, this.opts.maxBulkRows, true]]) {
			while (q.length > max) { this.drop(q.shift(), isBulk); }
		}
	}

	sqlFor(table, rows) {
		const def = TABLES[table];
		const params = [];
		const tuples = rows.map((row) => '(' + def.cols.map((c) => {
			let v = row[c];
			if (v === undefined) { v = null; }
			params.push(v);
			return '$' + params.length + ((def.json || []).includes(c) ? '::jsonb' : '');
		}).join(',') + ')');
		return {
			text: `INSERT INTO csc14.${table} (${def.cols.join(',')}) VALUES ${tuples.join(',')}` +
				(def.conflict ? ' ' + def.conflict : ''),
			values: params
		};
	}

	/* One transaction: consecutive rows of one table share one INSERT */
	async exec(client, batch) {
		let i = 0;
		while (i < batch.length) {
			const table = batch[i].table;
			let j = i + 1;
			if (!TABLES[table].single) {
				while (j < batch.length && batch[j].table === table) { j++; }
			}
			await client.query(this.sqlFor(table, batch.slice(i, j).map((x) => x.row)));
			i = j;
		}
	}

	async write(batch) {
		let client;
		try {
			client = await this.pool.connect();
		} catch (err) {
			this.down(err);
			this.requeue(batch);
			return false;
		}
		try {
			await client.query('BEGIN');
			await this.exec(client, batch);
			await client.query('COMMIT');
			client.release();
			this.committed(batch);
			return true;
		} catch (err) {
			await client.query('ROLLBACK').catch(() => {});
			client.release(retryable(err) ? err : undefined);
			if (retryable(err)) {
				this.down(err);
				this.requeue(batch);
				return false;
			}
			this.s.writeErrors++;
			this.noteError(err);
			if (batch.length === 1) {
				this.s.poisonRows++;
				this.log.warn(`csc14 store: dropped one ${batch[0].table} row the database refused: ${this.s.lastError}`);
				return true;
			}
			/* isolate the bad row: write the rows one at a time */
			for (const item of batch) {
				if (!(await this.write([item]))) {
					this.requeue(batch.slice(batch.indexOf(item)));
					return false;
				}
			}
			return true;
		}
	}

	down(err) {
		this.noteError(err);
		if (this.dbUp !== false) {
			this.s.outages++;
			this.dbDownSince = Date.now();
			this.log.warn('csc14 store: database unavailable, buffering (' + this.s.lastError + ')');
		}
		this.dbUp = false;
	}

	committed(batch) {
		const now = Date.now();
		if (this.dbUp === false) {
			this.log.info(`csc14 store: database back after ${((now - this.dbDownSince) / 1000).toFixed(1)} s, ` +
				`${this.queued()} row(s) still queued`);
		}
		this.dbUp = true;
		this.dbDownSince = null;
		this.backoff = this.opts.retryMinMs;
		this.s.batches++;
		this.s.maxBatch = Math.max(this.s.maxBatch, batch.length);
		this.s.lastWriteAt = now;
		for (const item of batch) {
			this.s.written++;
			this.s.writtenByTable[item.table] = (this.s.writtenByTable[item.table] || 0) + 1;
			this.latencies.push(now - item.t);
		}
		if (this.latencies.length > 1000) { this.latencies.splice(0, this.latencies.length - 1000); }
		if (this.loss) {
			const l = this.loss;
			this.loss = null;
			this.enqueue({ table: 'event', priority: true, row: {
				robot_id: this.opts.robotId || 'backend', source: 'live', received_at: new Date(now),
				origin: 'server', kind: 'BACKEND_BUFFER_OVERFLOW', severity: 'warning',
				message: `Backend buffer was full while the database was unavailable: ${l.rows} row(s) ` +
					`dropped (${Object.entries(l.byTable).map(([t, n]) => t + ' ' + n).join(', ')}); ` +
					'these measurements are missing from the history',
				data: { rows: l.rows, by_table: l.byTable, first_dropped_received_at: l.first,
					last_dropped_received_at: l.last }
			} });
		}
	}

	/* ------------------------------------------------------------------ */
	health() {
		const sorted = this.latencies.slice(-500).sort((a, b) => a - b);
		return {
			db_connected: this.dbUp,
			db_down_since: this.dbDownSince ? new Date(this.dbDownSince).toISOString() : null,
			queue_bulk: this.bulk.length,
			queue_priority: this.prio.length,
			inflight: this.inflight,
			limits: { bulk: this.opts.maxBulkRows, priority: this.opts.maxPriorityRows },
			latency_ms: { p50: percentile(sorted, 0.5), p95: percentile(sorted, 0.95),
				      max: sorted.length ? sorted[sorted.length - 1] : null, samples: sorted.length },
			counters: Object.assign({}, this.s),
			loss_pending: this.loss ? this.loss.rows : 0
		};
	}

	/* ------------------------------------------------------------------ */
	spill() {
		const f = this.opts.spillFile;
		const items = this.prio.toArray().concat(this.bulk.toArray());
		if (!f || items.length === 0) { return; }
		try {
			fs.mkdirSync(path.dirname(f), { recursive: true });
			const lines = items.map((i) => JSON.stringify({ table: i.table, key: i.key, bulk: i.bulk, t: i.t,
				row: Object.fromEntries(Object.entries(i.row).map(([k, v]) =>
					[k, (v instanceof Date) ? { $date: v.toISOString() } : v])) }, encodeValue));
			fs.writeFileSync(f + '.tmp', lines.join('\n') + '\n');
			fs.renameSync(f + '.tmp', f);
			this.s.spilled += items.length;
			this.log.warn(`csc14 store: ${items.length} unwritten row(s) saved to ${f}`);
		} catch (err) {
			this.log.error('csc14 store: could not save unwritten rows: ' + err.message);
		}
	}

	restoreSpill() {
		const f = this.opts.spillFile;
		let text;
		if (!f) { return; }
		try { text = fs.readFileSync(f, 'utf8'); } catch (e) { return; }
		let n = 0;
		for (const line of text.split('\n')) {
			if (!line.trim()) { continue; }
			try {
				const i = JSON.parse(line);
				const item = { table: i.table, key: i.key, bulk: i.bulk, t: i.t, row: reviveRow(i.row) };
				(item.bulk ? this.bulk : this.prio).push(item);
				if (item.key) { this.keyed.set(item.key, item); }
				n++;
			} catch (e) { /* skip a damaged line */ }
		}
		fs.unlinkSync(f);
		this.s.restored += n;
		if (n) { this.log.info(`csc14 store: ${n} row(s) from the previous run queued again`); }
	}
}

module.exports = { Store, TABLES, Deque, retryable };
