/*
 * nodered-api.js
 * What the Node-RED flow calls (settings.js puts it in global context as
 * 'csc14').  One Ingestor and one Store per Node-RED process.  The flow
 * decides the routing; this module only does the work:
 *   ingest(topic, payload, rxMs) -> { kind, decoded, ops }   (no I/O)
 *   store(ops)                   -> queue the rows for PostgreSQL
 *   tick(nowMs)                  -> link-state ops (online/stale/offline)
 *   healthOps()                  -> one ingest_health row
 *   health()                     -> pipeline health for /csc14/health
 */
'use strict';

const fs = require('fs');
const path = require('path');
const config = require('./config');
const { Ingestor } = require('./ingest');
const { Store } = require('./store');

const cfg = config.load();
const startedAt = Date.now();
let ingestor = null;
let store = null;
let cpu = { usage: process.cpuUsage(), at: Date.now(), pct: null };
let flowErrors = 0;
let lastStateSave = null;

const log = {
	info: (m) => console.log('[csc14] ' + m),
	warn: (m) => console.warn('[csc14] ' + m),
	error: (m) => console.error('[csc14] ' + m)
};

function init() {
	if (!ingestor) {
		ingestor = new Ingestor(cfg);
		try {
			const n = ingestor.importState(JSON.parse(fs.readFileSync(cfg.stateFile, 'utf8')));
			log.info(`restored ingest state for ${n} stream(s) from ${cfg.stateFile}`);
		} catch (e) {
			log.info('no saved ingest state; boot sessions start fresh');
		}
	}
	if (!store) {
		store = new Store({ host: cfg.pg.host, port: cfg.pg.port, database: cfg.pg.database,
			user: cfg.pg.user, password: cfg.pg.password },
		Object.assign({}, cfg.store, { robotId: cfg.robotId, instance: cfg.instance,
			spillFile: path.join(path.dirname(cfg.stateFile), 'unwritten-rows.jsonl') }), log);
		store.start();
	}
}

function sampleCpu() {
	const now = Date.now();
	const u = process.cpuUsage(cpu.usage);
	if (now - cpu.at >= 1000) {
		cpu = { usage: process.cpuUsage(), at: now, pct: 100 * (u.user + u.system) / 1000 / (now - cpu.at) };
	}
	return cpu.pct;
}

function health() {
	init();
	const st = store.health();
	const mem = process.memoryUsage();
	const streams = [];
	for (const s of ingestor.streams.values()) {
		streams.push({ robot_id: s.robotId, source: s.source, link: s.link.state,
			last_rx_age_s: s.lastRx.hb || s.lastRx.tlm
				? (Date.now() - Math.max(s.lastRx.hb || 0, s.lastRx.tlm || 0)) / 1000 : null,
			boot_id: s.boot ? s.boot.id : null });
	}
	return {
		ok: st.db_connected !== false && ingestor.brokerConnected !== false,
		instance: cfg.instance,
		uptime_s: Math.round((Date.now() - startedAt) / 1000),
		broker: { connected: ingestor.brokerConnected, host: cfg.mqtt.host, port: cfg.mqtt.port },
		database: { connected: st.db_connected, down_since: st.db_down_since, host: cfg.pg.host,
			    port: cfg.pg.port, database: cfg.pg.database },
		buffer: { bulk: st.queue_bulk, priority: st.queue_priority, inflight: st.inflight, limits: st.limits,
			  loss_pending: st.loss_pending },
		write_latency_ms: st.latency_ms,
		store: st.counters,
		ingest: ingestor.stats,
		streams,
		flow_errors: flowErrors,
		state_saved_at: lastStateSave,
		process: { rss_mb: +(mem.rss / 1048576).toFixed(1), heap_mb: +(mem.heapUsed / 1048576).toFixed(1),
			   cpu_pct: (sampleCpu() === null) ? null : +cpu.pct.toFixed(1) }
	};
}

module.exports = {
	config: cfg,

	ingest(topic, payload, rxMs) {
		init();
		return ingestor.handle(topic, payload, rxMs || Date.now());
	},

	store(ops) {
		init();
		store.enqueueAll(ops);
		return store.queued();
	},

	tick(nowMs) {
		init();
		return ingestor.tick(nowMs || Date.now());
	},

	brokerStatus(connected) {
		init();
		return ingestor.setBrokerConnected(connected, Date.now());
	},

	flowError() {
		flowErrors++;
	},

	stats() {
		init();
		return { ingest: ingestor.stats, queued: store.queued(), db: store.dbUp };
	},

	health,

	healthOps() {
		const h = health();
		return [{ table: 'ingest_health', priority: false, row: {
			at: new Date(), instance: cfg.instance, broker_connected: h.broker.connected,
			db_connected: h.database.connected, queue_bulk: h.buffer.bulk, queue_priority: h.buffer.priority,
			dropped_bulk_total: h.store.droppedBulk, dropped_priority_total: h.store.droppedPriority,
			written_total: h.store.written, write_errors_total: h.store.writeErrors,
			latency_p50_ms: h.write_latency_ms.p50, latency_p95_ms: h.write_latency_ms.p95,
			latency_max_ms: h.write_latency_ms.max, frames_total: h.ingest.frames,
			rejected_total: h.ingest.rejected, rss_mb: h.process.rss_mb, heap_mb: h.process.heap_mb,
			cpu_pct: h.process.cpu_pct, uptime_s: h.uptime_s,
			details: { by_kind: h.ingest.byKind, reject_reasons: h.ingest.rejectReasons,
				   duplicates: h.ingest.duplicates, seq_gap_frames: h.ingest.seqGapFrames,
				   poison_rows: h.store.poisonRows, outages: h.store.outages, inflight: h.buffer.inflight,
				   last_error: h.store.lastError, flow_errors: flowErrors }
		} }];
	},

	saveState() {
		if (!ingestor) { return false; }
		try {
			fs.mkdirSync(path.dirname(cfg.stateFile), { recursive: true });
			fs.writeFileSync(cfg.stateFile + '.tmp', JSON.stringify(ingestor.exportState()));
			fs.renameSync(cfg.stateFile + '.tmp', cfg.stateFile);
			lastStateSave = new Date().toISOString();
			return true;
		} catch (e) {
			log.error('could not save ingest state: ' + e.message);
			return false;
		}
	},

	/* Node-RED stop or re-deploy: keep the in-memory state, flush what can
	   be flushed, save the rest */
	async shutdown() {
		module.exports.saveState();
		if (store) {
			const s = store;
			store = null;
			await s.stop(2000);
		}
	}
};
