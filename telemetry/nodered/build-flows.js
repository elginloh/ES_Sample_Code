#!/usr/bin/env node
/*
 * build-flows.js -- writes flows.json, the CSC14 ingestion flow.
 * The Function node bodies are kept here as readable code; run
 *   node nodered/build-flows.js
 * after changing them (test/unit/flows.test.js checks flows.json is
 * current).  Node-RED loads flows.json automatically at start.
 */
'use strict';

const fs = require('fs');
const path = require('path');

const Z = 'csc14_tab';
const fn = (id, name, x, y, func, outputs, wires, extra) => Object.assign({
	id, type: 'function', z: Z, name, func: func.trim() + '\n', outputs, timeout: 0, noerr: 0,
	initialize: '', finalize: '', libs: [], x, y, wires
}, extra || {});

const DECODE = `
// Every robot frame arrives as a Buffer (mqtt in: output "a Buffer").
// api.ingest() checks length, CRC, version, type and the range of every
// field, interprets the frame and returns the database operations
// (telemetry/lib/ingest.js).  Malformed frames only produce a
// rejected_frame row; they never reach the measurement tables.
const api = global.get('csc14');
const r = api.ingest(msg.topic, msg.payload, Date.now());
msg.kind = r.kind;
msg.ops = r.ops;
msg.payload = r.decoded || { rejected: r.reason || r.kind, detail: r.detail || null };
const order = ['telemetry', 'heartbeat', 'event', 'ack', 'rejected'];
const out = [null, null, null, null, null, null];
const i = order.indexOf(r.kind);
out[i < 0 ? 5 : i] = msg;
const st = api.stats().ingest;
node.status({ fill: st.rejected ? 'yellow' : 'green', shape: 'dot',
	text: st.frames + ' frames, ' + st.rejected + ' rejected' });
return out;
`;

const STORE = `
// Rows -> bounded in-memory queue -> PostgreSQL (telemetry/lib/store.js):
// parameterized INSERTs, retried with back-off while the database is
// unavailable; only a full buffer drops rows (oldest first, counted).
const api = global.get('csc14');
if (msg.ops && msg.ops.length) {
	api.store(msg.ops);
}
const st = api.stats();
node.status({ fill: st.db === false ? 'red' : (st.db ? 'green' : 'grey'),
	shape: st.db ? 'dot' : 'ring',
	text: (st.db === false ? 'database unavailable, ' : 'database ok, ') + st.queued + ' queued' });
return null;
`;

const LINK = `
// Online / stale / offline from the age of the last heartbeat or
// telemetry frame (CSC14_STALE_AFTER_S, CSC14_OFFLINE_AFTER_S).
msg.ops = global.get('csc14').tick(Date.now());
return msg.ops.length ? msg : null;
`;

const HEALTH = `
// Pipeline health (backend, not robot): buffer, database, broker,
// write latency, memory.  Stored every 10 s in csc14.ingest_health.
msg.ops = global.get('csc14').healthOps();
const h = msg.ops[0].row;
node.status({ fill: (h.db_connected === false || h.broker_connected === false) ? 'red' : 'green',
	shape: 'dot', text: 'p95 ' + (h.latency_p95_ms === null ? '-' : h.latency_p95_ms) + ' ms, ' +
	(h.queue_bulk + h.queue_priority) + ' queued, ' + h.rss_mb + ' MB' });
return msg;
`;

const SAVE = `
// Boot-session and sequence bookkeeping survives a Node-RED restart.
global.get('csc14').saveState();
return null;
`;

const BROKER = `
// The mqtt in nodes report node-red:common.status.connected /
// .disconnected / .connecting.  Losing the broker makes the robot's link
// state UNKNOWN (a backend problem), not OFFLINE.
const text = (msg.status && msg.status.text) || '';
if (!/\\.(connected|disconnected)$/.test(text)) {
	return null;
}
msg.ops = global.get('csc14').brokerStatus(/\\.connected$/.test(text));
return msg.ops.length ? msg : null;
`;

const HTTP_HEALTH = `
// GET /csc14/health -> 200 when broker and database are reachable, else 503
const h = global.get('csc14').health();
msg.payload = h;
msg.statusCode = h.ok ? 200 : 503;
msg.headers = { 'content-type': 'application/json' };
return msg;
`;

const ERRORS = `
global.get('csc14').flowError();
node.warn('csc14 flow error: ' + ((msg.error && msg.error.message) || 'unknown'));
return null;
`;

const debug = (id, name, x, y) => ({
	id, type: 'debug', z: Z, name, active: false, tosidebar: true, console: false, tostatus: false,
	complete: 'payload', targetType: 'msg', statusVal: '', statusType: 'auto', x, y, wires: []
});
const inject = (id, name, seconds, x, y, target) => ({
	id, type: 'inject', z: Z, name, props: [{ p: 'payload' }], repeat: String(seconds), crontab: '',
	once: true, onceDelay: '2', topic: '', payload: '', payloadType: 'date', x, y, wires: [[target]]
});
const comment = (id, name, info, x, y) => ({ id, type: 'comment', z: Z, name, info, x, y, wires: [] });

const flows = [
	{ id: Z, type: 'tab', label: 'CSC14 telemetry', disabled: false,
	  info: 'Pico W -> Mosquitto -> **this flow** -> PostgreSQL -> Grafana.\n\n' +
		'Robot frames are compact binary (protocol v1, app_program/subsystem1/docs/MQTT_BINARY_PROTOCOL.md). ' +
		'The decoding, validation and storage code is in telemetry/lib and is loaded through ' +
		'global context `csc14` (telemetry/nodered/settings.js).\n\n' +
		'This flow only receives. It never publishes on csc14/cmd.' },
	{ id: 'csc14_broker', type: 'mqtt-broker', name: 'Robot broker (CSC14_MQTT_HOST)',
	  broker: '${CSC14_MQTT_HOST}', port: '${CSC14_MQTT_PORT}', clientid: '${CSC14_MQTT_CLIENT_ID}',
	  autoConnect: true, usetls: false, protocolVersion: '4', keepalive: '15', cleansession: true,
	  autoUnsubscribe: true, birthTopic: '', birthQos: '0', birthRetain: 'false', birthPayload: '',
	  birthMsg: {}, closeTopic: '', closeQos: '0', closeRetain: 'false', closePayload: '', closeMsg: {},
	  willTopic: '', willQos: '0', willRetain: 'false', willPayload: '', willMsg: {}, userProps: '',
	  sessionExpiry: '' },
	comment('csc14_c_in', 'Robot output topics, binary frames, QoS 0 (as published)',
		'Live: CSC14_TOPIC_LIVE/+ (csc14/tlm, hb, evt, ack). Simulated: CSC14_TOPIC_SIM/+ ' +
		'(stored with source = simulated, never in a live run).', 200, 40),
	{ id: 'csc14_in_live', type: 'mqtt in', z: Z, name: 'robot frames (live)', topic: '${CSC14_SUB_LIVE}',
	  qos: '0', datatype: 'buffer', broker: 'csc14_broker', nl: false, rap: true, rh: 0, inputs: 0,
	  x: 160, y: 100, wires: [['csc14_decode']] },
	{ id: 'csc14_in_sim', type: 'mqtt in', z: Z, name: 'simulated frames', topic: '${CSC14_SUB_SIM}',
	  qos: '0', datatype: 'buffer', broker: 'csc14_broker', nl: false, rap: true, rh: 0, inputs: 0,
	  x: 150, y: 160, wires: [['csc14_decode']] },
	fn('csc14_decode', 'validate, decode, route', 420, 130, DECODE, 6,
		[['csc14_store', 'csc14_dbg_tlm'], ['csc14_store'], ['csc14_store', 'csc14_dbg_evt'],
		 ['csc14_store', 'csc14_dbg_ack'], ['csc14_store', 'csc14_dbg_rej'], ['csc14_store']],
		{ outputLabels: ['telemetry', 'heartbeat', 'event', 'ack', 'rejected', 'other'],
		  finalize: '// Node-RED stop or re-deploy: flush, save state\nreturn global.get(\'csc14\').shutdown();\n' }),
	fn('csc14_store', 'store in PostgreSQL', 760, 130, STORE, 0, []),
	debug('csc14_dbg_tlm', 'decoded telemetry', 750, 220),
	debug('csc14_dbg_evt', 'decoded events', 740, 260),
	debug('csc14_dbg_ack', 'decoded ACKs', 730, 300),
	debug('csc14_dbg_rej', 'rejected frames', 740, 340),
	comment('csc14_c_time', 'Time-driven: link state, health, saved state', '', 210, 400),
	inject('csc14_t_link', 'every 1 s', 1, 140, 440, 'csc14_link'),
	fn('csc14_link', 'online / stale / offline', 400, 440, LINK, 1, [['csc14_store']]),
	inject('csc14_t_health', 'every 10 s', 10, 140, 500, 'csc14_health'),
	fn('csc14_health', 'pipeline health', 380, 500, HEALTH, 1, [['csc14_store']]),
	inject('csc14_t_save', 'every 5 s', 5, 140, 560, 'csc14_save'),
	fn('csc14_save', 'save ingest state', 380, 560, SAVE, 0, []),
	{ id: 'csc14_status', type: 'status', z: Z, name: 'broker connection',
	  scope: ['csc14_in_live', 'csc14_in_sim'], x: 150, y: 620, wires: [['csc14_broker_fn']] },
	fn('csc14_broker_fn', 'backend <-> broker', 390, 620, BROKER, 1, [['csc14_store']]),
	comment('csc14_c_http', 'GET /csc14/health (used by bin/csc14 status)', '', 220, 680),
	{ id: 'csc14_http_in', type: 'http in', z: Z, name: '', url: '/csc14/health', method: 'get',
	  upload: false, swaggerDoc: '', x: 160, y: 720, wires: [['csc14_http_fn']] },
	fn('csc14_http_fn', 'health JSON', 390, 720, HTTP_HEALTH, 1, [['csc14_http_out']]),
	{ id: 'csc14_http_out', type: 'http response', z: Z, name: '', statusCode: '', headers: {},
	  x: 590, y: 720, wires: [] },
	{ id: 'csc14_catch', type: 'catch', z: Z, name: 'flow errors', scope: null, uncaught: false,
	  x: 150, y: 780, wires: [['csc14_err_fn']] },
	fn('csc14_err_fn', 'count + log', 360, 780, ERRORS, 0, [])
];

const file = path.join(__dirname, 'flows.json');
const text = JSON.stringify(flows, null, 4) + '\n';
if (require.main === module) {
	fs.writeFileSync(file, text);
	console.log('wrote ' + file);
}
module.exports = { flows, text };
