/*
 * config.js
 * Backend configuration from environment variables (telemetry/.env, see
 * .env.example for the meaning of each one).  No defaults here invent
 * passwords: a missing password simply fails to log in.
 */
'use strict';

const fs = require('fs');
const os = require('os');
const path = require('path');

const ROOT = path.resolve(__dirname, '..');		/* telemetry/ */
const REPO = path.resolve(ROOT, '..');

function bool(v, dflt) {
	if (v === undefined || v === '') { return dflt; }
	return /^(1|true|yes|on)$/i.test(String(v).trim());
}

function int(v, dflt) {
	const n = parseInt(v, 10);
	return Number.isFinite(n) ? n : dflt;
}

/* Minimal KEY=VALUE reader for tools started without bin/csc14.  Values
   already in the environment win. */
function loadEnvFile(file, env) {
	const target = env || process.env;
	let text;

	try {
		text = fs.readFileSync(file, 'utf8');
	} catch (e) {
		return false;
	}
	for (const line of text.split(/\r?\n/)) {
		const m = /^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*)\s*$/.exec(line);
		if (!m || line.trim().startsWith('#')) { continue; }
		let v = m[2];
		if (/^(['"]).*\1$/.test(v)) { v = v.slice(1, -1); }
		if (target[m[1]] === undefined) { target[m[1]] = v; }
	}
	return true;
}

function load(env) {
	const e = env || process.env;
	const runtimeDir = e.CSC14_RUNTIME_DIR || path.join(ROOT, '.runtime');

	return {
		root: ROOT,
		repo: REPO,
		runtimeDir,
		instance: e.CSC14_INSTANCE || 'nodered',
		robotId: e.CSC14_ROBOT_ID || 'csc14-car',
		topics: {
			live: (e.CSC14_TOPIC_LIVE || 'csc14').replace(/\/+$/, ''),
			sim: (e.CSC14_TOPIC_SIM || 'sim/csc14').replace(/\/+$/, '')
		},
		mqtt: {
			host: e.CSC14_MQTT_HOST || '127.0.0.1',
			port: int(e.CSC14_MQTT_PORT, 1883),
			clientId: e.CSC14_MQTT_CLIENT_ID || 'csc14-nodered'
		},
		decode: {
			irSidesMirrored: bool(e.CSC14_IR_SIDES_MIRRORED, true),
			policy: {
				collisionStops: bool(e.CSC14_SYS_COLLISION_STOPS, true),
				imuRequired: bool(e.CSC14_SYS_IMU_REQUIRED, true),
				usRequired: bool(e.CSC14_SYS_US_REQUIRED, true)
			}
		},
		link: {
			staleAfterMs: int(e.CSC14_STALE_AFTER_S, 3) * 1000,
			offlineAfterMs: int(e.CSC14_OFFLINE_AFTER_S, 10) * 1000
		},
		pg: {
			host: e.CSC14_PG_HOST || '127.0.0.1',
			port: int(e.CSC14_PG_PORT, 5432),
			database: e.CSC14_PG_DATABASE || 'csc14_telemetry',
			user: e.CSC14_PG_WRITER_USER || 'csc14_writer',
			password: e.CSC14_PG_WRITER_PASSWORD || ''
		},
		pgReader: {
			user: e.CSC14_PG_READER_USER || 'csc14_reader',
			password: e.CSC14_PG_READER_PASSWORD || ''
		},
		pgAdmin: {
			user: e.CSC14_PG_ADMIN_USER || os.userInfo().username,
			password: e.CSC14_PG_ADMIN_PASSWORD || undefined
		},
		store: {
			maxBulkRows: int(e.CSC14_BUFFER_BULK_ROWS, 10800),
			maxPriorityRows: int(e.CSC14_BUFFER_PRIORITY_ROWS, 5000),
			batchRows: int(e.CSC14_STORE_BATCH_ROWS, 500),
			retryMinMs: int(e.CSC14_STORE_RETRY_MIN_MS, 500),
			retryMaxMs: int(e.CSC14_STORE_RETRY_MAX_MS, 5000),	/* bench: drain waited on a 10 s back-off */
			storeDecodedJson: bool(e.CSC14_STORE_DECODED_JSON, true)
		},
		stateFile: e.CSC14_STATE_FILE || path.join(runtimeDir, 'nodered', 'ingest-state.json')
	};
}

module.exports = { ROOT, REPO, load, loadEnvFile, bool, int };
