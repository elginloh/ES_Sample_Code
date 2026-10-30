#!/usr/bin/env node
/*
 * prune.js -- optional, explicit retention (bin/csc14 prune ...).  Nothing
 * is ever deleted automatically; without --yes this only counts.
 *
 *   prune --decoded-older-than DAYS     drop the JSONB copy of older telemetry
 *                                       (frame bytes and typed columns stay, so
 *                                       nothing is lost: decoded can be rebuilt)
 *   prune --health-older-than DAYS      delete old pipeline-health rows
 *   prune --unassigned-older-than DAYS  delete telemetry/heartbeat rows that
 *                                       belong to NO run (data loss, on purpose)
 *   add --yes to do it
 *
 * Uses the administrator account (the writer role cannot delete).
 */
'use strict';

const path = require('path');
const { Client } = require('pg');
const config = require('../lib/config');

config.loadEnvFile(path.join(config.ROOT, '.env'));
const cfg = config.load();

function days(name) {
	const i = process.argv.indexOf('--' + name);
	if (i < 0) { return null; }
	const d = Number(process.argv[i + 1]);
	if (!Number.isFinite(d) || d < 1) { throw new Error(`--${name} needs a number of days >= 1`); }
	return d;
}

async function main() {
	const yes = process.argv.includes('--yes');
	const jobs = [];
	const dec = days('decoded-older-than');
	const hea = days('health-older-than');
	const una = days('unassigned-older-than');
	if (dec) {
		jobs.push(['telemetry JSONB copies older than ' + dec + ' d',
			"SELECT count(*) FROM csc14.telemetry WHERE decoded IS NOT NULL AND received_at < now() - $1 * interval '1 day'",
			"UPDATE csc14.telemetry SET decoded = NULL WHERE decoded IS NOT NULL AND received_at < now() - $1 * interval '1 day'", dec]);
	}
	if (hea) {
		jobs.push(['health rows older than ' + hea + ' d',
			"SELECT count(*) FROM csc14.ingest_health WHERE at < now() - $1 * interval '1 day'",
			"DELETE FROM csc14.ingest_health WHERE at < now() - $1 * interval '1 day'", hea]);
	}
	if (una) {
		for (const t of ['telemetry', 'heartbeat']) {
			jobs.push([`${t} rows outside any run older than ${una} d`,
				`SELECT count(*) FROM csc14.${t} WHERE run_id IS NULL AND received_at < now() - $1 * interval '1 day'`,
				`DELETE FROM csc14.${t} WHERE run_id IS NULL AND received_at < now() - $1 * interval '1 day'`, una]);
		}
	}
	if (!jobs.length) {
		console.log('usage: prune --decoded-older-than D | --health-older-than D | --unassigned-older-than D  [--yes]');
		return;
	}
	const c = new Client({ host: cfg.pg.host, port: cfg.pg.port, database: cfg.pg.database,
		user: cfg.pgAdmin.user, password: cfg.pgAdmin.password, application_name: 'csc14-prune' });
	await c.connect();
	try {
		for (const [what, count, act, d] of jobs) {
			const n = (await c.query(count, [d])).rows[0].count;
			if (!yes) {
				console.log(`would change ${n} ${what}`);
				continue;
			}
			const r = await c.query(act, [d]);
			console.log(`changed ${r.rowCount} ${what}`);
		}
		if (yes) {
			await c.query('VACUUM (ANALYZE) csc14.telemetry');
			console.log('space is reused by new rows (VACUUM done)');
		} else {
			console.log('nothing changed; add --yes to apply');
		}
	} finally {
		await c.end();
	}
}

main().catch((e) => {
	console.error('prune: ' + e.message);
	process.exitCode = 1;
});
