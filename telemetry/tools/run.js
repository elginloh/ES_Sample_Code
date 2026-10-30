#!/usr/bin/env node
/*
 * run.js -- experiment runs (bin/csc14 run ...).  Only touches the
 * database: starting or ending a run never sends anything to the robot.
 *
 *   run start "name" [--notes "..."] [--description "..."] [--simulated]
 *   run end [--notes "..."] [--simulated]
 *   run list [--simulated] [--limit 20]
 *   run show [run-id]          (default: the newest run)
 *   run active
 */
'use strict';

const { execFileSync } = require('child_process');
const fs = require('fs');
const os = require('os');
const path = require('path');
const { Client } = require('pg');
const config = require('../lib/config');

config.loadEnvFile(path.join(config.ROOT, '.env'));
const cfg = config.load();
const argv = process.argv.slice(2);

function opt(name) {
	const i = argv.indexOf('--' + name);
	return (i < 0) ? undefined : (argv[i + 1] === undefined || argv[i + 1].startsWith('--') ? true : argv[i + 1]);
}

const source = opt('simulated') ? 'simulated' : 'live';

/* What we know about the firmware: the repository state, which is NOT proof
   of what is flashed on the robot */
function firmwareInfo() {
	const git = (...a) => {
		try { return execFileSync('git', a, { cwd: config.REPO, encoding: 'utf8', stdio: ['ignore', 'pipe', 'ignore'] }).trim(); }
		catch (e) { return null; }
	};
	const info = { note: 'repository state when the run started; not proof of the flashed image' };
	info.commit = git('rev-parse', '--short', 'HEAD');
	const dirty = git('status', '--porcelain', '--untracked-files=no', '--', '../app_program', '../lib', '../config');
	info.app_sources_modified = dirty === null ? null : dirty.length > 0;
	try {
		info.build_profile = fs.readFileSync(path.join(config.REPO, 'build_make', '.build_profile'), 'utf8').trim();
		const uf2 = fs.readdirSync(path.join(config.REPO, 'build_make')).filter((f) => f.endsWith('.uf2'))
			.map((f) => ({ f, t: fs.statSync(path.join(config.REPO, 'build_make', f)).mtime }))
			.sort((a, b) => b.t - a.t)[0];
		if (uf2) { info.newest_uf2 = { file: uf2.f, built: uf2.t.toISOString() }; }
	} catch (e) { /* no build folder */ }
	info.backend_interpretation = { ir_sides_mirrored: cfg.decode.irSidesMirrored, policy: cfg.decode.policy };
	return info;
}

async function main() {
	const c = new Client({ host: cfg.pg.host, port: cfg.pg.port, database: cfg.pg.database,
		user: cfg.pg.user, password: cfg.pg.password, application_name: 'csc14-run' });
	await c.connect();
	try {
		switch (argv[0]) {
		case 'start': {
			const name = argv[1];
			if (!name || name.startsWith('--')) { throw new Error('usage: run start "name" [--notes ...]'); }
			const r = await c.query('SELECT csc14.start_run($1, $2, $3, $4, $5, $6, $7) AS id',
				[cfg.robotId, source, name, opt('description') || null, opt('notes') || null,
				 firmwareInfo(), os.userInfo().username]);
			console.log(`started ${source} run "${name}" for ${cfg.robotId}: ${r.rows[0].id}`);
			console.log('(nothing was sent to the robot; rows received from now on belong to this run)');
			break;
		}
		case 'end': {
			const r = await c.query('SELECT csc14.end_run($1, $2, $3) AS id', [cfg.robotId, source, opt('notes') || null]);
			if (!r.rows[0].id) {
				console.log(`no active ${source} run for ${cfg.robotId}`);
				break;
			}
			console.log(`ended run ${r.rows[0].id}`);
			await show(c, r.rows[0].id);
			break;
		}
		case 'list': {
			const r = await c.query(`SELECT s.run_id, s.name, s.started_at, s.ended_at, s.tlm_frames, s.humps, s.hump_max_mm
				FROM (SELECT run_id FROM csc14.run WHERE source = $1 AND robot_id = $2 ORDER BY started_at DESC LIMIT $3) r
				CROSS JOIN LATERAL csc14.run_summary(r.run_id) s ORDER BY s.started_at DESC`,
			[source, cfg.robotId, Number(opt('limit') || 20)]);
			for (const x of r.rows) {
				console.log(`${x.run_id}  ${x.started_at.toISOString()}  ${x.ended_at ? 'ended ' : 'ACTIVE'}  ` +
					`${String(x.tlm_frames).padStart(6)} frames  humps ${x.humps} (max ${x.hump_max_mm ?? '-'} mm)  ${x.name}`);
			}
			if (r.rowCount === 0) { console.log(`no ${source} runs yet`); }
			break;
		}
		case 'show': {
			let id = argv[1];
			if (!id || id.startsWith('--')) {
				const r = await c.query('SELECT run_id FROM csc14.run WHERE robot_id = $1 AND source = $2 ORDER BY started_at DESC LIMIT 1',
					[cfg.robotId, source]);
				if (!r.rowCount) { console.log('no runs yet'); break; }
				id = r.rows[0].run_id;
			}
			await show(c, id);
			break;
		}
		case 'active': {
			const r = await c.query('SELECT name, source, started_at FROM csc14.run WHERE robot_id = $1 AND ended_at IS NULL',
				[cfg.robotId]);
			for (const x of r.rows) {
				console.log(`Run         ACTIVE ${x.source} "${x.name}" since ${x.started_at.toISOString()}`);
			}
			if (!r.rowCount) { console.log('Run         none active (data is still stored, without a run)'); }
			break;
		}
		default:
			console.log('usage: run start "name" | run end | run list | run show [id] | run active  [--simulated]');
		}
	} finally {
		await c.end();
	}
}

async function show(c, id) {
	const r = await c.query('SELECT * FROM csc14.run_summary($1)', [id]);
	if (!r.rowCount) { console.log('no such run'); return; }
	const s = r.rows[0];
	const lines = [
		['run', `${s.name} (${s.run_id}, ${s.source})`],
		['time', `${s.started_at.toISOString()} -> ${s.ended_at ? s.ended_at.toISOString() : 'active'} (${s.duration_s} s)`],
		['telemetry', `${s.tlm_frames} frames received of ~${s.tlm_expected} expected at 2 Hz; ` +
			`${s.tlm_missing_known} missing by sequence; longest receive gap ${s.max_rx_gap_s} s`],
		['robot', `${s.boot_sessions} boot session(s), ${s.robot_restarts} restart(s) detected; ` +
			`distance ${s.distance_m} m (odometer)`],
		['humps', `${s.humps} measured in the run, highest ${s.hump_max_mm ?? '-'} mm` +
			(Number(s.humps_without_height) ? `, ${s.humps_without_height} without height` : '')],
		['barcodes', `${s.barcode_reads_valid} valid, ${s.barcode_reads_invalid} invalid reads`],
		['obstacles', `${s.avoid_success} avoided, ${s.avoid_failed} failed; ${s.collisions} collision(s)`],
		['faults set', s.faults_raised.length ? s.faults_raised.join(', ') : 'none'],
		['link', `${s.link_outages} stale/offline period(s)`],
		['notes', s.notes.length ? s.notes.join('; ') : 'none: complete as far as the backend can tell']
	];
	for (const [k, v] of lines) { console.log(`  ${k.padEnd(10)} ${v}`); }
}

main().catch((e) => {
	console.error('run: ' + e.message);
	process.exitCode = 1;
});
