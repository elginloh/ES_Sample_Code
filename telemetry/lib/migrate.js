/*
 * migrate.js
 * Creates the database and roles if they are missing, applies the SQL
 * migrations in db/migrations that are not yet applied (each in its own
 * transaction), refreshes the lookup tables from the protocol codec and
 * grants:  writer = SELECT/INSERT/UPDATE (no DELETE), reader = SELECT.
 * Existing data is never dropped; an applied migration whose file changed
 * stops the run instead of being re-applied.
 */
'use strict';

const crypto = require('crypto');
const fs = require('fs');
const path = require('path');
const { Client } = require('pg');
const { ROOT } = require('./config');
const protocol = require('./protocol');

const MIGRATIONS = path.join(ROOT, 'db', 'migrations');

async function connect(cfg, database) {
	const c = new Client({ host: cfg.pg.host, port: cfg.pg.port, database,
		user: cfg.pgAdmin.user, password: cfg.pgAdmin.password, application_name: 'csc14-migrate' });
	await c.connect();
	return c;
}

async function ensureRole(c, name, password, log) {
	const id = c.escapeIdentifier(name);
	const r = await c.query('SELECT 1 FROM pg_roles WHERE rolname = $1', [name]);
	if (r.rowCount === 0) {
		await c.query(`CREATE ROLE ${id} LOGIN`);
		log(`created role ${name}`);
	}
	if (password) {
		await c.query(`ALTER ROLE ${id} WITH LOGIN PASSWORD ${c.escapeLiteral(password)}`);
	} else {
		log(`warning: no password set for ${name} (only works with trust authentication)`);
	}
}

async function migrate(cfg, opts) {
	const log = (opts && opts.log) || console.log;
	const dbName = cfg.pg.database;
	const applied = [];

	/* database and roles, from the maintenance database */
	const admin = await connect(cfg, (opts && opts.maintenanceDb) || 'postgres');
	try {
		await ensureRole(admin, cfg.pg.user, cfg.pg.password, log);
		await ensureRole(admin, cfg.pgReader.user, cfg.pgReader.password, log);
		const r = await admin.query('SELECT 1 FROM pg_database WHERE datname = $1', [dbName]);
		if (r.rowCount === 0) {
			await admin.query(`CREATE DATABASE ${admin.escapeIdentifier(dbName)}`);
			log(`created database ${dbName}`);
		}
	} finally {
		await admin.end();
	}

	const c = await connect(cfg, dbName);
	try {
		await c.query('CREATE SCHEMA IF NOT EXISTS csc14');
		await c.query(`CREATE TABLE IF NOT EXISTS csc14.schema_migration (
			version text PRIMARY KEY, checksum text NOT NULL,
			applied_at timestamptz NOT NULL DEFAULT now())`);
		const done = new Map((await c.query('SELECT version, checksum FROM csc14.schema_migration'))
			.rows.map((x) => [x.version, x.checksum]));
		const files = fs.readdirSync(MIGRATIONS).filter((f) => /^\d+_.*\.sql$/.test(f)).sort();

		for (const f of files) {
			const sql = fs.readFileSync(path.join(MIGRATIONS, f), 'utf8');
			const sum = crypto.createHash('sha256').update(sql).digest('hex');
			if (done.has(f)) {
				if (done.get(f) !== sum) {
					throw new Error(`${f} was changed after it was applied; add a new migration instead`);
				}
				continue;
			}
			await c.query('BEGIN');
			try {
				await c.query(sql);
				await c.query('INSERT INTO csc14.schema_migration (version, checksum) VALUES ($1, $2)', [f, sum]);
				await c.query('COMMIT');
			} catch (e) {
				await c.query('ROLLBACK');
				throw new Error(`${f}: ${e.message}`);
			}
			applied.push(f);
			log(`applied ${f}`);
		}

		/* lookup tables from the codec (one source of truth) */
		await c.query('BEGIN');
		for (const [enumName, code, label] of protocol.enumLabelRows()) {
			await c.query(`INSERT INTO csc14.enum_label (enum_name, code, label) VALUES ($1, $2, $3)
				ON CONFLICT (enum_name, code) DO UPDATE SET label = EXCLUDED.label`, [enumName, code, label]);
		}
		for (const [bit, faultName, owner, severity] of protocol.faultBitRows(cfg.decode.policy)) {
			await c.query(`INSERT INTO csc14.fault_bit (bit, name, owner, severity) VALUES ($1, $2, $3, $4)
				ON CONFLICT (bit) DO UPDATE SET name = EXCLUDED.name, owner = EXCLUDED.owner,
				severity = EXCLUDED.severity`, [bit, faultName, owner, severity]);
		}
		await c.query('COMMIT');

		const w = c.escapeIdentifier(cfg.pg.user);
		const rd = c.escapeIdentifier(cfg.pgReader.user);
		const db = c.escapeIdentifier(dbName);
		for (const sql of [
			`GRANT CONNECT ON DATABASE ${db} TO ${w}, ${rd}`,
			`GRANT USAGE ON SCHEMA csc14 TO ${w}, ${rd}`,
			`GRANT SELECT, INSERT, UPDATE ON ALL TABLES IN SCHEMA csc14 TO ${w}`,
			`GRANT SELECT ON ALL TABLES IN SCHEMA csc14 TO ${rd}`,
			`REVOKE INSERT, UPDATE, DELETE, TRUNCATE ON csc14.schema_migration FROM ${w}`,
			`ALTER DEFAULT PRIVILEGES IN SCHEMA csc14 GRANT SELECT, INSERT, UPDATE ON TABLES TO ${w}`,
			`ALTER DEFAULT PRIVILEGES IN SCHEMA csc14 GRANT SELECT ON TABLES TO ${rd}`
		]) {
			await c.query(sql);
		}
	} finally {
		await c.end();
	}
	return applied;
}

module.exports = { migrate, MIGRATIONS };
