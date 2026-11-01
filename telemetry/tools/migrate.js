#!/usr/bin/env node
/*
 * migrate.js -- create/upgrade the telemetry database (bin/csc14 migrate).
 */
'use strict';

const path = require('path');
const config = require('../lib/config');
const { migrate } = require('../lib/migrate');

config.loadEnvFile(path.join(config.ROOT, '.env'));
const cfg = config.load();

migrate(cfg).then((applied) => {
	console.log(applied.length ? `database ${cfg.pg.database} is up to date (${applied.length} migration(s) applied)`
				   : `database ${cfg.pg.database} was already up to date`);
}).catch((e) => {
	console.error('migration failed: ' + e.message);
	process.exitCode = 1;
});
