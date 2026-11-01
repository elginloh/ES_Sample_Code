/*
 * Generated artifacts: flows.json and the dashboard JSON match their
 * generators, Function nodes compile, the flow cannot publish, the
 * dashboard only uses known variables and a refresh Grafana allows.
 */
'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('fs');
const path = require('path');
const vm = require('vm');

const ROOT = path.resolve(__dirname, '..', '..');
const flowsGen = require('../../nodered/build-flows.js');
const dashGen = require('../../grafana/build-dashboard.js');

test('flows.json and the dashboard JSON are generated from the current sources', () => {
	assert.equal(fs.readFileSync(path.join(ROOT, 'nodered', 'flows.json'), 'utf8'), flowsGen.text,
		'run: node nodered/build-flows.js');
	assert.equal(fs.readFileSync(path.join(ROOT, 'grafana', 'dashboards', 'csc14-robot.json'), 'utf8'), dashGen.text,
		'run: node grafana/build-dashboard.js');
});

test('flow: subscribes as Buffer, never publishes, every Function node compiles', () => {
	const flows = flowsGen.flows;
	const ins = flows.filter((n) => n.type === 'mqtt in');
	assert.equal(ins.length, 2);
	for (const n of ins) {
		assert.equal(n.datatype, 'buffer');
		assert.match(n.topic, /^\$\{CSC14_SUB_(LIVE|SIM)\}$/);
	}
	assert.equal(flows.filter((n) => n.type === 'mqtt out').length, 0, 'the ingestion flow must not publish');
	const ids = new Set(flows.map((n) => n.id));
	for (const n of flows) {
		for (const w of n.wires || []) { for (const id of w) { assert.ok(ids.has(id), `${n.id} -> ${id}`); } }
		if (n.type === 'function') {
			for (const body of [n.func, n.finalize]) {
				if (body) { new vm.Script(`(async function (msg, node, global) {\n${body}\n})`); }
			}
			assert.equal(n.outputs, (n.wires || []).length);
		}
	}
	assert.deepEqual(flows.find((n) => n.type === 'status').scope.sort(), ins.map((n) => n.id).sort());
});

test('dashboard: provisioned data source, known variables, allowed refresh', () => {
	const d = dashGen.dashboard;
	const ini = fs.readFileSync(path.join(ROOT, 'grafana', 'grafana.ini'), 'utf8');
	const min = /min_refresh_interval\s*=\s*(\d+)s/.exec(ini)[1];
	assert.ok(parseInt(d.refresh, 10) >= Number(min), 'refresh must not be below min_refresh_interval');
	assert.ok(d.timepicker.refresh_intervals.every((r) => !/^\d+s$/.test(r) || parseInt(r, 10) >= Number(min)));
	const known = /\$\{(robot|source|run):sqlstring\}|\$__(timeFilter|timeFrom|timeTo)\(|\$__interval_ms/g;
	const titles = new Set();
	for (const p of d.panels.filter((x) => x.type !== 'row')) {
		assert.equal(p.datasource.uid, 'csc14-pg', p.title);
		assert.ok(!titles.has(p.title), 'duplicate title ' + p.title);
		titles.add(p.title);
		for (const t of p.targets) {
			const rest = t.rawSql.replace(known, '');
			assert.doesNotMatch(rest, /\$/, `${p.title}: unknown variable in ${rest.match(/\$\S*/)}`);
			assert.doesNotMatch(t.rawSql, /\b(insert|update|delete|drop|alter|truncate)\b/i, p.title);
		}
	}
});
