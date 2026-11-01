/*
 * grafana.js -- run every dashboard panel query through Grafana's own query
 * API (/api/ds/query), as the browser would after substituting the
 * dashboard variables.  Used by the integration test and tools/bench.js.
 */
'use strict';

function collectPanels(dashboard) {
	const out = [];
	for (const p of dashboard.panels) {
		if (p.type === 'row') { continue; }
		for (const t of p.targets || []) { out.push({ title: p.title, sql: t.rawSql, format: t.format }); }
	}
	return out;
}

function substitute(sql, vars) {
	return sql
		.replace(/\$\{robot:sqlstring\}/g, `'${vars.robot}'`)
		.replace(/\$\{source:sqlstring\}/g, `'${vars.source}'`)
		.replace(/\$\{run:sqlstring\}/g, `'${vars.run || 'all'}'`)
		.replace(/\$__interval_ms\b/g, String(vars.intervalMs || 1000))
		.replace(/\$__interval\b/g, `'${vars.interval || '1s'}'`);
}

async function queryAll(baseUrl, auth, dashboard, vars, fromMs, toMs) {
	const results = [];
	for (const p of collectPanels(dashboard)) {
		const body = {
			from: String(fromMs), to: String(toMs),
			queries: [{ refId: 'A', datasource: { type: 'grafana-postgresql-datasource', uid: 'csc14-pg' },
				rawSql: substitute(p.sql, vars), format: p.format, rawQuery: true, editorMode: 'code',
				intervalMs: 1000, maxDataPoints: 1000 }]
		};
		const t0 = process.hrtime.bigint();
		const res = await fetch(baseUrl + '/api/ds/query', {
			method: 'POST', headers: { 'content-type': 'application/json', authorization: auth },
			body: JSON.stringify(body)
		});
		const ms = Number(process.hrtime.bigint() - t0) / 1e6;
		const json = await res.json().catch(() => ({}));
		const r = json.results && json.results.A;
		const frames = (r && r.frames) || [];
		const rows = frames.reduce((n, f) => n + ((f.data && f.data.values && f.data.values[0]) ? f.data.values[0].length : 0), 0);
		results.push({ title: p.title, status: res.status, error: (r && r.error) || json.message || null, ms, rows });
	}
	return results;
}

module.exports = { collectPanels, substitute, queryAll };
