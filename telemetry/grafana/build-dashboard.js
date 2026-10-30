#!/usr/bin/env node
/*
 * build-dashboard.js -- writes dashboards/csc14-robot.json.
 * Panels and their SQL are kept here in readable form; run
 *   node grafana/build-dashboard.js
 * after changing them (test/unit/dashboard.test.js checks the JSON is
 * current).  Grafana provisions the JSON at start.
 *
 * Conventions used by every query:
 *   - robot, source (live / simulated) and run come from the variables;
 *     run 'all' means "everything in the time range".
 *   - time series are grouped into $__interval buckets with NULL fill, so
 *     missing data shows as a gap, never as a line across an outage.
 *   - "current" panels say STALE when the newest row is older than 5 s.
 *   - received_at is when the backend received a frame, not a robot clock.
 */
'use strict';

const fs = require('fs');
const path = require('path');

const DS = { type: 'grafana-postgresql-datasource', uid: 'csc14-pg' };
const ROBOT = "robot_id = ${robot:sqlstring} AND source = ${source:sqlstring}";
const RUN = "(${run:sqlstring} = 'all' OR run_id = (CASE WHEN ${run:sqlstring} = 'all' THEN NULL ELSE ${run:sqlstring} END)::uuid)";
const WHERE = `${ROBOT} AND $__timeFilter(received_at) AND ${RUN}`;
const LAST = (col, alias) => `(array_agg(${col} ORDER BY received_at DESC))[1] AS "${alias}"`;

/* Time series with one row per $__interval bucket over the WHOLE time
   range: a bucket without data is a NULL, so an outage is a gap at every
   zoom level (the data source's minimum interval is 1 s, two telemetry
   frames, so normal jitter does not create gaps).  cols: [expr AS "name"]. */
function series(cols, from, where, intervalMs, timeCol) {
	const ms = intervalMs || '$__interval_ms';
	const tc = timeCol || 'received_at';
	const names = cols.map((c) => /AS ("[^"]+")\s*$/.exec(c)[1]);
	return `WITH d AS (SELECT floor(extract(epoch FROM ${tc}) * 1000 / ${ms})::bigint AS k, ${cols.join(', ')}
	  FROM ${from} WHERE ${where || WHERE} GROUP BY 1),
	s AS (SELECT generate_series(floor(extract(epoch FROM $__timeFrom()::timestamptz) * 1000 / ${ms})::bigint,
	  floor(extract(epoch FROM $__timeTo()::timestamptz) * 1000 / ${ms})::bigint) AS k)
	SELECT to_timestamp(s.k * ${ms} / 1000.0) AS time, ${names.map((n) => 'd.' + n).join(', ')}
	FROM s LEFT JOIN d USING (k) ORDER BY 1`;
}
const LATEST = (cols, table) => `SELECT ${cols} FROM ${table || 'csc14.v_telemetry'} WHERE ${ROBOT} ORDER BY received_at DESC LIMIT 1`;
const FRESH = (expr, label) => `CASE WHEN now() - received_at > interval '5 seconds' THEN 'STALE ' || ` +
	`round(extract(epoch FROM now() - received_at)) || ' s: ' || coalesce((${expr})::text, '-') ` +
	`ELSE coalesce((${expr})::text, '-') END AS "${label}"`;

let nextId = 1;
let y = 0;

function target(sql, format) {
	return { refId: 'A', datasource: DS, editorMode: 'code', format: format || 'table', rawQuery: true, rawSql: sql.trim() };
}

function place(w, h, x) {
	return { h, w, x, y };
}

function panel(type, title, description, sql, w, h, x, extra) {
	return Object.assign({
		id: nextId++, type, title, description, datasource: DS, gridPos: place(w, h, x),
		targets: [target(sql)], fieldConfig: { defaults: {}, overrides: [] }, options: {}
	}, extra || {});
}

function row(title) {
	const r = { id: nextId++, type: 'row', title, collapsed: false, gridPos: { h: 1, w: 24, x: 0, y }, panels: [] };
	y += 1;
	return r;
}

/* Text mapping with colours */
function mapText(map) {
	const options = {};
	let i = 0;
	for (const [k, color] of Object.entries(map)) {
		options[k] = { color, index: i++, text: k };
	}
	return [{ type: 'value', options }];
}

function stat(title, description, sql, w, h, x, defaults, options) {
	return panel('stat', title, description, sql, w, h, x, {
		fieldConfig: { defaults: Object.assign({ color: { mode: 'thresholds' },
			thresholds: { mode: 'absolute', steps: [{ color: 'text', value: null }] } }, defaults || {}), overrides: [] },
		options: Object.assign({ reduceOptions: { calcs: ['lastNotNull'], fields: '/.*/', values: false },
			colorMode: 'background', graphMode: 'none', justifyMode: 'center', textMode: 'value_and_name',
			wideLayout: true, showPercentChange: false, orientation: 'auto' }, options || {})
	});
}

function timeseries(title, description, sql, w, h, x, unit, extraDefaults, overrides) {
	return panel('timeseries', title, description, sql, w, h, x, {
		fieldConfig: {
			defaults: Object.assign({ unit: unit || 'none', color: { mode: 'palette-classic' },
				custom: { drawStyle: 'line', lineWidth: 1, fillOpacity: 0, showPoints: 'never',
					  spanNulls: false, insertNulls: false, lineInterpolation: 'linear',
					  axisPlacement: 'auto' } }, extraDefaults || {}),
			overrides: overrides || []
		},
		options: { legend: { displayMode: 'list', placement: 'bottom', showLegend: true },
			   tooltip: { mode: 'multi', sort: 'none' } }
	});
}

function timeline(title, description, sql, w, h, x, mappings) {
	return panel('state-timeline', title, description, sql, w, h, x, {
		fieldConfig: { defaults: { color: { mode: 'thresholds' }, mappings: mappings || [],
			thresholds: { mode: 'absolute', steps: [{ color: 'blue', value: null }] },
			custom: { lineWidth: 0, fillOpacity: 75, spanNulls: false, insertNulls: false } }, overrides: [] },
		options: { mergeValues: true, showValue: 'auto', alignValue: 'left', rowHeight: 0.85,
			   legend: { displayMode: 'list', placement: 'bottom', showLegend: false },
			   tooltip: { mode: 'single', sort: 'none' } }
	});
}

function table(title, description, sql, w, h, x, overrides) {
	return panel('table', title, description, sql, w, h, x, {
		fieldConfig: { defaults: { custom: { align: 'auto', cellOptions: { type: 'auto' }, inspect: true } },
			       overrides: overrides || [] },
		options: { showHeader: true, cellHeight: 'sm', footer: { show: false } }
	});
}

const LINK_COLORS = mapText({ ONLINE: 'green', STALE: 'orange', OFFLINE: 'red', UNKNOWN: 'purple',
	'NO DATA': 'text', NEVER_SEEN: 'text', 'BACKEND NOT UPDATING': 'purple' });
const MODE_COLORS = mapText({ INIT: 'blue', NORMAL: 'green', NAV_ACTION: 'purple', OBSTACLE: 'orange',
	SAFE_STOP: 'red' });
const SEV_OVERRIDE = { matcher: { id: 'byName', options: 'severity' }, properties: [
	{ id: 'mappings', value: mapText({ critical: 'red', warning: 'orange', info: 'text' }) },
	{ id: 'custom.cellOptions', value: { type: 'color-text' } }] };
const MESSAGE_WIDE = { matcher: { id: 'byName', options: 'message' }, properties: [{ id: 'custom.width', value: 720 }] };

const panels = [];
const add = (...p) => panels.push(...p);

/* ------------------------------------------------------------------ */
add(row('Overview'));
add(stat('Robot link (backend view)',
	'From the age of the newest heartbeat or telemetry: ONLINE <= 3 s, STALE <= 10 s, OFFLINE beyond ' +
	'(CSC14_STALE_AFTER_S / CSC14_OFFLINE_AFTER_S). UNKNOWN = the backend lost its broker connection, ' +
	'a backend problem rather than a robot fault. BACKEND NOT UPDATING = Node-RED stopped writing.',
	`SELECT CASE WHEN updated_at IS NULL THEN 'NO DATA'
	  WHEN now() - updated_at > interval '15 seconds' THEN 'BACKEND NOT UPDATING'
	  ELSE state END AS "link",
	  round(extract(epoch FROM now() - last_rx_at)::numeric, 1) AS "last frame (s ago)"
	FROM (SELECT 1) one LEFT JOIN csc14.robot_link ON ${ROBOT}`,
	6, 4, 0, { mappings: LINK_COLORS, unit: 'none' }));
add(stat('Operating mode', 'Newest telemetry snapshot (500 ms). STALE = older than 5 s: not live.',
	LATEST(`${FRESH('mode_label', 'mode')}`), 5, 4, 6, { mappings: MODE_COLORS }, { textMode: 'value' }));
add(stat('Critical fault active', 'Newest snapshot: any active fault classed critical under the firmware ' +
	'policy flags (CSC14_SYS_*). LINE_RECOVERY is degraded, as in sys_state_logic.c.',
	LATEST(`${FRESH("CASE WHEN critical THEN 'YES' ELSE 'no' END", 'critical')}`), 4, 4, 11,
	{ mappings: mapText({ YES: 'red', no: 'green' }) }, { textMode: 'value' }));
add(stat('Backend pipeline', 'Node-RED -> PostgreSQL health (csc14.ingest_health, every 10 s). Separate from robot faults.',
	`SELECT CASE WHEN at IS NULL THEN 'NO DATA'
	  WHEN now() - at > interval '30 seconds' THEN 'NOT REPORTING'
	  WHEN broker_connected IS FALSE THEN 'BROKER DISCONNECTED'
	  WHEN queue_bulk + queue_priority > 100 THEN 'BUFFERING ' || (queue_bulk + queue_priority)
	  ELSE 'OK' END AS "backend", latency_p95_ms AS "write p95 (ms)"
	FROM (SELECT 1) one LEFT JOIN LATERAL (SELECT * FROM csc14.ingest_health ORDER BY at DESC LIMIT 1) h ON true`,
	5, 4, 15, { mappings: mapText({ OK: 'green', 'NOT REPORTING': 'red', 'BROKER DISCONNECTED': 'red',
		'NO DATA': 'text' }) }));
add(stat('Robot-reported link', 'comm_state in the newest telemetry and the reconnect count in the newest ' +
	'heartbeat (4 bits, saturates at 15). Frames queued during an outage arrive later and show the outage states.',
	`SELECT t."robot comm", h.reconnects AS "reconnects"
	FROM (${LATEST(FRESH('comm_state_label', 'robot comm'))}) t
	LEFT JOIN (SELECT reconnects FROM csc14.heartbeat WHERE ${ROBOT} ORDER BY received_at DESC LIMIT 1) h ON true`,
	4, 4, 20, { mappings: mapText({ ONLINE: 'green', RETRY_WAIT: 'orange', RECOVERY: 'orange',
		DISABLED: 'red' }) }, { text: { titleSize: 12, valueSize: 18 } }));
y += 4;
add(table('Active faults (newest snapshot)',
	'Active fault bits from the newest telemetry snapshot. The first FAULTS event after the robot connects ' +
	'lists faults since boot, not active ones, so it is not used here.',
	`WITH t AS (SELECT fault_mask, received_at,
	    CASE WHEN now() - received_at > interval '5 seconds' THEN 'STALE snapshot' ELSE 'live' END AS freshness
	  FROM csc14.telemetry WHERE ${ROBOT} ORDER BY received_at DESC LIMIT 1)
	SELECT fb.bit, fb.name AS fault, fb.severity, fb.owner, t.freshness
	FROM t JOIN csc14.fault_bit fb ON (t.fault_mask >> fb.bit) & 1 = 1
	UNION ALL SELECT NULL, 'none active', '-', '-', t.freshness FROM t WHERE t.fault_mask = 0
	ORDER BY 1 NULLS FIRST`,
	12, 6, 0, [{ matcher: { id: 'byName', options: 'severity' }, properties: [{ id: 'mappings',
		value: mapText({ CRITICAL_CONDITION: 'red', CRITICAL_EVENT: 'red', DEGRADED: 'orange' }) },
		{ id: 'custom.cellOptions', value: { type: 'color-text' } }] }]));
add(timeline('Mode over time', 'Operating mode from each telemetry snapshot (last value per bucket).',
	series([LAST('mode_label', 'mode'), LAST('motion_state_label', 'motion')], 'csc14.v_telemetry'), 12, 6, 12,
	mapText({ INIT: 'blue', NORMAL: 'green', NAV_ACTION: 'purple', OBSTACLE: 'orange', SAFE_STOP: 'red',
		IDLE: 'text', STRAIGHT: 'green', TURNING: 'blue', FAULT: 'red' })));
y += 6;

/* ------------------------------------------------------------------ */
add(row('Motion'));
add(timeseries('Wheel speed (measured) and cruise setting',
	'Measured left/right wheel speeds from the encoders (mm/s). The cruise SETTING is quantised to 10 mm/s ' +
	'and is not a per-wheel target. NULL (gap) = no encoder data.',
	series(['avg(speed_l_mms) AS "left wheel"', 'avg(speed_r_mms) AS "right wheel"',
		LAST('cruise_set_mms', 'cruise setting')], 'csc14.telemetry'), 16, 7, 0, 'suffix: mm/s', {},
	[{ matcher: { id: 'byName', options: 'cruise setting' }, properties: [
		{ id: 'custom.lineStyle', value: { fill: 'dash', dash: [10, 10] } }] }]));
add(stat('Distance since boot (odometer)', 'Newest odometer (cm, saturates at 65535 = 655 m).',
	LATEST(`round(odometer_cm / 100.0, 2) AS "metres"`), 4, 7, 16, { unit: 'lengthm', decimals: 2 }));
add(stat('Commands', 'Source of the running motion command and whether one is active.',
	LATEST(`${FRESH('motion_source_label', 'source')}, CASE WHEN cmd_active THEN 'active' ELSE 'none' END AS "command"`),
	4, 7, 20, {}, { textMode: 'value_and_name', colorMode: 'none', text: { titleSize: 12, valueSize: 18 } }));
y += 7;

/* ------------------------------------------------------------------ */
add(row('Line following and barcodes'));
add(timeline('IR sensors (car\'s real sides; BLACK = over the line)',
	'Physical left/right with IR_SIDES_MIRRORED = 1 (drv_ir.h): left = GP27 (also the barcode sensor), ' +
	'centre = GP6, right = GP2. CSC14_IR_SIDES_MIRRORED must match the flashed firmware.',
	series([LAST("CASE WHEN ir_left THEN 'BLACK' ELSE 'white' END", 'left (GP27)'),
		LAST("CASE WHEN ir_centre THEN 'BLACK' ELSE 'white' END", 'centre (GP6)'),
		LAST("CASE WHEN ir_right THEN 'BLACK' ELSE 'white' END", 'right (GP2)')], 'csc14.telemetry'), 12, 6, 0,
	mapText({ BLACK: 'dark-gray', white: 'super-light-yellow' })));
add(timeline('Tracking state and line position', 'Line state (ON_LINE / SEARCHING / LOST / UNKNOWN = not ' +
	'calibrated) and line position named after the car\'s real sides.',
	series([LAST('line_state_label', 'state'), LAST('line_pos_label', 'position')], 'csc14.v_telemetry'), 12, 6, 12,
	mapText({ ON_LINE: 'green', SEARCHING: 'orange', LOST: 'red', UNKNOWN: 'purple', CENTRE: 'green',
		JUNCTION: 'blue' })));
y += 6;
add(timeseries('Steering request (+ = turn right)', 'Steering the line follower asks Motion Control for ' +
	'(-63..63, motor convention: + = clockwise). 0 while not steering.',
	series(['avg(steer) AS "steer"'], 'csc14.telemetry'),
	12, 6, 0, 'none', { min: -63, max: 63 }));
add(stat('Last barcode', 'Newest snapshot. "mapped to" is the navigation MAPPING of the symbol; telemetry ' +
	'v1 does not report whether the turn was executed. The default map is empty (nav_map.c).',
	LATEST(`${FRESH("coalesce(bc_symbol, '(none)') || ' ' || bc_result_label", 'symbol / result')},
	  coalesce(nav_label, 'no mapping') AS "mapped to", bc_state_label AS "decoder"`), 5, 6, 12,
	{}, { colorMode: 'none', textMode: 'value_and_name', text: { titleSize: 12, valueSize: 18 } }));
add(table('Barcode reads (robot events)', 'One row per BARCODE event; counters in telemetry are modulo 64.',
	`SELECT received_at AS "received", message FROM csc14.event
	WHERE ${WHERE} AND kind = 'BARCODE' ORDER BY received_at DESC LIMIT 200`, 7, 6, 17));
y += 6;

/* ------------------------------------------------------------------ */
add(row('IMU and terrain'));
add(stat('IMU', 'IMU status and motion class (LSM303DLHC: accelerometer + magnetometer, no gyroscope).',
	LATEST(`${FRESH('imu_status_label', 'status')}, motion_class_label AS "motion", ` +
		"CASE WHEN tilt THEN 'TILT' ELSE 'level' END AS \"tilt\", " +
		"CASE WHEN collision THEN 'COLLISION' ELSE 'none' END AS \"collision\""), 6, 6, 0,
	{ mappings: mapText({ OK: 'green', CALIBRATING: 'blue', INIT: 'blue', FAULT: 'red', TILT: 'orange',
		COLLISION: 'red' }) }, { text: { titleSize: 12, valueSize: 18 } }));
add(timeseries('Pitch and roll', 'Degrees; pitch nose-up +, roll right-side-down +. Gap = invalid sample.',
	series(['avg(pitch_deg) AS "pitch"', 'avg(roll_deg) AS "roll"'], 'csc14.telemetry'), 9, 6, 6, 'degree'));
add(timeseries('Turn rate (IMU, + = left / counter-clockwise)',
	'Change of the magnetometer heading over 200 ms (no gyroscope on the LSM303DLHC). Invalid while the ' +
	'magnetometer overflows. Motor magnets disturb it.',
	series(['avg(yaw_rate_ccw_dps) AS "turn rate CCW+"'], 'csc14.telemetry'), 9, 6, 15, 'suffix: °/s'));
y += 6;
add(timeseries('Heading: encoder (+ clockwise) vs IMU magnetometer (+ counter-clockwise)',
	'Two independent estimates with OPPOSITE sign conventions, both relative to their own start: ' +
	'encoder heading from wheel travel (2 deg steps), IMU heading from the magnetometer relative to ' +
	'calibration. They are not expected to agree in sign.',
	series([LAST('heading_enc_cw_deg', 'encoder (CW+)'), LAST('heading_imu_ccw_deg', 'IMU (CCW+)')], 'csc14.telemetry'), 12, 7, 0, 'degree', { min: 0, max: 360 },
	[{ matcher: { id: 'byName', options: 'encoder (CW+)' }, properties: [{ id: 'custom.drawStyle', value: 'points' },
		{ id: 'custom.pointSize', value: 3 }] },
	 { matcher: { id: 'byName', options: 'IMU (CCW+)' }, properties: [{ id: 'custom.drawStyle', value: 'points' },
		{ id: 'custom.pointSize', value: 3 }] }]));
add(timeline('Tilt, collision, motion class', 'From each snapshot.',
	series([LAST('motion_class_label', 'motion class'), LAST("CASE WHEN tilt THEN 'TILT' ELSE 'level' END", 'tilt'),
		LAST("CASE WHEN collision THEN 'COLLISION' ELSE 'none' END", 'collision')], 'csc14.v_telemetry'), 12, 7, 12,
	mapText({ STATIONARY: 'text', MOVING: 'green', TURN_LEFT: 'blue', TURN_RIGHT: 'purple', TILT: 'orange',
		level: 'green', COLLISION: 'red', none: 'green' })));
y += 7;
add(stat('Highest hump (selected run / time range)',
	'Maximum of the individual humps received in the selection (FR6). Not the robot\'s since-boot maximum, ' +
	'which can include humps from before the run.',
	`SELECT max(height_mm) AS "highest hump", count(*) AS "humps",
	  count(*) FILTER (WHERE height_mm IS NULL) AS "without height"
	FROM csc14.hump WHERE ${WHERE}`, 6, 6, 0, { unit: 'lengthmm' }, { colorMode: 'none' }));
panels[panels.length - 1].fieldConfig.overrides.push({ matcher: { id: 'byRegexp', options: '^(humps|without height)$' },
	properties: [{ id: 'unit', value: 'none' }] });
add(stat('Robot hump counters (newest snapshot)', 'Since ROBOT BOOT, 2 mm steps: last height, highest, count ' +
	'(4 bits, saturates at 15: after that the robot sends no HUMP events).',
	LATEST(`hump_last_mm AS "last (mm)", hump_max_boot_mm AS "highest since boot (mm)",
	  hump_count || CASE WHEN hump_count = 15 THEN ' (saturated)' ELSE '' END AS "count",
	  CASE WHEN hump_active THEN 'measuring' ELSE 'idle' END AS "hump"`), 6, 6, 6, {}, { colorMode: 'none',
	text: { titleSize: 12, valueSize: 22 } }));
add(table('Individual humps', 'detected_by: robot_event / telemetry (count change) / max_increase or ' +
	'last_change (after the counter saturated); "+" joins two sources of the same hump.',
	`SELECT received_at AS "received", height_mm AS "height (mm)", count_after AS "robot count",
	  max_boot_mm AS "highest since boot (mm)", detected_by FROM csc14.hump WHERE ${WHERE}
	ORDER BY received_at DESC LIMIT 200`, 12, 6, 12));
y += 6;

/* ------------------------------------------------------------------ */
add(row('Obstacles'));
add(timeseries('Ultrasonic range', '"No echo" (nothing within 4 m) is drawn as a dashed line at 4 m. ' +
	'Forward range only while the servo is centred and ranging ' +
	'(avoidance IDLE, STOPPING, EXECUTING, SEARCHING, ALIGNING, FAILED). During COARSE/FINE scans the ' +
	'value is a sample at an unknown servo angle (not transmitted); during PLANNING it is held.',
	series(['avg(us_forward_mm) AS "forward range"',
		`max(CASE WHEN us_no_echo AND us_context = 'forward' THEN 4000 END) AS "no echo: clear beyond 4 m"`,
		`avg(us_dist_mm) FILTER (WHERE us_context = 'scan_sweep') AS "scan sample (angle unknown)"`], 'csc14.telemetry'), 12, 7, 0, 'lengthmm', {},
	[{ matcher: { id: 'byName', options: 'scan sample (angle unknown)' }, properties: [
		{ id: 'custom.drawStyle', value: 'points' }, { id: 'custom.pointSize', value: 4 }] },
	 { matcher: { id: 'byName', options: 'no echo: clear beyond 4 m' }, properties: [
		{ id: 'custom.lineStyle', value: { fill: 'dash', dash: [4, 6] } }, { id: 'color', value: { mode: 'fixed', fixedColor: 'blue' } }] }]));
add(timeline('Avoidance phase and outcome', 'avoid_state and the outcome of the last avoidance.',
	series([LAST('avoid_state_label', 'phase'), LAST('avoid_outcome_label', 'last outcome')], 'csc14.v_telemetry'), 12, 7, 12,
	mapText({ IDLE: 'green', FAILED: 'red', SUCCESS: 'green', CLEARED: 'blue', NONE: 'text' })));
y += 7;
add(stat('Latest range', 'Newest snapshot with what the value means right now.',
	LATEST(`${FRESH("CASE WHEN us_no_echo THEN 'clear (> 4 m)' WHEN us_dist_mm IS NULL THEN us_result_label " +
		"ELSE us_dist_mm || ' mm' END", 'range')}, us_context AS "meaning", avoid_state_label AS "phase"`),
	6, 6, 0, {}, { colorMode: 'none', text: { titleSize: 12, valueSize: 18 } }));
add(table('Last obstacle profile', 'Newest snapshot that carries a profile. "none since boot" = the profile ' +
	'words are still zero. Clearance "at least" = beyond the 2.54 m range of the field.',
	`SELECT received_at AS "received", obst_width_mm AS "width (mm)", obst_bearing_deg AS "bearing (deg, + right)",
	  clear_left_mm || CASE WHEN clear_left_at_least THEN '+' ELSE '' END AS "clear left (mm)",
	  clear_right_mm || CASE WHEN clear_right_at_least THEN '+' ELSE '' END AS "clear right (mm)",
	  avoid_side_label AS "side chosen"
	FROM csc14.v_telemetry WHERE ${ROBOT} AND profile_reported ORDER BY received_at DESC LIMIT 1`, 9, 6, 6));
add(table('Avoidance outcomes', 'Robot OBSTACLE events (success, cleared, failure with reason) and failures the ' +
	'backend saw in telemetry (the v1 firmware sends no event for a failure).',
	`SELECT received_at AS "received", origin, severity, message FROM csc14.event
	WHERE ${WHERE} AND kind IN ('OBSTACLE', 'AVOID_FAILED') ORDER BY received_at DESC LIMIT 100`, 9, 6, 15,
	[SEV_OVERRIDE]));
y += 6;

/* ------------------------------------------------------------------ */
add(row('Communication and backend'));
const rate = (table, name) => series([`count(*) / 10.0 AS "${name}"`], table, null, 10000)
	.replace(`d."${name}"`, `coalesce(d."${name}", 0) AS "${name}"`);
const framesPanel = timeseries('Frames received per second (10 s average)', 'Telemetry should be 2/s and ' +
	'heartbeats 1/s while the robot is online; 0 = nothing received (outage or robot off).',
	rate('csc14.telemetry', 'telemetry'), 8, 6, 0, 'suffix: /s', { min: 0 });
framesPanel.targets.push(Object.assign(target(rate('csc14.heartbeat', 'heartbeat')), { refId: 'B' }));
add(framesPanel);
add(stat('Lost frames and robot counters', 'Telemetry frames missing by sequence number in the selection ' +
	'(unknown = gaps too long to count), and the robot\'s own saturating counters in the newest snapshot.',
	`SELECT (SELECT coalesce(sum(seq_gap), 0) FROM csc14.telemetry WHERE ${WHERE}) AS "missing telemetry",
	  (SELECT count(*) FROM csc14.event WHERE ${WHERE} AND kind = 'TELEMETRY_GAP' AND data->>'exact' = 'false') AS "unknown-length gaps",
	  t.tlm_dropped AS "robot queue drops (sat. 255)", t.rx_rejected AS "commands rejected (sat. 255)"
	FROM (SELECT 1) one LEFT JOIN (SELECT tlm_dropped, rx_rejected FROM csc14.telemetry WHERE ${ROBOT}
	  ORDER BY received_at DESC LIMIT 1) t ON true`, 8, 6, 8, {}, { colorMode: 'none', text: { titleSize: 12, valueSize: 32 } }));
add(timeseries('Backend: buffer and write latency', 'Rows waiting for PostgreSQL and the 95th percentile ' +
	'of frame-received -> row-committed latency, from csc14.ingest_health.',
	series(['max(queue_bulk + queue_priority) AS "buffered rows"', 'max(latency_p95_ms) AS "write p95 (ms)"'],
		'csc14.ingest_health', '$__timeFilter(at)', 10000, 'at'), 8, 6, 16, 'none', {},
	[{ matcher: { id: 'byName', options: 'write p95 (ms)' }, properties: [{ id: 'custom.axisPlacement', value: 'right' },
		{ id: 'unit', value: 'ms' }] },
	 { matcher: { id: 'byName', options: 'buffered rows' }, properties: [{ id: 'min', value: 0 },
		{ id: 'custom.axisSoftMax', value: 10 }] }]));
y += 6;
add(table('Rejected frames', 'Frames that failed validation (length, CRC, version, type, field range). ' +
	'They never reach the measurement tables.',
	`SELECT received_at AS "received", topic, reason, detail, length, encode(frame, 'hex') AS frame
	FROM csc14.rejected_frame WHERE ${WHERE} ORDER BY received_at DESC LIMIT 100`, 24, 5, 0));
y += 5;

/* ------------------------------------------------------------------ */
add(row('Events'));
add(table('Events (newest first; time = backend receive time, not the physical event time)',
	'origin robot = event frame from the robot; server = derived by the backend from telemetry or time; ' +
	'bus = command seen on the broker. Events are derived from 500 ms snapshots on the robot, so very short ' +
	'changes can be missing.',
	`SELECT received_at AS "received", origin, severity, kind, message FROM csc14.event
	WHERE ${WHERE} ORDER BY received_at DESC, id DESC LIMIT 500`, 24, 10, 0, [SEV_OVERRIDE, MESSAGE_WIDE]));
y += 10;

/* ------------------------------------------------------------------ */
add(row('Runs'));
add(table('Runs (click a name to show that run)', 'Summary from what the backend received during each run; ' +
	'"notes" says why a figure may be incomplete.',
	`SELECT name, run_id::text AS run_id, started_at AS "started", ended_at AS "ended", duration_s AS "duration (s)",
	  tlm_frames AS "frames", tlm_missing_known AS "missing", humps, hump_max_mm AS "highest hump (mm)",
	  distance_m AS "distance (m)", avoid_success AS "avoid ok", avoid_failed AS "avoid failed",
	  collisions, array_to_string(notes, '; ') AS notes,
	  (extract(epoch FROM started_at) * 1000)::bigint AS from_ms,
	  (extract(epoch FROM coalesce(ended_at, now())) * 1000 + 5000)::bigint AS to_ms
	FROM (SELECT run_id AS rid FROM csc14.run WHERE ${ROBOT} ORDER BY started_at DESC LIMIT 50) r
	CROSS JOIN LATERAL csc14.run_summary(r.rid) ORDER BY started_at DESC`,
	24, 8, 0,
	[{ matcher: { id: 'byName', options: 'name' }, properties: [{ id: 'links', value: [{
		title: 'Show this run',
		url: '/d/csc14-robot?var-robot=${robot}&var-source=${source}&var-run=${__data.fields.run_id}' +
			'&from=${__data.fields.from_ms}&to=${__data.fields.to_ms}' }] }] },
	 { matcher: { id: 'byRegexp', options: '^(run_id|from_ms|to_ms)$' }, properties: [{ id: 'custom.hidden', value: true }] },
	 { matcher: { id: 'byName', options: 'notes' }, properties: [{ id: 'custom.width', value: 520 }] }]));
y += 8;

const dashboard = {
	uid: 'csc14-robot',
	title: 'CSC14 robot telemetry',
	description: 'Live and historical telemetry of the CSC14 robot car (Pico W -> Mosquitto -> Node-RED -> PostgreSQL).',
	tags: ['csc14'],
	timezone: 'browser',
	editable: false,
	graphTooltip: 1,
	refresh: '5s',
	time: { from: 'now-15m', to: 'now' },
	timepicker: { refresh_intervals: ['5s', '10s', '30s', '1m', '5m'] },
	schemaVersion: 41,
	version: 1,
	links: [],
	annotations: { list: [{ builtIn: 1, datasource: { type: 'grafana', uid: '-- Grafana --' }, enable: true,
		hide: true, iconColor: 'rgba(0, 211, 255, 1)', name: 'Annotations & Alerts', type: 'dashboard' }] },
	templating: { list: [
		{ name: 'robot', label: 'Robot', type: 'query', datasource: DS, refresh: 1, sort: 1,
		  query: 'SELECT robot_id FROM csc14.robot_link UNION SELECT robot_id FROM csc14.run ORDER BY 1',
		  definition: 'robot ids seen by the backend', includeAll: false, multi: false, current: {}, options: [] },
		{ name: 'source', label: 'Source', type: 'custom', query: 'live,simulated', includeAll: false,
		  multi: false, current: { selected: true, text: 'live', value: 'live' },
		  options: [{ selected: true, text: 'live', value: 'live' }, { selected: false, text: 'simulated', value: 'simulated' }] },
		{ name: 'run', label: 'Run', type: 'query', datasource: DS, refresh: 2, sort: 0,
		  query: "SELECT __value, __text FROM (SELECT 'all' AS __value, 'All data in the time range' AS __text, 1 AS o, now() AS s " +
			"UNION ALL SELECT run_id::text, name || ' (' || to_char(started_at, 'YYYY-MM-DD HH24:MI') || " +
			"CASE WHEN ended_at IS NULL THEN ', active' ELSE '' END || ')', 2, started_at FROM csc14.run " +
			"WHERE robot_id = ${robot:sqlstring} AND source = ${source:sqlstring}) x ORDER BY o, s DESC",
		  definition: 'runs of the robot', includeAll: false, multi: false,
		  current: { selected: true, text: 'All data in the time range', value: 'all' }, options: [] }
	] },
	panels
};

const file = path.join(__dirname, 'dashboards', 'csc14-robot.json');
const text = JSON.stringify(dashboard, null, 2) + '\n';
if (require.main === module) {
	fs.mkdirSync(path.dirname(file), { recursive: true });
	fs.writeFileSync(file, text);
	console.log(`wrote ${file} (${panels.length} panels)`);
}
module.exports = { dashboard, text };
