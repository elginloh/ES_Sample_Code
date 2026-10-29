/*
 * tlm_monitor.js
 * Live console view of the robot's MQTT frames, decoded with
 * nodered_decoder.js.  Needs mosquitto_sub on the PATH.
 *
 *   node tlm_monitor.js [broker-ip]        (default 127.0.0.1)
 *
 * One line per telemetry frame (mode, faults, speeds, line, IMU, range),
 * events and acks in full, and a banner on every mode change.  Every
 * decoded frame is also appended to tlm_log.jsonl in the current directory
 * as test evidence.
 */
'use strict';

const { spawn } = require('child_process');
const fs = require('fs');
const csc14 = require('./nodered_decoder.js');

const host = process.argv[2] || '127.0.0.1';
const log = fs.createWriteStream('tlm_log.jsonl', { flags: 'a' });
const sub = spawn('mosquitto_sub', ['-h', host, '-t', 'csc14/#', '-F', '%t %x']);
let lastMode = null;
let pending = '';

function stamp() {
	return new Date().toTimeString().slice(0, 8);
}

function show(topic, f) {
	if (f.error) {
		console.log(`${stamp()} ${topic} INVALID: ${f.error}`);
		return;
	}
	log.write(JSON.stringify(Object.assign({ t: Date.now(), topic }, f)) + '\n');

	if (f.mode && f.mode !== lastMode) {
		console.log(`\n${stamp()} ===== MODE ${lastMode || '?'} -> ${f.mode} =====`);
		lastMode = f.mode;
	}
	switch (f.kind) {
	case 'telemetry':
		console.log(`${stamp()} tlm  up=${f.uptime_s}s mode=${f.mode}`
			+ ` faults=[${f.faults.join(',')}]${f.critical ? ' CRITICAL' : ''}`
			+ ` spd=${f.speed_l_mms}/${f.speed_r_mms} cruise=${f.cruise_mms}`
			+ ` ir=${f.ir.left}${f.ir.centre}${f.ir.right} line=${f.line_state}`
			+ ` imu=${f.imu.status} pitch=${f.imu.pitch_deg}`
			+ ` us=${f.obstacle.distance_mm} avoid=${f.obstacle.avoid_state}`
			+ ` comm=${f.comm_state}`);
		break;
	case 'heartbeat':
		break;				/* telemetry already shows the mode */
	default: {
		const { type, seq, kind, ...rest } = f;
		console.log(`${stamp()} ${kind.toUpperCase()} ${JSON.stringify(rest)}`);
	}
	}
}

sub.stdout.on('data', (chunk) => {
	pending += chunk.toString();
	const lines = pending.split('\n');
	pending = lines.pop();
	for (const line of lines) {
		const [topic, hex] = line.trim().split(' ');
		if (topic && hex) {
			show(topic, csc14.decode(Buffer.from(hex, 'hex')));
		}
	}
});
sub.stderr.on('data', (d) => process.stderr.write(d));
sub.on('exit', (code) => process.exit(code || 0));
console.log(`listening on ${host} csc14/# (Ctrl-C to stop, log: tlm_log.jsonl)`);
