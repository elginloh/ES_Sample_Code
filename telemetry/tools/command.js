#!/usr/bin/env node
/*
 * command.js -- send ONE supervisory command to the robot and wait for its
 * acknowledgement (bin/csc14 command ...).  Optional; telemetry does not
 * need it.
 *
 *   command ping | status                 always allowed (answered by Buddy 1)
 *   command stop | resume | clear | speed <50..400>
 *                                         only with CSC14_ALLOW_MOTION_COMMANDS=1
 *
 * The frame comes from the shared encoder (encodeCommand), is published
 * once, QoS 0, never retained, on CSC14_TOPIC_LIVE/cmd, and recorded in
 * csc14.command.  The ACK, stored by Node-RED, updates that row.  An ACK
 * "accepted" means Buddy 2 took the request, not that the robot finished
 * doing it.  The automatic tests never run this tool.
 */
'use strict';

const path = require('path');
const mqtt = require('mqtt');
const { Client } = require('pg');
const config = require('../lib/config');
const { codec } = require('../lib/protocol');

config.loadEnvFile(path.join(config.ROOT, '.env'));
const cfg = config.load();
const MOTION = new Set(['STOP', 'RESUME', 'CLEAR', 'SET_SPEED']);
const NAMES = { ping: 'PING', status: 'REQ_STATUS', stop: 'STOP', resume: 'RESUME', clear: 'CLEAR', speed: 'SET_SPEED' };

async function main() {
	const name = NAMES[(process.argv[2] || '').toLowerCase()];
	const arg = (name === 'SET_SPEED') ? Number(process.argv[3]) : 0;
	if (!name) {
		console.log('usage: command ping | status | stop | resume | clear | speed <50..400>');
		process.exitCode = 1;
		return;
	}
	if (MOTION.has(name) && !config.bool(process.env.CSC14_ALLOW_MOTION_COMMANDS, false)) {
		console.error(`${name} changes what the robot does and is disabled. Set CSC14_ALLOW_MOTION_COMMANDS=1 ` +
			'in telemetry/.env to allow it (STOP is also on the robot itself).');
		process.exitCode = 2;
		return;
	}
	const db = new Client({ host: cfg.pg.host, port: cfg.pg.port, database: cfg.pg.database,
		user: cfg.pg.user, password: cfg.pg.password, application_name: 'csc14-command' });
	await db.connect();
	try {
		/* the robot refuses an id used in its last 4 commands: count up */
		const last = (await db.query(`SELECT cmd_id, seq FROM csc14.command WHERE robot_id = $1 AND source = 'live'
			ORDER BY id DESC LIMIT 1`, [cfg.robotId])).rows[0];
		const cmdId = last ? (last.cmd_id + 1) % 256 : Math.floor(Math.random() * 256);
		const seq = last ? (last.seq + 1) % 256 : 0;
		const frame = codec.encodeCommand(name, arg, cmdId, seq);	/* validates the argument */
		const topic = cfg.topics.live + '/cmd';
		const row = (await db.query(`INSERT INTO csc14.command (robot_id, source, topic, cmd, arg, cmd_id, seq, frame,
			requested_by) VALUES ($1, 'live', $2, $3, $4, $5, $6, $7, $8) RETURNING id`,
		[cfg.robotId, topic, name, arg, cmdId, seq, frame, require('os').userInfo().username])).rows[0];

		const client = mqtt.connect({ host: cfg.mqtt.host, port: cfg.mqtt.port,
			clientId: 'csc14-cmd-' + process.pid, reconnectPeriod: 0, connectTimeout: 5000 });
		const ok = await new Promise((resolve) => {
			client.on('connect', () => client.publish(topic, frame, { qos: 0, retain: false }, (e) => resolve(!e)));
			client.on('error', () => resolve(false));
		});
		client.end(true);
		if (!ok) {
			await db.query("UPDATE csc14.command SET status = 'send_failed' WHERE id = $1", [row.id]);
			console.error(`could not publish to ${cfg.mqtt.host}:${cfg.mqtt.port}`);
			process.exitCode = 1;
			return;
		}
		console.log(`sent ${name}${arg ? ' ' + arg : ''} (id ${cmdId}) on ${topic}; waiting for the ACK ...`);
		const until = Date.now() + 5000;
		while (Date.now() < until) {
			const c = (await db.query('SELECT status, ack_result, ack_detail FROM csc14.command WHERE id = $1', [row.id])).rows[0];
			if (c.status !== 'pending') {
				console.log(`${c.status.toUpperCase()}: ${c.ack_result}${c.ack_detail && c.ack_detail !== '0' ? ' (' + c.ack_detail + ')' : ''}` +
					(c.status === 'accepted' ? ' -- accepted by the robot, which is not the same as done' : ''));
				return;
			}
			await new Promise((r) => setTimeout(r, 200));
		}
		console.log('no ACK within 5 s (robot offline, command lost, or Node-RED not storing ACKs)');
		process.exitCode = 1;
	} finally {
		await db.end();
	}
}

main().catch((e) => {
	console.error('command: ' + e.message);
	process.exitCode = 1;
});
