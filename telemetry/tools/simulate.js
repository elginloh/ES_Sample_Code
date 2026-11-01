#!/usr/bin/env node
/*
 * simulate.js -- publish a virtual robot (lib/sim.js) in real time.
 *
 *   bin/csc14 simulate [--seconds 130] [--outage-at 85 --outage-for 8]
 *                      [--uptime-start 6500] [--fail-events] [--host h --port p]
 *
 * Frames go to CSC14_TOPIC_SIM/<tlm|hb|evt> (default sim/csc14/...), are
 * stored with source = 'simulated' and can never join a live run.  The
 * simulator refuses to publish on the live topic prefix, and never
 * publishes commands.
 */
'use strict';

const path = require('path');
const mqtt = require('mqtt');
const config = require('../lib/config');
const { VirtualRobot } = require('../lib/sim');

config.loadEnvFile(path.join(config.ROOT, '.env'));
const cfg = config.load();

function arg(name, dflt) {
	const i = process.argv.indexOf('--' + name);
	if (i < 0) { return dflt; }
	const v = process.argv[i + 1];
	return (v === undefined || v.startsWith('--')) ? true : v;
}

const seconds = Number(arg('seconds', 130));
const prefix = String(arg('prefix', cfg.topics.sim)).replace(/\/+$/, '');
const host = arg('host', cfg.mqtt.host);
const port = Number(arg('port', cfg.mqtt.port));
const outageAt = Number(arg('outage-at', -1));
const outageFor = Number(arg('outage-for', 8));
const uptimeStart = Number(arg('uptime-start', 0));

if (prefix === cfg.topics.live || prefix.startsWith(cfg.topics.live + '/')) {
	console.error(`refusing to publish simulated frames on the live prefix '${cfg.topics.live}'`);
	process.exit(2);
}

const client = mqtt.connect({ host, port, clientId: 'csc14-sim-' + process.pid, clean: true, reconnectPeriod: 1000 });
const start = Date.now();
const robot = new VirtualRobot({ bootMs: start - uptimeStart * 1000, failEvents: !!arg('fail-events', false),
	seed: Number(arg('seed', 7)) });
let cycles = 0;
let published = 0;
let bytes = 0;
let timer = null;

client.on('connect', () => {
	if (timer) { return; }
	console.log(`simulating on ${host}:${port}, topics ${prefix}/..., ${seconds} s` +
		(outageAt >= 0 ? `, Wi-Fi outage at ${outageAt} s for ${outageFor} s` : ''));
	timer = setInterval(tick, 500);
	tick();
});
client.on('error', (e) => console.error('mqtt: ' + e.message));

function tick() {
	const now = Date.now();
	const t = (now - start) / 1000;
	if (t >= seconds) {
		clearInterval(timer);
		client.end(false, {}, () => {
			console.log(`done: ${cycles} cycles, ${published} frames, ${bytes} payload bytes`);
		});
		return;
	}
	if (outageAt >= 0) {
		robot.setOnline(!(t >= outageAt && t < outageAt + outageFor));
	}
	const frames = robot.cycle(now, { heartbeat: cycles % 2 === 0 });
	for (const f of frames) {
		client.publish(prefix + '/' + f.channel, f.frame, { qos: 0, retain: false });
		published++;
		bytes += f.frame.length;
	}
	cycles++;
	if (cycles % 20 === 0) {
		process.stdout.write(`  t=${t.toFixed(0)} s, ${published} frames\n`);
	}
}
