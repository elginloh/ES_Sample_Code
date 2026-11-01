/*
 * settings.js -- Node-RED settings for the CSC14 telemetry backend.
 * Started by bin/csc14 (or the systemd unit on the Raspberry Pi) as:
 *   node node_modules/node-red/red.js --settings nodered/settings.js --userDir .runtime/nodered
 * The flow (flows.json, next to this file) loads automatically; its
 * Function nodes call lib/nodered-api.js through global context 'csc14'.
 * Everything configurable comes from telemetry/.env (see .env.example).
 */
'use strict';

const path = require('path');
const config = require('../lib/config');

config.loadEnvFile(path.join(config.ROOT, '.env'));
const cfg = config.load();

/* Values the flow reads as ${ENV} (Node-RED substitutes them at start) */
process.env.CSC14_MQTT_HOST = cfg.mqtt.host;
process.env.CSC14_MQTT_PORT = String(cfg.mqtt.port);
process.env.CSC14_MQTT_CLIENT_ID = cfg.mqtt.clientId;
process.env.CSC14_SUB_LIVE = cfg.topics.live + '/+';
process.env.CSC14_SUB_SIM = cfg.topics.sim + '/+';

const adminAuth = (process.env.CSC14_NODERED_ADMIN_HASH) ? {
	type: 'credentials',
	users: [{ username: process.env.CSC14_NODERED_ADMIN_USER || 'admin',
		  password: process.env.CSC14_NODERED_ADMIN_HASH, permissions: '*' }]
} : undefined;

module.exports = {
	flowFile: path.join(__dirname, 'flows.json'),
	flowFilePretty: true,
	credentialSecret: process.env.CSC14_NODERED_CREDENTIAL_SECRET || false,
	uiHost: process.env.CSC14_NODERED_HOST || '127.0.0.1',
	uiPort: parseInt(process.env.CSC14_NODERED_PORT || '1880', 10),
	adminAuth,
	functionGlobalContext: {
		csc14: require('../lib/nodered-api.js')
	},
	functionExternalModules: false,
	exportGlobalContextKeys: false,
	contextStorage: { default: { module: 'memory' } },
	logging: { console: { level: 'info', metrics: false, audit: false } },
	editorTheme: {
		page: { title: 'CSC14 telemetry (Node-RED)' },
		header: { title: 'CSC14 telemetry' },
		projects: { enabled: false },
		tours: false
	},
	diagnostics: { enabled: false, ui: false },
	runtimeState: { enabled: false, ui: false },
	telemetry: { enabled: false, updateNotification: false },
	mqttReconnectTime: 5000,
	debugMaxLength: 1000
};
