# CSC14 telemetry backend

The robot (Pico W) sends small binary messages to an MQTT broker
(Mosquitto). **Node-RED** checks and decodes them and writes them into a
**PostgreSQL** database. **Grafana** shows the database as a live and
historical dashboard.

```
Pico W ──Wi-Fi──> Mosquitto ──> Node-RED ──> PostgreSQL ──> Grafana (browser)
          csc14/tlm, hb, evt, ack   validate,     history      dashboard
                                    decode, store
```

Everything for the server is in this folder:
`/Users/junjiang/Embedded_System_Workspace/ES_Sample_Code/telemetry` (on the
Mac). The commands below are run from there.

## First time on this Mac (done once)

```sh
cd /Users/junjiang/Embedded_System_Workspace/ES_Sample_Code/telemetry
bin/csc14 setup
```

This installs the pinned Node-RED 5.0.9 and Grafana 13.2.5 inside this
folder (nothing global), creates `telemetry/.env` with random passwords and
creates the database `csc14_telemetry` in your existing PostgreSQL.

## Every session

| Step | Command |
|---|---|
| Start everything | `bin/csc14 start` |
| Is it working? | `bin/csc14 status` |
| Dashboard | http://127.0.0.1:3000/d/csc14-robot |
| Record an experiment | `bin/csc14 run start "track run 1" --notes "new tape"` |
| Stop recording, see the summary | `bin/csc14 run end` |
| Past runs | `bin/csc14 run list`, `bin/csc14 run show` |
| Stop Node-RED and Grafana | `bin/csc14 stop` (`stop --all` also stops the broker/PostgreSQL if csc14 started them) |

`start` reuses the broker already running on port 1883. If nothing is
running there, it starts `mosquitto -c ~/mosquitto-dev.conf` (your usual
broker setup) and stops it again with `stop --all`.

Starting or ending a run only writes to the database. It never sends
anything to the robot. Data that arrives while no run is active is
kept too (shown with "All data in the time range").

## Without the robot

```sh
bin/csc14 simulate --seconds 130                     # virtual robot, 2 frames/s
bin/csc14 simulate --outage-at 30 --outage-for 10    # with a Wi-Fi outage
```

Simulated frames use the topics `sim/csc14/...` and are stored as
`source = simulated`. In the dashboard choose **Source: simulated**. They
can never become part of a live run.

## With the robot

The robot connects to `MQTT_BROKER_IP` in `config/wifi_secrets.h`
(currently `172.20.10.7`, this Mac on the phone hotspot). Join that
hotspot, run `bin/csc14 start`, power the robot. The step-by-step check
is in [docs/FIRMWARE.md](docs/FIRMWARE.md) §5.

## Settings you may need to change (`telemetry/.env`)

| Setting | When |
|---|---|
| `CSC14_IR_SIDES_MIRRORED`, `CSC14_SYS_*` | if the flashed firmware was built with different `IR_SIDES_MIRRORED` / `SYS_COLLISION_STOPS` / `SYS_IMU_REQUIRED` / `SYS_US_REQUIRED` |
| `CSC14_MQTT_HOST`, `CSC14_MQTT_PORT` | broker on another computer |
| `CSC14_GRAFANA_HOST=0.0.0.0` | to open the dashboard from another laptop |
| `CSC14_ALLOW_MOTION_COMMANDS=1` | only if you want `bin/csc14 command stop/resume/clear/speed` |

## Tests

```sh
bin/csc14 test            # protocol self-test + unit tests (no services needed)
bin/csc14 test all        # + integration: private broker, throw-away database, real Node-RED and Grafana
bin/csc14 bench           # measurements, written to ../testing/subsystem1/logs/
```

The integration tests and the bench use their own broker, database and
ports. They never touch the live robot, its topics or `csc14_telemetry`.

## Troubleshooting

| Symptom | Check |
|---|---|
| Robot link **OFFLINE / NO DATA** | `bin/csc14 status`: broker up? On the hotspot? `mosquitto_sub -h 127.0.0.1 -t 'csc14/#' -v` shows frames? |
| Robot link **UNKNOWN** | Node-RED lost the broker: a backend problem, not the robot (`bin/csc14 logs nodered`) |
| **Backend pipeline: NOT REPORTING** | Node-RED stopped: `bin/csc14 start` |
| **BUFFERING** | PostgreSQL unreachable; rows wait in memory (1 hour) and are written when it is back |
| Every panel shows an error | Grafana cannot reach PostgreSQL: `bin/csc14 status` |
| Many **Rejected frames** | wrong firmware/protocol version or a corrupted connection; the reason column says which check failed |
| Left/right IR swapped | `CSC14_IR_SIDES_MIRRORED` must match the firmware's `drv_ir.h` |

Logs are in `telemetry/.runtime/logs/`.

## Where things are

| Path (under `telemetry/`) | What |
|---|---|
| `bin/csc14` | all commands above |
| `nodered/flows.json`, `nodered/build-flows.js` | the Node-RED flow (generated from build-flows.js) |
| `lib/` | decoding, validation, storage (loaded by Node-RED) |
| `db/migrations/` | database tables (applied in order, never edited once applied) |
| `grafana/` | data source, dashboard (generated from build-dashboard.js) |
| `deploy/pi/` | Raspberry Pi installation |
| `docs/DESIGN.md` | why it is built this way, limits of the data |
| `docs/FIRMWARE.md` | robot-side changes, live check |
| `docs/MEASUREMENTS.md` | measured timing, memory, traffic, storage |
| `.env` | your settings and passwords (not in git) |
| `.runtime/` | Grafana program, logs, Node-RED state, firmware images (not in git) |

The protocol codec is shared with the firmware documentation:
`../app_program/subsystem1/docs/nodered_decoder.js`.

## Undo

- Stop: `bin/csc14 stop --all`.
- Remove the database (deletes all recordings): `dropdb csc14_telemetry`
  and `dropuser csc14_writer; dropuser csc14_reader`.
- Remove everything else: delete `telemetry/.runtime`, `telemetry/node_modules`
  and `telemetry/.env`. Your Mosquitto, PostgreSQL and their settings were
  not modified, apart from the database and roles above.
