# CSC14 telemetry backend: design

Pico W → Mosquitto → **Node-RED → PostgreSQL → Grafana**. This folder
(`telemetry/`) is the server side of subsystem 1. The robot side
(`app_program/subsystem1/`) and the wire protocol
(`app_program/subsystem1/docs/MQTT_BINARY_PROTOCOL.md`) are unchanged
except for the one firmware change in [FIRMWARE.md](FIRMWARE.md).

## 1. Requirements and what is a choice

| Source | Requirement (mandatory) | How it is met |
|---|---|---|
| FR6 | record hump data, highest hump of the complete run | `csc14.hump` (one row per hump), run maximum in `csc14.run_summary()`; robot's since-boot maximum kept separately |
| FR7 | transmit speed, sensor status, state, measurements | robot side, protocol v1 (unchanged) |
| FR13 | heartbeat; online / disconnected / recovering | robot heartbeat stored; backend link state ONLINE / STALE / OFFLINE / UNKNOWN from frame age; robot-reported comm state and reconnect count shown |
| FR14 | validate, store valid measurements in PostgreSQL, show current and historical telemetry | Node-RED validation (length, CRC, version, type, every field range); PostgreSQL; Grafana dashboard |
| FR15–19 | validity/calibration, tilt, motion class, collision, turn rate | stored as typed columns and server events (TILT_ON/OFF, COLLISION, IMU status), shown with the IMU's real conventions |
| NFR9–11 | robot independent of server/database; non-blocking reconnection; malformed messages handled | backend never publishes; no firmware path waits for it; malformed frames go to `csc14.rejected_frame` |
| Change Req. 1 | compact binary application fields | unchanged binary v1 frames; JSON only on the server |
| Week 6 §6.1 | Mosquitto → Node-RED → PostgreSQL → Grafana, on the Raspberry Pi | same chain; native install (§3) |

**Implementation choices** (not required by any document): native
services instead of containers, Node-RED code kept in `lib/` and loaded via
global context, one row per telemetry frame plus JSONB, run assignment by a
database trigger, 3 s / 10 s stale / offline thresholds, the bounded
in-memory buffer sizes, Grafana's 5 s refresh, the dashboard layout.

Historical statements resolved against later documents and the source:
Week 6 says telemetry is "formatted as JSON" → superseded by Change
Requirement 1 (binary). Week 6 says the IMU has a gyroscope → the supplied
LSM303DLHC has none (`imu_proc.h`, `IMU_NOTES.md`). The protocol document
said LINE_RECOVERY is a critical event and the heading is gyro-integrated →
corrected from `sys_state_logic.c` and `imu_proc.h`.

## 2. Audit (2026-10-30, commit d81f8b9)

**Kept as is:** the binary protocol v1 and its golden frames; the MQTT
transport inside `cyw43_utk_task` (`mqtt_utk.c`, applied in 05a675c, the
integration note said otherwise and is corrected); the connection state
machine and its non-blocking retries; the task architecture; existing
Mosquitto 2.1.2 (`~/mosquitto-dev.conf`: `listener 1883`,
`allow_anonymous true`) and PostgreSQL 18.4 (Homebrew cluster, which was
simply not running).

**Missing, now implemented:** database schema, Node-RED flow, Grafana
data source and dashboard, run handling, simulator, tests, deployment.

**Interpretation errors, fixed in the shared decoder:**

| Finding (verified in source) | Effect | Fix |
|---|---|---|
| decoder marked LINE_RECOVERY (bit 7) critical; firmware: degraded | false "critical" | severity table follows `sys_state_logic.c` with its policy flags |
| `IR_SIDES_MIRRORED` = 1: IR_R (GP27) is the car's LEFT | decoder's `ir.left` was the right sensor | physical `ir` and `line.pos_phys`; firmware labels kept as `ir_code` |
| protocol doc: heading "gyro-integrated" | wrong source | magnetometer, + counter-clockwise; encoder heading + clockwise |
| `us_dist` during COARSE/FINE is a sweep sample, during PLANNING a held value | false forward ranges | `us_context`; forward range only while monitoring |
| profile words start at zero | phantom 0 mm obstacle | `profile = none_yet`, values NULL |
| first FAULTS event after going online = faults since boot | stale faults shown as active | active faults only from telemetry; that event labelled |
| failed avoidance never moved the avoidance counter → no OBSTACLE event | failures and their reasons invisible | backend derives AVOID_FAILED; firmware fix in FIRMWARE.md |
| hump count saturates at 15 → no more HUMP events | humps missed after 15 | detection from max/last changes, run summary marked incomplete |

**Other findings (not part of this task, reported only):**
`config/wifi_secrets.h` is committed although its comment says it is
ignored (Wi-Fi password in the history); `TEAM.md`, the `testing/*/TEST_LOG.md`
tables and the `app_main.c` header still describe the template example;
the left encoder uses GP0/GP1, which `CRASH_REPORT=1` takes for UART0;
TASK_STAT counters are only printed by the `TEST=1` image (no on-target
timing was ever recorded).

**Still uncertain (needs the robot):** which image is flashed; real Wi-Fi
loss and latency; on-target task timing with telemetry active; whether
the magnetometer heading is usable near the motors.

## 3. Deployment: native services

| | Native (chosen) | Containers (docker compose) |
|---|---|---|
| Broker | the existing Mosquitto, untouched | must reach the host broker via `host.docker.internal` / host-gateway |
| PostgreSQL | existing 18.4 cluster, new database | a second PostgreSQL next to the installed one |
| Node-RED, Grafana | pinned 5.0.9 (npm lock) / 13.2.5 (checksum-verified tarball), inside `telemetry/` | pinned images |
| Mac overhead | none beyond the services | Docker Desktop VM (not running on this Mac) |
| Raspberry Pi | apt packages + systemd (Week 6 plan) | Docker engine + images on the SD card |
| Restart, health | systemd `Restart=on-failure` on the Pi; `bin/csc14 status`, `/csc14/health` | compose restart policies, healthchecks |

Containers would pin the whole stack in one file, but here they would
duplicate the database already installed, need a VM on the Mac, and add
an address translation between the container and the broker the Pico
uses. Docker being installed did not decide it either way.

## 4. Data flow

1. `mqtt in` (output Buffer, QoS 0 as published) on `csc14/+` and
   `sim/csc14/+`.
2. Function "validate, decode, route" → `lib/ingest.js`: frame check, field
   ranges, interpretation, boot session, sequence gaps, derived events →
   routes telemetry / heartbeat / event / ack / rejected.
3. Function "store in PostgreSQL" → `lib/store.js`: bounded queue,
   parameterized INSERTs, retry.
4. Every 1 s link state, every 10 s pipeline health, every 5 s saved state.
5. Grafana reads with a SELECT-only role.

The flow never publishes; the only publisher in `telemetry/` is
`tools/command.js` (manual, PING/REQ_STATUS unless motion commands are
explicitly enabled).

## 5. Storage

| Table | One row per | Notes |
|---|---|---|
| `telemetry` | telemetry frame (2 Hz) | typed columns in units (`*_mms`, `*_mm`, `*_deg`, `*_cw_deg`, `*_ccw_dps`), NULL for unavailable values, enum codes (`csc14.lbl()` names them), original `frame`, optional `decoded` JSONB |
| `heartbeat` | heartbeat (1 Hz) | compact, no JSONB |
| `event` | robot event, derived change, link change, command seen | origin robot / server / bus, severity, readable message, JSONB data |
| `hump` | measured hump | robot event and telemetry count change of the same hump merge (`boot_id`, count) |
| `ack`, `command` | ACK frame / command sent | trigger links an ACK to its pending command |
| `rejected_frame` | invalid frame | reason, detail, first 128 bytes; at most 30 stored per minute |
| `boot_session`, `robot_link`, `run`, `ingest_health` | session / robot / run / 10 s | |

Duplication is deliberate but bounded: the 40-byte frame is the lossless
record; typed columns serve every dashboard query; the JSONB copy
(`CSC14_STORE_DECODED_JSON=1`) is about 3/4 of each telemetry row's size
(measured in MEASUREMENTS.md) and can be switched off without losing
information, because it can be rebuilt from `frame`. Retention: nothing is
deleted automatically.

Indexes follow the dashboard queries: `(robot_id, source, received_at DESC)`
for the newest row and time ranges, `(run_id, received_at)` for a selected
run. No partitioning: a 2 Hz robot adds a few MB per hour.

## 6. Runs, identifiers, time

- A run is started and ended explicitly (`bin/csc14 run start|end`). It is
  a database row only; the robot is never contacted. One active run per
  robot and source (live / simulated). A robot reboot or reconnect does not
  start or end a run.
- Rows join a run in the database (`assign_run` trigger) by `received_at`,
  so rows written late after a database outage still land in the right run.
  Rows outside a run are kept with `run_id` NULL.
- `received_at`: when Node-RED received the frame (UTC timestamptz). The
  robot sends no clock time; this is never presented as the physical event
  time. `rx_delay_ms` / `late` estimate how long a telemetry frame waited
  (relative to the session's fastest frame).
- Boot sessions are inferred from uptime continuity (no boot id in v1):
  uptime and receive time advance together; a mismatch beyond tolerance
  starts a new session (`robot_restart` when the uptime is shorter than
  the time since the previous frame). The session state survives Node-RED
  restarts (`.runtime/nodered/ingest-state.json`).
- Database ids are identities; 8-bit sequence numbers are only used to
  count gaps (exact up to 120 periods, otherwise estimated from time).

| Counter (source) | Limit | Handling |
|---|---|---|
| uptime_ds | wraps at 6553.6 s | unwrapped per session (`uptime_s`) |
| sequence numbers | wrap at 256, per type | gaps exact only while unambiguous |
| hump_count | saturates at 15 | humps then only from max/last changes; run summary says so |
| bc_valid / bc_invalid | wrap at 64 | run counts from robot BARCODE events |
| odometer_cm | saturates at 65535 | distance marked as lower bound |
| tlm_dropped, rx_rejected | saturate at 255 | shown as robot counters, never as totals |
| reconnects (heartbeat) | saturates at 15 | ROBOT_RECONNECTED events from increments |
| epoch | 3 bits | not an identifier |

## 7. Loss and outages

- **Robot side** (unchanged): QoS 0, a 4-frame queue while offline (newest
  kept), a lwIP output ring. Frames can be lost; subscribing with a higher
  QoS cannot recover them. Changes shorter than 500 ms may never appear.
- **Broker down / backend not subscribed:** frames are lost (QoS 0, clean
  session). Node-RED marks the robot UNKNOWN (a backend problem), and the
  first frame afterwards shows the sequence gap.
- **Database down:** rows wait in memory: bulk (telemetry, heartbeat,
  health) up to `CSC14_BUFFER_BULK_ROWS` (10800 = 1 hour at 3 rows/s),
  priority (events, humps, ACKs, link, sessions) up to
  `CSC14_BUFFER_PRIORITY_ROWS`. A full buffer drops its OLDEST row, counts
  it and reports a `BACKEND_BUFFER_OVERFLOW` event when the database is back.
  On Node-RED stop the remainder is saved to
  `.runtime/nodered/unwritten-rows.jsonl` and queued again at start.
- **Health** is separate from robot faults: `/csc14/health`,
  `csc14.ingest_health`, the "Backend pipeline" panel.

The history is therefore NOT guaranteed complete. The dashboard shows gaps,
the run summary lists the reasons a figure may be incomplete.

## 8. Dashboard rules

Every time series has one row per bucket over the whole range, so an
outage is a gap at every zoom; the smallest bucket is 1 s (two frames).
"Current" panels say STALE when the newest row is older than 5 s. Labels
name the car's real sides, units and sign conventions. Nothing is shown that
the robot does not measure (no battery, position, per-wheel target or PWM).
Refresh 5 s = Grafana's `min_refresh_interval`.

## 9. Security

Mosquitto accepts anonymous clients because the Pico has no MQTT
credentials; keep it on a private network. Node-RED's editor listens on
127.0.0.1 only (set `CSC14_NODERED_ADMIN_HASH` before exposing it).
Grafana uses a SELECT-only database role; anonymous viewing is on by
default for the local dashboard (`CSC14_GRAFANA_ANONYMOUS`). Passwords are
only in `telemetry/.env` (mode 600, ignored by git).
