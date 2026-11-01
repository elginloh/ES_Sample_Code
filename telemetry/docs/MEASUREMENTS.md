# Measurements (2026-10-30)

Where: the development Mac (Apple silicon, 10 CPUs, 16 GB, macOS 27,
Node.js 26.3.1, PostgreSQL 18.4, Mosquitto 2.1.2, Node-RED 5.0.9,
Grafana 13.2.5). Backend numbers come from `bin/csc14 bench` on isolated
services with the virtual robot (`lib/sim.js`) at the real 2 Hz rate.
**Measured** = taken by a tool on this computer, **estimate** = calculated.
Nothing here was measured on the Pico; those items are listed as outstanding.

Logs (all on this Mac, under
`/Users/junjiang/Embedded_System_Workspace/ES_Sample_Code/testing/subsystem1/logs/`):

| File | Content |
|---|---|
| `2026-10-30_before_d81f8b9_decoder_and_host_tests.txt` | unmodified commit: decoder self-test, host tests |
| `2026-10-30_after_decoder_and_host_tests.txt` | after the changes: same commands |
| `2026-10-30_backend_unit_tests.txt` | 34 backend unit tests |
| `2026-10-30_backend_integration_tests.txt` | real Node-RED + PostgreSQL + Grafana on isolated services |
| `2026-10-30_telemetry_backend_bench.md` / `.json` | backend measurements below |
| `2026-10-30_firmware_sizes.txt` | image sizes and SHA-256 |
| `2026-10-30_dashboard_simulated_demo.png` | screenshot of the dashboard showing the simulated demo run |

## Before / after

| Quantity | Before (audit, d81f8b9) | After | How |
|---|---|---|---|
| Server storage of telemetry | none (decoder file only) | PostgreSQL, every valid frame | integration tests |
| Fault severity in the decoder | LINE_RECOVERY wrongly critical | follows `sys_state_logic.c` and its flags | unit tests |
| IR side labels | left/right swapped on this car | car's real sides | unit tests |
| Decoder self-test | 7 checks pass | 14 checks pass (adds encoder ↔ golden frames, severity, IR sides, ranges) | logs before/after |
| Firmware host tests (s1 + int) | 55 checks, 0 failed | 56 checks, 0 failed (INT-16e) | logs before/after |
| Obstacle failure reported with reason | never | one OBSTACLE event per failure (candidate image, not flashed) | INT-16e |
| Firmware flash (text) | 373 782 B | 373 798 B (+16 B) | `arm-none-eabi-size`, measured |
| Firmware static RAM (data + bss) | 85 216 B | 85 216 B (unchanged) | measured |
| MQTT payload rate | 89 B/s (v1) | unchanged (v1 layout kept) | measured on simulated frames: 89.1 B/s |
| Control-task timing (TASK_STAT) | never recorded | **outstanding**: needs the robot (measurement image built) | FIRMWARE.md §6 |

## Firmware (measured)

| Image | text | data | bss |
|---|---|---|---|
| baseline d81f8b9 | 373 782 | 120 | 85 096 |
| candidate (F1) | 373 798 | 120 | 85 096 |
| measurement (F1 + `S1_STAT_REPORT_S 10`) | 374 670 | 120 | 85 096 |

256 KiB RAM: 85 216 B static (32.5 %); the rest is the kernel area for
task stacks (2 KiB each) and kernel objects. lwIP's pools are about 53 KB of
the static part. Stack high-water marks are not available: the kernel does
not paint stacks and no tool reads them (outstanding).

## Radio traffic (estimate from the protocol, unchanged by this work)

| Frame | Payload | MQTT PUBLISH (QoS 0) | Rate | MQTT bytes/s |
|---|---|---|---|---|
| telemetry `csc14/tlm` | 40 B | 53 B | 2 /s | 106 |
| heartbeat `csc14/hb` | 9 B | 21 B | 1 /s | 21 |
| event `csc14/evt` | 10 B | 23 B | on change | ~0 |

About 127 B/s of MQTT, about 250 B/s at the IP layer if every PUBLISH
travels in its own TCP segment (40 B of TCP/IP header each). The JSON design
of Week 6 would have been about 1.4 kB/s for telemetry alone.

## Backend (measured, `bench`, isolated services)

Latency (`normal` = virtual robot at 2 Hz for 120 s, 236 telemetry rows;
`stress` = 20 telemetry frames/s for 30 s, 600 rows):

| Path | normal p50 / p95 / max | stress p50 / p95 / max |
|---|---|---|
| publish → received by Node-RED | 1.0 / 4.0 / 41 ms | 2.0 / 4.0 / 9 ms |
| received → row committed in PostgreSQL | 2.4 / 7.0 / 905 ms (one outlier) | 3.7 / 6.4 / 20.8 ms |

Dashboard freshness: a frame is in the database within a few ms; Grafana
refreshes every 5 s, so a value is at most about 5 s old when shown, and
"current" panels say STALE beyond 5 s.

CPU (average of one core) and memory (RSS; PostgreSQL = all its processes):

| Service | idle | normal 2 Hz | dashboard open (all panels every 5 s) | stress 20 Hz |
|---|---|---|---|---|
| Node-RED | 0.2 %, 19–119 MB | 0.7 %, 38–72 MB | 0.4 %, 38–46 MB | 1.4 %, 71–80 MB |
| PostgreSQL | 0.1 %, 19–30 MB | 0.3 %, 30–32 MB | 0.9 %, 45–55 MB | 0.5 %, 39–47 MB |
| Grafana | 0.7 %, 26–293 MB | 0.9 %, 55–173 MB | 5.0 %, ≈ 324 MB | 0.6 %, 47–324 MB |
| Mosquitto | 0.0 %, 1–2 MB | 0.0 %, 1.5 MB | 0.0 %, 1.5 MB | 0.0 %, 1.5 MB |

Grafana is the heaviest service, mostly while a dashboard is open; the
ingestion path is far below one percent of a core at the real rate.

Dashboard queries (420 = every panel every 5 s for 60 s): p50 7.0 ms,
p95 14.5 ms, max 80 ms, 0 failed. Over a 6-hour range: p50 6.0 ms, max
11.5 ms. Every time series returns one row per time bucket (bounded by the
panel width), so the cost does not grow with the length of the history shown.

Storage per hour of robot operation at 2 Hz: **16.9 MiB** (telemetry,
heartbeat and event tables with indexes; measured over 120 s). Average
telemetry row 1 478 B, of which the original frame is 41 B and the
optional JSONB copy 1 167 B (79 %). Estimate without the JSONB copy
(`CSC14_STORE_DECODED_JSON=0`, or `bin/csc14 prune --decoded-older-than D`
for old rows): about 9 MiB per hour. A 10-minute run: about 2.8 MiB
(about 1.5 MiB without JSONB).

Outages (measured):

| Event | Result |
|---|---|
| database unreachable 30 s at 2 Hz | 93 bulk + 13 priority rows buffered, **0 lost**, all written 2.6 s after the database returned (retry back-off is capped at 5 s) |
| broker down 15 s | Node-RED noticed within 8 ms (robot link UNKNOWN, not OFFLINE), reconnected 4.9 s after the broker returned; frames published while it was down are **lost** (QoS 0, no queue, as on the Pico) |
| buffer overflow (integration test, buffer 60 rows, 100 frames) | 40 oldest rows dropped, counted and reported as one BACKEND_BUFFER_OVERFLOW event |
| Node-RED restart mid-run (integration test) | same boot session and run afterwards; the 2 frames sent while it was down are reported as a sequence gap |

Memory of a full default buffer (10 800 bulk rows ≈ 1 hour) is an
**estimate** of 25–35 MB (rows hold their JSONB as a string); not measured.

## Outstanding (needs the robot or a Raspberry Pi)

1. Motion, line and IMU execution times, period variation and overruns
   with telemetry running, broker down and Wi-Fi down (procedure: FIRMWARE.md §6).
2. Real publication frequency, gaps, drops and reconnect times over the
   Pico's Wi-Fi; end-to-end latency from the robot to the database.
3. Stack and queue high-water marks on the Pico (queue fill is printed by
   the measurement image; stacks are not measurable without new code).
4. The backend on a Raspberry Pi: install, CPU/RAM and query times
   (`bin/csc14 bench` on the Pi).
5. Physical accuracy (hump heights, turn rate, collision) and the 100 ms
   stop requirement: not measurable from a 2 Hz dashboard.
