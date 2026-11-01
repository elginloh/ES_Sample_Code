# Subsystem 1 — test log

**Subsystem:** WiFi / MQTT communication, commands and telemetry (Buddy 1)  ·  **Student ID:** 2501573  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | cyclic handler is running | | auto | `subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | LED task sleeps between ticks (not busy) | | auto | `subsystem1_test.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | LED on GP19 toggles every 250 ms | | manual | 1. Flash normal image. 2. Watch LED / measure with scope. | 2 Hz blink, 50 % duty |
| S1-U27 | server codec: golden frames decode and re-encode byte for byte; fault severity, IR sides, field ranges | FR14, Tribunal 1 R1 | auto | `node app_program/subsystem1/docs/nodered_decoder.js` | 14 × `PASS` |
| S1-U28 | telemetry backend unit tests: codec, ingest (sessions, wrap, gaps, saturation, link states), store (outage, overflow), generated flow and dashboard | FR6, FR13, FR14, NFR11 | auto | `telemetry/bin/csc14 test` | `pass 34`, `fail 0` |
| INT-16e | a failed avoidance (counter unchanged) sends one OBSTACLE event with its reason | FR12, FR7 | auto | `make -C app_program/subsystem2/host_test SUITES="s1 int" run` | `[TEST] INT-16e PASS` |
| INT-21 | telemetry pipeline on isolated services: real Node-RED, PostgreSQL, Grafana (routing, rejection, runs, restart, reboot, stale/offline, database outage, overflow, ACK, commands, every panel query) | FR13, FR14, NFR9–NFR11 | auto | `telemetry/bin/csc14 test integration` | every subtest ✔ |
| S1-U29 | telemetry backend measurements: latency, CPU/RAM, storage, outages | NFR3, A1 | auto | `telemetry/bin/csc14 bench` | report written; nothing lost in a 30 s database outage |
| S1-U30 | live delivery: Pico → broker → Node-RED → PostgreSQL → dashboard | FR7, FR13, FR14 | manual | `telemetry/docs/FIRMWARE.md` §5 | link ONLINE, 2 telemetry/s, real IR sides, outage shown as a gap |
| S1-U31 | control-task timing (TASK_STAT) with telemetry running, broker down, Wi-Fi down | NFR9, A1 | manual | `telemetry/docs/FIRMWARE.md` §6 (measurement image) | no extra overruns; periods unchanged |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S1-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-30 | S1-U27 | `d81f8b9` | Claude (for Buddy 1) | PASS (7 checks, before the changes) | `logs/2026-10-30_before_d81f8b9_decoder_and_host_tests.txt` | baseline |
| 2026-10-30 | S1-U27 | `d81f8b9` + uncommitted | Claude (for Buddy 1) | PASS (14 checks) | `logs/2026-10-30_after_decoder_and_host_tests.txt` | changes not committed yet |
| 2026-10-30 | INT-16e | `d81f8b9` + uncommitted | Claude (for Buddy 1) | PASS; all host suites 0 failed | `logs/2026-10-30_after_decoder_and_host_tests.txt` | |
| 2026-10-30 | S1-U28 | `d81f8b9` + uncommitted | Claude (for Buddy 1) | PASS (34/34) | `logs/2026-10-30_backend_unit_tests.txt` | |
| 2026-10-30 | INT-21 | `d81f8b9` + uncommitted | Claude (for Buddy 1) | PASS (12/12 subtests) | `logs/2026-10-30_backend_integration_tests.txt` | simulated frames, isolated broker and database |
| 2026-10-30 | S1-U29 | `d81f8b9` + uncommitted | Claude (for Buddy 1) | measured | `logs/2026-10-30_telemetry_backend_bench.md`, `logs/2026-10-30_dashboard_simulated_demo.png` | Mac, simulated frames; see telemetry/docs/MEASUREMENTS.md |
