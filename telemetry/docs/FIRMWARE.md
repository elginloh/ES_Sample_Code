# Firmware side of the telemetry work

Everything below is in `app_program/subsystem1/` (Buddy 1). No other
subsystem, pin, priority, queue, schedule or the wire layout (protocol v1)
was changed. **Nothing was flashed.**

## 1. Changes

| # | File | Change | Why the backend alone cannot do it |
|---|---|---|---|
| F1 | `tlm_build.c` (`tlm_events`) | also send the OBSTACLE event when the avoidance state **enters FAILED** | `task_avoid.c` `fail()` does not move the avoidance counter, so v1 never reported a failure. The backend sees FAILED in telemetry, but the **failure reason** (`AVF_*`) is only in the event, not in any telemetry field. Same event layout; one event per failure. |
| F2 | `task_telemetry.c`, `subsystem1.h` | optional `S1_STAT_REPORT_S` (default **0 = not compiled**): every N s print the existing TASK_STAT counters of motion, line, IMU, scan and telemetry tasks, the telemetry queue fill level and the comm counters on the USB console | TASK_STAT is only printed by the `TEST=1` image, which runs the self-tests instead of normal operation. Needed to measure control-loop timing with telemetry running (MEASUREMENTS.md). |
| docs | `MQTT_BINARY_PROTOCOL.md`, `MQTT_INTEGRATION_PATCH.md`, `mqtt_bridge.h`, `nodered_decoder.js`, `tlm_monitor.js` | corrected severity of LINE_RECOVERY, heading source/sign, IR sides, ultrasonic meaning, profile zeros, FAULTS first event, applied status of the transport patch; server-side codec fixes | — |

Host tests: `INT-16e` (new) checks F1: a failed avoidance with an
unchanged counter produces exactly one OBSTACLE event carrying the reason,
and nothing more while the state stays FAILED. All suites: 0 failed.

## 2. Images (built 2026-10-30 with `make NET=lwip MQTT=1 -j8`, not flashed)

| Image | File on this Mac | text | data | bss |
|---|---|---|---|---|
| baseline: commit d81f8b9, unmodified | `telemetry/.runtime/firmware/baseline_d81f8b9_smp1_usb_cdc_wifi_lwip_mqtt.uf2` | 373 782 | 120 | 85 096 |
| candidate: d81f8b9 + F1 (+ F2 off) | `telemetry/.runtime/firmware/candidate_d81f8b9+s1-obstacle-fail-event_smp1_usb_cdc_wifi_lwip_mqtt.uf2` (also `build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2`) | 373 798 | 120 | 85 096 |
| measurement: candidate with `S1_STAT_REPORT_S 10` | `telemetry/.runtime/firmware/measure_stat10_d81f8b9+s1_smp1_usb_cdc_wifi_lwip_mqtt.uf2` | 374 670 | 120 | 85 096 |

All three: 0 compiler warnings. RAM (data + bss) is identical; flash
+16 bytes (candidate) and +888 bytes (measurement). Folders are absolute
under `/Users/junjiang/Embedded_System_Workspace/ES_Sample_Code/`.

**Is a new flash required?** No for the backend: it reads current v1
firmware. Flash the candidate only to receive the failure REASON of an
obstacle avoidance. The images contain the Wi-Fi settings of
`config/wifi_secrets.h` at build time (broker `172.20.10.7`).

## 3. What the firmware can and cannot report

| Supplied by v1 (2 Hz snapshot) | NOT supplied (shown as unavailable, never invented) |
|---|---|
| mode, epoch, active fault mask | boot id (inferred from uptime by the server) |
| measured wheel speeds, odometer, encoder heading, cruise setting | event or sample time (only uptime in telemetry/heartbeat) |
| 3 IR bits, line position/state, steering request | per-wheel speed targets, PWM, battery, position on the track |
| last barcode symbol/result, its navigation MAPPING, read counters | whether a barcode turn was executed (`executed`/`mres` exist in the barcode status word, not in telemetry) |
| IMU status, pitch, roll, magnetometer turn rate and heading, motion class, tilt, collision latch, peak shock | IMU sample validity/freshness flags, IMU calibration flag |
| hump last/highest (since boot), count (≤ 15), in progress | humps after the 15th as events; hump length |
| ultrasonic range and result, avoidance phase/side/outcome | servo angle and scan mode of a range sample; profile status (PRF_*) and scan id |
| profile width, bearing, clearances | when the profile was measured |
| comm state, robot drop and reject counters | task timing (only with the F2 measurement image, on the console) |

## 4. Possible protocol v2 (NOT implemented; needs the team)

Each would be a new version number; the decoder would accept v1 and v2
side by side. Ordered by value for trustworthy measurements:

1. a random 16-bit **boot id** in the heartbeat (exact sessions instead of inferred);
2. **scan mode** (2 bits) and **servo angle** (8 bits) with the range (forward range without inference);
3. a 4-bit **hump completion counter** that wraps instead of saturating (needs a field in Buddy 4's hump word): HUMP events after the 15th hump;
4. barcode **executed** + motion result (3 bits): confirmed navigation;
5. profile **status** and **scan id** (5 bits): fresh vs. stale profile.

## 5. Shortest live check (robot on a stand, wheels off the ground)

1. Connect this Mac to the hotspot the robot uses and check that
   `ipconfig getifaddr en0` prints `172.20.10.7` (the `MQTT_BROKER_IP` in
   `config/wifi_secrets.h`). If not, the robot cannot reach this Mac.
2. `cd /Users/junjiang/Embedded_System_Workspace/ES_Sample_Code/telemetry && bin/csc14 start`
   (starts `mosquitto -c ~/mosquitto-dev.conf` if nothing listens on 1883).
3. Power the robot. Within about 30 s `bin/csc14 status` shows
   `robot csc14-car (live): ONLINE`; the dashboard
   http://127.0.0.1:3000/d/csc14-robot shows telemetry 2/s, heartbeat 1/s.
4. `bin/csc14 run start "stand check"`. Hold black tape under the car's LEFT
   IR sensor: the IR panel must show **left** BLACK. Tilt the car nose up:
   pitch positive. Turn it left by hand: IMU turn rate positive.
5. Switch the hotspot off for 20 s, then on: the link goes STALE, then
   OFFLINE, then ONLINE; the robot's reconnect count goes up by one; the
   time series show a gap.
6. `bin/csc14 run end` prints the run summary.
7. Optional: `bin/csc14 command ping` → `ACCEPTED: OK`.

Note: a dashboard at 2 Hz cannot show the 100 ms stop requirement (NFR6);
that needs the console or a scope.

## 6. On-target timing (TASK_STAT) with telemetry running

1. Flash the measurement image (path in §2).
2. `cat /dev/cu.usbmodem* | tee ../testing/subsystem1/logs/2026-MM-DD_task_stat_<condition>.txt`
   then reset the Pico. Every 10 s it prints lines such as
   `stat: motion runs 3000 exec 412/655 us (last/max) period 19980..20040 us overruns 0`.
3. Record 2 minutes for each condition, power-cycling between them (the
   counters are since boot): (a) broker and backend running, (b) broker
   stopped, (c) hotspot off.
4. Compare `exec max`, `period max - period min` and `overruns` of motion
   (20 ms, budget 10 ms), line (20 ms, budget 10 ms) and IMU (10 ms, budget
   5 ms) between the conditions. Add the results to
   `telemetry/docs/MEASUREMENTS.md`.
