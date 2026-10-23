# CSC14 firmware: verification report

Scope: the five subsystem folders `app_program/subsystem1..5/` (the only
writable locations, see `IMPLEMENTATION_PLAN.md` §1). Branch `elginloh`,
base commit `9460135`. Nothing has been committed. Companion documents
in this folder:

* `IMPLEMENTATION_PLAN.md`: plan + traceability matrix written before
  coding;
* `INTERFACES.md`: integration contract;
* `DATA_REPRESENTATION.md`: Week 13 evidence;
* `TEST_CASES.md`: rows for `testing/*/TEST_LOG.md`;
* `TEAM_MD_PROPOSAL.md`: proposed `TEAM.md` update.

## 0. Verification levels used in this report

| Level | Meaning | Reached by |
|---|---|---|
| **Built** | compiles and links in the professor's build | 10 firmware images, 0 errors, 0 warnings (§6.1); every subsystem file also clean with `-Wall -Wextra` |
| **Software-tested** | automated checks of the real code on a PC | 178 checks, 0 failures (§6.2) |
| **Simulated** | closed-loop runs against plant/geometry models | integration tests INT-10..20, barcode drive model, hump model (§6.2) |
| **Hardware-verified** | observed on the Pico / car | **none**: no hardware was available to the implementer |
| **Not yet tested** | everything listed in §7 | — |

The `TEST=1` image contains every automated check plus 24 runtime checks
(S1-U21..U26, S2-U40..U47, S3-U32..U35, S4-U30..U32, S5-U21..U23). It builds
in all profiles but **has not been run on a Pico**.

---

## 1. Implementation summary

**Platform layer (owned by Buddy 2, used by all):**

* `app_sync.h`: atomics, single-writer words and sequence counts, valid on
  both `SMP=1` and `SMP=0`;
* `drv_hwtime.h`: µs time base;
* `drv_gpio_irq.c`: the one shared GPIO interrupt dispatcher;
* `drv_pwm.h`: PWM slice helpers;
* `task_stat.h`: period / worst-case run-time instrumentation;
* `app_test.c`: on-target test runner, needed because the initial task
  has only 1 KB of stack.

**Buddy 1 — WiFi/MQTT, commands, telemetry**

* **Binary MQTT protocol v1** (`bitpack.c`, `mqtt_proto.c`):
  * five frame types with a version/type byte, sequence number, length,
    and CRC-16/CCITT-FALSE (little-endian);
  * LSB-first bit packing with masks and shifts;
  * a 279-bit / 40-byte telemetry frame (697 bytes as JSON);
  * full validation and ACK codes.
* Connection state machine (`comm_fsm.c`) with capped, non-blocking
  back-off (1→16 s) and periodic recovery.
* Two tasks kept separate (Tribunal 1):
  * **Telemetry Collection** (500 ms) builds the record from every
    subsystem's interface (`tlm_build.c`) and derives event frames;
  * **WiFi & MQTT Communication** (100 ms) runs the state machine, sends the
    heartbeat, and validates commands (rate limit, duplicates).
* Bounded drop-oldest queues.
* Supervisory commands go to Buddy 2. Network faults are degraded only.
* Non-blocking hand-over functions for the kernel's MQTT client
  (`mqtt_bridge.c`). Wiring them into the protected `mqtt_utk.c` is
  reported, not applied (§8).
* Server decoder: `docs/nodered_decoder.js`.

**Buddy 2 — Motion control, vehicle mode, faults**

* Sole owner of motor PWM (GP8–11).
* Encoder callbacks count and timestamp only, read under a sequence count.
* PID with feed-forward and anti-windup, at one fixed 20 ms period.
* Encoder-based MOVE/TURN with heading hold, plus creep-speed chaining
  (NFR12).
* Arbitration by source and vehicle mode; stale-command rejection by mode
  epoch; a 4-deep queue; the `stop()` bypass.
* Results DONE / ABORTED / FAULT / REJECTED per source.
* Numeric stall and encoder-fault detection; 300 ms clear debounce; bounded
  escalation after 3 faults.
* Motor watchdog in the cyclic handler.
* Owner of the vehicle mode machine (Tribunal 1), the 27-bit shared fault
  mask with its severity table, and the event word.
* Validated supervisory RESUME/CLEAR that can never override safety.

**Buddy 3 — IR line following and barcodes**

* IR polarity calibration once per INIT epoch.
* Line position from L/C; PD steering, published only in NORMAL.
* Bounded line-loss search; junction detection.
* Line status with a timestamp, plus FOUND/LOST/JUNCTION events for Buddy 5.
* **Code 39 derived from the supplied samples** (`docs/BARCODE_ANALYSIS.md`).
* GP27 edge capture into an SPSC ring, with a 1 ms TIMER-alarm polling
  backup (Tribunal 1).
* Speed-compensated, ratio-based decoding, forwards and reversed.
* Duplicate suppression and invalid-pattern rejection.
* Navigation commands as 2-bit codes; the symbol→command map is empty by
  default (team decision).

**Buddy 4 — IMU and terrain**

* Own interrupt-driven I2C0 driver.
* **WHO_AM_I probe** of the InvenSense and ST 6-DoF families (the chip is
  not assumed).
* Stationary calibration with acceptance checks.
* Integer complementary filter for pitch/roll, gyro heading and turn rate.
* Tilt, motion class and collision with minimum durations.
* Stuck/saturation detection leading to FAULT, with bounded recovery.
* Timestamped snapshot with a validity flag.
* Hump height = Σ ds·sin(pitch) from encoder distance, with the highest hump
  kept at full resolution and published as 6-bit 2 mm codes
  (`docs/IMU_NOTES.md`).

**Buddy 5 — scanning, profiling, avoidance, recovery**

* HC-SR04 echo timing via the shared interrupt; results OK / NO_ECHO /
  NO_RESPONSE / INVALID.
* SG90 sweeps: coarse ±60° at 15°, then fine 3° steps around the edges.
* Three tasks (Scan / Profile / Avoidance) exchanging packed messages with
  scan ids.
* Profile from forward-distance geometry: width, bearing, edges, and known /
  unknown clearance.
* Plan: side with more known clearance, 45° bypass, or no plan at all.
* Every step is executed through Buddy 2 and checked against its result.
* Line search uses Buddy 3's events.
* Bounded recovery (25 s, 1 re-plan); failures lead to Safe Stop
  (`docs/OBSTACLE_NOTES.md`).

## 2. File change summary (all inside `app_program/subsystem1..5/`)

124 entries: **105 created, 15 modified, 4 deleted**. About 15 800 lines of C
and headers, plus documentation.

| Subsystem | Modified (template files) | Deleted (template example) | Created |
|---|---|---|---|
| 1 | `subsystem1.c`, `subsystem1.h`, `subsystem1_test.c` | `task_led.c` | `bitpack.c/.h`, `mqtt_proto.c/.h`, `comm_fsm.c/.h`, `mqtt_bridge.c/.h`, `tlm_build.c/.h`, `task_comm.c`, `task_telemetry.c`, `s1_unit_tests.c`, `s1_integration_tests.c`, `s1_tests.h`; `docs/MQTT_BINARY_PROTOCOL.md`, `docs/MQTT_INTEGRATION_PATCH.md`, `docs/nodered_decoder.js` |
| 2 | `subsystem2.c`, `subsystem2.h`, `subsystem2_test.c` | `task_producer.c` | `app_sync.h`, `app_test.c/.h`, `drv_hwtime.h`, `drv_gpio_irq.c/.h`, `drv_pwm.h`, `drv_motor.c/.h`, `drv_encoder.c/.h`, `task_stat.h`, `sys_defs.h`, `sys_state.c`, `sys_state_logic.c`, `s2_private.h`, `motion_msg.h`, `motion_cmd.c`, `motion_cfg.h`, `pid.c/.h`, `motion_ctrl.c/.h`, `vehicle_fsm.c/.h`, `motion_api.c`, `task_motion.c`, `s2_unit_tests.c`, `s2_integration_tests.c`, `s2_tests.h`; `host_test/` (Makefile, `host_main.c`, `.gitignore`, `shim/tk/typedef.h`, `shim/tk/errno.h`, `shim/tm/tmonitor.h`); `docs/` (this report, `IMPLEMENTATION_PLAN.md`, `INTERFACES.md`, `DATA_REPRESENTATION.md`, `TEST_CASES.md`, `TEAM_MD_PROPOSAL.md`) |
| 3 | `subsystem3.c`, `subsystem3.h`, `subsystem3_test.c` | `task_consumer.c` | `code39.c/.h`, `barcode_dec.c/.h`, `nav_map.c/.h`, `line_follow.c/.h`, `s3_status.c/.h`, `drv_ir.c/.h`, `task_line.c`, `task_barcode.c`, `s3_unit_tests.c`, `s3_tests.h`; `docs/BARCODE_ANALYSIS.md` |
| 4 | `subsystem4.c`, `subsystem4.h`, `subsystem4_test.c` | `task_monitor.c` | `drv_i2c0.c/.h`, `drv_imu.c/.h`, `imu_proc.c/.h`, `hump.c/.h`, `s4_status.c/.h`, `task_imu.c`, `task_hump.c`, `s4_unit_tests.c`, `s4_tests.h`; `docs/IMU_NOTES.md` |
| 5 | `subsystem5.c`, `subsystem5.h`, `subsystem5_test.c` | — | `drv_ultrasonic.c/.h`, `drv_servo.c/.h`, `s5_msg.c/.h`, `obstacle_profile.c/.h`, `avoid_plan.c/.h`, `task_scan.c`, `task_profile.c`, `task_avoid.c`, `s5_unit_tests.c`, `s5_tests.h`; `docs/OBSTACLE_NOTES.md` |

* The deleted files were the template's LED/producer/consumer/monitor
  examples, which `app_main.c` describes as examples to replace. The kept
  file names and entry points (`subsystemN_init/_start/_test`) are
  unchanged, so `app_main.c` needs no edit.
* **Build integration:** new `.c` files sit directly in the subsystem
  folders, which `subdir.mk` already compiles (`app_program/*/*.c`).
  `docs/` and `host_test/` are deliberately one level deeper and so are not
  linked into the firmware.

## 3. Requirements compliance matrix

Legend: **Impl.** = implemented (and software-tested where possible);
**Partial**; **Blocked** (needs a protected-file change); **Not impl.**;
**TD** = team decision flagged; **HW** = hardware validation still needed.

### 3.1 Change Requirement 1 (mandatory)

| ID | Requirement | Status | Where | Evidence |
|---|---|---|---|---|
| CR1-1 | Each field uses only the bits its range needs (masks/shifts) | Impl. | every `*_pack/_unpack`; field table `DATA_REPRESENTATION.md` §A | round-trip tests §D.1 |
| CR1-2 | Faults as bits of one shared bitmask | Impl. | `sys_defs.h`, `sys_state.c` (atomic) | S2-U38/U39, INT-13 |
| CR1-3 | Applies to motion, line/barcode, obstacle, hump, states, ISR↔task, queues, MQTT | Impl. | §A.1–A.5 | — |
| CR1-4 | MQTT as compact binary fields | Impl. (transport Blocked, see FR7) | `mqtt_proto.c` | S1-U03..U13, golden frames |
| CR1-5 | Safe access to shared packed data | Impl. | `app_sync.h`; `INTERFACES.md` §9 | design review; HW stress pending |
| CR1-6 | Week 13: data representation table | Impl. | `DATA_REPRESENTATION.md` §A | — |
| CR1-7 | Week 13: RAM before/after | Impl. (measured; no historical "pre-change" build exists) | §B: linker before/after of the `OP_SAMPLE` change, per-subsystem RAM, packed vs unpacked | [ld], [sizeof] |
| CR1-8 | Week 13: message size before/after | Impl. | §C | S1-U13, [sizeof] |
| CR1-9 | Week 13: correctness and performance | Partial | §D: correctness done; timing instrumentation built in | timing needs a Pico run (§7) |

### 3.2 Tribunal 1 confirmed decisions

| ID | Decision | Status | Implementation |
|---|---|---|---|
| T1-01 | Keep Scan / Profile / Avoidance tasks separate | Impl. | `task_scan.c`, `task_profile.c`, `task_avoid.c` |
| T1-02 | Hump 0–100 mm in 2 mm steps (6 bits); 4 mm only after the known-hump test | Impl. + HW | `HUMP_STEP_MM 2`; S4-U24/U25 |
| T1-03 | GP27 edge interrupt → buffer → async decode; 1 ms timer poll as backup | Impl. + HW | `drv_ir.c` (both modes built) |
| T1-04 | MQTT binary, one encode/decode standard for Buddies 1–5 | Impl. | `mqtt_proto.c`, `tlm_build.c`, protocol doc, Node-RED decoder |
| T1-05 | Highest hump at full resolution, compacted only for storage/sending | Impl. | `hump.c` (µm) |
| T1-06 | Minimum-bit inter-task messages; safe queue transfer; verify unpack = pack | Impl. | `s5_msg.c`, `s4_status.c`, `motion_msg.h`; round-trip tests |
| T1-07 | IMU: calibrate while still, filter, thresholds from real tests | Impl. + HW | `task_imu.c` (calibrates only in INIT/SAFE_STOP), `imu_proc.c`; thresholds `[CAL]` |
| T1-08 | Shared field specification + known-value tests before integration | Impl. | `DATA_REPRESENTATION.md`, protocol doc, golden frames, INT-15 |
| T1-09 | Mode owned by B2; B3/B4/B5 send events; B1 reads the mode; B3 stops steering outside NORMAL | Impl. | `vehicle_fsm.c`, `sys_post_event()`, `task_line.c`, `tlm_build.c` |
| T1-10 | B2 sole motor owner; packed motion data; shared fault mask; safe access; verify decode + timing | Impl. (timing HW) | `drv_motor.c` only writer; S2-U01..U05 |
| T1-11 | B3 standard format; 2-bit navigation codes; bits for IR combinations | Impl. | `s3_status.h`, `nav_map.h` |
| T1-12 | B5 minimum bits, safe queue transfer, verify | Impl. | `s5_msg.h`; S5-U01..U05 |
| T1-13 | B4 full-resolution computation, compact storage | Impl. | `imu_proc.c` vs `s4_status.c` |
| T1-14 | Steering only in NORMAL; priority order; `stop()` bypass/flush ≤ 100 ms; reject invalid; Fault zero PWM | Impl. (100 ms at the pins: HW) | `motion_ctrl.c`, `task_motion.c`; S2-U12..U18, INT-12, INT-18, S2-U44 |
| T1-15 | Completion result done / aborted / fault | Impl. (+ REJECTED) | result word; S2-U17, U19 |
| T1-16 | One fixed control period; event-driven stop/fault | Impl. | 20 ms; `MF_STOP`, `MF_FAULT` |
| T1-17 | Fault retry limit escalating to Safe Stop | Impl. | `MAX_FAULT_RETRIES 3`; S2-U21, INT-13e |
| T1-18 | Creep = max(15 % cruise, measured minimum speed) | Impl. + HW | `mc_creep_speed()`; `MIN_CTRL_SPEED_MMS` `[CAL]` |
| T1-19 | Numeric stall criterion | Impl. + HW | ≥ 300 ‰ duty, no edge for 400 ms `[CAL]`; S2-U19 |
| T1-20 | Encoder ISR counts only; atomic reads; one shared GPIO handler | Impl. | `drv_encoder.c`, `drv_gpio_irq.c` |
| T1-21 | Barcode ISR records only; reject incomplete; overflow flag | Impl. | `drv_ir.c`, `barcode_dec.c` |
| T1-22 | Barcode task decodes; invalid barcode never makes a command | Impl. | `task_barcode.c`; S3-U05..U09, INT-19b |
| T1-23 | Line status (on line / lost / reacquired) with timestamp; no PWM | Impl. | `s3_get_line_status()`, `flg_line_evt` |
| T1-24 | IMU task the only I2C0 user; cleaned sample with timestamp and validity flag | Impl. | `task_imu.c`, `s4_get_imu_status()` |
| T1-25 | Hump uses validated pitch + encoder distance; no hardware access | Impl. | `task_hump.c` |
| T1-26 | Only Scan drives the sensor/servo; Profile rejects bad scans; Avoidance uses B3 line status | Impl. | §OBSTACLE_NOTES 1–5 |
| T1-27 | Bounded telemetry queue + overflow policy; Comm validates, never PWM; reconnection non-blocking | Impl. (publishing Blocked) | drop-oldest queues; `comm_fsm.c` |
| T1-28 | Distinct priorities, fixed periods, measure worst case | Impl. (measurement HW) | priorities 12–21; `task_stat.h` |
| T1-29 | R1: payload tags and error handling | Impl. | version/type/length/CRC + ACK |
| T1-30 | Bounded IMU queue; consumers ignore invalid/stale samples | Impl. | 32-entry queue; valid bit + 4-bit seq |
| T1-31 | Bounded scan queue with scan ids, validity, timeouts | Impl. | 3-bit scan id, status, 4 s profile timeout |
| T1-32 | Reject stale commands | Impl. | 3-bit mode epoch; S2-U14, INT-12b |
| T1-33 | Cross-subsystem tests | Partial | software/simulation: INT-10, INT-15, INT-16, S3-U10; physical runs §7 |

### 3.3 Week 6 functional requirements

| ID | Status | Where / evidence |
|---|---|---|
| FR1 line following | Impl. + HW | `line_follow.c`, `task_line.c`; S3-U18..U29, INT-14 |
| FR2 barcode scan/decode | Impl. + HW | Code 39 from samples; S3-U01..U14 |
| FR3 navigation by barcode with encoder motion | **Partial (TD)** | path implemented and tested (INT-11). No document defines which character means which command, so the map ships empty |
| FR4 encoder feedback for straight/turns | Impl. + HW | heading hold + arc turns; S2-U08..U11 |
| FR5 hump detection/height | Impl. + HW | `hump.c`; S4-U19..U23 |
| FR6 hump record + highest | Impl. | S4-U20, INT-16 |
| FR7 telemetry over WiFi/MQTT | **Blocked (transport)** | collected, encoded and queued; publishing needs the `mqtt_utk.c` patch (§8) |
| FR8 stop/reduce speed before scanning | Impl. + HW | speed cap < 450 mm, OBSTACLE stop |
| FR9 scan: position, width, clearance | Impl. + HW | `obstacle_profile.c`; S5-U06..U13 |
| FR10 choose action, send commands to B2 | Impl. | `avoid_plan.c`; S5-U14..U20, INT-10 |
| FR11 reacquire the line before resuming | Impl. + HW | search with B3 events; INT-14 |
| FR12 Safe Stop when unavoidable / profile fails / no line in time | Impl. | INT-14d, INT-18, S5-U18 |
| FR13 heartbeat; online / disconnected / recovering | Impl. (publishing Blocked) | heartbeat frame, `cfsm_availability()`; S1-U14..U18 |
| FR14 server validation, PostgreSQL, dashboard | **Not impl. (server side)** | outside the firmware repository; decoder for Node-RED provided |
| FR15 invalid sensor data cannot cause unsafe commands | Impl. | validity codes, fault bits, mode gating; S4-U31 logic, INT-13 |
| FR16 IMU calibration before monitoring | Impl. + HW | S4-U04..U07 |
| FR17 tilt events | Impl. + HW | S4 monitoring tests |
| FR18 motion class, collision | Impl. + HW | S4 monitoring tests, INT-13d |
| FR19 turn rate | Impl. + HW | S4 monitoring tests |

### 3.4 Week 6 non-functional requirements and architecture

| ID | Status | Where / evidence |
|---|---|---|
| NFR1 C on Pico + micro T-Kernel | Impl. | all C; professor's kernel, build and BSP unchanged |
| NFR2 Barr-C | Partial | followed where it matches the template style (fixed-width types, braces, no dynamic memory, bounded loops); no automated checker was run |
| NFR3 efficient memory/CPU | Impl. | static buffers, no `malloc`, integer maths; `DATA_REPRESENTATION.md` §B |
| NFR4 robust line tracking | Impl. + HW | polarity learnt, bounded search |
| NFR5 limited drift during avoidance | Impl. + HW | offset = edge + half width + margin |
| NFR6 `stop()` within 100 ms | Impl. + HW | event-driven; zero PWM next period; watchdog; S2-U18, S2-U44 (software path) |
| NFR7 ≤ 15 mm / m straight | Impl. + HW | heading hold; accuracy depends on encoder resolution (§7) |
| NFR8 turns ±10° | Impl. + HW | simulated turn errors 1.1–6.6° with 20-slot encoders (§6.2) |
| NFR9 network loss never stops local control | Impl. | INT-17, degraded-only network faults |
| NFR10 timed, capped, non-blocking retries | Impl. | S1-U17 |
| NFR11 malformed MQTT rejected safely | Impl. | S1-U08..U10, S1-U25 |
| NFR12 creep speed before a queued turn | Impl. | S2-U11 |
| NFR13 fixed-period IMU without blocking others | Impl. + HW | 10 ms; interrupt-driven I2C, task sleeps during transfers |
| NFR14 filtered/validated IMU | Impl. + HW | complementary filter, DLPF, minimum durations, stuck detection |
| W6-ARCH 10 tasks, no Vehicle Controller task | Impl. | `INTERFACES.md` §2 |
| W6-SM statecharts 6.1–6.5 + system | Impl. | `comm_fsm.c`, `motion_ctrl.c`, `line_follow.c` + `barcode_dec.c`, `task_imu.c`, `task_avoid.c`, `vehicle_fsm.c` |

### 3.5 Project Briefing deliverables and barcode specification

| ID | Status | Notes |
|---|---|---|
| PB-1 communication API, telemetry framework, MQTT documentation | Impl. | telemetry demo needs the transport patch |
| PB-2 motion library | Impl. | PID tuning report and motion accuracy evaluation need the car (S2-U48..U52) |
| PB-3 line following, barcode decoder, navigation interface | Impl. | meaning map is a TD |
| PB-4 IMU processing, hump detection | Impl. | terrain analysis report needs physical logs |
| PB-5 scanning, profile generator, avoidance + recovery | Impl. | |
| BC-1 symbology from the samples | Impl. | Code 39 established from all 11 samples |
| BC-2 geometry (141 / 20 / 18 mm) | Impl. | 47 units → 3 mm narrow; ratio-based classification |
| BC-3 symbol → command meaning | TD | not in any document; empty default |

## 4. Tribunal 1 compliance review (every confirmed change)

| Tribunal 1 item | Confirmed decision | How it is addressed |
|---|---|---|
| Change log 1 (no change) | Keep Scan / Profile / Avoidance separate while applying the bit requirement | three tasks; packed messages between them |
| Change log 2 (simplified) | Hump as a scaled integer 0–100 mm, 2 mm steps / 6 bits, or 4 mm / 5 bits after the noise check | `HUMP_STEP_MM 2`, code 63 = none, saturation flag; switch to 4 needs only the constant (5 bits then suffice) after S4-U36 |
| Change log 3 (redesign) | IR_RIGHT GP27 interrupt edge capture into a buffer, asynchronous decoding; 1 ms timer poll as backup | `drv_ir.c` ring + `task_barcode.c`; `S3_CAPTURE_POLL_1MS` (TIMER alarm 3), both built |
| Change log 4 (redesign) | All MQTT information as binary; standards enforced for Buddies 1–5 | protocol v1, `tlm_build.c` mapping, INT-15 across all five |
| Change log 5 (no change) | Highest hump at full resolution, compacted only when sent | `hump.c` µm; `hump_status()` compacts |
| Buddy 1 contribution | Standard encoding/decoding to keep values accurate | one table drives encode, decode and the size check; golden frames; third independent decoder (JS) |
| Buddy 2 contribution | B2 sole motor owner; minimum-bit ranges; shared fault bitmask; safe packed access; verify decoding and timing before integration | §3.2 T1-10, `INTERFACES.md` §9, S2-U01..U05; timing instrumentation |
| Buddy 3 contribution | Standard format; bit allocation for IR states, barcode, navigation, status flags | `s3_status.h` (32-bit line and barcode words) |
| Buddy 4 contribution | 2 mm steps, because the estimate is not 1 mm accurate | §A.3; quantisation ≤ 1 mm (S4-U25) |
| Buddy 5 contribution | Minimum bits, safe queue transfer, synchronisation; verify unpack = original | 10-bit 5 mm distances; round trips; profile accuracy unchanged (S5-U06) |
| Resource review 1 | Ranges/precision first; safe transfers; verify integrity | §A.4, S5-U01..U05 |
| Resource review 2 | IMU: calibrate still, filter, thresholds from tests; B2 keeps motors off during calibration | calibration only in INIT/SAFE_STOP (motors off there); thresholds `[CAL]` |
| Resource review 3 | Shared field specification + known-value test before integration | `DATA_REPRESENTATION.md`, golden frames, INT-15 |
| Resource review 4 | Mode owned by B2; events from B3/4/5 (barcode command, line lost, collision, obstacle, avoidance complete, recovery failed); B1 reads the mode; B3 stops steering outside NORMAL | `vehicle_fsm.c` + `SYS_EV_*`; barcode command via `nav_started`; mode in telemetry and heartbeat; `task_line.c` gating |
| Curveball B1 | Field widths, ranges, scaling, codes agreed; shared encode/decode; Node-RED update; before/after sizes | protocol doc §5–6, `nodered_decoder.js`, §C sizes |
| Curveball B2 | Minimum-bit motion data; fault bitmask; atomic access | command/result/steer/cap/status words; atomics |
| Curveball B3 | 2-bit codes for 4 navigation commands; extra bits for IR and barcode | `NAV_*` 2 bits + valid; 3-bit IR; 6-bit symbol |
| Curveball B4 | Full resolution in tasks; compact only when stored or sent | `imu_proc.c` vs `s4_status.c`, `hump.c` |
| Curveball B5 | Keep 3 tasks; define widths; safe transfers; verify pack/unpack | `s5_msg.c` + tests |
| Version A concessions (B2) | Single fixed period; event-driven stop/fault; done/aborted/fault result; retry limit → Safe Stop; creep from bench minimum | T1-15..T1-19 above |
| Version A positions (B3, B4) | Separate Line and Barcode tasks; hump from pitch + encoder distance; B4 never sends motor commands | implemented as stated |
| Version C 1 | Data representation table with range, bits, units, scaling, invalid codes | `DATA_REPRESENTATION.md` §A, protocol §5 |
| Version C 2 | Ownership, atomics/locking for bit-fields, ISR data, queues | `INTERFACES.md` §9 |
| Version C 3 | Obstacle precision: define ranges, verify packing and profile accuracy | 5 mm steps; S5-U06; INT-15b |
| Version C 4 | Fault bit allocation, ownership, Safe Stop conditions, B2/4/5 interaction | `sys_defs.h` + severity table; protocol §6 |
| Version C 5 | Measure execution time, jitter, queue latency with packing on | `task_stat.h` + on-target S*-U checks (to run) |
| Version C 6 | Firmware ↔ Node-RED known-value tests incl. invalid values and fault combinations | golden frames decoded by the JS decoder (7/7); INT-15a |
| Version C 7 | Hardware validation deferred until tested | §7 |
| Version C 8 | Quantify memory savings | `DATA_REPRESENTATION.md` §B–C ([ld], [sizeof]) |
| Risks R1–R4 | Payload tags; IMU period; barcode edge timing; measurement precision | R1: CRC/version/length/ACK. R2: interrupt-driven I2C, 10 ms + stats. R3: timestamp-only handlers, overflow flag, counters. R4: ranges + round trips |
| Readiness concerns | Wait-for-start, mission complete, obstacle during turning, fault clearing, collision response, shared fault-mask handling | Wait-for-start: INIT waits for both calibrations and the armed flag. Obstacle during a barcode turn → OBSTACLE aborts the turn (INT-12). Fault clearing: CLEAR/RESUME rules. Collision → Safe Stop (`SYS_COLLISION_STOPS`, TD flag). **"Mission complete" is not defined in any document and is not implemented (TD).** |

## 5. Binary data representation summary

* **Every shared item is packed with explicit masks and shifts:**
  * vehicle status word (18 bits);
  * motion command (28-bit word);
  * result (13), steering (16), speed cap (15);
  * motion status (2 words);
  * line status (32 + time) and barcode status (32);
  * IMU status (2 words + time), hump status (27), IMU→hump sample (24);
  * scan request (25), scan sample (25), profile (2 words);
  * scan status (28), avoidance status (20);
  * communication status (17);
  * fault mask (27), event word (10);
  * barcode edge record (31-bit time + level).
* **MQTT:** 5 binary frame types; telemetry 279 bits (40-byte frame),
  heartbeat 32 bits, event 36, command 24, ACK 20; CRC-16/CCITT-FALSE,
  LSB-first fields, little-endian CRC, version nibble.
* **Full tables:** `DATA_REPRESENTATION.md` §A and
  `subsystem1/docs/MQTT_BINARY_PROTOCOL.md` §5–6.

## 6. Test results (actual runs)

### 6.1 Builds (professor's makefile, on a scratch copy of the tree; `arm-none-eabi-gcc` 15.3)

| Profile | Exit | Warnings / errors | text | data | bss |
|---|---|---|---|---|---|
| template baseline (before any change) | 0 | — | 285 124 | 88 | 24 476 |
| default (`SMP=1`, `usb_cdc`, `cyw43`) | 0 | 0 | 317 414 | 112 | 27 128 |
| `TEST=1` | 0 | 0 | 385 432 | 112 | 29 396 |
| `SMP=0` | 0 | 0 | 310 414 | 100 | 22 980 |
| `SMP=0 TEST=1` | 0 | 0 | 378 160 | 100 | 25 312 |
| `CONSOLE=uart WIFI=none` | 0 | 0 | 93 755 | 36 | 18 684 |
| `CRASH_REPORT=1` | 0 | 0 | 317 630 | 112 | 27 128 |
| `NET=lwip MQTT=1` (dummy `wifi_secrets.h` in the scratch copy only) | 0 | 0 | 370 662 | 120 | 84 848 |
| `NET=lwip MQTT=1 TEST=1` | 0 | 0 | 438 656 | 120 | 87 180 |
| `S3_CAPTURE_POLL_1MS=1` (1 ms barcode polling backup) | 0 | 0 | 317 542 | 112 | 27 136 |
| `S3_CAPTURE_POLL_1MS=1 TEST=1` | 0 | 0 | 385 560 | 112 | 29 404 |

Every subsystem `.c` file also compiles with the firmware flags plus
`-Wall -Wextra`, with `APP_TEST=0` and `APP_TEST=1`: 0 warnings.

### 6.2 Software tests (`make -C app_program/subsystem2/host_test`)

**178 checks, 0 failures** (exit status 0). Suite totals:

| Suite | Checks | Content |
|---|---|---|
| S1 | 20 | CRC, bit order, golden frames, limits, every rejection path, command mapping, sizes (binary 40 B vs JSON 697 B), connection state machine incl. back-off 1/2/4/8/16/16 s and reconnection |
| S2 | 41 | formats, PID anti-windup, MOVE/TURN on a wheel model, creep chaining, arbitration, stop, stall/encoder faults, escalation, steering, mode machine, severity policy |
| S3 | 32 | Code 39 table; **all 11 supplied samples decode**; reversed; scale; invalid patterns; drive simulation 45/45 at 100/200/300 mm/s; duplicates; line following, search, junction; status words |
| S4 | 29 | integer maths, calibration acceptance, filter, tilt, collision, motion class, stuck/saturated data, hump height ±4 mm, highest hump, quantisation, packing |
| S5 | 21 | distance code, messages, profile accuracy (width within 10 %), clear path, far object, wall (unknown edges), frozen servo, planning, no invented bypass |
| INT | 35 | INT-10..INT-20 (see `TEST_CASES.md` §6) |

Figures printed by the simulation runs:

* bypass of a 150 mm box: closest 201 mm beside it, back on the line,
  heading −8.0°;
* barcode navigation turns: +91.1° (RIGHT), −83.5° (LEFT),
  +173.4° (U-turn), +3.2° drift on STRAIGHT 120 mm.

These are **model results** with 20-slot encoders (10.2 mm per tick =
4.5° of rotation per tick); see §7, S2-U52.

`node app_program/subsystem1/docs/nodered_decoder.js` gives 7/7 PASS. The
server decoder matches the firmware on all golden frames, and rejects a
corrupted CRC and a short frame.

### 6.3 Defects found and fixed during verification (before this report)

1. `op_build`:
   * "no echo" samples were not counted as readings, so a clear path was
     reported as INVALID;
   * grouping by range cut a flat wall into a fake finite obstacle.

   It now uses a forward-distance model (S5-U08..U10).
2. INT-11: test barcode re-read inside the duplicate window. This was a test
   bug, and the decoder behaved correctly.
3. Telemetry overwrote the drop counter and lost the MQTT-queue drops. The
   mapping now goes through `tlm_build.c`.
4. `motor_off()` / `servo_set()` could write PWM registers before the PWM
   block was released. Both are now guarded.
5. On-target tests would have run on the initial task's **1 KB stack**
   (several suites need more). They now run in a dedicated 8 KB test task
   (`app_test.c`).
6. A stale wake-up after a test run would have made `usermain()` return and
   shut the kernel down. Pending wake-ups are now cancelled, and a hung test
   task is terminated.
7. In `CRASH_REPORT=1` / `CONSOLE=uart` builds the encoder driver took
   GP0/GP1 from the UART console. Those pins are now left alone (README §3).
8. The IMU snapshot had no timestamp (Tribunal 1). A time word was added.
9. The obstacle profile buffer used 12-byte samples. It now uses 4 bytes
   (−512 B RAM, measured).

## 7. Remaining hardware validation (the team must do these on the car)

Rows are numbered as in `TEST_CASES.md` §7.

* **Calibration constants (`[CAL]`)** in `motion_cfg.h`, `line_follow.h`,
  `imu_proc.h`, `hump.h`, `drv_servo.h`, `drv_ultrasonic.h`,
  `subsystem5.h`, `avoid_plan.h`:
  * wheel diameter, encoder slots, track width, motor polarity;
  * feed-forward, PID gains, minimum controllable speed (creep);
  * stall thresholds;
  * IR polarity/placement;
  * IMU axis map and thresholds;
  * servo centre/scale/direction, speed of sound;
  * car width/length and margins;
  * detection distances.
* **Buddy 2:**
  * wheel/encoder geometry (S2-U48);
  * polarity and minimum speed (U49);
  * PID tuning report (U50);
  * NFR7 straight 1 m (U51);
  * NFR8 turns (U52). With 20 slots per revolution one tick is 10.2 mm
    (≈ 4.5° of rotation). If turns miss ±10°, set `ENC_COUNT_BOTH_EDGES 1`
    (or `ENC_QUADRATURE 1` once quadrature is confirmed, A3) to halve or
    quarter the step;
  * `stop()` latency at the PWM pins with a scope (U53);
  * stall/escalation (U54);
  * GP28 VBAT jumper and GP0/GP1 edges (U55).
* **Buddy 3:**
  * calibration on the line (S3-U36);
  * line following incl. curves and lighting (U37);
  * **15/15 barcode reads at 3 speeds** (U38, T1-33);
  * barcode vs curve (U39, A4);
  * line search (U40);
  * edge load at top speed (U41);
  * the team must fill the navigation map (FR3) and decide
    `S3_NAV_AT_JUNCTION` and `S3_STRAIGHT_MM`.
* **Buddy 4:**
  * identify the IMU module and verify its register map, addresses and axis
    signs (S4-U33);
  * I2C/period timing (U34);
  * thresholds from logs (U35);
  * **known hump ×5 within ±4 mm**, then decide 2 vs 4 mm steps (U36);
  * confirm the collision response (`SYS_COLLISION_STOPS`).
* **Buddy 5:**
  * ultrasonic accuracy at 3.3 V (S5-U24);
  * servo calibration (U25);
  * profile vs real boxes (U26);
  * full avoidance, timeout and unavoidable cases (U27).
* **Buddy 1:**
  * apply the MQTT patch, then end-to-end telemetry/heartbeat (S1-U27);
  * network loss during driving (U28, A1);
  * malformed server commands (U29);
  * update the Node-RED flow to the binary decoder, and the PostgreSQL /
    Grafana side (FR14).
* **System and timing:**
  * flash the `TEST=1` image and save the log. This gives the TIMER rate,
    task periods, worst-case run times, stop-request latency and every
    automated check on the real CPU;
  * INT-21..24 and SYS-01..02;
  * **all timing claims stay open until that log exists.**

## 8. Protected file verification

**Method.** Before any change, the SHA-1 of all 643 tracked files was
recorded, together with the git index hash. After implementation:

* `git status --porcelain --untracked-files=all` shows changes only under
  `app_program/subsystem1..5/`. The only other entries are the untracked
  reference documents in `documents/`, which the user supplied as inputs;
  none were created or edited.
* `git diff --cached` is empty: nothing staged, nothing committed.
* Re-hashing every tracked file: **all 34 differing hash entries are inside
  `app_program/subsystem1..5/`**. These are 15 modified files and the 4
  deleted template examples; 639 of the 643 files still exist.
  **No tracked file outside the subsystem folders changed.**
* `config/wifi_secrets.h` does **not** exist in the repository. The dummy
  copy used for the `NET=lwip MQTT=1` builds existed only in the scratch
  copy.
* All builds and host-test outputs were written outside the repository.

**Changes outside the subsystem folders that the implementation needs
(reported, NOT applied):**

| File (protected) | Change | Why | Document |
|---|---|---|---|
| `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` | call the four `mqtt_bridge.h` hooks; raw (not printable-filtered) inbound payloads; subscribe `csc14/cmd`; publish Buddy 1's queue instead of the demo heartbeat | FR7, FR13, remote commands; `TEAM.md` §3 requires network calls inside the service task | `subsystem1/docs/MQTT_INTEGRATION_PATCH.md` |
| `TEAM.md` | pins, peripherals, priorities (deviation from default bands), interfaces, kernel object budget | team bookkeeping; changed only by agreement | `TEAM_MD_PROPOSAL.md` |
| `testing/*/TEST_LOG.md` | test-case rows for the new test IDs | `testing/README.md` "define it first" | `TEST_CASES.md` |
| `config/wifi_secrets.h` (gitignored) | team creates it locally for `NET=lwip MQTT=1` | credentials | `MQTT_INTEGRATION_PATCH.md` |

No other external change is required. The build already compiles every
new file; the kernel configuration has enough objects (`INTERFACES.md` §3);
and the professor's I2C driver stays disabled because Buddy 4 has its own
driver.

## 9. Open team decisions (implemented with the safest default, flagged)

| Decision | Default | Where |
|---|---|---|
| Barcode character → navigation command | empty map: decode and report, never move | `nav_map.c` `S3_NAV_MAP_DEFAULT` |
| Execute the barcode command at once or at the next junction | at once | `S3_NAV_AT_JUNCTION` |
| Collision response | Safe Stop (Week 6 statechart) | `SYS_COLLISION_STOPS` |
| IMU / ultrasonic required for driving | yes (critical) | `SYS_IMU_REQUIRED`, `SYS_US_REQUIRED` |
| Hump step 2 mm or 4 mm | 2 mm | `HUMP_STEP_MM` |
| Mission complete / end of run | not defined anywhere; not implemented | — |
| Task priorities vs the `TEAM.md` default bands | Week 6 / Tribunal 1 functional order | `TEAM_MD_PROPOSAL.md` §4 |
