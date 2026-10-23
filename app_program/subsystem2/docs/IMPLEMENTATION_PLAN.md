# CSC14 Firmware — Implementation Plan and Requirements Traceability

Status legend used in this file: **Pending** (planned, not yet built),
**Done** (implemented in `app_program/subsystemN/`), **Partial**,
**Blocked** (needs a change outside the subsystem folders),
**Team decision** (implemented with the safest compatible default and
flagged for confirmation), **Hardware validation** (code exists, physical
test needed). The final status of every row is in
`VERIFICATION_REPORT.md` (same folder).

This plan was written before coding, as the brief requires, and is kept
unchanged as that record. Some planned names changed during
implementation:

| Planned name | Implemented as |
|---|---|
| `motion_if.h` | `motion_msg.h` (formats) + `subsystem2.h` (API) |
| `sys_state.h` | `sys_defs.h` (definitions) + `sys_state.c` |
| `MOTION_RESULT` | the result word in `motion_msg.h` |
| `MOTION_MAX_FAULT_RETRIES`, `MOTION_MIN_CTRL_SPEED_MMS` | `MAX_FAULT_RETRIES`, `MIN_CTRL_SPEED_MMS` (`motion_cfg.h`) |
| `motion_creep_speed()` | `mc_creep_speed()` / `motion_creep_speed_mms()` |
| `S3_NAV_*` | `NAV_*` (`nav_map.h`) |

Also, the hump height is kept internally in µm (not 0.1 mm).

## 1. Scope and interpretation

* The brief allows writes only inside the "existing `subsystems/`
  directory". The repository has no folder literally named `subsystems/`;
  the per-buddy folders are `app_program/subsystem1/` … `subsystem5/`
  (README §2.5: "One folder per student"). All work is confined to those
  five folders. `app_program/app.h`, `app_program/app_main.c`,
  `app_program/test_report.h`, `TEAM.md`, `testing/`, `config/`, `lib/`,
  `kernel/`, `device/`, `build_make/` are treated as protected.
* Build discovery: `build_make/mtkernel_3/app_program/subdir.mk` compiles
  `app_program/*.c` and `app_program/*/*.c` (one sub-level). Every new `.c`
  file placed directly in `app_program/subsystemN/` is built automatically;
  files in deeper folders (`docs/`, `host_test/`) are **not** linked into the
  firmware, which is used deliberately for host-side test harnesses.
* Target: Raspberry Pi Pico W (RP2040, Cortex-M0+, no FPU, soft-float),
  125 MHz `clk_sys`/`clk_peri`, micro T-Kernel 3.0 dual-core (SMP) port,
  10 ms system tick (`CNF_TIMER_PERIOD`), GCC `arm-none-eabi` 15.3,
  `-std=gnu11 -O2`, Pico SDK 2.x only for USB/CYW43/lwIP.

## 2. Repository facts that constrain the design

| Fact | Source | Consequence |
|---|---|---|
| 10 ms kernel tick | `config/config.h` | Periods are multiples of 10 ms; sub-ms timing uses the RP2040 µs TIMER (`TIMERAWL`), as the professor's USB/CYW43 code already does. A 1 ms poll needs a TIMER alarm IRQ. |
| Kernel I2C driver disabled (`DEVCNF_USE_IIC 0`) | `config/config_device.h` | Buddy 4 ships its own register-level I2C0 driver `drv_i2c0.c` (README: drivers go in the subsystem folder). |
| PWM block held in reset unless `USE_PTMR` | `kernel/sysdepend/pico_rp2040/hw_setting.c` | Motor/servo drivers release (never re-assert) the PWM reset themselves. |
| No GPIO IRQ used by kernel/USB/CYW43 (CYW43 is polled) | grep of `kernel/`, `lib/` | `IO_IRQ_BANK0` (IRQ 13) is free; one shared dispatcher (Tribunal 1 shared-resource row) serves encoders, barcode edges and ultrasonic echo. |
| SMP spinlock/atomic API only exists when `TK_SUPPORT_SMP` | `kernel/tkernel/smp_lock.c` | `app_sync.h` wraps it, falling back to interrupt masking on `SMP=0`. |
| ISR-callable kernel calls: `tk_set_flg`, `tk_wup_tsk`, `tk_sig_sem`, `tk_snd_mbf(TMO_POL)` | `kernel/tkernel/*.c` context checks | ISRs only timestamp/count and signal. |
| WiFi/lwIP/MQTT may only run inside `cyw43_utk_task` | `TEAM.md` §3, `mqtt_utk.c` | Buddy 1 cannot publish by itself. It owns bounded message buffers and non-blocking hook functions; **wiring them into `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` is a protected-file change** (reported, not applied — see `subsystem1/docs/MQTT_INTEGRATION_PATCH.md`). |
| `link_status` is only meaningful with `NET=lwip` | `cyw43_utk.h` | Communication state machine reports `DISABLED` on builds without the network stack. |
| Default build leaves GP0/GP1 free (`BOARD_UART0_FREE=1`); `CRASH_REPORT=1` takes them | `build_make/pico_rp2040.mk` | Left encoder (GP0/GP1) cannot coexist with `CRASH_REPORT=1`. Flagged. |
| Kernel objects: 16 flags, 12 message buffers, 32 tasks, 12 cyclic handlers; kernel/drivers already use ~4 flags, 3 tasks | `config/config.h`, kernel grep | Budget in §6 stays within limits. |

## 3. Document precedence applied

1. Change Requirement 1 (minimum-bit representation) — mandatory.
2. Tribunal 1 Version B decisions + the team's interface map / shared-resource
   matrix / Version C "action taken" entries.
3. Week 6 Design Review where not superseded.
4. Project Briefing.
5. Professor's code base (build, RTOS, BSP) as an implementation constraint.

AI suggestions in Tribunal 1 Version A/C are **not** treated as approved unless
the team adopted them in Version B, in the interface map, or as an
"Action taken".

## 4. Requirements traceability matrix

### 4.1 Mandatory professor requirement — Change Requirement 1 (CR1)

| ID | Requirement | Source | Buddy | Implementation plan | Status |
|---|---|---|---|---|---|
| CR1-1 | Each application field uses only the bits its valid range needs (bit-fields or masks/shifts) | CR1 ¶1 | 1–5 | Field table in `DATA_REPRESENTATION.md`; every shared word/queue message packed with explicit masks/shifts in `*_pack/*_unpack` helpers | Pending |
| CR1-2 | Faults as individual bits of one shared fault bitmask | CR1 ¶1 | 2 (owner), all | `sys_state.h` `SYS_FLT_*` bits, `sys_fault_set/clear` atomic | Pending |
| CR1-3 | Applies to motion/encoder, line/barcode, obstacle, hump, vehicle/controller states, ISR↔task data, queue messages, MQTT payloads | CR1 ¶2 | 1–5 | One packed representation per item (see §5) | Pending |
| CR1-4 | MQTT application data as compact binary fields, not text | CR1 ¶3 | 1 | `mqtt_proto.c` binary frames (version, type, seq, length, CRC-16, LSB-first bit fields) | Pending |
| CR1-5 | Safe access to shared packed data (no RMW races; atomic/sync) | CR1 ¶4 | 1–5 | `app_sync.h` (spinlock/atomic wrapper), single-writer words, seqlock for encoder ISR data, SPSC ring for barcode edges | Pending |
| CR1-6 | Week 13: data representation table | CR1 Wk13 | 1–5 | `DATA_REPRESENTATION.md` | Pending |
| CR1-7 | Week 13: RAM usage before/after | CR1 Wk13 | 1–5 | Measured `sizeof` (target ABI) and linker map figures; no fabricated baseline | Pending |
| CR1-8 | Week 13: message size before/after | CR1 Wk13 | 1–5 | Encoded byte counts (binary vs. equivalent JSON) and queue element sizes, measured by tests | Pending |
| CR1-9 | Week 13: correctness and performance evidence | CR1 Wk13 | 1–5 | Host + on-target known-value tests; task timing instrumentation (`task_stat`) | Pending |

### 4.2 Tribunal 1 confirmed decisions (T1)

| ID | Decision | Source | Buddy | Implementation plan | Status |
|---|---|---|---|---|---|
| T1-01 | Keep Obstacle Scan / Obstacle Profile / Avoidance & Recovery as three tasks while applying the bit requirement | T1 Vers. B change log row 1 | 5 | `task_scan.c`, `task_profile.c`, `task_avoid.c` | Pending |
| T1-02 | Hump height stored/sent as scaled integer 0–100 mm, 2 mm steps (6 bits); 4 mm/5 bits only after known-hump test | T1 Vers. B change log row 2, Buddy 4 row | 4 | `hump.h` `HUMP_STEP_MM` (default 2), code 63 = invalid | Pending |
| T1-03 | IR_RIGHT (GP27) interrupt edge capture into a buffer, decoded asynchronously; 1 ms timer poll retained as backup | T1 Vers. B change log row 3 | 3 | `drv_ir.c` ISR → SPSC ring; `S3_CAPTURE_POLL_1MS` backup using TIMER alarm 3 | Pending |
| T1-04 | MQTT information sent in binary; one encode/decode standard for Buddies 1–5 | T1 Vers. B change log row 4 | 1 | `bitpack.c`, `mqtt_proto.c`, `MQTT_BINARY_PROTOCOL.md` | Pending |
| T1-05 | Highest hump tracked at full resolution inside the task; compacted only when stored/sent | T1 Vers. B change log row 5 | 4 | `hump.c` keeps 0.1 mm internally | Pending |
| T1-06 | Minimum-bit inter-task messages; safe queue transfers; verify unpacked == original | T1 Vers. B resource review row 1, Buddy 5 row | 5 (and all) | Packed 32/64-bit messages over kernel message buffers; round-trip tests | Pending |
| T1-07 | IMU: calibrate while still, filter, thresholds from real tests | T1 Vers. B resource review row 2 | 4 | Stationarity-checked bias calibration, complementary filter, configurable thresholds | Hardware validation |
| T1-08 | Shared field specification + known-value encode/decode test before integration | T1 Vers. B resource review row 3; Vers. C rows 1, 6 | 1 coordinates, 2–5 | `DATA_REPRESENTATION.md`, `s1_unit_tests.c` known vectors | Pending |
| T1-09 | Operating mode owned by Buddy 2; Buddies 3/4/5 send events (barcode command, line lost, collision, obstacle detected, avoidance complete, recovery failed); Buddy 1 reads mode for telemetry; Buddy 3 reads mode to stop steering outside Normal Navigation | T1 Vers. B resource review row 4 | 2 (+3,4,5,1) | `sys_state.c` mode FSM inside Motion Control task; `sys_post_event()` | Pending |
| T1-10 | Buddy 2 remains sole motor owner; min-bit motion data; shared fault bitmask; safe packed access; verify decoding + timing | T1 Vers. B Buddy 2 row | 2 | `drv_motor.c` only writer of GP8–GP11 PWM | Pending |
| T1-11 | Standardised Buddy 3 format; 2-bit codes for 4 navigation commands; extra bits for IR combinations / barcode data | T1 Vers. B Buddy 3 row; curveball Buddy 3 | 3 | `S3_NAV_*` 2-bit enum + valid bit; 3-bit IR state; 6-bit symbol | Pending |
| T1-12 | Buddy 5 min-bit representation, safe queue transfer, verify unpack == pack | T1 Vers. B Buddy 5 row; curveball Buddy 5 | 5 | `s5_msg.c` pack/unpack + tests | Pending |
| T1-13 | Buddy 4 computes at full resolution, compacts only for storage/transfer | T1 Vers. B Buddy 4 row; curveball Buddy 4 | 4 | `imu_proc.c` (0.01°, mg, mdps internal), packed status word | Pending |
| T1-14 | Interface rule: steering accepted only in Normal Navigation; command priority Safe Stop/Fault > Obstacle Avoidance > Barcode Navigation > Line Following; `stop()` bypasses the queue, flushes queued commands, effective within 100 ms; invalid/out-of-range requests rejected; Fault disables motor output | T1 interface map (Motion Control row); Week 6 §8.3 | 2 | `motion_ctrl.c` arbitration, `motion_stop()` event-driven bypass | Pending |
| T1-15 | Motion-complete notification carries done / aborted / fault | T1 Buddy 2 position; interface map | 2 | `MOTION_RESULT` 2-bit status (+ REJECTED) per source | Pending |
| T1-16 | Fixed Motion Control period (single value) and event-driven stop/fault handling | T1 Buddy 2 position (conceded) | 2 | 20 ms cyclic handler + `flg_motion` STOP/FAULT bits | Pending |
| T1-17 | Fault retry limit that escalates to system Safe Stop (no endless Fault↔Idle cycling) | T1 Buddy 2 position; Version A gap | 2 | `MOTION_MAX_FAULT_RETRIES` → `SYS_FLT_MOTION_ESCALATED` (critical) | Pending |
| T1-18 | Creep speed = max(15 % cruise, measured minimum controllable speed) | T1 Buddy 2 position; assumption A2 next action | 2 | `motion_creep_speed()`; `MOTION_MIN_CTRL_SPEED_MMS` placeholder | Hardware validation |
| T1-19 | Numeric stall criterion (PWM applied, no encoder ticks) | T1 Buddy 2 position | 2 | `STALL_DUTY_MIN_PM`, `STALL_TIME_MS` | Pending |
| T1-20 | Encoder ISR only updates counters; counts read atomically; one shared GPIO interrupt handler agreed with Buddies 3/5 | T1 interface map (Encoder row); shared-resource matrix | 2, 3, 5 | `drv_gpio_irq.c` dispatcher; seqlock-read encoder snapshot | Pending |
| T1-21 | Barcode ISR only records edge times, no decoding/motor control; reject incomplete readings; overflow flag | T1 interface map (Barcode edge capture row) | 3 | `drv_ir.c` | Pending |
| T1-22 | Barcode Decoding Task decodes saved timings; invalid barcode must not produce a command | T1 interface map | 3 | `barcode_dec.c` + `code39.c` | Pending |
| T1-23 | Line Following publishes line status (on line / lost / reacquired) with timestamp; no motor PWM | T1 interface map; shared-resource row "Line status" | 3 | Packed line status word + `flg_line_evt` | Pending |
| T1-24 | IMU Monitoring is the only I2C0 user; cleaned sample with timestamp + validity flag; no motor commands | T1 interface map; shared-resource row | 4 | `task_imu.c` owns `drv_i2c0`; snapshot with seq/valid | Pending |
| T1-25 | Hump & Terrain uses validated pitch + encoder distance from Motion Control; no hardware access | T1 interface map | 4 | `mbf_imu_hump` samples carry pitch + Δdistance | Pending |
| T1-26 | Obstacle scanning: only Obstacle Scan controls scanning hardware; profile rejects invalid/incomplete scans; avoidance uses Buddy 3 line status, never motor PWM | T1 interface map (Buddy 5 rows) | 5 | `task_scan.c` owns US + servo | Pending |
| T1-27 | Telemetry: bounded queue, agreed overflow policy, non-blocking collection; WiFi task validates commands and never writes motor PWM; reconnection must not block local control | T1 interface map (Buddy 1 rows) | 1 | `mbf_telemetry` drop-oldest; `comm_fsm.c` capped backoff | Pending |
| T1-28 | Distinct priorities, fixed periods, no long operations in High tasks; measure worst-case run times | T1 shared-resource matrix (CPU row); Vers. C row 5 | all | Distinct priorities 12–21 (§6), `task_stat` instrumentation | Pending |
| T1-29 | Payload tags/error handling for MQTT values (R1) | T1 risk register R1 | 1 | Version/type/length/CRC-16 + range validation + ACK codes | Pending |
| T1-30 | Bounded event queue; consumers ignore invalid or stale samples | T1 shared-resource matrix (IMU row) | 4 | `mbf_imu_hump` bounded, `seq`/`valid` checks | Pending |
| T1-31 | Bounded scan queue with scan identifiers, validity, timeout handling | T1 shared-resource matrix (scan queue row) | 5 | 3-bit `scan_id` in every sample | Pending |
| T1-32 | Command queue: reject stale commands | T1 shared-resource matrix (motor queue row) | 2 | Mode-epoch (3 bits) in every command; epoch bump flushes queue | Pending |
| T1-33 | Cross-subsystem tests (bit-packed avoidance ↔ motion; hump ±4 mm; binary interfaces 1–5 incl. min/max/invalid/fault combos; barcode at 3 speeds × 15 trials) | T1 subsystem test plan | all | Host + on-target tests; physical ones documented as pending | Pending |

### 4.3 Week 6 Design Review (W6)

| ID | Requirement | Buddy | Implementation plan | Status |
|---|---|---|---|---|
| FR1 | Follow black line with IR sensors | 3 | `line_follow.c`, `task_line.c` | Pending |
| FR2 | Scan and decode barcodes | 3 | `drv_ir.c`, `barcode_dec.c`, `code39.c` | Pending |
| FR3 | Execute Turn Left / Turn Right / Go Straight / U-Turn with encoder-based motion | 3, 2 | `nav_map.c` → `MOTION_CMD` TURN/MOVE | Team decision (symbol→command map not specified) |
| FR4 | Encoder feedback for straight motion and turns | 2 | `motion_ctrl.c` heading hold + arc-length turns | Pending |
| FR5 | Detect humps and estimate height with IMU | 4 | `hump.c` | Pending |
| FR6 | Record hump data and highest hump of the run | 4 | `hump.c` max tracking (full res) | Pending |
| FR7 | Telemetry over WiFi/MQTT (speed, sensor status, state, measurements) | 1 | `task_telemetry.c` + `mqtt_proto.c` | Blocked (transport hook in protected `mqtt_utk.c`); encoding Done-able |
| FR8 | Stop or reduce speed before scanning an obstacle | 5, 2 | speed cap + obstacle event → controlled stop | Pending |
| FR9 | Servo-mounted ultrasonic scan; position, width, clearance | 5 | `obstacle_profile.c` | Pending |
| FR10 | Choose avoidance action and send stop/turn/reverse/move to Motion Control | 5 | `avoid_plan.c`, `task_avoid.c` | Pending |
| FR11 | Reacquire original line before resuming | 5, 3 | Search step + `flg_line_evt` | Pending |
| FR12 | Safe Stop if unavoidable, profiling fails, or line not reacquired in time | 5, 2 | `SYS_FLT_*` critical bits | Pending |
| FR13 | Periodic heartbeat; report online / disconnected / recovering | 1 | Heartbeat frame + comm state | Partial (transport blocked) |
| FR14 | Server-side validation, PostgreSQL storage, dashboard | 1 (server) | Outside firmware; decoder spec + Node-RED decoder reference provided | Blocked (external system) |
| FR15 | Invalid/unavailable sensor data must not cause unsafe commands | all | Validity flags, fault bits, mode gating | Pending |
| FR16 | Calibrate IMU before monitoring; apply offsets | 4 | `imu_proc.c` | Pending |
| FR17 | Detect tilt/orientation changes; report tilt events | 4 | `imu_proc.c` | Pending |
| FR18 | Classify motion; detect impacts/collisions | 4 | `imu_proc.c` | Pending |
| FR19 | Turn rate + motion events | 4 | `imu_proc.c` | Pending |
| NFR1 | Pico + micro T-Kernel + Pico C SDK in C | all | C only, existing kernel | Pending |
| NFR2 | Barr-C coding standard | all | Followed where compatible with template style | Pending |
| NFR3 | Efficient memory/CPU | all | Static buffers, packed data, integer maths | Pending |
| NFR4 | Robust line tracking (lighting, imperfections, speed) | 3 | Polarity calibration, debounced states | Hardware validation |
| NFR5 | Limit drift during avoidance | 5 | Bounded lateral offset from profile | Pending |
| NFR6 | `stop()` halts motor output within 100 ms | 2 | Event-driven stop + motor watchdog | Pending |
| NFR7 | ≤15 mm deviation per 1 m straight | 2 | Heading hold from encoder difference | Hardware validation |
| NFR8 | Turn within ±10° | 2 | Arc-length turn + creep end | Hardware validation |
| NFR9 | Network loss must not stop local control | 1 | Comm faults are non-critical; no blocking calls | Pending |
| NFR10 | Timed, non-blocking retries, capped delay | 1 | `comm_fsm.c` 1→16 s backoff | Pending |
| NFR11 | Malformed MQTT rejected without crash/block | 1 | Length/CRC/range validation | Pending |
| NFR12 | Creep speed before a queued turn (no full stop) | 2 | `motion_ctrl.c` chaining | Pending |
| NFR13 | Fixed-period IMU sampling without blocking others | 4 | 10 ms cyclic + IRQ-driven I2C | Pending |
| NFR14 | Filter/validate IMU to reduce false events | 4 | Complementary filter + min-duration thresholds | Pending |
| W6-ARCH | 10 RTOS tasks as in §3 task table; no separate Vehicle Controller task | all | Exactly the 10 tasks | Pending |
| W6-SM1..5 | Subsystem statecharts 6.1–6.5 and system statechart §5 | all | One FSM per chart (see §7) | Pending |

### 4.4 Project Briefing deliverables (PB)

| ID | Deliverable | Buddy | Plan | Status |
|---|---|---|---|---|
| PB-1 | Communication API, telemetry framework, MQTT documentation, telemetry demo | 1 | `subsystem1.h`, `MQTT_BINARY_PROTOCOL.md` | Pending (demo needs transport patch) |
| PB-2 | Motion-control library, PID tuning report, motion accuracy evaluation | 2 | `motion_if.h`, `pid.c`; tuning/accuracy are physical tests | Partial |
| PB-3 | Line-following module, barcode decoder, navigation command interface | 3 | `line_follow.c`, `code39.c`, `nav_map.c` | Pending |
| PB-4 | IMU processing module, hump-detection algorithm, terrain analysis report | 4 | `imu_proc.c`, `hump.c`; report needs physical logs | Partial |
| PB-5 | Scanning subsystem, obstacle profile generator, avoidance + recovery algorithms | 5 | `task_scan.c`, `obstacle_profile.c`, `avoid_plan.c` | Pending |

### 4.5 Barcode specification (BC)

| ID | Item | Evidence | Plan | Status |
|---|---|---|---|---|
| BC-1 | Symbology | All supplied samples decode as Code 39 with `*` start/stop: Sample PDF p1–2 `*A*`, p3–4 `*Z*`; BARCODE.pptx EMF/PNG `*A*`, `*F*`, `*Z*`, `*W*`, `*X*`, `*Y*`; spec photo `*A*` (measured, see `subsystem3/docs/BARCODE_ANALYSIS.md`) | `code39.c` | Pending |
| BC-2 | Geometry | 141 mm barcode (47 narrow units ⇒ narrow ≈ 3 mm, wide ≈ 9 mm, ratio 3:1 measured), 20 mm gap to line, 18 mm line | Width classification is ratio-based (scale independent) | Pending |
| BC-3 | Symbol → navigation meaning | **Not specified in any document** | Configurable table, empty by default | Team decision |

## 5. Integration design (summary — full detail in `INTERFACES.md`)

* **Ownership.** Buddy 2 owns motors, encoders, the shared GPIO IRQ, the
  vehicle mode and the shared fault bitmask. Buddy 3 owns IR sensors (incl.
  GP27 edge capture). Buddy 4 owns I2C0/IMU. Buddy 5 owns ultrasonic + servo.
  Buddy 1 owns the telemetry/command buffers (the radio stays with the
  kernel's `cyw43_utk_task`).
* **Commands to Buddy 2.** Discrete commands (MOVE/TURN/STOP/SET_SPEED/
  RESUME/CLEAR_FAULT) are packed 32-bit words in `mbf_motion_cmd`.
  Continuous line steering is a single-writer packed word (no queue, newest
  wins, freshness checked). Speed caps are single-writer words.
  `motion_stop()` bypasses the queue via `flg_motion`.
* **Arbitration.** Safe Stop/Fault > Obstacle Avoidance > Barcode Navigation
  > Line Following; supervisory STOP is always accepted, other supervisory
  commands only where they cannot override safety.
* **Completion.** Per-source result word + `flg_motion_done` bit.
* **Events to Buddy 2.** `sys_post_event()` sets bits Buddy 2 consumes in its
  mode FSM.
* **Faults.** One 32-bit bitmask; each bit has one owner and a severity
  (critical ⇒ Safe Stop, degraded ⇒ report only).

## 6. RTOS task set (Week 6 task table, distinct priorities per Tribunal 1)

| Task | Buddy | Prio | Core | Period/trigger | Week 6 class |
|---|---|---|---|---|---|
| Motion Control | 2 | 12 | 1 | 20 ms cyclic + STOP/FAULT events | High |
| IMU Monitoring | 4 | 13 | 1 | 10 ms cyclic | High |
| Line Following | 3 | 14 | 1 | 20 ms cyclic | High |
| Obstacle Scan | 5 | 15 | 1 | 60 ms ping cycle / requests | High |
| Avoidance & Recovery | 5 | 16 | any | event | High |
| Barcode Decoding | 3 | 17 | any | edge events | Medium |
| Hump & Terrain Analysis | 4 | 18 | any | queue | Medium |
| Obstacle Profile | 5 | 19 | any | queue | Medium |
| WiFi & MQTT Communication | 1 | 20 | 0 | 100 ms cyclic + events | Medium |
| Telemetry Collection | 1 | 21 | 0 | 500 ms cyclic | Low |

Priorities stay inside the template's application range 12–21 (TEAM.md §4
bands are a default; Week 6/Tribunal 1 ordering by function takes
precedence). High tasks run on core 1, away from the kernel's USB (prio 3) and
CYW43 (prio 4) service tasks pinned to core 0. On `SMP=0` affinity is dropped
by `app_start_task()`.

## 7. Implementation order

1. Buddy 2 platform layer (`app_sync.h`, `drv_hwtime.h`, `drv_gpio_irq.c`,
   `sys_state.c`) — everything else depends on it.
2. Buddy 2 motion (`motion_if.h`, `motion_cmd.c`, `pid.c`, `motion_ctrl.c`,
   `drv_motor.c`, `drv_encoder.c`, `task_motion.c`).
3. Buddy 3 (`code39.c`, `barcode_dec.c`, `nav_map.c`, `line_follow.c`,
   `drv_ir.c`, tasks).
4. Buddy 4 (`drv_i2c0.c`, `drv_imu.c`, `imu_proc.c`, `hump.c`, tasks).
5. Buddy 5 (`drv_ultrasonic.c`, `drv_servo.c`, `s5_msg.c`,
   `obstacle_profile.c`, `avoid_plan.c`, tasks).
6. Buddy 1 (`bitpack.c`, `mqtt_proto.c`, `comm_fsm.c`, `mqtt_bridge.c`,
   tasks).
7. Unit/integration tests (host + on-target), builds for all profiles,
   protected-file verification, documentation.
