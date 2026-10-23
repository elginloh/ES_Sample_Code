# CSC14 firmware: subsystem interfaces, ownership and arbitration

This document defines how the five subsystems work together. Buddy 2 owns
it because Buddy 2 owns the motors, the vehicle mode and the fault mask
(Tribunal 1, Version B). Packed layouts referenced here are in each
subsystem's header; the field-by-field table is `DATA_REPRESENTATION.md`.

## 1. How the code plugs into the professor's base

* **Build.** `build_make/mtkernel_3/app_program/subdir.mk` compiles
  `app_program/*.c` and `app_program/*/*.c`. Every `.c` placed directly in
  `app_program/subsystemN/` is built automatically, so no build file was
  changed. Folders one level deeper (`docs/`, `host_test/`) are not
  compiled into the firmware.
* **Start-up.** `usermain()` (`app_main.c`, protected) calls every
  `subsystemN_init()` (kernel objects, pins, GPIO-callback registration) and
  then every `subsystemN_start()` (tasks, cyclic handlers). In the `TEST=1`
  image it then calls every `subsystemN_test()`.
* **Tasks** are created with `app_start_task()` (2 KB stack, `TA_RNG3`,
  core affinity dropped on `SMP=0`).

## 2. Task set (Week 6 task table; Tribunal 1: distinct priorities, fixed periods)

| Task | Buddy | Prio | Core | Activation | Budget checked | Blocks on |
|---|---|---|---|---|---|---|
| Motion Control | 2 | 12 | 1 | 20 ms cyclic + STOP / FAULT / CMD / EVENT flag bits | 10 ms (`s2_stat_motion`) | `flg_motion` only |
| IMU Monitoring | 4 | 13 | 1 | 10 ms cyclic | 5 ms (`s4_stat_imu`) | `flg_imu` (tick, I2C done; I2C wait ≤ 20 ms) |
| Line Following | 3 | 14 | 1 | 20 ms cyclic, phase +10 ms | 10 ms (`s3_stat_line`) | wake-up |
| Obstacle Scan | 5 | 15 | 1 | 60 ms ping cycle / scan request | 60 ms (`s5_stat_scan`) | echo flag ≤ 40 ms; request queue |
| Avoidance & Recovery | 5 | 16 | any | obstacle event | — | result flags, profile queue, all with timeouts |
| Barcode Decoding | 3 | 17 | 1 | IR edge wake-up; 20 ms poll while capturing | — | wake-up; motion result ≤ 15 s |
| Hump & Terrain | 4 | 18 | any | IMU sample queue | — | queue |
| Obstacle Profile | 5 | 19 | any | sample queue | — | queue |
| WiFi & MQTT Communication | 1 | 20 | 0 | 100 ms cyclic + inbound frame | — | flag; motion result ≤ 300 ms |
| Telemetry Collection | 1 | 21 | 0 | 500 ms cyclic | 50 ms (`s1_stat_tlm`) | wake-up |

* All ten are application tasks (12–21). The kernel's USB console (3) and
  CYW43 radio (4) tasks run above them on core 0. The control tasks run on
  core 1, away from the radio.
* There is no Vehicle Controller task. Mission coordination is the mode
  machine inside Motion Control (Week 6 §4; Tribunal 1 rejected a
  supervisor task).
* High tasks never block on network or console I/O. All kernel calls from
  interrupt handlers are ISR-safe: `tk_set_flg`, `tk_wup_tsk`, or
  `tk_snd_mbf` with `TMO_POL`.
* "Budget checked" is the value `task_stat_end()` counts overruns against.
  The test image prints period, rate and worst run time: S2-U42, S3-U32,
  S4-U30, S5-U21, S1-U22. **These are not yet measured on hardware.**

## 3. Kernel objects (limits in `config/config.h`)

| Object | Owner | Size | Producers → consumer |
|---|---|---|---|
| `flg_motion` (single waiter) | B2 | — | cyclic, `motion_stop()`, `sys_fault_set()` (critical), `sys_post_event()`, `motion_request()` → Motion Control |
| `flg_motion_done` (multi) | B2 | bit per source | Motion Control → command producers |
| `mbf_motion_cmd` | B2 | 16 × 4 B (128 B) | Barcode, Avoidance, Communication → Motion Control |
| `flg_line_evt` (multi) | B3 | 3 bits | Line Following → Avoidance |
| `flg_imu` (single) | B4 | 2 bits | cyclic, I2C ISR → IMU task |
| `mbf_imu_hump` | B4 | 32 × 4 B (256 B) | IMU → Hump |
| `flg_avoid`, `flg_us` | B5 | — | Scan → Avoidance; echo ISR → Scan |
| `mbf_scan_req` / `mbf_scan_samples` / `mbf_profile` | B5 | 4 × 4 / 64 × 4 / 4 × 8 B | Avoidance → Scan → Profile → Avoidance |
| `flg_comm` (single) | B1 | 2 bits | cyclic, bridge (inbound frame), Telemetry → Communication |
| `mbf_telemetry` / `mbf_mqtt_tx` / `mbf_mqtt_rx` | B1 | 4 × 65 / 8 × 65 / 4 × 64 B | Telemetry → Comm → MQTT client → Comm |
| cyclic handlers | B1 ×2, B2, B3, B4 | 500/100, 20, 20, 10 ms | — |

* **Totals:** 10 tasks (+1 test task in `TEST=1`) of 32; 7 event flags of 16;
  8 message buffers of 12 (one temporary extra in S1-U26); 5 cyclic handlers
  of 12.
* The kernel and libraries add 2 tasks (USB console, CYW43) and no message
  buffers. Message-buffer storage totals 2112 B from the kernel heap.

## 4. Hardware ownership (Week 6 pin table; one owner per resource)

| Resource | Owner (task) | Notes |
|---|---|---|
| GP8/GP9, GP10/GP11; PWM slices 4, 5 | B2 Motion Control | **only** writer of motor outputs (`drv_motor.c`) |
| GP0/GP1 (left enc.), GP7/GP28 (right enc.) | B2 encoder callbacks | GP0/GP1 are left to UART0 in `CRASH_REPORT=1` / `CONSOLE=uart` builds (README §3) |
| GP2, GP6, GP27 (IR L/C/R) | B3 Line Following (levels), B3 edge callback (GP27) | GP3 (IR_LEFT A0) wired but unused |
| GP4/GP5 (HC-SR04), GP15 + PWM slice 7B (servo) | B5 Obstacle Scan | |
| GP16/GP17, I2C0, IRQ 23 | B4 IMU Monitoring | own driver: kernel I2C driver disabled in config |
| IO_IRQ_BANK0 (IRQ 13) | B2 dispatcher, enabled on core 1 | one handler; callbacks of B2 (GP0, GP7), B3 (GP27), B5 (GP5); registered in `_init()` only, frozen after start |
| TIMER (µs counter) | read-only for all | `TIMERAWL`, as the professor's USB/CYW43 code does |
| TIMER alarm 3, IRQ 3 | B3 | only in the `S3_CAPTURE_POLL_1MS` variant |
| PWM block reset | first PWM owner releases it (idempotent) | never re-asserted |
| Radio, lwIP, MQTT client | kernel `cyw43_utk_task` | Buddy 1 hands frames over through `mqtt_bridge.h` (`TEAM.md` §3) |

## 5. Interface catalogue

| ID | Provider → consumer | Mechanism | Format | Rules |
|---|---|---|---|---|
| IF-M1 | B3 barcode, B5 avoidance, B1 comm → B2 | `motion_command()` → `mbf_motion_cmd` (`TMO_POL`) | 32-bit command word (`motion_msg.h`) | one producer task per source; queue full → `E_TMOUT` + `CMD_QUEUE` fault |
| IF-M2 | B3 line → B2 | `motion_set_steer()`, single-writer word | 16-bit steering word | newest wins; must change within 100 ms or Motion Control stops following; used **only in NORMAL** |
| IF-M3 | B2 → producers | result word per source + `flg_motion_done` bit | 13-bit result word | DONE / ABORTED / FAULT / REJECTED + reason; producers wait with a timeout (`motion_wait_result()`) |
| IF-M4 | anyone (ISR-safe) → B2 | `motion_stop()`: atomic OR + `MF_STOP` | — | bypasses the queue, flushes it, zero PWM in the next period |
| IF-M5 | B5 scan → B2 | `motion_set_speed_cap()`, single-writer word | 15-bit cap word | must keep changing (300 ms) or is ignored |
| IF-S1 | B3, B4, B5 → B2 | `sys_post_event()` (atomic OR + `MF_EVENT`) | `SYS_EV_*` bits | consumed every control period |
| IF-S2 | all ↔ all | `sys_fault_set/clear/get()` (atomic) | 27-bit fault mask | one owner per bit; a new critical bit wakes Motion Control at once |
| IF-S3 | B2 → all | `sys_vsw()` | 18-bit vehicle status word | single writer (Motion Control) |
| IF-S4 | B2 → B1, B3, B4, B5 | `motion_get_status()` (seqlock), `motion_get_odometry()` | 2-word motion status | |
| IF-L1 | B3 → B1, B5 | `s3_get_line_status()` (seqlock) | line word + timestamp | seq advances every 20 ms |
| IF-L2 | B3 → B5 | `flg_line_evt` | FOUND / LOST / JUNCTION | consumer clears what it waits for |
| IF-L3 | B3 → B1 | `s3_get_barcode_status()` | barcode word | |
| IF-I1/I2 | B4 → B1 | `s4_get_imu_status()` (seqlock), `s4_get_hump_status()` | 2 words + time word; 1 word | validity flag, invalid codes and timestamp |
| IF-O1..O3 | B5 → B1 | `s5_get_scan_status()`, `s5_get_avoid_status()`, `s5_get_profile()` | 1 + 1 + 2 words | |
| IF-C1 | all → B1 | Telemetry Collection reads IF-S3/S4/L1/L3/I1/I2/O1..O3 | `tlm_build.c` | read-only; never blocks |
| IF-C2 | B1 ↔ MQTT client | `mqtt_bridge.h` (`TMO_POL`) | frames of `MQTT_BINARY_PROTOCOL.md` | wiring into `mqtt_utk.c` is a protected-file change: `subsystem1/docs/MQTT_INTEGRATION_PATCH.md` |

## 6. Command arbitration (Tribunal 1 interface map; Week 6 §8.3)

**Priority:** Safe Stop/Fault > Obstacle Avoidance > Barcode Navigation >
Line Following. It is enforced by the vehicle mode, because each mode
admits exactly one movement source:

| Mode | LINE (steering word) | BARCODE (MOVE/TURN/STOP) | AVOID (MOVE/TURN/STOP) | SUPER (STOP / SET_SPEED / RESUME / CLEAR) |
|---|---|---|---|---|
| INIT | ignored | MODE | MODE | STOP, SET_SPEED, CLEAR |
| NORMAL | **followed** | accepted → NAV_ACTION | MODE | STOP, SET_SPEED, CLEAR |
| NAV_ACTION | ignored | accepted (queued behind the active one) | MODE | STOP, SET_SPEED, CLEAR |
| OBSTACLE | ignored | MODE | **accepted** | STOP, CLEAR (SET_SPEED → MODE) |
| SAFE_STOP | ignored, zero output | MODE | MODE | STOP, SET_SPEED, CLEAR, **RESUME** |

* **Validation first:** reserved bits and unknown types → FORMAT; a type
  not allowed for the source → SOURCE (e.g. remote MOVE, line commands);
  argument out of range → RANGE.
* **Stale commands:** every command carries the 3-bit mode epoch of when it
  was built. The epoch advances on entering INIT, OBSTACLE and SAFE_STOP,
  and a command from an older epoch is rejected STALE (supervisory requests
  excepted). Entering OBSTACLE or SAFE_STOP also aborts the running command
  and flushes the queue (result ABORTED; barcode commands also end
  NAV_ACTION).
* **Queue:** 1 running + 4 queued per controller (`MC_QUEUE_DEPTH`).
  Overflow → QUEUE. A same-source MOVE→TURN chain passes through creep speed
  (NFR12).
* **STOP:** a supervisory STOP is always accepted and latches `REMOTE_STOP`
  (Safe Stop until RESUME). `motion_stop()` is the queue bypass for local
  safety (avoidance emergency, timeouts).
* **Completion:** a result word per source plus an event-flag bit. Each
  producer waits with its own timeout (barcode 15 s; avoidance step time +
  3 s; comm 300 ms). On timeout it calls `motion_stop()` itself, so nobody
  waits forever.
* **Fault state:** while Motion Control is in FAULT, only STOP and
  SET_SPEED are accepted. MOVE/TURN return MODE.

## 7. Vehicle mode (system statechart, Week 6 §5; owner Buddy 2)

| From | Event / condition | To |
|---|---|---|
| any | critical fault bit set (`sys_fault_critical_mask()`) | SAFE_STOP |
| INIT | IR calibrated (B3) **and** IMU calibrated (B4) **and** armed | NORMAL |
| INIT | 8 s without both (armed) → `SYS_FLT_INIT` | SAFE_STOP |
| NORMAL | obstacle detected (B5) | OBSTACLE |
| NORMAL | barcode command started | NAV_ACTION |
| NAV_ACTION | obstacle detected | OBSTACLE (barcode command aborted) |
| NAV_ACTION | barcode command finished (any result) | NORMAL |
| OBSTACLE | avoidance done + line reacquired (B5) | NORMAL |
| OBSTACLE | recovery failed (B5) | SAFE_STOP |
| SAFE_STOP | validated RESUME, no critical condition left | INIT (recalibrate) |

The test image never arms the motors (`motion_api.c`), so on the bench it
stays in INIT or SAFE_STOP.

## 8. Faults and recovery

The full bit table is in `subsystem1/docs/MQTT_BINARY_PROTOCOL.md` §6.

* **Degraded:** reported only, the car continues (NFR9 network, line lost
  while Buddy 3 searches, invalid barcode or reading, queue overflow,
  single motor stall/encoder fault while it is being retried).
* **Critical condition:** Safe Stop while present (IMU silent, ultrasonic
  silent).
* **Critical event:** Safe Stop, latched until CLEAR/RESUME (escalated
  motion fault, watchdog, calibration failure, collision, avoidance or
  recovery failure, line search exhausted, init timeout, kernel-object
  failure, operator STOP).
* **Bounded loops:**
  * motion fault → 3 retries → `MOTION_ESCALATED`;
  * IMU bring-up / recovery ×3, calibration ×3;
  * line search: 2 sweep pairs;
  * avoidance: 1 re-plan, 1 search retry, 25 s overall;
  * Wi-Fi/MQTT back-off 1→16 s (never critical);
  * barcode capture: 450 mm window / 3 s stall.
* **Motor watchdog:** if the Motion Control loop misses 5 periods, its own
  cyclic handler switches the motors off and raises `MOTOR_WDOG`.

## 9. Shared-data safety (Change Requirement 1)

| Shared item | Writers | Mechanism |
|---|---|---|
| fault mask, event word, stop request | many tasks + ISRs, both cores | `app_atomic_or/clear/xchg` (SMP atomics; `DI`/`EI` on `SMP=0`) |
| vehicle status, steering, speed cap, result words, line/barcode/hump/scan/avoid words | exactly one task each | aligned 32-bit store with barriers (`app_store`/`app_load`) |
| motion status (2 words), line status + time, IMU status (2 words), obstacle profile (2 words), encoder records | one task or one ISR | sequence count (`APP_SEQ`); readers retry ≤ 8 times, never block the writer |
| barcode edge ring | ISR (head), barcode task (tail) | single-producer/single-consumer ring, overflow flag |
| everything else between tasks | — | kernel message buffers / event flags (ownership transfer) |

No lock is ever held across a kernel call, and no read-modify-write happens
on a word with more than one writer.
