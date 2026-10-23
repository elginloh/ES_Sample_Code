# Proposed `TEAM.md` update (NOT applied: `TEAM.md` is a shared, protected file)

`TEAM.md` says shared files change "only by agreement of the team", and it
still lists the template's example subsystems. Below are ready-to-paste
replacements for its tables, matching the implemented firmware. Student IDs
are from the Week 6 design review. GitHub usernames are left for each member
to fill in.

## Section 1: Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | WiFi/MQTT communication, supervisory commands, telemetry (binary protocol v1) | 2501573 | | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motion control: motors, encoders, PID, turning; vehicle mode, fault mask, shared GPIO interrupt | 2502591 | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | IR line following, barcode capture and decoding, navigation commands | 2501720 | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU monitoring (tilt, turn rate, motion, collision) and hump/terrain analysis | 2500320 | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Ultrasonic + servo scanning, obstacle profiling, avoidance and line recovery | 2502075 | | `app_program/subsystem5/` | `testing/subsystem5/` |

## Section 2: Pins (Week 6 hardware table)

| Pin | Owner | Used for |
|---|---|---|
| GP0 | S2 | left encoder A (IRQ). **Not available in `CRASH_REPORT=1` / `CONSOLE=uart` builds**: the firmware then leaves GP0/GP1 to UART0 and the car cannot drive |
| GP1 | S2 | left encoder B (input; used only with quadrature) |
| GP2 | S3 | IR_LEFT D0 |
| GP3 | S3 | IR_LEFT A0 (wired through Grove, unused) |
| GP4 | S5 | HC-SR04 TRIG (output) |
| GP5 | S5 | HC-SR04 ECHO (input, both edges via the shared GPIO interrupt, pull-down) |
| GP6 | S3 | IR_CENTER D0 |
| GP7 | S2 | right encoder A (IRQ) |
| GP8, GP9 | S2 | left motor M1A/M1B (PWM slice 4 A/B) |
| GP10, GP11 | S2 | right motor M2A/M2B (PWM slice 5 A/B) |
| GP15 | S5 | SG90 servo (PWM slice 7 B), Robo Pico servo port 4 |
| GP16, GP17 | S4 | I2C0 SDA/SCL, IMU on Grove 4 |
| GP27 | S3 | IR_RIGHT D0: line level + barcode edge capture |
| GP28 | S2 | right encoder B (check the VBAT jumper, Week 6 A3) |

## Section 3: Peripherals

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| I2C0 (own driver `drv_i2c0.c`, IRQ 23; kernel I2C driver stays off) | S4 | `imu_task` | GP16, GP17 |
| PWM slices 4, 5 | S2 | `motion_task` | GP8–GP11 |
| PWM slice 7 | S5 | `scan_task` | GP15 |
| IO_IRQ_BANK0 (IRQ 13), one dispatcher | S2 (`drv_gpio_irq.c`); callbacks of S2, S3, S5 registered at init | taken on core 1 | GP0, GP7, GP27, GP5 |
| TIMER alarm 3 (IRQ 3) | S3, only in the `S3_CAPTURE_POLL_1MS` build | `barcode_task` (arms it) | GP27 |
| TIMER counter (read-only µs time base) | all | — | — |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` | GP23–25, GP29 |

**Shared-code change to agree (section 3, "WiFi and MQTT are special"):**
S1's frames reach the network only after the patch in
`app_program/subsystem1/docs/MQTT_INTEGRATION_PATCH.md` is applied to
`lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. Record it as interface
IF-C2 below.

## Section 4: Task priorities

**Change from the default bands, needs team agreement.** Week 6 orders tasks
by function (High / Medium / Low), and Tribunal 1 (CPU row) requires distinct
priorities for the five High tasks. The default per-subsystem bands
(S1 12–13 … S5 20–21) would put Buddy 1's network tasks above Motion
Control. The implemented assignment is:

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 1 | 20–21 | comm_task – 20 – core 0; telemetry_task – 21 – core 0 |
| 2 | 12 | motion_task – 12 – core 1 |
| 3 | 14, 17 | line_task – 14 – core 1; barcode_task – 17 – core 1 |
| 4 | 13, 18 | imu_task – 13 – core 1; hump_task – 18 – any |
| 5 | 15, 16, 19 | scan_task – 15 – core 1; avoid_task – 16 – any; profile_task – 19 – any |

Test image only: a temporary test task at priority 22 (`app_test.c`).

## Section 5: Interfaces between subsystems

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-M1 | S3, S5, S1 → S2 | message buffer (owned by S2) + event flag | `mbf_motion_cmd`, `flg_motion` | 32-bit command word, `motion_msg.h` | |
| IF-M2 | S3 → S2 | single-writer word | `motion_set_steer()` | 16-bit steering word | |
| IF-M3 | S2 → S1, S3, S5 | event flag (owned by S2) + result words | `flg_motion_done` | 13-bit result word per source | |
| IF-M4 | any → S2 | atomic word + event flag | `motion_stop()` | stop bypass | |
| IF-M5 | S5 → S2 | single-writer word | `motion_set_speed_cap()` | 15-bit cap word | |
| IF-S1 | S3, S4, S5 → S2 | atomic event word + flag | `sys_post_event()` | `SYS_EV_*` | |
| IF-S2 | all ↔ all | atomic bitmask | `sys_fault_set/clear/get()` | `SYS_FLT_*` (27 bits) | |
| IF-S3 | S2 → all | single-writer word | `sys_vsw()` | vehicle status word | |
| IF-L1, IF-L3 | S3 → S1, S5 | seqlock snapshot / word | `s3_get_line_status()`, `s3_get_barcode_status()` | `s3_status.h` | |
| IF-L2 | S3 → S5 | event flag (owned by S3) | `flg_line_evt` | FOUND / LOST / JUNCTION | |
| IF-I1, IF-I2 | S4 → S1 | seqlock snapshot / word | `s4_get_imu_status()`, `s4_get_hump_status()` | `s4_status.h` | |
| IF-O1..O3 | S5 → S1 | words / seqlock | `s5_get_scan_status()`, `s5_get_avoid_status()`, `s5_get_profile()` | `s5_msg.h` | |
| IF-C2 | S1 ↔ kernel MQTT client | message buffers (owned by S1), `TMO_POL` | `mbf_mqtt_tx`, `mbf_mqtt_rx` (`mqtt_bridge.h`) | binary frames, `MQTT_BINARY_PROTOCOL.md` | needs instructor agreement |

Integration tests: INT-10..INT-20 (automated), INT-21..INT-24 (manual);
see `TEST_CASES.md`.

## Section 6: Kernel object budget

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | 2 | 1 (+1 test task in `TEST=1`) | 2 | 2 | 3 |
| Semaphore | 16 | | | | | |
| Event flag | 16 | 1 | 2 | 1 | 1 | 2 |
| Mailbox | 12 | | | | | |
| Mutex | 12 | | | | | |
| Message buffer | 12 | 3 | 1 | | 1 | 3 |
| Fixed memory pool | 12 | | | | | |
| Variable memory pool | 12 | | | | | |
| Cyclic handler | 12 | 2 | 1 | 1 | 1 | |
| Alarm handler | 12 | | | | | |
