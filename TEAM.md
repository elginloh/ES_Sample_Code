# Team and subsystems

The tables below describe the integrated five-Buddy firmware in
`app_program/`. Keep them synchronized with pin, task and interface changes.

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | WiFi/MQTT communication, supervisory commands and telemetry | 2501573 | | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motion control, motors, encoders, PID, vehicle mode and fault handling | 2502591 | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | IR line following, barcode decoding and navigation commands | 2501720 | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU, collision and hump/terrain monitoring | 2500320 | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Ultrasonic/servo scanning, obstacle profiling, avoidance and line recovery | 2502075 | | `app_program/subsystem5/` | `testing/subsystem5/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | S2 | left encoder A; unavailable with `CRASH_REPORT=1` / `CONSOLE=uart` |
| GP1 | S2 | left encoder B; unavailable with `CRASH_REPORT=1` / `CONSOLE=uart` |
| GP2 | S3 | IR left digital output |
| GP3 | S3 | IR left analogue connection (currently unused) |
| GP4 | S5 | HC-SR04 trigger |
| GP5 | S5 | HC-SR04 echo, shared GPIO interrupt |
| GP6 | S3 | IR centre digital output |
| GP7 | S2 | right encoder A |
| GP8 | S2 | left motor M1A, PWM slice 4A |
| GP9 | S2 | left motor M1B, PWM slice 4B |
| GP10 | S2 | right motor M2A, PWM slice 5A |
| GP11 | S2 | right motor M2B, PWM slice 5B |
| GP12 | | |
| GP13 | | |
| GP14 | | |
| GP15 | S5 | SG90 scan servo, PWM slice 7B |
| GP16 | S4 | I2C0 SDA for IMU |
| GP17 | S4 | I2C0 SCL for IMU |
| GP18 | | |
| GP19 | | |
| GP20 | S2 | Safe Stop restart button, active low |
| GP21 | | |
| GP22 | | |
| GP26 | | (ADC0 capable) |
| GP27 | S3 | IR right digital output and barcode edge capture |
| GP28 | S2 | right encoder B; check the VBAT jumper |

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | | | |
| I2C0 (own driver, IRQ 23) | S4 | `imu_task` | GP16, GP17 |
| I2C1 | | | |
| SPI0 | | | |
| SPI1 | | | |
| ADC | | | GP26 |
| PWM slices 4 and 5 | S2 | `motion_task` | GP8–GP11 |
| PWM slice 7 | S5 | `scan_task` | GP15 |
| IO_IRQ_BANK0 (shared dispatcher) | S2; S2/S3/S5 register callbacks | core 1 interrupt | GP0, GP5, GP7, GP27 |
| TIMER counter (read-only microsecond clock) | all | — | — |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. It publishes a heartbeat
  today. A "cloud / reporting" subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it.
  Because this changes shared library code, **agree it with the team and
  the instructor first**, and record it as an interface in section 5.
  It needs the `NET=lwip MQTT=1` build (README section 3).

## 4. Task priorities

1 is the highest. Priorities 1–4 are used by the kernel. Application tasks
are ordered by control criticality rather than by subsystem number.

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 1 | 20–21 | `comm_task` – 20 – core 0; `telemetry_task` – 21 – core 0 |
| 2 | 12 | `motion_task` – 12 – core 1 |
| 3 | 14, 17 | `line_task` – 14 – core 1; `barcode_task` – 17 – core 1 |
| 4 | 13, 18 | `imu_task` – 13 – core 1; `hump_task` – 18 – any |
| 5 | 15, 16, 19 | `scan_task` – 15 – core 1; `avoid_task` – 16 – any; `profile_task` – 19 – any |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-M1 | S3, S5, S1 → S2 | message buffer + event flag | `mbf_motion_cmd`, `flg_motion` | packed motion command (`motion_msg.h`) | |
| IF-M2 | S3 → S2 | single-writer atomic word | `motion_set_steer()` | steering and speed fraction | |
| IF-M3 | S2 → S1, S3, S5 | event flag + result words | `flg_motion_done` | completion result for each command source | |
| IF-M4 | any → S2 | atomic request + event flag | `motion_stop()` | immediate stop bypass | |
| IF-M5 | S5 → S2 | single-writer atomic word | `motion_set_speed_cap()` | obstacle-distance speed cap | |
| IF-S1 | S3, S4, S5 → S2 | atomic event word + flag | `sys_post_event()` | vehicle events (`SYS_EV_*`) | |
| IF-S2 | all ↔ S2 | atomic fault bitmask | `sys_fault_set/clear/get()` | system faults (`SYS_FLT_*`) | |
| IF-S3 | S2 → all | single-writer atomic word | `sys_vsw()` | vehicle mode/status word | |
| IF-L1/L3 | S3 → S1, S5 | snapshot getters | `s3_get_line_status()`, `s3_get_barcode_status()` | line and barcode status | |
| IF-L2 | S3 → S5 | event flag | `flg_line_evt` | line FOUND/LOST/JUNCTION | |
| IF-I1/I2 | S4 → S1 | snapshot getters | `s4_get_imu_status()`, `s4_get_hump_status()` | IMU and terrain status | |
| IF-O1/O2/O3 | S5 → S1 | snapshot getters | `s5_get_scan_status()`, `s5_get_avoid_status()`, `s5_get_profile()` | obstacle telemetry | |
| IF-C1 | S1 → S2 | validated supervisory motion commands | `motion_command()` | stop, speed, clear and resume requests | |
| IF-C2 | S1 ↔ MQTT service | bounded message buffers | `mbf_mqtt_tx`, `mbf_mqtt_rx` | binary MQTT protocol frames | |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The numbers below are the current firmware's application usage.

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | 2 | 1 | 2 | 2 | 3 |
| Semaphore | 16 | | | | | |
| Event flag | 16 | 1 | 2 | 1 | 1 | 2 |
| Mailbox | 12 | | | | | |
| Mutex | 12 | | | | | |
| Message buffer | 12 | 3 | 1 | | 1 | 3 |
| Fixed memory pool | 12 | | | | | |
| Variable memory pool | 12 | | | | | |
| Cyclic handler | 12 | 2 | 1 | 1 | 1 | |
| Alarm handler | 12 | | | | | |

(The kernel's USB and WiFi service tasks also use a few of these.)
