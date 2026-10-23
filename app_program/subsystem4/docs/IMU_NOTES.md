# Buddy 4: IMU, terrain and hump notes

Code: `drv_i2c0.c`, `drv_imu.c` (hardware); `imu_proc.c`, `hump.c`,
`s4_status.c` (pure logic); `task_imu.c`, `task_hump.c` (tasks). Tests:
`s4_unit_tests.c` S4-U01..U29, on-target S4-U30..U32, integration INT-16.

## 1. The IMU chip is not known, and is not assumed

No supplied document names the chip. Week 6 lists an "IMU (6-DoF)" on
Grove 4, GP16/GP17. Tribunal 1, Buddy 4 says: "I still need to confirm which
sensors the IMU module provides". The professor's kernel I2C driver is
disabled (`config/config_device.h`: `DEVCNF_USE_IIC 0`, a protected file).
So:

* `drv_i2c0.c` is a register-level, interrupt-driven I2C0 master (400 kHz,
  IRQ 23). It pre-loads a whole register read into the 16-entry TX FIFO, and
  the task **sleeps** on its event flag until the transfer ends. Every wait
  is bounded (20 ms); a hung bus gets 9 SCL pulses + STOP.
* `drv_imu.c` **probes** `WHO_AM_I` for the two common 6-DoF families. It
  uses whichever register map answers, and no other:

| Family | Addresses | WHO_AM_I | Configuration written |
|---|---|---|---|
| InvenSense MPU-6050/6500/9250/9255, ICM-20600/20602 | 0x68, 0x69 | reg 0x75 ∈ {0x68, 0x70, 0x71, 0x73, 0x11, 0x12} | reset; PLL; 100 Hz; DLPF ≈ 44 Hz; ±500 dps; ±8 g |
| ST LSM6DS3 / DS3TR-C / DSL / DSO | 0x6A, 0x6B | reg 0x0F ∈ {0x69, 0x6A, 0x6C} | reset; BDU + auto-increment; 104 Hz; ±8 g; 500 dps |

If neither answers, `imu_probe()` fails. After 3 attempts, 1 s apart, the
task stays in FAULT with `SYS_FLT_IMU` set. It **never publishes data it did
not read** (on-target check S4-U31).

**Hardware validation required:**

* Identify the fitted module.
* Check its register values against its datasheet.
* Confirm the pull-ups on GP16/GP17.
* Set the axis mapping `IMU_MAP_*` / `IMU_SIGN_*` (`imu_proc.h`) to the
  mounting orientation.

## 2. Task and statechart (Week 6 §6.4, Tribunal 1)

* **IMU Monitoring:** priority 13, core 1, 10 ms cyclic. It is the only
  I2C0 user and never sends motor commands. The flow is INITIALISING →
  CALIBRATING → MONITORING, with FAULT for bounded recovery (3 tries).
  Calibration runs only while Buddy 2 keeps the motors off (vehicle mode
  INIT or SAFE_STOP; Tribunal 1: "Buddy 2 must keep the motors off during
  calibration"). It repeats on every new INIT epoch, i.e. after each RESUME.
* **Hump & Terrain Analysis:** priority 18, fed by a bounded 32-entry
  queue of 24-bit packed samples (pitch 0.1°, distance step 0.1 mm, valid
  bit, 4-bit sequence). A sequence gap (dropped sample) or an invalid sample
  discards the hump in progress rather than measuring across missing data
  (Tribunal 1: "consumers ignore invalid or stale samples").

## 3. Processing (`imu_proc.c`, integer only: no FPU, no libm)

| Function | Method | Thresholds `[CAL]` |
|---|---|---|
| Calibration (FR16) | 100 samples (1 s) of the still car. Gyro bias = mean; accel offset = mean − 1 g on the up axis | accept only if each gyro axis varies < 4 °/s and each accel axis < 80 mg, and the up axis reads 1 g ± 150 mg (else "not still" → retry up to 3×, or "orientation" → FAULT) |
| Attitude | Complementary filter, 98 % gyro / 2 % accelerometer | accelerometer used only while \|a\| is within 1 g ± 300 mg (bumps and braking: gyro only) |
| Tilt (FR17) | \|pitch\| or \|roll\| above threshold for a minimum time | 15° for 200 ms, 3° hysteresis |
| Turn rate (FR19) | Bias-corrected gyro Z (+ = left/CCW), integrated heading | turning above 15 °/s |
| Motion class (FR18) | STATIONARY / MOVING / TURN_LEFT / TURN_RIGHT | still: gyro < 3 °/s on all axes and horizontal accel < 50 mg for 300 ms |
| Collision (FR18) | Gravity-compensated horizontal acceleration magnitude | > 1.5 g; 500 ms refractory period; latched until CLEAR / RESUME |
| Data validity (FR15, NFR14) | Saturated samples, identical raw data 20 times (stuck sensor), I2C failures | 10 bad samples in a row (100 ms) → FAULT |

Computation stays at full resolution (mg, mdps, 0.01°). Only the published
values are compacted (`s4_status.h`): pitch/roll 8 bits at 1° (−128
invalid), yaw rate 9 bits at 1 °/s (−256 invalid), heading 9 bits (511
invalid), shock 5 bits at 0.5 g (Tribunal 1, Buddy 4: "full resolution
inside the task, compacted only when stored or sent").

**Collision response (team decision flagged).** Tribunal 1's readiness
table lists "collision response … not finalised". The default
`SYS_COLLISION_STOPS 1` follows the Week 6 system statechart: a collision
puts the vehicle in Safe Stop, latched until a supervisory CLEAR/RESUME
(INT-13d). Setting it to 0 makes collisions degraded (report only).

## 4. Hump height (FR5, FR6; Tribunal 1 decisions)

* **Method (Tribunal 1, Buddy 4):** height = Σ ds · sin(pitch), with ds the
  encoder distance step from Buddy 2 and pitch from the filter.
  Double-integrated acceleration was rejected by the team (drift).
* **Detection:** pitch ≥ 3° for 2 samples starts a climb (the elevation
  already gained is kept). A pitch ≤ −3° marks the descent. Level within
  1.5° for 3 samples completes it. Heights < 3 mm are rejected as noise, and
  anything longer than 1 m is a ramp, not a hump.
* **Representation (Tribunal 1 Version B change log):** stored/sent as
  0–100 mm in **2 mm steps → 6 bits** (codes 0..50, 63 = none, saturation
  flag above 100 mm). `HUMP_STEP_MM` may become 4 (5 bits) **after** the
  known-hump test, as the change log allows. Quantisation error ≤ 1 mm
  (S4-U25).
* **Highest hump** is tracked at full resolution (µm) inside the task and
  compacted only when published (Tribunal 1: "no change, full resolution").
* **Evidence (simulation):** a 40 mm hump is measured within ±4 mm
  (S4-U19, the Tribunal 1 success criterion). Over 3 humps the maximum is
  tracked (S4-U20). A flat track with vibration and a 5° ramp produce no
  hump (S4-U21/U22). An invalid sample discards the measurement (S4-U23). A
  30 mm hump arrives at the server within ±4 mm (INT-16a).

## 5. Remaining hardware validation (assumption A5 stays open)

1. Identify the IMU module; verify addresses, register map, scale factors
   and axis mapping (stationary: +1 g on the up axis; tilt nose-up:
   positive pitch; turn left: positive yaw rate).
2. Measure the I2C read time and the IMU task period/jitter with all tasks
   running. The test image prints runs per second (S4-U30); Tribunal 1
   risk R2.
3. Log stationary and driving data; set the tilt, collision and still
   thresholds from the logs (vibration, hard braking).
4. Drive over a hump of known height 5 times (Tribunal 1 test plan):
   estimate within ±4 mm, dashboard equal to what the Pico sent; then decide
   2 mm vs 4 mm steps.
