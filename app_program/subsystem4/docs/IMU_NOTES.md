# Buddy 4: IMU, terrain and hump notes

Code: `drv_i2c0.c`, `drv_imu.c` (hardware); `imu_proc.c`, `hump.c`,
`s4_status.c` (pure logic); `task_imu.c`, `task_hump.c` (tasks). Tests:
`s4_unit_tests.c` S4-U01..U29 and S4-U33..U42 (no-gyro path), on-target
S4-U30..U32, integration INT-16.

## 1. The IMU chip: LSM303DLHC (GY-511), no gyroscope

Week 6 lists an "IMU (6-DoF)" on Grove 4, GP16/GP17. The module the course
actually supplies is a **GY-511 board with an ST LSM303DLHC**: a 3-axis
accelerometer and a 3-axis magnetometer, **with no gyroscope**. Section 3a
describes how the processing works without one. Support for the two common
6-DoF families is kept, in case the module is ever swapped. The professor's
kernel I2C driver is disabled (`config/config_device.h`: `DEVCNF_USE_IIC 0`,
a protected file). So:

* `drv_i2c0.c` is a register-level, interrupt-driven I2C0 master (400 kHz,
  IRQ 23). It pre-loads a whole register read into the 16-entry TX FIFO, and
  the task **sleeps** on its event flag until the transfer ends. Every wait
  is bounded (20 ms); a hung bus gets 9 SCL pulses + STOP.
* `drv_imu.c` **probes** for three chip families. It uses whichever register
  map answers, and no other:

| Family | Addresses | Identified by | Configuration written |
|---|---|---|---|
| InvenSense MPU-6050/6500/9250/9255, ICM-20600/20602 | 0x68, 0x69 | reg 0x75 ∈ {0x68, 0x70, 0x71, 0x73, 0x11, 0x12} | reset; PLL; 100 Hz; DLPF ≈ 44 Hz; ±500 dps; ±8 g |
| ST LSM6DS3 / DS3TR-C / DSL / DSO | 0x6A, 0x6B | reg 0x0F ∈ {0x69, 0x6A, 0x6C} | reset; BDU + auto-increment; 104 Hz; ±8 g; 500 dps |
| **ST LSM303DLHC (GY-511)** | accel 0x19, mag 0x1E | mag regs 0x0A..0x0C = `H` `4` `3` (the accelerometer has no WHO_AM_I) | accel: reboot; 100 Hz; BDU; ±8 g high-resolution (4 mg/LSB). Mag: 220 Hz; ±1.9 gauss; continuous |

LSM303DLHC quirks handled in `drv_imu.c`: accelerometer data is 12-bit and
left-justified, and a multi-byte read needs bit 7 of the register address
set. Magnetometer data is big-endian in the order **X, Z, Y**, and −4096
means overflow.

If none answers, `imu_probe()` fails. After 3 attempts, 1 s apart, the
task stays in FAULT with `SYS_FLT_IMU` set. It **never publishes data it did
not read** (on-target check S4-U31). At boot the console prints
`imu: ST LSM303DLHC (accel + mag, no gyro) at 0x19 (id 0x48)`.

**Hardware validation required:**

* Check the register values in `drv_imu.c` against the LSM303DLHC datasheet.
* Confirm the pull-ups on GP16/GP17 (the GY-511 has its own).
* Set the axis mapping `IMU_MAP_*` / `IMU_SIGN_*` (`imu_proc.h`) to the
  mounting orientation. The same map is applied to the accelerometer and
  the magnetometer, whose axes are aligned on the LSM303DLHC.
* Measure the magnetometer hard-iron offsets `IMU_MAG_OFF_X/Y/Z`
  (section 3a).

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

### 3a. Without a gyroscope (LSM303DLHC)

The table above is the gyro path. For a chip without a gyro (`has_gyro` is
FALSE), `imu_proc_step()` changes three rows. Everything else stays the same:
calibration (the gyro checks pass trivially), tilt, collision, data validity
and the hump measurement.

| Function | Method without a gyro | Settings `[CAL]` |
|---|---|---|
| Attitude | Accelerometer angles → median of the last 3 (removes single-sample bump spikes) → low-pass filter. Held while \|a\| is outside 1 g ± 300 mg | `IMU_ACC_LPF_PCT` 20 (≈ 45 ms) |
| Heading, turn rate (FR19) | Magnetometer field, hard-iron offsets removed, levelled with the median pitch/roll, heading = atan2(right, forward) relative to the direction at calibration (+ = left/CCW). Yaw rate = change of heading over the last 200 ms. Heading and yaw rate are published as invalid while the magnetometer overflows; the IMU itself stays valid | `IMU_MAG_RATE_WIN` 20, `IMU_MAG_OFF_X/Y/Z` |
| Motion class "still" | Magnetometer yaw rate < 8 °/s instead of gyro < 3 °/s, and horizontal accel < 50 mg for 300 ms | `IMU_STILL_MAG_MDPS` 8000 |

Known limits, to check on the car:

* **Steady acceleration reads as pitch.** The accelerometer can't tell tilt
  from speeding up: 0.05 g (about 0.5 m/s²) looks like 3° nose-up. Short
  speed changes produce humps below the 3 mm noise floor, but log the pitch
  while the car accelerates and brakes on a flat track.
* **The motors disturb the magnetometer.** Their fixed magnets are mostly
  removed by `IMU_MAG_OFF_X/Y/Z`. To measure those offsets, build a `TEST=1`
  image (the motors stay off) with `S4_MAG_LOG 1` in `task_imu.c`. Turn the
  car slowly through 360° by hand, then copy the last "offset estimate"
  printed into `imu_proc.h`. Current through the motors can still move the
  heading, so mount the GY-511 as far from the motors as you can.
* **Heading is magnetic, not integrated.** It doesn't drift, but steel in
  the floor or table under the track bends it.

Evidence (simulation, `s4_unit_tests.c`): a 10° pitch is read within 0.5°
(S4-U33), and a single bump spike moves it < 1° (S4-U34). A 30 °/s turn
reads within 3 °/s and the heading within 2° (S4-U37/U38). Climbing 10° in
100 ms moves the heading < 2° and causes no false turn (S4-U39). A
magnetometer overflow invalidates only the heading (S4-U40). A 40 mm hump
fed through the no-gyro filter is measured within ±4 mm (S4-U42).

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

1. Confirm the console reports the LSM303DLHC; verify the register map,
   scale factors and axis mapping (stationary: +1 g on the up axis; tilt
   nose-up: positive pitch; turn left: positive yaw rate). If a left turn
   gives a negative yaw rate, the magnetometer axes don't match the
   accelerometer's; check against the datasheet before flipping a sign.
2. Measure the magnetometer hard-iron offsets (section 3a).
3. Measure the I2C read time (two reads per sample on the LSM303DLHC) and the IMU task period/jitter with all tasks
   running. The test image prints runs per second (S4-U30); Tribunal 1
   risk R2.
4. Log stationary and driving data; set the tilt, collision and still
   thresholds from the logs (vibration, hard braking, and the pitch error
   while accelerating, section 3a).
5. Drive over a hump of known height 5 times (Tribunal 1 test plan):
   estimate within ±4 mm, dashboard equal to what the Pico sent; then decide
   2 mm vs 4 mm steps.
