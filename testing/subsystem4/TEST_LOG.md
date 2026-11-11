# Subsystem 4 — test log


**Subsystem:** MU monitoring and hump detection  ·
**Student ID:** 2500320  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Test definitions below mirror the current automated checks. Pico-only tests must be run on target.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | integer square root exact (floor) incl. 32-bit maximum |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | sine within 0.002 at 0/30/90/-30/180/270 deg |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U02 PASS` |
| S4-U03 | atan2 within 0.3 deg (0, 45, 90, -135, 10 deg) |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U03 PASS` |
| S4-U04 | stationary calibration recovers gyro bias and level offsets |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U04 PASS` |
| S4-U05 | car moving during calibration: rejected (retry) |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U05 PASS` |
| S4-U06 | gravity not on the configured up axis: calibration failure |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U06 PASS` |
| S4-U07 | no samples: failure |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U07 PASS` |
| S4-U08 | static 10 deg pitch measured within 0.5 deg |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U08 PASS` |
| S4-U09 | short braking/bump spike: pitch error < 1 deg (filter) |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U09 PASS` |
| S4-U10 | tilt event only after 200 ms beyond 15 deg (no false tilt) |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U10 PASS` |
| S4-U11 | 2.5 g impact detected once (refractory), peak recorded |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U11 PASS` |
| S4-U12 | 0.9 g bump below the collision threshold |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U12 PASS` |
| S4-U13 | still for 300 ms: STATIONARY |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U13 PASS` |
| S4-U14 | +30 dps yaw: TURN_LEFT, turn rate reported |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U14 PASS` |
| S4-U15 | -30 dps yaw: TURN_RIGHT |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U15 PASS` |
| S4-U16 | vibration/acceleration: MOVING |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U16 PASS` |
| S4-U17 | frozen raw data -> invalid run -> IMU fault |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U17 PASS` |
| S4-U18 | saturated sample rejected, never used |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U18 PASS` |
| S4-U19 | 40 mm hump measured within +-4 mm (Tribunal 1 success criterion) |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U19 PASS` |
| S4-U20 | highest hump of the run tracked at full resolution |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U20 PASS` |
| S4-U21 | flat track with vibration: no hump |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U21 PASS` |
| S4-U22 | long ramp (> 1 m) is not reported as a hump |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U22 PASS` |
| S4-U23 | invalid sample mid-hump: measurement discarded |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U23 PASS` |
| S4-U24 | 6-bit hump code: 2 mm steps, saturation flag, 63 = none |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U24 PASS` |
| S4-U25 | quantisation error <= 1 mm over 0..100 mm |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U25 PASS` |
| S4-U26 | IMU status words round trip at limits and invalid codes |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U26 PASS` |
| S4-U27 | hump status word (count saturates at 15) |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U27 PASS` |
| S4-U28 | IMU->hump queue message: pitch clamped to 11-bit range |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U28 PASS` |
| S4-U29 | compaction rounds to nearest and saturates |  | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U29 PASS` |
| S4-U30 | IMU Monitoring runs every 10 ms (100 per s) |  | auto (target only) | `subsystem4/subsystem4_test.c` | `[TEST] S4-U30 PASS` |
| S4-U31 | IMU snapshot honest and fresh: valid data only from a calibrated chip, else invalid codes + IMU fault |  | auto (target only) | `subsystem4/subsystem4_test.c` | `[TEST] S4-U31 PASS` |
| S4-U32 | standing car: no hump reported; IMU -> hump queue never overflowed |  | auto (target only) | `subsystem4/subsystem4_test.c` | `[TEST] S4-U32 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |