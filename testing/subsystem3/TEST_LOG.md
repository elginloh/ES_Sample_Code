# Subsystem 3 — test log


**Subsystem:** Line following and barcode navigation  ·
**Student ID:** 2501720  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Test definitions below mirror the current automated checks. Pico-only tests must be run on target.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | Code 39 table: 44 unique patterns, 3 wide each; * A Z as measured |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U01 PASS` |
| S3-U02 | every supplied sample decodes (PDF A/Z, PPTX A/F/Z/W/X/Y, photo A) |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U02 PASS` |
| S3-U03 | barcode read in the opposite direction decodes (reversed flag) |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U03 PASS` |
| S3-U04 | print-scale independent (ratio classification) |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U04 PASS` |
| S3-U05 | invalid: element count not 10k-1 |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U05 PASS` |
| S3-U06 | invalid: no wide/narrow contrast |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U06 PASS` |
| S3-U07 | invalid: corrupted start symbol rejected (no command) |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U07 PASS` |
| S3-U08 | invalid: wide inter-character gap rejected |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U08 PASS` |
| S3-U09 | invalid: missing start/stop character |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U09 PASS` |
| S3-U10 | 15/15 reads at 100, 200 and 300 mm/s (+-15 % ripple, 5 % speed bias, 0.6 mm blur) |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U10 PASS` |
| S3-U11 | same barcode again within 600 mm: DUPLICATE (no second command) |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U11 PASS` |
| S3-U11a | first pass over Z: VALID |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U11a PASS` |
| S3-U12 | 60 mm black run rejected as not-a-barcode at once |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U12 PASS` |
| S3-U13 | no edge for 3 s while capturing: aborted (bounded) |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U13 PASS` |
| S3-U14 | capture longer than the 450 mm window: aborted |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U14 PASS` |
| S3-U15 | default mapping A=LEFT, F=RIGHT, Z=U-TURN, X=STRAIGHT |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U15 PASS` |
| S3-U16 | mapping override/reset works; '*' and nav > 3 rejected |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U16 PASS` |
| S3-U17 | LEFT/RIGHT/U-TURN/STRAIGHT -> encoder TURN/MOVE commands |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U17 PASS` |
| S3-U18 | calibration on the line: black = high |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U18 PASS` |
| S3-U19 | inverted sensor polarity is learnt, not assumed |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U19 PASS` |
| S3-U20 | ambiguous start position -> calibration failure |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U20 PASS` |
| S3-U21 | flickering sensor while still -> calibration failure |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U21 PASS` |
| S3-U22 | centred: straight at full cruise fraction |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U22 PASS` |
| S3-U23 | line drifting to IR_L: steer towards it |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U23 PASS` |
| S3-U24 | line under IR_L only: stop and pivot towards it |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U24 PASS` |
| S3-U25 | centre and IR_L white: arc towards IR_R (car's left) at once, then pivot; LOST |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U25 PASS` |
| S3-U26 | line reacquired -> FOUND; keeps turning gently towards IR_R for LF_REALIGN_MS, then straight |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U26 PASS` |
| S3-U27 | line never found: keeps pivoting towards IR_R; no RECOVERY_FAIL |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U27 PASS` |
| S3-U28 | all three sensors black for >= 8 mm: one JUNCTION event |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U28 PASS` |
| S3-U29 | black under the barcode sensor (IR_R) beside the line does not disturb steering |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U29 PASS` |
| S3-U52 | side pivot held through all-white until the centre has the line; then half speed, no kick back |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U52 PASS` |
| S3-U53 | black under IR_R alone is not the line: lost, turning towards IR_R, search not ended |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U53 PASS` |
| S3-U54 | lost-line turn, then IR_L has the line: FOUND, pivot towards IR_L |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U54 PASS` |
| S3-U55 | lost-line arc ends after LF_LOST_ARC_MM of travel, then pivot |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U55 PASS` |
| S3-U56 | turning on after a find, line drifts to IR_L: steer towards IR_L |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U56 PASS` |
| S3-U57 | short lost-line arc (< LF_REALIGN_MIN_MS) finds the line: straight on, no turn-on |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U57 PASS` |
| S3-U30 | line status word round trip at limits (steer -63) |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U30 PASS` |
| S3-U31 | barcode status word round trip at limits |  | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U31 PASS` |
| S3-U32 | Line Following runs every 20 ms (50 per s) within half its period |  | auto (target only) | `subsystem3/subsystem3_test.c` | `[TEST] S3-U32 PASS` |
| S3-U33 | line status word republished every period with a fresh timestamp (IF-L1) |  | auto (target only) | `subsystem3/subsystem3_test.c` | `[TEST] S3-U33 PASS` |
| S3-U34 | IR calibration decided within 1 s of INIT (calibrated, or IR_CAL fault): never stalls |  | auto (target only) | `subsystem3/subsystem3_test.c` | `[TEST] S3-U34 PASS` |
| S3-U35 | barcode reader idle outside NORMAL, no edge overflow; GPIO callbacks frozen once running |  | auto (target only) | `subsystem3/subsystem3_test.c` | `[TEST] S3-U35 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
