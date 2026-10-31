# Test cases: rows for `testing/*/TEST_LOG.md`

`testing/` is outside the subsystem folders and was not edited. These rows
are ready to paste into the "Test cases" table of each log
(`testing/README.md`: "Define it first"). They replace the template's
*example* rows (S1-U01..U03, S2-U01..U02, S3-U01, S4-U01..U02, INT-01..02),
which described the example LED / producer / consumer / monitor code that
no longer exists and were never run.

* **Automated rows** (generated from the `TEST_CHECK` calls in the code, so
  ID and description match the console line exactly). Run them with
  `cd build_make && make TEST=1 -j8`, flash, save the console
  (`testing/README.md` §4). Rows marked *host + target* also run on a PC
  with no hardware: `make -C app_program/subsystem2/host_test`.
* **Manual rows** (section 7) need the car. They are proposals: confirm
  them in the team before use.
* Integration IDs start at INT-10 so they cannot clash with the example
  rows INT-01/INT-02 already in the shared integration log.

## 1. Subsystem 1 (`testing/subsystem1/TEST_LOG.md`)

| ID | What is tested | Method | Where | Expected |
|---|---|---|---|---|
| S1-U01 | CRC-16/CCITT-FALSE: catalogue check value 0x29B1; empty input 0xFFFF | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | LSB-first bit order (known bytes FD BC DA), signed fields, overflow/underflow flagged | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | heartbeat = golden bytes 12 2A 04 34 12 EC 3D 60 E5; decodes; reconnects saturate at 15 | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U03 PASS` |
| S1-U04 | command frame = golden bytes; decoded and validated (SET_SPEED 250, id 0x5A) | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U04 PASS` |
| S1-U05 | acknowledgement and event frames = golden bytes; both decode | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U05 PASS` |
| S1-U06 | telemetry: 279 bits -> 40-byte frame = golden frame; decodes field by field | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U06 PASS` |
| S1-U07 | telemetry round trip with every field at its maximum and at its minimum | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U07 PASS` |
| S1-U08 | rejected: bad CRC, truncated, version, unknown type, length byte, wrong type, schema size | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U08 PASS` |
| S1-U09 | command fields: reserved bits / codes 6-7 rejected; SET_SPEED 50..400 only; no stray args | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U09 PASS` |
| S1-U10 | decoder errors map to distinct acknowledgement codes | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U10 PASS` |
| S1-U11 | commands map to supervisory requests only; Buddy 2's refusal is acknowledged with its reason | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U11 PASS` |
| S1-U12 | every encoder refuses a buffer one byte too small (no overrun) | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U12 PASS` |
| S1-U13 | binary telemetry frame (40 B) at least 10x smaller than the JSON text form | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U13 PASS` |
| S1-U14 | no network stack in the image: DISABLED without faults; starts once configured | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U14 PASS` |
| S1-U15 | connect sequence WIFI -> MQTT -> SUBSCRIBE -> ONLINE | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U15 PASS` |
| S1-U16 | WiFi lost while online: WIFI_LOSS (degraded) and RECOVERING reported | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U16 PASS` |
| S1-U17 |  | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U17 PASS` |
| S1-U18 | automatic reconnection: ONLINE again, counted, back-off reset, faults cleared | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U18 PASS` |
| S1-U19 | broker lost with WiFi up: MQTT_LOSS only; each wait for the broker bounded to 10 s | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U19 PASS` |
| S1-U20 | communication status word: 17 bits, counters saturate at 15 | auto (host + target) | `subsystem1/s1_unit_tests.c` | `[TEST] S1-U20 PASS` |
| S1-U21 | message buffers, event flag and both cyclic handlers exist and run | auto (target only) | `subsystem1/subsystem1_test.c` | `[TEST] S1-U21 PASS` |
| S1-U22 | telemetry task: one frame per 500 ms, never over its 50 ms budget | auto (target only) | `subsystem1/subsystem1_test.c` | `[TEST] S1-U22 PASS` |
| S1-U23 | connection state machine running (DISABLED without NET=lwip MQTT=1, no false fault) | auto (target only) | `subsystem1/subsystem1_test.c` | `[TEST] S1-U23 PASS` |
| S1-U24 | MQTT hand-over: wrong topic and empty frame refused; PING answered with ACK OK | auto (target only) | `subsystem1/subsystem1_test.c` | `[TEST] S1-U24 PASS` |
| S1-U25 | SET_SPEED reaches Buddy 2 and is acknowledged; a corrupted frame is counted, flagged, never run | auto (target only) | `subsystem1/subsystem1_test.c` | `[TEST] S1-U25 PASS` |
| S1-U26 | bounded queue full: the OLDEST frame is dropped and counted (overflow policy) | auto (target only) | `subsystem1/subsystem1_test.c` | `[TEST] S1-U26 PASS` |

## 2. Subsystem 2 (`testing/subsystem2/TEST_LOG.md`)

| ID | What is tested | Method | Where | Expected |
|---|---|---|---|---|
| S2-U01 | command word round trip at field limits (arg -2048) | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U01 PASS` |
| S2-U01b | command word arg +2047 | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U01b PASS` |
| S2-U02 | reserved bits, unknown type and source rules rejected | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U02 PASS` |
| S2-U03 | argument ranges enforced at the boundaries | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U03 PASS` |
| S2-U04 | result, steering (clamped -63) and speed-cap words | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U04 PASS` |
| S2-U05 | motion status 2-word pack/unpack at limits | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U05 PASS` |
| S2-U06 | zero error: output = feed-forward | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U06 PASS` |
| S2-U07 | saturated output: integrator frozen (anti-windup) | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U07 PASS` |
| S2-U08 | turn arc for 90 deg = pi*track/4 (102.1 mm at 130 mm track) | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U08 PASS` |
| S2-U09 | MOVE 500 mm: DONE, within 25 mm, wheels within 15 mm | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U09 PASS` |
| S2-U09a | MOVE accepted in Obstacle Handling | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U09a PASS` |
| S2-U09b | bounded AVOID reverse accepted only in Obstacle Handling | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U09b PASS` |
| S2-U09c | negative MOVE drives both wheels backward and completes | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U09c PASS` |
| S2-U10 | TURN +90: left forward, right backward, arc within 12 mm | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U10 PASS` |
| S2-U11 | queued TURN starts straight from creep speed (no full stop) | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U11 PASS` |
| S2-U12 | AVOID command rejected outside Obstacle Handling | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U12 PASS` |
| S2-U13 | BARCODE command rejected in Obstacle Handling | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U13 PASS` |
| S2-U14 | command from an older mode epoch rejected as stale | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U14 PASS` |
| S2-U15 | nothing accepted in INIT or SAFE_STOP | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U15 PASS` |
| S2-U16 | bounded command queue: overflow rejected (QUEUE) | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U16 PASS` |
| S2-U17 | STOP bypass aborts the running command (ABORTED, nav finished) | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U17 PASS` |
| S2-U18 | stop(): PWM forced to zero in the very next period | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U18 PASS` |
| S2-U19 | stall (duty, no ticks for 400 ms): FAULT result and MOTOR_STALL | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U19 PASS` |
| S2-U20 | FAULT -> IDLE only after 300 ms clear; motors off meanwhile | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U20 PASS` |
| S2-U21 | 3rd consecutive stall raises MOTION_ESCALATED (bounded recovery) | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U21 PASS` |
| S2-U22 | one wheel without ticks while driven: motion FAULT raised | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U22 PASS` |
| S2-U23 | positive steer turns right (left wheel faster) | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U23 PASS` |
| S2-U24 | stale steering: the car stops instead of driving blind | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U24 PASS` |
| S2-U25 | obstacle speed cap limits line-following speed (FR8) | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U25 PASS` |
| S2-U26 | steering ignored outside Normal Navigation | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U26 PASS` |
| S2-U27 | SAFE_STOP: zero motor output whatever the inputs | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U27 PASS` |
| S2-U40 | stop for an obstacle: wheels coast to rest, no reverse duty (no runaway backwards) | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U40 PASS` |
| S2-U28 | INIT waits for both IR and IMU calibration | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U28 PASS` |
| S2-U29 | not armed (test image): stays in INIT | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U29 PASS` |
| S2-U30 | INIT -> NORMAL when calibrated and armed | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U30 PASS` |
| S2-U31 | barcode command -> NAVIGATION ACTION | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U31 PASS` |
| S2-U32 | obstacle during a barcode action -> OBSTACLE, new epoch | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U32 PASS` |
| S2-U33 | avoidance done + line found -> NORMAL | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U33 PASS` |
| S2-U34 | critical fault (collision) -> SAFE_STOP | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U34 PASS` |
| S2-U35 | SAFE_STOP holds; events cannot leave it | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U35 PASS` |
| S2-U36 | validated RESUME -> INIT with calibration required again | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U36 PASS` |
| S2-U37 | initialisation timeout -> SAFE_STOP with SYS_FLT_INIT | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U37 PASS` |
| S2-U38 | severity: network/line-lost/stall degraded; FR12 and escalation critical | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U38 PASS` |
| S2-U39 | condition faults clear themselves; event faults need CLEAR | auto (host + target) | `subsystem2/s2_unit_tests.c` | `[TEST] S2-U39 PASS` |
| S2-U40 | command queue, 20 ms cyclic handler and shared GPIO interrupt running; encoder records readable | auto (target only) | `subsystem2/subsystem2_test.c` | `[TEST] S2-U40 PASS` |
| S2-U41 | microsecond TIMER runs at 1 MHz against the kernel clock | auto (target only) | `subsystem2/subsystem2_test.c` | `[TEST] S2-U41 PASS` |
| S2-U42 | Motion Control runs every 20 ms and finishes within half its period | auto (target only) | `subsystem2/subsystem2_test.c` | `[TEST] S2-U42 PASS` |
| S2-U43 | test image disarmed: INIT or SAFE_STOP only; motor watchdog never tripped | auto (target only) | `subsystem2/subsystem2_test.c` | `[TEST] S2-U43 PASS` |
| S2-U44 | stop() bypass served by Motion Control within 100 ms (NFR6, software path) | auto (target only) | `subsystem2/subsystem2_test.c` | `[TEST] S2-U44 PASS` |
| S2-U45 | IF-M1/M3 on target: a BARCODE MOVE outside navigation is answered REJECTED (MODE) | auto (target only) | `subsystem2/subsystem2_test.c` | `[TEST] S2-U45 PASS` |
| S2-U46 | remote STOP: DONE, SAFE_STOP and REMOTE_STOP latched within 60 ms | auto (target only) | `subsystem2/subsystem2_test.c` | `[TEST] S2-U46 PASS` |
| S2-U47 | RESUME clears latched events; leaves SAFE_STOP only when no critical condition remains | auto (target only) | `subsystem2/subsystem2_test.c` | `[TEST] S2-U47 PASS` |

## 3. Subsystem 3 (`testing/subsystem3/TEST_LOG.md`)

| ID | What is tested | Method | Where | Expected |
|---|---|---|---|---|
| S3-U01 | Code 39 table: 44 unique patterns, 3 wide each; * A Z as measured | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U01 PASS` |
| S3-U02 | every supplied sample decodes (PDF A/Z, PPTX A/F/Z/W/X/Y, photo A) | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U02 PASS` |
| S3-U03 | barcode read in the opposite direction decodes (reversed flag) | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U03 PASS` |
| S3-U04 | print-scale independent (ratio classification) | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U04 PASS` |
| S3-U05 | invalid: element count not 10k-1 | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U05 PASS` |
| S3-U06 | invalid: no wide/narrow contrast | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U06 PASS` |
| S3-U07 | invalid: corrupted start symbol rejected (no command) | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U07 PASS` |
| S3-U08 | invalid: wide inter-character gap rejected | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U08 PASS` |
| S3-U09 | invalid: missing start/stop character | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U09 PASS` |
| S3-U10 | 15/15 reads at 100, 200 and 300 mm/s (+-15 % ripple, 5 % speed bias, 0.6 mm blur) | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U10 PASS` |
| S3-U11 | same barcode again within 600 mm: DUPLICATE (no second command) | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U11 PASS` |
| S3-U11a | first pass over Z: VALID | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U11a PASS` |
| S3-U12 | 60 mm black run rejected as not-a-barcode at once | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U12 PASS` |
| S3-U13 | no edge for 3 s while capturing: aborted (bounded) | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U13 PASS` |
| S3-U14 | capture longer than the 450 mm window: aborted | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U14 PASS` |
| S3-U15 | default mapping A=LEFT, F=RIGHT, Z=U-TURN, X=STRAIGHT | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U15 PASS` |
| S3-U16 | mapping override/reset works; '*' and nav > 3 rejected | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U16 PASS` |
| S3-U17 | LEFT/RIGHT/U-TURN/STRAIGHT -> encoder TURN/MOVE commands | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U17 PASS` |
| S3-U18 | calibration on the line: black = high | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U18 PASS` |
| S3-U19 | inverted sensor polarity is learnt, not assumed | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U19 PASS` |
| S3-U20 | ambiguous start position -> calibration failure | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U20 PASS` |
| S3-U21 | flickering sensor while still -> calibration failure | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U21 PASS` |
| S3-U22 | centred: straight at full cruise fraction | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U22 PASS` |
| S3-U23 | line drifting to IR_L: steer towards it | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U23 PASS` |
| S3-U24 | line under IR_L only: stop and pivot towards it | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U24 PASS` |
| S3-U25 | centre and IR_L white: arc towards IR_R (car's left) at once, then pivot; LOST | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U25 PASS` |
| S3-U26 | line reacquired -> FOUND; keeps turning gently towards IR_R for LF_REALIGN_MS, then straight | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U26 PASS` |
| S3-U27 | line never found: keeps pivoting towards IR_R; no RECOVERY_FAIL | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U27 PASS` |
| S3-U28 | all three sensors black for >= 8 mm: one JUNCTION event | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U28 PASS` |
| S3-U29 | black under the barcode sensor (IR_R) beside the line does not disturb steering | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U29 PASS` |
| S3-U52 | side pivot held through all-white until the centre has the line; then half speed, no kick back | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U52 PASS` |
| S3-U53 | black under IR_R alone is not the line: lost, turning towards IR_R, search not ended | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U53 PASS` |
| S3-U54 | lost-line turn, then IR_L has the line: FOUND, pivot towards IR_L | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U54 PASS` |
| S3-U55 | lost-line arc ends after LF_LOST_ARC_MM of travel, then pivot | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U55 PASS` |
| S3-U56 | turning on after a find, line drifts to IR_L: steer towards IR_L | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U56 PASS` |
| S3-U57 | short lost-line arc (< LF_REALIGN_MIN_MS) finds the line: straight on, no turn-on | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U57 PASS` |
| S3-U30 | line status word round trip at limits (steer -63) | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U30 PASS` |
| S3-U31 | barcode status word round trip at limits | auto (host + target) | `subsystem3/s3_unit_tests.c` | `[TEST] S3-U31 PASS` |
| S3-U32 | Line Following runs every 20 ms (50 per s) within half its period | auto (target only) | `subsystem3/subsystem3_test.c` | `[TEST] S3-U32 PASS` |
| S3-U33 | line status word republished every period with a fresh timestamp (IF-L1) | auto (target only) | `subsystem3/subsystem3_test.c` | `[TEST] S3-U33 PASS` |
| S3-U34 | IR calibration decided within 1 s of INIT (calibrated, or IR_CAL fault): never stalls | auto (target only) | `subsystem3/subsystem3_test.c` | `[TEST] S3-U34 PASS` |
| S3-U35 | barcode reader idle outside NORMAL, no edge overflow; GPIO callbacks frozen once running | auto (target only) | `subsystem3/subsystem3_test.c` | `[TEST] S3-U35 PASS` |

## 4. Subsystem 4 (`testing/subsystem4/TEST_LOG.md`)

| ID | What is tested | Method | Where | Expected |
|---|---|---|---|---|
| S4-U01 | integer square root exact (floor) incl. 32-bit maximum | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | sine within 0.002 at 0/30/90/-30/180/270 deg | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U02 PASS` |
| S4-U03 | atan2 within 0.3 deg (0, 45, 90, -135, 10 deg) | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U03 PASS` |
| S4-U04 | stationary calibration recovers gyro bias and level offsets | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U04 PASS` |
| S4-U05 | car moving during calibration: rejected (retry) | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U05 PASS` |
| S4-U06 | gravity not on the configured up axis: calibration failure | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U06 PASS` |
| S4-U07 | no samples: failure | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U07 PASS` |
| S4-U08 | static 10 deg pitch measured within 0.5 deg | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U08 PASS` |
| S4-U09 | short braking/bump spike: pitch error < 1 deg (filter) | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U09 PASS` |
| S4-U10 | tilt event only after 200 ms beyond 15 deg (no false tilt) | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U10 PASS` |
| S4-U11 | 2.5 g impact detected once (refractory), peak recorded | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U11 PASS` |
| S4-U12 | 0.9 g bump below the collision threshold | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U12 PASS` |
| S4-U13 | still for 300 ms: STATIONARY | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U13 PASS` |
| S4-U14 | +30 dps yaw: TURN_LEFT, turn rate reported | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U14 PASS` |
| S4-U15 | -30 dps yaw: TURN_RIGHT | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U15 PASS` |
| S4-U16 | vibration/acceleration: MOVING | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U16 PASS` |
| S4-U17 | frozen raw data -> invalid run -> IMU fault | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U17 PASS` |
| S4-U18 | saturated sample rejected, never used | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U18 PASS` |
| S4-U19 | 40 mm hump measured within +-4 mm (Tribunal 1 success criterion) | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U19 PASS` |
| S4-U20 | highest hump of the run tracked at full resolution | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U20 PASS` |
| S4-U21 | flat track with vibration: no hump | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U21 PASS` |
| S4-U22 | long ramp (> 1 m) is not reported as a hump | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U22 PASS` |
| S4-U23 | invalid sample mid-hump: measurement discarded | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U23 PASS` |
| S4-U24 | 6-bit hump code: 2 mm steps, saturation flag, 63 = none | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U24 PASS` |
| S4-U25 | quantisation error <= 1 mm over 0..100 mm | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U25 PASS` |
| S4-U26 | IMU status words round trip at limits and invalid codes | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U26 PASS` |
| S4-U27 | hump status word (count saturates at 15) | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U27 PASS` |
| S4-U28 | IMU->hump queue message: pitch clamped to 11-bit range | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U28 PASS` |
| S4-U29 | compaction rounds to nearest and saturates | auto (host + target) | `subsystem4/s4_unit_tests.c` | `[TEST] S4-U29 PASS` |
| S4-U30 | IMU Monitoring runs every 10 ms (100 per s) | auto (target only) | `subsystem4/subsystem4_test.c` | `[TEST] S4-U30 PASS` |
| S4-U31 | IMU snapshot honest and fresh: valid data only from a calibrated chip, else invalid codes + IMU fault | auto (target only) | `subsystem4/subsystem4_test.c` | `[TEST] S4-U31 PASS` |
| S4-U32 | standing car: no hump reported; IMU -> hump queue never overflowed | auto (target only) | `subsystem4/subsystem4_test.c` | `[TEST] S4-U32 PASS` |

## 5. Subsystem 5 (`testing/subsystem5/TEST_LOG.md`)

| ID | What is tested | Method | Where | Expected |
|---|---|---|---|---|
| S5-U01 | 10-bit distance code: 5 mm steps, 0..4 m, 1022/1023 reserved | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U01 PASS` |
| S5-U02 | scan sample message round trip at limits | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U02 PASS` |
| S5-U02b | scan sample positive limits | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U02b PASS` |
| S5-U03 | scan request message round trip | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U03 PASS` |
| S5-U04 | profile message: limits kept, 'unknown' clearance stays unknown | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U04 PASS` |
| S5-U05 | scan and avoidance status words round trip | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U05 PASS` |
| S5-U06 | width within 10 % and distance within 1 cm (4 boxes, coarse+fine) | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U06 PASS` |
| S5-U07 | obstacle left of the path: negative bearing, more room on the right | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U07 PASS` |
| S5-U08 | no echo anywhere: path clear (NO_OBSTACLE) | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U08 PASS` |
| S5-U09 | object beyond 700 mm is not an obstacle to avoid | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U09 PASS` |
| S5-U10 | wall across the whole scan: edges unknown (INSUFFICIENT) | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U10 PASS` |
| S5-U11 | identical range at every angle: servo suspect (INVALID) | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U11 PASS` |
| S5-U12 | too few valid samples: INVALID | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U12 PASS` |
| S5-U13 | wall beside the obstacle limits that side's clearance | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13 PASS` |
| S5-U13b | isolated 3-degree dropout remains one obstacle; open sides ignore deeper background reflections | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13b PASS` |
| S5-U13c | extreme-angle near echoes do not extend the obstacle beyond the symmetric front-plane band | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13c PASS` |
| S5-U13d | captured 49-point hardware fine scan gives a valid profile and open bypass sides | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13d PASS` |
| S5-U13e | hardware-calibrated beam correction removes false obstacle width without losing clearance | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U13e PASS` |
| S5-U14 | both sides free: the side with more clearance; leave, pass, return, search, align | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U14 PASS` |
| S5-U15 | lateral leg gives the configured 50 mm robot-to-obstacle side gap | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U15 PASS` |
| S5-U16 | more clearance on the left: bypass on the left | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U16 PASS` |
| S5-U16b | excessive lateral offset is rejected instead of driving off the course | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U16b PASS` |
| S5-U17 | obstacle closer than 200 mm: no plan until bounded reverse/rescan succeeds | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U17 PASS` |
| S5-U17b | known safe side and close front requests a bounded reverse to 220 mm | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U17b PASS` |
| S5-U17c | required reverse beyond 150 mm is refused | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U17c PASS` |
| S5-U18 | no known safe side: no plan (Safe Stop), never an invented bypass | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U18 PASS` |
| S5-U19 | no valid profile: no plan | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U19 PASS` |
| S5-U20 | every planned step is a valid AVOID motion command | auto (host + target) | `subsystem5/s5_unit_tests.c` | `[TEST] S5-U20 PASS` |
| S5-U21 | Obstacle Scan ranges every 60 ms (+ one 10 ms tick) while monitoring | auto (target only) | `subsystem5/subsystem5_test.c` | `[TEST] S5-U21 PASS` |
| S5-U22 | ranging result and distance code agree; a silent sensor raises US_NO_RESPONSE | auto (target only) | `subsystem5/subsystem5_test.c` | `[TEST] S5-U22 PASS` |
| S5-U23 | outside navigation: avoidance idle, servo centred, forward monitoring | auto (target only) | `subsystem5/subsystem5_test.c` | `[TEST] S5-U23 PASS` |

## 6. Integration (`testing/integration/TEST_LOG.md`)

| ID | What is tested | Method | Where | Expected |
|---|---|---|---|---|
| INT-10a | box 150 mm wide at 300 mm: packed profile -> plan; obstacle event -> OBSTACLE | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-10a PASS` |
| INT-10b | every planned step crosses the interface packed and completes; search stops on the line | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-10b PASS` |
| INT-10c | car centre kept >= 150 mm right of the box edge; back on the line, heading within 10 deg | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-10c PASS` |
| INT-10d | AVOID_DONE -> NORMAL: line steering drives the car again | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-10d PASS` |
| INT-11a | valid unmapped barcode decoded and reported without motion | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-11a PASS` |
| INT-11b | A/F/Z/X -> LEFT/RIGHT/U-TURN/STRAIGHT -> BARCODE commands -> NAV_ACTION -> DONE -> NORMAL | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-11b PASS` |
| INT-11c | barcode status word carries symbol, command and motion result | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-11c PASS` |
| INT-12a | obstacle during a barcode turn: OBSTACLE wins, turn ABORTED, motors off, line loss kept | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-12a PASS` |
| INT-12b | then: stale command, barcode command and speed change refused; Buddy 5's command accepted | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-12b PASS` |
| INT-13a | degraded faults from Buddies 1, 3, 5 are reported; the car keeps driving | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-13a PASS` |
| INT-13b | IMU fault -> SAFE_STOP in one cycle, motors held off; RESUME refused while it persists | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-13b PASS` |
| INT-13c | condition gone: RESUME -> INIT; both recalibrations required before NORMAL | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-13c PASS` |
| INT-13d | collision latched until CLEAR; CLEAR alone never leaves SAFE_STOP, RESUME does | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-13d PASS` |
| INT-13e | jammed wheels: 3 local motion faults, then escalation to SAFE_STOP (no endless cycling) | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-13e PASS` |
| INT-14a | during avoidance Buddy 3 reports the loss once and neither steers nor searches | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-14a PASS` |
| INT-14b | line under the sensors again: FOUND event; AVOID_DONE -> NORMAL | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-14b PASS` |
| INT-14c | line steering resumes driving only after the hand-back to NORMAL | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-14c PASS` |
| INT-14d | line never found: avoidance recovery gives up -> SAFE_STOP; Buddy 3's own search keeps turning left in NORMAL (no SAFE_STOP) | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-14d PASS` |
| INT-15a | Buddies 1-5 at their limits and invalid codes: all 48 fields reach the server unchanged | auto (host + target) | `subsystem1/s1_integration_tests.c` | `[TEST] INT-15a PASS` |
| INT-15b | engineering values arrive within their step: 2 deg, 10 mm/s, 1 deg, 5 mm, 10 mm | auto (host + target) | `subsystem1/s1_integration_tests.c` | `[TEST] INT-15b PASS` |
| INT-16a | 30 mm hump measured by Buddy 4 arrives at the server within +-4 mm | auto (host + target) | `subsystem1/s1_integration_tests.c` | `[TEST] INT-16a PASS` |
| INT-16b | after humps of 30, 20, 45 mm: count 3, highest 45 mm within +-4 mm | auto (host + target) | `subsystem1/s1_integration_tests.c` | `[TEST] INT-16b PASS` |
| INT-16c | one EVT_HUMP frame carries last, highest and count | auto (host + target) | `subsystem1/s1_integration_tests.c` | `[TEST] INT-16c PASS` |
| INT-16d | mode, barcode, obstacle and collision events decode to their fields; no change, no event | auto (host + target) | `subsystem1/s1_integration_tests.c` | `[TEST] INT-16d PASS` |
| INT-17a | WiFi lost for 9 s while driving: WIFI_LOSS + RECOVERING reported, not critical; car keeps going | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-17a PASS` |
| INT-17b | link back: automatic reconnection clears the network faults; driving never interrupted | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-17b PASS` |
| INT-18a | critical fault in the same cycle as AVOID_DONE: SAFE_STOP wins, manoeuvre ABORTED | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-18a PASS` |
| INT-18b | in SAFE_STOP no event leaves it and only supervisory requests are accepted | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-18b PASS` |
| INT-18c | remote STOP: zero output in the next period, held as SAFE_STOP until RESUME | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-18c PASS` |
| INT-19a |  | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-19a PASS` |
| INT-19b | barcode edge buffer full: capture rejected as INVALID (never a command), decoder idle | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-19b PASS` |
| INT-20a | MQTT SET_SPEED 300 -> validated -> supervisory request -> applied, ACK OK | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-20a PASS` |
| INT-20b | same request during avoidance: refused by Buddy 2, ACK carries the reason (MODE) | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-20b PASS` |
| INT-20c | out-of-range or corrupted MQTT commands are rejected by Buddy 1, never reach Buddy 2 | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-20c PASS` |
| INT-20d | remote STOP accepted in all 5 modes; RESUME refused outside SAFE_STOP | auto (host + target) | `subsystem2/s2_integration_tests.c` | `[TEST] INT-20d PASS` |

## 7. Manual tests on the car (proposed; none run yet)

| ID | What is tested | Requirement | Steps / how | Expected result |
|---|---|---|---|---|
| S1-U27 | Telemetry and heartbeat reach the broker and Node-RED | FR7, FR13 | Apply `subsystem1/docs/MQTT_INTEGRATION_PATCH.md`; `make NET=lwip MQTT=1`; Node-RED with `nodered_decoder.js` | `csc14/tlm` at 2 Hz, `csc14/hb` at 1 Hz; decoded values match the console |
| S1-U28 | Network loss during driving | A1, NFR9, NFR10 | Switch the access point off 30 s, then on, while the car follows the line | car keeps driving; WIFI_LOSS; back-off 1..16 s; ONLINE again; heartbeat RECOVERING |
| S1-U29 | Malformed commands from the server | NFR11 | Send frames with a bad CRC, wrong length, reserved bits, SET_SPEED 401 | no crash; ACK BAD_FIELD / BAD_ARG where the id is readable; `rx_rejected` counts |
| S2-U48 | Wheel and encoder geometry | A2, A3 | Turn each wheel 10 revolutions by hand; read the counts (console / telemetry odometer) | set `WHEEL_DIAMETER_MM`, `ENC_PULSES_PER_REV`, `TRACK_WIDTH_MM` |
| S2-U49 | Motor polarity, feed-forward, minimum controllable speed | T1-18, A2 | Step the duty slowly from 0; note when each wheel turns smoothly | set `MOTOR_*_INVERT`, `FF_OFFSET_PM`, `MIN_CTRL_SPEED_MMS` (creep) |
| S2-U50 | PID speed control | PB-2 | Step 100→200→300 mm/s; log measured speed | settles within 0.5 s, no sustained oscillation; record the gains |
| S2-U51 | Straight-line accuracy | NFR7 | MOVE 1000 mm × 5 | ≤ 15 mm lateral deviation |
| S2-U52 | Turn accuracy | NFR8 | TURN ±90°, 180° × 5 each | within ±10° |
| S2-U53 | stop() latency at the motor pins | NFR6 | Scope on a motor PWM pin and on a GPIO toggled at `motion_stop()` | PWM 0 within 100 ms |
| S2-U54 | Stall handling | T1-17, T1-19 | Hold one wheel, then both, while driving | FAULT within ~0.4 s, motors off; after 3 stalls SAFE_STOP |
| S2-U55 | Encoder pins | A3 | Check GP28 with the VBAT jumper; GP0/GP1 with the USB console | clean edges, counts match S2-U48 |
| S3-U36 | IR polarity and placement | FR1 | Car on the line in INIT | `line: IR calibrated` on the console |
| S3-U37 | Line following | FR1, NFR4 | Straight, curves, 3 speeds, varied lighting | stays on the line |
| S3-U38 | Barcode reading at speed | FR2, T1-33 | Drive over a known barcode 15 times each at low, medium, high speed | 15/15 correct at each speed, no duplicates |
| S3-U39 | Barcode vs curve | A4 | Curves and junctions without barcodes | no barcode reported |
| S3-U40 | Line loss and search | FR1 (6.3) | Lift the car off the line while driving; watch the `line: lost, turning left` / `line: found` console lines | with the centre and right sensors white the car arcs left at once while moving, then pivots left on the spot if the line is still missing, until one of them sees it; once found it keeps turning gently left briefly; the barcode (left) sensor never steers; found line ends centred under the centre sensor |
| S3-U41 | Edge load at top speed | R3, T1-20 | Log `ir_edge_count()` and `gpio_irq_count()` while crossing barcodes | no BARCODE_OVERFLOW |
| S4-U33 | IMU identity and axes | A5 | Read the console at boot; tilt nose-up; turn left | chip found; +1 g up; pitch +; yaw rate + |
| S4-U34 | IMU timing | R2, NFR13 | Test image S4-U30 with all tasks running | 100 runs/s |
| S4-U35 | Thresholds from logs | A5, NFR14 | Log stationary, driving, braking, bumping | no false tilt/collision; set `IMU_*` thresholds |
| S4-U36 | Known hump | FR5, FR6, T1-33 | Drive over a hump of known height 5 times | within ±4 mm; decide 2 mm or 4 mm steps (T1-02) |
| S5-U24 | Ultrasonic accuracy | A6 | Targets at 100..1500 mm | within ±10 mm, repeatable |
| S5-U25 | Servo calibration | A6 | Sweep −80..80°; check the angles | set `SERVO_CENTER_US`, `SERVO_US_PER_DEG_X10`, `SERVO_DIR` |
| S5-U26 | Obstacle profile | FR9 | Boxes of known width at 250..600 mm | width within 10 %; set `US_BEAM_COMP_DEG` |
| S5-U27 | Avoidance and recovery | FR8–FR12 | Obstacle on the line; also a wall (unavoidable) and an obstacle with no line behind it | bypass + reacquire + NORMAL; unavoidable / timeout → SAFE_STOP |
| INT-21 | Barcode → turn on the track | FR3 | After the team fills the navigation map | the commanded turn is executed, then line following resumes |
| INT-22 | Obstacle during line following | FR8–FR11 | Full avoidance on the track | OBSTACLE → bypass → line → NORMAL |
| INT-23 | Hump → dashboard | T1-33 | Known hump with MQTT running | dashboard value = Pico value |
| INT-24 | Shared GPIO interrupt at top speed | T1-20, R3 | Encoders + barcode + ultrasonic together | no missed counts, no overflow |
| SYS-01 | Full run | FR1–FR13 | Whole track: line, barcodes, hump, obstacle, telemetry | mission completed; telemetry consistent |
| SYS-02 | Safety | FR12, FR15, NFR6 | Remote STOP, unplug the ultrasonic, impact | SAFE_STOP each time; RESUME only when safe |
