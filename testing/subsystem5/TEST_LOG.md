# Subsystem 5 — test log

**Subsystem:** Obstacle scanning, avoidance and line recovery  ·
**Student ID:** 2502075  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01..U05 | Packed distance, request, sample, profile and status formats round-trip | internal task interfaces | auto | `s5_unit_tests.c` | all checks PASS |
| S5-U06..U13e | Obstacle geometry, width, clearance, scan-dropout/extreme-angle/beam filtering, captured hardware scan and invalid-data handling | FR8–FR10 | auto, synthetic/captured scans | `s5_unit_tests.c` | all checks PASS |
| S5-U14..U20 (including U17b/U17c) | Side choice, bounded reverse/manoeuvre and refusal of unknown/unsafe paths | FR10–FR12 | auto | `s5_unit_tests.c` | all checks PASS |
| S5-U21 | Scan task runs at the configured monitoring rate | task timing | auto, Pico | `subsystem5_test.c` | 15–21 samples in 1.2 s |
| S5-U22 | Ultrasonic status, distance code and fault agree | sensor safety | auto, Pico | `subsystem5_test.c` | PASS with sensor present or absent |
| S5-U23 | Idle state keeps servo centred and forward monitoring active | startup integration | auto, Pico | `subsystem5_test.c` | PASS |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
