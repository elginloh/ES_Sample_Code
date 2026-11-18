# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-10a/b | Buddy 5 profile → plan → packed AVOID commands → Buddy 2; line search stops on the line | IF-M1, IF-M3, IF-S1 | auto, host + target | `subsystem2/s2_integration_tests.c` | `[TEST] INT-10a/b PASS` |
| INT-14a–d | Buddy 3 line loss/found events during Buddy 5 avoidance; hand-back to NORMAL or Safe Stop | IF-L1, IF-L2, IF-S1 | auto, host + target | `subsystem2/s2_integration_tests.c` | `[TEST] INT-14a-d PASS` |
| INT-18a–c | Critical safety stop overrides Buddy 5 completion and all motion commands | IF-M3, IF-S2 | auto, host + target | `subsystem2/s2_integration_tests.c` | `[TEST] INT-18a-c PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2025-11-18 | INT-15a | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-15b | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-16a | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-16b | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-16c | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-16d | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-16e | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-10a | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-10b | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-10c | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-10d | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-11a | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-11b | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-11c | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-12a | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-12b | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-13a | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-13b | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-13c | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-13d | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-13e | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-14a | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-14b | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-14c | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-14d | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-17a | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-17b | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-18a | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-18b | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-18c | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-19a | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-19b | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-20a | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-20b | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-20c | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| 2025-11-18 | INT-20d | not recorded | Vanessa | PASS | `logs/2025-11-18_intergation_test.txt` |  |
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
