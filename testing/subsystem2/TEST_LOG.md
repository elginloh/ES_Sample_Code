# Subsystem 2 — test log

**Subsystem:** Motion control, motor arbitration and vehicle mode  ·  **Student ID:** 2502591  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for the project test and result-recording rules.

## Test cases

Define each test before running it. Test IDs are permanent. These automated checks are implemented in `app_program/subsystem2/s2_unit_tests.c` and `subsystem2_test.c`.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01..U05 (including U01b) | Motion command validation and packing, result/steering/speed-cap words, motion status round-trip | IF-M1..M3 | auto, host + Pico | `s2_unit_tests.c` | each `[TEST]` check PASS |
| S2-U06..U08 | PID feed-forward, anti-windup and turn geometry | motion control | auto, host + Pico | `s2_unit_tests.c` | each `[TEST]` check PASS |
| S2-U09..U11 (including U09a..c and U10b..j) | Simulated MOVE/turn execution, bounded reverse, barcode turn/line confirmation, queued-turn creep | IF-M1..M3, NFR12 | auto, simulated plant, host + Pico | `s2_unit_tests.c` | each `[TEST]` check PASS |
| S2-U12..U18 | Mode/epoch arbitration, queue limit, STOP pre-emption and immediate zero PWM | IF-M1, IF-M4 | auto, host + Pico | `s2_unit_tests.c` | each `[TEST]` check PASS |
| S2-U19..U27 (including U40) | Stall detection/recovery/escalation, steering freshness and direction, speed cap, safe-stop output | NFR6, FR8, IF-M2/IF-M5 | auto, simulated plant, host + Pico | `s2_unit_tests.c` | each `[TEST]` check PASS |
| S2-U28..U39 | Vehicle mode transitions, calibration/arming, safe stop, resume and fault policy | vehicle mode, IF-S1/IF-S2 | auto, host + Pico | `s2_unit_tests.c` | each `[TEST]` check PASS |
| S2-U40..U47 | Live Pico objects/interrupts, timer and motion-task timing, disarmed safety, stop latency, command/result interface, STOP/RESUME | IF-M1/IF-M3/IF-M4, NFR6 | auto, Pico only | `subsystem2_test.c` | each `[TEST]` check PASS |

**ID issue to resolve before recording U40:** `s2_unit_tests.c` and `subsystem2_test.c` both currently use `S2-U40` for different checks. Treat the U40 result as ambiguous until one code-side test ID is changed and that change is committed. The groupings above retain the current source IDs for traceability.

The target image also runs Subsystem 2 integration checks (`s2_integration_tests.c`). Record those under their `INT-*` IDs in `testing/integration/TEST_LOG.md`, as integration results are shared across subsystem owners.

## How to run for accurate results

1. Commit the exact source you intend to test; record its short hash with the results. Use the same commit for the host and Pico runs.
2. Run the host simulation suite first. From the repository root, with GNU Make and a C compiler available:

   ```sh
   make -C app_program/subsystem2/host_test clean
   make -C app_program/subsystem2/host_test SUITES=s2
   ```

   Require a successful exit and PASS output for the S2 checks. This uses a simulated wheel plant; it does not verify physical motor/encoder behavior or the Pico scheduler.
3. Build the on-target test image from the firmware build directory:

   ```sh
   cd build_make
   make TEST=1 -j8
   ```

   Flash `mtk3pico_smp1_usb_cdc_wifi_test.uf2` to the Pico. The image runs the automated subsystem tests at startup and keeps the motors disarmed.
4. Capture the complete serial console from before resetting the board through the final `[TEST] subsystem2 TOTAL ...` line. Save it as `testing/subsystem2/logs/YYYY-MM-DD_subsystem2_test.txt`. Use the correct serial port and baud/configuration for this firmware; on Windows PuTTY, enable **Session → Logging → All session output**. Start capture before reset so startup checks are not missed.
5. Check that every expected `[TEST] S2-U.. PASS` line appears and there are no FAIL lines. Also inspect the total: other subsystems' tests may run in the same image, so the global total alone does not identify Subsystem 2 failures. Add a result row per executed test ID and link the saved log. Do not record a PASS for a test that did not emit a PASS line.
6. For repeatable hardware timing, use the same board, power source, firmware commit and test setup; allow a clean reset between runs. Pico-only timing results are measurements of that run, not proof of real-world motor accuracy.

`make TEST=1` runs the firmware's full test image, including other subsystems. The host target `SUITES=s2` selects Subsystem 2 unit/integration logic in the host harness; host-only results do not replace the Pico-only S2-U40..U47 checks. Manual car calibration and physical motor accuracy tests are not automated by these suites.

## Results

Add one row for every test result and every run. Never edit or delete old rows. For grouped checks, add rows using the individual emitted test IDs.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S2-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_subsystem2_test.txt` | |
