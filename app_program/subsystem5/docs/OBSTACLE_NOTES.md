# Buddy 5: scanning, profiling, avoidance and recovery notes

Code: `drv_ultrasonic.c`, `drv_servo.c` (hardware); `s5_msg.c`,
`obstacle_profile.c`, `avoid_plan.c` (pure logic); `task_scan.c`,
`task_profile.c`, `task_avoid.c` (tasks). Tests: `s5_unit_tests.c`
S5-U01..U20, on-target S5-U21..U23, integration INT-10, INT-14, INT-18.

## 1. Three tasks, kept separate (Tribunal 1 Version B: "No change")

| Task | Prio | Owns | Does |
|---|---|---|---|
| Obstacle Scan | 15, core 1 | HC-SR04 (TRIG GP4, ECHO GP5 via the shared GPIO interrupt), SG90 servo (GP15, PWM slice 7B) | forward ranging every 60 ms; detection (250 mm, 2 readings); speed cap 100 mm/s under 450 mm; emergency stop under 100 mm while avoiding; coarse/fine sweeps on request |
| Obstacle Profile | 19 | nothing | collects the packed samples of one scan id, builds the profile, sends it on |
| Avoidance & Recovery | 16 | nothing | stop → coarse → fine → profile → plan → execute through Buddy 2 → search line → align → resume, all bounded |

Messages between them are packed words over bounded message buffers
(`s5_msg.h`):

* scan request, 25 bits;
* sample, 25 bits, carrying a 3-bit scan id, a phase bit, a last-sample
  bit and a 2-bit status;
* profile, 2 words.

Each round-trips exactly (S5-U01..U05). Distances use 10 bits at **5 mm**:
the HC-SR04 resolves about 3 mm and is accurate to about 1 cm, so 5 mm
keeps the measurement (Tribunal 1 Buddy 5: "verify unpacked == original";
risk R4).

## 2. Ranging results are never faked

| Result | Meaning | Code / fault |
|---|---|---|
| `US_OK` | 20..4000 mm | distance code 0..800 |
| `US_NO_ECHO` | echo pulse longer than 4 m range: **nothing there** (a valid "clear" reading) | code 1022, no fault |
| `US_NO_RESPONSE` | ECHO never rose (sensor missing / dead) | code 1023; after 5 in a row `SYS_FLT_US_NO_RESPONSE` (critical condition, `SYS_US_REQUIRED 1`); cleared after 3 good pings |
| `US_INVALID` | impossible pulse (< 2 cm, ECHO stuck high) | code 1023; `SYS_FLT_US_INVALID` (degraded); one retry during sweeps |

An SG90 gives no position feedback. A frozen servo is detected indirectly:
identical ranges (± 10 mm) over a ≥ 60° sweep give an INVALID profile
(S5-U11).

## 3. Obstacle profile (`obstacle_profile.c`; Briefing p.8: W ≈ f(dL, dC, dR, θ))

Each sample becomes a point: lateral x = d·sin θ, forward y = d·cos θ.

1. **Validity:** at least 5 readings (OK or NO_ECHO), and not a frozen
   sweep. Otherwise INVALID.
2. **Nearest face:** the smallest forward distance y within ±30°. If it is
   farther than 700 mm the path is clear (NO_OBSTACLE).
3. **Extent:** neighbouring samples belong to the obstacle while their
   **forward** distance is within 150 mm of that face. Forward distance,
   not range, is used, so a flat face or a wall is recognised as one
   surface (S5-U10).
4. **Edges:** midway between the last hit and the first miss. The fine
   scan (3° steps) narrows that gap. Edge positions are taken on the face
   plane: x = y_near · tan θ.
5. **Clearance per side:** from the edge to the nearest other object
   within 500 mm deeper, or to the lateral reach of the scan at that depth.
   It is capped at 2.54 m.
6. **Unknown is never free:** if the obstacle reaches the scan limit on a
   side, that edge and its clearance are UNKNOWN (code 255). If both are
   unknown the profile is INSUFFICIENT, and no bypass is ever planned
   through unknown space (brief: "do not invent a successful bypass").

Accuracy against ray-cast synthetic boxes is within 10 % in width and
1 cm in distance (S5-U06, 4 boxes, simulation). `US_BEAM_COMP_DEG` (default
0, conservative: the obstacle looks wider) can subtract a measured beam
half-width. Tribunal 1 rejected "assuming ultrasonic scanning determines
exact obstacle geometry".

## 4. Avoidance plan (`avoid_plan.c`; Briefing p.8 workflow)

* **Side choice:** the side with more known clearance; on a tie, right
  (`AV_PREFER_RIGHT`). A side qualifies only if its clearance ≥ car width +
  2 margins (150 + 2 × 60 mm `[CAL]`). If neither qualifies →
  `OBSTACLE_UNAVOIDABLE` → Safe Stop (FR12).
* **Manoeuvre** (side s = ±1, T = 45°):
  `TURN s·T, MOVE d1, TURN −s·T, MOVE d2, TURN −s·T, SEARCH ≤ dS, TURN s·T`.
  Here d1 = lateral offset / sin T, and d2 carries the whole car past the
  assumed 200 mm obstacle depth `[CAL]`. If the obstacle is closer than
  180 mm, no avoidance plan is produced and the vehicle enters Safe Stop.
* **Bounded drift (NFR5):** the lateral offset is just edge + half width +
  margin.
* **Commands:** every step is an `MSRC_AVOID` command to Buddy 2 at speed
  level 6 (150 mm/s; search at level 3). Each waits for its done/aborted/fault
  result with a timeout (Tribunal 1: "Buddy 5 waits on the completion
  status"). Buddy 5 never touches the motors.

## 5. Recovery and line reacquisition (FR11, FR12)

* **SEARCH:** move forward until Buddy 3 reports the line, either through
  the `LEV_FOUND` event bit of `flg_line_evt` or the line status word
  showing L or C black. Then stop (`motion_stop`, queue bypass) and align.
  One retry with a 20° steeper approach.
* **Bounds:** the whole avoidance is limited to 25 s
  (`RECOVERY_TIMEOUT`). An emergency stop (something new within 100 mm)
  allows one re-plan from where the car stands.
* **Failures → Safe Stop** with a reason (telemetry `avoid_state`
  FAILED, fail code): profile invalid, no clearance, manoeuvre failed, line
  not found, timeout. Leaving Safe Stop takes a supervisory RESUME, and only
  once no critical condition remains.
* **Mode hand-over:** Buddy 5 posts `SYS_EV_OBSTACLE`; Buddy 2 switches to
  OBSTACLE (new epoch, current command aborted, motors stopped). Line
  steering is ignored in that mode. On success Buddy 5 posts
  `SYS_EV_AVOID_DONE` and Buddy 2 returns to NORMAL (INT-10, INT-14).

Simulated bypass of a 150 mm box at 300 mm with the plant model (INT-10):

* closest approach 201 mm beside the box edge;
* back on the line (offset 1 mm);
* end heading −8.0° (within ±10°).

The heading error comes mostly from the 10 mm encoder tick (20 slots/rev
`[CAL]`); see `subsystem2/docs/VERIFICATION_REPORT.md` §7.

## 6. Remaining hardware validation (assumption A6 stays open)

1. HC-SR04 at 3.3 V: compare with known distances (100 mm–1.5 m) and
   angles; record repeatability (Tribunal 1).
2. Servo: calibrate `SERVO_CENTER_US`, `SERVO_US_PER_DEG_X10` and
   `SERVO_DIR`; measure the settle time per degree.
3. Scan known boxes at several distances: profile width and edges against
   the tape measure; set `US_BEAM_COMP_DEG`.
4. Measure the car's width, length, turning behaviour and the margins in
   `avoid_plan.h`.
5. Full avoidance on the track (FR8–FR12), including the recovery timeout
   and an unavoidable obstacle → Safe Stop.
