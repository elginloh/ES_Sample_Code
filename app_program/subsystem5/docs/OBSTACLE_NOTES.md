# Buddy 5: scanning, profiling, avoidance and recovery notes

Code: `drv_ultrasonic.c`, `drv_servo.c` (hardware); `s5_msg.c`,
`obstacle_profile.c`, `avoid_plan.c` (pure logic); `task_scan.c`,
`task_profile.c`, `task_avoid.c` (tasks). Tests: `s5_unit_tests.c`
S5-U01..U20, on-target S5-U21..U23, integration INT-10, INT-14, INT-18.

## 1. Three tasks, kept separate (Tribunal 1 Version B: "No change")

| Task | Prio | Owns | Does |
|---|---|---|---|
| Obstacle Scan | 15, core 1 | HC-SR04 (TRIG GP4, ECHO GP5 via the shared GPIO interrupt), SG90 servo (GP15, PWM slice 7B) | forward ranging every 60 ms; detection begins at 260 mm so braking settles near the requested 200 mm bumper gap; speed cap 100 mm/s below 410 mm; emergency stop at 100 mm bumper gap; smooth coarse/fine sweeps on request |
| Obstacle Profile | 19 | nothing | collects one packed scan phase, builds the profile, sends it on |
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

Scan motion is slew-limited to 3° every 20 ms (about 150°/s) and allowed
50 ms to settle before each range measurement.  These values are hardware
calibration points in `drv_servo.h`; PWM remains at the servo-required 50 Hz.
The per-scan timeout is 15 s. A 49-point fine scan with three pings per angle
takes about 10 s, leaving margin for serial/Wi-Fi scheduling.

With `SERVO_DEBUG=1`, the console reports servo initialisation, each target
angle and PWM pulse, the slew step count, the detection distance, and every
scan sample.  Sample result codes are `0=OK`, `1=NO_ECHO`,
`2=NO_RESPONSE`, and `3=INVALID`.  Reported angles are commanded positions;
the SG90 has no physical position feedback.

With `US_DEBUG=1`, every ultrasonic trigger reports its sequence number,
result code, final ECHO pin level, captured edge mask (`1=rise`, `2=fall`,
`3=both`), trigger-to-rise time, echo pulse width, and converted distance.
This raw timing distinguishes a missing sensor, stuck ECHO, out-of-range
target, and a valid but geometrically unexpected reflection.

During coarse and fine sweeps the scanner takes three pings at every settled
servo angle. Two valid returns must agree within 80 mm; three valid returns
use their median. Two NO_ECHO results establish open space. This prevents one
off-axis/background echo from becoming an obstacle edge. The normal forward
monitor remains one ping per 60 ms cycle so it can stop promptly.

## 3. Obstacle profile (`obstacle_profile.c`; Briefing p.8: W ≈ f(dL, dC, dR, θ))

Each sample becomes a point: lateral x = d·sin θ, forward y = d·cos θ.

1. **Validity:** at least 5 readings (OK or NO_ECHO), and not a frozen
   sweep. Otherwise INVALID.
2. **Nearest face:** the smallest forward distance y within ±30°. If it is
   farther than 700 mm the path is clear (NO_OBSTACLE).
3. **Extent:** neighbouring samples belong to the obstacle while their
   **forward** distance is within a measured symmetric ±50 mm band around
   that face.
   The lower bound is important: a high-angle reflection can project much
   closer than the actual front face and must not extend the obstacle to the
   scan limit. Forward distance, not range, is used, so a flat face or wall
   is recognised as one surface (S5-U10, S5-U13c).
4. **Edges:** midway between the last hit and the first miss. The fine
   scan (3° steps) narrows that gap. Its bounds extend far enough beyond
   the coarse edges to measure the complete 352 mm robot corridor, rather
   than using only a fixed angular margin. Edge positions are taken on the
   face plane: x = y_near · tan θ.
5. **Clearance per side:** from the edge to the nearest other object
   within the ±50 mm front-plane band, or to the lateral reach of the scan
   at that depth. Nearer specular returns and deeper background reflections
   are not treated as an immediate side wall. One isolated fine-scan miss is
   bridged when a front-face hit exists immediately beyond it; two consecutive
   misses still mark an edge. Clearance is capped at 2.54 m.
6. **Unknown is never free:** if the obstacle reaches the scan limit on a
   side, that edge and its clearance are UNKNOWN (code 255). If both are
   unknown the profile is INSUFFICIENT, and no bypass is ever planned
   through unknown space (brief: "do not invent a successful bypass").

Accuracy against ray-cast synthetic boxes is within 10 % in width and
1 cm in distance (S5-U06, 4 boxes, simulation). `US_BEAM_COMP_DEG` (default
0, conservative: the obstacle looks wider) can subtract a measured beam
half-width. Tribunal 1 rejected "assuming ultrasonic scanning determines
exact obstacle geometry".

On this car, repeated hardware sweeps expanded a roughly 200 mm central
obstacle to 470–525 mm and requested a 426 mm offset. Production profiles
therefore apply a measured 24° inward edge correction (`S5_BEAM_COMP_DEG`)
after raw geometry is built. The console prints both corrected and raw widths.
Unknown edges are never corrected, and the 320 mm maximum-offset guard remains
active.

## 4. Avoidance plan (`avoid_plan.c`; Briefing p.8 workflow)

* **Side choice:** the side with more known clearance; on a tie, right
  (`AV_PREFER_RIGHT`). A side qualifies only if its clearance ≥ car width +
  two side gaps (152 + 2 × 100 = 352 mm `[CAL]`). Board edges are intentionally
  not constrained because the test board is assumed wide enough. If neither qualifies →
  `OBSTACLE_UNAVOIDABLE` → Safe Stop (FR12).
* **Manoeuvre** (side s = ±1):
  `TURN s·45°, MOVE d1, TURN −s·45°, PASS, TURN −s·30°, SEARCH ≤ dS, TURN s·30°`.
  Here d1 = lateral offset / sin 45°. During PASS the servo looks 75° toward
  the obstacle. Two side hits establish it; five consecutive readings beyond
  230 mm (or NO_ECHO) establish the far edge. The 230 mm centre-sensor limit
  includes tolerance above the expected 176 mm reading at a 100 mm edge gap.
  Buddy 5 stops the bounded pass, advances only 160 mm, then turns 30° and
  travels forward-diagonally toward the line. The shallower return uses the
  100 mm lateral gap while avoiding the inner-corner sweep of an immediate
  45° pivot. It therefore handles variable obstacle length instead of
  assuming 200 mm or travelling a full body length while still parallel.
  The measured geometry is 152 mm width, 215 mm length, with the sensor
  directly above the front bumper. If the initial bumper gap is below 200 mm,
  no avoidance plan is produced until the bounded reverse/rescan succeeds.
* **Bounded drift (NFR5):** the lateral offset is measured edge + 76 mm
  half-width + the requested 100 mm side gap. Because the ultrasonic edge is
  treated conservatively, this is a minimum predicted gap rather than extra
  drift. A centre offset above 320 mm is rejected as an implausible/noisy
  profile instead of allowing the robot to leave the course.
* **Too-close recovery:** when a bypass side is already known safe but the
  bumper gap is below 200 mm, Buddy 5 may reverse once along the path just
  travelled. It targets a 220 mm gap, is capped at 150 mm, runs at 25 mm/s,
  stops, and repeats the complete coarse/fine scan. Reverse remains disabled
  for every other source and mode; failure or a second unsafe scan → Safe Stop.
* **Commands:** every step is an `MSRC_AVOID` command to Buddy 2 at speed
  level 2 (50 mm/s; search at level 1). Each waits for its done/aborted/fault
  result with a timeout (Tribunal 1: "Buddy 5 waits on the completion
  status"). Buddy 5 never touches the motors.

## 5. Recovery and line reacquisition (FR11, FR12)

* **SEARCH:** move towards the line. A side-sensor `LEV_FOUND` event only
  wakes the task; it continues until the centre IR sensor is black. The
  centre sensor is approximately 80 mm ahead of the wheel axle, so the robot
  then advances that calibrated distance before its final alignment turn.
  This puts the body on the line instead of turning as soon as the front
  sensor touches it. One retry uses a 20° steeper approach.
* **Bounds:** the whole avoidance is limited to 90 s
  (`RECOVERY_TIMEOUT`), including one bounded reverse and rescan. An emergency
  stop (something new within 100 mm)
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

* closest approach 193 mm beside the box edge;
* back on the line (offset 0 mm);
* end heading 0.6° (within ±10°).

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
