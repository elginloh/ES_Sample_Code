# Buddy 5: scanning, profiling, avoidance and recovery notes

Code: `drv_ultrasonic.c`, `drv_servo.c` (hardware); `s5_msg.c`,
`obstacle_profile.c`, `avoid_plan.c` (pure logic); `task_scan.c`,
`task_profile.c`, `task_avoid.c` (tasks). Tests: `s5_unit_tests.c`
S5-U01..U20, on-target S5-U21..U23, integration INT-10, INT-14, INT-18.

## 1. Three tasks, kept separate (Tribunal 1 Version B: "No change")

| Task | Prio | Owns | Does |
|---|---|---|---|
| Obstacle Scan | 15, core 1 | HC-SR04 (TRIG GP4, ECHO GP5 via the shared GPIO interrupt), SG90 servo (GP15, PWM slice 7B) | forward ranging every 60 ms; detection begins at 210 mm so braking settles near the requested 150 mm bumper gap; speed cap 100 mm/s below 360 mm; emergency stop at 100 mm bumper gap; one full-resolution sweep on request |
| Obstacle Profile | 19 | nothing | collects one packed scan phase, builds the profile, sends it on |
| Avoidance & Recovery | 16 | nothing | stop → full fine scan → profile → plan → execute through Buddy 2 → search line → align → resume, all bounded |

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
The per-scan timeout is 22 s. The 51-point −75°..+75° fine scan uses three
pings per angle. Normal echoes finish in about 10 s, but many NO_ECHO angles
can approach 18 s because each waits for the ultrasonic timeout.

With `SERVO_DEBUG=1`, the console reports servo initialisation, each target
angle and PWM pulse, the slew step count, the detection distance, and every
scan sample.  Sample result codes are `0=OK`, `1=NO_ECHO`,
`2=NO_RESPONSE`, and `3=INVALID`.  Reported angles are commanded positions;
the SG90 has no physical position feedback.

With `US_DEBUG=1`, ultrasonic output is event-based rather than printed for
every 60 ms ping. It reports when a target enters `S5_DETECT_MM`, while a
nearby distance changes by at least 25 mm, once when it clears the threshold
(with 50 mm hysteresis), and on sensor fault/recovery transitions. Each event
still includes its sequence number, result code, final ECHO pin level,
captured edge mask (`1=rise`, `2=fall`, `3=both`), trigger-to-rise time, echo
pulse width, and converted distance. This keeps the serial monitor readable
without losing the timing needed to diagnose the sensor.

During the full fine sweep the scanner takes three pings at every settled
servo angle. Two valid returns must agree within 80 mm; three valid returns
use their median. Two NO_ECHO results establish open space. This prevents one
off-axis/background echo from becoming an obstacle edge. The normal forward
monitor remains one ping per 60 ms cycle so it can stop promptly.

## 3. Obstacle profile (`obstacle_profile.c`; Briefing p.8: W ≈ f(dL, dC, dR, θ))

Each sample becomes a point: lateral x = d·sin θ, forward y = d·cos θ.

1. **Validity:** at least 5 readings (OK or NO_ECHO), and not a frozen
   sweep. Otherwise INVALID.
2. **Nearest face:** the median forward distance at central −3°, 0° and +3° is
   used when available, with the smallest return within ±30° only as a
   fallback. This prevents one short/specular sample from making a rescanned
   obstacle appear tens of millimetres closer. If it is farther than 700 mm
   the path is clear (NO_OBSTACLE).
3. **Extent:** neighbouring samples belong to the obstacle while their
   **forward** distance is within a measured symmetric ±50 mm band around
   that face.
   The lower bound is important: a high-angle reflection can project much
   closer than the actual front face and must not extend the obstacle to the
   scan limit. Forward distance, not range, is used, so a flat face or wall
   is recognised as one surface (S5-U10, S5-U13c).
4. **Edges:** midway between the last hit and the first miss. The fine
   scan (3° steps) narrows that gap. Its fixed ±75° bounds measure the widest
   corridor visible to the sensor without needing a preliminary coarse pass.
   Edge positions are taken on the
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
obstacle to 470–525 mm. Production profiles therefore apply a measured 24°
inward edge correction (`S5_BEAM_COMP_DEG`) after raw geometry is built. If
the apparent angular blob is narrower than twice that correction, one observed
3° scan cell is retained. Its millimetre width therefore still comes from the
measured range and angle; there is no fixed minimum obstacle width. The console
prints both corrected and raw widths. Unknown edges are never corrected.

## 4. Avoidance plan (`avoid_plan.c`; Briefing p.8 workflow)

* **Side choice:** with `AV_ASSUME_OPEN_BOARD=1`, the board is treated like an
  open road. The 240/130 mm scan reaches are not interpreted as board edges.
  Each known obstacle edge defines a candidate route and the planner chooses
  the one requiring less lateral deviation; equal routes prefer right
  (`AV_PREFER_RIGHT`). The path uses the measured 152 mm robot width, requested
  100 mm obstacle gap and 25 mm scan/turn/odometry allowance. If both obstacle
  edges are unknown →
  `OBSTACLE_UNAVOIDABLE` → Safe Stop (FR12).
* **Manoeuvre** (side s = ±1):
  `[MOVE approach], TURN s·T, MOVE d1, TURN −s·T, PASS, TURN −s·30°, SEARCH ≤ dS, TURN s·30°`.
  The planner uses all measured spare forward space, then calculates
  the smallest safe integer departure angle T between 30° and 75° from the
  measured lateral offset and remaining front clearance. Here
  d1 = lateral offset / sin T. During PASS the servo looks 80° toward
  the obstacle. Two side hits establish it; five consecutive readings beyond
  a plan-derived side-range limit (or NO_ECHO) establish the far edge. The
  limit is the planned sensor-to-edge distance at 80° plus a 40 mm range/pose
  tolerance, rather than a fixed board-clearance number. Buddy 5 records the
  closest valid side range, subtracts 10 mm measurement uncertainty, and uses
  the resulting live body-to-obstacle gap to find the earliest safe 30° return.
  The corner calculation checks the complete in-place pivot and subsequent
  diagonal move while retaining the requested 100 mm gap. A conservative full
  body-length fallback remains when the side measurement proves no spare gap.
  The far-edge search itself is bounded only by Buddy 2's 2 m single-motion
  safety limit.
  The measured geometry is 152 mm width, 215 mm length, with the sensor
  directly above the front bumper. The forward monitor triggers at 210 mm,
  including 60 mm of stopping allowance, so the requested stopped gap is
  approximately 150 mm. The planner may then use verified spare space and move
  to a 120 mm minimum calculated turn-start gap. A plan is accepted only when its diagonal
  can establish the side gap before the front face; otherwise bounded
  reverse/rescan is attempted.
* **Bounded drift (NFR5):** the lateral offset is measured obstacle edge +
  76 mm half-width + requested 100 mm side gap + 25 mm measured motion
  allowance. Obstacle width and route offset are not fixed. There is no board
  width or board length assumption; only the motion command's representable
  distance remains a hard safety limit.
* **Too-close recovery:** when a bypass side is already known safe but the
  calculated 75° departure cannot establish the required lateral offset
  before the obstacle, Buddy 5 reverses only the missing distance along the
  path just travelled, plus a 20 mm rescan tolerance so range noise cannot
  leave the second profile just short of the same limit. The reverse is
  capped at 150 mm, runs at 25 mm/s, stops, and repeats the complete
  full-resolution scan. At the 120 mm calculated turn-start gap no reverse is needed;
  the 100 mm regression case reverses only to the calculated safe gap,
  rather than backing away toward a roughly 300 mm front gap. Reverse remains disabled
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
  sensor touches it. If the search command finishes just before Buddy 3's next
  20 ms sample, a 300 ms stationary grace period accepts that final centre-IR
  reading. One retry uses a 20° steeper approach.
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

* closest robot-centre position 252 mm beside the 75 mm half-width box,
  giving about 101 mm edge-to-edge clearance for the 152 mm-wide robot;
* back on the line (offset 0 mm);
* end heading 0.455° (within ±10°).

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
