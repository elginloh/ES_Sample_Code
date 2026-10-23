# Minimum-bit data representation: Week 13 evidence

Change Requirement 1 asks for four kinds of evidence. This file gives the
data representation table (§A), the RAM comparison (§B), the message size
comparison (§C) and the correctness and performance evidence (§D).

**Sources of every number** (nothing here is estimated unless marked
*computed*):

* **[ld]** GNU ld map file of the default image
  (`make -j8`: SMP=1, USB console, CYW43, TEST=0), parsed per object file.
* **[sizeof]** `arm-none-eabi-gcc` 15.3 with the firmware's own flags
  (`-mcpu=cortex-m0plus -O2 …`), measured with `nm -S` on a probe object.
  These are target-ABI sizes, padding included.
* **[test]** printed by an automated test (host run; the same code runs in
  the `TEST=1` image).
* ***computed*** derived from [sizeof] with the kernel's documented formula
  (message buffer = n × (4 + size rounded up to 4)).

Rule for "required bits": ⌈log₂(number of distinct values)⌉, counting
invalid or reserved codes. All packing uses explicit masks and shifts,
never C bit-fields, so the layout is compiler-independent (Tribunal 1).

---

## A. Data representation table

"Original" means the representation before this change, where a document
records one. Where none existed (the repository had only the template's
example subsystems), the column gives the natural unpacked C form, which is
the type the getter returns, measured in §C.

### A.1 Buddy 2: motion, vehicle state, faults

| Field | Valid range | Resolution | Values | Req. bits | Actual representation | Original / unpacked |
|---|---|---|---|---|---|---|
| Vehicle mode | INIT, NORMAL, NAV_ACTION, OBSTACLE, SAFE_STOP | — | 5 | 3 | VSW bits 0–2 | enum / `UB` |
| Mode epoch | 0..7, wraps | 1 | 8 | 3 | VSW 3–5, command word 9–11 | — |
| Motion state | IDLE, STRAIGHT, TURNING, FAULT | — | 4 | 2 | VSW 6–7, status w0 22–23 | `UB` |
| Command source | LINE, BARCODE, AVOID, SUPER | — | 4 | 2 | command 3–4, VSW 8–9 | `UB` |
| Command type | MOVE, TURN, STOP, SET_SPEED, RESUME, CLEAR (+2 invalid) | — | 8 | 3 | command 0–2 | `UB` |
| Command sequence | 0..15, wraps | 1 | 16 | 4 | command 5–8, result 0–3 | `UB` |
| Speed level | 0 = default, 1..15 × 25 mm/s | 25 mm/s | 16 | 4 | command 12–15 | `UB` |
| Command argument | −2000..2000 mm or −360..360° | 1 mm / 1° | 4001 | 12 | command 16–27, two's complement | `INT` (32) |
| Result status / reason / valid | DONE, ABORTED, FAULT, REJECTED / 8 reasons / flag | — | 4 / 8 / 2 | 2 / 3 / 1 | result word bits 4–5 / 9–11 / 12 | `UB`, `UB`, `BOOL` |
| Steering | −63..63 (+ right) | 1 unit (4 mm/s wheel diff.) | 127 | 7 | steering word 5–11 | `INT` |
| Steering speed fraction | 0..15 of cruise | 1/15 | 16 | 4 | steering word 12–15 | `UB` |
| Speed cap | 0..1023 mm/s (same range as wheel speed) | 1 mm/s | 1024 | 10 | cap word 5–14 | `UW` |
| Wheel speed (L, R) | −1023..1023 mm/s + invalid | 1 mm/s | 2048 | 11 | status w0 0–10, 11–21 | `H` (16) |
| Odometer | 0..655.35 m, saturates | 1 cm | 65536 | 16 | status w1 0–15 | `UH` |
| Encoder heading | 0..358° | 2° | 180 | 8 | status w1 16–23 | `UH` |
| Cruise speed | 0..630 mm/s (set 50..400) | 10 mm/s | 64 | 6 | status w1 24–29 | `UH` |
| Flags (cmd active, stall L/R, encoders ok) | 0/1 | — | 2 each | 1 each | status w0 26–29 | `UB` each |
| IR/IMU calibrated, armed, line cap, obstacle cap | 0/1 | — | 2 each | 1 each | VSW 11–15 | `BOOL` (4 B) each |
| Fault-retry count | 0..3, saturates | 1 | 4 | 2 | VSW 16–17 | `UB` |
| **Faults** (27 independent conditions) | each 0/1 | — | 2²⁷ | 27 | **one shared bitmask**, bits 0–26, atomic set/clear | CR1: "rather than separate integer variables" (27 × `BOOL` = 108 B) |
| Events to Buddy 2 (10) | each 0/1 | — | 2¹⁰ | 10 | one word, bits 0–9, atomic OR / exchange | — |

**Not compacted, deliberately (internal computation precision; the brief:
"do not reduce precision used internally by the PID controller"):** the
encoder ISR record (32-bit tick count, µs timestamps), PID integrators,
and the controller's µm distances. They never leave Buddy 2 unpacked; the
encoder record is read under a sequence count.

### A.2 Buddy 3: line and barcode

| Field | Valid range | Resolution | Values | Req. bits | Actual | Original / unpacked |
|---|---|---|---|---|---|---|
| IR sensor states (L, C, R) | black/white each | — | 8 | 3 | line word 0–2 (bit per sensor) | 3 × `UB` |
| Line position | CENTER, LEFT_NEAR, LEFT_FAR, RIGHT, LOST_LEFT, LOST_RIGHT, JUNCTION | — | 7 | 3 | line word 3–5 | `UB` |
| Line state | ON_LINE, SEARCHING, LOST, UNKNOWN | — | 4 | 2 | line word 6–7 | `UB` |
| Sequence (freshness) | 0..15, wraps | 1 | 16 | 4 | line word 8–11 | `UB` |
| Steering output | −63..63 | 1 | 127 | 7 | line word 12–18 | `B` |
| Calibrated | 0/1 | — | 2 | 1 | line word 19 | `UB` |
| Junction / line-loss counts | mod 16 | 1 | 16 | 4 + 4 | line word 20–27 | `UB` each |
| Speed fraction | 0..15 | 1/15 | 16 | 4 | line word 28–31 | `UB` |
| Line event (FOUND, LOST, JUNCTION) | flags | — | — | 1 each | `flg_line_evt` bits | — |
| Barcode state | IDLE, CAPTURING, DECODING, EXECUTING | — | 4 | 2 | barcode word 0–1 | `UB` |
| Decoded symbol | Code 39 index 0..43, + none (63) | — | 45 | 6 | barcode word 2–7 | `UB` / char |
| Read result | NONE, VALID, INVALID, DUPLICATE | — | 4 | 2 | barcode word 8–9 | `UB` |
| Navigation command | LEFT, RIGHT, STRAIGHT, U_TURN | — | 4 | **2** (+1 valid bit) | barcode word 10–12 | `UB` + `BOOL`. Tribunal 1, Buddy 3: "2-bit codes for 4 navigation commands" |
| Reversed read, executed | 0/1 | — | 2 | 1 each | barcode word 13, 28 | — |
| Valid / invalid read counts | mod 64 | 1 | 64 | 6 + 6 | barcode word 14–25 | `UB` each |
| Motion result of the command | MRES_* | — | 4 | 2 | barcode word 26–27 | `UB` |
| Error class | 8 classes | — | 8 | 3 | barcode word 29–31 | `INT` error code |
| Barcode edge (ISR → task) | µs time + level | 2 µs | 2³¹ × 2 | 32 | ring entry: bits 31..1 time, bit 0 level | 8 B (`UW` + `UB`) |

### A.3 Buddy 4: IMU and terrain

| Field | Valid range | Resolution | Values | Req. bits | Actual | Original / unpacked |
|---|---|---|---|---|---|---|
| Pitch, roll | −90..90° + invalid | 1° | 182 | 8 | IMU w0 0–7, 8–15 (−128 invalid) | Buddy 4: "float or int" |
| Yaw rate | −255..255 °/s + invalid | 1 °/s | 512 | 9 | IMU w0 16–24 (−256 invalid) | float/int |
| Heading (gyro) | 0..359° + invalid | 1° | 361 | 9 | IMU w1 0–8 (511 invalid) | float/int |
| Motion class | STATIONARY, MOVING, TURN_L, TURN_R | — | 4 | 2 | IMU w0 25–26 | `UB` |
| IMU status | INIT, CALIBRATING, OK, FAULT | — | 4 | 2 | IMU w0 27–28 | `UB` |
| Tilt, collision, sample valid, calibrated | 0/1 | — | 2 | 1 each | IMU w0 29–31, w1 24 | `BOOL` |
| Peak shock | 0..15.5 g | 0.5 g | 32 | 5 | IMU w1 9–13 | `W` (mg) |
| Sample sequence | mod 256 | 1 | 256 | 8 | IMU w1 14–21 | — |
| Chip family | none, InvenSense, ST | — | 3 | 2 | IMU w1 22–23 | `UB` |
| Snapshot time | ms since boot (wraps after 49 days) | 10 ms (kernel tick) | 2³² | 32 | third word under the same sequence count (Tribunal 1: "timestamped snapshot with a validity flag") | — |
| **Hump height** (last, highest) | 0..100 mm + none | **2 mm** | 52 | **6** | hump word 0–5, 6–11 (63 none, saturation flags 17–18) | 7 bits at 1 mm before Tribunal 1; **Version B decision: 2 mm / 6 bits** |
| Hump count | 0..15, saturates | 1 | 16 | 4 | hump word 12–15 | `UB` |
| Hump in progress | 0/1 | — | 2 | 1 | hump word 16 | `BOOL` |
| Last hump length | 0..255 cm | 1 cm | 256 | 8 | hump word 19–26 | `W` mm |
| IMU → Hump sample: pitch | −102.3..102.3° | 0.1° | 2047 | 11 | message 0–10 | `W` centi-deg |
| IMU → Hump sample: distance step | 0..25.5 mm per 10 ms | 0.1 mm | 256 | 8 | message 11–18 | `W` µm |
| IMU → Hump sample: valid, sequence | flag, mod 16 | — | 2 / 16 | 1 / 4 | message 19, 20–23 | — |

The highest hump is kept at full resolution (µm) inside the task and only
compacted for storage or transmission (Tribunal 1: "No change").

### A.4 Buddy 5: obstacle

| Field | Valid range | Resolution | Values | Req. bits | Actual | Original / unpacked |
|---|---|---|---|---|---|---|
| Distance | 0..4000 mm + no echo + invalid | **5 mm** (sensor ≈ 3 mm resolution, ≈ 1 cm accuracy) | 803 | 10 | codes 0..800, 1022, 1023 | `W` mm |
| Servo angle | −80..80° (+ right) | 1° | 161 | 8 | sample 0–7, scan status 10–17 | `INT` |
| Scan id | 0..7, wraps | 1 | 8 | 3 | request 2–4, sample 18–20, profile 0–2 | `UB` |
| Sample phase, last flag | coarse/fine, flag | — | 2 / 2 | 1 / 1 | sample 21, 22 | `UB` |
| Sample status | OK, NO_ECHO, INVALID, ABORTED | — | 4 | 2 | sample 23–24 | `UB` |
| Request type / step | 4 types / 1..15° | 1° | 4 / 16 | 2 / 4 | request 0–1 / 21–24 | `UB` |
| Profile status | VALID, NO_OBSTACLE, INVALID, INSUFFICIENT | — | 4 | 2 | profile w0 3–4 | `UB` |
| Obstacle forward distance | as distance | 5 mm | 803 | 10 | profile w0 5–14 | `W` |
| Bearing | −127..127° | 1° | 255 | 8 | profile w0 15–22 | `W` |
| Width | 0..1270 mm + unknown | 5 mm | 256 | 8 | profile w0 23–30 (255 unknown) | `W` |
| Edge positions (L, R) | −1270..1270 mm | 10 mm | 255 | 8 + 8 | profile w1 0–15 | `W` each |
| Clearance (L, R) | 0..2540 mm, ≥ 2.54 m, unknown | 10 mm | 256 | 8 + 8 | profile w1 16–31 (254 = ≥ 2.54 m, 255 unknown) | `W` each |
| Avoidance state | 9 states | — | 9 | 4 | avoid word 0–3 | `UB` |
| Side, plan step, retries, outcome | 3 / 0..7 / 0..3 / 4 | — | — | 2 / 3 / 2 / 2 | avoid word 4–12 | `UB` each |
| Avoidance count, fail reason | mod 16 / 7 reasons | — | 16 / 7 | 4 / 3 | avoid word 13–19 | `UB` each |
| Scan status: result, mode, seq, cap, detected | 4 / 4 / 16 / flag / flag | — | — | 2/2/4/1/1 | scan word 18–27 | `UB` each |
| **Profile-task sample buffer** (64 samples) | angle ±80°, status, 0..4000 mm or −1 | 1°, 1 mm | 161 / 4 / 4002 | 8 + 2 + 12 | `OP_SAMPLE` = `B` + `UB` + `H` = **4 B** | `INT` + `W` + `UB` = 12 B (until 2026-10-23) |

### A.5 Buddy 1: communication

| Field | Valid range | Values | Req. bits | Actual | Original |
|---|---|---|---|---|---|
| Telemetry (48 fields) | see `subsystem1/docs/MQTT_BINARY_PROTOCOL.md` §5.1 | — | 279 total | 35-byte payload, 40-byte frame | **Week 6 §6.1: JSON**, 697 B for the same fields [test] |
| Heartbeat (6 fields) | §5.2 | — | 32 | 4-byte payload, 9-byte frame | not specified |
| Command / ACK / event | §5.3–5.5 | — | 24 / 20 / 36 | 3 / 3 / 5-byte payloads | not specified |
| Connection state | 8 states | 8 | 3 | comm word 0–2 | `UB` |
| Availability | ONLINE, DISCONNECTED, RECOVERING | 3 | 2 | comm word 3–4 | `UB` |
| Link / session state | 4 / 4 | 4 / 4 | 2 / 2 | comm word 5–8 | `UB` each |
| Attempts / reconnects | 0..15, saturate | 16 | 4 / 4 | comm word 9–16 | `UB` each |
| Dropped / rejected counters | 0..255, saturate | 256 | 8 / 8 | telemetry fields 47–48 | `UW` |

---

## B. RAM usage

### B.1 Measured before/after of one representation change in this code [ld]

The obstacle profile task buffered 64 samples as `{INT angle; W dist;
UB status}` = 12 B each. The packed form `{B angle; UB status; H dist}` is
4 B each. Same firmware otherwise, default profile:

| Item | Before | After | Change |
|---|---|---|---|
| `subsystem5/task_profile.o` `.bss` (of which `.bss.buf`) | 780 B (768 B) | 268 B (256 B) | −512 B |
| subsystem 5 total `.data` + `.bss` | 899 B | 387 B | −512 B |
| Whole image `.bss` (`arm-none-eabi-size`) | 27 572 B | 27 060 B | −512 B |
| `op_build()` local copy + forward array (stack) | 768 + 256 B | 256 + 128 B | −640 B of stack *computed* |

The host and on-target unit tests were run before and after (all pass).
The profile accuracy test S5-U06 (width within 10 %, distance within
1 cm) is unchanged.

### B.2 Application RAM of the final image, per subsystem [ld]

Final default image: `arm-none-eabi-size` text 317 414, data 112,
bss 27 128 (the template image: 285 124 / 88 / 24 476).

| Subsystem | `.data` | `.bss` | Static RAM | Code + rodata |
|---|---|---|---|---|
| subsystem1 | 0 | 417 | 417 | 11 008 |
| subsystem2 | 8 | 618 | 626 | 10 563 |
| subsystem3 | 8 | 851 | 859 | 6 921 |
| subsystem4 | 8 | 350 | 358 | 7 160 |
| subsystem5 | 0 | 387 | 387 | 8 488 |
| **Total** | **24** | **2 623** | **2 647** | 44 624 |

Largest items (`.data` + `.bss` per object):

* barcode decoder state, 451 B: 64 edge times kept at full µs resolution
  for decoding;
* Motion Control, 392 B;
* telemetry snapshot + 2 records, 299 B;
* IR edge ring, 276 B;
* IMU processing, 276 B;
* profile buffer, 268 B.

Also allocated by the kernel at start-up (heap, not in the map):

* message buffers: 2 112 B *computed* (`INTERFACES.md` §3);
* task stacks: 10 × 2 KB.

The professor's template image (LED/producer/consumer examples) had 56 B of
application `.bss` [ld]. It implements none of these functions, so it is
**not** a meaningful "before" for the minimum-bit change.

### B.3 Packed versus unpacked storage of the shared data [sizeof]

Each shared item is stored packed and unpacked by its readers on demand.
The unpacked view types are the ones the getters return.

| Shared item (one instance) | Packed | Unpacked view | Saving |
|---|---|---|---|
| Fault conditions | 4 B (27-bit mask) | 108 B (27 × `BOOL`), or 27 B as `UB` | 104 B |
| Vehicle status word | 4 B | 32 B (`VSW_UNPACKED`: mode, epoch, state, source, 6 flags, retries) | 28 B |
| Motion status | 8 B | 16 B (`MOTION_STATUS`) | 8 B |
| Line status (+ time word) | 4 + 4 B | 16 B (`S3_LINE_STATUS`) | 8 B |
| Barcode status | 4 B | 11 B (`S3_BC_STATUS`) | 7 B |
| IMU status (+ time word) | 8 + 4 B | 20 B (`S4_IMU_STATUS`) | 8 B |
| Hump status | 4 B | 7 B (`S4_HUMP_STATUS`) | 3 B |
| Scan status | 4 B | 8 B (`S5_SCAN_STATUS`) | 4 B |
| Avoidance status | 4 B | 7 B (`S5_AVOID_STATUS`) | 3 B |
| Obstacle profile | 8 B | 32 B (`S5_PROFILE`) | 24 B |
| Communication status | 4 B | 8 B (`S1_COMM_STATUS`) | 4 B |
| Steering setpoint | 4 B | 16 B (`STEER_UNPACKED`) | 12 B |
| Speed cap | 4 B | 12 B (`SCAP_UNPACKED`) | 8 B |
| Result word (× 4 sources) | 4 × 4 B | 4 × 8 B (`MRES_UNPACKED`) | 16 B |
| **Sum** | **88 B** | **325 B** | **237 B** |

Packing also makes each item **atomic**. Single-word items are published
with one aligned store, which is what makes the lock-free single-writer
scheme in `INTERFACES.md` §9 safe. The unpacked structs could not be
updated that way.

---

## C. Message sizes

### C.1 Inter-task queue messages [sizeof]; kernel buffer *computed*

| Queue (depth) | Packed message | Unpacked struct | Kernel buffer, packed | Kernel buffer, unpacked |
|---|---|---|---|---|
| `mbf_motion_cmd` (16) | 4 B command word | 12 B (`MCMD_UNPACKED`) | 128 B | 256 B |
| `mbf_imu_hump` (32) | 4 B (24 bits used) | 6 B (`S4_HUMP_SAMPLE`) | 256 B | 384 B |
| `mbf_scan_req` (4) | 4 B (25 bits) | 5 B (`S5_SCAN_REQ`) | 32 B | 48 B |
| `mbf_scan_samples` (64) | 4 B (25 bits) | 8 B (`S5_SAMPLE`) | 512 B | 768 B |
| `mbf_profile` (4) | 8 B (2 words) | 32 B (`S5_PROFILE`) | 48 B | 144 B |
| Barcode edge ring (64) | 4 B | 8 B (`EDGE_UNPACKED`) | 256 B (static) | 512 B |
| **Total** | | | **1 232 B** | **2 112 B** |

### C.2 MQTT and telemetry [test]

| Message | Week 6 / unpacked | Bytes | Binary v1 | Bytes | Ratio |
|---|---|---|---|---|---|
| Telemetry (48 fields) | JSON text (Week 6 §6.1) | 697 | frame | 40 | 5.7 % |
| Telemetry record in RAM | `TLM_RECORD` struct [sizeof] | 60 | packed payload | 35 | 58 % |
| Telemetry queue slot (B1) | JSON slot *computed* | 4 + 700 | frame slot | 4 + 68 | 10 % |
| Heartbeat | — | — | frame | 9 | |
| Command / ACK | — | — | frame | 8 / 8 | |
| Event | — | — | frame | 10 | |

With 4 + 8 telemetry/MQTT queue slots, JSON frames would need about
8.4 KB of message-buffer RAM *computed*; the binary frames need 864 B.

---

## D. Correctness and performance

### D.1 Encoding and decoding correctness (automated, all pass on the host)

| What | Tests |
|---|---|
| Every packed word / message round-trips at its limits, signs and invalid codes | S2-U01, U01b, U04, U05 (command, result, steering, cap, motion status); S3-U30, U31 (line, barcode); S4-U26..U28 (IMU, hump, queue sample); S5-U01..U05 (distance code, request, sample, profile, status words); S1-U20 (comm word) |
| Bit order and CRC against independent references | S1-U01 (CRC catalogue value), S1-U02 (known bytes) |
| MQTT frames equal golden frames from an independent Python implementation; the Node-RED decoder decodes them | S1-U03..U06; `node nodered_decoder.js` (7/7 pass) |
| Every field at max and min | S1-U07 |
| All 48 telemetry fields, produced by each subsystem's own pack function, reach the server unchanged; invalid codes preserved | INT-15a |
| Malformed input rejected, never executed | S1-U08..U10, INT-20c, S3-U05..U09 |

### D.2 No unacceptable precision loss

| Quantity | Step | Worst error | Evidence |
|---|---|---|---|
| Hump height | 2 mm | ≤ 1 mm; end-to-end 30 mm and 45 mm humps within ±4 mm at the server | S4-U24/U25, S4-U19/U20, INT-16a/b |
| Obstacle distance | 5 mm | ≤ 2.5 mm (sensor accuracy ≈ 10 mm) | INT-15b; S5-U06 (profile within 1 cm) |
| Obstacle width / clearance | 5 / 10 mm | ≤ 2.5 / 5 mm | INT-15b |
| Encoder heading / cruise speed in telemetry | 2° / 10 mm/s | ≤ 1° / 5 mm/s | INT-15b |
| Pitch, yaw rate | 1°, 1 °/s | ≤ 0.5 | S4-U29, INT-15b |
| Control loop | not compacted | — | §A.1 note |

### D.3 No introduced race conditions

Every shared packed item has one of the protections in `INTERFACES.md` §9:

* atomic read-modify-write for multi-writer words;
* single-writer aligned stores;
* sequence counts for multi-word records;
* a single-producer/single-consumer ring for the ISR.

There is no read-modify-write on a word with several writers. This is a
design argument checked by code review; data races cannot be proven by
the host tests. Hardware stress (all tasks, edges at full speed) remains to
be run.

### D.4 Preserved behaviour and fault handling

* 178 automated checks pass on the host, with 0 failures (unit suites
  S1–S5 plus the integration tests).
* Integration tests INT-10..INT-20 run the real pure modules of all five
  subsystems together:
  * avoidance;
  * barcode navigation;
  * simultaneous events;
  * fault propagation;
  * line reacquisition;
  * network loss;
  * Safe Stop priority;
  * queue-full conditions;
  * remote command validation.

### D.5 Task timing (must be measured on the Pico)

The `TEST=1` image measures and prints, with all tasks running:

* the TIMER rate (S2-U41);
* Motion Control period/worst run (S2-U42);
* stop-request latency (S2-U44);
* Line Following, IMU, scan and telemetry rates (S3-U32, S4-U30, S5-U21,
  S1-U22).

**No hardware run has been made by the implementer, so no timing figure is
claimed here.** Record the console log in `testing/*/logs/` (README §4). For
a before/after timing comparison, the per-task worst run times are the
figures to compare.
