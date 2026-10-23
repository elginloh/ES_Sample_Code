# CSC14 binary MQTT protocol, version 1

Owner: Buddy 1 (subsystem 1). Firmware reference implementation:
`app_program/subsystem1/mqtt_proto.c` + `bitpack.c`. Server reference
implementation: `nodered_decoder.js` (this folder).

This protocol replaces the JSON telemetry of the Week 6 design (section 6.1:
"formatted as JSON and published through Mosquitto to Node-RED").
Change Requirement 1 requires compact binary fields. Tribunal 1 Version B
requires one encode/decode standard for Buddies 1–5 and a shared field
specification with known-value tests (risk R1: payload tags and error
handling).

> **The server must be updated.** Node-RED flows written for the Week 6 JSON
> payloads cannot read these frames. Use `nodered_decoder.js` in a Function
> node, with the `mqtt in` node's output set to **"a Buffer"**.

## 1. Topics

| Topic | Direction | Frame type | Rate |
|---|---|---|---|
| `csc14/tlm` | robot → server | 1 telemetry | every 500 ms (Telemetry Collection task) |
| `csc14/hb` | robot → server | 2 heartbeat | every 1 s while online (FR13) |
| `csc14/evt` | robot → server | 3 event | on change (mode, faults, barcode, hump, obstacle, collision) |
| `csc14/cmd` | server → robot | 4 command | on demand, at most 5 per second (STOP is never throttled) |
| `csc14/ack` | robot → server | 5 acknowledgement | one per command whose id is readable |

All payloads are binary. The topic does not change the meaning of a frame;
the type nibble inside the frame does.

## 2. Frame

```
byte 0        (version << 4) | type      version = 1, type = 1..5
byte 1        sequence number            per type, modulo 256
byte 2        payload length L           must equal the schema size of the type
bytes 3..2+L  payload                    fields packed LSB-first (section 3)
bytes 3+L,4+L CRC-16/CCITT-FALSE         over bytes 0..2+L, LOW byte first
```

* **Byte order.** Fields are bit-packed, not byte-aligned (section 3). The
  only multi-byte quantity outside the bit stream is the CRC, sent
  little-endian.
* **CRC.** CRC-16/CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF, no
  reflection, no final XOR. Check value for the ASCII string `123456789` =
  `0x29B1` (test S1-U01). In Python this is `binascii.crc_hqx(data, 0xFFFF)`.
* **Field identifiers.** None per field. The pair (version, type) identifies
  the schema, so a field's position is its identifier. Any change to a
  schema needs a new version number. Receivers reject versions they do not
  know (`BAD_VERSION`) rather than guessing.
* **Maximum frame size.** 40 bytes (telemetry). The firmware buffer is 64.

## 3. Bit packing

Fields are written in the schema order. Each field is written least
significant bit first. Stream bit *k* is bit (*k* mod 8) of payload byte
⌊*k*/8⌋. Signed fields are two's complement of the stated width. Unused
bits in the last byte are 0. The firmware uses explicit masks and shifts
(`bitpack.c`), never C bit-field layout, so the format does not depend on
the compiler (Tribunal 1).

Example (S1-U02): writing 5 (3 bits), 31 (5 bits), 0xABC (12 bits) and −3
(4 bits) gives the bytes `FD BC DA`.

## 4. Receiver validation (robot side, NFR11)

`mp_frame_check()` and `mp_cmd_decode()` check the frame in this order and
stop at the first failure. Nothing is executed and nothing blocks.

| Check | Error | ACK sent? |
|---|---|---|
| fewer than 5 bytes | `MPE_SHORT` | no (frame untrusted) |
| CRC mismatch | `MPE_CRC` | no |
| version ≠ 1 | `MPE_VERSION` | no |
| type not 1..5 | `MPE_TYPE` | no |
| length byte ≠ actual length, or ≠ schema size | `MPE_LENGTH` | no |
| command: reserved bits ≠ 0, or command code 6/7 | `MPE_FIELD` | yes, `BAD_FIELD` |
| command: argument out of range | `MPE_ARG` | yes, `BAD_ARG` |

When the header or CRC is bad, the command id cannot be trusted, so these
frames are only counted. Each rejection increments `rx_rejected` (telemetry
field 48) and sets the degraded fault bit `CMD_INVALID` (bit 22). Accepted
commands then pass three more checks: rate (5 per second, STOP exempt →
`RATE`), duplicate id among the last 4 (→ `DUPLICATE`), and Buddy 2's
arbitration (→ `REJ_STATE` + reason).

## 5. Message schemas

### 5.1 Telemetry (type 1): 48 fields, 279 bits, 35-byte payload, 40-byte frame

"Off" is the bit offset in the payload. "Special" values never collide
with valid data.

| # | Field | Off | Bits | Signed | Unit / scale | Valid range | Special | Producer |
|---|---|---|---|---|---|---|---|---|
| 1 | uptime_ds | 0 | 16 | no | 0.1 s | 0..6553.5 s, wraps | — | B1 |
| 2 | mode | 16 | 3 | no | INIT, NORMAL, NAV_ACTION, OBSTACLE, SAFE_STOP | 0..4 | 5..7 unused | B2 |
| 3 | epoch | 19 | 3 | no | mode epoch | 0..7 | — | B2 |
| 4 | faults | 22 | 27 | no | fault bitmask (section 6) | bits 0..26 | — | all |
| 5 | mstate | 49 | 2 | no | IDLE, STRAIGHT, TURNING, FAULT | 0..3 | — | B2 |
| 6 | msource | 51 | 2 | no | LINE, BARCODE, AVOID, SUPER | 0..3 | — | B2 |
| 7 | cmd_active | 53 | 1 | no | flag | 0/1 | — | B2 |
| 8 | speed_l | 54 | 11 | yes | mm/s | −1023..1023 | −1024 = no encoder data | B2 |
| 9 | speed_r | 65 | 11 | yes | mm/s | −1023..1023 | −1024 = no encoder data | B2 |
| 10 | odometer_cm | 76 | 16 | no | cm | 0..65535, saturates | — | B2 |
| 11 | heading2 | 92 | 8 | no | 2° (encoder heading) | 0..179 (0..358°) | — | B2 |
| 12 | cruise10 | 100 | 6 | no | 10 mm/s | 0..63 (0..630 mm/s) | — | B2 |
| 13 | ir_black | 106 | 3 | no | bit0 L, bit1 C, bit2 R; 1 = black | 0..7 | — | B3 |
| 14 | line_pos | 109 | 3 | no | CENTER, LEFT_NEAR, LEFT_FAR, RIGHT, LOST_LEFT, LOST_RIGHT, JUNCTION | 0..6 | 7 unused | B3 |
| 15 | line_state | 112 | 2 | no | ON_LINE, SEARCHING, LOST, UNKNOWN | 0..3 | 3 = not calibrated | B3 |
| 16 | steer | 114 | 7 | yes | steer units (+ = right) | −63..63 | — | B3 |
| 17 | bc_state | 121 | 2 | no | IDLE, CAPTURING, DECODING, EXECUTING | 0..3 | — | B3 |
| 18 | bc_symbol | 123 | 6 | no | Code 39 index (§5.1.1) | 0..43 | 63 = none | B3 |
| 19 | bc_result | 129 | 2 | no | NONE, VALID, INVALID, DUPLICATE | 0..3 | — | B3 |
| 20 | nav | 131 | 2 | no | LEFT, RIGHT, STRAIGHT, U_TURN | 0..3 | valid only if nav_valid | B3 |
| 21 | nav_valid | 133 | 1 | no | flag | 0/1 | — | B3 |
| 22 | bc_valid | 134 | 6 | no | count of valid reads | mod 64 | — | B3 |
| 23 | bc_invalid | 140 | 6 | no | count of invalid reads | mod 64 | — | B3 |
| 24 | imu_status | 146 | 2 | no | INIT, CALIBRATING, OK, FAULT | 0..3 | — | B4 |
| 25 | pitch | 148 | 8 | yes | 1° (nose up +) | −90..90 | −128 = invalid | B4 |
| 26 | roll | 156 | 8 | yes | 1° (right side down +) | −90..90 | −128 = invalid | B4 |
| 27 | yaw_rate | 164 | 9 | yes | 1 °/s (left / CCW +) | −255..255 | −256 = invalid | B4 |
| 28 | heading | 173 | 9 | no | 1° (gyro-integrated) | 0..359 | 511 = invalid | B4 |
| 29 | motion_class | 182 | 2 | no | STATIONARY, MOVING, TURN_LEFT, TURN_RIGHT | 0..3 | — | B4 |
| 30 | tilt | 184 | 1 | no | flag | 0/1 | — | B4 |
| 31 | collision | 185 | 1 | no | latched until CLEAR / RESUME | 0/1 | — | B4 |
| 32 | shock | 186 | 5 | no | 0.5 g (peak horizontal impact) | 0..31 (0..15.5 g) | — | B4 |
| 33 | hump_last | 191 | 6 | no | 2 mm | 0..50 (0..100 mm) | 63 = none yet | B4 |
| 34 | hump_max | 197 | 6 | no | 2 mm | 0..50 (0..100 mm) | 63 = none yet | B4 |
| 35 | hump_count | 203 | 4 | no | count | 0..15, saturates | — | B4 |
| 36 | hump_active | 207 | 1 | no | flag | 0/1 | — | B4 |
| 37 | us_dist | 208 | 10 | no | 5 mm | 0..800 (0..4000 mm) | 1022 = no echo (clear), 1023 = invalid | B5 |
| 38 | us_result | 218 | 2 | no | OK, NO_ECHO, NO_RESPONSE, INVALID | 0..3 | — | B5 |
| 39 | avoid_state | 220 | 4 | no | IDLE, STOPPING, COARSE, FINE, PLANNING, EXECUTING, SEARCHING, ALIGNING, FAILED | 0..8 | 9..15 unused | B5 |
| 40 | avoid_side | 224 | 2 | no | NONE, LEFT, RIGHT | 0..2 | — | B5 |
| 41 | avoid_outcome | 226 | 2 | no | NONE, SUCCESS, FAILED, CLEARED | 0..3 | — | B5 |
| 42 | obst_width | 228 | 8 | no | 5 mm | 0..254 (0..1270 mm) | 255 = unknown | B5 |
| 43 | obst_bearing | 236 | 8 | yes | 1° (+ = right) | −127..127 | — | B5 |
| 44 | clear_left | 244 | 8 | no | 10 mm | 0..253; 254 = ≥ 2.54 m | 255 = unknown (never free) | B5 |
| 45 | clear_right | 252 | 8 | no | 10 mm | as clear_left | 255 = unknown | B5 |
| 46 | comm_state | 260 | 3 | no | INIT, CONNECTING_WIFI, CONNECTING_MQTT, SUBSCRIBING, ONLINE, RETRY_WAIT, RECOVERY, DISABLED | 0..7 | — | B1 |
| 47 | tlm_dropped | 263 | 8 | no | frames dropped (queues) | 0..255, saturates | — | B1 |
| 48 | rx_rejected | 271 | 8 | no | command frames rejected | 0..255, saturates | — | B1 |

Fields come from each owner's interface (motion
status words, line and barcode status words, IMU snapshot, hump word,
scan/avoid status words, obstacle profile). They are mapped in
`tlm_build.c` without changing scale, except the three profile fields
(width, clearances), which are compacted there. Integration test INT-15
checks every field at its limit and at its invalid code, from each
subsystem's pack function to the decoded server value.

#### 5.1.1 Code 39 symbol index

`0`..`9` → 0..9, `A`..`Z` → 10..35, `-` 36, `.` 37, space 38, `$` 39,
`/` 40, `+` 41, `%` 42, `*` 43. 63 = no symbol.

### 5.2 Heartbeat (type 2): 32 bits, 4-byte payload

| Field | Bits | Meaning |
|---|---|---|
| uptime_ds | 16 | 0.1 s, wraps |
| mode | 3 | vehicle mode |
| comm_state | 3 | as telemetry field 46; **online / disconnected / recovering** (FR13) follows from it |
| critical | 1 | 1 if any critical fault bit is set |
| fault_count | 5 | number of fault bits set (0..27) |
| reconnects | 4 | successful reconnections, saturates at 15 |

### 5.3 Event (type 3): 4-bit code + 32-bit data word, 5-byte payload

| Code | Event | Data word layout (LSB first) |
|---|---|---|
| 1 | MODE | old mode 3, new mode 3, epoch 3 |
| 2 | FAULTS | fault mask 27 (the new mask) |
| 3 | BARCODE | symbol 6, result 2, nav 2, nav valid 1 |
| 4 | HUMP | last height code 6, highest code 6, count 4 |
| 5 | OBSTACLE | outcome 2, fail reason 3, side 2, width 8 (5 mm), distance 10 (5 mm) |
| 6 | COLLISION | shock 5 (0.5 g) |

Fail reasons: 0 NONE, 1 PROFILE, 2 CLEARANCE, 3 MOTION, 4 LINE, 5 TIMEOUT,
6 SENSOR.

### 5.4 Command (type 4): 24 bits, 3-byte payload

| Field | Bits | Meaning |
|---|---|---|
| cmd | 3 | 0 PING, 1 STOP, 2 RESUME, 3 CLEAR, 4 SET_SPEED, 5 REQ_STATUS; 6, 7 invalid |
| arg | 10 | SET_SPEED: cruise speed 50..400 mm/s; all others: must be 0 |
| cmd_id | 8 | chosen by the server; echoed in the ACK; duplicates refused |
| reserved | 3 | must be 0 |

Commands are **supervisory requests only**. There is deliberately no
move/turn command (Week 6 §8.1: "remote commands are validated high-level
requests rather than direct motor-control instructions"). Buddy 1 maps them
onto Buddy 2's supervisory source (`mp_cmd_to_motion()`). Buddy 2 decides
whether each is safe in the current mode:

* STOP is accepted in every mode. It latches `REMOTE_STOP`, which puts the
  vehicle in Safe Stop.
* RESUME is accepted only in Safe Stop, and only when no critical
  *condition* remains. It clears latched critical *events*, then the vehicle
  re-initialises and recalibrates.
* CLEAR clears latched events and transient bits. It never leaves Safe Stop.
* SET_SPEED is refused during obstacle avoidance.
* PING and REQ_STATUS are answered by Buddy 1 itself. REQ_STATUS triggers an
  immediate telemetry frame.

### 5.5 Acknowledgement (type 5): 20 bits, 3-byte payload

| Field | Bits | Meaning |
|---|---|---|
| cmd_id | 8 | id of the command answered |
| cmd | 3 | its command code |
| result | 4 | 0 OK, 1 BAD_VERSION, 2 BAD_TYPE, 3 BAD_LENGTH, 4 BAD_FIELD, 5 BAD_ARG, 6 REJ_STATE, 7 DUPLICATE, 8 RATE, 9 BUSY |
| detail | 5 | with REJ_STATE: Buddy 2's reason (0 NONE, 1 FORMAT, 2 RANGE, 3 SOURCE, 4 MODE, 5 STALE, 6 PRIORITY, 7 QUEUE) |

## 6. Fault bitmask (27 bits, telemetry field 4, event FAULTS)

Owner of the mask: Buddy 2 (`sys_defs.h`). Severity: **D** degraded
(reported only), **C** critical condition (Safe Stop while present), **E**
critical event (Safe Stop, latched until CLEAR / RESUME).

| Bit | Name | Sev | Set by | Bit | Name | Sev | Set by |
|---|---|---|---|---|---|---|---|
| 0 | MOTOR_STALL | D | B2 | 14 | US_INVALID | D | B5 |
| 1 | ENCODER | D | B2 | 15 | SERVO_SUSPECT | D | B5 |
| 2 | MOTION_ESCALATED | E | B2 | 16 | OBSTACLE_UNAVOIDABLE | E | B5 |
| 3 | MOTOR_WDOG | E | B2 | 17 | PROFILE_INVALID | E | B5 |
| 4 | CMD_QUEUE | D | B2 | 18 | AVOID_FAILED | E | B5 |
| 5 | IR_CAL | E | B3 | 19 | RECOVERY_TIMEOUT | E | B5 |
| 6 | LINE_LOST | D | B3 | 20 | WIFI_LOSS | D | B1 |
| 7 | LINE_RECOVERY | E | B3 | 21 | MQTT_LOSS | D | B1 |
| 8 | BARCODE_INVALID | D | B3 | 22 | CMD_INVALID | D | B1 |
| 9 | BARCODE_OVERFLOW | D | B3 | 23 | TLM_OVERFLOW | D | B1 |
| 10 | IMU | C | B4 | 24 | INIT | E | B2 |
| 11 | IMU_CAL | E | B4 | 25 | TASK_COMM | E | any |
| 12 | COLLISION | E | B4 | 26 | REMOTE_STOP | E | B1→B2 |
| 13 | US_NO_RESPONSE | C | B5 | | | | |

The team decision flags `SYS_COLLISION_STOPS`, `SYS_IMU_REQUIRED` and
`SYS_US_REQUIRED` (`sys_state_logic.c`, all 1 by default) can downgrade bits
10–13 to degraded. Network faults are never critical (NFR9).

## 7. Golden frames (known-value tests)

These frames were generated from this specification by an independent
implementation: Python with its own bit packer and `binascii.crc_hqx` for
the CRC (generator in section 9). The firmware must produce them byte for
byte (S1-U03..S1-U06, run on the host and in the `TEST=1` image), and
`nodered_decoder.js` must decode them (`node nodered_decoder.js`). Both pass.

| Frame | Content | Bytes (hex) |
|---|---|---|
| Heartbeat | uptime 0x1234 (466.0 s), mode 4, comm 5, critical 1, 27 faults, 3 reconnects, seq 0x2A | `12 2A 04 34 12 EC 3D 60 E5` |
| Command | SET_SPEED 250 mm/s, id 0x5A, seq 7 | `14 07 03 D4 47 0B 59 72` |
| ACK | id 0x5A, SET_SPEED, REJ_STATE, detail MODE (4), seq 3 | `15 03 03 5A 34 02 87 52` |
| Event | HUMP: last 20 (40 mm), max 25 (50 mm), count 3, seq 9 | `13 09 05 44 65 03 00 00 9B 88` |
| Telemetry | record below, seq 0x80 | `11 80 23 EF BE 6B 69 69 69 35 00 FF 17 43 3D 4B D5 05 57 FA 7F 08 A8 05 F0 FF 7D 4A FE FE 87 FA 1F E8 0F F0 FF 00 89 17` |

Golden telemetry record. It exercises signs, maxima and every invalid code:
uptime_ds 0xBEEF, mode 3, epoch 5, faults 0x5A5A5A5, mstate 2, msource 2,
cmd_active 1, speed_l −1024, speed_r 1023, odometer 54321, heading2 179,
cruise10 20, ir_black 5, line_pos 6, line_state 1, steer −63, bc_state 3,
bc_symbol 10 ('A'), bc_result 1, nav 3, nav_valid 1, bc_valid 63,
bc_invalid 7, imu_status 2, pitch −128, roll 90, yaw_rate −256, heading 511,
motion_class 3, tilt 1, collision 0, shock 31, hump_last 20, hump_max 50,
hump_count 15, hump_active 1, us_dist 1022, us_result 1, avoid_state 8,
avoid_side 2, avoid_outcome 2, obst_width 255, obst_bearing −127,
clear_left 254, clear_right 0, comm_state 7, tlm_dropped 255,
rx_rejected 1.

## 8. Size comparison (Week 13 evidence C)

Measured by S1-U13 (`mp_tlm_json()` renders exactly the same 48 fields as
the Week 6 JSON design would; numbers from the host run, identical code on
the Pico):

| Message | Week 6 form | Bytes | Binary v1 | Bytes |
|---|---|---|---|---|
| Telemetry (golden record) | JSON object, 48 named integer fields | 697 | frame incl. header and CRC | **40** (5.7 %) |
| Heartbeat | — (not specified in Week 6) | — | frame | 9 |
| Command | — | — | frame | 8 |
| Acknowledgement | — | — | frame | 8 |
| Event | — | — | frame | 10 |

At one telemetry frame per 500 ms, the payload rate drops from about
1.4 kB/s to 80 B/s.

## 9. Golden frame generator (independent reference)

```python
import binascii
def crc(b): return binascii.crc_hqx(bytes(b), 0xFFFF)     # CRC-16/CCITT-FALSE
class Packer:
    def __init__(s): s.bits = []
    def put(s, v, n, signed=False):
        if signed: v &= (1 << n) - 1
        assert 0 <= v < (1 << n)
        s.bits += [(v >> i) & 1 for i in range(n)]           # LSB first
    def data(s):
        out = bytearray((len(s.bits) + 7) // 8)
        for k, b in enumerate(s.bits):
            if b: out[k // 8] |= 1 << (k % 8)
        return out
def frame(typ, seq, payload):
    f = bytearray([(1 << 4) | typ, seq, len(payload)]) + payload
    c = crc(f)
    return f + bytes([c & 0xFF, c >> 8])                  # CRC little-endian
p = Packer(); p.put(4, 3); p.put(250, 10); p.put(0x5A, 8); p.put(0, 3)
print(frame(4, 7, p.data()).hex(" "))                     # 14 07 03 d4 47 0b 59 72
```

## 10. Versioning rules

* Adding, removing, re-ordering or re-scaling a field means a new version
  number. Old receivers then refuse the frame instead of misreading it.
* The robot answers a command of an unknown version with nothing; the
  server sees no ACK and must not retry faster than the rate limit.
* Keep `TLM_FIELDS` (`mqtt_proto.c`), the table in section 5.1 and
  `nodered_decoder.js` in step. The firmware's compile-time size check
  catches a width change that is not reflected in `MP_TLM_BITS`.
