# Barcode analysis: what the supplied samples establish

Owner: Buddy 3. Code: `code39.c` (symbology), `barcode_dec.c` (capture
state machine), `drv_ir.c` (edge capture), `nav_map.c` (meanings),
`task_barcode.c` (task). Tests: `s3_unit_tests.c` S3-U01..U17.

## 1. Sources examined

| Source | What it contains | How it was measured |
|---|---|---|
| `Barcode & Line Specification.pdf` | Photo of a printed barcode beside the track line. Annotations: approx. barcode length **141 mm**, gap barcode→line **20 mm**, line width **18 mm** | Page rendered at high resolution; element run lengths measured along one horizontal scan line through the bars (dark/light threshold), x = 340..1400 px |
| `Barcode Sample.pdf`, pages 1–4 | Vector barcodes labelled "A" (pages 1–2) and "Z" (pages 3–4), two print scales | Pages rendered; run lengths measured on a horizontal line |
| `BARCODE.pptx` | Slides with barcodes labelled A, F, Z (EMF vector shapes) and W, X, Y (PNG images) | Slide XML unpacked; EMF bar geometry read, PNG run lengths measured |
| Project Briefing | Three IR sensors; the right one reads barcodes; barcodes encode navigation commands | Read |

All eleven measured element-width vectors are kept verbatim in
`s3_unit_tests.c` (`bc_vectors[]`) as test inputs.

## 2. Findings

Every sample, from all three sources, has the same structure:

* **29 elements** (bar, space, bar, …, bar) = 3 groups of 9 elements, with
  one extra narrow space between groups.
* Each group has **5 bars and 4 spaces**, and **exactly 3 of its 9 elements
  are wide**.
* There are only **two widths**, with a measured **wide:narrow ratio of
  about 3:1**: PDF 118–119 vs 39–40 px; PPTX 351 vs 117 units, 78 vs 26 px,
  66 vs 22 px, 27 vs 9 px. The photo of a real print gives 58–64 vs
  18–23 px; ink spread and blur make bars look wider and spaces narrower.
* The first and last group are identical in every sample: pattern
  `n w n n w n w n n` (`0x094`).
* The middle group differs with the printed label. "A" =
  `w n n n n w n n w` (`0x109`); "Z" = `n w w n w n n n n` (`0x0D0`); and so
  on for F, W, X, Y.

That is exactly the **Code 39** structure: 9 elements per character, 3 wide,
2 widths, `*` as start/stop (`0x094`). Each group is 6 × 1 + 3 × 3 = 15
units, so `*X*` with two 1-unit gaps is **47 narrow units**. The
specification's 141 mm then gives a **3.0 mm narrow** and **9.0 mm wide**
element, which matches the 3:1 ratio. The middle groups decode to the
printed labels, and only with the standard Code 39 table.

The symbology is therefore **derived from the samples, not assumed**. Other
common symbologies do not fit:

* **Code 128:** 6 elements per character in 4 widths.
* **EAN/UPC:** 4 widths, fixed guard patterns, digits only.
* **Interleaved 2 of 5:** characters in pairs, no `*` frame.

`code39.c` builds its table from the Code 39 rule (bar pattern × wide-space
group) rather than from a typed list. S3-U01 checks the table: 44 unique
patterns, 3 wide each, and `*`, `A` and `Z` equal to the measured patterns.

## 3. Buddy 3 navigation mapping

The supplied Buddy 3 mapping is:

| Code 39 character | Buddy 3 command |
|---|---|
| A | Turn Left |
| F | Turn Right |
| Z | U-turn |
| X | Straight |

`nav_map.c` loads these defaults at barcode-task startup. Other valid Code 39
characters are decoded and reported (telemetry fields 17–23, `EVT_BARCODE`)
but produce no motion unless explicitly mapped. S3-U15 verifies these
assignments.

* Navigation codes use 2 bits plus a separate valid bit (Tribunal 1,
  Buddy 3: "2-bit codes for up to 4 navigation commands").

Also open:

* Whether a command is executed **at once** (Week 6 statechart order
  decode → execute) or **at the next junction**. This is the switch
  `S3_NAV_AT_JUNCTION` in `subsystem3.h`, default 0 = at once.
* How far `STRAIGHT` drives through a junction: `S3_STRAIGHT_MM` = 120 mm
  `[CAL]`.

## 4. Why edge capture is needed (Tribunal 1, Version B change)

A 3 mm narrow bar under the sensor lasts **10 ms at 300 mm/s** and 7.5 ms
at 400 mm/s. The Week 6 plan polled the IR sensors every 20–50 ms, so whole
bars would be missed. That was the reason for Tribunal 1's change:

* **Primary:** the IR_RIGHT (GP27) edge interrupt, a callback of the one
  shared GPIO handler, stores a 32-bit record per edge (31-bit µs time + 1
  level bit) in a 64-entry single-producer/single-consumer ring. Overflow is
  flagged (`SYS_FLT_BARCODE_OVERFLOW`), never silent.
* **Backup:** `-DS3_CAPTURE_POLL_1MS=1` samples GP27 every 1 ms from RP2040
  TIMER alarm 3. The 10 ms kernel tick is too coarse for this. At 300 mm/s,
  1 ms is 0.3 mm, so a 3 mm bar spans about 10 samples. This variant was
  build-tested (normal and `TEST=1` images); it is not hardware-tested.

Estimated interrupt load at 400 mm/s (an engineering estimate, **to be
measured** on the car):

* Barcode: ≤ 133 edges/s.
* Encoders: 2 × 39 rising edges/s, with 20 slots per revolution `[CAL]` and
  a 65 mm wheel.
* Ultrasonic echo: 33 edges/s.

That is well under 300 interrupts/s, and each handler only timestamps. The
handler count `gpio_irq_count()` and `ir_edge_count()` let the team measure
the real rate (Tribunal 1 risk R3).

## 5. Decoding method (`barcode_dec.c`, `code39.c`)

1. **Start:** a white→black edge after at least 12 mm of white (quiet
   zone).
2. **Element widths:** each width is the time between edges × the wheel
   speed from Buddy 2's encoders (Tribunal 1: "consider vehicle speed and
   encoder distance"). Encoder ticks (about 10 mm) are coarser than a 3 mm
   bar, so distance alone cannot measure one.
3. **Glitch merge:** an interior element shorter than 0.8 mm is merged into
   its neighbours.
4. **Classification:** each 9-element group is classified by **ratio**,
   bars against bars and spaces against spaces. This makes it independent
   of print scale, speed drift, and a constant black/white bias from blur
   or threshold. The wide/narrow split must exceed 1.5:1, otherwise
   `C39_ERR_WIDTH`.
5. **Validation:** the run must be 10k − 1 elements and start and end with
   `*`, with no `*` inside, narrow inter-character gaps and valid patterns.
   It is tried forwards, then **reversed** (car driving the other way).
6. **End:** white for at least 20 mm and 2.5× the widest element. The
   capture window is a **distance** (450 mm), so the timeout adapts to speed
   (Week 6 statechart); a 3 s stall also aborts.
7. **Duplicates:** the same symbol within 600 mm is reported `DUPLICATE`,
   with no second command. Starts are ignored for 150 mm after a decode.
8. **Invalid input never produces a command** (Tribunal 1 interface map). It
   increments the invalid count, sets the degraded fault
   `BARCODE_INVALID`, and line following continues.

## 6. Evidence and remaining tests

| Evidence | Result | Kind |
|---|---|---|
| S3-U02: all 11 supplied samples (PDF A/A/Z/Z, PPTX A/F/Z/W/X/Y, photo A) | all decode to their printed label | software test, real sample geometry |
| S3-U03/U04: reversed sample; 3.5× print scale | decode | software test |
| S3-U05..U09: wrong element count, no contrast, corrupted start, wide gap, missing start/stop | rejected | software test |
| S3-U10: drive simulation, 15 trials × 100/200/300 mm/s, ±15 % speed ripple, encoder speed 5 % high, 0.6 mm blur | 45/45 correct | **simulation** |
| S3-U11..U14: duplicate, line under sensor, stall, window | as specified | simulation |
| INT-11: barcode → navigation → Buddy 2 turn/move → NORMAL | pass | simulation |

**Not yet done:** physical tests on the track (Tribunal 1 test plan: 15
consecutive correct reads at each of 3 speeds; assumption A4, barcode vs
curve). Also not yet done: checking GP27 wiring and the black level, and
measuring the edge rate.
