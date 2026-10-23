# Ultimate Claude Code Prompt — INF2004 CSC14 Full Subsystem Implementation

## 1. Your Role and Objective

Act as a **Senior Embedded Systems Engineer, Raspberry Pi Pico C Developer, Real-Time Systems Architect, and Firmware Integration Specialist**.

Your task is to inspect my existing INF2004 Embedded Systems project repository and **implement the complete functionality for all five Buddy subsystems (Buddy 1–5)**.

This is an actual university embedded systems project involving an autonomous robotic car. I do not want a theoretical design, pseudocode, or incomplete skeleton implementation.

I want functional, modular, well-structured C firmware that follows the project requirements, respects the existing professor-provided codebase, and is suitable for integration and deployment on our Raspberry Pi Pico hardware.

You must thoroughly understand the existing project before modifying anything.

**Critical restriction: You are only authorised to create, modify, or delete files within the existing `subsystems/` directory.**

Everything outside that directory has already been provided or configured by my professor and must remain unchanged.

---

## 2. Mandatory Reference Documents

You must read and analyse ALL the following documents before beginning implementation.

### Document 1 — Project Briefing

This is the original project specification provided by our professor.

It defines:
- Overall autonomous robotic car objectives.
- Responsibilities of all five buddies.
- Hardware components.
- Functional requirements.
- Expected subsystem deliverables.
- Integration and demonstration expectations.

Treat this document as the foundation of the project.

### Document 2 — Week 6 Design Review

This is our team's initial architecture and design specification, prepared based on the Project Briefing.

It includes:
- Functional and non-functional requirements.
- Hardware and GPIO pin assignments.
- RTOS task definitions and scheduling.
- Task interaction diagrams.
- System-level and subsystem statecharts.
- Communication interfaces.
- Fault handling and recovery.
- Assumptions and design decisions.

You must implement the relevant architecture described in this document unless it has been superseded by Tribunal 1.

Do not arbitrarily redesign our existing architecture.

### Document 3 — Tribunal 1

**THIS DOCUMENT IS EXTREMELY IMPORTANT.**

Tribunal 1 records the changes made to our original Week 6 architecture following the additional requirements introduced by our professor.

You must read the entire document, including:

- Version A: Pre-Tribunal AI Ideation.
- Version B: Live Updated Design Decisions.
- Version C: Final AI-Vetted Gap Check.
- Requirement changes.
- Resource and assumption reviews.
- Risk assessments.
- Interface decisions.
- Final Tribunal Summary.

The implementation must reflect **all confirmed decisions and mandatory changes**, particularly those recorded in Version B and the professor's injected requirements.

Distinguish carefully between:
1. Confirmed implementation decisions.
2. Mandatory professor requirements.
3. Proposed improvements not yet approved.
4. Open assumptions requiring physical validation.
5. Risks identified for future testing.

Do not treat every AI suggestion in Version A or Version C as an approved architectural change.

However, do not omit any confirmed decision merely because it is difficult to implement.

### Document 5 — Barcode Specification and Samples

This document and its attachments contain the actual barcode requirements and example barcode patterns.

**These specifications belong specifically to Buddy 3.**

You must inspect the barcode specification and every supplied barcode sample, including the visual patterns, rather than relying only on extracted PDF text.

Do not invent a barcode encoding standard or assume that these are ordinary retail barcodes.

Derive the decoding implementation from the supplied specifications and examples.

---

## 3. Document Priority and Conflict Resolution

Apply this precedence:

1. Professor's mandatory change requirements.
2. Confirmed Tribunal 1 decisions, especially Version B.
3. Week 6 Design Review, where not superseded.
4. Original Project Briefing.
5. Existing professor-provided source code, APIs, hardware interfaces and build constraints.

The professor-provided codebase must be respected as an implementation constraint. If a documented requirement cannot be achieved without modifying protected files, do not silently modify them.

Instead, identify the conflict, implement everything possible within `subsystems/`, and report the exact external change that would be necessary.

Before coding, produce a traceability matrix showing:

| Requirement | Source | Responsible Buddy | Implementation Plan | Status |
|---|---|---|---|---|
| Requirement ID | Document and section | Buddy 1–5 | Files/functions | Pending |

Every mandatory requirement and confirmed Tribunal 1 change must appear in this matrix.

Nothing mandatory may be silently excluded.

---

## 4. Repository Restrictions — Strictly Enforced

The existing repository contains base code supplied by our professor.

Your only permitted write location is:

`subsystems/`

You may inspect the entire repository to understand:

- Project structure.
- Existing APIs and header files.
- GPIO mappings.
- Hardware drivers.
- RTOS implementation.
- Scheduler configuration.
- Task initialization.
- Build configuration.
- Existing subsystem interfaces.

However, you MUST NOT modify any file outside `subsystems/`.

This includes, but is not limited to:

- Main application files.
- Professor-provided C files.
- Professor-provided headers.
- CMake files.
- Makefiles.
- SDK configuration.
- RTOS kernel files.
- Startup code.
- Linker scripts.
- Hardware driver libraries.
- GitHub configuration.

Do not replace the existing RTOS, scheduler, SDK, or hardware abstraction layer.

Do not create a new independent firmware project.

Do not move or restructure professor-provided code.

Implement the required functionality within the existing subsystem structure, using the professor's interfaces wherever possible.

If additional source files, private headers, test files, or documentation are necessary, create them only inside `subsystems/`.

**Important:** Do not modify external build files to register new source files. First inspect how the existing project discovers and compiles subsystem implementations. Ensure any newly created files are actually included in the existing build.

If this cannot be achieved within the permitted directory, clearly report the blocker instead of claiming successful integration.

Before and after implementation, verify that no tracked files outside `subsystems/` have been modified.

---

## 5. Programming Language and Hardware Environment

All embedded firmware must be implemented in **C**.

Target platform: Raspberry Pi Pico-based robotic car.

Use the exact Pico board variant, processor capabilities, SDK, compiler, RTOS and hardware drivers established by the repository.

The project documentation references micro-T-Kernel. Verify its actual implementation and supported APIs before using any RTOS features.

Do not assume this project uses FreeRTOS.

Do not introduce C++, Arduino frameworks, MicroPython, or another programming environment.

Follow the existing:

- GPIO configuration.
- PWM implementation.
- Interrupt handling.
- I2C communication.
- Hardware timers.
- Task creation conventions.
- RTOS queues and event mechanisms.
- Memory allocation conventions.

Firmware must be appropriate for a resource-constrained microcontroller.

Avoid unnecessary dynamic memory allocation, excessive stack usage, blocking delays, and busy-wait loops.

All timing-sensitive operations must be designed with deterministic behaviour wherever practical.

---

# 6. Buddy 1 — Wi-Fi, MQTT, Commands and Telemetry

Implement the complete Buddy 1 subsystem.

Responsibilities:

- Wi-Fi initialization and connection management.
- MQTT connection management.
- MQTT topic and payload handling.
- Telemetry collection.
- Telemetry publishing.
- Remote command reception.
- Command validation.
- Heartbeat reporting.
- Disconnection detection.
- Automatic reconnection.
- Fault reporting.

### Architecture

Follow the Week 6 architecture and confirmed Tribunal 1 decisions.

Retain the separation between:

1. Telemetry Collection.
2. Wi-Fi/MQTT Communication.

Use the existing RTOS mechanisms for task communication.

### Mandatory Behaviour

- Network operations must not interfere with real-time motion control.
- Local robot operation and safety must not depend on Wi-Fi availability.
- MQTT disconnection must not cause the robot to freeze.
- Reconnection must use bounded, non-blocking or appropriately scheduled retries.
- Telemetry queues must have defined capacity and overflow handling.
- Commands must be validated before being forwarded to the appropriate subsystem.
- Remote commands must not override higher-priority safety actions.
- Network faults must be represented using the agreed compact fault representation.

### Binary MQTT Requirement

Do not use JSON or text-based representations for application telemetry fields where the professor requires compact binary fields.

Implement a consistent binary protocol with:

- Explicit field widths.
- Field identifiers where needed.
- Defined scaling and units.
- Explicit byte order.
- Version information where appropriate.
- Payload length validation.
- Safe encoding and decoding.
- Invalid command rejection.

Use portable masks and shifts for wire serialization rather than relying on compiler-dependent C bit-field layout.

Document the MQTT binary protocol and provide known-value encoding and decoding tests.

Do not assume the external MQTT consumer already supports the new binary format.

---

# 7. Buddy 2 — Motion Control, Encoders and PID

Implement the complete Buddy 2 subsystem.

Responsibilities:

- Motor driver integration.
- PWM speed control.
- Wheel encoder acquisition.
- Speed estimation.
- Distance estimation.
- PID speed control.
- Straight-line correction.
- Encoder-based turning.
- Movement command execution.
- Motor stall detection.
- Motion fault handling.

### Critical Architecture Requirement

Buddy 2 must be the **sole owner of motor actuation**.

Other buddies must not independently write to motor PWM or direction GPIOs.

Instead, they must communicate movement requests through the established command interface.

### Mandatory Behaviour

Implement:

- Safe motor initialization.
- Encoder interrupt processing.
- Encoder count handling.
- PID update logic.
- Speed and distance calculations.
- Left and right turning.
- U-turn execution.
- Controlled stopping.
- Command prioritization.
- Motion completion and failure reporting.
- Motor stall detection based on available encoder feedback.
- Bounded fault recovery and escalation.

Preserve the confirmed Tribunal 1 decisions concerning motion ownership, command arbitration, fault handling, and recovery.

Ensure that motor stall recovery does not repeatedly cycle indefinitely between Fault and Idle states.

Implement safety precedence so that emergency stopping and relevant fault conditions override ordinary navigation commands.

### Minimum-Bit Requirement

Apply compact representation to:

- Movement commands.
- Motion states.
- Encoder-related application messages.
- Speed and distance reports.
- Fault flags.
- Inter-task messages.

Do not unnecessarily reduce measurement precision used internally by the PID controller if that would compromise control stability.

Distinguish internal computation precision from the compact representation used for stored or transmitted application data, while complying with the professor's requirements.

---

# 8. Buddy 3 — IR Line Following and Barcode Decoding

Implement the complete Buddy 3 subsystem.

This subsystem is responsible for:

- IR sensor initialization.
- IR calibration.
- Line position detection.
- Line-following decisions.
- Junction detection.
- Line-loss detection.
- Line reacquisition reporting.
- Barcode detection.
- Barcode decoding.
- Navigation command generation.

### IR Sensor Configuration

The Project Briefing specifies three IR sensors, with two used for line detection and the right-side sensor used for barcode reading.

Verify the actual GPIO assignments and physical configuration against the Week 6 design and repository.

Do not assume analog sensing if the installed hardware and design use digital sensors.

### Barcode Specification — Mandatory

The supplied barcode attachments contain example patterns and physical dimensions.

The provided track reference includes approximately:

- 141 mm barcode length.
- 20 mm separation between the barcode and the line.
- 18 mm line width.

Treat these as supplied reference dimensions and verify the precise interpretation against the diagrams and physical samples.

You must examine every supplied sample, including the examples labelled A and Z.

Implement barcode decoding based on the actual supplied patterns and documented encoding rules.

Do not assume Code 39, Code 128, EAN, or another standard without evidence.

### Barcode Decoding Requirements

Implement, as supported by the specification:

- Barcode start detection.
- Black/white transition detection.
- Edge timing or distance measurement.
- Bar-width classification.
- Pattern reconstruction.
- Symbol decoding.
- Invalid pattern rejection.
- Duplicate-reading prevention.
- Barcode completion detection.
- Navigation command generation.

Consider vehicle speed and encoder distance when decoding patterns.

The Week 6 architecture identifies potential sampling limitations. Ensure that the implementation accounts for barcode edge detection at the robot's actual operating speed.

Use an appropriate timing or edge-capture mechanism supported by the hardware and existing firmware.

Do not introduce unsupported interrupt capabilities.

If the provided samples do not establish a unique decoding rule, identify the missing information rather than inventing a mapping.

### Line Following

Implement the line-following behaviour specified in the design documents.

Buddy 3 must generate steering or navigation requests for Buddy 2 rather than directly controlling the motors.

Line following must not issue conflicting movement requests while obstacle avoidance has control.

Line-loss and line-reacquisition events must be available to Buddy 5.

### Minimum-Bit Representation

Use compact representation for:

- IR sensor states.
- Line position.
- Barcode detection states.
- Decoded symbols.
- Navigation commands.
- Line-loss events.
- Inter-task messages.

Ensure encoding and decoding preserve all required barcode distinctions.

---

# 9. Buddy 4 — IMU, Motion and Terrain Monitoring

Implement the complete Buddy 4 subsystem.

Responsibilities:

- IMU initialization.
- Sensor calibration.
- Accelerometer and gyroscope acquisition.
- Motion classification.
- Tilt detection.
- Hump detection.
- Hump height estimation.
- Maximum hump tracking.
- Collision/shock detection.
- Turn-rate reporting.
- Terrain event reporting.

### Mandatory Behaviour

Implement:

- Stationary IMU calibration.
- Sensor bias compensation.
- Appropriate filtering.
- Tilt detection.
- Hump detection.
- Hump height estimation.
- Highest hump peak tracking.
- Collision event detection.
- Relevant fault detection.
- Event communication to other subsystems.

Follow the Week 6 task architecture and Tribunal 1 decisions.

Do not invent the exact IMU chip, register map, or hardware interface.

Inspect the professor-provided drivers and hardware specifications.

### Timing

Respect the documented IMU sampling requirements, including the 10 ms task period where applicable.

Do not allow expensive processing, blocking I2C operations, or unnecessary synchronization to cause unacceptable task delays.

### Measurement Accuracy

Use appropriate units, scaling, signed ranges and numeric precision.

The Tribunal 1 discussion considered quantization trade-offs for hump measurements.

Follow the team's confirmed decision, while retaining sufficient internal calculation precision for accurate estimation.

Do not assume a particular quantization step is mandatory unless the documents confirm it.

### Minimum-Bit Representation

Apply compact representations to:

- IMU status.
- Motion classification.
- Tilt events.
- Hump measurements.
- Maximum hump records.
- Collision events.
- Fault flags.
- Shared task messages.

Document any measurement precision trade-offs.

---

# 10. Buddy 5 — Ultrasonic Scanning, Obstacle Profiling and Recovery

Implement the complete Buddy 5 subsystem.

Responsibilities:

- Ultrasonic sensor initialization.
- Servo initialization and control.
- Forward obstacle detection.
- Coarse scanning.
- Fine scanning.
- Distance measurement.
- Obstacle width estimation.
- Obstacle clearance estimation.
- Obstacle profiling.
- Avoidance decision-making.
- Avoidance command generation.
- Line reacquisition and recovery.

### Mandatory Behaviour

Implement the obstacle handling workflow defined by the Project Briefing, Week 6 architecture and confirmed Tribunal 1 decisions.

This includes:

1. Detect an obstacle.
2. Request a safe stop or reduced speed as appropriate.
3. Perform coarse ultrasonic scanning.
4. Perform finer scanning around the detected obstacle.
5. Estimate obstacle position, width and available clearance.
6. Select an appropriate avoidance action.
7. Send movement requests to Buddy 2.
8. Execute bounded avoidance and recovery.
9. Coordinate with Buddy 3 to reacquire the line.
10. Resume normal navigation only when safe.

Buddy 5 must not directly control the motors.

### Task Architecture

Preserve the intended separation between:

- Obstacle Scanning.
- Obstacle Profiling.
- Avoidance and Recovery.

Use the existing RTOS communication mechanisms.

### Fault Handling

Handle:

- Ultrasonic timeout.
- No echo.
- Invalid distance measurement.
- Servo failure or unavailable response where detectable.
- Insufficient clearance.
- Recovery timeout.
- Failure to reacquire the line.

Do not invent a successful bypass when the sensors cannot establish a safe route.

Escalate unrecoverable conditions to the agreed Safe Stop behaviour.

### Minimum-Bit Representation

Use compact representations for:

- Ultrasonic distance samples.
- Servo scan angles.
- Obstacle profiles.
- Clearance measurements.
- Avoidance states.
- Recovery states.
- Fault conditions.
- Inter-task messages.

Preserve enough distance resolution for safe obstacle avoidance.

---

# 11. System-Wide Minimum-Bit Representation

This requirement applies to every buddy.

For each relevant application field:

1. Determine its valid minimum and maximum.
2. Determine its required measurement resolution.
3. Calculate the minimum number of bits required.
4. Choose a suitable representation.
5. Implement range validation.
6. Implement encoding and decoding.
7. Verify boundary values.

Use the following principle:

`required_bits = ceil(log2(number_of_distinct_valid_values))`

Account for reserved values, invalid states, and signed representations where necessary.

Examples:

- Values 1–5 require 3 bits.
- Up to 8 states require 3 bits.
- Independent faults should use individual bits in a shared bitmask.

Use C bit-fields or explicit masks and shifts where appropriate.

Do not assume that C bit-fields automatically reduce memory usage.

Consider:

- Padding.
- Alignment.
- Compiler behaviour.
- Queue message sizes.
- Structure layout.
- Portability.
- Atomicity.

### Shared Data Safety

Any packed application data shared between tasks or between an ISR and task must be accessed safely.

Avoid unsafe read-modify-write operations.

Use the existing synchronization primitives, atomic mechanisms, or ownership-based message passing as appropriate.

Do not hold locks across blocking operations.

Keep interrupt handlers short.

### Binary Protocol Consistency

All buddies must use consistent field definitions.

For each shared field, define:

- Field name.
- Valid range.
- Bit width.
- Unit.
- Scale.
- Signedness.
- Invalid/reserved representation.
- Producer.
- Consumer.

Ensure the receiving subsystem reconstructs the correct value.

---

# 12. Full Integration Across All Five Buddies

Implement the subsystems as one coherent robotic system, not five isolated demonstrations.

The expected logical architecture is:

```text
                 Vehicle Coordination
                   Mission State
                  Command Arbitration
                          |
       +------------------+------------------+
       |                  |                  |
    Buddy 3            Buddy 2            Buddy 5
  IR + Barcode      Motion Control      Obstacle Handling
       |                  |                  |
       +------------------+------------------+
                          |
                       Buddy 4
                    IMU + Terrain
                          |
                       Buddy 1
                   Wi-Fi + MQTT
```

This is a conceptual illustration only.

The actual implementation must follow the existing RTOS architecture and confirmed design decisions. Do not introduce a new Vehicle Controller task unless already required or supported by the existing design.

### Integration Requirements

Ensure:

- Buddy 3 navigation commands reach Buddy 2.
- Buddy 5 avoidance requests reach Buddy 2.
- Buddy 4 motion and terrain events are available to relevant consumers.
- Buddy 1 collects relevant telemetry.
- Buddy 1 remote commands are validated before affecting motion.
- Buddy 2 remains the sole owner of motor actuation.
- Line following is suspended appropriately during avoidance.
- Faults propagate correctly.
- Recovery follows agreed state transitions.
- Safe Stop has appropriate priority.
- Inter-task data uses compatible binary representations.

Explicitly define command arbitration, stale-command handling, completion acknowledgements, timeouts, and subsystem ownership.

Where the Tribunal 1 document identifies an unresolved issue, use the safest compatible implementation where justified and clearly flag the decision for team confirmation.

Do not silently present unresolved team decisions as professor-approved requirements.

---

# 13. RTOS, Timing and Resource Constraints

Inspect the actual RTOS task table and scheduling design from Week 6.

Retain the documented separation of responsibilities unless Tribunal 1 explicitly changes it.

For every task, identify:

- Task name.
- Owner.
- Priority.
- Period or activation event.
- Expected execution time.
- Input/output.
- Shared resources.
- Queue usage.
- Blocking behaviour.
- Deadline or response-time requirements.

Do not invent measured timing figures.

Minimise:

- Blocking calls.
- Excessive polling.
- Interrupt overhead.
- Stack consumption.
- Heap fragmentation.
- Race conditions.
- Priority inversion.
- Unnecessary context switching.

Ensure that telemetry, ultrasonic scanning, and barcode decoding do not unnecessarily block motor control.

Check for resource conflicts involving GPIO, PWM, I2C, timers, interrupts, and task synchronization.

---

# 14. Error Handling and Safety

Implement a consistent system-wide fault-handling strategy.

Cover at least:

- Motor stall.
- Encoder fault.
- IMU fault.
- Ultrasonic no echo.
- Wi-Fi loss.
- Invalid barcode.
- Lost line.
- Obstacle recovery failure.
- Invalid remote command.
- Task communication failure.

Not every fault should automatically stop the vehicle. Follow the severity and recovery behaviour established in the design documents.

Distinguish recoverable faults from faults requiring Safe Stop.

Avoid infinite recovery loops.

Use bounded retry mechanisms where appropriate.

Ensure that a loss of network connectivity cannot override local safety decisions.

---

# 15. Testing and Verification

I expect implementation plus verification, not just code generation.

### Static and Build Verification

Check:

- C syntax.
- Header compatibility.
- Symbol definitions.
- Function signatures.
- Missing implementations.
- Duplicate symbols.
- Include dependencies.
- Compiler warnings.
- Existing build compatibility.

Use the project's existing build commands.

Do not modify build files outside `subsystems/`.

### Subsystem Tests

Provide meaningful tests for:

**Buddy 1:** Binary MQTT encoding, decoding, command validation, queue overflow, disconnection and reconnection.

**Buddy 2:** PID calculations, encoder processing, distance estimation, turning commands, stall detection and fault escalation.

**Buddy 3:** IR state interpretation, barcode decoding using provided samples, invalid patterns, line loss and navigation command generation.

**Buddy 4:** IMU calibration logic, event thresholds, hump tracking, scaling and measurement boundaries.

**Buddy 5:** Scan interpretation, obstacle width estimation, clearance decisions, avoidance transitions and recovery timeouts.

### Integration Tests

Verify communication between all five subsystems.

Include:

- Valid and invalid packed messages.
- Queue-full conditions.
- Fault propagation.
- Command arbitration.
- Navigation during obstacle detection.
- Line reacquisition.
- Network disconnection during motion.
- Simultaneous subsystem events.
- Safe Stop behaviour.

Where physical hardware is unavailable, use isolated tests or simulations only where supported by the existing architecture.

Clearly distinguish:

- Compiled successfully.
- Passed software tests.
- Simulated successfully.
- Verified on physical Pico hardware.
- Not yet tested.

Never claim physical testing occurred unless actual test evidence exists.

---

# 16. Week 13 Evidence Requirements

The professor requires evidence for the minimum-bit representation changes.

Prepare the implementation so our team can produce:

### A. Data Representation Table

For each application field, document:

- Field name.
- Subsystem.
- Valid range.
- Resolution.
- Required bits.
- Actual representation.
- Original representation where known.

### B. RAM Usage Comparison

Compare the original and revised application-data memory usage where measurable.

Use actual compiler or linker results when available.

Do not fabricate before-and-after RAM figures.

### C. Message Size Comparison

Measure and compare relevant original and packed:

- Queue messages.
- Inter-task messages.
- MQTT payloads.

Use actual structure sizes and encoded byte counts.

### D. Correctness and Performance

Provide tests or measurement procedures demonstrating:

- Correct encoding and decoding.
- No unacceptable precision loss.
- No introduced race conditions.
- Preserved subsystem behaviour.
- Acceptable task timing.
- Correct fault handling.

If baseline measurements are unavailable, explain what must be captured rather than inventing results.

Store any new implementation notes or test evidence only within `subsystems/`.

---

# 17. Code Quality Expectations

Write production-style embedded C suitable for a university engineering project.

Requirements:

- Modular implementation.
- Clear responsibilities.
- Meaningful function names.
- Consistent naming conventions.
- Minimal global mutable state.
- Well-defined interfaces.
- Safe integer arithmetic.
- Range validation.
- Explicit error handling.
- Clear units and scaling.
- Bounded buffers.
- Minimal unnecessary allocations.
- No unused placeholder functions.
- No invented hardware readings.
- No fake successful sensor operations.

Add concise comments explaining non-obvious logic.

Avoid unnecessary complexity or overengineering.

Do not replace the professor's existing implementation conventions with a completely different architecture.

---

# 18. Required Implementation Workflow

Follow these phases in order.

## Phase 1 — Repository Inspection

Inspect the repository and identify:

- Existing subsystem files.
- Available professor-provided APIs.
- RTOS implementation.
- Pico board variant.
- Hardware drivers.
- GPIO assignments.
- Build process.
- Existing integration points.

Do not modify files yet.

## Phase 2 — Requirements Analysis

Read all documents.

Create a complete requirements traceability matrix.

Identify every confirmed Tribunal 1 change.

Highlight conflicts, missing information, and hardware-dependent assumptions.

## Phase 3 — Integration Design

Define:

- Subsystem interfaces.
- Shared packed message formats.
- Fault bit allocation.
- Command priority rules.
- Task communication.
- Resource ownership.
- State transitions.
- Timing expectations.

Do not create a new architecture unnecessarily.

## Phase 4 — Implementation

Implement all five buddies within `subsystems/`.

Proceed systematically, ensuring shared interfaces are consistent.

Do not stop after implementing only one or two buddies.

## Phase 5 — Integration

Connect all subsystem interactions supported by the existing project structure.

Check that functions and message formats match.

Ensure no conflicting ownership of hardware.

## Phase 6 — Verification

Run available build and software tests.

Fix implementation defects found during testing.

Verify that files outside `subsystems/` remain unchanged.

## Phase 7 — Final Audit

Compare the final implementation against:

1. Project Briefing.
2. Week 6 Design Review.
3. Tribunal 1.
4. Minimum-Bit Change Requirement.
5. Barcode Specification.

Identify any mandatory requirements that remain unimplemented and explain the exact blocker.

---

# 19. Final Deliverables

At the end, provide the following.

### 1. Implementation Summary

Explain what was implemented for each buddy.

### 2. File Change Summary

List every created or modified file within `subsystems/`.

### 3. Requirements Compliance Matrix

For each mandatory requirement, indicate:

- Implemented.
- Partially implemented.
- Blocked.
- Not implemented.

Include source references and implementation locations.

### 4. Tribunal 1 Compliance Review

List every confirmed Tribunal 1 change and show exactly how it was addressed.

### 5. Binary Data Representation Summary

Show the compact field definitions, scaling, bit widths and serialization formats.

### 6. Test Results

Provide actual build and software test results.

Clearly identify tests requiring physical Pico hardware.

### 7. Remaining Hardware Validation

Identify calibration, sensor, timing and integration tests that our team must conduct on the physical robotic car.

### 8. Protected File Verification

Confirm whether any files outside `subsystems/` were changed.

If the implementation requires changes outside that directory, report them separately without applying them.

---

# 20. Non-Negotiable Instructions

**DO NOT:**

- Modify any file outside `subsystems/`.
- Rewrite professor-provided base code.
- Replace the existing RTOS.
- Introduce an incompatible framework.
- Ignore confirmed Tribunal 1 changes.
- Ignore the minimum-bit requirement.
- Invent barcode encoding rules.
- Invent hardware APIs or pin assignments.
- Allow multiple buddies to control motors directly.
- Introduce unsafe shared packed data access.
- Produce only pseudocode or placeholder implementations.
- Claim tests passed without executing them.
- Claim hardware validation without evidence.
- Silently skip requirements because they are difficult.

**YOU MUST:**

- Read every provided reference document.
- Inspect the existing repository.
- Follow the confirmed architecture.
- Implement all five Buddy subsystems.
- Use C for the Raspberry Pi Pico.
- Respect existing RTOS and hardware interfaces.
- Implement compact application data representations.
- Integrate subsystem communications.
- Implement appropriate safety and fault handling.
- Perform available build and software verification.
- Clearly disclose any unresolved requirements.

## Final Instruction

I want you to **actually implement the firmware in the existing repository**, not merely explain how it could be implemented.

Begin by inspecting the repository and reference documents. Produce a concise implementation plan and requirements matrix, then proceed with implementation.

You may make routine implementation decisions when they are consistent with the approved architecture.

Ask me only when a missing decision would materially affect hardware safety, contradict a mandatory requirement, or require changing protected files.

Work through all five buddies systematically. If execution or context limits prevent completion, leave an accurate checkpoint identifying completed files, remaining requirements, and verification status.

**The final objective is a fully integrated, testable five-subsystem firmware implementation, with all permitted code changes confined to `subsystems/`, following our Week 6 design and every confirmed mandatory change from Tribunal 1.**
