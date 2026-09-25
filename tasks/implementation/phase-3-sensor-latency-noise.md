# Task P3: Sensor Latency & Noise

**Type:** Implementation
**Depends on (research):**
- [`tasks/research/04-non-ideality-modeling.RESULTS.md`](../research/04-non-ideality-modeling.RESULTS.md) — the motor/battery/sensor-latency equations to implement.
- [`tasks/research/05-telemetry-schema-logging-wrapper.RESULTS.md`](../research/05-telemetry-schema-logging-wrapper.RESULTS.md) — the log schema and calibration pass to build the first version of.
**Depends on (prior phase):** [`tasks/implementation/phase-2-kinematics-2d-canvas.RESULTS.md`](phase-2-kinematics-2d-canvas.RESULTS.md)
**Produces:**
- Code: `physics-engine/motor-model/`, `physics-engine/battery/`, `physics-engine/sensor-queue/` (if not deferred per R4), `gui-runner/calibration/` (log import + calibration pass).
- Results doc: `tasks/implementation/phase-3-sensor-latency-noise.RESULTS.md`
**Plan reference:** [Next-Gen FTC Robot Simulator Plan.md](../../Next-Gen%20FTC%20Robot%20Simulator%20Plan.md), § 4 Phase 3
**Estimated duration (from plan):** 3 weeks

## Context

This is the first phase past the MVP checkpoint — confirm Phase 2's "Recommendation: Ready to Ship MVP?" was acted on and, ideally, real feedback was gathered before starting this. This phase makes the simulation non-ideal: real motor behavior, real battery sag, and (if R4 didn't defer it) sensor/bus latency — plus the first working version of the telemetry-based calibration loop.

## Objective

1. Threaded IMU/encoder emulation reflecting R4's sensor latency model (if not deferred).
2. Battery sag engine per R4's model, driven by total simulated current draw.
3. Motor model (speed-torque + back-EMF + thermal) per R4, replacing Phase 2's ideal kinematics with realistic torque/speed behavior.
4. First working pass of R5's calibration flow: import a real telemetry log, replay it through the sim, diff against recorded values.

## Instructions

1. Read R4's RESULTS in full — implement its motor model and battery sag model exactly as specified (constants, equations). If R4 deferred sensor/bus latency, implement it anyway if time allows, but it's not required for this phase per R4's own sequencing decision — check what it says and follow it.
2. Read R5's RESULTS — implement the telemetry log schema as an import format, and build the calibration pass it specifies (even a simple least-squares/grid-search fit, as R5 should have scoped).
3. Wire the motor model into Phase 2's kinematics so that commanded power now produces realistic torque/speed rather than an idealized instant response.
4. Wire the battery sag model so total current draw across all active motors reduces effective voltage, feeding back into motor torque.
5. If sensor latency is in scope for this phase: implement the deterministic per-sensor ring buffer R4 spec'd (revised from an earlier wall-clock `DelayQueue` design specifically because it must run on the simulator's own clock, not real time), and route IMU/encoder reads through it with configurable delay.
6. **Validate calibration:** produce (or obtain) a sample real-robot telemetry log in R5's schema, run it through the calibration pass, and confirm it converges to a reasonable parameter fit rather than diverging or erroring.

## Output contract

Save `tasks/implementation/phase-3-sensor-latency-noise.RESULTS.md` with:

- `## Summary`
- `## Motor Model Implementation`
- `## Battery Sag Implementation`
- `## Sensor Latency Implementation` — or explicit note that it was deferred per R4, and why that's still correct.
- `## Calibration Pass` — how it was implemented, and the result of running it against a sample log.
- `## Deviations from Research`
- `## Open Issues for Phase 4`

## Definition of done

- [ ] R4 and R5 RESULTS read and implemented as specified (or deviations explicitly justified).
- [ ] Motor model replaces Phase 2's ideal response.
- [ ] Battery sag affects motor behavior under load.
- [ ] Calibration pass runs against at least one sample telemetry log with a sane result.
- [ ] RESULTS.md saved at the path above.
