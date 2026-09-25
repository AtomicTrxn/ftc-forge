# Task P3: Sensor Latency & Noise

**Type:** Implementation
**Depends on (research):**
- [`tasks/research/04-non-ideality-modeling.RESULTS.md`](../research/04-non-ideality-modeling.RESULTS.md) — the motor/battery/sensor-latency equations to implement.
- [`tasks/research/05-telemetry-schema-logging-wrapper.RESULTS.md`](../research/05-telemetry-schema-logging-wrapper.RESULTS.md) — the log schema and calibration pass to build the first version of.
**Depends on (prior phase):** [`tasks/implementation/phase-2-kinematics-2d-canvas.RESULTS.md`](phase-2-kinematics-2d-canvas.RESULTS.md)
**Produces:**
- Code: `physics-engine/motor-model/`, `physics-engine/battery/`, `physics-engine/sensor-ring-buffer/` (deterministic per-sensor latency, per R4's revised design — reassess whether it ships here rather than deferring, see Instructions), `gui-runner/calibration/` (log import + staged calibration pass).
- Results doc: `tasks/implementation/phase-3-sensor-latency-noise.RESULTS.md`
**Plan reference:** [Next-Gen FTC Robot Simulator Plan.md](../../Next-Gen%20FTC%20Robot%20Simulator%20Plan.md), § 4 Phase 3
**Estimated duration (from plan):** 3 weeks

## Context

This is the first phase past the MVP checkpoint — confirm Phase 2's "Recommendation: Ready to Ship MVP?" was acted on and, ideally, real feedback was gathered before starting this. This phase makes the simulation non-ideal: real motor behavior, real battery sag, and (if R4 didn't defer it) sensor/bus latency — plus the first working version of the telemetry-based calibration loop.

## Objective

1. Deterministic, per-sensor ring-buffer IMU/encoder latency emulation per R4's model (revised from an earlier threaded/wall-clock design — see Instructions on whether this ships here or is deferred).
2. Battery sag engine per R4's **corrected, closed-form** multi-motor model, driven by total simulated current draw.
3. Motor model (corrected torque/current equations, not the plan's original physically-wrong one — see R4) plus `DcMotorEx` run-mode behavior (`RUN_USING_ENCODER`'s onboard velocity PIDF, `RUN_TO_POSITION`), replacing Phase 2's ideal kinematics with realistic torque/speed behavior.
4. First working pass of R5's **staged** calibration flow: import a real telemetry log, replay it through the sim, diff against recorded values.

## Instructions

1. Read R4's RESULTS in full — implement its **corrected** motor model (`τ = τ_stall(V/V_nom − ω/ω_nl)`, not the plan's original equation, which R4's amendment explains is physically wrong) and closed-form battery sag model exactly as specified (constants, equations). Also implement `DcMotorEx`'s `RUN_USING_ENCODER` (onboard velocity-PIDF target, not a direct voltage passthrough) and `RUN_TO_POSITION` behavior per R4's addendum — these were flagged as unaddressed by the original research pass and are real gaps otherwise.
2. **Sensor/bus latency:** R4's redesigned deterministic ring-buffer approach (sim-clock-based, single-threaded) is cheaper to implement than the original threaded `DelayQueue` design that motivated deferring it — R4 explicitly flags this as worth reassessing. Default is still deferred per R4's original sequencing decision, but consider shipping it here if the ring buffer proves as cheap as expected; note whichever call you make and why in this phase's RESULTS.
3. Read R5's RESULTS — implement the telemetry log schema (including its metadata block: per-motor SKU/ratio, run mode, battery ID, bulk-caching mode) as an import format, and build the **staged** calibration pass it specifies: (a) closed-form linear fit of `R_battery`/`V_internal` from recorded current/voltage, (b) one-step-ahead motor/drivetrain friction fit resetting to recorded state each row (not open-loop replay over a whole match), (c) thermal throttling only if long-run residuals justify it. A single undifferentiated grid search over all parameters at once is exactly what R5 found doesn't work — don't fall back to that.
4. Wire the motor model into Phase 2's kinematics so that commanded power now produces realistic torque/speed rather than an idealized instant response — Phase 2 should already expose kinematics as chassis-frame force/velocity, so this is substitution, not a rewrite.
5. Wire the battery sag model so total current draw across all active motors reduces effective voltage, feeding back into motor torque, using R4's closed-form multi-motor solve (no per-tick iteration needed).
6. The staged calibration's motor/drivetrain fit step needs a chassis mass/wheel-radius estimate to convert torque to expected velocity — R5 flagged this as unspecified; source it from Phase 1's preset config (which now carries a per-motor SKU/ratio field) rather than inventing a new input.
7. **Validate calibration:** produce (or obtain) a sample real-robot telemetry log in R5's schema, run it through the staged calibration pass, and confirm each stage converges to a reasonable parameter fit rather than diverging or erroring.

## Output contract

Save `tasks/implementation/phase-3-sensor-latency-noise.RESULTS.md` with:

- `## Summary`
- `## Motor Model Implementation` — including which `DcMotorEx` run modes are backed.
- `## Battery Sag Implementation`
- `## Sensor Latency Implementation` — shipped here or deferred; either way, the reasoning (per the reassessment in Instructions).
- `## Calibration Pass` — how the staged fit was implemented, and the result of running it against a sample log.
- `## Drivetrain Model Inputs Used` — the chassis mass/wheel-radius estimate the calibration fit needed, and where it came from.
- `## Deviations from Research`
- `## Open Issues for Phase 4`

## Definition of done

- [ ] R4 and R5 RESULTS read and implemented as specified (or deviations explicitly justified).
- [ ] Motor model uses the corrected torque/current equations (verified against R4's distinguishing unit test: `V=0.5·V_nom, ω=0.5·ω_nl` → `τ=0`), not the plan's original equation.
- [ ] `RUN_USING_ENCODER` and `RUN_TO_POSITION` behave as onboard-controller modes, not direct voltage passthroughs.
- [ ] Battery sag uses the closed-form multi-motor solve and affects motor behavior under load.
- [ ] Sensor latency shipped-or-deferred decision explicitly recorded, not silently defaulted.
- [ ] Calibration pass runs its staged fit (not a single undifferentiated grid search) against at least one sample telemetry log with a sane result.
- [ ] RESULTS.md saved at the path above.
