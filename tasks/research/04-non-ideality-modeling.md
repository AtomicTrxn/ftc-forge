# Task R4: Sim-to-Real Non-Ideality Modeling

**Type:** Research
**Depends on:** None — pure math, independent of the engine choice. Can run in parallel with R1–R3.
**Produces:** `tasks/research/04-non-ideality-modeling.RESULTS.md`
**Plan reference:** [Next-Gen FTC Robot Simulator Plan.md](../../Next-Gen%20FTC%20Robot%20Simulator%20Plan.md), § 3 Step 4

## Context

The simulator's core differentiator is modeling real hardware imperfections, not just ideal kinematics. This task establishes the math — motor behavior, battery sag, sensor latency — as plain, engine-agnostic, unit-testable functions. It doesn't touch the simulator's UI, rendering, or classloading; it's the physics core that Phase 3 will wire in, and that R5's calibration pass will tune against real telemetry.

## Objective

Produce validated, testable formulations for:
1. Motor speed-torque curve with back-EMF and thermal throttling.
2. Battery voltage sag under load.
3. Configurable sensor/bus read latency.

## Instructions

1. **Motor model.** Starting from the standard DC motor equation:
   $$\tau = \tau_{stall} \left(1 - \frac{\omega}{\omega_{no\_load}}\right) \cdot \frac{V_{actual}}{V_{nominal}}$$
   — implement this as a pure function taking motor spec constants (stall torque, no-load speed, nominal voltage — pull real values from goBILDA/REV motor datasheets) and current state (angular velocity, applied voltage), returning torque. Add thermal throttling as a derating factor over sustained high-current operation.
2. **Battery sag model.** Implement:
   $$V_{actual} = V_{internal} - I_{total} \cdot R_{battery}$$
   as a function of total instantaneous current draw across all simulated motors, using a representative internal resistance for typical FTC 12V battery packs.
3. **Sensor/bus latency model.** Design (spec only, or a minimal prototype) an asynchronous, thread-safe message queue that delays sensor reads by a configurable $N$ ms, modeling I2C bus latency (7–10 ms typical) and jitter.
4. **Decide sequencing:** must all three ship together, or can one be deferred? Recommended default: **ship motor curve + battery sag first** (directly affect autonomous path accuracy); **defer bus/sensor latency** to a fast-follow (smaller gameplay impact, higher implementation cost due to the threading).
5. Write unit tests for the motor and battery models against known reference points (e.g., a motor at zero load should approach `ω_no_load`; a fully-loaded stalled motor should draw current consistent with `τ_stall`).

## Decisions required

| Question | Recommended default |
|---|---|
| Must all three non-idealities ship together? | No — motor curve + battery sag first; sensor/bus latency deferred to fast-follow. |

## Output contract

Save to `tasks/research/04-non-ideality-modeling.RESULTS.md`:

- `## Summary`
- `## Motor Model` — final equations, reference motor spec constants used (cite datasheet), pseudocode/function signature.
- `## Battery Sag Model` — final equations, reference internal resistance value used, function signature.
- `## Sensor/Bus Latency Model` — queue design (even if deferred, spec it so Phase 3 can implement directly).
- `## Sequencing Decision` — what ships in Phase 3 vs. fast-follow, and why.
- `## Validation Notes` — the unit-test reference points used.
- `## Risks / Unknowns Remaining`

## Definition of done

- [ ] All three models have concrete equations and cited real-world constants (not placeholders).
- [ ] Function signatures are language/engine-agnostic (plain math, no engine-specific types).
- [ ] Sequencing decision recorded.
- [ ] RESULTS.md saved at the path above.
