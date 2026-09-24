# Task R5: Telemetry Schema & Logging Wrapper

**Type:** Research
**Depends on:** None to start the schema/wrapper design. The calibration algorithm section depends on [`tasks/research/04-non-ideality-modeling.RESULTS.md`](04-non-ideality-modeling.RESULTS.md) — read that before writing the calibration pass, since it must tune the exact parameters R4 defines.
**Produces:** `tasks/research/05-telemetry-schema-logging-wrapper.RESULTS.md`
**Plan reference:** [Next-Gen FTC Robot Simulator Plan.md](../../Next-Gen%20FTC%20Robot%20Simulator%20Plan.md), § 3 Step 5

## Context

The project's "bridges the sim-to-real gap" claim is only credible if it's measurable. This task defines the real-robot data format and a zero-effort way for teams to produce it, so data collection can start immediately — independent of whether the simulator itself is finished — and defines the calibration pass that uses that data to tune R4's models.

## Objective

1. A telemetry log file schema.
2. A drop-in logging `OpMode`/wrapper teams run once on their physical robot.
3. A calibration algorithm: replay real logs through the simulated models and auto-tune parameters to minimize error.

## Instructions

1. **Define the schema** (timestamped CSV or JSON — pick one and justify it): fields must include, at minimum, per-timestep motor power commands (per motor), encoder ticks (per motor), IMU orientation/angular velocity, battery voltage, and loop iteration timing. Specify units and sample rate.
2. **Design the logging wrapper.** It should be a small `OpMode` (or a `Telemetry`-layer shim any OpMode can adopt with one line) that teams add to their existing code, requiring no custom instrumentation. Decide how the resulting file gets off the robot: recommended default is **local storage (Control Hub internal storage or USB), plain file, drag-and-drop off** — not a network/cloud upload, since competition venue wifi is unreliable and this needs to work in the pits.
3. **Design the calibration pass** (read R4 first): given a recorded log, replay the same motor commands through R4's motor/battery models in the simulator, diff simulated vs. recorded trajectories/voltages, and auto-tune the free parameters (motor curve coefficients, battery internal resistance, I2C latency once that model exists) to minimize error. Specify the optimization approach (even a simple grid search or least-squares fit is fine for v1 — note it as such).
4. Confirm the local-storage-only decision, or override it with reasoning.

## Decisions required

| Question | Recommended default |
|---|---|
| How does the log file get off the robot? | Local storage (USB/internal), plain-file drag-and-drop. No network/cloud dependency. |

## Output contract

Save to `tasks/research/05-telemetry-schema-logging-wrapper.RESULTS.md`:

- `## Summary`
- `## Telemetry Log Schema` — exact field list, format (CSV/JSON), units, sample rate, with a worked example row/object.
- `## Logging Wrapper Design` — how a team integrates it, and how the file gets off the robot.
- `## Calibration Algorithm` — references R4's model parameters by name, describes the replay/diff/tune approach.
- `## Risks / Unknowns Remaining`

## Definition of done

- [ ] Schema is fully specified with a worked example, not just a field list.
- [ ] Wrapper integration requires no more than a one-line addition to an existing OpMode.
- [ ] Calibration algorithm explicitly ties back to R4's named parameters.
- [ ] RESULTS.md saved at the path above.
