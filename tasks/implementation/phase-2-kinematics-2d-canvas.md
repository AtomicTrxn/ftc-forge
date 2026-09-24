# Task P2: Kinematics & 2D Canvas

**Type:** Implementation
**Depends on (research):** None directly — Mecanum kinematics is standard, well-documented math, not a dedicated research step in this plan.
**Depends on (prior phase):** [`tasks/implementation/phase-1-core-executor-hal.RESULTS.md`](phase-1-core-executor-hal.RESULTS.md) — read this for what the executor produces (motor power commands) and the preset config format, both of which this phase visualizes.
**Produces:**
- Code: `physics-engine/kinematics/` (Mecanum wheel solver), `gui-runner/canvas/` (2D field rendering).
- Results doc: `tasks/implementation/phase-2-kinematics-2d-canvas.RESULTS.md`
**Plan reference:** [Next-Gen FTC Robot Simulator Plan.md](../../Next-Gen%20FTC%20Robot%20Simulator%20Plan.md), § 4 Phase 2
**Estimated duration (from plan):** 2 weeks

## Context

**This phase completes the MVP checkpoint.** After this phase, a real team should be able to point the simulator at their own code (Phase 1) and watch it drive on a 2D field (this phase). This is the point to pause and get real feedback before continuing to Phase 3. Do not add non-idealities (motor curves, sensor latency) here — that's explicitly Phase 3; this phase uses ideal kinematics only.

## Objective

1. A 12x12 ft field rendered as a tile grid on a 2D canvas.
2. An ideal Mecanum wheel kinematic solver: takes each wheel's commanded power (from Phase 1's executor output) and produces robot-frame velocity/position.
3. USB gamepad integration, so a person can drive the simulated robot manually in addition to running autonomous OpModes.

## Instructions

1. Read Phase 1's RESULTS file to confirm exactly how motor power commands are exposed from the executor (function signature, units, update rate).
2. Implement the standard ideal Mecanum inverse/forward kinematics (four-wheel power → robot-frame vx/vy/omega, integrated over time into field-frame pose). Use ideal (no-slip) equations — non-ideal wheel slip is out of scope until Phase 4/5.
3. Render a 12x12 ft field (use standard FTC field tile dimensions) and the robot's pose on a 2D canvas, updating in real time as the executor drives it.
4. Wire a USB gamepad (or the Web Gamepad API, if R2 chose the web engine) so a human can drive the robot directly, independent of any loaded OpMode — useful for manual testing and demos.
5. **Validate the MVP checkpoint:** run a real sample autonomous OpMode (e.g. "drive forward 24 inches, turn 90 degrees") end-to-end and visually confirm the rendered robot's motion matches the expected path.

## Output contract

Save `tasks/implementation/phase-2-kinematics-2d-canvas.RESULTS.md` with:

- `## Summary`
- `## Kinematics Implementation` — equations used, function signature.
- `## Field Rendering` — approach, tile dimensions used.
- `## Gamepad Integration` — what was wired up.
- `## MVP Checkpoint Validation` — the sample OpMode used and confirmation the visualized motion is correct.
- `## Deviations from Research`
- `## Open Issues for Phase 3`
- `## Recommendation: Ready to Ship MVP?` — explicit yes/no with reasoning; if yes, this is the point to pause for real-team feedback per the plan.

## Definition of done

- [ ] Phase 1's RESULTS read.
- [ ] Ideal Mecanum kinematics implemented and correct (verified against a known test path).
- [ ] 12x12 ft field renders with correct proportions.
- [ ] Gamepad driving works.
- [ ] MVP checkpoint explicitly validated with a real sample OpMode.
- [ ] RESULTS.md saved at the path above.
