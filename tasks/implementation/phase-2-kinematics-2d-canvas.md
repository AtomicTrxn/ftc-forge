# Task P2: Kinematics & 2D Canvas

**Type:** Implementation
**Depends on (research):** [`tasks/research/02-engine-tech-stack-decision.RESULTS.md`](../research/02-engine-tech-stack-decision.RESULTS.md) — R2 chose jMonkeyEngine + Minie as the full-project rendering/physics stack, which determines how this phase's "2D canvas" should actually be built (see Context) — Mecanum kinematics itself is standard, well-documented math and isn't a dedicated research step.
**Depends on (prior phase):** [`tasks/implementation/phase-1-core-executor-hal.RESULTS.md`](phase-1-core-executor-hal.RESULTS.md) — read this for what the executor produces (motor power commands) and the preset config format, both of which this phase visualizes.
**Produces:**
- Code: `physics-engine/kinematics/` (Mecanum wheel solver), `gui-runner/renderer/` (jMonkeyEngine scene setup with an orthographic top-down camera — the same renderer Phase 4 extends, not a separate 2D toolkit).
- Results doc: `tasks/implementation/phase-2-kinematics-2d-canvas.RESULTS.md`
**Plan reference:** [Next-Gen FTC Robot Simulator Plan.md](../../Next-Gen%20FTC%20Robot%20Simulator%20Plan.md), § 4 Phase 2
**Estimated duration (from plan):** 2 weeks

## Context

**This phase completes the MVP checkpoint.** After this phase, a real team should be able to point the simulator at their own code (Phase 1) and watch it drive on a 2D field (this phase). This is the point to pause and get real feedback before continuing to Phase 3. Do not add non-idealities (motor curves, sensor latency) here — that's explicitly Phase 3; this phase uses ideal kinematics only.

**Rendering approach, now settled by R2 (was ambiguous when this phase was first scoped):** R2 chose jMonkeyEngine + Minie for the whole project, including Phase 4's full 3D physics — not a separate lightweight 2D toolkit. "2D Canvas" in this phase's name means a **top-down orthographic camera view in jME**, not a Java2D/JavaFX canvas. Building the real renderer now, viewed from directly above, means Phase 4 *extends* this same renderer (swap the camera to a perspective/3D view, add depth) rather than replacing it outright — avoiding exactly the renderer-rewrite risk R2's own fallback-options note flagged as a downside of a JavaFX-Canvas-only approach. jME's LWJGL3 backend also provides gamepad/joystick input natively, resolving the plan's original "Jamepad" library question — no separate input library is needed.

## Objective

1. A 12x12 ft field rendered as a tile grid, viewed top-down through jME's orthographic camera (not a separate 2D canvas toolkit — see above).
2. An ideal Mecanum wheel kinematic solver: takes each wheel's commanded power (from Phase 1's executor output) and produces robot-frame velocity/position. **Structure this as a force/torque output applied to a chassis body**, even though Phase 2 has no physics engine driving it yet — Phase 4 will apply this same kinematics model's output as a force to a real rigid body (per R2's Mecanum-drivetrain finding: wheel-ground contact physics cannot produce strafing, so the kinematics model, not the physics engine, is and remains the authority on chassis motion). Getting this interface right now avoids a rework in Phase 4.
3. Gamepad integration via jME/LWJGL3's native input handling, so a person can drive the simulated robot manually in addition to running autonomous OpModes.

## Instructions

1. Read Phase 1's RESULTS file to confirm exactly how motor power commands are exposed from the executor (function signature, units, update rate).
2. Implement the standard ideal Mecanum inverse/forward kinematics (four-wheel power → robot-frame vx/vy/omega, integrated over time into field-frame pose). Use ideal (no-slip) equations — non-ideal wheel slip is out of scope until Phase 4/5. Expose this as a function producing chassis-frame velocity/force, not something baked directly into a 2D-only position update, so Phase 4 can reuse it unchanged against a real rigid body.
3. Set up jMonkeyEngine with an orthographic top-down camera over a 12x12 ft field (use standard FTC field tile dimensions), rendering the robot's pose, updating in real time as the executor drives it.
4. Wire gamepad input through jME's LWJGL3 backend so a human can drive the robot directly, independent of any loaded OpMode — useful for manual testing and demos.
5. **Validate the MVP checkpoint:** run a real sample autonomous OpMode (e.g. "drive forward 24 inches, turn 90 degrees") end-to-end and visually confirm the rendered robot's motion matches the expected path.

## Output contract

Save `tasks/implementation/phase-2-kinematics-2d-canvas.RESULTS.md` with:

- `## Summary`
- `## Kinematics Implementation` — equations used, function signature, and confirmation the output is chassis-frame force/velocity (reusable by Phase 4) rather than a 2D-only position update.
- `## Field Rendering` — jME orthographic camera setup, tile dimensions used.
- `## Gamepad Integration` — what was wired up via jME/LWJGL3.
- `## MVP Checkpoint Validation` — the sample OpMode used and confirmation the visualized motion is correct.
- `## Deviations from Research`
- `## Open Issues for Phase 3`
- `## Recommendation: Ready to Ship MVP?` — explicit yes/no with reasoning; if yes, this is the point to pause for real-team feedback per the plan.

## Definition of done

- [ ] Phase 1's RESULTS read.
- [ ] Ideal Mecanum kinematics implemented and correct (verified against a known test path), exposed as chassis-frame force/velocity output rather than a direct 2D position update.
- [ ] Renderer built in jME with an orthographic top-down camera (not a separate 2D toolkit) — confirmed reusable by Phase 4 without a rewrite.
- [ ] 12x12 ft field renders with correct proportions.
- [ ] Gamepad driving works via jME/LWJGL3.
- [ ] MVP checkpoint explicitly validated with a real sample OpMode.
- [ ] RESULTS.md saved at the path above.
