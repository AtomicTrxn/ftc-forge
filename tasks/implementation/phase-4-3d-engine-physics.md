# Task P4: 3D Engine & Physics

**Type:** Implementation
**Depends on (research):** [`tasks/research/02-engine-tech-stack-decision.RESULTS.md`](../research/02-engine-tech-stack-decision.RESULTS.md) — the specific physics/rendering stack chosen there (Libbulletjme + jMonkeyEngine/Minie), plus its Mecanum-drivetrain modeling constraint (drivetrain forces applied at the chassis, not through wheel-contact physics) that this phase must follow.
**Depends on (prior phase):** [`tasks/implementation/phase-3-sensor-latency-noise.RESULTS.md`](phase-3-sensor-latency-noise.RESULTS.md)
**Produces:**
- Code: `physics-engine/rigid-body/`, `gui-runner/3d-renderer/`.
- Results doc: `tasks/implementation/phase-4-3d-engine-physics.RESULTS.md`
**Plan reference:** [Next-Gen FTC Robot Simulator Plan.md](../../Next-Gen%20FTC%20Robot%20Simulator%20Plan.md), § 4 Phase 4
**Estimated duration (from plan):** 4 weeks — this is the largest single phase; per the plan's earlier analysis, treat this estimate skeptically and re-scope mid-phase if intake/collision work is running long rather than cutting corners on Phases 1-3's validated foundation.

## Context

This phase upgrades Phase 2's jME renderer (top-down orthographic) to full 3D rigid-body physics — walls, field game pieces, and mechanism (intake) interactions. It's the highest-effort, least-certain phase in the plan; the physics/rendering stack decided in R2 (Libbulletjme via Minie, inside jMonkeyEngine) is load-bearing here, and R2/R6 have since settled a modeling constraint this phase must follow: **Mecanum wheels cannot produce strafing motion through wheel-ground contact physics** — no plain rigid-body engine models the 45°-roller behavior that makes them work. Drivetrain motion stays owned by Phases 2/3's kinematics/motor model, applied to the chassis as a direct force/torque; Bullet's job here is walls, game pieces, and intake contact, not wheel traction.

## Objective

1. Rigid-body collision: field walls and game pieces, with collision geometry built via V-HACD convex decomposition of exported CAD meshes (bundled in Libbulletjme per R2) rather than raw triangle meshes.
2. 3D visualization extending Phase 2's jME scene (swap the orthographic camera for a perspective one, add depth) — not a separate renderer.
3. Wheel joints driven visually/for encoder-tracking by Phases 2/3's motor model output; chassis motion driven by that same model's force/torque applied directly to the chassis rigid body, not by simulated wheel-ground contact (see Context).
4. Intake mechanism simulation (robot mechanism interacting with loose game pieces).

## Instructions

1. Read R2's RESULTS for the exact physics/rendering stack (Libbulletjme + Minie + jMonkeyEngine) and its Mecanum-drivetrain constraint — don't re-evaluate the engine choice or attempt to make wheel contact produce strafing.
2. Model the field boundary and standard game pieces as rigid bodies. Build collision shapes from exported STL meshes using V-HACD convex decomposition (bundled in Libbulletjme) — never collide against a raw imported mesh directly.
3. Extend Phase 2's jME scene to a full 3D perspective view, reusing Phase 2/3's pose and sensor data as the source of truth, not re-deriving it. Apply chassis-level force/torque from the kinematics/motor model directly to the chassis rigid body each tick; let Bullet resolve chassis-vs-wall and chassis-vs-game-piece contact, but not wheel-vs-ground traction.
4. Implement intake mechanism simulation: define a simplified but plausible model for how a mechanism captures/holds/releases a loose game piece rigid body — this is the hardest sub-problem in this phase; if full accuracy proves too costly within the estimate, ship a simplified version (e.g. proximity-trigger capture rather than full contact dynamics) and say so explicitly in Deviations rather than silently under-delivering.
5. **Validate:** run a sample OpMode that drives into a game piece, activates an intake, and confirms the piece is captured and tracked correctly in 3D. Separately confirm strafing still works correctly now that a real physics engine is under the chassis — this is exactly the scenario the Mecanum constraint above exists to protect.

## Output contract

Save `tasks/implementation/phase-4-3d-engine-physics.RESULTS.md` with:

- `## Summary`
- `## Rigid-Body Collision Implementation` — including the V-HACD collision-shape pipeline.
- `## 3D Rendering` — how Phase 2's scene was extended, not replaced.
- `## Mecanum Drivetrain Confirmation` — evidence strafing still works correctly via the chassis-force approach, not wheel-contact physics.
- `## Intake Simulation` — the model used, and how it compares to full contact dynamics if simplified.
- `## Validation` — the sample scenario run.
- `## Deviations from Research`
- `## Open Issues for Phase 5`

## Definition of done

- [ ] R2's physics/rendering stack used as specified (Libbulletjme + Minie + jMonkeyEngine), extending Phase 2's renderer rather than replacing it.
- [ ] Collision geometry built via V-HACD from exported meshes, not raw triangle meshes.
- [ ] Field + game pieces collide correctly.
- [ ] Chassis motion (including strafing) is driven by the kinematics/motor model's applied force, confirmed *not* dependent on wheel-ground contact physics.
- [ ] 3D visualization reflects Phase 2/3's pose and sensor data.
- [ ] Intake simulation implemented (simplified model acceptable if documented).
- [ ] RESULTS.md saved at the path above.
