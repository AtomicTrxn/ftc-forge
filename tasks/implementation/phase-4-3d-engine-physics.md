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

This phase upgrades the 2D canvas simulation to full 3D rigid-body physics — walls, field game pieces, and mechanism (intake) interactions. It's the highest-effort, least-certain phase in the plan; the physics engine binding decided in R2 is load-bearing here.

## Objective

1. Rigid-body collision: field walls and game pieces.
2. 3D visualization replacing/extending the 2D canvas.
3. Intake mechanism simulation (robot mechanism interacting with loose game pieces).

## Instructions

1. Read R2's RESULTS for the exact physics engine binding to use, and its rationale — don't re-evaluate the engine choice here.
2. Model the field boundary and standard game pieces as rigid bodies with realistic mass/collision geometry.
3. Extend the renderer to 3D (Three.js if R2 chose Web, or the equivalent 3D layer for the chosen desktop engine), reusing Phase 2/3's pose and sensor data as the source of truth, not re-deriving it.
4. Implement intake mechanism simulation: define a simplified but plausible model for how a mechanism captures/holds/releases a loose game piece rigid body — this is the hardest sub-problem in this phase; if full accuracy proves too costly within the estimate, ship a simplified version (e.g. proximity-trigger capture rather than full contact dynamics) and say so explicitly in Deviations rather than silently under-delivering.
5. **Validate:** run a sample OpMode that drives into a game piece, activates an intake, and confirms the piece is captured and tracked correctly in 3D.

## Output contract

Save `tasks/implementation/phase-4-3d-engine-physics.RESULTS.md` with:

- `## Summary`
- `## Rigid-Body Collision Implementation`
- `## 3D Rendering`
- `## Intake Simulation` — the model used, and how it compares to full contact dynamics if simplified.
- `## Validation` — the sample scenario run.
- `## Deviations from Research`
- `## Open Issues for Phase 5`

## Definition of done

- [ ] R2's physics engine binding used as specified.
- [ ] Field + game pieces collide correctly.
- [ ] 3D visualization reflects Phase 2/3's pose and sensor data.
- [ ] Intake simulation implemented (simplified model acceptable if documented).
- [ ] RESULTS.md saved at the path above.
