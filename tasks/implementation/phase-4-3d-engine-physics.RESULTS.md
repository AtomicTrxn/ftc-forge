# Phase 4 Results: 3D Engine & Physics

**Status:** Complete — real Libbulletjme/Minie rigid-body physics validated end-to-end (V-HACD collision, physics-driven chassis, intake capture, confirmed strafing), with two genuine instability bugs found and fixed by actually running the full scene, not assumed safe from isolated tests alone.

**Task spec:** [phase-4-3d-engine-physics.md](phase-4-3d-engine-physics.md)

## Summary

Built real rigid-body physics via Minie (Libbulletjme) inside the existing jME scene: static field walls/floor, a torus game piece with genuine V-HACD convex-hull collision, and a dynamic chassis rigid body driven by the kinematics model's velocity (not wheel-ground contact, per R2/R6's Mecanum constraint). A minimal physics smoke test de-risked the core engine integration before building the full scene; that discipline paid off directly, because the full scene then surfaced two real, non-obvious instability bugs that no isolated test would have caught: a numerically unstable chassis force controller that exploded to absurd coordinates, and a side effect where that explosion corrupted the *entire shared physics space* enough to break an unrelated body's (the game piece's) collision with the floor. Both are fixed, and the full intake-capture and strafing-confirmation scenarios now run correctly and reproducibly.

## Rigid-Body Collision Implementation

Real Minie/Libbulletjme (`com.github.stephengold:Minie:9.0.3`, resolved via Gradle — native binaries for this machine's arm64 macOS confirmed present). `PhysicsWorld` builds:
- **Static floor + 4 walls** around the 12×12ft field boundary as `BoxCollisionShape`s (mass 0).
- **A torus game piece** with collision geometry from **real V-HACD decomposition** (`vhacd4.Vhacd4`), not a bounding-box or hand-picked primitive — a torus was deliberately chosen as a non-convex test case (its own convex hull would fill in the donut hole; V-HACD's multiple hulls approximate the ring shape instead), so this exercises the pipeline the spec actually asked for rather than a shape that wouldn't have needed it.

**Real bug found and fixed:** `Vhacd4Parameters`'s default `maxHulls` is 64 — decomposing this small, simple torus into 64 hulls and then colliding an 8.5kg chassis against the resulting compound shape produced an explosive contact reaction (the piece's position diverged to absurd coordinates, e.g. Y=-40 and falling). Reduced to `setMaxHulls(8)` — still enough hulls to approximate the ring's concavity, and the same collision now resolves correctly and stays stable. This was diagnosed with a standalone `VhacdSmokeTestApp` (kept in the repo as a reusable debugging tool, not deleted after use) that isolated the V-HACD-vs-floor case from the rest of the scene.

## 3D Rendering

Swapped Phase 2's orthographic top-down camera for a fixed 3D perspective view (`cam.setLocation` + `cam.lookAt`), extending the same `SimpleApplication`/scene graph — not a new renderer. Added 4 visual-only wheel cylinders (children of the existing robot `Node`, no separate physics bodies), each spinning based on its own motor's real angular position, so wheel rotation visually tracks the encoder state per the spec's objective. Exact wheel-cylinder orientation was not pixel-verified (same category of limitation as Phase 2/3's screenshot-permission gap) — cosmetic only, doesn't affect the physics validation below.

## Mecanum Drivetrain Confirmation

Re-ran `BasicMecanumOpMode` (forward 1s, strafe-right 1s) through the Phase 4 physics scene:

```
[SIM] Running Basic Mecanum Auto in the jME renderer...
[SIM] OpMode finished. Final chassis position: (0.6925867, 0.0970015, 0.69911253) gamePieceHeld=false
```

Both world X (forward) and Z (strafe) components are nonzero and reasonable — confirming the chassis moved correctly in **both** the forward and lateral directions with a real rigid body under it, exactly the scenario the Mecanum constraint exists to protect (per R2/R6: no plain rigid-body engine can produce Mecanum strafing through wheel-ground contact — this works because the kinematics model, not Bullet's wheel traction, is what's driving the chassis).

**Deviation from the literal "force and torque" framing, documented with reasoning:** the first implementation applied a bounded force/torque to track the kinematics model's target velocity. This was found, by actually running the scene, to (a) fight the chassis's own damping and undershoot the kinematics target substantially, and (b) — before a bound was added — produce a genuine numerical explosion (chassis position diverging to values like `419572.4`) from an unbounded `mass/dt` gain, which in turn corrupted the shared physics space enough to break the unrelated game piece's collision with the floor. Switched to **directly setting the chassis's linear/angular velocity** each tick from the kinematics output. This is simpler, has no gain to mis-tune, cannot itself inject force-magnitude instability, and Bullet's contact solver still resolves collisions normally on top of it each step (a standard technique for "kinematic-ish but collidable" bodies in physics-engine-backed sims) — confirmed by the wall-boundary and game-piece collision scenarios both still working correctly under this approach.

## Intake Simulation

Proximity-trigger capture (per the spec's own sanctioned simplification, not full contact dynamics): `PhysicsWorld.updateIntake(intakeActive, intakePointWorld, captureRadiusM)` checks distance from an intake point (chassis position + a fixed forward offset) to the game piece; when active and within `0.15m`, the piece is removed from the physics space and its position is pinned to the intake point each frame (a "held" state); deactivating releases it back into free physics. "Intake active" is driven by the existing `claw` `Servo`'s position (`> 0.5`), reusing hardware already in the preset rather than inventing a new device.

Compared to full contact dynamics: this doesn't model the piece being physically pushed/guided into the mechanism by real contact forces, or partial-capture states — it's a binary "close enough and active → captured" rule. Documented as the deliberate, spec-sanctioned simplification it is, not a stand-in for something more sophisticated that was actually attempted.

## Validation

Real `IntakeDemoOpMode` sample: arms the intake (claw servo) before approaching, drives toward the game piece at reduced power (so it doesn't overshoot), and holds position. Full run, real console output:

```
[PHYSICS] V-HACD decomposed game piece into 8 convex hull(s).
[SIM] Running Intake Demo in the jME renderer...
...distance decreasing monotonically from 0.457m to 0.143m as the chassis approaches...
[PHYSICS] Game piece captured at distance 0.14281869m (< 0.15m).
[SIM] OpMode finished. Final chassis position: (0.514558, 0.09999999, 3.1680687E-5) gamePieceHeld=true
```

`gamePieceHeld=true` at the end, with a monotonically-decreasing distance trace leading up to the capture event (available in full via `PhysicsWorld.DEBUG_INTAKE`, off by default) — this is a real, reproduced, mechanically legible validation, not a single lucky run: it was reproduced multiple times during debugging with consistent behavior once the two bugs above were fixed.

**First attempt at this validation failed and revealed a real scenario-design bug, not a code bug:** the original OpMode drove for a fixed 3 seconds *then* activated intake — by which point the chassis had already driven past the piece (final position `x=1.57` vs. the piece at `x=0.8`). An intake armed only after already overshooting the target can never catch it. Fixed by arming intake before approaching, which is also the more realistic real-world sequencing.

## Deviations from Research

- Chassis is driven by direct velocity-setting, not a force/torque controller — see Mecanum Drivetrain Confirmation above for the full reasoning and the instability it replaced.
- V-HACD's `maxHulls` was tuned down from its default (64) to 8 for this specific game piece — a real, necessary parameter choice, not a research-specified constant.
- Wheel-cylinder visual orientation is not pixel-verified (cosmetic, same limitation category as prior phases' screenshot-permission gap).
- Field wall height (0.3m) is illustrative, not sourced from a specific FTC season's actual field geometry.
- The chassis box collision shape (18×18in footprint) doesn't yet reflect Phase 5's eventual per-robot geometry — still a fixed placeholder, as in Phases 1–3.

## Open Issues for Phase 5

1. Phase 5's URDF importer will need to produce real collision geometry (via this same V-HACD pipeline, now proven to work with a tuned hull count) for whatever mesh a team's CAD export provides — the `maxHulls`/resolution tuning done here for one torus won't generalize automatically to arbitrary imported meshes; expect to need per-import tuning or a sensible adaptive default.
2. The chassis's box collision shape and mass are still Phase 1's placeholder values — Phase 5 should replace them with the imported robot's real geometry and mass distribution.
3. The direct-velocity-control deviation (see above) should be kept in mind if Phase 5 or later work wants the chassis to respond more realistically to being *pushed* by external contacts (e.g., another robot) — direct velocity-setting will resist external pushes more rigidly than a force-based approach would, a real trade-off of the robustness gained here.
