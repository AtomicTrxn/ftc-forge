# physics-engine

- **`physics.MecanumKinematics`** / **`physics.ChassisPose`** (Phase 2) — ideal Mecanum forward/inverse kinematics and field-frame pose integration. Engine-agnostic: produces a chassis-frame velocity, not a 2D-only position update, so Phase 4 reuses it unchanged against a real rigid body (per R2/R6's finding that wheel-ground contact physics can't itself produce Mecanum strafing).
- Phase 3 adds the real motor/battery non-ideality models here (per R4's corrected equations).
- Phase 4 adds Libbulletjme-backed rigid-body physics in `gui-runner` (per R2/R6).
- Physics refinement adds a torque-speed servo model and calibration fits for battery, motor friction, and planar drive response. The simulator applies these fits through an opt-in profile; see [`gui-runner/CALIBRATION.md`](../gui-runner/CALIBRATION.md).

`simcore.SimDcMotorEx` in `core-sdk-mock` uses the motor torque/current model and a shared battery rail. `gui-runner` couples imported mechanism joints to the physical motor shaft.
