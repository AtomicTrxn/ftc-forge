# physics-engine

- **`physics.MecanumKinematics`** / **`physics.ChassisPose`** (Phase 2) — ideal Mecanum forward/inverse kinematics and field-frame pose integration. Engine-agnostic: produces a chassis-frame velocity, not a 2D-only position update, so Phase 4 reuses it unchanged against a real rigid body (per R2/R6's finding that wheel-ground contact physics can't itself produce Mecanum strafing).
- Phase 3 adds the real motor/battery non-ideality models here (per R4's corrected equations).
- Phase 4 adds Libbulletjme-backed rigid-body physics here (per R2/R6).

`simcore.SimDcMotorEx` (in `core-sdk-mock`) still uses a placeholder linear tick integration
standing in for Phase 3's real motor model — see Phase 1's RESULTS.md.
