# physics-engine

Empty scaffold, per Phase 1's spec — intentionally not implemented yet.

- Phase 3 adds the real motor/battery non-ideality models here (per R4's corrected equations).
- Phase 4 adds Libbulletjme-backed rigid-body physics here (per R2/R6).

`simcore.SimDcMotorEx` (in `core-sdk-mock`) currently uses a placeholder linear tick
integration standing in for this module until Phase 3 lands — see Phase 1's RESULTS.md.
