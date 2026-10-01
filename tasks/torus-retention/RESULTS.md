# Torus retention — implementation and validation

Validated on 2026-10-01 using the supplied standard REV DUO CAD export.

## Implemented

- Explicit opt-in `--torus-retention` preparation profile: tire/flexible contacts plus a compliant pinch and finite torus contact compression approximation.
- Loaded native paddle contact, inward electrical command and actual shaft speed gate engagement. Shaft-driven linear/angular anchor travel, bounded spring/damping reactions, passive zero-power retention, outward reverse release, cooldown and displacement breakaway.
- Piece remains a native dynamic colliding body throughout. Capture/release never assign its pose or velocity, freeze it or remove it.
- Travel work loads the actual motor shaft. Reflected motor/gear inertia and a finite native hinge motor replace the flexible axis's numerical acceleration cap. Encoders/current follow physical shaft feedback; BRAKE/FLOAT and chain phase remain physical.
- Configurable field spawn, matching the native probe and renderer; coherent seat/envelope/exit validation and rejection of nonfinite, overflowing or underflowing material constants.
- Reusable `verifyTorusRetention` hard-gate probe, retained/released FTC autonomous demonstrations, configuration guide and preparation report. Legacy and contact-only profiles remain available.

## Native evidence

The 480 Hz / 100-iteration suite passes three initial paddle phases (0.05, 0.23, 0.41 s), lateral offsets 0 / +20 / −20 mm, intake powers 1 / 0.7 / 1 and approach powers 0.1 / 0.1 / 0.12. The 0.7 case also exercises FLOAT; the others use BRAKE.

Every case requires at least 0.5 s of sustained capture, retention at every step during a >0.1 m carry and >0.25 rad turn, two seconds at zero intake power, <40 mm carried/stopped center drift, outward reverse release and continued native dynamics. Case 0 completes two capture/release cycles without resetting the piece. Seated centers settle near local x=0.23–0.24 m, height=0.136–0.140 m. Reverse releases beyond local x=0.30 m.

Latest recorded coupling peaks:

| Case | Captures / releases | Peak force | Peak torque | Peak opposing shaft load |
| --- | --- | --- | --- | --- |
| Center, repeated cycle | 2 / 2 | 2.815 N | ~0.080 Nm | 0.0349 Nm |
| +20 mm, power 0.7, FLOAT | 1 / 1 | 2.330 N | ~0.080 Nm | 0.0242 Nm |
| −20 mm | 1 / 1 | 2.419 N | ~0.080 Nm | 0.0284 Nm |

Configured caps are 8 N and 0.08 Nm; assertions allow 1e-5 for float rounding. FLOAT can allow the shaft to coast or be back-driven while the compliant grip retains the piece. BRAKE cases settle to near-zero shaft speed.

Additional hard gates pass: nearby uncontacted piece, inactive contact, reverse contact, unpowered coasting and an intentionally blocked shaft cannot engage. A 20 N external pull breaks the grip rather than creating an infinite weld; force is removed as soon as breakaway occurs, and the piece remains finite, dynamic and in the physics space.

The existing CAD drive probe also passes with retention enabled: straight x=1.2083 m, yaw=−0.00150 rad, captured torus height=0.1379 m, reverse release, and subsequent turn yaw=0.7187 rad. Assembled mass remains 2.727384 kg plus the separate 0.1 kg piece. The added shaft inertia does not add assembly mass.

## Real application and regressions

All **63 Java tests** pass: 5 SDK, 25 physics and 33 GUI/native tests. All **4 Python preparation tests** pass. New regressions cover motor load/current, reverse, chain phase, BRAKE/FLOAT, coordinate conventions, defensive configuration vectors, force/material validation, coherent seat/exit/spawn geometry and explicit preparation/source preservation.

Actual FTC `RevDuoRetentionAuto` completes in jME with `gamePieceContained=true`, `state=SEATED`, one acquisition, zero releases and the intake stopped. Latest final chassis is (0.3867, 0.0587, 0.0407) m, yaw 0.4296 rad; piece is (0.6010, 0.1398, −0.0625) m. Coupling peaks are 2.264 N and 0.0596 Nm. The inspected screenshot shows the tilted torus in the intake at approximately 20 FPS.

`RevDuoRetentionReleaseAuto` also completes in the renderer with `gamePieceContained=false`, `state=FREE`, one acquisition and one release. Its inspected screenshot shows the ring outside the robot. Recorded final piece is (0.6915, 0.0334, −0.1292) m.

The legacy powered-CAD probe passes proximity capture/release and differential turning (yaw 1.2035 rad). Contact-only CAD passes its physical contact, mass, straight-drive and turn gates; it still reports no torus retention, as expected. Dashboard completes normally. HungOpMode is marked DEAD by the watchdog, with every motor power zero afterward.

## Failures exposed during development

1. **Rigid ring/ramp edge blockage.** Traced Bullet contacts show a mostly forward/downward reaction at the tilted ramp's thin front edge, opposing inward paddle motion. Increasing the torus decomposition from 8 to 16 hulls did not solve it; the shipped shape remains 8 hulls.
2. **A spring-only grip passed once but failed phase/repeat cases.** Fully rigid torus/core contacts can wedge the ring beneath the lower roller. Finite torus contact stiffness is necessary in this explicit compression approximation. The final native suite validates phase, offset and repeated-cycle cases rather than accepting the first successful capture.
3. **Light-core motor response.** Applying pre-solve impulses to the tiny remaining rigid core required a numerical acceleration cap. Native motor constraints and explicit reflected shaft inertia provide stable finite effort and load feedback. The reflected inertia is unmeasured and requires principal-axis alignment.
4. **Equivalent torus normals after release.** A released torus can flip its normal while representing the same physical plane. Acquisition now chooses a consistent downward normal before following the configured intake tilt; passive seating treats equivalent normals consistently.
5. **Probe/renderer spawn mismatch.** The old renderer spawned at x=0.8 m while the slow native pickup scenario started at x=0.42 m. An explicit profile spawn makes the real autonomous and probe test the same approach geometry.

## Limits and reproduction

This validates **reliable behavior of the declared approximation**, not measured robot accuracy. Torus compression allows collision overlap and is not rendered as mesh deformation. Pinch geometry, material constants, holding force, reflected inertia and operating mass remain unmeasured. Full soft-body dynamics, arbitrary imported intake layouts, actual-season game pieces, tire load transfer, tip-over, detailed omni rollers, chain elasticity and gearbox losses remain outside this baseline. Heavy flexible stops can overshoot (the faster drive probe peaked at 1.857 rad despite its iterative 1.2 rad stop).

```sh
python3 tools/prepare_rev_duo.py /absolute/source.urdf /absolute/retention-project --torus-retention
./gradlew test --offline --no-daemon
python3 -m unittest discover -s tools -p 'test_*.py'
./gradlew :gui-runner:verifyTorusRetention --args='/absolute/retention-project'
./gradlew :gui-runner:runSimulatorApp --args='/absolute/retention-project RevDuoRetentionAuto /absolute/retained.png'
./gradlew :gui-runner:runSimulatorApp --args='/absolute/retention-project RevDuoRetentionReleaseAuto /absolute/released.png'
```

Source CAD remains unchanged: SHA-256 `36b1106071e8d9e86b6866c9260fe65cc7e3923ea1a1e51376e1391f141d5195`, 614 links and 750 visuals preserved. Private CAD, generated projects, development diagnostics, final native/test/renderer logs and screenshots remain under local `.local/torus-retention/` and `.local/robots/rev-duo-starter/`, excluded from Git.

See [the implementation plan](PLAN.md) and [configuration guide](../../gui-runner/TORUS_RETENTION.md).
