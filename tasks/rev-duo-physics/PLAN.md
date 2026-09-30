# Standard REV DUO powered CAD plan

## Objective
Run the user's native Onshape starter-base export with a two-motor differential drivetrain and motor-powered intake, using the existing FTC OpMode executor and Bullet physics. Preserve the original export and keep its meshes local.

## Identified gaps and implementation
1. **Four-motor Mecanum assumptions:** add validated differential drive configuration (left/right hardware names, track width, wheel radius), engine-independent kinematics, and configurable wheel ownership. Keep existing Mecanum projects working.
2. **Principal-inertia frame versus robot frame:** separate the physical body's principal axes from the chassis forward/up frame used for drive targets, IMU and intake positioning. Add a regression with rotated inertia.
3. **All-fixed CAD joints:** provide a reproducible preparation tool/profile that preserves CAD visuals, masses and tensors while assigning six wheel assemblies to their drive sides and the intake roller to a finite-torque continuous joint. Reject mismatched exports instead of silently guessing.
4. **No collisions or hardware bindings:** add explicit, documented primitive collision proxies fitted to the assembly, wheel transmissions, an intake transmission, and a paired FTC hardware/preset template. Use REV manufacturer motor constants. Expose motor names and measured-mass override for team projects.
5. **Servo-only capture:** allow a configured intake motor, physical shaft speed threshold/direction and robot-frame capture location. Verify capture and release; retain the simplified proximity capture model as an explicit limitation.
6. **No usable powered project:** prepare a private local project and launchers for autonomous verification and keyboard TeleOp, retaining the real exported meshes.

## Validation and shipping
- Unit tests for differential forward/turn/no-strafe behavior and invalid configuration.
- Native Bullet tests for custom drive names, physical intake motion/encoder feedback, principal-frame drive direction, bounded wall contact and capture/release.
- Full test suite including existing Mecanum, Dashboard/watchdog, motor and articulated-mechanism tests.
- Run the actual supplied 614-link CAD in the renderer with real FTC OpModes; record positions, yaw, intake shaft motion and capture evidence.
- Document evidence, remaining calibration limits and commands; commit, push, create and merge a PR.

## Accuracy boundaries
The supplied CAD mass is 2.6974 kg and is not a measured operating weight. Gear ratio and hardware XML remain a documented standard starter template until checked against the team's hub and motor cartridges. Six-wheel skid steering still uses bounded aggregate traction, with wheels rendered from encoders; individual tire contact/slip and deformable intake contact are not modeled. The demonstration game piece is the existing illustrative torus, not a claim about the current season's game.

## Completion

All six readiness items above are implemented and validated. Execution also exposed and fixed chassis leveling under joint impulses, collision margins blocking intake capture, held-piece release placement and reversed drive encoder feedback. See [RESULTS.md](RESULTS.md) for real CAD evidence and the remaining measurement-dependent accuracy limits.
