# Physics follow-up plan

Baseline: shipped application review, `910ce52`.

## Goals and implementation

1. Replace direct chassis velocity writes with a force/torque controller on fixed Bullet ticks. Use an exponential response with bounded acceleration and traction, preserve vertical velocity/gravity, and leave external impulses/contact response to Bullet. Verify pushes, driving into walls, turn/strafe, and long-run stability.
2. Split imported mechanisms into dynamic rigid bodies at their aggregate centers of mass. Weld fixed subtrees, connect revolute/continuous mechanisms with hinges and prismatic mechanisms with sliders, apply joint limits, and exclude collisions within the same robot. Drive wheels remain visual: mecanum motion comes from the drivetrain constraint.
3. Apply motor-model torque through each transmission to the real joint, with equal reaction on the parent. Feed actual shaft motion back into motor encoders/current calculations. Keep encoder resets as sensor offsets, without teleporting geometry. Servo actuators use bounded joint effort.
4. Validate real Bullet behavior with headless integration tests for external pushes, walls, mechanism contact/stall, limits, encoder feedback, mass accounting, and nested mechanisms. Run the existing 3D drive/turn/intake scenarios. Record evidence and update guides.
5. Commit and ship the reviewed implementation through a PR.

## Limits to retain honestly

Mecanum traction remains a constrained planar model rather than simulated rollers. CAD inertia is represented by a diagonal aggregate approximation. Collision within the same robot is excluded to avoid unstable overlap in imported assemblies; mechanisms collide with the environment and game pieces. Real robot calibration remains dependent on team data.

## Execution and evidence

Implemented bounded planar traction impulses on Bullet ticks, preserving vertical motion. Imported mechanisms now have dynamic bodies at subtree centers of mass, hinge/slider constraints, joint limits, finite motor effort, and actual encoder feedback. Fixed bodies are welded; wheel mass remains ballast. Motor direction now has the correct signed duty cycle in battery calculations.

`./gradlew test --offline --no-daemon` passes, including real native Bullet integration checks:

- An external push retains velocity and displaces the chassis instead of being overwritten.
- A sustained wall command remains bounded through changing 60/30 Hz steps.
- A chassis initialized 2 m above ground falls under gravity and settles on the floor.
- A powered slide stalls against an obstacle, stops accumulating encoder ticks, and draws stall current. Resetting its encoder does not move it.
- Nested hinge/slider mechanisms with a rotated joint origin reach both lower and upper limits and remain connected. Link mass totals are counted once.
- Reversed motors report logical shaft encoder/velocity feedback and use the correct physical torque sign.

Real renderer checks:

- Preset forward/strafe completed at approximately `(0.689, 0.100, 0.681)` m.
- Turn/reset reported approximately 140 degrees before reset and zero afterwards.
- Intake finished with `gamePieceHeld=true`.
- The imported sample built three dynamic bodies and two mechanism joints with 11 kg total mass, and completed forward/strafe at approximately `(0.934, 0.072, 0.667)` m.
- The powered imported mechanism sample stalled against floor contact at arm `0.420` rad and slide `0.0234` m, with arm/slide encoder feedback of 95/283 ticks and slide current about 4 A.

### Bugs found during validation

1. A simple hinge's inferred reference frame gave rotated URDF origins a nonzero initial angle. Replaced it with an explicitly aligned [Minie New6Dof constraint](https://stephengold.github.io/Minie/javadoc/master/com/jme3/bullet/joints/New6Dof.html) configured as a hinge; arbitrary-frame lower/upper limit tests now pass.
2. Attaching the chassis control copied the visual node's transform over the requested Bullet start position. Chassis placement now occurs after attachment, with a nonzero-position/gravity regression test. This also corrected mechanism pivot placement relative to the chassis.

### Remaining approximations

Drive response (0.10 s), traction acceleration cap (7.8 m/s²), and yaw acceleration cap (20 rad/s²) are conservative defaults, not calibrated robot measurements. Body inertia is diagonal; the drive controller uses the initial assembly yaw-inertia estimate. Motors on physical mechanisms use CAD body inertia; unbound/drive motors still use the standalone load-inertia approximation. Servo effort is a generic bounded model. Chassis pitch/roll, self-collision, simulated wheel rollers, and real robot calibration remain outside this delivery.
