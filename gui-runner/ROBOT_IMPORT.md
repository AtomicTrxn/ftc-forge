# Importing a custom FTC robot

The simulator loads three complementary files from the team project directory:

1. The exported FTC robot-configuration XML names the hardware devices.
2. The existing motor preset JSON supplies each motor's SKU, gear ratio, and model constants. The FTC XML does not contain those values.
3. An optional URDF supplies geometry, links, joints, transmissions, mass, and inertia. If `urdf` is absent, the original preset chassis still runs.

Put paths in `sim.config`. See [`sample-teamcode/sim-urdf.config.example`](sample-teamcode/sim-urdf.config.example) and its paired [`robot.urdf`](sample-teamcode/robot.urdf). Copy the example config to `sim.config` in a test copy of the project. Keep the existing `robotConfig` and `presetMotors` entries.

```json
{
  "sourceRoot": "TeamCode/src/main/java",
  "robotConfig": "robot_config.xml",
  "presetMotors": "preset_motors.json",
  "urdf": "robot.urdf",
  "total_mass_kg": 11.0,
  "vhacd_max_hulls": 8,
  "imu_latency_ms": 8,
  "extraClasspath": []
}
```

`total_mass_kg` is optional. Use the robot's measured mass if CAD material assignments are inaccurate; all link masses and inertia tensor entries scale proportionally. `vhacd_max_hulls` is optional and must be 1–16. The default is 8, chosen conservatively after Phase 4 exposed unstable contacts at a much higher hull count. Inspect collisions for each imported mesh before trusting its accuracy. `imu_latency_ms` is optional (0–200, default 8) and applies to physics-driven yaw and yaw rate in the renderer.

Each movable URDF joint gets a `<transmission>` whose actuator names match names in the FTC XML. A transmission may have two actuators for a two-motor slide. `mechanicalReduction` is motor output-shaft radians per joint radian, or radians per joint meter for a prismatic joint. The importer rejects an actuator absent from the paired `HardwareMap` at load time. Motor specs remain in the preset JSON and do not move into URDF.

The parser handles `fixed`, `continuous`, `revolute`, and `prismatic` joints; box, sphere, cylinder, and STL collision geometry; and binary or ASCII STL meshes. URDF units are meters, kilograms, and radians, with x forward, y left, z up. Relative STL paths and `package://` paths are resolved beside the URDF. The renderer converts those coordinates into its x forward, y up, z right scene. Drive wheel joints animate from encoder readings; chassis movement uses bounded traction impulses derived from Mecanum wheel speed, preserving environmental contact and external pushes rather than overwriting body velocity. Wheels and their descendants remain visual and contribute mass as chassis ballast; they do not simulate individual rollers. Imported wheel radius and direct chassis-child wheel positions set the Mecanum dimensions when all four are found.

Fixed-link subtrees are welded into rigid bodies at their aggregate centers of mass. Each movable mechanism has a dynamic body connected by a limited hinge or slider constraint. Nested mechanisms collide with the floor, walls, and game pieces. Motor torque is transformed through `mechanicalReduction`, with equal reaction on the parent, and actual joint motion feeds the shaft encoders. A blocked mechanism can therefore stall while its powered motor draws current. Encoder reset changes the sensor reference without moving the body.

Mass is counted once per link. Body inertia uses every CAD tensor term and the full parallel-axis contribution, then rotates the rigid body into its principal-inertia frame for Bullet. Each dynamic subtree needs positive mass and a positive-definite inertia tensor. Revolute limits must be within `[-pi, pi]`; continuous joints may rotate freely. Optional positive URDF `limit effort` values cap actuator effort. Electrical damping is integrated implicitly, and numerical impulse caps keep high-reduction mechanisms stable. Nonadjacent robot bodies collide; directly joined bodies are exempt because their CAD geometry may overlap at the joint. The chassis remains level.

For each servo actuator in a movable URDF transmission, add measured or manufacturer-based shaft parameters to `servoPhysics` in `sim.config`:

```json
"servoPhysics": {
  "claw_servo": {
    "stall_torque_nm": 1.0,
    "no_load_speed_rad_s": 5.0,
    "travel_rad": 3.14,
    "position_gain_per_s": 10.0,
    "velocity_gain_nm_per_rad_s": 0.5,
    "deadband_rad": 0.01
  }
}
```

These values describe the servo output shaft, before any URDF `mechanicalReduction`. `travel_rad` is the shaft travel from command 0 to 1; `position_gain_per_s` and `velocity_gain_nm_per_rad_s` set the finite position and velocity response. Adjust them against observed motion. The importer rejects a driven servo without a matching entry. Servo direction and the URDF joint effort limit are respected. See [calibration](CALIBRATION.md) for the battery, motor, and drivetrain profile.

For STL collision on every body, the importer runs V-HACD to create convex hulls and rejects an empty decomposition. Very dense meshes may need simplification before import.

To run the sample with the included Gradle wrapper:

```sh
cp -R gui-runner/sample-teamcode /tmp/ftc-import-example
cp /tmp/ftc-import-example/sim-urdf.config.example /tmp/ftc-import-example/sim.config
./gradlew :gui-runner:run --args='/tmp/ftc-import-example BasicMecanumOpMode'
./gradlew :gui-runner:runSimulatorApp --args='/tmp/ftc-import-example BasicMecanumOpMode'
./gradlew :gui-runner:runSimulatorApp --args='/tmp/ftc-import-example MechanismPhysicsOpMode'
```

## CAD export reality check

The public [Knock Out V4 Onshape assembly from FTC Team 11285](https://roboftc.github.io/robots/knockout.html) was used for a hands-on export attempt with `onshape-to-robot` 1.8.3. The exporter stopped before contacting the assembly because no Onshape API access and secret keys are configured locally. The [exporter's examples](https://github.com/Rhoban/onshape-to-robot-examples#why-do-i-get-error-403-while-using-onshape-api) also warn that public viewing alone may not grant API export rights; a team may need to copy the document to its own account. Teams must install the Python exporter, configure API credentials, select the Onshape document/assembly, use correctly named assembly mates for joints, and then add FTC transmissions and hardware XML names after export. This is a multistep import, not drag and drop.

As an independent file-format check, the importer parsed a published `onshape-to-robot` export of the Robot Soccer Kit (114 links, 113 joints) and loaded its real binary STL wheel mesh (64,594 triangles). That is evidence for parsing exporter output, not a completed export of the FTC assembly or a full FTC robot physics validation.
