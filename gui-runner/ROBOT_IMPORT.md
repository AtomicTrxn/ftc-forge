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

The parser handles `fixed`, `continuous`, `revolute`, and `prismatic` joints; box, sphere, cylinder, and STL collision geometry; and binary or ASCII STL meshes. URDF units are meters, kilograms, and radians, with x forward, y left, z up. Relative STL paths and `package://` paths are resolved beside the URDF. The renderer converts those coordinates into its x forward, y up, z right scene. Drive wheel joints animate from encoder readings; chassis movement still comes from the Mecanum kinematics model rather than wheel-ground traction. Imported wheel radius and direct chassis-child wheel positions set the Mecanum dimensions when all four are found.

The chassis rigid body uses the imported fixed-link collision subtree, total link mass, and an aggregate inertia estimate that updates as moving links change position. Movable mechanism geometry is rendered and encoder tracked, but does not yet collide as separate articulated rigid bodies. For STL chassis collision, the importer runs V-HACD to create convex hulls and rejects an empty decomposition. Very dense meshes may need simplification before import.

To run the sample with the included Gradle wrapper:

```sh
cp -R gui-runner/sample-teamcode /tmp/ftc-import-example
cp /tmp/ftc-import-example/sim-urdf.config.example /tmp/ftc-import-example/sim.config
./gradlew :gui-runner:run --args='/tmp/ftc-import-example BasicMecanumOpMode'
./gradlew :gui-runner:runSimulatorApp --args='/tmp/ftc-import-example BasicMecanumOpMode'
```

## CAD export reality check

The public [Knock Out V4 Onshape assembly from FTC Team 11285](https://roboftc.github.io/robots/knockout.html) was used for a hands-on export attempt with `onshape-to-robot` 1.8.3. The exporter stopped before contacting the assembly because no Onshape API access and secret keys are configured locally. The [exporter's examples](https://github.com/Rhoban/onshape-to-robot-examples#why-do-i-get-error-403-while-using-onshape-api) also warn that public viewing alone may not grant API export rights; a team may need to copy the document to its own account. Teams must install the Python exporter, configure API credentials, select the Onshape document/assembly, use correctly named assembly mates for joints, and then add FTC transmissions and hardware XML names after export. This is a multistep import, not drag and drop.

As an independent file-format check, the importer parsed a published `onshape-to-robot` export of the Robot Soccer Kit (114 links, 113 joints) and loaded its real binary STL wheel mesh (64,594 triangles). That is evidence for parsing exporter output, not a completed export of the FTC assembly or a full FTC robot physics validation.
