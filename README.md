# FTC Forge

FTC Forge is a desktop Java simulator for testing FTC `LinearOpMode` and iterative `OpMode` code against a mocked FTC SDK. It compiles a team's `TeamCode` source, builds a hardware map from the robot configuration, and can run headlessly or in a 3D jMonkeyEngine field with a Bullet rigid-body chassis. Motor, battery, encoder, sensor, and optional URDF robot models provide a starting point for iteration before testing on the robot.

## Requirements

- JDK 17 with the `jdk.compiler` module (a full JDK, not a JRE)
- macOS for the tested 3D desktop path; other platforms may work but have not been validated here
- A team project directory containing `sim.config`, FTC robot configuration XML, motor preset JSON, and `TeamCode` Java sources

The included Gradle wrapper downloads Gradle 8.9 on its first run. From the repository root:

```sh
./gradlew test
./gradlew :gui-runner:run --args='gui-runner/sample-teamcode IterativeDriveOpMode'
./gradlew :gui-runner:run --args='gui-runner/sample-teamcode DashboardOpMode'
./gradlew :gui-runner:runSimulatorApp --args='gui-runner/sample-teamcode TurnAndResetOpMode'
```

The last command opens a 3D window; the sample turns the robot and prints IMU yaw before and after `resetYaw()`. Other renderer samples are `BasicMecanumOpMode` (forward/strafe) and `IntakeDemoOpMode` (game-piece capture). On macOS, the renderer task already sets `-XstartOnFirstThread`.

New unconfigured runs use the generic field with no pieces. Select a field and mode with `./gradlew :gui-runner:selectField --args='gui-runner/sample-teamcode FieldDriveOpMode'`. The supplied BIOBUZZ CAD supports field-only and field-with-pieces operation at the same meter-based scale as robot CAD. See [field import, selection and physical limits](gui-runner/FIELD_IMPORT.md). For the illustrative intake demo, add `--field generic --mode game-pieces --piece-set torus` to the renderer arguments.

To use your own robot, replace `gui-runner/sample-teamcode` in the commands with an absolute path or a path relative to the repository root. The second argument is the annotated OpMode's class name or display name. The headless runner accepts an optional third argument for watchdog timeout in milliseconds.

## Team project configuration

Place `sim.config` at the team project root. Paths inside it are relative to that directory:

```json
{
  "sourceRoot": "TeamCode/src/main/java",
  "robotConfig": "robot_config.xml",
  "presetMotors": "preset_motors.json",
  "extraClasspath": [],
  "imu_latency_ms": 8
}
```

`extraClasspath` may contain paths to local jars or class directories used by team code. They are added to both compilation and runtime class loading. `imu_latency_ms` accepts 0–200 and defaults to 8; yaw and yaw rate in the 3D renderer come from the physics chassis. Headless runs do not create a physics world, so their IMU remains at its initial orientation. The simulated chassis stays level, so pitch and roll remain zero.

To import custom robot geometry, add `urdf` and optionally `total_mass_kg` and `vhacd_max_hulls`. See [the import guide](gui-runner/ROBOT_IMPORT.md) for the XML/URDF pairing, a sample package, and the current CAD export constraints.

To inspect an exported CAD package before supplying hardware and collision data:

```sh
./gradlew :gui-runner:previewRobot --args='/absolute/package/urdf/robot.urdf'
```

Drag to orbit and scroll to zoom. An optional second argument saves a PNG and closes the preview. This viewer loads Onshape visual meshes, colors, and assembly frames without running physics or an OpMode.


## Current limits

The drivetrain uses capped traction impulses to approach configured Mecanum or differential wheel speed while preserving external pushes, gravity, and contact response. Imported arm/slide mechanisms use separate dynamic bodies and constrained joints; encoders follow actual joint motion, so contacts can stall them. See [the physics implementation and evidence](tasks/physics/PLAN.md).

Imported bodies use the full CAD inertia tensor in principal-axis frames. Nonadjacent robot bodies collide with each other; directly joined bodies are exempt because their CAD geometry may overlap at the joint. Each driven servo joint requires explicit torque, speed, travel, and feedback parameters. [The calibration guide](gui-runner/CALIBRATION.md) explains how to record robot telemetry, fit a profile, and load it into the simulator. Real-world accuracy remains unmeasured until a team supplies a hardware recording. The chassis stays level and drive wheels do not simulate individual rollers. The supplied native REV DUO Onshape export now runs through a [powered preparation workflow](gui-runner/REV_DUO.md) with differential drive, collision proxies, wheel bindings and a physical chain-coupled intake. Its original export remains unchanged. Opt-in [tire slip and flexible intake contact](gui-runner/CONTACT_MODELS.md) now model per-wheel friction, shaft reactions and deforming CAD paddles. CAD mass, friction, rubber properties and real-world accuracy still require measurement and calibration. An additional opt-in [compliant torus retention model](gui-runner/TORUS_RETENTION.md) now validates contact-triggered pickup, carrying, stopped retention and reverse release with the supplied CAD. Its pinch and material parameters remain unmeasured; the contact-only baseline still does not retain the illustrative torus. See [the refinement plan](tasks/physics-refinement/PLAN.md) and [application review](tasks/review/application-gap-review-and-plan.md).
