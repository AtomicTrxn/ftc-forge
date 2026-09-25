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

## Current limits

The drivetrain sets target chassis velocity from mecanum wheel speed rather than simulating tire traction under external pushes. Moving URDF mechanisms animate but do not collide as independent rigid bodies. Physics and sensor calibration have not yet been checked against a real team's robot log. A real FTC-team Onshape export was attempted but could not complete without API credentials; the importer has been validated with its bundled example and a published exporter package. These points are tracked in [the application review](tasks/review/application-gap-review-and-plan.md).
