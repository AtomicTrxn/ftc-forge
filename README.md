# FTC Forge

FTC Forge is a desktop Java simulator for testing FTC `LinearOpMode` and iterative `OpMode` code against a mocked FTC SDK. It compiles a team's `TeamCode` source, builds a hardware map from the robot configuration, and can run headlessly or in a 3D jMonkeyEngine field with a Bullet rigid-body chassis. Motor, battery, encoder, sensor, and optional URDF robot models provide a starting point for iteration before testing on the robot.

Run reusable synthetic CAD and native physics scenarios with
`./gradlew :gui-runner:validateSimulator`. JSON and Markdown reports, fixture packages and
isolated settings are retained for each run. See [simulator validation](gui-runner/SIMULATOR_VALIDATION.md).
Add `--args='--matrix'` to check documented mass, wheel dimensions, grip, gearing and timestep
ranges with native force, drift, stability and convergence bounds.

Both 3D windows offer [live physics diagnostics](gui-runner/PHYSICS_DIAGNOSTICS.md): **D**
toggles native contact/load and drive-force vectors, wheel grip/slip, motor effort/current,
and mechanism travel/limits; **P** saves a read-only snapshot. **C** still shows collision shapes.

Library compatibility: hubs are `LynxModule`s, and goBILDA Pinpoint and SparkFun OTOS devices report the
physics pose. [`gui-runner/COMPATIBILITY.md`](gui-runner/COMPATIBILITY.md) says exactly what the Road Runner- and
Pedro-style fixtures prove; [`core-sdk-mock/SDK_COVERAGE.md`](core-sdk-mock/SDK_COVERAGE.md) lists the stub surface, and
compile errors for unsupported APIs now explain themselves.

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

Prepare new robot and field URDF/STL ZIPs with `./gradlew :gui-runner:prepareModels`. Guided setup supplies instructions, size/orientation checks, migration choices, collision review, resumable progress and a scene/project handoff. Advanced settings, live native collision preview and reusable portable profiles remain available. Identical CAD reuses its reviewed tuning; changed CAD requires review. See [model preparation and portable settings](gui-runner/MODEL_PREPARATION.md).

The [robot measurement guide](gui-runner/ROBOT_MEASUREMENTS.md) adds operating weight, wheel diameter/spacing and motor-binding instructions to guided setup. Selected measurements use kg/lb or mm/cm/in and are saved with the robot profile; changed settings require renewed model review.

Guided robot setup also offers a [short motion demo](gui-runner/ROBOT_MOTION_DEMO.md): differential/tank forward, reverse and turns; configured Mecanum adds strafing. Powered mechanisms move separately through native physics. Assess each observed movement, follow a selectable correction path and retest only that movement/mechanism. Explicit feedback travels with exported robot settings; changed CAD/settings/hardware require fresh review.

Fresh robot imports use [native wheel support and contact settings](gui-runner/DRIVE_CONTACTS.md): traction requires supported wheel collision shapes and respects floor grip, while belly scraping and wall friction remain active. The guide exposes saved tuning and contact diagnostics; existing reviewed models retain their previous mode until explicitly changed.

**Wheel grip…** in Physics assumptions adds independent, portable friction settings for configured drive wheels. Belly friction stays separate; changed CAD remaps compatible joints or offers replacement/removal choices. Motion results show simulated effective grip. Measure the actual tread/field pair before treating these assumptions as calibrated.

**Measure wheel grip…** provides a [guided force/load calibration](gui-runner/MEASURED_GRIP.md) with units, CSV exchange, independent validation trials and saved quality criteria. Passing measurements and their surface context travel with exported profiles and compatible CAD migration; applying them requires renewed collision and motion review.

For configured differential tires, **Measure tire slip…** adds [steady longitudinal curve fitting](gui-runner/MEASURED_TIRE_SLIP.md) from independent speed/load/force CSV trials. It checks whether the data identify stiffness, peak/sliding grip and transition speed, validates reserved trials and preserves the evidence with portable settings. Lateral behavior and shaft inertia remain explicit assumptions.

Imported robot collision coverage is checked per rigid body before physics starts. The selector's **Review collisions** action shows actual native shapes for the selected robot and field; C toggles collision shapes and V toggles CAD visibility. Save a coverage report with `./gradlew :gui-runner:auditCollisions --args='/absolute/robotProject /absolute/report.json'`. See [collision review](gui-runner/COLLISION_REVIEW.md).

To use your own robot, replace `gui-runner/sample-teamcode` in the commands with an absolute path or a path relative to the repository root. The second argument is the annotated OpMode's class name or display name. The headless runner accepts an optional third argument for watchdog timeout in milliseconds.

## Team project configuration

Place `sim.config` at the team project root. Paths inside it are relative to that directory:

```json
{
  "sourceRoot": "TeamCode/src/main/java",
  "robotConfig": "robot_config.xml",
  "presetMotors": "preset_motors.json",
  "extraClasspath": [],
  "imu_latency_ms": 8,
  "encoder_latency_ms": 8,
  "battery": { "internal_voltage_v": 12.6, "internal_resistance_ohm": 0.15 }
}
```

`extraClasspath` may contain paths to local jars or class directories used by team code. They are added to both compilation and runtime class loading. `imu_latency_ms` and `encoder_latency_ms` accept 0–200 and default to 8 (see [battery sag and bus latency](gui-runner/POWER_AND_BUS.md), including what is not modeled); yaw and yaw rate in the 3D renderer come from the physics chassis. Headless runs do not create a physics world, so their IMU remains at its initial orientation. The simulated chassis stays level, so pitch and roll remain zero.

To import custom robot geometry, add `urdf` and optionally `total_mass_kg` and `vhacd_max_hulls`. See [the import guide](gui-runner/ROBOT_IMPORT.md) for the XML/URDF pairing, a sample package, and the current CAD export constraints.

To inspect an exported CAD package before supplying hardware and collision data:

```sh
./gradlew :gui-runner:previewRobot --args='/absolute/package/urdf/robot.urdf'
```

Drag to orbit and scroll to zoom. An optional second argument saves a PNG and closes the preview. This viewer loads Onshape visual meshes, colors, and assembly frames without running physics or an OpMode.


## Current limits

The drivetrain uses capped traction impulses to approach configured Mecanum or differential wheel speed while preserving external pushes, gravity, and contact response. Imported arm/slide mechanisms use separate dynamic bodies and constrained joints; encoders follow actual joint motion, so contacts can stall them. See [the physics implementation and evidence](tasks/physics/PLAN.md).

Imported bodies use the full CAD inertia tensor in principal-axis frames. Nonadjacent robot bodies collide with each other; directly joined bodies are exempt because their CAD geometry may overlap at the joint. Each driven servo joint requires explicit torque, speed, travel, and feedback parameters. [The calibration guide](gui-runner/CALIBRATION.md) explains how to record robot telemetry, fit a profile, and load it into the simulator. Real-world accuracy remains unmeasured until a team supplies a hardware recording. The chassis stays level and drive wheels do not simulate individual rollers. The supplied native REV DUO Onshape export now runs through a [powered preparation workflow](gui-runner/REV_DUO.md) with differential drive, collision proxies, wheel bindings and a physical chain-coupled intake. Its original export remains unchanged. Opt-in [tire slip and flexible intake contact](gui-runner/CONTACT_MODELS.md) now model per-wheel friction, shaft reactions and deforming CAD paddles. CAD mass, friction, rubber properties and real-world accuracy still require measurement and calibration. An additional opt-in [compliant torus retention model](gui-runner/TORUS_RETENTION.md) now validates contact-triggered pickup, carrying, stopped retention and reverse release with the supplied CAD. Its pinch and material parameters remain unmeasured; the contact-only baseline still does not retain the illustrative torus. See [the refinement plan](tasks/physics-refinement/PLAN.md) and [application review](tasks/review/application-gap-review-and-plan.md).
