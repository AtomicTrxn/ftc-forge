# Calibrating FTC Forge against a robot

The simulator's default battery, motor friction, and drive-response values are estimates. A calibration profile replaces them for one team project. The process needs measurements from the physical robot; synthetic tests verify the fitting code but cannot establish real-world accuracy.

Start with the [robot measurement guide](ROBOT_MEASUREMENTS.md): weigh the operating robot, measure wheel diameter/center spacing, and verify motor bindings. **Measure robot and verify motors…** is available from guided robot Physics assumptions and Finish. It saves manual values with the reusable robot profile; recording-derived response fitting below is a separate step.

## Record telemetry

For material grip, use [Measure wheel grip…](MEASURED_GRIP.md) in guided Physics
assumptions. It fits restrained-wheel sliding-onset force/load trials with independent
validation and portable evidence. Motor and free-drive telemetry fitting below covers
different response parameters; it does not identify tread/field friction.

For an existing differential tire model, [Measure tire slip…](MEASURED_TIRE_SLIP.md)
fits its steady longitudinal curve from direct force/load measurements and independent
hub/wheel speeds. The ordinary motor-current telemetry file below is insufficient for
this fit. Lateral behavior and reflected shaft inertia are retained as assumptions.

Copy [`CalibrationRecorder.java`](sample-teamcode/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/CalibrationRecorder.java) into your Android TeamCode source tree. It uses FTC SDK `DcMotorEx`, `HardwareMap`, and `CurrentUnit`. Create it once in an OpMode and close it when recording ends:

```java
File csv = new File("/sdcard/FIRST/calibration/motor.csv");
try (CalibrationRecorder recorder = new CalibrationRecorder(csv, hardwareMap,
        java.util.Arrays.asList("left_front_drive", "right_front_drive",
            "left_back_drive", "right_back_drive"))) {
    while (opModeIsActive()) {
        // Set test motor powers here, then record once per loop.
        recorder.record();
        idle();
    }
}
```

Copy the CSV from the Control Hub into the team project directory before fitting. The logger writes timestamp, loop duration, battery voltage, and each motor's commanded power, encoder position, velocity, and current. Use a bench recording with the wheels unloaded, multiple power levels and directions, and varying current load. Measure or estimate the effective rotational inertia reflected at the motor output shaft; the fitter cannot infer friction without it. Keep a safely restrained robot when testing wheels or mechanisms.

For drive response, make a separate `drive.csv` while the robot accelerates forward, strafes, reverses, and turns at several commands. Supply robot-frame forward and left velocity in m/s plus counterclockwise yaw rate in rad/s to `recorder.record(vx, vy, omega)`. These must come from a localization measurement independent of the drive motor encoders; encoder-derived chassis velocity would measure the commanded wheel motion instead of actual traction. Record at least 30 timed sample pairs with strong acceleration steps. Omit the drive file if independent chassis measurements are unavailable.

## Fit and load a profile

Run from the FTC Forge repository root, replacing dimensions and the measured output-shaft inertia with your robot's values:

```sh
./gradlew :gui-runner:calibrate --args='/absolute/team/project motor.csv calibration.json 0.0015 0.048 0.30 0.35 drive.csv'
```

The arguments are team directory, unloaded motor CSV, output JSON path, effective motor load inertia in kg·m², wheel radius in meters, drive track width in meters, wheelbase in meters, and optional drive CSV. CSV and output paths are relative to the team directory. The motor CSV must contain at least 30 samples with varied current and speed. The fitter checks input shape and fit quality, reports drive residuals if supplied, and stores sample counts and battery error in `evidence`. Inspect the output and compare its measured response with robot runs before treating it as accurate.

Add `"calibration": "calibration.json"` to the team's `sim.config`. Both the headless runner and 3D renderer load battery and motor-friction values; the renderer also applies the fitted drive response. Removing that entry returns to defaults. Servo shaft parameters are configured separately under `servoPhysics` in `sim.config`, as described in [the import guide](ROBOT_IMPORT.md).

The current fit covers battery open-circuit voltage/internal resistance, static and viscous motor friction, and planar drive response/acceleration caps. It does not fit thermal behavior, wheel roller contact, mechanism backlash, or mass/inertia from robot logs. The drive fit models free planar acceleration; reject recordings dominated by collisions, external pushes, or localization loss. No real team recording is included in the repository, so the supplied defaults and synthetic recovery test are not evidence of hardware agreement.
