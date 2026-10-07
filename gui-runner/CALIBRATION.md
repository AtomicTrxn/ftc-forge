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

Add `"calibration": "calibration.json"` to the team's `sim.config`. Both the headless runner and 3D renderer load battery and motor-friction values; the renderer and physics-backed headless runner also apply the fitted drive response. Removing that entry returns to defaults. Servo shaft parameters are configured separately under `servoPhysics` in `sim.config`, as described in [the import guide](ROBOT_IMPORT.md).

## Fit the power-system, bus and sensor-error parameters

The same command takes optional `key=value` arguments that run extra stages and write their results into the profile's
`electrical` block, which uses `sim.config`'s own schema (see [the power and bus models](POWER_AND_BUS.md)) and **overlays
`sim.config` setting by setting**. The profile's own battery and friction values still win over both.

```sh
./gradlew :gui-runner:calibrate --args='/team motor.csv calibration.json 0.0015 0.048 0.30 0.35 capacity_mah=3000 fit_inertia=true fit_velocity_window=true sensor_log=sensors.csv'
```

| Option | Stage | Needs in the recording | Writes |
|---|---|---|---|
| `capacity_mah=N` | Battery depletion: voltage versus charge drawn, and resistance rise | A long loaded run that draws at least 2% of the pack, with varied current | `battery.capacity_mah`, `internal_voltage_v`, `internal_resistance_ohm`, `empty_voltage_v`, `empty_resistance_ohm` |
| `fit_inertia=true` | Rotor/load inertia fitted together with friction | Spin-up transients (power steps or ramps); refused when inertia is not identifiable | `motor_overrides.<motor>.rotor_inertia_kg_m2`, and the friction in `motors` |
| `fit_velocity_window=true` | Hub velocity estimation window | Speed that changes over the run | `bus.velocity_window_ms` |
| `sensor_log=file.csv` | IMU and odometry latency, scale, drift, bias and noise | An independent **reference** for the same quantity, in the same frame | `imu_latency_ms`, `imu.*`, `odometry.*` |

**Sensor columns.** The log may add `imu_yaw_rad, ref_yaw_rad, imu_rate_rad_s, ref_rate_rad_s, odo_x_m, odo_y_m,
odo_heading_rad, ref_x_m, ref_y_m, ref_heading_rad` (all ten or none; a cell may be blank, but each reading needs its
reference on the same row). References come from something other than the sensor under test: a turntable, motion capture
or a measured run, expressed in the start-pose frame with x forward, y left and counterclockwise radians. The IMU yaw and
rate must change by at least 0.5 rad and the odometry reference must move at least 0.3 m; gyro bias and noise come from
stretches where the reference rate stays near zero for the whole latency window (at least 30 samples). `TelemetryLogWriter`
writes these columns; the on-robot `CalibrationRecorder` does not, so join a reference recording by timestamp.

A stage whose data cannot support it is skipped, and the reason is printed and stored under `fit_notes` rather than
guessing a value. Values outside the range `sim.config` accepts are not applied. Fitted noise values are the residual
spread after alignment, so they include any unmodeled error. The depletion stage assumes a linear discharge curve and the
capacity you give; its full voltage includes any constant hub or servo load, because that current is not in the log.

**Not fitted** (the recording has no reference for them): encoder, voltage and current latency, bus read cost, loads,
brownout threshold, thermal derating, and seeds. Set those by hand. The closed-loop tests in `CalibrationPassTest` run
the simulator with known parameters, record a log, fit it and check the values come back; that validates the fitting
code only. No real robot recording is included, so agreement with hardware is unmeasured.

The current fit covers battery open-circuit voltage/internal resistance, static and viscous motor friction, and planar drive response/acceleration caps. It does not fit thermal behavior, wheel roller contact, mechanism backlash, or mass/inertia from robot logs. The drive fit models free planar acceleration; reject recordings dominated by collisions, external pushes, or localization loss. No real team recording is included in the repository, so the supplied defaults and synthetic recovery test are not evidence of hardware agreement.
