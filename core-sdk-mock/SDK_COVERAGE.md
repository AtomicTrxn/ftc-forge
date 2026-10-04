# SDK coverage

`sdk-surface.txt` is the authoritative list of FTC SDK, FtcDashboard and vendor classes the simulator
stubs. `SdkSurfaceTest` fails the build if a listed class or method disappears or a new stub class is
added without being listed. This page explains the support levels and what is deliberately missing.

| Status | Meaning |
|---|---|
| simulated | Behavior is wired to the simulator (motor/battery model, physics pose, IMU, hub voltage, OpMode lifecycle). |
| state-only | Holds the commanded state; no physics effect yet (servos, CR servos). |
| shim | API-compatible replacement that prints to the console instead of a web UI (FtcDashboard, `Canvas`, `TelemetryPacket`). |
| inert | Accepts calls and returns constants (vision, distance/color/touch sensors, gamepad rumble). |
| support | Data and utility types (units, `Pose2D`, `ElapsedTime`, annotations). |

## Hardware worth knowing about

- **Hubs** are `LynxModule`s and `VoltageSensor`s. `hardwareMap.getAll(LynxModule.class)` works and reports the shared
  battery-sag voltage. Bulk-caching modes are stored but have no read-batching effect.
- **goBILDA Pinpoint** (`com.qualcomm.hardware.gobilda.GoBildaPinpointDriver`) and **SparkFun OTOS**
  (`com.qualcomm.hardware.sparkfun.SparkFunOTOS`) report the physics chassis pose in the robot start frame
  (x forward, y left, counterclockwise heading). Add `<goBILDAPinpoint name="pinpoint" .../>` or
  `<SparkFunOTOS name="otos" .../>` to the robot configuration XML. Headless runs have no physics world, so these
  stay at their start/set pose. Pod offsets, encoder settings, I2C latency, drift and pod faults are not modeled.
  Their method lists were checked against the vendors' published drivers (see `gui-runner/COMPATIBILITY.md`).
- A **vendored driver copy** (for example `GoBildaPinpointDriver.java` copied into `TeamCode`) extends raw I2C classes that
  are not simulated. Delete the copy and use the built-in class; the compiler explains this when it hits the import.

## Not supported

Compile errors for these show a "Simulator support notes" section naming the import and what to do:
camera pipelines (EasyOpenCV, OpenCV, TFOD, Limelight), Android APIs, `ftccommon`, raw I2C device classes, and any
other `com.qualcomm.*` / `org.firstinspires.ftc.*` class not listed in `sdk-surface.txt`. Road Runner and Pedro Pathing are
real libraries: put their jars on `extraClasspath`.
