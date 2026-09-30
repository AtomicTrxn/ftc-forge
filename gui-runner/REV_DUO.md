# Running the standard REV DUO CAD robot

The supplied native Onshape starter-base export now has a powered preparation path. Its original 614 links, 750 visuals, STL files, masses and inertia tensors are preserved. The prepared model adds six encoder-driven wheel joints and two physical intake hinges connected by an ideal chain constraint. It uses two differential drive motors, rather than four Mecanum motors.

## Prepare and run

Use Python 3.9 or newer for preparation. Extract the downloaded ZIP with its package/urdf and package/meshes layout intact. From the repository root:

```sh
python3 tools/prepare_rev_duo.py '/absolute/package/urdf/robot.urdf' '/absolute/private-powered-project'
./gradlew :gui-runner:runSimulatorApp --args='/absolute/private-powered-project RevDuoAuto'
./gradlew :gui-runner:runSimulatorApp --args='/absolute/private-powered-project RevDuoTeleOp'
./gradlew :gui-runner:verifyRobotPhysics --args='/absolute/private-powered-project'
```

The tool supports the verified 614-link all-fixed REV DUO starter-base assembly and checks its component names and wheel positions. It refuses an existing destination or incompatible export. It writes a prepared URDF, configuration, hardware/motor template, sample TeamCode and `preparation-report.json` (source SHA-256, assembly membership, dimensions and limits). Mesh references point to the original local package, so keep that folder in place. No CAD files are bundled in the repository. Team project paths containing spaces must be quoted individually inside Gradle's `--args` string.

`RevDuoAuto` drives forward with intake enabled, then pivots and exits. `RevDuoTeleOp` accepts arrows for drive/turn, Space for intake and E for reverse. Drag the camera to orbit and scroll to zoom. A third renderer argument saves a PNG when an autonomous OpMode completes:

```sh
./gradlew :gui-runner:runSimulatorApp --args='/absolute/private-powered-project RevDuoAuto /absolute/result.png'
```

## Configuration and coordinate conventions

The preparation rotates the source CAD frame so URDF x points into the intake, y points left and z points upward. Motor mounting signs use **physical shaft rotation** and remain independent of the OpMode's `Direction` setting. The template reverses the left motor in TeamCode so positive power moves both sides forward.

```json
"drive": {
  "type": "differential",
  "left_motor": "leftDrive",
  "right_motor": "rightDrive",
  "track_width_m": 0.380996749,
  "wheel_radius_m": 0.045,
  "left_shaft_sign": -1,
  "right_shaft_sign": 1
},
"intake": {
  "motor": "intake",
  "shaft_sign": 1,
  "min_speed_rad_s": 1,
  "point_xyz_m": [0.245, 0, 0.065],
  "capture_radius_m": 0.12
}
```

Omit `drive` to retain the existing four-motor Mecanum configuration. Wheel transmissions identify the configured drive motors; their descendants remain animated ballast in the chassis body. Differential motion has zero commanded lateral velocity. Bullet still resolves external pushes and wall contacts through bounded aggregate traction impulses. The six-wheel track starts with the CAD center-wheel spacing; effective skid-steering track requires calibration.

The intake activates only when its physical motor shaft exceeds the configured inward speed threshold. Reverse rotation releases the piece. The capture point is relative to the robot's URDF origin, not its center of mass or principal-inertia frame. The illustrative torus is held by proximity capture; actual roller torque, chain coupling, encoder feedback and collision response run under Bullet.

URDF `<mimic joint="source" multiplier="1" offset="0"/>` is supported for passive continuous sibling shafts with aligned parallel axes. One transmission powers the source shaft; a Bullet gear constraint transmits motion to the follower. Physical import rejects unsupported multipliers/axes rather than duplicating motor torque. Chains have no slack, elasticity or breakage.

## Manufacturer baseline and remaining approximations

The supplied assembly contains REV-41-1602 and REV-41-1603 cartridges on each drive motor. REV specifies actual ratios [76:21 and 68:13](https://docs.revrobotics.com/rev-crossover-products/ultraplanetary/cartridge-details). The preset derives gearbox output torque/speed and encoder counts from the [HD Hex bare-motor specifications](https://docs.revrobotics.com/duo-build/motion/motors/hd-hex-motor). The intake uses [Core Hex specifications](https://docs.revrobotics.com/duo-build/motion/motors/core-hex-motor). Gearbox torque initially assumes no losses. Confirm physical cartridges and names against the actual robot.

The prepared collisions are explicit envelopes: six 90 mm wheel cylinders, two chassis rails, a rear crossbar and two intake cylinders. They preserve an open intake mouth and use a 2 mm contact skin; they do not reproduce every screw, opening or flexible flap. Full CAD tensors and centers of mass still determine rigid-body inertia. The level chassis constraint locks pitch/roll while allowing yaw and translation; tip-over behavior is therefore outside this model. This default workflow retains aggregate traction, the unbound drive motor load-inertia baseline and proximity capture. Enable the separate [contact-model workflow](CONTACT_MODELS.md) for individual tire slip, wall-load drive feedback and flexible paddle dynamics. Its material parameters remain unmeasured, and its illustrative torus is not retained.

The source's **2.6974 kg is CAD mass, not measured robot weight**. Set `total_mass_kg` to operating weight with battery and use the [calibration workflow](CALIBRATION.md) for real telemetry. The fixture is a starter **base**, so mechanisms absent from the export are absent from simulation. The game piece is an illustrative torus, not current-season field validation.

Validation and discovered fixes are recorded in [the results](../tasks/rev-duo-physics/RESULTS.md).
