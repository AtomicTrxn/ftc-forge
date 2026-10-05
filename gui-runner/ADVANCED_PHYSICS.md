# Advanced physical models

## Lateral and transient tire response

An existing differential `tires` configuration may include `traction_response` and/or
`omni_response`. In guided setup, edit the robot's runtime JSON in Advanced settings:

```json
"traction_response": {
  "lateral_stiffness_n_per_mps": 70,
  "lateral_mu": 0.4,
  "relaxation_time_s": 0.08
}
```

These are example assumptions, not measured coefficients. Each response object requires
all three values. Lateral stiffness is 0..100000 N/(m/s), lateral friction 0..2, and force
relaxation time 0..2 seconds. Zero lateral grip models free transverse rolling; zero
relaxation retains an instantaneous brush response. Absent response objects retain the
previous lateral scale/stiffness and zero relaxation. Measured longitudinal evidence is
retained: it identifies a steady curve, not the new transient/lateral parameters.

The contact-utilization filter uses an exponential time response, an anisotropic friction ellipse,
the native material cap and a no-overshoot impulse bound. A lagging force is removed
when it would add slip energy after reversal. Loss of support resets its history.
This is a dissipative empirical force response, not a deforming tire or an identified
relaxation-length model. Detailed Mecanum roller kinematics require separate geometry.
Settings travel with exported profiles; changed CAD still requires collision review.

## Rotating wheels, slopes and tip-over

For a differential URDF robot, remove `tires`, disable `drive_contacts.enabled`, and add
the following runtime setting in Advanced settings:

```json
"rotating_wheels": { "reflected_motor_inertia_kg_m2": 0.0015 }
```

Every drive wheel needs a continuous joint, reviewed collision geometry, positive mass
and inertia, and one 1:1 transmission to its side motor. Multiple wheels sharing a motor
are coupled by native gear constraints and divide the available torque; reflected motor
inertia is distributed once over that shaft's wheels. Axles must align with their CAD
principal inertia axes. Inertia must be measured/estimated for the motor and attached
drivetrain. Use the motion demo to review axle directions and motor mounting signs.
This mode supports differential/tank running gear, not individual Mecanum rollers.

Wheels are separate rotating bodies: their collision geometry rotates, normal forces
and traction come from Bullet, and chassis force is **not** also applied by the aggregate
or brush model. Motor feedback follows native joint motion. Native friction is a Coulomb
contact approximation; choose brush tires for the tunable steady/transient slip curve.
These are alternative models, not additive forces. Incompatible settings are rejected.

Set the model parameter `chassis_lock_level` to **false** to allow pitch, roll, slopes and
tip-over. A CAD package may define passive prismatic suspension joints above its wheels;
set each joint's spring (N/m), damping (N*s/m), rest position (m) and travel limits in Parts.
In rotating-wheel mode those springs use native 6-DOF spring constraints with stability
limits. They must be unpowered. Spring effort in diagnostics is an estimate from the
configured law, not a claimed solved constraint force. Full pitch/roll and body angular
rates feed the simulated IMU; yaw reset changes only its yaw reference.

Small-wheel suspension and impacts can need 480 Hz physics. Imported rotating-wheel
and brush-tire simulation use that fixed step; hardware accuracy, terrain compliance, bearing/gear
losses, tire deformation, articulated tank treads and detailed rollers are not inferred
from CAD. Level chassis remains the default for existing profiles.

## Scene sensors and geometric AprilTags

In the robot's Advanced settings, add a `sensors` list. Each name must match a device
in the project XML. Distance/color/touch devices live under `LynxModule`; cameras use
`<Webcam name="camera"/>` under `Robot`. The import motion demo creates matching virtual
bindings when no project is selected; its neutral floor has no field tags or painted regions.

```json
"sensors": [
  {"name":"range", "type":"distance", "link":"base",
   "xyz_m":[0.2,0,0.05], "rpy_rad":[0,0,0],
   "min_range_m":0.01, "max_range_m":2, "update_hz":30,
   "latency_ms":50, "noise_std_m":0.002, "seed":7},
  {"name":"switch", "type":"touch", "xyz_m":[0.15,0,0],
   "contact_radius_m":0.015, "threshold_n":0.1, "min_press_normal_cos":0.5},
  {"name":"camera", "type":"camera", "xyz_m":[0.2,0,0.15],
   "max_range_m":4, "horizontal_fov_rad":1.0472, "vertical_fov_rad":0.7854}
]
```

Poses use meters, URDF x forward/y left/z up and intrinsic roll/pitch/yaw radians;
local **+X points out of the sensor**. An omitted link means the robot frame, while
an explicit link follows that mechanism's native motion. Measurements use native
physics time, 1–120 Hz sampling, 0–1000 ms latency and seeded Gaussian distance/pose
noise (0–0.1 m standard deviation). These are configurable assumptions, not sensor calibration.

| Device | Measurement | Limits |
|---|---|---|
| Distance | Closest native collision hit along +X, reported in requested SDK units | One ray; no valid hit returns positive infinity; ignores all robot self-hits. |
| Color | RGBA from the first hit's configured field color region or first visual material | No illumination, reflectance spectrum or camera processing; no hit/unknown material gives black/zero alpha. |
| Touch | Local contact, pressure impulse / native step, normal opposing +X | Sensor must be mounted on a body with collision coverage; radius and normal cosine set its sensitive face. |
| Camera | Fully visible configured tag corners, front face, range, frustum and native occlusion | Geometric AprilTags only; robot self-occlusion and image/exposure effects are absent. |

Unconfigured SDK devices retain their previous placeholders; an unconfigured webcam
reports closed/no detections. Configured `VisionPortal` supports processor enable,
stop/resume streaming and close. `AprilTagProcessor` returns pose, metadata and acquisition
time, in inches/degrees by default; `setOutputUnits` changes pose units. Pose uses FTC
camera axes (x right/y forward/z up), left-positive bearing and planar range. Tag local
+X is its outward face normal; its xyz/rpy are in field coordinates. No image frames,
EasyOpenCV, TFOD or arbitrary object recognition are provided.

Robot orientation reports intrinsic ZYX in this simulator's URDF robot frame, **x forward,
y left, z up**; IMU mounting parameters do not currently remap that frame. Do not infer a
physical hub installation or default SDK mounting frame from these values.

## Portable field behavior and practice scoring

Add `field_behavior` to a field model's Advanced settings or project `sim.config`.
Project/scene overrides take priority. Exported model settings retain the complete object;
changed CAD requires review even when references migrate automatically. All geometry is in
meters at the same field/robot scale. This example describes **practice objectives**, not
an official BIOBUZZ scoring preset:

```json
"field_behavior": {
  "schema_version":1,
  "clock":{"auto_s":30,"transition_s":8,"teleop_s":120},
  "tags":[{"id":7,"name":"practice target","size_m":0.16,
           "xyz_m":[1,0.2,0.35],"rpy_rad":[0,0,3.1415926536]}],
  "colors":[{"min_xyz_m":[1.7,-2,0],"max_xyz_m":[2,2,1],
             "rgba":[220,10,20,255]}],
  "rules":[{"id":"practice garden","alliance":"neutral","mode":"occupancy",
            "containment":"partial","types":["pollen","red_nectar","blue_nectar"],
            "min_xyz_m":[0.6,-1.2,0],"max_xyz_m":[1.6,1.2,0.5],
            "points":1,"from_s":38,"until_s":158}]
}
```

- **M** starts/pauses the native match clock. It begins paused, including previews.
  AUTO, transition, TELEOP and FINISHED are derived from the configured durations.
  Pausing the clock does not pause robot physics or TeamCode.
- **R** resets field pieces/apparatus and scoring, deduplication and clock to paused zero;
  it does not restart TeamCode or return the robot to its starting pose.
- **K** writes JSON evidence under `build/field-behavior`: rules/bounds/clock, qualifying
  IDs, native positions, score changes and acquisition times. Event history caps at 10,000
  with an explicit truncation flag; final score sets remain complete.
- `occupancy` scores objects currently inside during its window and freezes the last
  evaluation at the window end; removals during that window remove points. `entry_once`
  credits each object's ID once per rule per reset, including objects present at the first
  active tick. `snapshot` captures occupancy at `until_s`. Native tick boundaries determine
  the snapshot; physics keeps running after the clock finishes.
- `center` tests the rigid-body origin. `partial`/`full` test its native axis-aligned bounds,
  which approximate irregular geometry. Boundaries are inclusive. `types:["robot"]` uses
  the **chassis only**, not all extended mechanism bodies. Piece types are exact strings.
  `field-only` suppresses piece awards but permits robot objectives and sensors.
- Each rule has its own region, red/blue/neutral alliance, integer points 0–1000 and time
  window. Objects can score in multiple rules intentionally. Nothing detects fouls,
  ownership stacks, ranking points, human-player actions, or referee judgement.

### BIOBUZZ layout audit

```sh
./gradlew :gui-runner:auditFieldLayout --args='/absolute/prepared/field.json /absolute/audit.json'
```

This compares identifiable assembly inventory with the FIRST Event Field Setup Guide
V1.0 (2026-09-12): two hives, four flowers, 40 pollen and eight nectar of each color.
It records all preparation approximations. The supplied CAD passes these inventory checks,
but its replaced practice piece poses and uncalibrated hive spring **remain unverified**.
Inventory is not the match-start distribution.
[Official setup guide](https://ftc-resources.firstinspires.org/ftc/field/eventfieldguide).

An optional third CLI argument supplies `reference.json` with `source_url`, `revision`,
`tolerance_m` and `instances:[{"id":"CAD instance ID","xyz_m":[x,y,z]}]`. It reports
missing/displaced centers against those reference poses. The same reference may be saved
under `field_behavior.layout_reference`. Audit does not certify orientations, unlisted
parts, tape/tag placement, piece preloads or physical damper calibration. A competition
layout and complete season scoring still need that evidence and game-specific evaluators;
a hive angle alone cannot establish an official tip.

## Reproduce the advanced checks

```sh
./gradlew :gui-runner:generateAdvancedFixture --args='build/advanced-fixture'
./gradlew :gui-runner:runSimulatorApp --args='build/advanced-fixture/project AdvancedDemo --preview --diagnostics'
./gradlew test
python3 -m unittest discover -s tools -p 'test_*.py'
./gradlew :gui-runner:validateSimulator --args='--matrix'
```

The generated four-wheel/two-motor robot has passive suspension, four sensor bindings,
a geometric tag, painted field wall and generic practice objectives. It contains no private
CAD or measured parameters. Tests independently exercise powered rotation and airborne
reaction, ramp settling, tip-over, suspension travel, sensor linkage/latency/occlusion,
SDK units/lifecycle, score windows/reset/boundaries and profile export/import/CAD migration.
Tire convergence is checked with yaw fixed to isolate its traction response; unrestricted
native yaw/contact trajectories can diverge numerically and require model-specific review.
