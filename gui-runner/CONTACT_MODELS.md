# REV DUO tire slip and flexible intake

## Enable and run

Prepare a separate project from the supplied native REV DUO export:

```sh
python3 tools/prepare_rev_duo.py /absolute/package/urdf/robot.urdf /absolute/contact-project --contact-models
./gradlew :gui-runner:verifyRobotPhysics --args='/absolute/contact-project'
./gradlew :gui-runner:runSimulatorApp --args='/absolute/contact-project RevDuoTeleOp'
```

Use a new output directory. The source CAD and STL files remain untouched. The prepared project keeps all original visuals, adds the CAD polycarbonate ramp's collision envelope and uses a 13 mm rigid lower-roller core surrounded by flexible paddle contacts. The preparation report lists mass overrides and accuracy limits. Omit `--contact-models` to retain the earlier rigid/proximity workflow. The new JSON sections can be enabled separately; `tires` requires differential drive and URDF, and `flexible_intake` requires URDF and motor intake.

## Tire forces

Each of the six wheel hubs ray-tests a short distance toward the floor. Only flat, upward-facing static support supplies traction; an airborne wheel spins without driving the chassis. Total assembly weight is divided equally among supported wheels. The rear omni wheels use less lateral friction than the front and center traction wheels.

Longitudinal slip is `wheel angular speed × radius − hub forward speed`. Lateral slip is hub sideways speed. Velocity-proportional contact forces are integrated implicitly using chassis mass, yaw inertia and shared shaft inertia, then clipped to an anisotropic friction ellipse. Friction transitions from `static_mu` toward `sliding_mu` as slip increases. Each longitudinal force applies an equal reaction torque to its side's shaft. Thus encoder speed can exceed ground speed on slippery floors, and blocking a high-grip drivetrain reduces shaft speed and raises motor current.

The original imported wheel tensors supply rotational inertia. `reflected_motor_inertia_kg_m2` adds an explicit output-shaft motor/gear inertia baseline. The electrical torque/back-EMF model drives that physical shaft; encoder resets do not reset tire angle. The legacy speed-target controller is bypassed when tires are enabled.

| `tires` parameter | Prepared baseline | Meaning |
| --- | --- | --- |
| `traction.static_mu`, `sliding_mu` | 0.9, 0.7 | Longitudinal friction coefficients |
| `traction.lateral_scale` | 1 | Traction-wheel lateral coefficient multiplier |
| `omni.lateral_scale` | 0.05 | Approximate omni lateral resistance |
| `stiffness_n_per_mps` | 100 | Slip-to-force gain before limiting |
| `transition_mps` | 0.15 | Static/sliding transition speed |
| `omni_joints` | left/right rear joints | Explicit wheel classification |
| `reflected_motor_inertia_kg_m2` | 0.0015 | Additional inertia per side |
| `contact_tolerance_m` | 0.004 | Floor support tolerance beyond tire radius |

These are **unmeasured baselines**, not manufacturer tire measurements. Completed runs print `[TIRES]` support, wheel surface speed, hub speed, slip, longitudinal force and sliding state. Existing calibration profiles' drive response/acceleration parameters apply to the legacy controller; the current telemetry fitter does not estimate tire friction or reflected inertia.

## Flexible paddles

Each double-ended rubber paddle has two arms with three collidable segments per arm. Native [Minie spring constraints](https://stephengold.github.io/Minie/javadoc/master/com/jme3/bullet/joints/New6Dof.html) provide bending stiffness, damping and angular stops. Finite [contact stiffness/damping](https://stephengold.github.io/Minie/javadoc/master/com/jme3/bullet/collision/PhysicsCollisionObject.html#setContactStiffness(float)) soften impacts. Contact and bending loads pass through the physical shaft into motor torque/current feedback. Contact-mode chain coupling locks relative shaft orientation while leaving translation free, preventing the velocity-only gear constraint's phase drift under load.

The export's mesh X axis runs across the shaft, while mesh Z is radial. Import validates shaft alignment, and deformation occurs in the radial/tangential plane. Each rubber mesh gets independent buffers; the shaft/core geometry stays rigid. Deformation uses current native body poses and updates at most 30 times per second. Physics uses a fixed **480 Hz** step, 100 solver iterations and up to 64 substeps per rendered frame for this light-body contact model. The inspected full CAD run rendered at about 20 FPS on the development machine.

The CAD assigns each complete flap/core link approximately **6.123 mg**. Preparation explicitly changes each to a **5 g unmeasured baseline**, scaling its CAD tensor. Ten percent stays on the rigid rotor and ninety percent is transferred into the flexible segments: 27 g total in 36 segments, counted once. Prepared total CAD/baseline mass is 2.727384 kg. `total_mass_kg` still scales the complete assembly, including the transferred mass; measure operating weight before claiming hardware accuracy.

| `flexible_intake` parameter | Prepared baseline | Meaning |
| --- | --- | --- |
| `links` | six `flap` links | Explicit REV paddle instances |
| `segments_per_arm` | 3 | Beam discretization, allowed 2–6 |
| `flex_mass_fraction` | 0.9 | Link mass transferred into flexible arms |
| `width_m`, `arm_length_m` | 0.0116118, 0.0507965 | Shaft width and radial reach from STL bounds |
| `thickness_m` | 0.003 | Approximate box contact cross section |
| `stiffness_nm_per_rad` | 0.003 | Bending stiffness per hinge |
| `damping_ratio` | 0.5 | Spring damping ratio |
| `max_bend_rad` | 1.2 | Iteratively solved angular stop |
| `friction` | 1 | Paddle contact coefficient |
| `contact_stiffness_n_per_m` | 500 | Normal contact stiffness |
| `contact_damping_ns_per_m` | 0.3 | Normal contact damping |

Rubber thickness, stiffness, damping, mass allocation and friction require measurement. Angular stops can overshoot during heavy impacts; they are not guaranteed hard geometric bounds. Segments approximate bending and contact, not full material FEM, stretch, tearing or detailed rubber compression. This importer currently expects the verified REV `Flap.stl` coordinate convention and aligned shafts; it does not classify arbitrary flexible meshes automatically.

## Game-piece behavior and limits

Flexible mode keeps the game piece in Bullet at all times. No capture impulse, proximity snap, freeze or removal is applied. `containment_min_xyz_m` and `containment_max_xyz_m` define a reporting region in the robot's URDF coordinates; `[SIM] gamePieceContained` means the piece's center is inside that region, not that it is latched. The legacy API `isPieceHeld()` returns that same containment flag in contact mode. Stopping/reversing the intake lets physical contact, friction and gravity determine release.

The illustrative torus now spawns flat above the floor, avoiding initial floor penetration. Native paddle tests transport a small free sphere in both shaft directions. The supplied CAD also contacts and redirects the 100 g torus, but the baseline does **not** retain it: `gamePieceContained=false` is the correct observed result. Actual-season game-piece geometry, intake calibration and retention behavior remain necessary before treating this as a validated capture model.

Other approximations remain: equal quasi-static wheel loads, a level chassis with no tip-over, flat static wheel support, simplified tire/roller/ramp envelopes, no individual omni rollers, lossless gearbox baseline and ideal chain coupling. This is suitable for exercising slip, collisions, motor loading and flexible contact; real-world accuracy still needs hardware telemetry.

See [validation results](../tasks/contact-models/RESULTS.md) and [the original REV setup guide](REV_DUO.md).
