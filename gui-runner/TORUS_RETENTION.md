# Reliable illustrative torus retention

## Prepare and run

```sh
python3 tools/prepare_rev_duo.py /absolute/package/urdf/robot.urdf /absolute/retention-project --torus-retention
./gradlew :gui-runner:verifyTorusRetention --args='/absolute/retention-project'
./gradlew :gui-runner:runSimulatorApp --args='/absolute/retention-project RevDuoRetentionAuto'
./gradlew :gui-runner:runSimulatorApp --args='/absolute/retention-project RevDuoRetentionReleaseAuto'
```

Use a new output directory. `--torus-retention` includes tire slip and flexible paddles. Source CAD and meshes remain unchanged. `RevDuoRetentionAuto` approaches slowly, intakes, carries, turns and finishes with the intake stopped. The release demonstration adds reverse and a settling pause. `RevDuoTeleOp` uses Space for intake and E for reverse; approach slowly enough for the rollers to draw the piece in.

The generated profile sets `game_piece_start_xyz_m` to `[0.42, 0, 0.034]`. These are field coordinates with x forward, y left and z up. The optional setting also works independently of retention. Existing projects keep their earlier spawn when the field is omitted.

## What the approximation does

The illustrative piece is a 100 g torus with a 240 mm outside diameter and a 60 mm tube diameter. Its V-HACD collision shape remains eight convex hulls. Fully rigid torus contacts can wedge against the CAD ramp's front edge and beneath the lower roller. Increasing hull detail, slowing the approach and stiffening the paddles did not establish reliable retention.

The opt-in `torus_retention` model represents omitted rubber/torus compression with **finite native contact stiffness plus a compliant pinch coupling**. This is an explicit approximation, not measured REV hardware behavior or a full soft-body solver.

- Acquisition requires sustained **loaded Bullet contact with an actual flexible paddle**, positive physical shaft speed and an inward electrical command. Proximity, a stopped motor, unpowered coasting and reverse contact cannot acquire.
- The grip anchor starts at the actual piece center and moves toward the configured seat only as the shaft turns inward. Its angular anchor tilts the torus into the intake. Equivalent torus normals are handled consistently after the piece flips during release.
- Bounded spring and damping impulses act on the dynamic piece, with equal opposite force/torque reactions on the chassis. Travel work supplies an opposing shaft load; actual shaft motion still determines encoders, back-EMF and motor current.
- The stopped grip remains compliant. Excess displacement breaks it. Reversing moves the anchor outward only after real reverse shaft motion, then disengages. A short cooldown prevents immediate re-engagement.
- The piece stays dynamic, colliding and in Bullet throughout. No pose assignment, velocity assignment, freeze or body removal implements capture or release.

`gamePieceContained` requires a seated grip **and** a center inside the declared reporting envelope. It does not report a transient passage through that envelope as capture. The prepared envelope is URDF `[0.18, -0.08, 0.08]` to `[0.28, 0.08, 0.20]` m.

## Configuration

All `torus_retention` fields are required; the section requires `flexible_intake`. Points use the robot's URDF frame. The seat must be inside the containment envelope and the exit ahead of it.

| Field | Prepared baseline | Meaning |
| --- | --- | --- |
| `seat_xyz_m`, `exit_xyz_m` | `[.23, 0, .145]`, `[.33, 0, .045]` | Pinch seat and outward release target |
| `contact_dwell_s` | .015 | Continuous loaded paddle contact before engagement |
| `travel_m_per_rad` | .012 | Anchor travel per actual shaft radian |
| `angular_travel_rad_per_rad` | .15 | Angular anchor travel per shaft radian |
| `tilt_rad` | π/3 | Seated torus plane tilt |
| `stiffness_n_per_m`, `damping_ns_per_m` | 150, 4 | Translational pinch compliance |
| `max_force_n` | 8 | Coupling force cap |
| `angular_stiffness_nm_per_rad`, `angular_damping_nms_per_rad` | .15, .012 | Angular pinch compliance |
| `max_torque_nm` | .08 | Coupling torque cap |
| `break_distance_m` | .12 | Maximum displacement from grip anchor |
| `contact_stiffness_n_per_m`, `contact_damping_ns_per_m` | 100, 1 | Torus native contact compression/damping |
| `reflected_shaft_inertia_kg_m2` | .0015 | Additional motor/gear inertia on the driven shaft |

The reflected inertia models internal motor/gear rotation missing from the fixed CAD export, without adding assembly mass. It requires a principal-axis-aligned rotor. The flexible motor uses a finite torque motor constraint solved alongside contacts, instead of applying an impulse to the tiny rigid core and imposing an acceleration cap. BRAKE, FLOAT, reverse and load/current feedback have native regressions.

## Validation and limits

The fixed-step probe fails unless capture is sustained for at least 0.5 s, retention survives an actual translation and turn plus two seconds stopped, reverse ejects outward, repeated acquisition works, forces/torques remain capped and the body stays dynamic and finite. It varies paddle phase, lateral offset and intake power, rejects inactive/reversed/coasting/stalled capture, and breaks retention under an external overload.

The supplied CAD passes these scenarios. See [results and reproduction evidence](../tasks/torus-retention/RESULTS.md).

Contact compression permits geometric overlap; the rendered torus remains an undeformed mesh. The pinch seat, travel, holding strength, inertia and material parameters are **unmeasured**. Acquisition/reporting tolerances are 25 mm, and release settles within 35 mm of its exit anchor. Only one illustrative torus is modeled. This does not establish behavior with arbitrary meshes, season game pieces, high-speed impacts or real robot loads. Operating mass, material measurements and hardware telemetry remain necessary for predictive accuracy. The [tire/flexible-paddle limitations](CONTACT_MODELS.md) also apply. Omitting this section retains the earlier contact baseline; omitting contact models retains the legacy proximity workflow.
