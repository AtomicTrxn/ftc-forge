# Standard REV DUO powered CAD results

## Delivered

- Differential kinematics/configuration with two independently named motor sides, mounting signs, wheel radius and CAD track width. Default Mecanum projects still run.
- A reproducible Python preparation tool and paired FTC hardware/motor/TeamCode template. The actual supplied export retains all 614 original links and 750 visuals; nine assembly frames produce a 623-link prepared model with six wheel joints and two physical intake hinges.
- The CAD's rear omni wheels, center grip wheels and front traction wheels stay animated chassis ballast. Two Core Hex-driven intake shafts use one motor transmission and a physical 1:1 chain follower constraint. No duplicate motor torque is applied.
- Wheel/chassis/intake collision envelopes preserve the open mouth. Native convex cylinder collision avoids unnecessary V-HACD. Mesh and game-piece hulls use a 2 mm contact margin.
- Robot coordinates are separate from Bullet principal-inertia axes. A physical level constraint locks pitch/roll while keeping translation and yaw free.
- Motor intake capture checks actual shaft speed/direction and a robot-origin capture point. Held pieces move their collision body to the held point before release.
- Drive encoder signs, velocity feedback and encoder resets now respect FTC logical direction while wheel visuals retain physical shaft angles.
- Local powered project plus autonomous and keyboard launchers. Camera orbit/zoom follows the imported differential robot; coincident floor faces and a one-tile visual offset and the close camera's near-plane clipping were corrected.

## Actual CAD evidence

Source URDF SHA-256: `36b1106071e8d9e86b6866c9260fe65cc7e3923ea1a1e51376e1391f141d5195`.

The original package remains private under `.local/robots/rev-duo-starter`; the prepared project, report, screenshots and validation logs are local. No CAD meshes are committed.

Representative real renderer run of `RevDuoAuto`, using the original meshes and FTC executor:

```text
[PHYSICS] Imported 3 dynamic bodies and 2 mechanism joints; mass=2.697420669238
[PHYSICS] Game piece captured at distance 0.11837651m (< 0.12m).
[TELEMETRY] Drive ticks : 1478, 1478 | Straight yaw : -0.000204563 | Intake ticks : 890
[TELEMETRY] Turn yaw : 1.4752364
[SIM] Final chassis position: (0.7342874, 0.05869383, 0.0060120253)
      yaw=1.4756997 gamePieceHeld=true
```

A separate renderer run ended with intake source/follower angles `34.3375575` and `34.3375879` radians. A screenshot of the powered CAD and held piece was inspected; the floor-offset and camera-clipping corrections were also inspected.

The deterministic native probe ran the actual prepared CAD at 60 Hz:

```text
straight=(1.2084863, 0.05869274, 0.0067666923) yaw=-0.001784504
intake_lower_joint=26.7800177283, intake_upper_joint=26.7800151949
capture/release passed
turn yaw=1.0245592 PASS
```

The probe checks straight displacement, lateral/yaw drift, source/follower phase, actual intake encoder movement, capture/release placement and positive turn yaw. It directly advances motor/physics state; the renderer run above separately verifies real TeamCode execution. Timing-dependent renderer positions are illustrative, not calibration measurements.

## Bugs found through execution

1. **Principal frame confused with chassis forward:** CAD inertia can rotate the body's local principal axes away from robot forward. Drive/IMU/intake previously used the body frame directly. An explicit robot-frame transform now supplies heading and robot-origin points; a rotated-frame native regression verifies forward motion.
2. **Level constraint leaked under mechanism impulses:** Bullet angular factors alone did not prevent imported joint constraints from tilting the chassis. The actual CAD veered to yaw `-0.6099` radians over a nominal straight run even without a game piece. A single-ended 6-DOF constraint locks pitch/roll while leaving yaw and translations free. The same CAD probe now stays within about `0.002` radians of straight yaw.
3. **Collision skin prevented capture:** The robot initially pushed the torus without reaching its configured 0.12 m capture radius. Default 0.04 m convex margins are large relative to FTC parts. Using 0.002 m margins brought the physical surface into agreement with the documented envelopes; capture then passed without widening the capture radius.
4. **Released piece retained its pre-capture physics pose:** Held visual movement previously left the removed physics body at an old pose. Moving its body during hold and setting its release location fixes this; a native capture/move/release regression checks the resulting body position.
5. **Reversed unbound motor exposed physical encoder sign:** Left drive initially reported negative ticks under logical forward power. Unified shaft-to-logical feedback now reports positive ticks/velocity for both sides. Reset changes the encoder reference while preserving wheel visual angle.

## Validation

- `./gradlew test --offline --no-daemon`: **50 Java tests, zero failures** (motor, battery, calibration, importer, physics, executor, IMU and kinematics).
- `python3 -m unittest discover -s tools -p 'test_*.py'`: **2 preparation tests, passed**. Synthetic fixtures check source preservation, world transforms including Euler singularities, masses/visuals, mesh references, and rejected exports/destinations/entities. Private CAD is not needed for these tests.
- Actual CAD renderer autonomous: **capture true**, logical drive ticks positive, turn yaw positive, both physical intake shafts rotate together.
- Actual CAD fixed-timestep probe: **passed**.
- Existing `BasicMecanumOpMode`: forward/strafe run exits successfully; final x/z approximately `(0.869, 0.760)` m.
- Existing `IntakeDemoOpMode`: capture true, final position `(0.617, 0.1, 0)` m.
- Existing `DashboardOpMode`: completed normally with Dashboard config snapshot and motor feedback.
- Existing `HungOpMode`: watchdog marks the session dead and Gradle/JVM exits successfully.
- `git diff --check`: clean.

## Remaining accuracy limits

The import/readiness gaps are closed for this verified export. **Real-world accuracy has not been measured.** The 2.6974 kg CAD mass needs an operating-weight measurement; the hardware XML/names are a template until matched to the team's hub. Gearbox output constants use REV's actual cartridge reductions and a lossless torque baseline. Calibration is still required for gearbox losses, effective skid-steering track and aggregate traction.

Six wheel contacts are chassis collision envelopes, with aggregated planar drive traction. Individual tire slip/roller friction, drive encoder stalling against walls, deformable flaps, chain slack and chassis tip-over are not simulated. Drive motor load inertia retains the unbound model's baseline. The illustrative torus and proximity hold are not current-season field/game validation. The prepared model represents the supplied starter base; it cannot supply mechanisms absent from that CAD.

See [the operating guide](../../gui-runner/REV_DUO.md) and [calibration guide](../../gui-runner/CALIBRATION.md).
