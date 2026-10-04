# Inspecting live physics

In the field simulator and import-time robot motion demo:

- **D** shows/hides the diagnostics panel and vectors.
- **P** saves a JSON snapshot in `build/physics-diagnostics/` from the working directory.
  Snapshots contain model/body/device names and simulated state, not CAD asset bytes.
- **C** shows/hides native collision shapes independently.
- **V** shows/hides CAD independently; diagnostics remain visible.

Add `--diagnostics` to either renderer command to start with the panel visible:

```sh
./gradlew :gui-runner:runSimulatorApp --args='gui-runner/sample-teamcode TurnAndResetOpMode --diagnostics'
./gradlew :gui-runner:demoRobot --args='/absolute/profile.json - /absolute/session/demo.json --diagnostics'
```

The demo also supports `--headless --diagnostics`, adding a final diagnostic snapshot to
its normal JSON movement report. This reads the profile without approving or changing it.

## What the panel means

| Value | Meaning / units |
| --- | --- |
| Wheel ON/OFF | Last native drive support state; legacy proxy support is explicitly unavailable |
| Load / grip limit | Normal support impulse divided by native timestep / available material grip, N |
| Slip | Wheel surface speed minus forward hub speed, m/s; unavailable in aggregate drive mode |
| Tire force | Last individual longitudinal tire solver force, N; snapshot also includes lateral force |
| Motor command | Physical signed effective command (including configured direction), unitless |
| Motor speed | Physical shaft speed, rad/s |
| Motor torque / current | Motor model estimates, N*m / A; these are not physical measurements |
| Joint position / limits | m for slides, rad for hinges/continuous rotors |
| Joint effort | Applied impulse effort, N or N*m; elastic native motors show an **effort limit**, not claimed actual constraint force |
| Joint rate | Cached last-tick relative rate, m/s or rad/s |
| Drive yaw | Aggregate controller's applied yaw impulse divided by timestep, N*m |

Joint values come from the native solver's cached telemetry. Rendering never samples or
unwraps joint angles, changes motor commands, refreshes contact ownership or advances time.
Loads derived from impulses are averages over one native tick and can fluctuate during
contact changes. Contact positions/gaps are from the last solved manifold; positive gaps
are retained in saved snapshots. Wheel support follows the production solver's accepted
contacts rather than treating every visible contact as traction.

The panel distinguishes no wheel support, belly scraping, zero available material grip,
missing expected drive bindings and tires reaching their grip limit. **Possible blocking**
requires drive effort, little motion and loaded obstacle contact, or mechanism effort with
little motion and contact. It is a diagnostic suggestion: contact, gearing, torque, load,
joint limits or setup may each contribute. A near-limit message reports the configured
boundary, without claiming that the limit caused a stall. Default slow-drive thresholds
are 0.01 m/s and 0.03 rad/s; near-limit tolerances are 2 mm or 0.015 rad. No-motion mechanism
thresholds are 0.002 m/s or 0.02 rad/s, with effort above 0.01 N or N*m.

## Vector legend

| Color | Meaning |
| --- | --- |
| Green | Native contact normal, pointing toward the robot body |
| Yellow | Normal load at that native contact |
| Red | Individual tire force (longitudinal and lateral) |
| Blue | Aggregate drive force at the chassis center |
| Cyan | Mechanism joint axis at its pivot |

Force arrows use **1 N = 1 cm**, capped at **35 cm**. Green normals are fixed at 8 cm and
cyan axes at 12 cm; those two arrows show direction only. These display lengths do not
change model scale or physics. The full force value remains in the snapshot even when an
arrow is capped. Existing collision outlines retain their separate cyan/orange legend.

## Sampling and snapshots

Visible diagnostics update at most 10 times per second and reuse arrow geometries/materials.
Hidden diagnostics do not poll native state. At most 4 wheels, 4 motors and 4 mechanisms
are printed on the panel; **P** includes up to 32 of each and 64 contact points. The contact
scan is capped at 4,096 manifolds. Sampling limits are disclosed in the panel and JSON;
diagnostics are not an exhaustive contact audit of a large assembly. Flexible paddle
segments are included in robot contact sampling.

Replay rebuilds the diagnostics with the new native robot, retains the D visibility choice,
and removes the old panel/vectors. Closing removes diagnostics. Saving creates a separate
file and never modifies a profile, receipt, review or calibration.

For a reproducible rendered image of the demo, add
`--screenshot /absolute/output/demo.png`. This captures after the first movement begins
(at least 60 render frames) and closes the window; its movement report is intentionally
incomplete. A screenshot requires a window, and must be outside the model directory.
The field simulator's existing preview screenshot path also accepts `--diagnostics`.

These observations improve setup/debugging. They do not establish real robot accuracy,
replace collision review or certify measured calibration.
