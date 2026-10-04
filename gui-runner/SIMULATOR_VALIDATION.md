# Simulator validation without another robot

From the repository root, run:

```sh
./gradlew :gui-runner:validateSimulator
```

Each run creates a separate `build/simulator-validation/run-…/` directory containing
`report.json`, `report.md`, source ZIP packages and isolated model libraries. The command
exits unsuccessfully when any scenario fails. Failures do not skip subsequent scenarios.
The Markdown report gives each expectation and its observed values; JSON is suitable for CI.
No private CAD, graphics window, TeamCode, gamepad or physical measurements are required.
The normal model preparation Python dependencies and native Bullet library are required.

## Scenarios

The suite imports generated packages through the same preparation path as Guided setup.
It runs differential and Mecanum drive directions, articulated arm/slide/servo/continuous
intake travel, airborne/no-grip/belly-grounded cases, wall blocking and external pushes.
Flexible paddle contact transports a small native sphere in both directions. A separate
legacy proximity scenario checks torus pickup, carry and release. The latter is explicitly
not evidence of compliant contact retention; the existing `verifyTorusRetention` probe
continues to cover the configured REV retention model.

A ZIP containing one URDF and its referenced STL, expressed in millimeters, checks metric
size and axis conversion. Bare STL exporter archives require an adapter before generic
preparation; rejection is tested. STL alone has no actuator information and must not acquire
drive capabilities. The portable-profile scenario checks export/import and changed-CAD
migration preserve drive/contact settings, and changed geometry requires renewed collision
review. Synthetic fixtures are labeled as such; only the isolated portability fixture is
marked reviewed to exercise that workflow.

The fixed motion timestep is 1/480 s; contact scenarios use 1/120 s, paddles 1/240 s.
Direction and joint-limit checks use broad behavior bounds rather than exact golden positions.
The generated fixture specification and reports include version/timestep metadata.

## Check an existing prepared robot

```sh
./gradlew :gui-runner:validateSimulator --args='--profile /absolute/path/profile.json --output build/my-robot-check'
```

Optionally add `--project /absolute/path/team-project` to use that project's hardware and
motor presets. Without it the motion demo uses its explicitly labeled generic motors.
This checks every configured movement on the neutral floor, including its declared sign
and joint limits. An empty movement plan fails. The original profile is read only; this
does not approve collisions, resolve migration choices, or establish hardware accuracy.
The output contains a fresh run directory; choose an output outside the model directory.

## Physics matrix

```sh
./gradlew :gui-runner:validateSimulator --args='--matrix'
# Optional bounded test ranges/criteria:
./gradlew :gui-runner:validateSimulator --args='--matrix --matrix-config /absolute/path/matrix.json'
```

The bundled specification is `src/main/resources/simrunner/validation-matrix.json`.
Copy it to change the test ranges or tolerances; every field is required, unknown fields,
duplicate values and invalid ranges are rejected. Its exact contents are included in the
report. This config changes test scenarios, not saved robot assumptions.

Defaults test 1/3/12 kg, 40/75 mm wheel radius, 240/500 mm track width, floor coefficients
0/0.01/0.6, and 30/120/480 Hz native stepping. The synthetic wheel coefficient is 0.6.
The 108 drive cases check mass once, zero-command drift, finite/bounded state, material
impulse limits, the independent speed-gain bound `wheel_mu × floor_mu × g × time`,
penetration and passive braking. Thirty-six comparisons bound travel spread across timesteps
by `absolute_tolerance + relative_tolerance × max_abs_travel` (default 0.04 m + 15%).
This is a sensitivity check, not a proof of asymptotic convergence or identical trajectories.

Additional cases exercise signed mechanism reductions 1/−5/10, and motor-driven native tire support,
wall loading, airborne/zero-grip spin, material force caps and contact/shaft momentum reaction
at each timestep. Wall loading must reduce speed and increase current; unsupported tires
accelerate faster. Settled robots teleported upward cannot reuse a previous floor contact.
Together with the base suite this is **174 reported scenarios**.

Supported custom ranges: mass 0.5–20 kg, radius 35–120 mm, track 230–800 mm, floor coefficient
0–1.2, native timestep 1/480–1/30 s, signed mechanism reduction with magnitude 1–20. At most four values per
dimension and 200 drive combinations are allowed; at least two timesteps are required.
Acceptance-tolerance bounds are validated by `ValidationMatrixConfig`; defaults explicitly
include 5 mm drift, 10 mm penetration, 0.03 m/s speed-bound allowance, 0.0001 kg mass error,
0.00001 N·s drive impulse allowance, 0.001 J passive-energy allowance and 0.00001 N·m
shaft-reaction error. All ranges and tolerance values are recorded in each report.

The drive sweep supplies a target to the production aggregate controller to isolate contact
response from motor speed. Separate native tire scenarios integrate production motors and
shaft loading. Mechanism motion uses the production demo timestep (1/480 s). The matrix
does not cover every mechanism/configuration/timestep cross-product or private imported CAD.

## Interpretation

Passing establishes expected software/native behavior for these bounded scenarios on the
reported platform. It is not exhaustive coverage, an exact deterministic replay guarantee,
or proof of real friction, compliance, motor response or simulation-versus-hardware accuracy.
The level-chassis and welded wheel assumptions still apply.
