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

## Interpretation

Passing establishes expected software/native behavior for these bounded scenarios on the
reported platform. It is not exhaustive coverage, an exact deterministic replay guarantee,
or proof of real friction, compliance, motor response or simulation-versus-hardware accuracy.
The level-chassis and welded wheel assumptions still apply.
