# Phase 5 Results: Custom Robot Importer

**Status:** Implementation and worked-example validation complete; real FTC Onshape export remains blocked by missing API access. This phase is not marked fully done yet.

## Summary

Added optional paired FTC XML + URDF loading to the headless executor and jME renderer. Existing preset JSON still supplies motor SKU/ratio constants. The imported chassis uses URDF collision geometry, total mass, and an aggregate inertia estimate. Four imported wheel joints set wheel radii and, when mounted directly on the chassis, Mecanum track/wheelbase; their rotation is visual/encoder tracking only. See [`gui-runner/ROBOT_IMPORT.md`](../../gui-runner/ROBOT_IMPORT.md) for the user workflow and current scope.

## Parser Implementation

`RobotUrdf` parses URDF links, inertial mass/origin/tensor, box/cylinder/sphere/STL collision geometry, `fixed`/`continuous`/`revolute`/`prismatic` joints, limits, and `<transmission>` actuator lists. It rejects malformed link trees, unsupported joint types and geometry, missing links, invalid limits/reductions, and duplicate names. XML external entities and doctypes are disabled. Binary and ASCII STL are supported; the published Robot Soccer Kit `onshape-to-robot` export parsed as 114 links/113 joints, and its real binary `wheel2.stl` loaded as 64,594 triangles.

The renderer builds a hierarchy of link nodes and animates movable joints from hardware readings. The chassis collision uses the root and fixed-link subtree; STL and cylinder chassis geometry goes through V-HACD, with `vhacd_max_hulls` configurable from 1–16 (default 8). Moving links are drawn and tracked but have no separate articulated collision bodies yet.

## HardwareMap Resolution

The FTC XML still defines actual device names. Each URDF transmission targets a free-standing joint name and contains one or more actuator names; each actuator is resolved against the built `HardwareMap` as `DcMotorEx` or positional `Servo`. Missing or wrong-type names produce an `IllegalArgumentException` naming the transmission and actuator. A two-actuator `slide` transmission was tested with a paired XML that supplied `slide_motor` and `slide_right`; removing the latter produced the expected failure. `mechanicalReduction` converts motor output-shaft radians into joint radians or meters. For linked motors the displayed joint position is the mean of their reported positions, clamped to URDF limits where present.

## CAD Mass Override

Optional `total_mass_kg` scales every link mass and every inertial tensor entry by the measured-total/CAD-total factor. The renderer sums transformed link inertia tensors plus parallel-axis terms about the chassis origin, updating its diagonal as mechanisms move. Bullet uses that diagonal as its local rotational inertia and the total imported mass as rigid-body mass. This is an aggregate body approximation; it is not an articulated rigid-body simulation of arms/slides.

## Validation

- R6's four-wheel Mecanum + arm + slide example is checked in as paired [`robot.urdf`](../../gui-runner/sample-teamcode/robot.urdf) and the existing [`robot_config.xml`](../../gui-runner/sample-teamcode/robot_config.xml), plus the existing motor preset JSON. The optional example config sets `total_mass_kg=11.0`.
- Gradle `test --offline`: passed, including the parser/mass/multi-actuator tests and existing physics-engine tests.
- Headless `BasicMecanumOpMode` with the URDF example: import reported 7 links, 6 joints, 11.0 kg; the OpMode completed normally and final ticks were LF=2662, RF=46, LB=-46, RB=-2662, preserving the expected forward-plus-strafe sign pattern.
- The same imported example in the real jME/Minie scene: 3D application completed normally and final chassis position was approximately `(0.725, 0.025, 0.722)` metres, demonstrating forward and lateral motion with imported mass/collision under rigid-body physics.
- Import-time validation and load-time mismatch behavior are exercised by `RobotUrdfTest`.

## Backward Compatibility

When `sim.config` lacks `urdf`, both entry points use the Phase 1 preset chassis path. Existing `presetMotors` parsing and per-motor SKU/ratio values were left in place. Re-ran the unchanged sample project's Dashboard OpMode: `Config snapshot={Tuning.DRIVE_POWER=0.5}` and normal completion. Re-ran `HungOpMode` with a 2000 ms watchdog: session marked DEAD and the process exited normally. Both Gradle runs passed.

## Real CAD Import Spike Findings

Used [FTC Team 11285's public Knock Out V4 Onshape assembly](https://roboftc.github.io/robots/knockout.html) as the target. Installed `onshape-to-robot` 1.8.3 into `/tmp/ftc-cad-spike` and ran it against the assembly's document/workspace IDs with `outputFormat=urdf`. It stopped immediately with `ERROR: No Onshape API access key are set`; no FTC assembly URDF was produced. The local environment has no `ONSHAPE_ACCESS_KEY` or `ONSHAPE_SECRET_KEY`. This is a tested blocker, not a successful export. [The exporter's own example documentation](https://github.com/Rhoban/onshape-to-robot-examples#why-do-i-get-error-403-while-using-onshape-api) also warns that users may need export rights or their own copy of a public document. The hands-on exercise therefore confirms setup friction but cannot establish how cleanly this FTC team's mates, parts, and masses export.

For a separate downstream file-format check, cloned the exporter's public examples and successfully parsed a real exported URDF and loaded a real STL as noted above. That is a different robot and does not close the FTC export gate.

## Deviations from Research

- `mechanicalReduction` is applied to motor output-shaft radians calculated from encoder ticks, with an average for multi-actuator joint display. Actuator torque is still owned by the Phase 3 motor model.
- Imported moving links are visual/encoder tracked and contribute to chassis mass/inertia, but have no independent Bullet constraints or collision bodies. Static/fixed collision links are welded into the chassis collision compound.
- STL collision import is limited to `.stl`; other URDF mesh formats produce a clear error. URDF visual tags are not separately rendered; collision geometry is drawn for inspection.
- V-HACD hull count is configurable per import rather than assumed to generalize from Phase 4's torus.
- Phase 4's direct chassis velocity control remains in place, with the same external-push realism tradeoff.

## Remaining Gaps / Future Work

1. Complete one actual FTC Onshape export once API credentials/access or a team-provided URDF+mesh package is available; inspect mate naming, motor/servo mappings, mass accuracy, and mesh complexity. Do not advertise drag-and-drop CAD import until then.
2. Add separate constrained rigid bodies and collision for movable imported arms/slides if mechanism contact physics is required. The current aggregate inertia updates with mechanism pose, but mechanism contacts do not.
3. Verify each imported STL's V-HACD shape and stability in the full scene; simplify high-triangle meshes when needed. The 8-hull default is conservative, not a universal guarantee.
4. Imported drive-wheel geometry mounted through fixed intermediate links needs a more general transform walk to derive track/wheelbase; those layouts currently use preset dimensions with a warning.
5. The existing `RobotConfigXml` tag resolver still covers only the device tag patterns tested in earlier phases. Test a real exported Control Hub XML for the chosen team.
