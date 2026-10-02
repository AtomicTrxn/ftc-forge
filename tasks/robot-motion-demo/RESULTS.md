# Robot motion demo — results

Validated on macOS with JDK 17 and native Libbulletjme 22.0.3, 2026-10-02.

## Delivered

- Robot-only Motion demo step before Review and save; reused robots have replay/results buttons at Finish.
- Capabilities derive from effective saved drive mode, continuous wheel bindings, independent powered joints and selected hardware. Differential/tank has four stages; Mecanum has six. Powered hinge/continuous/slide/rotational-servo joints have two bounded target stages; mimic followers run with their source.
- Actual production motor/battery, articulated-body, chassis, tire and flexible-intake physics on a neutral floor. No TeamCode startup, field pieces, retention sequence or review proof writes.
- Named stages, pause/stop/replay, CAD/shape toggles and actual measured observations. Setup errors, no clear movement, wrong direction and unstable simulation are surfaced without pretending every action succeeded.
- Reports remain separate from saved physical settings. Profile digest plus project settings/hardware XML/preset hashes pin successful reports; changes require replay. Generic model-only motor assumptions are disclosed.
- [User guide](../../gui-runner/ROBOT_MOTION_DEMO.md), command-line graphical/headless entry point and reusable native tests.

## Automated evidence

`./gradlew test` passes: 80 GUI-runner cases, 25 physics-engine cases and 5 core cases, zero failures/errors. Core/physics tasks reuse their unchanged passing outputs; GUI-runner tests execute against this implementation. `:gui-runner:installDist` also succeeds.

Eight new `RobotMotionDemoTest` cases cover:

1. Actual differential forward/reverse/both turns with signed shafts.
2. Actual Mecanum forward/reverse/both strafes/both turns.
3. Actual hinge, positive-limit slide and rotational servo travel within configured bounds.
4. Pause freezes physics and zeros commands; early stop is incomplete. Completed-stop idempotency is checked in both drivetrain cases.
5. Missing drive binding does not invent motion; selected project hardware mismatch is rejected without changing sim.config.
6. Unbound mechanism notes, robot-only page selection, optional Next acknowledgment without a collision proof, and unresolved migration rejection.
7. Continuous mimic follower travels with its powered source without a second independent action.
8. Motor shafts held at a RUN_TO_POSITION target report no clear movement instead of successful animation.

The drivetrain cases also check zero commands after completion, unchanged draft files, absence of validation.json, and stale-report rejection after weight edits.

## Real CAD and desktop evidence

The existing reviewed REV DUO profile ran with its selected project's actual XML/presets and saved tire/flexible-intake settings. The headless sequence completed all six actions, each with observed native movement. Forward/reverse were approximately +0.239/−0.238 m; turns approximately +0.683/−0.684 rad; intake target stages ended near +0.595 and −0.490 rad. The lower intake joint was identified as a follower of the upper joint. Reports and CAD remain private under `.local/`.

The production desktop app was then tested with the same REV profile. A fresh corrected launch rendered the frame, wheels and intake visibly; the six actions completed with every outcome “movement observed.” The last desktop run measured +0.239/−0.238 m, +0.666/−0.667 rad turns, and intake endpoints +0.596/−0.490 rad. Collision outlines could be toggled, and pressing S after completion retained `complete=true` and `status=finished`.

A synthetic Mecanum robot with motor hinge, motor slide and servo completed all 12 stages in the desktop app. Replay, pause/resume and collision outlines were exercised. The actual GuidedSetupWizard Run button launched that production app and its session report. View demo results displayed all 12 measured observations in meters/radians/degrees. Next reached Review and save with all three collision review checkboxes unchecked, retaining the draft. Field setup did not gain a robot demo page.

## Problems found and corrected

- The first mechanism controller stopped commanding at the target, allowing slide recoil before the reverse stage. Bounded proportional/damping control now holds the target through each stage; native hinge/slide/servo tests verify actual travel.
- Reading a static floor's linear velocity triggered a native assertion. Stability checks now inspect velocities only for dynamic bodies.
- Initially disabling the chassis controller when tires were installed also disabled the shared tire tick. Drive demos now keep that tick enabled, allowing the real REV tire model to drive the chassis.
- Desktop testing caught Next lacking the new motion acknowledgment. A dedicated optional handoff now acknowledges the step without collision approval; both controller and actual desktop flow were verified.
- Stopping a finished sequence originally downgraded its report. Stop is now idempotent after completion, verified in native tests and the desktop REV run.
- Detached native construction had already updated scene light caches, causing a black first render. Attaching lights directly to the demo root refreshes those caches, as in the existing preparation preview. A fresh REV launch verified visible CAD detail before replay.
- Mechanism “positive/reverse” labels were misleading when passive motion had already carried a joint beyond its first target. Labels now name the actual target and unit. Results show start/end/target separately; observed movement does not imply the target was reached.

## Scope

This is a setup comparison using existing physical models. It does not infer tread/roller mechanics from a mesh, exercise full joint ranges, validate pickup/retention, replace collision review, or establish measured real-world accuracy. Generic motors and floor friction are demonstration assumptions, explicitly disclosed. Model physical settings and calibration remain reusable/exportable; motion reports are local session observations.
