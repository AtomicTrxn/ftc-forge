# Robot motion demo

Guided robot setup offers **Motion demo** after Physics assumptions and before Review and save. **Finish and use** offers the same demo for reused robots. Run it, watch each named motion, then return to **View demo results** to compare observed movements with your physical robot.

## What runs

| Configured capability | Demonstration |
| --- | --- |
| Differential / tank | Forward, backward, turn left, turn right |
| Mecanum | Those four movements plus strafe left and strafe right |
| Powered hinge or continuous mechanism | Short travel toward two angular targets |
| Powered slide | Short travel toward two linear targets |
| Rotational servo | Two targets inside saved servo travel and joint limits |
| Coupled follower | Moves with its powered source; no separate command |
| Fixed, passive or unbound part | No powered demo; unbound joints appear in setup notes |

Capabilities follow saved drive mode, joint types and actuator bindings. Mesh appearance cannot establish whether a wheel has Mecanum rollers or a tread. Every configured drive slot must bind a continuous wheel and an actual motor. Choose differential drive for tank behavior; this uses the simulator's existing differential model rather than a detailed tread solver.

The demo uses the prepared collision geometry, articulated joints, motors, battery, calibration and any configured tire/flexible-intake models. It applies short commands through native physics; contacts, joint limits, motor effort and passive motion can affect the result. Translation stops being powered around 0.25 m; stages last about 1.15 simulated seconds. Mechanisms aim at nearby bounded targets, not their complete range. A target is a command, not a promise that it was reached.

With a selected project, the demo uses its hardware XML and motor presets, while physical settings come from the saved robot model. Missing motor presets use generic fallback specs; the guide, window and results list the affected names. Without a project, it creates temporary generic motors and uses saved servo settings; the window and results disclose this assumption. Missing selected-project hardware is an actionable setup error. TeamCode is neither compiled nor started, and the demo does not apply settings to the project.

## Controls and results

- **Space**: pause/resume, freezing physics and zeroing motor commands.
- **S**: stop early; results remain incomplete. A completed run stays completed.
- **R**: rebuild from current settings and replay.
- **Esc**: close; unfinished runs record stopped status.
- **C**: toggle native collision outlines.
- **V**: toggle CAD visibility. Drag to orbit; scroll to zoom.

The window shows the active action and recent outcomes. Results contain actual forward/left displacement in meters, turn in degrees, or joint start/end/target in meters or radians. **No clear movement**, **unexpected direction** and **needs attention** include instructions to inspect bindings, axes, signs, gearing, limits, collisions and motor effort. Observed movement still needs your comparison with the actual robot.

The demo uses a neutral floor without field CAD or game pieces. The assembly is lifted together if its initial geometry penetrates that floor. It does not test pickup/retention, certify all collision situations or establish measured physical accuracy. Collision review remains a separate explicit step; no review proof is written by the demo. Next can defer this optional check.

## Review, correct and retest

**View demo results** opens a movement table. Select a row to read actual travel, then choose an assessment and optionally add a note:

| Assessment | Meaning |
| --- | --- |
| Not reviewed | No current explicit assessment |
| Correct | You compared the observed movement with your expected robot behavior |
| Reversed | Visible direction differs from your expectation |
| Blocked | Contacts, load or setup prevent the intended movement |
| Incorrect travel | Direction/range/amount differs from your expectation |

Only completed observations for the current settings and hardware can be assessed. **Correct** also requires native evidence of movement; commanding a target alone is insufficient. **Save reviews** stores explicit choices. Closing the dialog discards edits. Choosing Not reviewed does not erase previously saved history; retests or setting changes make old assessments stale.

**Guided correction…** offers motor bindings/gearing, joint axes/limits, collision geometry, motor effort/servo travel and drivetrain signs/dimensions. The recommended path follows your assessment; you can select another. The guide opens the relevant part or runtime settings with instructions and **Retest this movement…** / **Return to movement review** controls. Changes retain the normal draft and collision-review requirements. Mechanism demos control joint targets and compensate signed gearing; visible reversal primarily calls for checking axes. Verify gearing separately for TeamCode and encoder direction.

**Retest selected movement…** runs one selected drivetrain direction, or both targets of one mechanism with other motors stopped. A completed guide-launched demo is replaced when retesting; a running demo must first be closed with Esc. New observations require another explicit assessment, even if they moved successfully. Reviews of other movements accumulate. A setting or hardware change invalidates all reviews tied to the earlier configuration.

Raw reports live under the setup library's `sessions/motion-demos/` and are not exported. Explicit assessments and their reviewed observations are saved as `motion_review` metadata in the robot profile and included in portable bundles. They retain the CAD/settings digest, project hardware context and demo run ID. Adding feedback to a saved robot creates another immutable revision without changing physical settings or granting collision approval. Unchanged imported settings can retain current reviews; changed CAD, physics, calibration or project hardware retain historical feedback requiring retest. Guide-launched retests save the pending state with the model before launching; command-line/window replays are also detected through the newer local report.

## Command line

```sh
# Use '-' instead of a project for disclosed generic motor assumptions:
./gradlew :gui-runner:demoRobot --args='/absolute/profile.json /absolute/team-project /absolute/session/demo.json'
# Same native sequence without opening a window:
./gradlew :gui-runner:demoRobot --args='/absolute/profile.json - /absolute/session/demo.json --headless'
# Retest just one drive action or both targets of one mechanism:
./gradlew :gui-runner:demoRobot --args='/absolute/profile.json - /absolute/session/demo.json --only drive/forward'
./gradlew :gui-runner:demoRobot --args='/absolute/profile.json - /absolute/session/demo.json --headless --only joint/lift_joint'
```

The profile must have current prepared artifacts and no unresolved CAD migration choices. Report paths must be outside the model directory. See [demo evidence](../tasks/robot-motion-demo/RESULTS.md) and [guided review/correction evidence](../tasks/motion-review/RESULTS.md).
