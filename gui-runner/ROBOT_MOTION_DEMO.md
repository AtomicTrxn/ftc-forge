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

With a selected project, the demo uses its hardware XML and motor presets, while physical settings come from the saved robot model. Without a project, it creates temporary generic motors and uses saved servo settings; the window and results disclose this assumption. Missing selected-project hardware is an actionable setup error. TeamCode is neither compiled nor started, and the demo does not apply settings to the project.

## Controls and results

- **Space**: pause/resume, freezing physics and zeroing motor commands.
- **S**: stop early; results remain incomplete. A completed run stays completed.
- **R**: rebuild from current settings and replay.
- **Esc**: close; unfinished runs record stopped status.
- **C**: toggle native collision outlines.
- **V**: toggle CAD visibility. Drag to orbit; scroll to zoom.

The window shows the active action and recent outcomes. Results contain actual forward/left displacement in meters, turn in degrees, or joint start/end/target in meters or radians. **No clear movement**, **unexpected direction** and **needs attention** include instructions to inspect bindings, axes, signs, gearing, limits, collisions and motor effort. Observed movement still needs your comparison with the actual robot.

The demo uses a neutral floor without field CAD or game pieces. The assembly is lifted together if its initial geometry penetrates that floor. It does not test pickup/retention, certify all collision situations or establish measured physical accuracy. Collision review remains a separate explicit step; no review proof is written by the demo. Next can defer this optional check.

Reports live under the setup library's `sessions/motion-demos/`, separate from profiles and portable bundles. Model edits or changed project configuration/hardware invalidate earlier observations; replay with the current setup. Physical parameters and calibration continue to be saved/exported with the model itself.

## Command line

```sh
# Use '-' instead of a project for disclosed generic motor assumptions:
./gradlew :gui-runner:demoRobot --args='/absolute/profile.json /absolute/team-project /absolute/session/demo.json'
# Same native sequence without opening a window:
./gradlew :gui-runner:demoRobot --args='/absolute/profile.json - /absolute/session/demo.json --headless'
```

The profile must have current prepared artifacts and no unresolved CAD migration choices. Report paths must be outside the model directory. See [implementation and validation evidence](../tasks/robot-motion-demo/RESULTS.md).
