# R6 Results: CAD & Robot Configuration Pipeline

**Status:** Complete (amended after Opus review pass — see changelog)
**Task spec:** [06-cad-robot-config-pipeline.md](06-cad-robot-config-pipeline.md)
**Reads:** [03-classloading-local-directory-import.RESULTS.md](03-classloading-local-directory-import.RESULTS.md) — R3 found that hardware *naming* should come from a team's real, exportable FTC robot-configuration XML, not a bespoke scheme. This task is scoped accordingly: it is not a naming design, it's a *geometry and mass* format that must reconcile with names R3 already settled.

**Changelog:** replaced the "joint name = device name" mapping rule, which cannot express two motors driving one joint, servo-driven joints, or dead-wheel encoders rerouted through a drive motor's port — all real, common FTC patterns — with URDF's own `<transmission>` element, which handles all three; added a mecanum-wheel extension, since a plain `continuous` joint cannot strafe in Bullet; added missing `<inertial>`/`<collision>` blocks to the worked example's wheel links (the checklist requires a "runnable-looking" example, and massless, collision-less links aren't that); added the paired robot-configuration XML fragment the worked example was missing; clarified that R4's motor model, not URDF's `effort`/`velocity` limits, is the sole source of torque.

## Summary

URDF is the clear choice, and more decisively than the plan originally framed it: URDF's native joint-type vocabulary (`revolute`, `continuous`, `prismatic`, `fixed`, `floating`, `planar`) is an exact match for this task's requirements, not just a loose fit, and real, actively-maintained Onshape-to-URDF exporters already exist (`Rhoban/onshape-to-robot`, among others) — meaning the plan's original "Onshape/URDF/JSON" pipeline idea is not aspirational, it's assemblable today from existing open-source tooling. Per R3's finding, this format's job is strictly geometry/mass/joints — hardware naming is inherited from the real robot-configuration XML, connected to URDF joints via `<transmission>` elements (below) rather than by name-string coincidence, which is more robust and covers real multi-actuator/servo/rerouted-encoder cases the original name-matching rule couldn't.

## Format Decision

**URDF**, not a custom JSON schema.

- URDF's defined joint types are `revolute`, `continuous`, `prismatic`, `fixed`, `floating`, `planar` — this is a one-to-one match for the three types this task needs (revolute, prismatic, continuous), plus `fixed` for structural/sensor mounts at no added schema cost.
- Real CAD export tooling already exists and is maintained: `Rhoban/onshape-to-robot` (Python, uses the Onshape REST API, exports URDF/SDF/MuJoCo with STL meshes and computed mass/inertia from the solid model), plus alternatives (`onshape-urdf-exporter`, `ExportURDF` for Fusion360/Onshape/SolidWorks). This directly answers the plan's original "Onshape/URDF/JSON" framing — Onshape-to-URDF is the well-trodden path, not a hypothetical one.
- The usual objection to URDF (verbose, ROS-flavored, tedious mass/inertia authoring) is substantially blunted here because the CAD exporters compute mass/inertia automatically from the solid model — a team isn't hand-typing inertia tensors, the exporter derives them from their actual CAD geometry and material properties.
- A custom JSON schema would have to reinvent this joint vocabulary and would have no equivalent CAD-export ecosystem to lean on — strictly worse on both axes for this task's specific needs.

## Joint Type Specification

| FTC concept | URDF joint type | Key parameters |
|---|---|---|
| Drive wheel (continuous rotation) | `continuous` | `axis` (rotation axis in link frame); no position limits. See the Mecanum Drivetrain Extension below — a plain `continuous` joint alone does not produce strafing motion. |
| Arm / pivot mechanism | `revolute` | `axis`, `limit` (`lower`/`upper` in radians). `effort`/`velocity` are not the torque source — see Joint Limits Ownership below. |
| Linear slide | `prismatic` | `axis`, `limit` (`lower`/`upper` in meters). Same `effort`/`velocity` caveat. |
| Rigid structural member or fixed sensor mount | `fixed` | No motion parameters — free structural type URDF already provides. |

Each `<link>` carries its own `<inertial>` block (`mass`, `origin`, `inertia` tensor) — sourced from the CAD exporter's computed values, not hand-authored, per the Format Decision above — **and** a `<collision>` geometry, without which Phase 4's physics engine has nothing to collide against (see Full Worked Example below, which the original pass omitted for the wheel links).

## HardwareMap Mapping

**Revised: not a bare name-matching rule.** The original design ("a URDF joint's `name` attribute is the same string as its `HardwareMap` device name") cannot express several common, real FTC patterns:
- **Two actuators driving one joint** (e.g. `slide_left` and `slide_right` both moving one linear-slide mechanism).
- **Servo-driven joints** (a wrist or claw) — the original rule only mentioned `DcMotor`.
- **Dead-wheel odometry pods read through a drive motor's encoder port** — a real, common Road-Runner-quickstart pattern where an odometry pod's encoder is wired into an unused encoder port on a *drive* motor's connector, so the encoder reading for one joint (odometry) is registered under a different joint's (drivetrain) `HardwareMap` name entirely. A 1:1 joint-name-to-device-name rule cannot represent this at all.

**Replacement: URDF's own `<transmission>` element**, which exists specifically to map one or more named actuators onto a joint (with a mechanical reduction each), rather than assuming a 1:1 name coincidence:

```xml
<transmission name="slide_tx">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="slide"/>
  <actuator name="slide_left"><mechanicalReduction>52.6</mechanicalReduction></actuator>
  <actuator name="slide_right"><mechanicalReduction>52.6</mechanicalReduction></actuator>
</transmission>
```

Here, `slide` is the URDF joint's own free-standing name (no longer required to match any `HardwareMap` string), and each `<actuator name="...">` names a real `HardwareMap` device the executor resolves via `hardwareMap.get(DcMotor.class, actuatorName)`. `mechanicalReduction` (illustrative value above, not sourced from a real robot) converts between the actuator's native units (motor ticks/velocity) and the joint's own units (radians for revolute, meters for prismatic) — e.g. for a slide, this is effectively `1 / spool_radius`. A dead-wheel odometry pod rerouted through a drive motor's port is handled the same way: a `<transmission>` naming that drive motor's `HardwareMap` name as its actuator, feeding a *separate* odometry joint that has no independent motor of its own.

Standard URDF-consuming tools ignore elements they don't recognize, so this project's own parser can freely add non-standard attributes (e.g. binding an actuator to a specific R4 motor SKU/ratio) without breaking compatibility with any other URDF tooling in the pipeline (the CAD exporters, primarily).

This keeps R3's and R6's concerns cleanly separated, exactly as R3's own recommendation anticipated: the XML still owns "what hardware exists and what it's called"; URDF owns "where it is, how it moves, and how much it weighs"; and `<transmission>` is the (now more capable) bridge between the two, rather than a bare name-string coincidence.

## Mecanum Drivetrain Extension

**A plain `continuous` joint, as modeled above, cannot strafe** — per R2's finding, standard rigid-body vehicle physics (including Libbulletjme/Bullet) has no native concept of a Mecanum wheel's 45°-angled rollers, which is exactly what produces sideways motion from purely-rotating wheels. Consistent with R2's resolution (drivetrain forces are computed by this project's own kinematics/motor model and applied directly to the chassis, not through simulated wheel-ground contact): the URDF wheel joints above should be treated as **visual/encoder-tracking geometry only** — their rotation is driven by (and purely reflects) the motor model's simulated encoder position, not the other way around, and chassis motion comes from the kinematics model's force/torque output applied at the chassis link, not from Bullet's contact response at the wheels. Phase 4/5 should not attempt to make the wheel-ground contact itself produce correct strafing behavior.

## Joint Limits Ownership

URDF's `<joint>` `effort`/`velocity` attributes are **not** the source of truth for how much torque a joint can actually produce — that's R4's motor model (via the `<transmission>`'s actuator, above), which already knows the specific motor SKU/ratio and its real stall-torque limits. Treat URDF's `effort`/`velocity` as, at most, a hard safety clamp (or simply informational/omit them), never as an independent torque source that could disagree with R4's math.

## Full Worked Example

4-motor Mecanum chassis + one revolute arm + one prismatic slide, now including the `<inertial>`/`<collision>` blocks the original pass omitted (a massless, collision-less link is not "runnable-looking," which this task's own checklist requires) and `<transmission>`-based hardware mapping in place of the withdrawn name-matching rule. Wheel and link dimensions below are illustrative, not sourced from a real robot.

**Paired robot-configuration XML** (per R3 — establishes the four device names the transmissions below reference; this is the piece the original pass described in prose but never actually showed alongside the URDF):

```xml
<Robot type="FirstInspires-FTC">
  <LynxUsbDevice name="Control Hub" serialNumber="(embedded)">
    <LynxModule name="Control Hub" port="173">
      <goBILDA5202SeriesMotor name="left_front_drive" port="0"/>
      <goBILDA5202SeriesMotor name="right_front_drive" port="1"/>
      <goBILDA5202SeriesMotor name="left_back_drive" port="2"/>
      <goBILDA5202SeriesMotor name="right_back_drive" port="3"/>
      <goBILDA5202SeriesMotor name="arm_motor" port="0" bus="1"/>
      <goBILDA5202SeriesMotor name="slide_motor" port="1" bus="1"/>
    </LynxModule>
  </LynxUsbDevice>
</Robot>
```

**URDF geometry/mass/joints file:**

```xml
<?xml version="1.0"?>
<robot name="team_robot">

  <link name="chassis">
    <inertial>
      <mass value="8.5"/>
      <origin xyz="0 0 0.05"/>
      <inertia ixx="0.08" iyy="0.10" izz="0.14" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <collision>
      <origin xyz="0 0 0.05"/>
      <geometry><box size="0.40 0.35 0.15"/></geometry>
    </collision>
  </link>

  <!-- Drive wheels: joint names are now free-standing (no longer required to match a HardwareMap name) -->
  <link name="wheel_left_front">
    <inertial>
      <mass value="0.15"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.0002" iyy="0.0003" izz="0.0002" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry><cylinder radius="0.048" length="0.04"/></geometry>
    </collision>
  </link>
  <joint name="wheel_lf" type="continuous">
    <parent link="chassis"/>
    <child link="wheel_left_front"/>
    <origin xyz="0.18 0.15 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  <transmission name="wheel_lf_tx">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="wheel_lf"/>
    <actuator name="left_front_drive"><mechanicalReduction>1.0</mechanicalReduction></actuator>
  </transmission>

  <!-- (right_front_drive, left_back_drive, right_back_drive wheels follow the same wheel+joint+transmission pattern, omitted for brevity) -->

  <!-- Arm: revolute -->
  <link name="arm_link">
    <inertial>
      <mass value="1.2"/>
      <origin xyz="0.15 0 0"/>
      <inertia ixx="0.01" iyy="0.02" izz="0.02" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <collision>
      <origin xyz="0.15 0 0"/>
      <geometry><box size="0.30 0.05 0.05"/></geometry>
    </collision>
  </link>
  <joint name="arm" type="revolute">
    <parent link="chassis"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.12"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.09" effort="10" velocity="3.0"/>
  </joint>
  <transmission name="arm_tx">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="arm"/>
    <actuator name="arm_motor"><mechanicalReduction>1.0</mechanicalReduction></actuator>
  </transmission>

  <!-- Linear slide: prismatic -->
  <link name="slide_link">
    <inertial>
      <mass value="0.6"/>
      <origin xyz="0 0 0.1"/>
      <inertia ixx="0.002" iyy="0.002" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <collision>
      <origin xyz="0 0 0.1"/>
      <geometry><box size="0.06 0.06 0.20"/></geometry>
    </collision>
  </link>
  <joint name="slide" type="prismatic">
    <parent link="arm_link"/>
    <child link="slide_link"/>
    <origin xyz="0.3 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="0.45" effort="10" velocity="0.5"/>
  </joint>
  <transmission name="slide_tx">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="slide"/>
    <actuator name="slide_motor"><mechanicalReduction>52.6</mechanicalReduction></actuator>
  </transmission>

</robot>
```

At runtime, the executor parses both files, builds the rigid-body tree for Phase 4's physics (Libbulletjme, per R2), and for each `<transmission>`, resolves its named actuator(s) against the team's real `HardwareMap` (from the paired XML) — so `hardwareMap.get(DcMotor.class, "arm_motor").setPower(0.5)` in the team's actual OpMode drives the `arm` joint via the `arm_tx` transmission's `1.0` reduction, and `slide_motor` drives the `slide` joint scaled by its `52.6` reduction (converting motor rotation to linear travel). The four drive-wheel joints follow R2's Mecanum extension above: their rotation reflects the motor model's encoder output for visualization, but chassis motion itself comes from the kinematics model's force/torque applied at the chassis link, not from Bullet's wheel-contact physics.

## Risks / Unknowns Remaining

- This task did not verify whether `Rhoban/onshape-to-robot` or the alternative exporters handle FTC-specific hardware (goBILDA/REV motor mounts, off-the-shelf gearbox parts common in FTC CAD) cleanly, or whether teams will need to hand-edit exporter output. Specifically for `onshape-to-robot`: it requires a local Python environment, Onshape API keys, and (per its own convention) assembly mates named with a `dof_` prefix to identify which mates become joints — real setup friction against this project's "low-friction setup" goal, not a one-click experience. The other two exporters named (`onshape-urdf-exporter`, `ExportURDF`) were identified by name only and not independently inspected — verify their exact capabilities (or drop them from consideration) before citing them as options in a Phase 5 spec.
- **CAD-derived mass should not be trusted as automatically accurate.** Imported goBILDA/REV CAD parts (STEP files from vendor sites) frequently have no material assigned in the source model, which makes exporter-computed masses wrong even though the geometry is right. The importer should accept an optional measured `total_mass_kg` override and scale all link masses proportionally to match it — CAD gives good relative mass *distribution*, a scale on a real postal scale gives the correct *total*.
- Fixed joints (structural members, sensor mounts) are cheap in URDF but were not exercised in the worked example beyond a mention — Phase 5 should confirm the parser handles a `fixed`-only subtree correctly (no motion, but still contributes mass/inertia to the parent body).
- The `<transmission>`-based mapping between actuator names and real XML device names is a convention this project defines, not something URDF or the FTC config format natively enforces — Phase 5's importer must validate at load time that every actuator name actually resolves to a real `HardwareMap` entry, and fail loudly (not silently ignore) on a mismatch.
- **Frames and units were not specified in the original pass.** Recommend following URDF's own standard convention (ROS REP-103): meters, kilograms, radians, x-forward/y-left/z-up. This is a different convention from either Road Runner's (inches, center-origin) or Pedro Pathing's (inches, 0–144 corner-origin) field-coordinate systems — the renderer/localizer layer is responsible for that conversion; it should never leak into the URDF file itself.
