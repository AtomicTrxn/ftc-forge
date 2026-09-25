# R6 Results: CAD & Robot Configuration Pipeline

**Status:** Complete
**Task spec:** [06-cad-robot-config-pipeline.md](06-cad-robot-config-pipeline.md)
**Reads:** [03-classloading-local-directory-import.RESULTS.md](03-classloading-local-directory-import.RESULTS.md) — R3 found that hardware *naming* should come from a team's real, exportable FTC robot-configuration XML, not a bespoke scheme. This task is scoped accordingly: it is not a naming design, it's a *geometry and mass* format that must reconcile with names R3 already settled.

## Summary

URDF is the clear choice, and more decisively than the plan originally framed it: URDF's native joint-type vocabulary (`revolute`, `continuous`, `prismatic`, `fixed`, `floating`, `planar`) is an exact match for this task's requirements, not just a loose fit, and real, actively-maintained Onshape-to-URDF exporters already exist (`Rhoban/onshape-to-robot`, among others) — meaning the plan's original "Onshape/URDF/JSON" pipeline idea is not aspirational, it's assemblable today from existing open-source tooling. Per R3's finding, this format's job is strictly geometry/mass/joints — hardware naming is inherited from the real robot-configuration XML by giving each driven joint the same name string as its corresponding `HardwareMap` device, avoiding any separate naming scheme entirely.

## Format Decision

**URDF**, not a custom JSON schema.

- URDF's defined joint types are `revolute`, `continuous`, `prismatic`, `fixed`, `floating`, `planar` — this is a one-to-one match for the three types this task needs (revolute, prismatic, continuous), plus `fixed` for structural/sensor mounts at no added schema cost.
- Real CAD export tooling already exists and is maintained: `Rhoban/onshape-to-robot` (Python, uses the Onshape REST API, exports URDF/SDF/MuJoCo with STL meshes and computed mass/inertia from the solid model), plus alternatives (`onshape-urdf-exporter`, `ExportURDF` for Fusion360/Onshape/SolidWorks). This directly answers the plan's original "Onshape/URDF/JSON" framing — Onshape-to-URDF is the well-trodden path, not a hypothetical one.
- The usual objection to URDF (verbose, ROS-flavored, tedious mass/inertia authoring) is substantially blunted here because the CAD exporters compute mass/inertia automatically from the solid model — a team isn't hand-typing inertia tensors, the exporter derives them from their actual CAD geometry and material properties.
- A custom JSON schema would have to reinvent this joint vocabulary and would have no equivalent CAD-export ecosystem to lean on — strictly worse on both axes for this task's specific needs.

## Joint Type Specification

| FTC concept | URDF joint type | Key parameters |
|---|---|---|
| Drive wheel (continuous rotation) | `continuous` | `axis` (rotation axis in link frame); no position limits. |
| Arm / pivot mechanism | `revolute` | `axis`, `limit` (`lower`/`upper` in radians, `effort`, `velocity`). |
| Linear slide | `prismatic` | `axis`, `limit` (`lower`/`upper` in meters, `effort`, `velocity`). |
| Rigid structural member or fixed sensor mount | `fixed` | No motion parameters — free structural type URDF already provides. |

Each `<link>` carries its own `<inertial>` block (`mass`, `origin`, `inertia` tensor) — sourced from the CAD exporter's computed values, not hand-authored, per the Format Decision above.

## HardwareMap Mapping

**Rule: a URDF joint's `name` attribute is the same string as its corresponding `HardwareMap` device name from the team's real robot-configuration XML (R3).** No separate name-mapping table is needed — the executor resolves a driven joint directly via `hardwareMap.get(DcMotor.class, jointName)` using the URDF joint's own name. Joints with no real hardware backing (a `fixed` structural joint, for instance) simply have no corresponding `HardwareMap` lookup and are treated as pure geometry.

This keeps R3's and R6's concerns cleanly separated, exactly as R3's own recommendation anticipated: the XML still owns "what hardware exists and what it's called"; URDF owns "where it is, how it moves, and how much it weighs" — and the two files agree with each other purely by using matching name strings, with no new schema needed to reconcile them.

## Full Worked Example

4-motor Mecanum chassis + one revolute arm + one prismatic slide. Device names (`left_front_drive`, etc.) are assumed to already exist in the team's real robot-configuration XML per R3 — this URDF file only adds geometry/mass on top of them:

```xml
<?xml version="1.0"?>
<robot name="team_robot">

  <link name="chassis">
    <inertial>
      <mass value="8.5"/>
      <origin xyz="0 0 0.05"/>
      <inertia ixx="0.08" iyy="0.10" izz="0.14" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Drive wheels: joint name == HardwareMap device name from the real config XML -->
  <link name="wheel_left_front"/>
  <joint name="left_front_drive" type="continuous">
    <parent link="chassis"/>
    <child link="wheel_left_front"/>
    <origin xyz="0.18 0.15 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_right_front"/>
  <joint name="right_front_drive" type="continuous">
    <parent link="chassis"/>
    <child link="wheel_right_front"/>
    <origin xyz="0.18 -0.15 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_left_back"/>
  <joint name="left_back_drive" type="continuous">
    <parent link="chassis"/>
    <child link="wheel_left_back"/>
    <origin xyz="-0.18 0.15 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="wheel_right_back"/>
  <joint name="right_back_drive" type="continuous">
    <parent link="chassis"/>
    <child link="wheel_right_back"/>
    <origin xyz="-0.18 -0.15 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Arm: revolute, matches a "arm_motor" HardwareMap entry -->
  <link name="arm_link">
    <inertial>
      <mass value="1.2"/>
      <origin xyz="0.15 0 0"/>
      <inertia ixx="0.01" iyy="0.02" izz="0.02" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="arm_motor" type="revolute">
    <parent link="chassis"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.12"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.09" effort="10" velocity="3.0"/>
  </joint>

  <!-- Linear slide: prismatic, matches a "slide_motor" HardwareMap entry -->
  <link name="slide_link">
    <inertial>
      <mass value="0.6"/>
      <origin xyz="0 0 0.1"/>
      <inertia ixx="0.002" iyy="0.002" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="slide_motor" type="prismatic">
    <parent link="arm_link"/>
    <child link="slide_link"/>
    <origin xyz="0.3 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="0.45" effort="10" velocity="0.5"/>
  </joint>

</robot>
```

At runtime, the executor parses this file, builds the rigid-body tree for Phase 4's physics (Libbulletjme, per R2), and for each non-`fixed` joint whose `name` matches an entry in the team's real `HardwareMap` (from R3's XML), wires that joint's simulated position/velocity directly to the corresponding mock `DcMotor`/`DcMotorEx` instance — so `hardwareMap.get(DcMotor.class, "arm_motor").setPower(0.5)` in the team's actual OpMode drives exactly this joint.

## Risks / Unknowns Remaining

- This task did not verify whether `Rhoban/onshape-to-robot` or the alternative exporters handle FTC-specific hardware (goBILDA/REV motor mounts, off-the-shelf gearbox parts common in FTC CAD) cleanly, or whether teams will need to hand-edit exporter output — worth a hands-on spike with a real team's Onshape assembly before Phase 5 commits to "drag-and-drop CAD import" as a user-facing promise.
- Fixed joints (structural members, sensor mounts) are cheap in URDF but were not exercised in the worked example beyond a mention — Phase 5 should confirm the parser handles a `fixed`-only subtree correctly (no motion, but still contributes mass/inertia to the parent body).
- The name-matching rule between URDF joint names and real XML device names is a convention this project defines, not something URDF or the FTC config format natively enforces — Phase 5's importer must validate at load time that every non-`fixed` joint's name actually resolves to a real `HardwareMap` entry, and fail loudly (not silently ignore) on a mismatch.
