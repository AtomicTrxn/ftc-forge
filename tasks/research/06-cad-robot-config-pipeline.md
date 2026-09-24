# Task R6: CAD & Robot Configuration Pipeline

**Type:** Research
**Depends on:** [`tasks/research/03-classloading-local-directory-import.RESULTS.md`](03-classloading-local-directory-import.RESULTS.md) — read this first for the finalized `HardwareMap` naming convention; imported robot configs must map onto it exactly.
**Produces:** `tasks/research/06-cad-robot-config-pipeline.RESULTS.md`
**Plan reference:** [Next-Gen FTC Robot Simulator Plan.md](../../Next-Gen%20FTC%20Robot%20Simulator%20Plan.md), § 3 Step 6

## Context

This is the full custom-robot-import pipeline (URDF/CAD), scoped for **Phase 5**, not the MVP. Phases 1–4 ship with hand-authored preset configs instead (see Phase 1's spec) — this task defines the general pipeline that eventually replaces those presets, so it should be thorough, but it is explicitly not on the MVP critical path.

## Objective

Specify how teams will import custom robot geometry, mass properties, and motor/mechanism configuration.

## Instructions

1. **Evaluate formats:** full **URDF** (Unified Robot Description Format, from ROS) vs. a simplified custom JSON schema. Weigh: URDF is a standard with existing tooling (e.g. exporters from Onshape/SolidWorks) but is verbose and ROS-flavored; a custom JSON schema is easier to hand-author and parse but has no ecosystem tooling. Recommend one, with reasoning.
2. **Define joint relationships** the schema must express: Revolute (pivots, arms), Prismatic (linear slides), Continuous (drive wheels). For each, specify what geometric/physical parameters are needed (axis, range of motion, mass, moment of inertia where applicable).
3. **Map imported joint/motor definitions onto the `HardwareMap` naming convention finalized in R3** — e.g. a JSON node named `"left_front_drive"` must resolve to the exact same lookup a mocked `hardwareMap.get(DcMotor.class, "left_front_drive")` call expects. Give a complete worked example: a 4-motor Mecanum chassis + one arm (revolute) + one slide (prismatic), from CAD/JSON source through to how the executor resolves it at runtime.
4. Note explicitly that this pipeline targets Phase 5, and is not required for Phases 1-4, which use static hand-authored presets (see Phase 1's spec for the preset format expected there).

## Output contract

Save to `tasks/research/06-cad-robot-config-pipeline.RESULTS.md`:

- `## Summary`
- `## Format Decision` — URDF vs. custom JSON, with reasoning.
- `## Joint Type Specification` — Revolute/Prismatic/Continuous, parameters for each.
- `## HardwareMap Mapping` — worked example tying back to R3's naming convention.
- `## Full Worked Example` — complete config for a 4-motor Mecanum + arm + slide robot.
- `## Risks / Unknowns Remaining`

## Definition of done

- [ ] R3's results read; naming convention matches exactly.
- [ ] Format decision made (not left open).
- [ ] All three joint types specified with concrete parameters.
- [ ] A complete, runnable-looking worked example included.
- [ ] RESULTS.md saved at the path above.
