# Task P5: Custom Robot Importer

**Type:** Implementation
**Depends on (research):**
- [`tasks/research/06-cad-robot-config-pipeline.RESULTS.md`](../research/06-cad-robot-config-pipeline.RESULTS.md) — the format decision and full worked example to implement.
- [`tasks/research/03-classloading-local-directory-import.RESULTS.md`](../research/03-classloading-local-directory-import.RESULTS.md) — the `HardwareMap` naming convention imported configs must resolve to.
**Depends on (prior phase):** [`tasks/implementation/phase-1-core-executor-hal.RESULTS.md`](phase-1-core-executor-hal.RESULTS.md) — this phase replaces Phase 1's hand-authored presets with the full importer; read what format those presets used so the migration is backward-compatible (existing preset-based test setups shouldn't break).
**Produces:**
- Code: `core-sdk-mock/robot-import/` (URDF parser + `<transmission>` resolver, paired with the existing robot-configuration XML parser from Phase 1/R3 — this phase adds geometry/mass on top of that XML, it doesn't replace it).
- Results doc: `tasks/implementation/phase-5-custom-robot-importer.RESULTS.md`
**Plan reference:** [Next-Gen FTC Robot Simulator Plan.md](../../Next-Gen%20FTC%20Robot%20Simulator%20Plan.md), § 4 Phase 5
**Estimated duration (from plan):** 3 weeks

## Context

This is explicitly the last phase and was deliberately descoped out of the MVP — Phases 1-4 shipped and were validated using hand-authored preset configs (which, per R3/R4's findings, already carry a per-motor SKU/ratio field the real robot-configuration XML can't express). This phase generalizes that into the full CAD/URDF import pipeline R6 specified, letting any team import their actual custom robot geometry rather than picking from presets. R6 was substantially revised after its first pass — this phase's instructions below reflect the corrected design, not the original name-matching approach.

## Objective

A working URDF parser and import pipeline covering Revolute/Prismatic/Continuous joints, mass/weight distribution, and **`<transmission>`-based actuator mapping** (not a bare joint-name-to-device-name match — R6 found that fails for multi-actuator joints, servo-driven joints, and rerouted odometry encoders, all real FTC patterns). The URDF file is paired with, not a replacement for, the real robot-configuration XML from R3/Phase 1 — URDF owns geometry/mass/joints, the XML still owns what hardware exists and what it's called.

## Instructions

1. Read R6's RESULTS in full, including its worked example (now a **paired** robot-configuration XML + URDF file) — implement the parser against that exact example first, as your initial test case.
2. Read R3's RESULTS for the `HardwareMap` naming convention. Resolution is now two-step, not a direct name match: parse a `<transmission>` block's named actuator(s), then resolve each actuator name against the real `HardwareMap` (from the paired XML) exactly as Phase 1's executor already does — a transmission's `mechanicalReduction` converts between actuator units (motor ticks/velocity) and joint units (radians or meters).
3. Implement the URDF parser, covering all three joint types (Revolute, Prismatic, Continuous) with their physical parameters, plus `<transmission>` parsing for actuator resolution. Validate at load time that every actuator name in a `<transmission>` actually resolves to a real `HardwareMap` entry, and fail loudly (not silently) on a mismatch.
4. Wire mass/weight distribution from the imported config into Phase 4's rigid-body physics, so imported robots have realistic inertial behavior, not just visual geometry. Accept an optional `total_mass_kg` override that scales all link masses proportionally — R6 found that CAD-derived mass is frequently wrong (source STEP files often have no material assigned) even when geometry is right, so a real measured total should be able to correct it.
5. Apply R2/R6's Mecanum finding here too: imported wheel joints are visual/encoder-tracking only, driven by the motor model's output — do not expect wheel-ground contact on an imported robot to produce strafing any more than it does on the preset robots in Phase 4.
6. **Validate:** import R6's full worked example (4-motor Mecanum + arm + slide, paired XML + URDF) and confirm it runs identically to how an equivalent Phase 1 hand-authored preset would have — same OpMode, same `hardwareMap.get(...)` calls, same behavior.
7. Confirm backward compatibility: Phase 1's existing presets (including their SKU/ratio field) should still load (either natively, or converted once into the new format) — don't break existing setups.
8. **Before promising "drag-and-drop CAD import" as a user-facing feature**, run a hands-on spike importing one real team's actual Onshape assembly through `Rhoban/onshape-to-robot` (or an equivalent exporter). R6 flagged real friction here — the exporter needs a local Python environment, Onshape API keys, and assembly mates named with a specific convention — that a worked example alone doesn't surface. Document what a team actually has to do, not just what the pipeline supports in principle.

## Output contract

Save `tasks/implementation/phase-5-custom-robot-importer.RESULTS.md` with:

- `## Summary`
- `## Parser Implementation` — URDF coverage of joint types, plus `<transmission>` parsing.
- `## HardwareMap Resolution` — confirmation the transmission→actuator→HardwareMap chain matches R3's convention, and that mismatches fail loudly.
- `## CAD Mass Override` — how `total_mass_kg` scaling was implemented.
- `## Validation` — R6's paired-file worked example run end-to-end.
- `## Backward Compatibility` — how Phase 1's presets (including their SKU/ratio field) are handled.
- `## Real CAD Import Spike Findings` — what actually happened importing a real Onshape assembly, and what friction a team would hit.
- `## Deviations from Research`
- `## Remaining Gaps / Future Work`

## Definition of done

- [ ] R6 and R3 RESULTS read and implemented as specified.
- [ ] All three joint types parse and resolve correctly via `<transmission>`, not a bare name match.
- [ ] A multi-actuator joint (e.g. the two-motor slide from R6's transmission example) resolves correctly — this is the case the original name-matching design couldn't handle.
- [ ] R6's full paired-file worked example imports and runs correctly.
- [ ] `total_mass_kg` override scales link masses correctly.
- [ ] Phase 1's existing presets (including SKU/ratio) still work.
- [ ] A real CAD-export spike was run and its friction documented, not assumed away.
- [ ] RESULTS.md saved at the path above.
