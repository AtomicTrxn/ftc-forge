# Task P5: Custom Robot Importer

**Type:** Implementation
**Depends on (research):**
- [`tasks/research/06-cad-robot-config-pipeline.RESULTS.md`](../research/06-cad-robot-config-pipeline.RESULTS.md) — the format decision and full worked example to implement.
- [`tasks/research/03-classloading-local-directory-import.RESULTS.md`](../research/03-classloading-local-directory-import.RESULTS.md) — the `HardwareMap` naming convention imported configs must resolve to.
**Depends on (prior phase):** [`tasks/implementation/phase-1-core-executor-hal.RESULTS.md`](phase-1-core-executor-hal.RESULTS.md) — this phase replaces Phase 1's hand-authored presets with the full importer; read what format those presets used so the migration is backward-compatible (existing preset-based test setups shouldn't break).
**Produces:**
- Code: `core-sdk-mock/robot-import/` (URDF/JSON parser).
- Results doc: `tasks/implementation/phase-5-custom-robot-importer.RESULTS.md`
**Plan reference:** [Next-Gen FTC Robot Simulator Plan.md](../../Next-Gen%20FTC%20Robot%20Simulator%20Plan.md), § 4 Phase 5
**Estimated duration (from plan):** 3 weeks

## Context

This is explicitly the last phase and was deliberately descoped out of the MVP — Phases 1-4 shipped and were validated using hand-authored preset configs. This phase generalizes that into the full CAD/URDF import pipeline R6 specified, letting any team import their actual custom robot geometry rather than picking from presets.

## Objective

A working parser and import pipeline for the format R6 chose (URDF or custom JSON), covering motor locations, slide mechanics, and mass/weight distribution, fully wired into the executor via the `HardwareMap` naming convention from R3.

## Instructions

1. Read R6's RESULTS in full, including its worked example — implement the parser against that exact example first, as your initial test case.
2. Read R3's RESULTS for the `HardwareMap` naming convention, and confirm every parsed joint/motor resolves to it identically to how Phase 1's hand-authored presets did.
3. Implement the parser for the chosen format (URDF or JSON per R6), covering all three joint types (Revolute, Prismatic, Continuous) with their physical parameters.
4. Wire mass/weight distribution from the imported config into Phase 4's rigid-body physics, so imported robots have realistic inertial behavior, not just visual geometry.
5. **Validate:** import R6's full worked example (4-motor Mecanum + arm + slide) and confirm it runs identically to how an equivalent Phase 1 hand-authored preset would have — same OpMode, same `hardwareMap.get(...)` calls, same behavior.
6. Confirm backward compatibility: Phase 1's existing presets should still load (either natively, or converted once into the new format) — don't break existing setups.

## Output contract

Save `tasks/implementation/phase-5-custom-robot-importer.RESULTS.md` with:

- `## Summary`
- `## Parser Implementation` — format handled, coverage of joint types.
- `## HardwareMap Resolution` — confirmation it matches R3's convention.
- `## Validation` — R6's worked example run end-to-end.
- `## Backward Compatibility` — how Phase 1's presets are handled.
- `## Deviations from Research`
- `## Remaining Gaps / Future Work`

## Definition of done

- [ ] R6 and R3 RESULTS read and implemented as specified.
- [ ] All three joint types parse and resolve correctly.
- [ ] R6's full worked example imports and runs correctly.
- [ ] Phase 1's existing presets still work.
- [ ] RESULTS.md saved at the path above.
