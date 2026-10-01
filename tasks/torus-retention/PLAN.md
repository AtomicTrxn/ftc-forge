# Reliable physical torus retention

## Goal
Capture the illustrative 100 g torus, carry it through straight motion and a turn, retain it during a stopped intake, and release it deliberately by reversing. The game piece must stay a native dynamic body with collision response throughout.

## Plan
1. Reproduce capture failure on the supplied CAD and inspect piece/shaft trajectories and contact geometry. Check approach speed against actual roller surface speed, motor mounting signs, ramp support and containment bounds.
2. Correct evidenced geometry or control/model errors. Prefer actual contact and existing CAD surfaces. If retention needs an explicit approximation, make its physical limits and activation conditions visible in configuration and documentation; validate contact before acquisition and retain motor loading.
3. Add a deterministic native retention probe with hard gates for capture, carrying, stopped retention and reverse release. Exercise initial offset/phase variation and repeat capture/release. Check that a nearby uncontacted piece and an inactive/reversed intake cannot be acquired.
4. Add regression tests for any model changes, run all existing Java/Python checks and legacy powered-CAD behavior. Run real FTC TeamCode in the renderer and inspect the retained piece and release.
5. Update the preparation profile, sample TeamCode, guide and evidence/results. Commit, push, merge a PR and update the local prepared project/launchers.

## Success criteria
- Acquisition is observed and sustained, not inferred from a one-frame reporting-region crossing.
- Retained piece stays inside a declared physical envelope with finite pose/speed and bounded drift during carrying/turning and stopped intake.
- Reverse releases outward and cannot immediately reacquire; a second forward command can acquire again after real contact.
- No teleportation, body removal, velocity overwrite or unconditional distance-triggered capture.
- Source CAD remains unchanged; any approximation is identified separately from measured hardware accuracy.

## Outcome

Implementation and validation are complete. The contact-triggered compliant pinch/contact-compression approximation passes sustained capture, carry/turn, zero-power BRAKE/FLOAT retention, reverse, repeated cycles, phase/offset variations, stalled/inactive acquisition rejection and overload breakaway. See [results and remaining accuracy limits](RESULTS.md).
