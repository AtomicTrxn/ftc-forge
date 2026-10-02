# Guided motion correction and retest

## Goal

Close the import demo loop: inspect actual native movement, record whether it matches the real robot, correct the relevant setup, retest, and retain the review with reusable robot settings.

## Implementation plan

1. Give every scripted movement a stable action ID and every drivetrain movement / independent mechanism a retest group. A selected drivetrain action runs alone; a selected mechanism runs its two targets so reverse travel has a meaningful starting point. Keep native commands, limits, pause/stop/replay and safety checks.
2. Replace the plain observation dialog with a per-movement review table. Offer Not reviewed, Correct, Reversed, Blocked and Incorrect travel. Show actual units, native outcomes, saved assessment and current/stale/unreviewed status. Save only explicit assessments of current completed observations; never automatically turn a retest into approval.
3. Add selectable correction paths for binding/gearing, axes/limits, collision geometry, motor/servo effort and drivetrain settings. Open the relevant part/runtime settings with instructions and a return/retest action. Edits use the existing draft and collision-review workflow.
4. Store motion review metadata with the robot profile, separate from its physical digest and collision proof. Preserve saved revisions by creating another revision when adding a review. Pin each reviewed observation to CAD/settings digest, hardware context and demo run ID. Preserve prior entries through export/import and migration; changed CAD/settings/hardware require retest. A newer observation of the same action requires fresh user assessment.
5. Test real native targeted drive/mechanism retests, correction selection, explicit assessments, stopped/missing/stale observation rejection, unchanged prior revisions, portable round trips, migration invalidation and partial-review accumulation. Validate a distinct packaged robot through import/setup/demo/correction/retest/save/export/import, and exercise the actual desktop flow. Document the evidence and ship through a merged PR.

## Boundaries

Motion reviews record user expectations; they are not collision approval or measured physical accuracy. Unchanged portable settings may retain review; changed CAD is retained as historical information requiring another check. A distinct test robot package verifies general import behavior; a second team's physical CAD or real telemetry must be supplied before claiming validation against that hardware.

## Completion

- [x] Stable movement IDs and targeted native retests.
- [x] Explicit assessments, notes and stale/current status.
- [x] Selectable corrections with part focus, instructions and retest/return controls.
- [x] Immutable saved revisions, portable feedback and migration invalidation.
- [x] Automated and desktop validation, including a distinct URDF/STL package.

See [results and remaining limits](RESULTS.md).
