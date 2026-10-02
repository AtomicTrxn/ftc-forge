# Independent wheel grip calibration

## Why this phase

Native support now distinguishes wheels from belly/obstacle contacts, but every welded
wheel still inherits chassis friction. A team cannot tune rubber traction without
changing scraping friction. Separate, portable wheel grip is the next bounded physics
improvement for importing a different robot.

## Implementation

1. Add optional `drive_contacts.wheel_friction` entries keyed by a saved drive joint:
   `{ "joint": "left_wheel", "friction": 0.9 }`. Allow 0–2, reject duplicate/invalid
   targets; absent entries inherit the chassis. Existing profiles keep their behavior.
2. Apply the selected wheel coefficient to native support grip budgets, aggregate
   drive and optional differential tires. Apply it to native wheel/obstacle friction
   too; retain the chassis material for belly contacts. Keep restitution and native
   rolling/spinning coefficients on the welded body and explain this scope.
3. Add a guided **Wheel grip** action with configured wheel choices, explicit inheritance,
   dimensionless coefficient, and instructions for measuring/reviewing grip. Save edits
   into a draft, invalidate prior review/motion evidence, and expose provenance.
4. Preserve settings through export/import and identical CAD reuse. Remap unique renamed
   joints automatically; missing joints require selectable replacement or removal of
   that override, retaining other wheel settings and native support mode.
5. Verify native behavior: different wheel vs chassis grip, per-wheel support budgets,
   zero-grip wheels/floor, obstacle friction, tire shaft reaction, and legacy inheritance.
   Test invalid settings and guided save/export/import/migration/review flows. Run the
   full Java/Python suite and existing real REV physics/retention probes; document evidence.
6. Update user docs, commit, push, and ship the reviewed branch.

## Acceptance and limits

- A wheel can be grippy while the belly is slippery (or the reverse), without inventing
  support or changing mass/inertia.
- Saved settings are editable, portable and require review after CAD/settings changes.
- No automatic rubber measurement: coefficients multiply the contacted surface friction.
  Teams must measure the robot/field pair; motion demos alone establish no physical accuracy.
- This phase retains welded wheel proxies and a level chassis. It does not add rolling
  bodies, Mecanum rollers, suspension, slope traction or a measured tire-curve fitter.

## Progress

- [x] Inspect remaining gaps and select this scope.
- [x] Implement physics/settings/guided controls and migration.
- [x] Validate behavior, portability and regression gates.
- [x] Document results.
- [x] Publish tested branch and pull request ([#19](https://github.com/AtomicTrxn/ftc-forge/pull/19)).
