# Guided motion review — implementation and evidence

## Delivered

- A review table at Motion demo and Finish shows each configured movement, native outcome, actual travel, review status, explicit assessment and optional note. Correct, Reversed, Blocked and Incorrect travel are user choices; there is no automatic approval.
- Guided correction offers binding/gearing, joint axes/limits, collision geometry, effort/servo travel and drivetrain signs/dimensions. It selects the affected part or runtime settings and provides instructions, return-to-review and targeted-retest controls.
- Retesting one drive direction commands only that direction. Retesting one mechanism runs both nearby targets with other motors stopped. Completed guide-owned demos can be replaced; a running demo is protected, including while an earlier finished report is still present.
- Profile `motion_review` metadata retains the reviewed observation, note, run ID, CAD/settings digest and hardware context. Feedback on saved models creates another immutable revision. Motion metadata does not change the physical digest, approve collisions or remove provisional provenance.
- Portable export/import preserves feedback. Migration retains history with its original digest/context. CAD, physics, calibration or hardware changes invalidate old assessments. New local observations and guide-launched pending retests require another explicit review; unrelated current assessments accumulate.
- Missing selected-project motor presets are disclosed by motor name in the guide and demo, including generic fallback effort assumptions.

## Automated validation

`./gradlew test` passes **122 Java cases**: GUI runner 90, physics engine 25, core SDK mock 7. `python3 -m unittest discover -s tools -p 'test_*.py'` passes **37 Python cases**. `:gui-runner:installDist` succeeds.

The existing eight motion cases remain green. Ten additional review cases cover one-direction drive execution, two-target hinge/slide/servo execution with unrelated motor commands zero, explicit review accumulation, new-run rejection of stale dialogs, missing/incomplete/no-motion rejection, immutable revisions, portable restoration without a local session report, resume, pending retests, CAD migration, settings/hardware invalidation, correction focus and completed-process replacement guards. Tests exercise native Bullet bodies and production motors, rather than animated or mocked movement.

Python cases verify that feedback leaves physical/collision identity unchanged, survives portable export/import and retains the original digest after changed-CAD migration. Two core JSON cases cover Python-style Unicode/control escapes, surrogate pairs and malformed escape rejection. The Java portable review case also uses Unicode notes through actual revision save/export/import.

## Desktop validation — distinct robot package

Actual production Swing and OpenGL entry points were tested using private macOS app wrappers and a separate library. The authored test ZIP contains five URDF links and a referenced twelve-triangle STL mesh: a differential/tank chassis, two driven wheels, a prismatic lift and a rotational servo gate. Hardware XML provides `leftDrive`, `rightDrive`, `liftMotor` and `gateServo`. This package differs from the supplied REV DUO CAD and the prior Mecanum/hinge fixture; it is synthetic, not another team's measured robot.

1. Imported the URDF/STL ZIP through the guide, checked meter dimensions (0.660 × 0.350 × 0.365 m), added differential drive and bound the gate servo through the normal controls. Remaining assumptions stayed provisional.
2. Ran the full native eight-stage demo. Four mechanism observations showed movement. Four drivetrain observations reported **no clear movement**. Forward displacement was about 0.000034 m at the default contact assumptions. The aggregate drive controller's bounded low-power demo cannot be assumed to overcome floor friction for arbitrary imported bodies; the fixture's body/floor relationship is also provisional. No commands or friction were silently increased to manufacture successful movement.
3. Selected Forward → Blocked → Guided correction. Collision geometry was recommended; selectable driven-wheel parts led to Physics assumptions focused on `wheel0`, with collision guidance and retest/return buttons. Reopening results showed the saved Blocked assessment.
4. Selected the lift's first target → Incorrect travel → Guided correction. Joint axis and travel limits was recommended, with `lift` focused in Parts and movement. Edited its upper limit from 0.08 to 0.06 m. Returning to review showed prior feedback as historical and unavailable for current assessment.
5. Used Retest selected movement. The native report contained exactly `joint/lift_joint/first` and `/second`. Actual travel was approximately 0.03868 → 0.03966 m toward 0.040 m and 0.03966 → 0.00457 m toward 0.000 m; both recorded movement. The table required fresh assessments. Saved both as Correct for this fixture's expected short travel.
6. Retested again while the first targeted demo remained completed. The guide replaced its owned completed process and produced a different native run ID; saved Correct rows became Not reviewed with the prior assessment visible. Explicitly reassessed both, adding `QA fixture — return travel within 5 mm; café 🤖` to the return observation.
7. Inspected the live collision preview (three native bodies, meter grid and shape coverage), then completed the separate collision acknowledgments. Run checks and save reviewed model succeeded with a native construction proof and immutable revision.
8. Exported through Finish, started a clean library, imported that bundle through the guide and reused its saved collision review. With the same hardware context and **no local motion report**, results showed both lift observations as Reviewed / Correct and the exact Unicode note. The earlier Blocked drivetrain observation remained historical after the physical edit.

All CAD, private libraries, app wrappers, raw reports and exported test bundles remain under ignored `.local/desktop-qa/`; they are not included in repository commits.

## Problems found and fixed

- Restoring feedback without a local report initially attempted to read a missing observation list using a strict parser. Empty report/list defaults now support portable feedback and session resume; automated and desktop round trips confirm it.
- The shared JSON reader previously treated `\u2014` as literal `u2014`, corrupting labels and non-ASCII notes after Python saved/exported settings. It now decodes Unicode and control escapes correctly and rejects malformed escapes. Unicode notes survive the full desktop round trip.
- The guide formerly described selected project motor presets as available even when individual motors used generic fallback specs. The affected names are now disclosed.

## Remaining limits

This validates the general preparation/review workflow with another **synthetic** CAD package. A second team's real CAD, measured dimensions and hardware telemetry are still needed to validate that physical robot. The blocked generic drivetrain result is retained as evidence; this change does not redesign the aggregate traction controller or calibrate contact materials. Native movement thresholds establish observation, not measured accuracy, full-range reachability, pickup/retention or comprehensive collision approval. Project path/configuration and hardware file hashes deliberately form a conservative review context; moving or changing the project can require retesting unchanged robot settings.
