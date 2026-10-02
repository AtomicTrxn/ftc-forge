# Robot measurement guide — results

Date: 2026-10-02

Implemented the authorized first calibration step: manual operating weight, wheel diameter/center spacing, and motor-binding instructions. Telemetry fitting and response comparison remain separate.

## Delivered

- **Measure robot and verify motors…** opens an optional guide from robot Physics assumptions and Finish.
- Instructions cover installed battery/components, empty robot weight, tread diameter versus radius, center-to-center track, Mecanum wheelbase, physical shaft signs, output-shaft gearing, named project slots and deferred hardware mapping.
- Explicit section selection distinguishes confirmed manual measurements from existing CAD/default values. kg/lb and mm/cm/in selectors convert displayed values. Selected numbers must be positive and finite; errors preserve the form for correction.
- Saved values retain manual-measurement provenance in reusable model profiles. Existing bindings, unselected settings and recording-derived calibration are preserved. Updating a saved model creates a draft and invalidates scene/project checks; renewed native checking and review remain required.
- Differential dimensions use existing drive settings. Explicit Mecanum `drive_geometry` overrides feed the renderer's actual wheel radii and turning dimensions; absence preserves CAD inference, including distinct cylinder radii, and the established fallback dimensions. Adding differential drive from either editor clears the incompatible Mecanum override.
- The motor table lists binding, signed ratio and project-name availability. **Save and open motor bindings** hands off to the existing project/joint/hardware selectors. Passive mimic followers show their coupled source instead of requesting another motor. Unavailable hardware is deferred so manual measurements remain usable.
- Native model preview/check now applies the same total-mass override as runtime and scene preflight. Generic renderer chassis mass also honors an explicit total mass.
- The [user guide](../../gui-runner/ROBOT_MEASUREMENTS.md) is linked from README, model preparation and calibration documentation.

## Validation

| Check | Evidence |
| --- | --- |
| Java suite | 100 passing tests: 5 core SDK, 25 physics, 70 GUI/controller/native. `.local/models/measurement-tests.log`, final GUI rerun `.local/models/measurement-final-tests.log` and JUnit XML. |
| New measurement tests | Five tests cover unit conversion/selective updates, calibration preservation, invalid input without profile mutation, ambiguous drive configurations, original revision preservation, review/scene invalidation, native body mass, portable export/import/resume, CAD/default Mecanum inference and changed turning calculations, passive chain followers and deferred unreadable hardware. |
| Native weight and portability | Test model native bodies sum to 9 kg after the override; exported/imported reviewed profile retains mass, geometry, provenance and the existing battery calibration. |
| Real REV powered physics | Forward 1.2088 m, intake containment, reverse release and turn yaw 0.7186 passed. `.local/models/measurement-powered.log`. |
| Real torus retention | Acquisition/carry/turn/reverse scenarios and inactive/reverse/coasting/stalled/overload gates passed. `.local/models/measurement-retention.log`. |

Actual desktop checks used the Computer tool and the ignored QA app bundles running production entry points:

1. Opened the guide from a reviewed model's Finish page. Cancel preserved review/settings.
2. Visually inspected layout. Initial long instructions widened the form and hid motor columns; the corrected guide tracks viewport width, wraps instructions and scrolls vertically. Final screenshot starts at step 1.
3. Selected operating weight, entered -1, received an actionable error, and corrected the retained input. Converted 9.0718474 kg to 20 lb; switched mm to inches and inspected converted diameter/spacing values.
4. Entered **synthetic QA values** of 20 lb, 4 in diameter, 16 in track and 18 in wheelbase. Save and open bindings produced an unreviewed draft with 9.0718474 kg, radius 0.0508 m, track 0.4064 m and wheelbase 0.4572 m. These are test values, not measurements of the user's robot.
5. Selected a test project and bound the movable wheel to `left_front_drive` through the existing dropdown. Reopened the guide; its row showed the selected name, ratio 1 and **Name found in project**. Runtime settings showed measured provenance. Inspected unchanged CAD/native geometry, ran the native check, saved a separate reviewed revision and completed Finish.
6. Inspected the actual reviewed REV DUO profile's guide: 2.727384 kg prepared mass, 90 mm diameter, 380.996749 mm track, no Mecanum wheelbase control, `leftDrive`/`rightDrive` with -1/+1 shaft signs and existing wheel/intake bindings. The initial unbound lower intake row exposed a missing follower distinction. The corrected final guide shows **Passive follower → Coupled to intake_upper_joint** and the supported 1:1 tire-gearing constraint. Cancel retained the real profile unchanged.

Private QA evidence: `.local/desktop-qa/models/sessions/484b2443-f246-4f6f-88a5-75cc1792b69d.json` and reviewed revision `b8c54d2b-362b-4059-b5ca-504dd1ae257c`; real profile inspection session `.local/models/sessions/76c0edc7-d088-4989-b35a-eb9b61db9439.json`. CAD, profiles, test projects, logs and bundles stay ignored; no user CAD is included in the PR.

## Limits

Manual entry records the operator's measurement confirmation; it does not independently verify a scale or ruler. Total mass scales the existing distribution/inertias proportionally. Drive dimensions do not resize CAD, move wheel joints or rewrite collision envelopes. Mecanum's explicit override uses one diameter for all four slots. Geometric skid-steering track is a starting value; effective track, friction, motor/battery response, rubber and retention behavior still require independent measurements/recordings. This change adds no general drivetrain gearing model, telemetry-import UI or real-world accuracy certification.
