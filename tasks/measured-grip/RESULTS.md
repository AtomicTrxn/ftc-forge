# Measured wheel grip — results

Changelog: the [measured tire-slip follow-up](../measured-tire-slip/RESULTS.md) now
fits steady longitudinal curves from independent direct-force trials. Transient and
shaft-inertia identification remain open.

Published in [pull request #20](https://github.com/AtomicTrxn/ftc-forge/pull/20).

## Delivered

- **Measure wheel grip…** guides the selected configured wheel through physical
  sliding-onset measurement, units, surface context, editable readings, CSV exchange,
  quality criteria, preview and explicit application. Empty templates contain no forces
  or loads that could be mistaken for measurements.
- Fitting uses physical trial means through the origin. Whole trials are assigned to
  fitting or independent validation; repeated readings cannot inflate trial counts or
  give one experiment extra weight. All readings contribute to residual checks.
- Saved criteria bound trial counts, load variation, overall fit/validation relative
  error and each individual trial's error. Poor data and coefficients above 2 are
  rejected instead of clamped. A bad trial is named in the preview.
- Readings, reference context, criteria/results and hashes travel with the wheel
  setting. Native loading recomputes the evidence and rejects stale settings. Compatible
  CAD renames retain historical measurement identity and require fresh review.
- Explicit manual tuning discards only the affected wheel's evidence. Unchanged values
  preserve it unless discard is selected. Multiple stale wheels can be corrected
  individually; simulation remains blocked until every remaining invalid wheel is fixed.
- Applying measurements saves a draft, marks the coefficient measured/calibrated,
  preserves immutable originals and invalidates old motion/collision review.

## Automated checks

The full regression gate was run:

```sh
./gradlew test :gui-runner:installDist \
  :gui-runner:verifyRobotPhysics --args='.local/robots/rev-duo-starter/retention-project' \
  :gui-runner:verifyTorusRetention --args='.local/robots/rev-duo-starter/retention-project'
python3 -m unittest discover -s tools -p 'test_*.py'
```

That gate passed **155 Java cases** and **46 Python cases**, together with both native
REV probes. Final desktop refinements and the added multiple-stale-wheel case were
followed by `./gradlew test :gui-runner:installDist`, covering **156 Java cases**
(116 GUI/import/native, 33 physics-engine, 7 SDK). Distribution builds passed.

### Fitting and rejection

- Synthetic forces proportional to load with effective grip **0.45** recover wheel
  coefficient **0.75** against surface coefficient **0.6**, using three fitting and
  two reserved validation trials. Errors are zero for this exact fixture.
- Replacing the first held-out force (15 N normal load) from **6.75 N** to **3 N**
  leaves the fitted coefficient **0.75**, but fails validation: **2.6517 N** RMSE,
  **0.3221** relative RMSE and **1.25** worst-trial relative error.
- Noisy but repeatable data passes and reports nonzero residuals. Duplicating one
  experiment's rows 100 times leaves its coefficient/error weight unchanged. Repeated
  rows cannot satisfy missing independent trials. One bad trial among 100 good ones
  still fails its individual gate.
- Zero signal, constant fitting loads, invalid/nonfinite inputs, cross-set trial IDs
  and out-of-range quality parameters are rejected. A fit requiring wheel coefficient
  4.5 reports rejection without clipping it to 2.
- All six load/force unit combinations (N/kg/lb × N/lbf) recover the same coefficient.
  Quoted joint names and normalized CSV round trips pass; malformed units/quotes,
  missing selected wheels and blank templates fail with actionable messages.

### Persistence and native application

Controller tests apply explicit confirmed evidence to a previously reviewed synthetic
robot, check changed motion context and immutable originals, run forward movement
through native physics, review/export it, and import into a separate library. Saved
readings and provenance survive. An unchanged coefficient preserves review/evidence;
a changed manual coefficient drops evidence and requires review.

Both Java and Python tests import renamed CAD joints and retain the parent wheel's
coefficient plus original recorded joint and readings, while keeping the changed model
unreviewed. Python also rejects inconsistent coefficients, cross-set trials and edited
stored coefficients. Java rejects edited readings, results and criteria (including
relaxed tolerances), and verifies stale-evidence repair without changing other wheels.

### Existing REV regression

The prepared REV drivetrain/mechanism probe traveled **1.20879 m**, exercised the
coupled intake, contact-driven containment/release and turning (**0.71893 rad**).
The torus probe passed three pickup/carry/retention/reverse-release scenarios and
inactive, reverse, coasting, stalled and overload gates. These probes retain their
existing saved legacy support settings; they demonstrate regression coverage and do
not establish measured native-mode REV rubber behavior.

## Desktop validation

The rebuilt macOS QA application used an isolated copied library and synthetic tank
fixture. Native UI interactions verified:

1. Wheel choices include the configured joint, part and bound motor.
2. The initial form starts with measurement instructions and empty readings. CSV import
   loads the selected wheel's five readings; the fixture was labeled **Synthetic QA
   fixture — not hardware data**.
3. A passing preview without source confirmation offers no Apply action. A deliberately
   bad validation force shows the errors above, names `validation1` and blocks Apply.
4. Restoring the fixture and explicitly exercising the source-confirmation/apply path
   saves 0.75 for wheel0, keeps wheel1's 0.45 and shows **Measured / calibrated**.
   Collision and motion review remain required. The synthetic QA model was not approved.
5. Save/close/resume reloads the saved surface, criteria and readings. A final reopened
   form visibly shows the complete numeric surface and quality settings.

Desktop testing found and fixed three presentation problems: the form initially opened
partway down (fixed initial focus/scroll position), numeric spinners clipped decimals
(fixed editor width), and measured coefficient provenance matched the generic user label
(fixed the stored coefficient label). Rebuilt desktop checks confirmed the fixes.

Private source CAD, libraries and QA fixtures remain outside version control. The
temporary root-level CSV used to navigate the desktop file picker was removed.

## Limits and next candidate

All measurement fixtures here are synthetic. No hardware force/load recording was
provided, so this is a validated calibration workflow, not evidence that the supplied
robot's rubber properties match reality.

The experiment identifies scalar sliding-onset grip for one tread/surface/direction
pair. Surface settings must match when reusing it. Optional tire curves remain a second
limit; sliding-speed dependence, detailed asymmetric steering, rotating contact geometry,
roller anisotropy, suspension and slopes remain gaps. Dynamic tire fitting needs
independent wheel/hub velocities, force/load and shaft-inertia information. A
steady direct-force recording and validation workflow is delivered in the
[measured tire-slip follow-up](../measured-tire-slip/RESULTS.md), with whole trials held
out and explicit identification checks. Acceleration/current-derived force and
shaft-inertia fitting still need additional independent instrumentation and validation.
