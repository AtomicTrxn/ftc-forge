# Independent wheel grip — results

Changelog: the [measured-grip follow-up](../measured-grip/RESULTS.md) now supplies
guided scalar sliding-onset fitting and independent validation; dynamic slip fitting remains open.

Published in [pull request #19](https://github.com/AtomicTrxn/ftc-forge/pull/19).

## Delivered

- Optional `drive_contacts.wheel_friction` entries select a coefficient (0–2) by
  configured wheel joint. Empty/absent entries inherit the actual chassis material.
- Native normal support and mass/inertia are unchanged. Wheel grip × surface grip
  bounds aggregate traction and optional tire forces, including matching shaft reaction.
- Wheel/obstacle sliding contacts use wheel grip; belly and other body contacts keep
  their material. Body restitution and native rolling/spinning coefficients are retained.
- **Wheel grip…** provides joint/part/motor choices, inheritance, a numeric control,
  measurement instructions and draft/review guidance. Runtime settings also expose the
  entries with selectable joints, coefficient help and provenance.
- Motion results show simulated per-wheel effective grip. This is a contact budget,
  not a measured coefficient or proof of physical accuracy.
- Export/import, identical CAD reuse and compatible joint renames preserve tuning.
  Missing targets offer a replacement or removal of that one override. Multiple removals
  reindex outstanding choices and provenance; remaining tuning and native support survive.
- Edits leave saved revisions immutable and invalidate collision/motion evidence.
  Unchanged edits retain review and measured provenance. Removing one override retains
  other wheels' measurement labels.

## Validation

Full regression command:

```sh
./gradlew test :gui-runner:installDist \
  :gui-runner:verifyRobotPhysics --args='.local/robots/rev-duo-starter/retention-project' \
  :gui-runner:verifyTorusRetention --args='.local/robots/rev-duo-starter/retention-project'
python3 -m unittest discover -s tools -p 'test_*.py'
```

All **141 Java cases** passed (109 GUI/import/native physics, 25 physics-engine,
7 SDK), as did **44 Python cases**. After the final provenance/help refinement,
the five `WheelGripTest` cases were rerun and the distribution rebuilt successfully.
`git diff --check` passed.

### Native physics evidence

- Rubber coefficient 0.9 with chassis friction 0 moved **0.209436 m** in the forward
  demo. Rubber coefficient 0 with chassis friction 0.9 moved **−0.0000175 m** (no drive).
  Both cases had two supported wheels. These are synthetic fixtures, not measured REV behavior.
- A mixed pair (wheel coefficients 0 and 1.2, floor 0.6) had independent native grip
  impulse / normal impulse ratios **0 and 0.72**, while the body stayed at friction
  **0.02**, with mass **3 kg**, counted once. Zero floor friction removed both budgets.
- A wheel-only obstacle contact with wheel coefficient 1.2 and obstacle coefficient
  0.5 had native combined friction **0.6** while the chassis remained **0.15**.
  Belly-grounded geometry retained combined friction **0.09** against floor 0.6,
  and reported no supporting wheels despite the grippy wheel overrides.
- Optional tire tests compare positive/zero grip through both surface settings and
  wheel overrides. Zero grip produces no longitudinal force/travel and a higher peak
  unloaded shaft speed; the existing force/reaction path is unchanged.
- Existing inherited-material tests pass: low-power travel, airborne pushes/gravity,
  scraping, wheel/chassis wall blocking, coasting drag, decomposed mesh wheel identity,
  missing geometry, and traction bounds at 30/120/480 Hz.

### Saved settings and migration

Java controller tests exercise reviewed revision → edit → renewed review → export →
import → inheritance restoration, including context invalidation, measured provenance
and immutable originals. Invalid/nonfinite/duplicate coefficients and unused mechanism
targets are rejected; legacy mode requires explicit opt-in before guided grip edits.

Python tests preserve coefficients/provenance in a separate library and identical CAD
reuse, remap unique compatible joint renames, offer a replacement for changed geometry,
and remove two missing joints without disabling support or losing the third wheel's
coefficient/measurement labels. Changed CAD remains a draft.

### Desktop check

The macOS QA app was tested through native UI controls using an isolated copy of the
previous tank fixture. **Wheel grip…** displayed both configured joints and bound motors,
saved/reloaded wheel0 **0.95**, retained wheel1 inheritance, then saved wheel1 **0.45**.
Restoring wheel0 inheritance removed only its override and left wheel1 **0.45** visible.
The physics page displayed the new controls, provenance and draft/retest instructions.
The QA app was saved/closed with its model still unapproved. Original CAD was untouched.

### Supplied REV regression

The existing prepared REV drivetrain/mechanism probe passed. The torus probe passed
pickup, carrying, stopped retention and reverse release, plus inactive, reverse, coasting,
stalled and overload gates. These projects retained their saved legacy support settings;
this regression does not claim measured native-mode REV rubber calibration. Source CAD,
private QA libraries and logs remain outside version control.

## Remaining limits / next candidate

Sliding coefficients still approximate the tread/field pair. A pair measurement cannot
identify each material independently; calibration must use a consistent surface setting.
Optional tires apply a second static/sliding-curve limit. Aggregate driving pools supported
grip and does not model detailed asymmetric wheel steering. Restitution and native
rolling/spinning coefficients remain shared by the welded body.

Wheels remain welded collision proxies on a level chassis. Detailed rotating wheels,
rollers/treads, suspension, slope/moving-platform traction and a measured tire-curve
fitter remain gaps. Guided scalar sliding-onset measurement and portable validation
are delivered in the [measured-grip follow-up](../measured-grip/RESULTS.md). Dynamic
slip fitting still needs independent wheel/hub velocity, force/load and shaft-inertia
recordings with held-out validation against actual hardware data.
