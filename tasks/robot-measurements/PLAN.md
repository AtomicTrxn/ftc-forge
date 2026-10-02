# Robot measurement guide — calibration step 1

Status: implemented and validated; evidence is in RESULTS.md. Scope is manual measurements and motor setup; telemetry import/fitting and response comparisons are later steps.

1. Add an optional guide from robot Physics assumptions and Finish. Explain weighing the operating robot, measuring tread diameter and center-to-center drive spacing, and checking motor names, gearing and direction against the real robot.
2. Save explicitly confirmed measurements in the reusable model with measured provenance. Support kg/lb and mm/cm/in; preserve values left unselected and preserve existing telemetry calibration. A saved model becomes a new draft requiring native check/review.
3. Apply differential dimensions through the existing drive settings. Add explicit Mecanum geometry overrides with the existing CAD/default fallback when absent. Keep CAD/collision geometry at its imported size; physical review must reconcile any mismatch.
4. Show current transmission bindings and hardware availability. Offer Save and open motor bindings to continue through existing joint/motor selectors. Explain deferred project mapping and physical shaft signs separately from TeamCode Direction.
5. Apply the total mass override in native model preview/validation as already done in scene checking and normal runtime.
6. Verify unit conversion, invalid measurement rejection, selective updates, preserved calibration and original revisions, portable export/import, native mass and Mecanum override/fallback behavior. Exercise the focused guide and motor handoff through the desktop UI. Document evidence and ship.
