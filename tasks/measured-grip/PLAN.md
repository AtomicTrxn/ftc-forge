# Measured wheel grip workflow

## Selected gap

The previous phase separated wheel and chassis friction. A user can enter a coefficient,
but has no guided way to fit repeated physical readings, check independent trials, or
retain the measurements with portable settings. Close that gap before extending tire
curves or rotating wheel dynamics.

## Plan

1. Fit sliding-onset force versus normal load using trials explicitly assigned to fitting
   or validation. Keep whole trials in one set and average repeated readings per trial
   when fitting so extra samples do not masquerade as independent experiments.
2. Expose saved quality parameters: minimum trial counts, load variation, relative residual
   tolerance and per-trial error tolerance. Report fit and held-out errors; reject poor
   data or coefficients outside the supported range instead of silently clamping.
3. Add a guided measurement form for a configured wheel: experiment instructions,
   N/lbf force and N/kg/lb normal-load units, editable readings, CSV import/template,
   reference surface name/coefficient, fit preview and explicit application of a passing
   result. No synthetic readings are prefilled into user forms.
4. Store normalized readings, fit criteria/results and surface context beside the wheel
   coefficient. Preserve evidence in export/import and compatible CAD migration;
   removal/manual tuning should discard or flag stale evidence for only that wheel.
   Changed CAD and changed physics still require review/retest.
5. Test known-coefficient recovery, noisy held-out data, bad data, trial leakage, unequal
   sample counts, units/CSV, guided application, stale evidence, portability/migration and
   native motion. Run full regression gates and inspect the desktop flow.
6. Document evidence and limits; commit, publish and ship.

## Physical scope

This measures the effective tread/surface pair at sliding onset with wheel rotation
restrained and known normal load. The fitted wheel coefficient is effective grip divided
by the saved nonzero surface coefficient. Surface settings must remain consistent.
For Mecanum/omni wheels, direction matters; a scalar result does not identify roller
anisotropy. Optional tire curves remain a separate force limit. No hardware recording is
available here: synthetic recovery tests verify software, not actual robot accuracy.

## Progress

- [x] Inspect current calibration and choose scope.
- [x] Implement fitting, guided controls and portable evidence.
- [x] Verify physics, data quality and workflows.
- [x] Document and publish [PR #20](https://github.com/AtomicTrxn/ftc-forge/pull/20).
