# Measuring and saving wheel grip

In guided robot setup, **Physics assumptions → Measure wheel grip…** fits one
configured drive wheel's sliding coefficient from physical force/load readings.
Enable native contacts in **Wheel support model…** first, and verify motor bindings
and wheel collision geometry. CAD geometry and motion-demo travel do not measure grip.

## Guided process

1. Choose the drive joint whose tread you tested. Use the actual tread on the reference
   surface, with wheel rotation restrained and this contact isolated from other wheels
   and chassis scraping. Measure the normal load on that wheel and the horizontal
   force at sliding onset, in the intended traction direction. Rolling drag is a
   different experiment. Keep surface condition and test direction consistent.
2. Name the reference surface and enter its simulator friction coefficient. The
   simulator multiplies wheel and surface coefficients. This experiment identifies
   their effective pair; it cannot determine two independent material coefficients.
   The motion-demo floor uses 0.6. Match the saved surface coefficient to the actual
   field material setting when using the result elsewhere.
3. Record at least three fitting trials at different known normal loads. Reserve at
   least two fresh trials for validation before fitting. Repeated readings from the
   same experiment use the same trial ID; a whole trial belongs to only one set.
   The empty five-row form is a starting structure, with no example readings.
4. Enter normal load in N, kg or lb and force in N or lbf, or import CSV. Mass units
   convert to weight using 9.80665 m/s². Changing the unit selectors converts entered
   numeric values. Remove unused blank rows before fitting.
5. Review the quality criteria and choose **Fit and preview**. Inspect the coefficient,
   trial counts, force RMSE, relative errors and any named failing trial. Poor quality
   blocks application. Confirm that the readings come from physical tests before
   choosing **Apply measured settings**.
6. Review collision geometry and rerun the motion assessment before saving a reviewed
   revision. Application saves a draft with normalized readings and reference context.

## CSV exchange

**Save blank CSV template…** writes headers for the selected units and wheel, with
three `fit` and two `validate` trial IDs. Fill the blank load/force cells. A CSV uses
exactly these five columns, in any order:

| Column | Accepted values |
| --- | --- |
| `joint` | Configured drive joint name |
| `trial` | Nonblank physical experiment ID; repeat it for repeated readings |
| `set` | `fit` or `validate` |
| Normal load | Exactly one of `normal_load_n`, `normal_mass_kg`, `normal_mass_lb` |
| Sliding force | Exactly one of `sliding_force_n`, `sliding_force_lbf` |

Quoted names, commas and doubled quotes are supported; multiline values are not.
The import checks all rows and reports rows omitted for other wheels. Trial IDs must
use one partition throughout the CSV. There is a 1 MB / 1,000-reading limit.
**Export entered readings…** writes normalized N/N CSV for the selected wheel.

## Fit and quality checks

For fitting trial means Nᵢ and Fᵢ, effective grip is
`sum(Nᵢ × Fᵢ) / sum(Nᵢ²)`. The wheel coefficient is effective grip divided by the
saved surface coefficient. Validation trials never determine that coefficient.
Repeated readings count as one trial in fitting and trial-count checks. Error checks
include every reading, weight each trial equally and compare predicted force with
measured force. Relative RMSE divides force RMSE by the RMS measured force using the
same trial weights. Each trial must also pass its own relative error limit.

| Saved criterion | Default | Allowed range |
| --- | --- | --- |
| Minimum fitting trials | 3 | 3–100 |
| Minimum validation trials | 2 | 2–100 |
| Minimum fitting-load variation `(max − min) / max` | 0.20 | 0.05–1 |
| Maximum fitting and validation relative RMSE | 0.15 | 0.01–0.50 |
| Maximum individual-trial relative error | 0.25 | 0.01–0.75 |

Normal load must be finite and 0.000001–100,000 N; force must be finite and
0–100,000 N. Surface friction must be 0.000001–2. Zero measured signal, insufficient
load variation, excessive errors and a fitted wheel coefficient above 2 are rejected.
Results are not silently clamped. Relaxing tolerances does not improve measurement accuracy.

## Portable evidence and corrections

Evidence lives in `runtime.drive_contacts.wheel_friction[].measurement`: original
recorded joint, method/version, named surface and coefficient, normalized readings,
quality criteria, results, and hashes of readings and fit context. Export/import carries
these values inside the model bundle without an external CSV dependency. Native loading
recomputes hashes, fitting and quality metrics before accepting measured settings.

Compatible CAD migration remaps the parent wheel entry while preserving the original
recorded joint in the historical report. Ambiguous/removed joints use the existing
replacement/removal choices. Changed CAD still requires review of bindings, geometry
and measurement applicability; preserved evidence does not prove unchanged hardware.

Reopen **Measure wheel grip…** to edit and refit saved readings. **Wheel grip…** can
discard evidence explicitly, including stale evidence. A changed manual coefficient
or inheritance removes that wheel's evidence; an unchanged value preserves it unless
discard is selected. Other wheels retain their tuning. Direct edits to readings,
surface, criteria, results or coefficient require refitting or discarding stale evidence.
If several wheels have stale evidence, corrections are saved individually and loading
continues to report the remaining invalid wheel. Correct that wheel next before running.
Detailed evidence is available in Advanced settings; it is kept out of the runtime
parameter table to keep ordinary tuning readable.

## Remaining physical limits

This measures a scalar coefficient at sliding onset. It does not fit sliding-speed
dependence, longitudinal slip curves, Mecanum roller anisotropy or rolling resistance.
Optional tire static/sliding parameters impose a separate force limit; inspect them
alongside material grip. Detailed asymmetric steering and individual wheel dynamics
remain approximations described in [wheel contact physics](DRIVE_CONTACTS.md).
Passing independent trials checks agreement with supplied data, not the correctness
of the experiment. No hardware readings are bundled with this feature.

See [implementation plan](../tasks/measured-grip/PLAN.md) and
[validation evidence](../tasks/measured-grip/RESULTS.md).
