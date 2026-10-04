# Measured longitudinal tire slip

**Physics assumptions → Measure tire slip…** fits an existing differential tire
class (`traction` or `omni`) from steady physical speed/load/force trials. The class
selector lists every current wheel receiving that curve. Configure the optional
[tire model](CONTACT_MODELS.md) and wheel bindings first; this guide does not enable
an unconfigured tire model or infer wheel type from CAD.

## What to measure

Use the actual tread, reference surface and intended longitudinal direction. Hold a
steady test condition with negligible lateral motion. Record:

- Wheel surface speed along its forward axis, in m/s. If derived from a shaft encoder,
  use measured radius and verified gearing/sign. Wheel surface speed may come from the
  encoder; hub speed must come from an independent ground/localization measurement.
- Hub ground speed along that same axis, in m/s. Encoder-derived hub speed cannot
  distinguish wheel spin from traction.
- Normal load on that wheel, positive newtons.
- Directly measured longitudinal tire force, signed newtons. Its positive direction
  matches `wheel surface speed − hub speed`. Motor current is not a direct tire-force
  measurement. Avoid chassis scraping, other-wheel loads and transient acceleration.

Name the reference surface/condition and the independent hub-speed and direct-force
measurement sources in the guide. The fit applies to every wheel in the selected class;
check that their treads and conditions are representative before sharing a curve.

Reserve whole physical experiments for validation before fitting. Defaults require
12 fitting and 6 validation trials. Each set needs at least three positive-slip and
three negative-slip trials. Include different normal loads and low-slip, transition
and high-slip plateau conditions. More samples from one experiment are repeated
readings, rather than additional independent trials. Keep one trial ID for those samples.
Different wheels need distinct trial IDs.

## CSV and guided application

1. Select a configured tire class and enter source/surface context.
2. **Save blank tire CSV template…** selects a current wheel and writes 12 `fit` and
   6 `validate` IDs with empty numeric cells. Fill actual measurements and add more
   conditions as needed. No synthetic readings are prefilled.
3. **Import tire readings CSV…** accepts exactly these seven SI headers, in any order:
   `joint,trial,set,wheel_surface_mps,hub_speed_mps,normal_load_n,longitudinal_force_n`.
   Use `fit` or `validate`; quoted names/commas/doubled quotes are supported. Multiline
   fields are unsupported. Files are limited to 1 MB and 1,000 nonblank readings.
   Imports reject joints outside the selected class. Export writes loaded SI readings.
4. Review saved search bounds and quality criteria; choose **Fit and preview**.
   Inspect force errors, independent counts, parameter sensitivity and correlation
   checks. Failed fits cannot be applied. Correct measurements in the CSV and import
   again, or add fresh independent conditions. Do not turn failed validation trials
   into fitting trials to obtain a passing score.
5. Confirm the readings are independent physical steady measurements, then explicitly
   **Apply measured tire settings**. Collision review and fresh motion assessment
   remain required before saving a reviewed revision.

## Curve and fitting

For slip `s = wheel_surface_mps − hub_speed_mps`, normal load `N`, and fitted
stiffness `k`, the continuous steady prediction is:

```text
mu(s) = sliding_mu + (static_mu − sliding_mu) × exp(−(s / transition_mps)²)
F(s,N) = sign(s) × min(k × abs(s), mu(s) × N)
```

The envelope is shared with the runtime tire law. Native runtime uses an implicit
timestep integration and a friction ellipse for longitudinal/lateral coupling;
this experiment fits the continuous purely longitudinal steady limit.

Fitting minimizes mean squared force error divided by normal load, using each fitting
trial's mean condition and giving every trial equal weight. A deterministic logarithmic
grid starts eight bounded coordinate refinements. Validation trials never choose parameters.
Residual checks include every reading and weight each trial equally. Relative RMSE
divides force RMSE by RMS measured force under those same weights. Each trial must also
pass its own relative error limit.

| Search parameter | Default lower / upper | Units |
| --- | --- | --- |
| `static_mu` | 0.1 / 2 | dimensionless |
| `sliding_mu` | 0.05 / 2 | dimensionless; ≤ static |
| `stiffness_n_per_mps` | 1 / 2,000 | N per m/s |
| `transition_mps` | 0.01 / 1 | m/s |

Bounds are positive and ordered. Friction cannot exceed 2; stiffness cannot exceed
100,000 N per m/s; transition cannot exceed 5 m/s. A fit within 0.5% of an individual
logarithmic search boundary is rejected; investigate units, test coverage and bounds.

| Saved criterion | Default |
| --- | --- |
| Minimum fitting / validation trials | 12 / 6 |
| Fitting normal-load variation `(max − min) / max` | 0.20 |
| Maximum fitting and validation relative RMSE | 0.15 |
| Maximum individual-trial relative error | 0.25 |
| Maximum within-trial slip / normal-load relative RMS spread | 0.10 / 0.05 |
| Minimum log-parameter normalized-force sensitivity | 0.01 |
| Minimum eigenvalue of column-normalized information matrix | 0.001 |

Slip spread divides by the larger of absolute mean slip and 0.001 m/s; load spread
divides by mean load. Sensitivity is RMS change of force/load per log-parameter change.
The information check detects correlated columns in that local sensitivity matrix.
Low slip alone cannot identify the friction envelope; plateau data alone cannot
identify stiffness/transition. These checks are local diagnostics, not confidence
intervals or guarantees of a unique global fit. Relaxed tolerances do not improve accuracy.

## Reuse and corrections

Each class stores optional `measurement` evidence with original recorded joints,
normalized readings, source context, bounds, criteria, results and a canonical context
hash. Native loading verifies the hash, reruns fitting, checks identification/errors,
and compares fitted parameters/results. Lateral scale is preserved as a separate
assumption. Shaft inertia, contact tolerance, wheel geometry, other-class settings and
wheel friction are also retained.

Export/import includes the evidence without an external CSV dependency. Compatible CAD
renames preserve original joint labels as historical measurement identity while current
bindings/classification migrate through existing choices. Changed CAD still needs review
of whether the same physical tread and tests apply. Reopening the guide loads evidence
for explicit refitting; old recorded names are explained as historical after migration.

Changing a fitted number manually makes its evidence stale. Use **Discard tire
measurements…** to keep numeric settings as manual assumptions before editing, or import
corrected measurements and refit. Discard affects only the chosen class and requires
renewed review. Multiple invalid classes can be corrected one at a time, while simulation
continues to reject remaining invalid evidence. Evidence stays in Advanced settings and
the dedicated guide instead of cluttering the runtime parameter table.

## Physical limits

This identifies a tread/surface pair, so use the same reference surface and condition.
Native wheel friction × surface friction is an **additional cap**: it can mask this
curve if lower than fitted grip. For example, a curve with `static_mu = 0.9` needs
a material budget of at least 0.9 to expose its whole envelope; with surface friction
0.6, wheel friction 1.5 supplies that budget. This is a parameter consistency example,
not a hardware measurement. The guide does not automatically change field materials.

The experiment does not fit transient tire relaxation, shaft inertia, lateral scale,
rollers, steering asymmetry, suspension or tip-over. No hardware recording is included.
Passing synthetic fixtures proves software behavior; passing supplied validation checks
agreement with the supplied measurements, not physical accuracy of the experiment.

See [plan](../tasks/measured-tire-slip/PLAN.md) and
[validation evidence](../tasks/measured-tire-slip/RESULTS.md).
