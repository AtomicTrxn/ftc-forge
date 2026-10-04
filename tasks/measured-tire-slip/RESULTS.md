# Measured longitudinal tire slip — results

## Delivered

- Shared steady longitudinal envelope with the native tire law. The runtime integration
  and its lateral ellipse/material cap are retained.
- Bounded fitting of `static_mu`, `sliding_mu`, `stiffness_n_per_mps` and `transition_mps`
  from independent wheel/hub speed, normal load and direct-force trials. Physical trial
  means are weighted equally; entire validation trials are excluded from fitting.
- Input/sign/partition/steadiness checks, both slip directions, varied fitting loads,
  force residual gates, boundary rejection, log-parameter sensitivities and information
  correlation checks. Low-slip-only and plateau-only data cannot pass as full calibration.
- **Measure tire slip…** for an existing differential traction/omni class: source and
  surface context, blank CSV templates, SI import/export, adjustable saved bounds and
  criteria, preview and explicit confirmed application. It does not silently configure
  tires or prefill synthetic measurements.
- Evidence travels with the selected tire spec. Native loading recomputes canonical
  hashes, fitting, results and quality/identification checks. Export/import and compatible
  CAD migration preserve original measured identities, with renewed model review.
- Lateral scale, shaft inertia, geometry, wheel friction and the other tire class are
  retained. **Discard tire measurements…** preserves numeric settings as assumptions;
  multiple stale classes can be corrected individually while physics remains blocked.

## Automated validation

```sh
./gradlew test :gui-runner:installDist \
  :gui-runner:verifyRobotPhysics --args='.local/robots/rev-duo-starter/retention-project' \
  :gui-runner:verifyTorusRetention --args='.local/robots/rev-duo-starter/retention-project'
python3 -m unittest discover -s tools -p 'test_*.py'
```

The full gate passed **170 Java cases** (122 GUI/native/import, 41 physics-engine,
7 SDK; zero failures/errors/skips) and **48 Python cases**, plus both existing REV
native probes. The final readable/scrollable preview refinement and one additional
multiple-stale-class test were followed by the seven `TireSlipMeasurementTest` cases
and another successful distribution build. This covers **171 distinct Java cases**.
`git diff --check` passed.

### Mathematical checks

- Synthetic reference `(static=0.9, sliding=0.6, stiffness=100 N per m/s,
  transition=0.15 m/s)` recovered approximately **0.8999999997, 0.5999999997,
  100.00000056, 0.1499999998** from **48 fitting** trials. **42 reserved trials**
  had force RMSE **1.63e-8 N** and relative RMSE **3.10e-9**.
- Minimum normalized-force log sensitivity **0.08610** and information eigenvalue
  **0.24750** passed the default identification checks for this well-covered fixture.
- Halving all reserved force readings leaves fitted parameters unchanged and rejects
  validation. Mild synthetic noise passes and reports residuals. Duplicating one
  experiment's samples leaves its trial count and fitting weight unchanged.
- Low-slip-only and plateau-only data fail parameter identification. Unsteady grouped
  trials, insufficient independent trials, fit/validation leakage, incorrect force
  signs, nonfinite data and a stiffness search range excluding the true value fail.
- The continuous envelope agrees with the runtime longitudinal solver as timestep
  tends to zero at positive/negative, unsaturated/saturated and zero slip.

### Workflow and portability

Java tests cover SI CSV and quoted names, empty templates, malformed headers/numbers,
trial partition/ownership and wrong-class wheel rejection. Context, criteria, readings,
stored results and fitted parameter edits invalidate evidence; canonical JSON round
trips retain it. Source confirmation and correct class/recorded-wheel selection are
required at controller application.

A reviewed synthetic differential robot receives the fitted class as a draft, changes
its motion context, retains original saved bytes and runs forward through native tire
physics. Review/export/import into another library preserves the complete evidence.
The other tire class, lateral scale and reflected shaft inertia remain unchanged.
Compatible wheel-joint renames keep original recorded names and data while current
wheel choices use the new joint; changed CAD remains unreviewed. Manual parameter
changes block compilation until explicit discard/refit. Multiple stale classes can be
discarded sequentially without allowing a native run between corrections.

Python checks preserve report metadata through bundles and compatible CAD renames and
reject wrong class, inconsistent force signs, cross-set trials and manual coefficient
changes. Those fixtures exercise metadata/transport, rather than claiming a native-valid
fit receipt; the complete fit/hash validation is covered by the Java native pipeline.

### Supplied REV regressions

The original prepared drivetrain/intake probe passed with **1.20877 m** forward travel,
coupled shaft motion, contact containment/release and **0.70952 rad** turn. Three torus
pickup/carry/stopped-retention/reverse-release scenarios passed, as did inactive,
reverse, coasting, stalled and overload gates. These retain their existing unmeasured
prepared tire/support settings; the probes establish regression coverage, not hardware
calibration of the supplied robot.

## Desktop check and limitation

The rebuilt macOS QA app used an isolated copied model library with an explicitly
configured synthetic differential tire fixture. Native UI testing verified the new
controls, current wheel/class choices, initial instructions, readable bounds, empty
source fields and absence of prefilled measurements. The actual file picker imported
a **90-reading** synthetic CSV. The preview showed the recovery and identification
values above, and offered **no Apply action** with source confirmation unchecked.
The surface and measurement sources were explicitly labeled synthetic QA data.

The Mac locked during the return-to-readings action. An unlock was requested. Final
desktop Apply/save/resume/discard interactions could not be completed while locked;
their controller/native/portable paths are covered by the seven automated workflow tests.
The preview was subsequently made concise and scrollable, compiled and covered by that
focused test run. This final presentation refinement still needs a desktop recheck.
No synthetic model was approved and original CAD/preparations were untouched. Private
libraries, fixtures and logs remain outside version control.

## Remaining physical limits and next candidate

No physical tire recordings were supplied. This validates the software workflow and
synthetic recovery only. The experiment fits a continuous steady longitudinal law;
local identification diagnostics do not prove unique global parameters or establish
confidence intervals. Material wheel/surface grip is an additional cap and must be
consistent with the reference conditions to expose the fitted curve.

Transient relaxation, lateral/roller behavior, shaft inertia, steering asymmetry,
rotating wheel contact geometry, slopes, suspension and tip-over remain approximations.
The next bounded candidate is independent lateral tire/omni resistance measurement and
validation, followed by transient response/inertia work when sufficient independent
instrumentation is available.
