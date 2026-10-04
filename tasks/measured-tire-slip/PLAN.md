# Measured longitudinal tire-slip calibration

## Selected gap and scope

The optional differential tire model has manually assumed longitudinal stiffness,
peak/sliding friction and transition speed. Add a repeatable measured fitting workflow
for those four parameters. Use steady bench trials with independently measured hub
velocity, wheel surface velocity, normal load and longitudinal contact force. Direct
force measurement avoids trying to infer tire force or shaft inertia from motor current.
Keep lateral scale, reflected inertia and tire geometry as explicit existing assumptions.

## Plan

1. Share the steady longitudinal brush envelope with the runtime tire law. Fit the
   four coefficients within saved user-adjustable bounds, weighting physical trial
   means equally and excluding entire validation trials from fitting.
2. Reject invalid/dissipative-sign errors, unsteady trial groups, poor held-out agreement,
   unsupported coefficients, boundary fits and weak/correlated parameter identification.
   Save quality thresholds, optimizer bounds, force errors and sensitivity evidence.
3. Add a guided CSV measurement flow for an existing differential tire class (traction
   or omni): experiment instructions, blank template, surface/source context, import,
   fit preview, quality controls, explicit source confirmation and application. Do not
   prefill measurements or automatically enable an unconfigured tire model.
4. Store normalized readings and fit evidence alongside the selected tire spec. Validate
   it before native simulation; preserve export/import and compatible CAD migration,
   with renewed review. Provide explicit discard/refit for manual corrections.
5. Add synthetic recovery/noise and identification/rejection tests, workflow/native
   loading tests, portable bundle and CAD migration tests, then run the full regression
   suite and existing REV probes. Inspect the desktop flow and correct usability issues.
6. Document limits, measured-vs-assumed values and evidence; commit, publish and merge.

## Acceptance and limits

A passing report demonstrates agreement with supplied independent steady trials, not
hardware accuracy. No physical recordings have been supplied. Dynamic acceleration
trials, current-derived tire force, lateral/roller anisotropy and shaft-inertia fitting
are excluded. Native material grip is an additional cap and must be consistent with the
reference surface; implicit timestep integration is validated separately from the
continuous steady curve. Changed CAD/settings still need collision and motion review.

## Progress

- [x] Inspect current physics and choose bounded scope.
- [x] Implement fitting and evidence validation.
- [x] Implement guided application and portability.
- [x] Test and inspect available desktop flow; final UI actions await Mac unlock.
- [ ] Document and ship.
