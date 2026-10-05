# Advanced physics and field behavior

Requested 2026-10-04: implement remaining physics accuracy (item 2) and sensors/game
behavior (item 5). Ship verified stages; preserve reviewed legacy profiles and private CAD.

## A. Tire response

- [x] Add independent lateral stiffness/grip and a configurable transient relaxation time.
  Preserve existing steady tire settings and measured longitudinal evidence.
- [x] Bound impulses by the actual contact friction budget and dissipative slip response;
  reset transient state when support is lost. Verify timestep sensitivity and shaft reaction.
- [x] Preserve new settings in export/import/CAD migration; document units and assumptions.

## B. Spatial running gear

- [x] Add opt-in rotating native wheel bodies/joints rather than welded wheel proxies.
  Share driven shaft response for multiple wheels on one motor; keep motor reaction/encoders.
- [x] Allow full chassis orientation for slopes/tip-over, use native wheel contact friction,
  and support URDF suspension joints with explicit spring/damping/travel parameters.
  Reject incompatible aggregate/tire ownership instead of applying traction twice.
- [x] Report pitch/roll and test ramp, unsupported drive, rotating wheel contact, loaded motor,
  suspension response and tip-over with generated CAD. Keep defaults unchanged.

## C. Physical sensors and geometric vision

- [x] Configure sensor poses relative to chassis or mechanism links, range/update/latency
  and deterministic measurement assumptions. Missing configurations remain disclosed stubs.
- [x] Feed distance rays, contact switches and sampled scene colors into SDK devices.
  Ignore robot self-hits; test range, obstruction, movement, linkage and latency.
- [x] Add configured geometric AprilTag observations with camera frustum/range, occlusion,
  pose/units and portal lifecycle. This is geometry-based vision, not rendered image processing.
- [x] Exercise real TeamCode-facing APIs; update SDK coverage and diagnostics/documentation.

## D. Field layout and scoring

- [x] Add portable, versioned field behavior settings: named scoring regions, object types,
  time windows, occupancy/entry rules and match clock/reset; display and export evidence.
- [x] Keep generic rules configurable for other CAD; pin any BIOBUZZ preset to official
  documents and distinguish practice geometry from a verified competition layout.
- [x] Validate official layout against published setup sources and current CAD, reporting
  deviations/uncertainties rather than silently approving existing practice placements.
- [x] Test boundary crossings, reset, duplicate prevention, field-only mode and time windows;
  document scoring scope and unsupported officiating decisions.

## Gates and delivery

- [x] Run affected native/unit/workflow tests and the existing validation matrix.
- [x] Inspect actual renderer output for enabled features; record desktop limitations.
- [x] Ship tested work and record PRs, results and remaining evidence needs here.

## Sources and evidence

- FIRST BIOBUZZ competition manual TU03 (Oct 1, 2026):
  https://ftc-resources.firstinspires.org/ftc/archive/2027/game
- FIRST event field setup guide V1.0 (Sep 12, 2026):
  https://ftc-resources.firstinspires.org/ftc/field/eventfieldguide
- CAD is geometry, not a source of measured friction, damping, inertia of installed motors,
  material compliance or referee judgement. Synthetic checks establish software consistency;
  physical validation still requires robot measurements.

## Progress

Implementation and validation complete. Configurable practice rules were selected as the
portable default after the optional field-rule preference received no reply. No official
season scoring equivalence or competition placement certification is claimed.

See [RESULTS.md](RESULTS.md) for native evidence, regression counts and remaining physical
validation needs. Delivery: [PR #25](https://github.com/AtomicTrxn/ftc-forge/pull/25), merged as
`06e69f1` on October 4, 2026 (America/New_York).
