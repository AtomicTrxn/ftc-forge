# Simulator validation and diagnostics

Requested: 2026-10-04. Implement and ship each item separately, in the order below.

## Purpose

Improve reliability before another robot CAD package or physical measurements are available.
Use the production importer, model profiles, motors, articulated bodies and Bullet contacts.
Synthetic checks establish software behavior and mathematical consistency; they do not establish
real robot accuracy or approve a user's collision review/calibration.

## 1. Reusable validation suite

- [x] Implement shared, generated URDF ZIP fixtures for differential and Mecanum drive,
  arm, slide, servo and continuous intake, plus an STL geometry fixture wrapped in one URDF.
  Generic preparation supports URDF/STL ZIPs; bare exporter archives require their adapter first.
- [x] Add a headless command that runs bounded scenarios through the production importer
  and native physics: expected drive directions, mechanism travel/limits, wall blocking,
  airborne wheels, external push, missing support/belly scraping and pickup/release.
- [x] Exercise export/import and changed-CAD migration; check settings survive and changed
  geometry requires renewed review. Retain fixture packages for inspection.
- [x] Write JSON and readable Markdown reports with stable scenario IDs, expectations,
  measured values, failure details, scope, engine/timestep metadata and nonzero exit on failure.
  Support checking declared movements of an existing prepared robot profile as well.
- [x] Add automated positive and negative tests; run native scenarios and existing affected tests.
- [x] Ship item 1 in its own merged PR (implementation and documentation verified).

Acceptance: one documented command produces an inspectable pass/fail report without private
CAD; a deliberately broken expectation fails, without suppressing the other scenario results.

## 2. Physics configuration and timestep matrix

- [x] Extend the runner with bounded mass, radius, track width, gearing, grip and timestep
  cases. Use independent analytic bounds and comparison tolerances, not golden positions.
- [x] Check finite motion, mass counted once, material force bounds, zero-grip response,
  zero-command drift, unpowered dissipation, blocked movement and convergence.
- [x] Include motor/shaft load and tire reaction checks, and mechanism gearing/limits.
- [x] Put all tolerances and test ranges in a documented versioned suite configuration;
  record them and per-case measurements in reports. Avoid claiming exhaustive parameter coverage.
- [x] Test rejected invalid configuration and deliberately violated bounds; run the matrix
  and relevant regression tests, document evidence.
- [x] Ship item 2 in its own merged PR.

Acceptance: the matrix tests actual production dynamics at multiple native timesteps and
reports both individual bounds and cross-timestep comparisons. Failures identify the configuration.

## 3. Live physics diagnostics

- [x] Provide one read-only snapshot for native contacts/normals, wheel support/load/grip,
  tire slip/force, motor effort/current and mechanism position/limits.
- [x] Add a toggleable overlay and concise diagnostics panel in both the simulator and the
  import-time motion demo. Reuse the existing collision-shape toggle.
- [x] Explain missing support, belly scraping, no grip and blocked/no-motion conditions;
  distinguish observed facts from possible causes. Mark unavailable tire data as unavailable.
- [x] Display SI units and a vector legend/scaling; bound and throttle rendering work.
  Hide all diagnostics when toggled off and clean up on demo replay/close.
- [x] Test live snapshots and overlay behavior with synthetic native scenes, including
  unsupported, slippery and blocked cases; verify diagnostics do not change physics.
- [x] Document controls and evidence and run regression checks.
- [ ] Ship item 3 in its own merged PR.

Acceptance: a user can see what contacts support the robot and inspect drive/mechanism effort
while it moves; toggling diagnostics has no effect on the physics result.

## Shipping and evidence

Each item is complete only after its acceptance checks pass and its PR is merged. Check off
implementation steps when verified; check off shipping after merge. Link each PR and record
test commands/results below. Preserve unrelated local files and private CAD.

### Item 1

Shipped: [PR #22](https://github.com/AtomicTrxn/ftc-forge/pull/22), merge `f71a918`.

- `./gradlew :gui-runner:validateSimulator`: **12/12 native scenarios pass**.
- Affected demo/review/contact tests and the suite: **36 distinct Java cases pass**
  (the initial affected run had 35; the final suite run adds the profile-command case).
- Positive and negative CLI paths preserve input profiles; failed checks write reports
  and return an unsuccessful process result. A deliberately stalled drive cannot pass.
- Differential forward travel: 0.20978 m; Mecanum left strafe: 0.20956 m.
  Wall blocks at x=0.18000 m; backward push moves 0.08072 m. Airborne push: 0.50000 m.
  Native flexible paddle transport: +0.02188 m / -0.05082 m. No-grip and belly cases
  correctly report no clear movement. All 12 mechanism/drive targets pass.
- The first run exposed an incorrect suite assumption about bare STL ZIP support.
  Corrected the fixture to the supported single-URDF/referenced-STL package and added a
  bare-STL rejection test. No production importer behavior was silently changed.
- Reports retained under `build/simulator-validation/`; fixtures and model libraries are
  generated artifacts, excluded from Git. No private CAD or physical data used.

### Item 2

Shipped: [PR #23](https://github.com/AtomicTrxn/ftc-forge/pull/23), merge `ef5b903`.

- `./gradlew test :gui-runner:validateSimulator --args='--matrix'`: **174/174 scenarios
  and all 180 Java cases pass** (GUI 132, physics 41, SDK 7; no skips/errors/failures).
- Matrix: 108 native drive combinations, 36 timestep comparisons, three signed mechanism
  reductions (1, -5, 10), 12 native tire cases and three shaft-load/current comparisons,
  plus item 1's 12 scenarios. Configuration and every tolerance travel with the report.
- Negative coverage rejects invalid/oversized ranges, unknown fields, duplicate values,
  missing convergence samples and excessive timestep spread.
- The first matrix run found stale native support after teleporting a settled robot up.
  Bullet retained its prior manifold until collision detection. The support check now
  reprojects local contact points through current body transforms before accepting support.
  A direct regression asserts zero support and zero drive acceleration immediately after
  the teleport; airborne tires also pass at 30/120/480 Hz. No settling delay hides the bug.
- Instrumentation records actual applied drive impulse and contact/shaft momentum reaction;
  it does not change the commanded solver dynamics. Existing tire/contact/mechanism tests pass.
- Generated reports remain under `build/simulator-validation/`; these are synthetic
  consistency/sensitivity results, not real-world calibration or exhaustive coverage.

### Item 3

Implementation and verification complete; shipment pending.

- `./gradlew test :gui-runner:validateSimulator --args='--matrix'`: **174/174 native
  scenarios and all 190 Java cases pass** (GUI 142, physics 41, SDK 7; no skips/errors/failures).
  Final report: `build/simulator-validation/run-03bce8c5-d694-45a5-8900-cd94d57cf0c3/report.md`.
- Ten diagnostics tests cover native load/normals, unavailable aggregate tire data,
  zero grip versus airborne versus belly support, loaded wall blocking, spinning tires,
  blocked slide effort/limits, hidden/throttled pooled rendering, snapshot immutability,
  shared D/P input events and replay, sampling limits/missing bindings, and the headless
  report/export path. Full native demo outcomes remain equal with polling/overlay on or off;
  immediate capture leaves native pose, shaft angle and body count unchanged.
- Actual OpenGL frame captures inspected at 1280 x 800 for the mechanism demo, individual
  tire demo and generic field simulator. Labels, SI units, unavailable support, limits,
  contact/load vectors and panel layout render correctly. Captures retained in
  `build/desktop-diagnostics/{demo,tires,field}.png`; capture processes close automatically.
- Desktop automation still timed out attaching to the GLFW renderer after the user confirmed
  the Mac was unlocked. OS-level keyboard automation was not performed. Shared D/P bindings
  were verified through jME InputManager events, including export and a replaced replay overlay;
  rendered images came from the actual renderer's frame capture, not fabricated screenshots.
- Initial native diagnostic testing found a static-body velocity getter assertion in the
  blocked-slide fixture. Static chassis velocities now read as zero; dynamic bodies retain
  their native values. No motion equations were changed. Joint telemetry is cached in the
  production tick, without renderer angle unwrapping; native motor effort limits are labeled
  as limits rather than actual solved constraint forces.
- Diagnostics run at most 10 Hz, poll nothing while hidden, cap sampled contacts/devices,
  reuse arrows and clean up on replay/close. P creates a separate JSON file; prepared model,
  collision review and calibration are unchanged. See `gui-runner/PHYSICS_DIAGNOSTICS.md`.

## Limits and follow-ups

- Existing level-chassis, welded wheel and flat static support assumptions remain explicit.
- Reliable coefficients and simulation-versus-hardware error still need physical measurements.
- Slopes, suspension, tip-over and individual Mecanum rollers are separate physics extensions.
- Desktop screenshots require an unlocked computer; native headless rendering/snapshot tests
  can run independently. Any unperformed desktop check must be recorded accurately.
