# Reusable robot and field model preparation

Status: implemented and validated; see RESULTS.md for evidence and the interactive desktop QA limitation.

## Accepted requirements

- Support URDF/STL ZIP packages for robots and fields. Standalone STL, STEP and direct Onshape connections are outside this phase.
- Generate editable preparation defaults. Require a quick collision review before a new or changed model is ready for normal simulation.
- Save a portable model physics profile. Store robot/field placement, modes and game-piece starting arrangements in a separate scene profile.
- Provide a desktop editor with live CAD/collision preview and selectable options. Users can edit and save their settings after the initial import.
- Recognize identical CAD and reuse its saved profile without repeating calibration. Try automatic migration for changed CAD; request user choices or missing measurements when migration cannot resolve them.
- Preserve source CAD. Generated geometry, profiles, reports and review records belong to separate prepared revisions.

## Highest value first

The first deliverable is a shared, versioned profile and preparation backend used by the editor and simulator. Saved values must become the source of simulation settings, rather than another report alongside hardcoded defaults. This closes the repeated-work problem and makes later generation and migration reproducible.

Repository evidence:

- `prepare_rev_duo.py` requires a specific 614-link assembly and embeds collision envelopes, wheel/shaft relationships and contact constants.
- `prepare_biobuzz.py` recognizes specific mesh names, corrects disconnected CELL groups and embeds collision/profile assumptions.
- `FieldPackage` requires two HIVES and a narrow range of field dimensions. It cannot serve as a general field format.
- `CollisionAudit` already checks robot rigid-body coverage and identifies shared proxies; `CollisionOverlay` already displays actual native shapes.
- Swing selection and the macOS GLFW renderer run in separate JVMs. The editor preview must preserve that working process arrangement.

## User workflow

1. Choose **Import robot** or **Import field**, then select a URDF/STL ZIP.
2. The importer extracts and fingerprints the package, validates references and checks the local model library for matching profiles.
3. For identical CAD, select a saved profile and reuse it. For changed CAD, review an automatically proposed migration. For new CAD, create a draft from generated defaults or compatible profile templates.
4. The editor shows CAD and candidate collision geometry together, with a searchable part/body list, assumption indicators and editable properties. Choose roles and resolve any missing information through dropdowns and numeric controls.
5. Review the new/changed collision bodies and unresolved items, run the applicable preparation checks, then **Save reviewed profile**. Review applies to the exact source/settings revision.
6. Select that profile in a scene with robot/field placement and game-piece layout. Reopen the editor or import/export its portable settings whenever needed.

Collision inspection must work before motor bindings or all physics parameters are complete. Full powered simulation continues to require valid hardware bindings, mass/inertia and constraints. A draft may be previewed and tested in the preparation workspace; it cannot be selected as a ready model for a normal run.

## Saved data

### Model profile

Use a versioned JSON schema shared by the preparation tools and Java runtime. Give each saved profile a persistent ID and each prepared revision an immutable revision ID. Keep the following explicit:

| Group | Saved values |
| --- | --- |
| Source identity | Model type, package/content fingerprint, URDF and referenced asset hashes, normalized source identities, source-to-profile part/body mapping |
| Coordinates | Source length units, coordinate basis, origin transform, mesh scales and the resulting metric dimensions |
| Structure and roles | Rigid body grouping, fixed/movable/passive joints, chassis, field surface, barrier, apparatus, game piece, decoration and reference roles |
| Collision | Source/authored/generated provenance; primitive dimensions/poses, convex meshes or hulls, margins, exclusions, contact groups and generation settings |
| Mass and inertia | Source or user values, center of mass, inertia tensor, explicit fallback method and provisional values |
| Materials | Friction, restitution, rolling/spinning friction, contact stiffness/damping where supported |
| Mechanisms | Joint axes/limits, passive spring/damping, actuator roles/bindings, motor/servo parameters, transmissions and mechanical reductions |
| Contact models | Supported drivetrain, tire, flexible-part and retention settings; per-part overrides |
| Preparation fidelity | Mesh simplification tolerance, decomposition limits, thin-surface handling and retained assumptions |
| Review and evidence | Effective settings digest, generation/runtime schema versions, review state, validation results, user notes and calibration provenance |

Each physical assumption has a named parameter, value, units, allowed range and provenance: **source CAD**, **generated default**, **user supplied** or **measured/calibrated**. Store the effective values used by that revision. Updating application defaults must not overwrite a saved model's choices.

Existing simulator limitations must be visible. A parameter cannot claim a physical behavior the engine does not support. Algorithm availability and resource validation are distinguished from adjustable physical assumptions.

### Scene profile

Store references to model profile/revision IDs, robot start placement/yaw, field choice, field-only/game-pieces mode and game-piece spawn/reset layout. Field game-piece types and their physical properties belong to model profiles; instance placements and counts for a practice/match layout belong to the scene.

Hardware names saved in a robot profile are checked against the selected team project. Missing names are resolved using available hardware choices rather than silently rebinding actuators.

### Persistence and portability

Maintain a local model library indexed by model/profile IDs and source fingerprints. Save atomically, retain prior revisions, and allow duplicate profiles for alternative tunings. Export a self-contained profile bundle containing JSON, copied source CAD, generated collision assets and calibration using relative paths. The implementation includes source CAD so a relocated bundle remains editable without the original Downloads path; the export dialog and guide disclose that the CAD is included.

Importing settings validates schema, source compatibility and referenced assets. Reusing a profile must reproduce its effective parameters, generated shapes and review state. No stored profile depends on an old absolute Downloads/worktree path.

## Implementation sequence

### 1. Profile schema, storage and adapters

- Define model, scene, revision and parameter/provenance schemas, with shared validation fixtures.
- Add deterministic save/load, local library lookup and portable profile export/import.
- Convert the current prepared REV DUO and BIOBUZZ settings into explicit profile data. Preserve their working geometry and calibrated/provisional values; capture existing assembly corrections as saved mappings/overrides.
- Add profile-based model selection to configuration. Keep existing project configurations working through compatibility adapters; conflicting legacy/profile settings produce a resolution choice.

**Gate:** export, move and reimport the current profiles without losing collision geometry, settings, bindings or scene independence. Existing native physics probes reproduce their established behavior.

### 2. General ZIP preparation and defaults

- Build a bounded ZIP/URDF/STL reader shared by robot and field import. Validate archive paths, references, finite geometry, rooted transforms and units; preserve original files.
- Resolve all source poses before grouping parts or separating pieces. Use collision data already supplied by CAD when valid; generate candidates for missing physical surfaces.
- Default primitive visuals to corresponding primitive colliders. For mesh visuals, offer bounded convex decomposition and appropriate primitive/thin-surface candidates with visible generation assumptions. Cache reusable collision assets.
- Preserve openings through multipart candidates where supported. Flag uncertain hollow/thin/overlapping geometry for focused review. A whole-assembly box must not silently replace an intake or hollow goal.
- Generate role suggestions from source structure and geometry. Static field structure is the initial field default; loose pieces, special apparatus and floor/surface choices are explicit selections. Show heuristic suggestions as assumptions, not verified classifications.
- Expose unit/origin transforms, material defaults, mass/inertia fallbacks, collision margins and generation tolerances as editable parameters. Ambiguous units or missing structural relationships require a choice.
- Replace the BIOBUZZ-specific field loader contract with a general prepared field format supporting any number of passive mechanisms, arbitrary verified extents, reference/decorative exclusions and optional game pieces. Preserve BIOBUZZ through an adapter.

**Gate:** unrelated synthetic robot and field ZIP packages prepare successfully; neither requires REV DUO/BIOBUZZ names, dimensions or mechanism counts. Missing data remains identifiable and editable.

### 3. Desktop editor and mandatory quick review

- Add a model-library/import entry point for robots and fields, accessible from the existing selector.
- Show searchable parts/bodies, bulk role choices, editable parameter groups and saved-profile selection. Provide units, provenance and concise explanations beside generated assumptions.
- Reuse native collision overlays in an unpowered preparation preview that can open incomplete models. Keep the macOS renderer in its own process; send versioned profile snapshots for debounced live updates. Ignore stale preview results.
- Support changing shape strategy, primitive dimensions/pose, margins, material/mass values, grouping, exclusions and supported mechanism/contact settings. Edit choices through controls; allow precise numeric input when needed.
- Focus review on new/changed bodies and unresolved assumptions. A quick overview confirmation can approve generated collision geometry once required issues are resolved; do not require hundreds of redundant per-fastener confirmations.
- Save reviewed profiles, save drafts, duplicate tunings, export/import settings and restore earlier revisions. Changes invalidate affected review/validation records; saving a draft must not discard the last ready revision.

**Gate:** a user can import, edit, preview, review, save, close and reopen a model with the same effective settings. Normal simulation rejects unresolved or unreviewed revisions.

### 4. Automatic migration with selectable resolutions

- Fingerprint semantic URDF/asset content, ignoring ZIP ordering and timestamps. Identical CAD reuses its compatible reviewed revision without re-calibration.
- Match changed CAD using trustworthy source identifiers where present, then unique names, asset/geometry fingerprints and assembly/joint context. Repeated identical wheels, renamed parts and changed parentage require confidence/ambiguity handling.
- Automatically preserve compatible user material, calibration, mechanism and binding settings on uniquely matched entities. Preserve local-frame settings when placement changes and record the migration decisions.
- Changed dimensions or mesh topology invalidate collision review. Regenerate or propose compatible collision candidates; do not silently stretch old proxies or label them reviewed.
- Present unresolved items with choices: match to one of the new parts; generate defaults for a new part; choose an existing material/parameter template; acknowledge a removed part; repair a hardware binding. Request numeric measurements only when required data cannot be selected or inferred.
- Keep migration as a new draft revision. Retain the old model/profile unchanged, show added/removed/changed entities and parameter differences, and require a quick review of the changed geometry.
- If an imported profile belongs to a different model, offer compatible parameter templates and mapping choices rather than applying the entire profile blindly.

**Gate:** exact reuse, confident migration, ambiguous rename, repeated meshes, removed/new bodies, changed units/dimensions and changed joint/hardware topology all follow predictable paths without losing prior tuning.

### 5. Runtime integration and evidence checks

- Consume the selected profile's effective parameters when constructing robot, field and piece bodies. Reports and previews must use the same prepared collision assets as simulation.
- Expand coverage validation to general field bodies and declared role exclusions as well as robot bodies. Check transforms, initial penetration, body support, dimensions and review/source identity.
- Keep scene layout separate and migrate references when model entity IDs change. Invalidated placements receive selectable alternatives; source-staged/below-floor pieces are not silently adopted as a usable layout.
- Retain native powered/contact/torus and BIOBUZZ evidence gates. Add model-library and scene launch shortcuts after validation.

**Gate:** every edited supported parameter affects the intended runtime property, and saved profile/scene combinations reproduce geometry, physical settings and reset behavior.

### 6. Validate, document and ship

- Test portable save/reload, multiple tunings, atomic saves, version/schema failures, CAD identity and source preservation.
- Test identical repacked ZIP reuse, changed geometry review invalidation, stable mappings, ambiguous migration choice persistence and unchanged measured calibration values.
- Test unrelated robot/field preparation, primitive/convex/thin geometry, openings and missing mass/units/roles.
- Test ready-state enforcement against the exact settings/source digest and preview/runtime collision agreement.
- Inspect real editor/preview screenshots and exercise save/reopen/import/migration workflows. If Java accessibility remains unavailable, separate tested editor controllers from UI controls and document precisely which interactions lack end-to-end evidence.
- Run existing suites and native physics gates, record startup/preview responsiveness and rendering costs, and fix concrete regressions.
- Write implementation results with remaining assumptions, update user guides, commit and ship through a reviewed PR.

## Definition of done

1. New robot and field URDF/STL ZIPs use one repeatable preparation workflow.
2. Physical assumptions and supported generation choices are visible, editable and persisted.
3. Collision geometry is reviewed before a new/changed model is ready for normal simulation.
4. Saved model profiles are reusable and portable; scene placement/layout is stored separately.
5. Identical CAD reuses its tuning. Changed CAD attempts migration and offers choices for unresolved mappings/data.
6. Edits can be made after import without losing the previous ready revision or original source.
7. Editor, preview, reports and runtime use the same effective prepared settings and collision geometry.
8. Current REV DUO/BIOBUZZ behavior and unrelated model fixtures pass meaningful validation gates.
