# Reusable robot and field preparation

Use one workflow for a **ZIP containing one URDF and its referenced STL meshes**. STEP, direct Onshape access and standalone STL assemblies are not supported by this workflow. Python 3.10+ and the existing JDK 17 installation are required; the preparation backend uses Python's standard library.

```sh
./gradlew :gui-runner:prepareModels
# Include a team project to select hardware bindings and apply saved models:
./gradlew :gui-runner:prepareModels --args='.local/models /absolute/team-project'
```

The field selector also has **Guided model and scene setup…**. The detailed table editor is available with `./gradlew :gui-runner:advancedModels`.

## Guided setup

The default entry opens a wizard. Choose **Set up a simulation**, **Prepare a robot**, or **Prepare a field**. A project is optional while preparing a model.

1. **Choose model**: select a CAD ZIP, saved revision, portable bundle or captured preparation. In a full simulation, **Generic field — no CAD import** skips field preparation. Identical reviewed CAD skips completed model checks. **Generate fresh defaults** starts a separate profile without relying on damaged or unwanted saved settings.
2. **Size and orientation**: read metric dimensions, select source units, inspect the grid/axes preview and confirm the scale. Robot forward direction is editable. Captured BIOBUZZ retains its authored metric basis.
3. **Parts and movement**: resolve migration choices, choose roles/joint movement and bind motors/servos from a selected project. Hardware mapping can be deferred for model-only preparation.
4. **Physics assumptions**: use named controls with explanations and units. Values retain CAD/default/user/calibration provenance. Robot models also offer **Measure robot and verify motors…** for [operating weight, wheel dimensions and motor bindings](ROBOT_MEASUREMENTS.md). Confirm that remaining defaults are provisional. **Advanced settings** edits the same draft. Close that editor to return to the guide.
5. **Review and save**: inspect cyan moving shapes, orange fixed shapes and a magenta focused body. Confirm the checklist, then **Run checks and save reviewed model**. Native checks cannot replace visual review or measurement.
6. **Arrange scene**: choose field-only/game-pieces mode, robot pose, piece templates, duplication/disable and saved poses. **Preview combined scene** requires neither hardware nor TeamCode. **Check scene** validates exact model identities, field bounds and native contacts.
7. **Finish and use**: export the model bundle, or choose a project and physics precedence, check hardware and discover OpModes. **Use and run selected OpMode** explicitly starts TeamCode; discovery and previews do not.

Every step includes instructions. Select a parameter to read its full explanation and provenance below the table. File/folder dialogs include an optional absolute-path control for hidden library locations. **Back** revisits earlier pages, **Save and close** retains work, and **Continue saved setup…** resumes the first incomplete or invalidated check. Missing assets or invalid edited values retain the draft and return to the relevant page. Full diagnostics are available through **Technical details**.

Sessions are local working records under `.local/models/sessions/`; they are separate from physical settings and scene layouts. A reused model is collision-reviewed, while running code additionally requires valid scene placement, project configuration and hardware. Model-only preparation can finish without a project. Editing scene placement does not invalidate model collision review.

```sh
# Unpowered scene checks and preview; no TeamCode discovery required:
./gradlew :gui-runner:checkScene --args='/absolute/scene.json'
./gradlew :gui-runner:previewScene --args='/absolute/scene.json'
```

See [guided setup implementation evidence](../tasks/guided-setup/RESULTS.md) for automated checks and the remaining interactive QA gate.

## Import, tune and review

1. Choose **Import robot** or **Import field** and select the CAD ZIP. Automatically reuse identical CAD, or choose an older saved revision for migration.
2. Inspect **Defaults**, the searchable **Parts / bodies** list, and **Runtime / contact models**. Generated values and source assumptions are displayed beside effective values. Choose source units and up axis before tuning dimensions.
3. Open **Live collision preview**. It requires neither TeamCode nor a hardware map. Cyan wireframes are dynamic native bodies; orange wireframes are static bodies. Drag to orbit, scroll to zoom, C toggles shapes, V toggles CAD. Edits compile after a short debounce and reload the preview. Construction failures show a CAD-only view and prevent approval.
4. Resolve migration choices, inspect openings and thin surfaces, and check coordinates, mass and body coverage. Choose **Save reviewed profile** when the geometry is satisfactory. A native construction/constraint check must pass for these exact settings. **Save draft** retains the earlier reviewed revision.
5. Use a reviewed robot/field in the project, or choose **Scene…** to pin both models and save their layout. If the project already uses a scene, model selection updates its revision reference while retaining placement.

Importing identical CAD selects the existing reviewed revision. Opening it for editing creates a separate draft. No calibration is repeated; the saved settings and assets are retained. **Alternative tuning** creates another profile identity; **Open revision** restores an earlier tuning as a draft.

### Collision and physical controls

| Control | Meaning |
| --- | --- |
| Source | Preserve authored URDF collisions. |
| Visual / mesh | Use visual primitives or STL geometry; native V-HACD decomposes contact meshes with the saved hull limit. |
| Box / sphere / cylinder | Explicit primitive dimensions and pose. Boxes can fill openings; inspect before approving. |
| None / decoration / reference | Remove colliders. Entire nonphysical bodies receive explicit coverage exclusions; physical bodies still require usable coverage. |
| Mass / inertia | Source values, an override, or explicit fallback mass with box inertia; COM and full inertia tensor are editable. |
| Materials | Global defaults and overrides on native body roots. A welded body has one material; fixed children cannot claim independent friction. |
| Passive joints | Parent, type, pose, axis, limits, rest position, spring and damping. Sliders use N/m, N·s/m and meters; hinges use N·m/rad, N·m·s/rad and radians. Native stability checks reject invalid trees or inertias. |
| Fidelity | Separate visual/contact mesh tolerances, hull limit, contact margin, minimum candidate thickness. |
| Generated floor | For general fields without an inferred surface, an explicit floor uses saved extents, thickness, height and material values. |

All profile dimensions are meters except **joint xyz and prismatic source limits**, which use the selected source units. Angles are radians, mass is kg, inertia is kg·m². Units alter the source geometry, not the camera framing. Generated primitive dimensions follow a unit change; numeric values explicitly supplied by the user retain their meter interpretation. Profile identity fingerprints normalized URDF structure and referenced STL content, ignoring ZIP order and timestamps. Re-triangulated or byte-changed assets are conservatively treated as changed CAD.

Global compliance is opt-in (`use_contact_compliance`); stiffness and damping are saved independently. The existing tire, flexible intake, retention and servo settings are editable under Runtime when captured from a prepared project. **Add drive / intake…** supplies editable defaults with project motor choices. **Bind selected joint…** selects a motor/servo and exposes mechanical reduction; new servo bindings receive provisional, editable effort/speed/travel defaults. Flexible intake visuals use saved indices and a saved mesh-to-beam orientation, so portable asset filenames need not be `Flap.stl`.

The flexible model still represents the supported segmented paddle geometry. It is not a general soft-body solver. Native construction validation is a consistency check, not evidence of measured friction, mass, rubber properties or real robot accuracy.

## Model profiles and scenes

The default library is `.local/models`:

```text
models/<profile-id>/revisions/<revision-id>/
  profile.json             source mapping, parameters, runtime tuning, review
  source/                  preserved normalized URDF and original STL bytes
  prepared/                effective geometry and portable calibration
  receipt.json             hashes, artifacts, body grouping, effective settings
  validation.json          native evidence for the exact settings digest
drafts/<draft-id>/           editable preparation workspace
```

Saved revisions are immutable. Editing copies a revision into a draft; atomic saves retain prior versions. Normal profile-based simulation rejects unresolved/draft revisions or changed settings/assets. Legacy direct URDF and `field.json` configurations remain available for compatibility and existing probes; adopt profiles to obtain revision integrity and the mandatory review workflow.

**Export profile…** creates a portable ZIP containing settings, source CAD copies, prepared assets and calibration. Those CAD copies are included in the export: treat the bundle as sharing the model itself. **Import profile bundle** restores it without old Downloads/worktree paths. Model bundles have bounded extraction and reject traversal, symlinks and unsupported versions. Public repository commits do not include private CAD/library assets.

Scene JSON stores model/revision references, robot pose, field-only/game-pieces mode and piece poses. Select source piece instances, duplicate or disable rows, and change starting positions/rotations. R restores those saved poses. Model physical properties stay in the model revision. Scene references use paths relative to the scene file; moving a library and its scenes together retains those references. For a separately relocated model, reopen the scene and select the replacement revision. Missing piece IDs receive selectable placement mappings or source-layout choices.

Robot model settings can conflict with older project physics entries. Choose saved model or project values explicitly; the choice is saved as `model_settings`. Decomposition changes require review in the model editor. Hardware names continue to be checked against the selected project's FTC configuration; the editor offers available hardware names for rebinding. Changing geometry or bindings creates another reviewed revision.

```json
{
  "robot_model_profile": "/absolute/library/models/robot-id/revisions/revision-id/profile.json",
  "model_settings": "profile",
  "scene_profile": "practice-scene.json"
}
```

A scene pins exact IDs as well as paths. A stale identity or penetrating/out-of-bounds starting layout is rejected with an actionable message. General piece poses refer to the piece assembly root; its CAD center-of-mass offset is preserved.

## Existing REV DUO and BIOBUZZ preparations

Use **Capture prepared model** to capture a working REV DUO project (`sim.config`) or the prepared BIOBUZZ `field.json`. This retains authored proxies, drive/intake/contact tuning, calibrated data when present, and BIOBUZZ's assembly corrections, skins, ring openings, passive HIVE and piece profiles.

The BIOBUZZ adapter exposes its actual authored shapes and mechanical values under `Runtime / field_manifest`, rather than pretending that generic per-link candidates control it. Its metric basis is fixed. It retains separate fixed-surface and piece materials, including the zero rolling friction of its fixed surfaces. General imports use the general field format and have no two-HIVE or FTC-specific size requirement. A changed BIOBUZZ export that cannot safely migrate its authored corrections requires an explicit regeneration choice and review of all general field bodies.

The previously prepared source remains unchanged. The current team's validated local revisions and scene live in `.local/models`; they are not part of the public repository.

## Backend and native checks

```sh
python3 tools/model_preparation.py import /absolute/CAD.zip .local/models --kind robot
python3 tools/model_preparation.py import /absolute/new-CAD.zip .local/models --kind field --reuse /absolute/older/profile.json
python3 tools/model_preparation.py capture-robot /absolute/prepared-project .local/models
python3 tools/model_preparation.py capture-field /absolute/prepared/field.json .local/models
python3 tools/model_preparation.py edit /absolute/saved/profile.json .local/models
python3 tools/model_preparation.py compile /absolute/draft/profile.json
./gradlew :gui-runner:previewModel --args='/absolute/draft/profile.json'
./gradlew :gui-runner:validateModel --args='/absolute/draft/profile.json'
# After inspecting the collision geometry:
python3 tools/model_preparation.py save /absolute/draft/profile.json .local/models --reviewed
python3 tools/model_preparation.py export /absolute/saved/profile.json /absolute/model-profile.zip
python3 tools/model_preparation.py import-profile /absolute/model-profile.zip .local/models
```

Migration matches names, compatible geometry and assembly context, preserving stable entity IDs and user joint/contact overrides. Changed dimensions, topology, repeated/ambiguous parts, removed parents and changed transmission bindings receive choices. Hardware names are not mistaken for same-named CAD parts. Unsupported old contact references can be mapped to new parts or disabled explicitly, including their dependent models. Calibration values are retained; they are not regenerated from CAD.

Use `verifyRobotPhysics`, `verifyTorusRetention`, and `verifyFieldPhysics` for the existing powered/native gates. The BIOBUZZ probe accepts a reviewed BIOBUZZ `profile.json` as well as a legacy manifest. See the [implementation evidence](../tasks/model-preparation/RESULTS.md) for test coverage and desktop QA limitations.
