# Reusable model preparation — implementation and evidence

Implemented and validated on October 1, 2026. The accepted plan is in [PLAN.md](PLAN.md); the user workflow is in [MODEL_PREPARATION.md](../../gui-runner/MODEL_PREPARATION.md).

## Delivered

- Shared bounded URDF/STL ZIP import for robot and field models, source copies, content identity, explicit editable defaults and assumption provenance.
- Versioned model library with immutable saved revisions, alternative tunings, editable drafts, portable bundles and calibration preservation. Bundles include copied source CAD as well as prepared geometry/settings; the export UI discloses this.
- Swing model editor with searchable parts, typed choices, hardware bindings, migration resolutions, native collision preview, draft/review saves and project selection.
- Separate scene editor for pinned model revisions, field-only/game-pieces modes, robot placement and duplicate/disabled piece instances. Selecting a replacement model preserves an active scene's placements.
- Automatic unchanged-CAD reuse and conservative migration of changed CAD. Compatible settings and joint overrides survive; ambiguous parts, removed references and changed transmissions receive choices.
- General field runtime with static structure, arbitrary passive mechanisms, independent rolling piece assemblies and generated/source floor choices. BIOBUZZ's authored assembly corrections remain supported through a captured adapter.
- Shared preview/runtime construction and integrity guards. New/changed settings require an explicit collision review and native validation for the exact effective digest. Normal profile-based simulation rejects drafts, unresolved migrations, tampered assets and stale scene references.
- Explicit passive hinge and slider spring/damping/rest controls with angular and linear units. Native body materials are applied at body roots; welded children cannot claim independent friction.

## Automated checks

| Gate | Result |
| --- | --- |
| `./gradlew test` | 84 tests passed: core SDK 5, physics engine 25, GUI/native/controller 54; no failures/errors |
| `python3 -m unittest discover -s tools -p 'test_*.py'` | 27 tests passed, including 16 preparation tests |
| Unrelated robot/field ZIP fixtures | Generated primitives, static floor, passive gate, freely rotating sphere, piece duplication, COM-preserving root placement and reset passed |
| Native passive slider | Nonzero linear spring moves toward the saved meter rest position |
| Review/integrity | Draft rejection, immutable saved revisions, changed settings/source/artifact detection and exact identity pinning passed |
| Migration | Unique rename, changed geometry, changed hardware, removal/disable dependencies, joint override preservation and same-named motor protection passed |
| Library reuse | Identical repacked CAD chooses an existing reviewed revision even when a newer draft exists; robot/field settings cannot be interchanged |
| Scene/project selection | Separate scene values, preserved placements, explicit project/profile precedence and missing piece template rejection passed |
| `git diff --check` | Passed |

Python fixtures use a synthetic validation record to isolate persistence behavior. The Java tests and real-model gates below construct and step actual Minie/Bullet bodies; Python fixture records are not the native evidence.

## Real REV DUO and BIOBUZZ evidence

The user's source ZIPs and existing working preparations were retained unchanged. Local captures preserve authored geometry, drive/intake/tire/flexible/retention settings and calibration data when present.

| Evidence | Observed result |
| --- | --- |
| REV DUO native preparation | 39 bodies: 3 rigid bodies plus 36 flexible segments; source assembly mass 2.727383929318 kg; six paddles transfer 0.027 kg |
| BIOBUZZ native preparation | 146 field/piece bodies, 56 pieces, two passive HIVES |
| Normal simulator collision scene | 185 native shapes/bodies; saved robot and field at metric scale |
| Saved robot `verifyRobotPhysics` | Forward travel about 1.209 m, supported tire contacts, intake containment, reverse release and turn yaw about 0.722 rad passed |
| Saved robot `verifyTorusRetention` | Capture/carry/stop/reverse/coast/stall/overload gates passed; bounded force and torque |
| Saved field `verifyFieldPhysics` | Both modes passed: field-only 0 pieces/91 probe bodies; game-pieces 56 pieces/147 probe bodies; support, perimeter, CELL contacts, FLOWER openings, HIVE motion and reset |
| Legacy field regression | Both legacy modes passed after material correction |
| Actual original BIOBUZZ ZIP reimport | Returned the existing reviewed revision, without new calibration/review; about 6.236 seconds |
| Real portable robot bundle | Exported/imported into another library; import/compile about 0.819 seconds; native rebuild reproduced 39 bodies |
| Real portable field bundle | Exported/imported into another library; import/compile about 1.074 seconds; native rebuild reproduced 146 bodies |

Native field probe wall times were about 0.17 seconds for 14.5 simulated seconds and 0.61 seconds for 27.5 simulated seconds on this machine. These are headless probe measurements, not general renderer performance promises. Repeated cached REV compilation was about 0.48 seconds during development.

Private evidence remains ignored under `.local/models/`: `full-tests.log`, `python-tests.log`, `robot-validation.log`, `field-validation.log`, `profile-robot-physics.log`, `profile-retention.log`, `profile-field-physics.log`, `legacy-field-physics.log`, `portable-reuse-result.json`, `real-reimport-result.json`, and the corresponding portable validation logs. Renderer screenshots `rev-duo-profile.png`, `biobuzz-profile.png` and `profile-scene.png` were visually inspected. These private artifacts and CAD files are not committed.

## Problems found and corrected

1. **Tiny source masses rejected:** the original numeric floor excluded real fasteners around 1e-7 kg. Positive source masses down to 1e-12 kg are supported; fallback mass remains a separate explicit parameter.
2. **Captured zero-mass parts inflated robot mass:** applying fallback mass to intentionally massless visual links added roughly 3 kg. Captures now retain source zero values and reproduce the original total mass.
3. **Field bodies inherited robot drive constraints:** generic field/piece construction installed drive braking and chassis-level locking. Local field worlds disable the drive controller and permit full XYZ piece rotation.
4. **Portable filenames broke flexible visuals:** matching only `Flap.stl` failed after source assets received content-hash filenames. Profiles retain visual indices and mesh-to-beam orientation; the old filename fallback remains compatible.
5. **Preview CAD rendered black:** lighting was attached before the prepared root joined the scene. Lights are now applied to each prepared preview root; subsequent screenshots show the CAD and native overlay.
6. **Field material regression closed a FLOWER gate:** accidentally applying piece restitution/rolling/compliance settings to fixed surfaces changed contact behavior. The existing native opening probe failed. Fixed-surface coefficients are now separate saved parameters, compliance is opt-in, and both saved and legacy field probes pass.
7. **Scene placement lost piece COM offsets:** a second placement pass treated assembly-root positions as body COM positions. Generic pieces are constructed at root poses once; a native offset-COM test verifies the resulting position.
8. **Migration renamed hardware or reset joint overrides:** broad string replacement could rename a motor matching a part name, and source-pose changes could overwrite calibrated joint settings. Typed references and per-property joint merging now preserve those values.
9. **Model selection remained pinned to an old scene revision:** selecting a new model updated project configuration but the scene overrode it. Selection now updates the scene reference/identity and retains placements; field selection preserves existing robot precedence.
10. **macOS path aliases broke relative references:** `/var` versus `/private/var` produced invalid paths in scene/model selection tests. Scene paths and saved model references now use canonical filesystem paths.
11. **Editor retained stale property/model state:** switching captures/imports could leave a prior saved selection or table references. Switching clears those references; scene loading suppresses unrelated field-change prompts and rebuilds source layout before applying saved placements.
12. **Missing generic scene pieces were silently skipped:** runtime now rejects an enabled placement without a source template and points to selectable scene migration.

## Remaining limits and QA coverage

- **Interactive editor click-through is unverified.** The Swing editor launched without Java exceptions, but computer-use inventory reported a locked Mac and failed automatic unlock. No native UI interactions were fabricated. Editor operations/controllers, save/load/migration behavior, native previews and normal renderer output have automated or screenshot evidence; the complete mouse/keyboard workflow still needs an unlocked desktop check.
- Imported geometry does not provide measured friction, rubber stiffness, mass or drivetrain calibration. Defaults are saved and editable; users must review generated boxes, floor/extents/roles and openings. A native construction check is a short unpowered consistency test, not a physical accuracy certification.
- Mesh byte/topology changes are treated conservatively. Whole boxes can fill holes; changed BIOBUZZ CAD may require regenerating general bodies because its authored corrections cannot always migrate safely.
- The flexible model supports the existing segmented intake paddle representation, not arbitrary soft bodies. Native welded bodies have one contact material.
- Legacy direct URDF/`field.json` configurations remain compatible. The mandatory profile review/integrity guards apply when the model-profile workflow is selected.
- Only URDF with referenced STL ZIP packages and version-1 model bundles are supported in this workflow. Bundles include source CAD; scene files remain separate and relocated model references can be selected again in the scene editor.

## Reproduction

```sh
./gradlew :gui-runner:prepareModels --args='.local/models /absolute/team-project'
./gradlew test
python3 -m unittest discover -s tools -p 'test_*.py'
./gradlew :gui-runner:validateModel --args='/absolute/draft/profile.json'
./gradlew :gui-runner:verifyRobotPhysics --args='/absolute/profile-project'
./gradlew :gui-runner:verifyTorusRetention --args='/absolute/profile-project'
./gradlew :gui-runner:verifyFieldPhysics --args='/absolute/reviewed/BIOBUZZ/profile.json'
```
