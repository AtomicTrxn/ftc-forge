# Guided model and simulation setup — implementation results

Date: 2026-10-01

Status: implemented with passing automated and native validation. **Desktop interaction QA is outstanding; retain the PR as a draft until that gate is completed.**

## Delivered

- Guided setup is the default model-preparation entry point and is available from the field selector. Instructions cover source selection, size/orientation, parts/movement, assumptions, collision inspection, scene placement and project handoff.
- Robot-only and field-only preparation can finish without TeamCode. Complete simulation setup accepts a generic field or a reviewed imported field, independently of field-only/game-pieces mode.
- Existing reviewed models and identical CAD reuse their settings. Changed CAD uses the existing selectable migration queue. An explicit fresh-defaults option recovers from unusable saved assets without altering the old model.
- Named parameter controls expose units, explanations and CAD/default/user/calibration provenance. Advanced editing uses the same draft. Visual review and matching native checks remain required before saving a reviewed model.
- Atomic local sessions retain references, drafts and acknowledgments. Resume recomputes readiness and affected step completion. Workflow metadata, selected preview body and camera state are separate from model physics and exported settings.
- Native model preview adds metric guides and focused body highlighting. The combined scene preview constructs actual native bodies without a project, powered drive controller or TeamCode.
- Scene placement checks are shared with simulator startup. They inspect reviewed identities, rotated robot footprint, complete native piece bounds, floor position and penetrating contacts. Piece templates, duplicated/disabled instances and reset poses work for generic and imported fields.
- Project handoff validates typed runtime settings, URDF bindings, motor/servo hardware, servo physics and OpMode discovery before writing configuration. Discovery does not initialize TeamCode. Running a selected OpMode is an explicit final action.

## Validation evidence

Private CAD, prepared model assets, logs and renderer screenshots remain in ignored `.local/`; no user CAD is included in the change.

| Gate | Result | Local evidence |
| --- | --- | --- |
| Full Java suite | 94 tests passed: 5 core SDK, 25 physics, 64 GUI/controller/native tests | `.local/models/guided-tests.log`; module JUnit XML reports |
| Python suites | 28 tests passed, including explicit fresh-import recovery from damaged saved assets | `.local/models/guided-python-tests.log` |
| New guided workflow gates | 10 tests passed | `GuidedSetupTest` |
| Real reviewed REV DUO + BIOBUZZ scene | Valid placement; **185 native bodies**, no TeamCode started | `.local/models/guided-real-scene.log` |
| Real powered robot physics | Forward drive, intake contact/containment, reverse release and turn passed | `.local/models/guided-powered-regressions.log` |
| Real torus retention | Acquisition/carry/turn/stopped retention/reverse release variations and inactive/reverse/coasting/stalled/overload gates passed | `.local/models/guided-powered-regressions.log` |
| Real BIOBUZZ field-only | 0 pieces, 91 probe bodies; 14.5 simulated seconds, about 0.16 wall seconds | `.local/models/guided-field-regressions.log` |
| Real BIOBUZZ game-pieces | 56 pieces, 147 probe bodies; 27.5 simulated seconds, about 0.60 wall seconds; support, CELL/FLOWER/HIVE and reset checks passed | `.local/models/guided-field-regressions.log` |
| Combined scene rendering | New unpowered scene preview rendered and its output was visually inspected | `.local/models/guided-scene-preview.log`, `.local/models/guided-scene.png` |
| Established simulator rendering | Real startup rendered the same combined robot/field at metric scale, with 185 collision bodies/shapes; output was visually inspected | `.local/models/guided-runtime.log`, `.local/models/guided-runtime.png` |

The guided tests cover conditional navigation, rejection of misleading completion flags, unchanged-CAD reuse, immutable saved models, dirty-draft resume and missing files, scene proof invalidation, unchanged-placement proof retention, generic piece duplication/disable/reset, partial piece footprint outside the field, imported field modes, missing templates, project configuration preservation, discovery without static TeamCode initialization, invalid drive dimensions, and generated floors with an empty assembly frame.

## Problems found and corrected

1. Separating every physical child into independent field pieces left a zero-mass assembly frame without collision geometry. Constructing it as a chassis failed. It now remains a visual transform frame; the generated floor and separated piece bodies are still built and validated.
2. Saving an unchanged scene for preview discarded its successful placement proof. The controller now retains proof only when its exact scene/model token still matches. Changed placements or revisions require another check.
3. Back navigation initially required forward-step acknowledgment. Back now flushes edits without demanding a visual/provisional confirmation intended for Next.
4. Backend geometry compilation could accept invalid runtime drive dimensions. Guided compilation now validates the typed runtime configuration before accepting the step.
5. Scene bounds previously tested piece centers. They now use complete native bounds, including rotation, with the same 4 mm contact tolerance. A regression case places the center inside the field but part of the shape beyond its boundary above the walls.
6. An edited calibration value could still display a measured label. Explicit user provenance now takes precedence.
7. Invalid advanced-editor values could trap the user in that window during close. The draft is retained and returned to the guide for correction even when preparation fails; invalid data still cannot pass review or readiness checks.

## Outstanding desktop QA gate

The wizard launched without a Java startup error, but the Mac locked before its screens could be exercised with mouse/keyboard. Native-app inventory repeatedly returned: **“The Mac is locked and automatic unlock could not unlock it.”** The user was asked to unlock the desktop while independent validation continued.

The rendered simulator/preview screenshots and automated controller tests do **not** complete this gate. On an unlocked desktop, exercise:

1. A new robot import through units/orientation, movement/binding, assumptions, collision preview, explicit review and model-only finish.
2. A new field import with role/piece selection and both scene modes.
3. Saved reviewed reuse, changed-CAD migration choices and fresh-default recovery.
4. Save/close/resume, an invalid edit, Back navigation and advanced-editor return.
5. Generic/imported scene placement, duplicate/disable/reset, project selection, hardware failures, export and explicit OpMode launch.

Record actual interactions and any fixes here before marking the PR ready or merging it.

## Limits

- URDF plus referenced STL ZIP packages remain the supported source format. Captured supported preparations and exported profile bundles can also be selected; this is not direct Onshape or STEP import.
- Default mass, friction, box proxies, tire/elastic parameters and retention assumptions remain provisional. Native construction and collision inspection do not establish measured physical accuracy.
- The flexible intake model retains its supported segmented paddle representation; preparation does not add a general soft-body solver.
- Session files are local workflow state. Exported model bundles carry CAD/settings/assets and integrity records; they do not export wizard sessions or team projects.
- Placement checks certify the saved starting configuration, not every future trajectory or the behavior of arbitrary team code.
