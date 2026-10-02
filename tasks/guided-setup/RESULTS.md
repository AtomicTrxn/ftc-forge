# Guided model and simulation setup — implementation results

Date: 2026-10-01 (updated after desktop walkthrough)

Status: implemented and validated. Automated/native gates and the remaining interactive desktop release checks passed; ready to ship.

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
| Full Java suite | 95 tests passed: 5 core SDK, 25 physics, 65 GUI/controller/native tests | `.local/models/guided-ui-tests.log`; module JUnit XML reports |
| Python suites | 29 tests passed, including fresh-import recovery and removal-queue correction after retaining settings | `.local/models/guided-python-tests.log` |
| New guided workflow gates | 11 tests passed, including Swing spinner/editor state restoration | `GuidedSetupTest` |
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
8. On macOS the Swing open chooser hid its filename field. A visible optional absolute-path control now permits selecting files/folders, including hidden library locations. Normal browsing remains available. Import, export and project-folder selection were exercised through the new control.
9. Busy-state snapshots were captured while recursively disabling controls. `JSpinner.setEnabled(false)` also disabled its editor before that editor's original state was captured, leaving numeric placements disabled afterward. Capture now happens in a separate pass before disabling. A regression test and actual placement editing after a native check verify the fix; originally unavailable buttons remain disabled.
10. Returning from Advanced settings left the reused instruction component dimmed. The guide now restores its controls before rebuilding the page. The corrected return was visually inspected and exercised.
11. Completed operation text remained on later pages, and scene readiness/checkmarks remained stale after edits. Navigation now sets current instructions; scene edits immediately update readiness and the sidebar. Actual UI tests verified invalidation, out-of-bounds rejection and recovery.
12. Long parameter descriptions were clipped in table cells. A wrapped selection detail area shows the full explanation/provenance, and provenance receives more column width. The corrected layout was inspected and selection details exercised.
13. After manually retaining old part settings, that part remained in the removal-acknowledgment queue. Migration now removes retained parts from that queue and drops empty acknowledgments; the earlier saved revision remains unchanged and the changed model still requires review. A dedicated Python regression passes. Piece-template choices and table rendering now use readable CAD/type names while preserving internal IDs.

14. A motor selected through the binding dialog and its generated gear ratio inherited a CAD provenance label. Binding now records the motor name as user supplied and the ratio as a provisional default; the final desktop panel verified both labels.

## Desktop interaction evidence

The Mac was unlocked for this follow-up. Plain Java processes appeared in native inventory but could not be selected by the desktop tool. Temporary local `jpackage` app bundles under ignored `.local/desktop-qa/` provided stable app identities. They run the same compiled wizard/model-preview/scene-preview entry points; the bundles are QA artifacts, not a new distribution feature. All mouse/keyboard actions used the Computer tool. Native preview child processes were also opened through the wizard; companion app bundles allowed visual inspection of those same entry points.

Actual walkthroughs completed:

1. New robot ZIP import, metric dimensions, millimeter conversion and restoration, explicit scale confirmation, deferred hardware, save/close/resume to Parts, advanced-editor open/return, invalid fallback mass rejection and correction, collision preview/CAD toggle/zoom, review gate, matching native construction, portable export and **model-only Finish without a project**. The saved session has `finished=true`; its exported bundle contains source/prepared/settings files.
2. Saved reviewed robot selection satisfied the preparation checks and Next skipped directly to field selection.
3. Generic field selection, field-only preflight, enabling six game pieces, duplicating a piece with an offset, disabling its source instance, saved game-piece preflight and combined native scene inspection.
4. After the busy-state fix, editing Robot X after a native check remained possible. X=5 immediately invalidated scene/Finish readiness, native preflight rejected it, and restoring X=-1.2 recovered a valid checked scene.
5. New field ZIP import, 2 m dimensions, changing the ball role from structure to independent piece using a dropdown, advanced return, readable physics help, fixed-floor/moving-piece native preview, explicit review and save. Both imported-field modes passed preflight after adjusting the robot pose to the smaller field.
6. Switching from generic to imported field retained the old scene layout and offered a selectable replacement template. Selecting a replacement updated that placement; **Use model piece layout** then loaded the field's own piece layout for successful checking.
7. Project folder selection, a missing `left_front_drive` error from compatibility checks, and recovery by selecting the valid project.
8. Changed robot CAD import with a selected prior revision, renewed dimension confirmation, named **Use settings: base** migration and acknowledgment of the old removal queue. The redundant removal entry exposed item 13 above. Repeating the import on the corrected build cleared both pending choices after retaining base, with no redundant removal acknowledgment.
9. Selected the movable wheel and bound it to project motor **left_front_drive**, inspected the changed model through native preview, saved the reviewed revision and rechecked the smaller imported-field scene. Piece table and replacement choices displayed **ball** while retaining the underlying source ID.
10. Project checks discovered **Guided UI launch check** without creating its class-load or initialization markers. Selecting it and clicking **Use and run selected OpMode** created both markers (`loaded` and `init`), applied the saved scene with model-settings policy `profile`, and displayed successful launch. Simulation Finish completed.
11. Imported the previously exported robot profile bundle through the UI; its intact collision review was reused and Next skipped directly to Finish. Importing identical original CAD with automatic reuse also retained review. Selecting **Generate fresh defaults** for that same CAD created an unreviewed draft and Next required size/orientation confirmation.
12. On the final build, edited a saved model and bound its movable joint to **arm_motor**. The panel displayed **User supplied** for the motor name and **Default — provisional** for the generated ratio 1.0. The earlier reviewed revision and scene remain available; this last editable QA draft was saved separately.

Private evidence: `.local/desktop-qa/models/sessions/3929eec5-b6e4-4694-8499-a9d84de6630f.json` records completed robot preparation; `355ad7a3-140e-415f-8c57-f4df3a2d42f7.json` records simulation/field/migration work. Their native check logs and scene JSON are stored beside them. UI states/screenshots are visible in the testing conversation. Synthetic test CAD/projects and the exported robot bundle remain ignored under `.local/desktop-qa/fixtures/`. Existing real REV DUO/BIOBUZZ regression evidence remains as recorded above.

Desktop lock interruptions and a temporary automatic-approval usage-limit error delayed testing. After desktop/tool access resumed, all release checks above were completed. Probe markers are `.local/desktop-qa/fixtures/team/guided-class-loaded.txt` and `guided-launch-init.txt`; the fresh-import session is `5b6b51c9-5ddb-4223-a2cd-f0889d03802a.json`. The final GUI build and its 65 tests passed after the provenance correction; the unchanged core/physics suites retain their 30 passing results.

## Limits

- URDF plus referenced STL ZIP packages remain the supported source format. Captured supported preparations and exported profile bundles can also be selected; this is not direct Onshape or STEP import.
- Default mass, friction, box proxies, tire/elastic parameters and retention assumptions remain provisional. Native construction and collision inspection do not establish measured physical accuracy.
- The flexible intake model retains its supported segmented paddle representation; preparation does not add a general soft-body solver.
- Session files are local workflow state. Exported model bundles carry CAD/settings/assets and integrity records; they do not export wizard sessions or team projects.
- Placement checks certify the saved starting configuration, not every future trajectory or the behavior of arbitrary team code.
