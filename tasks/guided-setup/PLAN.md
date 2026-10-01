# Guided import and simulation setup

Status: approved by the user and implemented. Automated/native validation is recorded in RESULTS.md; interactive desktop QA remains pending an unlocked Mac.

## Objective

A user should be able to import a robot or field, understand the choices needed to prepare it, review its collision geometry, save reusable settings, and set up a simulation without knowing the internal configuration keys or the order of editor actions.

Recommendation: make a **resumable guided setup wizard** the default entry point. Each page explains what to do, why it matters, and how to recognize a satisfactory result. Keep the existing detailed editor available through **Advanced settings** within the relevant step.

The highest value change is sequencing the existing working import, migration, preview and save operations with clear instructions and actionable checks. Reuse their implementations and integrity rules.

## Current application evidence

- `ModelPreparationEditor` exposes import, migration, binding, preview, review, export and project selection as two toolbars. The user must infer the sequence.
- Its parameter table displays internal JSON paths; help consists primarily of a units paragraph and raw assumption data.
- `SceneEditor` is a separate dialog with revision selectors, coordinate rows and piece layouts. Importing and creating a runnable scene are disconnected flows.
- Native validation already constructs the same geometry used by simulation, but it is an unpowered consistency check. Visual collision inspection remains a separate user action.
- Model revisions, exact CAD reuse, migration choices, portable bundles and scene identity checks already exist and should underpin the guide.
- Model preview does not require a team project. Normal simulator startup currently discovers an OpMode and validates hardware even for its scene preview; scene preflight should become reusable independently of running TeamCode.
- The previous release has automated/controller/native evidence, but complete interactive editor click-through was blocked by a locked desktop. Guided setup needs an explicit interactive QA gate.

## Proposed experience

### Entry and navigation

Provide **Guided setup** from the model editor and field launcher. The welcome page offers:

1. **Set up a simulation**: select or import the robot and field, then create the scene and choose a team project.
2. **Prepare a robot or field**: save a reusable model without requiring TeamCode or hardware configuration.
3. **Continue saved setup**: reopen an unfinished session at its first incomplete or invalidated step.

A left sidebar shows step names and status. The main panel contains a short instruction, focused controls, contextual help and an issue list. The footer has **Back**, **Next**, **Save and close**, and the appropriate final action. Completed earlier steps remain accessible. Optional steps are explicitly labeled.

For a full simulation, model preparation runs for each model that needs work; the sidebar identifies **Robot**, **Field** and **Scene** sections. A saved reviewed model follows the shorter reuse path.

### Steps

| Step | Instructions and controls | Completion condition |
| --- | --- | --- |
| 1. Choose setup | Choose model-only or simulation setup; optionally select a team project. Explain what can be completed without one. | Goal selected; any chosen project location is valid. |
| 2. Choose models | Import a robot/field URDF/STL ZIP, select a saved revision, import a profile bundle, or capture an existing preparation. For the field, offer **Generic field — no CAD import needed**. Display detected package contents and saved matches. | Supported source loaded or a saved/generic selection made; extraction/reference failures have actionable fixes. |
| 3. Check size and orientation | Display overall dimensions with familiar units, a metric grid and labeled axes. Choose source units, upward direction and robot forward direction; show the effective result in preview. Ask the user to confirm the model's dimensions make sense. | Explicit scale/orientation confirmation for new or changed CAD. Geometry remains at true metric scale. |
| 4. Identify parts and movement | Show grouped bodies and movable joints. Robot: choose wheels/intake/joints and map supported motors/servos from the project's hardware names. Field: distinguish fixed structures, passive mechanisms, loose pieces, floor and decoration. Explain that welded parts move together. | Ambiguous structural/migration choices resolved. Hardware can be deferred for model-only preparation; simulation readiness requires valid bindings. |
| 5. Review physics assumptions | Group controls as mass, contact surfaces, collision shapes and mechanisms. Show **From CAD**, **Default — needs confirmation**, **User supplied** or **Measured/calibrated** values. Offer **Keep this default for now** with a short explanation, or a number/choice control. | Supported required values are valid; provisional values remain visibly provisional and saved. |
| 6. Review collisions and save model | Open native CAD/collision preview, focus selected problem bodies, explain overlay colors and provide a short inspection checklist. Run native construction checks, resolve failures, then explicitly save the reviewed revision. | Native evidence matches current settings, migrations are resolved, and the user confirms visual review. Saved models satisfy the existing ready-state guard. |
| 7. Set up the scene | Select **Field only** or **Field with game pieces** independently of generic/imported field choice. Place the robot, choose piece instances, duplicate/disable them and preview the combined layout. Preserve valid saved placements when changing models. | Selected models are ready; scene references and starting layout pass preflight. This step is optional for model-only preparation. |
| 8. Finish and use | Show saved model names/revisions, scene, provisional assumptions and outstanding project requirements. Offer export, use in project and launch a discovered OpMode when ready. | A saved model or validated scene is produced. Apply/launch requires project compatibility; model-only users can finish without it. |

### Instructions the user will see

**Size:** “Check the width, length and height against your robot. If a 45 cm robot appears as 0.45 mm or 450 m, change the source units before continuing.”

**Collision shapes:** “The colored outline is the surface that objects collide with. Check that wheels touch the floor and openings remain open. A box around an intake can block game pieces even when the drawing looks correct.”

**Missing mass:** “This part has no usable mass in the CAD. We supplied a starting value. Enter a measured mass if you have it, or keep the default for now and tune it later.”

**Migration:** “This part changed in the new CAD. Choose the matching old part to keep its settings, or generate new defaults. Your previous saved model remains available.”

**Field modes:** “Field only includes the floor, walls and field mechanisms. Field with game pieces also includes the movable pieces you select below.”

**Finish without a project:** “Your model is saved and collision-reviewed. Choose a team project later to map hardware and run robot code.”

Use friendly parameter names, visible units, concise descriptions and selectable fixes. Put raw diagnostic output behind **Technical details**. Size plausibility suggestions are advisory because unrelated models and non-FTC dimensions are supported. Never resize the field or robot merely to make their drawings look proportional.

## Conditional paths

### Existing reviewed model / unchanged CAD

- Verify identity and assets, then reuse the reviewed revision and calibration.
- Skip already satisfied model checks. Show a summary with **Use saved settings** and **Change settings**.
- Recheck project hardware and scene compatibility if those contexts changed.
- **Change settings** creates an editable draft and invalidates the affected validation/review according to existing rules.
- A matching portable bundle follows the same reuse rule after import/integrity checks.

### Changed CAD

- Present a summary of matched, changed, added and removed entities.
- Keep confident migrations; offer selectable resolutions for unresolved mappings and bindings.
- Direct attention to changed or uncertain bodies. Show unchanged settings for context without asking for hundreds of repeated confirmations.
- Require the existing native check and explicit collision review before saving the new ready revision.

### No project or incomplete hardware

- Allow full geometry preparation, review and model saving.
- Clearly identify deferred motor/servo mappings and their effect on simulation readiness.
- Offer **Choose project now** and **Finish model preparation** instead of an unexplained disabled Next button.
- Project binding changes that alter the effective model produce a new reviewed revision.

### Error recovery

- Show issues on the relevant page with **Select file**, **Choose units**, **Map part**, **Change shape**, **Choose hardware** or **Edit placement** actions.
- Preserve the draft when imports, native construction or scene checks fail.
- Show meaningful operation stages during extraction, preparation and validation; do not invent percentages or completion times.
- Prevent double submissions and ignore stale compile/preview results after navigation or source replacement.

## Saving and resuming

Save a versioned setup-session record separately from model physics and scene JSON, under the local library's `sessions/` directory. It records goal, selected models/drafts, project, scene draft, current step and acknowledgments tied to relevant source/settings digests.

- Persist changes and step transitions atomically; expose **Save and close**.
- Resume from actual current validity, not just stored completion flags. Missing assets, changed settings and outdated validation records return the user to the appropriate step.
- Editing a model does not overwrite an earlier reviewed revision. Editing scene placement does not invalidate model collision review.
- Keep setup-session metadata out of model physics digests and exported calibration.
- Model bundles remain portable; workflow sessions are local working state. Reopening imported models can create a new session without old absolute paths.
- Native proof and ready-state checks remain authoritative; completing a wizard page cannot bypass them.

## Implementation plan after approval

### Phase 1 — Workflow controller and navigation

- Add a testable setup state/controller independent of Swing.
- Define robot, field, full-simulation, reuse and resume paths, prerequisites and completion states.
- Add atomic session persistence and dependency-based invalidation.
- Introduce a wizard shell and entry points; route existing operations through their current backend/controller.

**Gate:** import, save/close and resume cannot lose state or mark an invalid model ready.

### Phase 2 — Guided model pages

- Build friendly source, scale, body/role, hardware and physical-assumption pages.
- Add parameter labels, units, ranges and contextual help; keep advanced editing available against the same draft.
- Present migration choices in a review queue with a change summary.
- Add preview focus/highlight commands for selected bodies through separate preview-view state; camera/selection changes do not modify model settings or review digests.
- Add explicit visual inspection and native validation results to the review/save page.

**Gate:** new and changed robot/field packages can be prepared through the guided controls; saved models follow the short reuse path.

### Phase 3 — Scene setup and project handoff

- Integrate scene selection, mode, robot pose and piece layout into the guide.
- Extract shared layout/contact checks from simulator startup into a reusable scene-preflight component, consumed by both wizard and simulator.
- Separate model/scene preview readiness from TeamCode execution readiness.
- Check project hardware and discover available OpModes for the final handoff; display and resolve project/profile setting conflicts.
- Preview/preflight does not start an OpMode. **Run selected OpMode** remains an explicit final action.
- Show a finish summary and export controls, including that portable bundles contain source CAD.

**Gate:** a complete robot/field setup produces the same valid profiles/configuration as the established runtime path; failures return to the relevant page.

### Phase 4 — Validation, documentation and shipment

- Test conditional navigation, resume, stale-result handling, review invalidation and model-ready versus simulation-ready states.
- Test generic/imported fields in both modes, identical CAD reuse, changed-CAD migration, missing hardware, missing meshes, wrong units, thin/hollow collision geometry and invalid layouts.
- Exercise new robot import, new field import, saved reuse, migration, interrupted/resumed setup and model-only completion through the actual desktop UI.
- Run the existing Java/Python suites and relevant real REV DUO/BIOBUZZ native regression gates.
- Update the user guide and record evidence, then ship through a PR after implementation is authorized.

**Interactive QA gate:** the desktop must be unlocked for complete mouse/keyboard verification. If it is unavailable, report that specific outstanding gate; automated controller/native tests do not count as complete interactive evidence.

## Acceptance criteria

1. A new user can follow visible instructions from supported CAD ZIP selection to a saved reviewed model.
2. A complete setup can select generic/imported field and either piece mode, save a valid scene, connect a project and launch a chosen OpMode.
3. Every required decision has a description, units where relevant and a selectable option or focused input.
4. Existing reviewed CAD/calibration is reused without repeated collision review when unchanged.
5. Changed CAD offers migration choices and focuses review on changed/uncertain geometry.
6. Save/close/resume preserves work and recomputes any invalidated completion state.
7. Provisional assumptions remain identifiable and editable after finishing.
8. Geometry review, scene readiness and project execution readiness are distinguishable and accurately enforced.
9. Native preview, saved settings and simulation continue to agree; existing physics gates pass.
10. Interactive workflows are exercised and documented before claiming full guided-setup QA.

## Scope

This proposal covers the guided desktop experience and the shared preflight/state support needed for it. Current URDF/STL ZIP support, physics algorithms and profile integrity remain the foundation. A full CAD editor, arbitrary soft-body physics, new CAD file formats and direct Onshape integration would be separate work.

Recommended defaults for review: guided entry first, advanced settings available per step, automatic local draft/session saving, explicit collision-review approval, and optional project connection for model-only preparation.
