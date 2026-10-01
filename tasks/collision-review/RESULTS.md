# Collision review — implementation and evidence

Date: 2026-10-01. Implements [the phase plan](PLAN.md).

## Highest value finding and fix

A partially collidable robot could previously create an `EmptyShape` for a moving rigid subtree while its chassis had collision geometry. That mechanism could pass through obstacles. The original whole-model check did not catch it.

The new audit uses the simulator's actual rigid-body grouping and wheel exclusions. Each rigid body must have effective collision declarations. Unexpectedly empty bodies fail before any robot native bodies/joints are added or TeamCode starts. Fixed children can share a proxy; missing collision tags on every screw are not treated as separate missing bodies.

An explicit noncontact mechanism can use `collision_omissions` with a body-root name and reason. Unknown roots, blank reasons, chassis omissions and redundant exemptions fail. Its omission is preserved in the report and the body remains joint constrained with its mass/inertia. Existing wheel ballast/tire force and flexible paddle classifications are reported separately.

## Delivered workflow

- `:gui-runner:auditCollisions` writes a JSON coverage report and returns an unsuccessful exit for missing coverage after saving the report. It does not run TeamCode or require hardware XML. Other parsing/mesh errors may prevent report creation.
- Reports identify body members, effective declaration counts, intentional omissions and visual links whose surface coverage relies on a shared proxy. Passing declaration coverage is not a geometric/material accuracy certificate.
- **Review collisions** in the field selector, or `--collision-review`, opens the normal robot/field physical preview with actual Bullet debug shapes. TeamCode is compiled/discovered but never started in review mode.
- C toggles collisions during review or ordinary simulation. V hides/shows CAD without removing native bodies. `--collisions-only` starts with CAD hidden. Existing orbit/zoom and field reset remain available.
- Dynamic bodies are cyan; static bodies are orange. Shapes follow native body positions/rotations and include compounds, convex hulls, robot mechanisms, flexible segments, field apparatus and balls. Tire traction forces are not rendered as rigid wheel surfaces.
- The collision report can also be saved from the normal loading path with `--collision-report`. Report output cannot replace the source URDF or target a symlink.
- Usage is documented in `gui-runner/COLLISION_REVIEW.md`, README and import guides. Local REV DUO `review-collisions.command` and `audit-collisions.command` are ready.

## Validation

**77 Java tests pass:** 5 SDK, 25 physics, 47 GUI/native. Seven new collision-review tests cover:

1. A collidable chassis plus missing slider collider fails with the body's name; zero robot native bodies/joints are added. A report is saved and source overwrite is rejected.
2. Adding that slider's collider lets real contact stop it at an obstacle under applied force.
3. A fixed child supplies a shared body proxy; other visual children remain flagged for surface review.
4. Collision tags only on driven wheel branches cannot hide a missing chassis collider.
5. A reasoned noncontact mechanism remains an explicitly reported, constrained dynamic body.
6. Invalid, blank, chassis and redundant omission configurations fail.
7. The overlay agrees with a native convex child's composed rotated/offset surface, follows body relocation and removal, and adds no native bodies.

**11 Python preparation tests pass.** Existing native powered REV DUO, tire/flexible-contact and reliable torus-retention probes pass, including physical straight drive/turn, chain phase, mass accounting, contact release, acquisition/carry/stopped retention and rejection/overload gates.

The BIOBUZZ native probe passes both field-only and populated modes: 0/56 pieces and 91/147 bodies including its test chassis, with floor/perimeter support, HIVE stops/motion, ball fall/roll/wall response, four CELL contacts, four FLOWER openings and repeated resets.

### Actual supplied CAD

| Input / view | Observed result |
| --- | --- |
| Original REV DUO export | One rigid body with zero collision declarations; JSON saved with `coverage_usable=false`; audit exits unsuccessfully |
| Prepared REV DUO retention project | Three rigid bodies with 10/1/1 effective collision declarations; no missing bodies or intentional omissions |
| Prepared robot visual detail | 566 visual links flagged as relying on aggregate proxies; not labeled individually verified |
| Populated BIOBUZZ/REV DUO overlay | Actual imported field, moving HIVES, balls, robot and flexible contacts visible at their native transforms |
| Populated collision-only view | 185 native bodies / 185 rendered shapes; CAD hidden and physics inventory retained |
| Close REV DUO/torus view | Chassis rail/wheel envelopes, both intake shafts, flexible paddle segments and decomposed torus visible |

All three renderer screenshots were inspected. The field overlay aligns with the rendered perimeter and open goals; decorations and source visual detail do not appear as invented collision surfaces. Native shape inventories match the previous prepared scene.

### Bounds test correction

The first overlay regression compared against the outer compound's broadphase AABB and failed on extents. Native compounds add conservative bounding padding; that box is not the contact surface. The corrected regression compares the actual child's native surface after composing its offset and body rotation. The overlay correctly renders Bullet child surfaces and does not invent padded boxes as collision geometry.

Evidence remains private in `.local/collision-review/evidence/`: real/raw coverage reports, Java/Python logs, native robot/field/torus logs and three screenshots. Source CAD and existing prepared geometry are unchanged and not redistributed.

## Remaining work

- Declaration coverage cannot prove that shared proxies cover every important surface, preserve every opening, or work throughout joint travel. The 566 aggregate-only visual links make that limitation explicit.
- Debug sphere/cylinder/hull meshes are tessellated displays of native shapes; they are not sampled contact points. The overlay is an inspection tool.
- A physical review still requires valid geometry, positive mass/inertia and usable hardware configuration. Failed imports can be inspected through their coverage report and source CAD preview.
- Arbitrary automatic collision candidate generation and geometric clearance scoring remain a later phase. Model-specific REV DUO/BIOBUZZ preparation remains in use. Material and hardware calibration remain separate.
- Field structures use their prepared collision profiles; this phase's declaration coverage gate applies to robot rigid bodies. The overlay exposes actual field shapes without certifying the field's approximations.
- Native renderer paths are verified. The selector review button delegates to the same path; this host's Java accessibility limitation from the field-import phase prevents automated button-click verification.

## Delivery

Implementation, plan, guide and regressions are ready for the shipping PR.
