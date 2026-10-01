# Reviewing an imported model's collisions

A CAD visual does not create a collision surface. FTC Forge now checks collision declarations for every robot rigid body, using the same fixed-subtree and drive-wheel grouping as the simulator. A chassis collider cannot hide a missing arm/slide collider. An unexpectedly empty body stops the run before robot bodies or TeamCode start.

## 1. Save the coverage report

From the repository root:

```sh
./gradlew :gui-runner:auditCollisions --args='/absolute/robotProject /absolute/collision-report.json'
```

The project needs `sim.config` with its `urdf` path. Motor hardware and TeamCode are not needed by this command. It loads the CAD geometry, applies the project's drive/flexible contact classification, and writes JSON. An unresolved coverage error causes an unsuccessful exit **after** the report is saved. XML/mesh/configuration errors can prevent the report from being created.

The report contains each rigid body's members, collision declaration count, status and omission reason. Every link has a treatment:

| Treatment | Meaning |
| --- | --- |
| `declared_on_link` | Collision geometry is declared on this link |
| `aggregate_proxy_review_required` | A fixed assembly shares a body proxy; this visual link's surface coverage needs review |
| `drive_wheel_ballast` / `tire_contact_model` | Existing drivetrain contact treatment replaces wheel-link rigid collision |
| `flexible_contacts_plus_rigid_proxy` | Configured paddle contacts are installed separately from shaft collision |
| `assembly_or_inertial_frame` | No visual surface; mass/frame belongs to its body |
| `missing` | Its rigid body has no effective collision declarations |
| `intentional_noncontact` | Its rigid body has an explicit reason for having no collision response |

`coverage_usable=true` means declaration coverage passes. It does **not** establish accurate geometry, decomposition, mass, materials, or working actuators. Shared proxies frequently omit screws, ribs or other detail; they can also miss important surfaces. Review the identified members and actual collision shapes.

## 2. Inspect actual native collision shapes

Choose **Review collisions** in the desktop field selector, or run:

```sh
./gradlew :gui-runner:runSimulatorApp --args='/absolute/robotProject OpModeName --collision-review --collision-report /absolute/collision-report.json'
```

Select Generic/imported field and field-only/game-pieces as usual. This is a physical preview: TeamCode is compiled/discovered to use the normal project loading path, but it is not started. Gravity and passive contacts still run. The coverage report is saved before robot construction and may contain errors when construction is blocked.

- **C:** toggle the native collision overlay, also available during ordinary simulation.
- **V:** hide/show CAD rendering while keeping physics active.
- **Drag / scroll:** orbit and zoom.
- **R:** reset the field and pieces.
- **Cyan:** dynamic shapes, including robot mechanisms, flexible beam segments and balls.
- **Orange:** static shapes, including floor and field apparatus.

Add `--collisions-only` to start with CAD hidden. An optional PNG argument captures the preview and closes. The overlay uses Bullet's actual primitive/convex/compound shapes and body transforms; it does not substitute visual meshes or padded broadphase bounding boxes. Wireframe is drawn through CAD so internal proxies are visible. Sphere/cylinder/hull display meshes are debug tessellations, not exact rendered contact points. Drive tire forces/rays are not rigid collision shapes and are not shown as wheel colliders.

If an import fails coverage, use its report and `previewRobot` to inspect the source CAD, then add the required collision geometry to a **prepared copy**. A model must pass coverage and existing mass/inertia/hardware checks before the physical overlay can run.

## 3. Resolve intentional noncontact mechanisms explicitly

For an internal mechanism that intentionally has no environment contact, add a reason keyed by its **rigid body root link**:

```json
"collision_omissions": {
  "internal_sensor_carriage": "Internal sensor carriage intentionally has no environment contact"
}
```

The body stays dynamic and joint constrained, with mass/inertia, but has no contact shape. This is an explicit physical approximation. Unknown body names, blank reasons, omissions on already collidable bodies, and chassis omissions are rejected. Fixed hardware sharing a collidable body does not need exemptions. Existing wheel/flexible models remain explicit in the report.

## Authoring and validation

Use primitives for simple solid parts; use several convex pieces for irregular bodies. Preserve hollow intakes and goal openings. Review shape placement, thickness, margins and reach throughout joint travel; test obstruction and release with native contacts. The BIOBUZZ backstop correction is an example of a whole-part box incorrectly filling an opening.

Automatic authoring of arbitrary collision meshes is a later phase. Existing REV DUO/BIOBUZZ preparation profiles already add model-specific geometry. This phase makes missing body coverage visible and provides the review tools required to judge future generated candidates. Material properties and hardware calibration remain separate.

See [robot imports](ROBOT_IMPORT.md), [field imports](FIELD_IMPORT.md), and [phase evidence](../tasks/collision-review/RESULTS.md).
