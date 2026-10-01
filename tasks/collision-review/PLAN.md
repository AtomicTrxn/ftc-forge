# Collision review phase

Implementation and validation: [RESULTS.md](RESULTS.md).

## Priority and evidence

The highest value improvement is detecting partially collidable imports before TeamCode starts. `ImportedRobotScene.parts()` currently substitutes `EmptyShape` when a rigid subtree has no collision elements. `ArticulatedRobot` rejects a wholly visual robot, but allows a collidable chassis with an invisible-to-physics moving arm. This can give misleading mechanism results.

Fixed children may intentionally share a chassis/shaft proxy. Drive wheel branches use the existing traction model; flexible paddles use separately installed beam contacts. Counting missing collision tags per link alone would wrongly reject these models.

## Implementation

1. Share the simulator's rigid subtree grouping with a collision coverage audit. Report body members, effective collision declarations, wheel/flexible treatment and links represented only by aggregate proxies. Reject every unexpectedly empty rigid body before adding native bodies or starting TeamCode.
2. Support explicit `collision_omissions` reasons keyed by rigid body root for deliberate noncontact mechanisms. Validate identifiers, reject chassis omission and stale/redundant exemptions. Show intentional omissions in reports; they do not certify accuracy.
3. Add a read-only project audit command producing JSON, including invalid import findings. It uses real grouping/configuration without requiring actuator hardware or powered simulation. Presence is distinguished from geometric/contact validation.
4. Add native collision overlays to the simulator/physical preview. Render the actual Bullet shape hierarchy at each body's real transform, including convex decompositions, imported field structures, balls and flexible segments. Toggle overlay and CAD visibility; add a review launch path to the desktop field selector.
5. Keep prepared CAD/source files unchanged. Use the report and overlay to guide primitive/convex authoring. Automatic mesh-derived shape authoring and material inference will be a later phase: this phase establishes the checks needed to review such candidates, especially hollow goals and intakes.
6. Document the workflow and current limits, validate the supplied REV DUO and BIOBUZZ packages, and ship through a PR.

## Validation gates

- Native regression: a partially collidable robot with an unprotected moving mechanism fails before any bodies/joints are added; adding the mechanism collider makes actual obstacle contact stop it.
- Fixed subtree proxies, driven wheel branches and configured flexible contact models remain usable. Intentional omissions require valid reasons and cannot exempt the chassis.
- Audit JSON identifies missing bodies and aggregate-only link coverage; a failed audit exits unsuccessfully after saving the report.
- Overlay bounds/placement agree with native shapes for a compound at a rotated offset/principal frame. Visual-only CAD never appears as collision geometry.
- Real renderer screenshots of supplied robot/field collision overlays are inspected; the overlay does not change native body inventories or physical behavior.
- Existing Java/Python suites, powered/contact/torus native probes and BIOBUZZ field gates pass.

## Definition of done

No unexpected empty robot rigid body can start simulation. A user can inspect a saved coverage report and the actual runtime collision geometry for the selected robot/field. Intentional and approximate coverage remain clearly identified, with existing prepared profiles passing regression gates.
