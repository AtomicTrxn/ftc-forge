# Imported drivetrain support and contact ownership

## Problem

Imported drive wheels are currently welded into assembly mass but omitted from native collision geometry. The aggregate velocity-target drive therefore moves a chassis proxy, while the preparation pipeline applies default sliding friction to that proxy. At low commanded speeds the drive impulse can be canceled by chassis/floor friction. Setting every chassis surface to zero friction also removes legitimate scraping and obstacle friction, and the aggregate drive can produce traction while airborne.

The previous synthetic test package additionally has provisional wheel geometry/floor alignment. A physical correction must distinguish valid wheel support from a chassis resting on its belly; it must not manufacture a successful movement for invalid geometry.

## Implementation plan

1. Reproduce the low-power/default-friction behavior in a native imported robot fixture. Establish an aligned wheel-supported fixture, an unsupported/airborne fixture and a grounded chassis with wheels clear of the floor.
2. Add an explicit, portable `drive_contacts` mode for prepared general robot imports. Keep existing saved profiles and direct projects on their existing mode unless the user enables the new mode, edits the draft and reviews collisions. Expose support-normal and contact-distance assumptions as editable parameters; use saved welded-body/floor material coefficients.
3. Include configured drive-wheel collision geometry in the welded body's compound shape, retaining mass/inertia once and source transforms. Identify wheel children even for decomposed mesh geometry. Preserve wheel collision response against obstacles. Wheels remain welded proxies; this phase does not add detailed rotating treads or Mecanum rollers.
4. Use native wheel/support contacts to supply traction only when supported. Let the aggregate drive own tangential wheel/floor response, with finite impulses capped by available normal load and material friction. Keep chassis/mechanism friction, wall friction, gravity, external pushes and normal contact response under Bullet. Support the optional tire solver without double-counting native wheel-floor friction.
5. Add setup/demo diagnostics for missing wheel contact and chassis-floor scraping, so bad CAD alignment remains an actionable correction. Update collision coverage and native review identity for the selected mode. Save/export/import/migrate the settings through the existing preparation workflow.
6. Verify native low-power drive at default friction, all drive directions, air/no-support behavior, belly contact, wall blocking/friction, external pushes, material limits, transformed/decomposed wheel geometry and timestep stability. Run existing REV tire/intake/retention, motion review, model preparation and full regression suites. Document results, private-package evidence and limitations, then ship a merged PR.

## Accuracy boundaries

The aggregate drive remains a calibrated speed-target approximation, with a level chassis and simplified wheel geometry. Native support and material bounds improve contact consistency; they do not establish measured motor loading or real-world traction accuracy. Existing tire mode retains its shaft-reaction model. CAD axes, dimensions, masses and material coefficients require user review or measurements. Invalid wheel/floor alignment must be reported and corrected, not hidden by larger demo commands or zeroing chassis friction.

## Completion

Implemented and verified. See [results](RESULTS.md) for native and desktop evidence, regression commands and remaining accuracy limits.
