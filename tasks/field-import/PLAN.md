# BIOBUZZ field CAD import plan

## Request and scope

Import the supplied `am-5850 BIOBUZZ.zip` independently of the robot CAD, support field-only and field-with-game-pieces operation, and preserve the physical size relationship between both imports. Implementation was authorized after this plan was drafted; see [results, corrections and validation](RESULTS.md).

Robot selection remains independent: the same REV DUO robot, another imported robot, or the existing generic robot can run on either field source.

## User choices

Expose **Field source** (`Generic` or `Imported CAD`) and **Mode** (`Field only` or `Field + game pieces`) before starting a run. When no field has been imported, select Generic automatically. An explicitly selected imported package that fails validation must produce an actionable error.

| Field source | Field only | Field + game pieces |
| --- | --- | --- |
| Generic / no field import | Existing practice floor and perimeter; no game pieces | Practice floor and perimeter with an explicitly selected piece set/layout |
| Imported BIOBUZZ CAD | CAD floor, perimeter and field apparatus; no loose scoring pieces | Same field plus independent POLLEN and NECTAR bodies |

“Field only” removes loose scoring pieces; FLOWERS, HIVES and other field structures remain part of an imported field. The illustrative torus remains a separately named legacy practice preset. It is never substituted for BIOBUZZ balls.

## Inspection evidence

Source: `/Users/tomhess/Downloads/am-5850 BIOBUZZ.zip`.

- SHA-256: `182d84076f54d9e975251a5ba9cf04cc11beba65a4e0ac576a64c8fe2ce30d7c`.
- 107 ZIP entries: 105 binary STL meshes, one URDF, one ROS launch file; approximately 60.5 MB compressed / 172.7 MB expanded.
- URDF: `am_5850_biobuzz/urdf/am_5850_biobuzz.urdf`; one rooted tree with 1,081 links, 1,080 joints, 997 visuals and **no collision elements**. All declared mesh scales are `1 1 1`; all mesh references resolve inside the archive.
- 36 floor tiles, 40 POLLEN, 8 blue NECTAR, 8 red NECTAR, and 4 `Flower_Scoring_Volume` references.
- Five nonfixed joints: three planar-placement degrees of freedom attached to a tape assembly, and two revolute HIVE joints. Placement joints should become fixed prepared poses; HIVE pivots require separate passive physics.
- Approximately 3.43 million unique triangles and 15.97 million triangles across visual instances. Goal rib meshes each have 630,088 triangles and the top corner has 542,778, exceeding the current visual loader's 500,000-triangle limit. Many also exceed the collision decomposition limit of 200,000 triangles.
- At joint position zero, transformed tile bounding boxes span approximately **3.58574 × 3.58573 m**, tile top is approximately source Z=0, thickness is 14.962 mm, and representative placement increments are 596.953 mm. These are source CAD measurements, not certified playable dimensions.
- Local bounding dimensions: POLLEN approximately 71.12 × 71.04 × 70.24 mm; NECTAR approximately 91.94 × 91.87 × 90.52 mm. CAD masses are about 20.98 g and 40.59 g respectively; treat these as uncalibrated CAD estimates.
- Some piece placements lie outside the perimeter or below the tile surface at zero joint positions. Some are children of other pieces or HIVE links. Bake complete assembly transforms before separating/removing pieces.
- The full assembly bounding box includes external infrastructure and is approximately 6.38 m wide. It cannot define the playable field or determine a scale factor.

The [FIRST BIOBUZZ Competition Manual, TU02, sections 9.2 and 9.8](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/cm-html/BIOBUZZ%20Competition%20Manual%20-%20TU02.htm) specifies an approximately 144-inch square interior, nominal 24-inch tiles, approximately 2.8-inch POLLEN and 3.6-inch NECTAR, and 40/8/8 piece totals. The piece counts and dimensions agree with the export. The CAD floor footprint still needs reconciliation against the inside wall faces and [official field setup resources](https://ftc-resources.firstinspires.org/ftc/archive/2027/game).

## Implementation sequence

### 1. Prepare and audit the package

- Add `tools/prepare_biobuzz.py`, accepting this ZIP or an extracted URDF package. Check entry paths, expansion limits, binary STL structure, finite coordinates and package-local mesh references before extraction.
- Preserve original bytes in ignored `.local/fields/biobuzz/source`; produce a separate prepared package, manifest and audit report. Do not execute the included ROS launch file.
- Resolve every link/visual pose through the complete source tree before categorizing instances. Classify by source mesh identity and explicit manifest overrides: floor, perimeter, fixed apparatus, HIVE assemblies, loose scoring pieces, nonphysical reference volumes and visual detail.
- Record mesh dimensions, units, instance IDs, materials, source poses, joint state and provenance. Ensure every visual receives a classification.
- Measure interior wall-to-wall distances, floor coverage/seams, HIVE pivot heights and representative ball dimensions. Explain the difference from the nominal generic field. Use official setup measurements to resolve misplaced assembly parts through documented placement corrections; do not stretch the entire CAD assembly.
- Retain a source-layout preview and prepare a separate simulation layout. Validate any corrected starting positions, HIVE initial angles and staged pieces. Label layouts as CAD/practice or verified match setup according to evidence.

**Gate:** all references resolve, units are established, and a reviewed dimension/layout report explains the floor and perimeter measurements before physics integration.

### 2. Establish one metric world and field configuration

- Add an optional `field` block to `SimConfig` holding source, prepared package path and mode. Store field assets/layout metadata separately from robot hardware configuration. Retain explicit legacy torus presets for existing demos.
- Use meters throughout. The existing URDF-to-jME basis is `(x, y, z) -> (x, z, -y)`. Define the field center and tile top as the shared world origin/surface, and apply only declared rigid placement transforms after verified unit conversion.
- Preserve the robot's prepared dimensions and the field's verified dimensions. Camera fit adjusts viewpoint only. No independent auto-fitting, nonuniform scaling, or scale derived from overall assembly bounds.
- Add robot start X/Y/yaw in the shared field frame, validate floor support and obstacles, and replace hardcoded +/-1.8 m piece-start checks with actual field/layout bounds. Staging objects outside the field are handled explicitly.
- Log source units, conversions, field interior dimensions, robot wheel/track dimensions and selected layout at startup. Invalid scales/poses fail with useful messages.

**Gate:** importing/selecting the field never changes robot size; generic and imported scenes share the same physical meter convention.

### 3. Render the field and build appropriate physics

- Add `ImportedFieldScene` and shared URDF pose/mesh-resolution utilities. Reuse safe XML parsing and STL loading where applicable, without requiring robot motors/transmissions for passive field elements.
- Cache each unique mesh and instance its geometry/materials. Prepare reduced visual meshes for oversized parts with measured dimensional error; retain an optional full-detail preview path with explicit memory/triangle budgets. Avoid simply increasing limits without measuring startup/memory/frame performance.
- Replace generic floor/walls when using an imported field. Use one continuous floor collision surface at the verified tile top; retain CAD tiles visually. There must be no duplicated floor or perimeter collision.
- Build explicit primitive/compound convex collision proxies for perimeter, frame and FLOWERS. Preserve accessible openings, ramps and HIVE cell interiors. Exclude tape, fasteners, logos and scoring reference volumes from collision. Validate each proxy against its CAD surface; do not collide every rivet or collapse a hollow structure to one solid hull.
- Separate HIVE moving assemblies at the two revolute joints. Implement passive hinges, stops and measured or documented provisional mass/inertia/damping. Validate supported initial poses and tipping with finite contact forces. The URDF's exporter effort/velocity values are not calibrated motor specifications.
- Record any missing physical parameters, collision approximation or unvalidated HIVE behavior in the field profile and results. A static visual preview is an intermediate milestone; interactive game mode needs working collision bodies and passive apparatus.

**Gate:** the robot can drive on the floor, contact barriers/frame, and approach FLOWER/HIVE openings without invisible generic walls or explosive overlaps.

### 4. Support multiple game pieces and both modes

- Replace the single `PhysicsWorld.gamePieceControl` assumption with an instance registry containing ID, type, alliance/color, pose, mass, collision body and initial/reset state.
- For field-only, spawn zero loose pieces and disable piece-dependent reporting/controllers safely. Keep non-piece physics active.
- For BIOBUZZ game mode, use actual ball meshes and appropriately sized convex/spherical collision proxies, with any hole/compliance approximation stated. Use CAD mass/inertia as provisional defaults and expose friction/restitution/material overrides for later measurements.
- Detach pieces from source fixed joints after baking their initial world poses. Validate floor penetration, inter-piece overlap, HIVE support and out-of-field staging; resolve documented layout errors before stepping physics. Reset must restore the selected layout without duplicate bodies.
- Update flexible intake contacts, containment reporting and selection to work across typed bodies. Keep torus retention restricted to torus practice pieces; POLLEN/NECTAR require their own validated contact behavior. An incompatible torus profile should be clearly identified before the run.
- Game mode includes physical pushing, rolling and contacts with robot/apparatus. Automatic scoring, complete match officiating and simulated AprilTag vision are separate follow-up features; preserve relevant metadata for them.

**Gate:** zero pieces in field-only; the configured piece inventory in game mode; stable reset and physical response for each piece type.

### 5. Make selection usable

- Add a small pre-run field selector/import action to the current desktop launch flow, plus equivalent CLI/config options for reproducible runs. Do not require editing robot URDFs or changing hardware bindings to switch fields.
- Show Generic as the no-import default, the imported field name, both modes and selected starting layout. Provide a preview with robot and field together, metric measurements and camera orbit/zoom.
- Default new unconfigured runs to Generic / Field only. Keep existing intake and torus demos working by explicitly selecting their practice presets.
- Update local REV DUO launchers with named field-only and BIOBUZZ game options after validation, and document import/reset/start-pose usage in `gui-runner/FIELD_IMPORT.md` and README.

### 6. Validate and deliver

- Preparation tests: archive/path failures, classification coverage, mesh budgets, transform composition, nested pieces, preserved dimensions and deterministic preparation.
- Configuration tests: absent field fallback, both modes, independent robot choice, missing explicitly selected package, invalid units/start poses and incompatible piece-specific models.
- Native physics probes: robot floor support; boundary/obstacle contact; all piece types falling/rolling/contacting; populated scene stability; HIVE stops/tipping; repeated resets; no initial explosive penetrations.
- Scale assertions: prepared/source representative dimensions agree within 1 mm; REV DUO wheel diameter stays 90 mm and track approximately 381 mm; ball sizes stay consistent with source dimensions; field proxy boundaries agree with measured CAD boundaries within 5 mm. Official nominal tolerances are checked separately from numerical import accuracy.
- Confirm proxy opening clearance for the actual balls, not only overall bounds. Verify geometry and collider transforms coincide.
- Run existing Java/Python suites, powered REV DUO drive/contact/torus probes, and Dashboard/watchdog scenarios. Capture real renderer evidence for Generic field-only, imported field-only and imported field + pieces, with both the actual REV DUO and the generic robot.
- Record preparation/startup time, memory, frame timing and fixed-step stability on this Mac with the populated scene. Require no physics backlog and no sustained rendering below 30 FPS in the standard view; reduce optional detail if needed, without changing physical dimensions.
- Write `tasks/field-import/RESULTS.md` with commands, measurements, screenshots, dimensions, source-layout corrections and remaining calibration gaps. Then commit, push and ship through a reviewed PR once implementation is authorized.

## Definition of done

1. No field import produces a usable generic field-only run.
2. Both sources support the two modes without changing the chosen robot model or physical scale.
3. Imported field structures retain verified metric dimensions, with the source/nominal discrepancy explained.
4. Field-only contains no loose scoring pieces; BIOBUZZ game mode uses the configured actual POLLEN/NECTAR instances and preserves independent dynamics.
5. Floor, perimeter, apparatus, pieces and robot have aligned visuals/collision and validated contacts; resets reproduce the selected state.
6. Performance and existing simulator behaviors pass the evidence gates above, with provisional material/goal parameters identified.
