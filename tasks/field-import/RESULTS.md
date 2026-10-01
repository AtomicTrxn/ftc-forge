# BIOBUZZ field import — implementation and evidence

Date: 2026-10-01. Implemented from [the approved plan](PLAN.md).

## Delivered

- Standard-library ZIP/URDF preparation for the supplied BIOBUZZ assembly, including original source copies, mesh provenance, bounded visual preparation, collision profiles and a dimensional audit.
- Independent field source and mode: Generic / Imported CAD, and Field only / Field + game pieces. An absent field block defaults to Generic / Field only.
- Desktop field selector/importer, config/CLI overrides, physical pre-run preview, and source/practice CAD audit preview. Camera orbit/zoom changes only viewpoint.
- One imported floor, CAD perimeter/apparatus collision, two passive HIVE assemblies with CAD aggregate mass/inertia, and a typed registry for all 56 scoring balls. No generic floor/walls are added in imported mode.
- Field-only has zero loose pieces. Populated mode has 40 POLLEN, 8 red NECTAR and 8 blue NECTAR as independent native dynamic bodies. Generic game mode has an explicit six-ball practice set or legacy torus preset.
- `R` restores piece/HIVE state without duplicate bodies. Initial robot CAD footprints and native field/piece penetrations are checked before TeamCode starts.
- Legacy demos explicitly select their torus practice preset. Torus-specific retention is inactive when running actual balls; their collision response remains physical.
- Field/robot guide, runnable generic/REV DUO field demos, native field probe, and local field-only/game/selection/verification launchers.

## Scale evidence

Source ZIP SHA-256: `182d84076f54d9e975251a5ba9cf04cc11beba65a4e0ac576a64c8fe2ce30d7c`. Original ZIP and URDF bytes remain unchanged. All source field mesh scales are `1 1 1`; source and prepared dimensions are in meters.

| Measurement | Result |
| --- | --- |
| Imported inside wall distances | 3.59023683 × 3.59023633 m |
| CAD tile footprint | approximately 3.58574 × 3.58573 m |
| Generic field | 3.6576 × 3.6576 m |
| Prepared/source maximum per-axis mesh-bound deviation | below 0.5 mm |
| Visual retained-vertex movement bound | approximately 0.433 mm |
| POLLEN collision diameter | 71.12 mm |
| NECTAR collision diameter | approximately 91.94 mm |
| REV DUO wheel diameter / track | 90 mm / 380.996749 mm |
| REV DUO prepared mass | 2.727383929 kg |
| Complete HIVE CAD masses | approximately 2.422187 kg each |

No bounding-box fitting or independent field/robot resizing occurs. Camera framing is separate. Field collision boundaries use the measured CAD interior planes, so imported visuals/collision align at the source scale.

The supplied field interior is approximately 67.36 mm narrower than the simulator's nominal generic field. This is an assembly/dimension discrepancy, not a unit conversion error: ball sizes and the source HIVE pivot height also agree with the meter convention. The import preserves the source dimensions and is labeled **CAD practice field**. It does not claim to reproduce a certified nominal competition field. See [FIRST's nominal dimensions](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/cm-html/BIOBUZZ%20Competition%20Manual%20-%20TU02.htm), sections 9.2 and 9.8.

## Problems reproduced and corrected

### Disconnected HIVE cells

The source's two revolute branches contain pivot/damper hardware only. Its four CELL assemblies are separate fixed subtrees attached directly to `root`. Following the exported joint tree alone left moving hardware without CELL collision skins; the first native probe rejected that assembly.

Preparation now bakes every source pose, identifies the four colored CELL groups by their rib meshes, and explicitly assigns both cells and their hardware to the corresponding HIVE pivot. Loose balls are removed from that rigid grouping. Each complete HIVE's source mass, center of mass and full inertia are aggregated in principal axes. Corrections are recorded in `cell_group_corrections`.

### Native hinge reference angle

The native hinge's angle is relative to its automatically chosen frame, which differs from URDF zero after principal-axis transformation. Using its absolute angle/sign caused an immediate stop violation. The importer now measures the native reference and derivative sign with a small reversible construction-time rotation, restores the original pose, and offsets the physical stops/reporting correctly. No runtime pose writes control the goal.

### Backstop clearance

A whole-part box filled the source backstop's L-shaped opening. An actual pollen-drop test failed. Preparation now rasterizes its real XZ silhouette into merged convex boxes with 3 mm cells and preserved thickness, rather than covering its empty center. All four FLOWER pollen-clearance gates then passed.

### Offset ball mesh origins

A final dimensional audit found that the NECTAR STL origins are offset from their centers. Using maximum absolute coordinates inflated the blue collision diameter to 115.53 mm and red to 101.01 mm. Preparation now derives radius from half the maximum mesh dimension and records the bounds center. Practice visuals are translated around the sphere center, giving both colors the source 91.94 mm diameter. Original source-layout transforms remain unchanged. A Python regression covers offset origins; the native probe checks every ball’s visual/body center and radius within 0.5 mm. The corrected populated REV DUO scene was rendered and rechecked.

### Visual cost

The unprepared field's goal rib/top corner meshes exceed existing loader limits. The source field has approximately 15.97 million displayed triangles before preparation; the detailed robot adds approximately 10.19 million. Initial REV DUO field rendering measured about 17 FPS.

Field preparation reduced the standard displayed field to approximately 1.86 million triangles. Metric clustering of dense **robot visuals only** reduced unique robot triangles from 4,840,520 to 688,929. The original collision meshes and mass/inertia remain in use. Flexible paddle visuals have independent reduced deformable buffers. A unit test verifies the visual movement bound and that source buffers are untouched. Resetting the renderer timer after initialization prevents preparation time from entering the first simulated frame.

## Validation

### Automated checks

- **70 Java tests pass** (5 SDK, 25 physics, 40 GUI/native integration), including field defaults/modes, rejected units/configuration, meter-preserving visual simplification and rotated articulated start/joint axes.
- **11 Python tests pass**, including archive traversal/symlink rejection, metric visual bounds, deterministic clustering, nested pose composition, explicit piece classification, convex opening preservation, offset ball-origin dimensions and the L-shaped backstop regression, plus existing REV DUO preparation checks.
- Native field probe passes **both modes**: 0 / 56 pieces; 91 / 147 bodies including the test chassis. It simulates 14.5 / 27.5 seconds respectively. Wall-clock physics/probe times were approximately 0.18 / 0.58 seconds.
- Native gates cover floor support, imported perimeter collision, finite body states/speeds, all three ball types falling/rolling/contacting walls, visual/collider centers and radii within 0.5 mm, HIVE stops and finite-torque motion, actual pollen contacts with all four CELL floors, pollen passage through all four FLOWER openings, and three repeated resets without duplicate bodies.
- Existing native powered REV DUO probe passes straight drive, chain phase, capture/release and turning. The tire-slip/flexible-intake contact probe also passes straight drive, mass accounting, physical release and turning.
- Existing native torus probe still passes acquisition, carry/turn, stopped BRAKE/FLOAT retention, reverse release, repeated cycles, phase/offset variation, uncontacted/inactive/reverse/coasting/stalled rejection and overload breakaway.
- Real Dashboard sample completes normally; deliberately hung TeamCode is marked DEAD by its watchdog.

Representative commands (from repository root):

```sh
python3 tools/prepare_biobuzz.py '/absolute/am-5850 BIOBUZZ.zip' .local/fields/biobuzz/prepared
python3 -m unittest discover -s tools -p 'test_*.py'
./gradlew test
./gradlew :gui-runner:verifyFieldPhysics --args='.local/fields/biobuzz/prepared/field.json'
./gradlew :gui-runner:verifyTorusRetention --args='.local/robots/rev-duo-starter/retention-project'
./gradlew :gui-runner:run --args='gui-runner/sample-teamcode DashboardOpMode'
./gradlew :gui-runner:run --args='gui-runner/sample-teamcode HungOpMode 1000'
```

### Real renderer evidence

Screenshots were inspected for scale, mode inventory and complete field framing. Both the generic robot and real REV DUO run on Generic field-only, imported field-only and imported field-with-pieces scenes. The REV DUO demo physically drives and turns with its existing motor/wheel bindings and flexible paddle model.

| Scene | Measured run mean FPS | Loose pieces |
| --- | --- | --- |
| Generic robot / generic field-only | above 30 (initial run above 3,000) | 0 |
| Generic robot / imported field-only | approximately 119 | 0 |
| Generic robot / imported game mode | approximately 119 | 56 |
| REV DUO / generic field-only | approximately 116 | 0 |
| REV DUO / imported field-only | approximately 113 | 0 |
| REV DUO / imported game mode | approximately 114 | 56 |
| REV DUO / legacy torus retention | approximately 118 | 1 torus |

Optimized field runs recorded zero frames exceeding configured fixed-step capacity after startup. The REV DUO uses 480 Hz physics; generic chassis runs use 120 Hz. Example optimized REV DUO populated run: 185 bodies, approximately 226 MiB Java heap used at completion, maximum runtime frame about 25 ms. Heap readings include unreclaimed preparation allocations and are not total native/GPU memory.

The legacy renderer finishes with the torus **SEATED**, one contact-triggered acquisition, physical carrying/turning, and stopped retention. Its native body remains present.

The desktop selector was launched successfully and observed as a running Java application. This host's computer-use API could not bind Java's accessibility surface, so import-button click automation was unavailable. Preparation, selection overrides, both modes and preview rendering were exercised through their equivalent command-line paths.

Private evidence is retained under `.local/fields/biobuzz/evidence/`: preparation/audit and scale checks, Java/Python logs, native field and torus logs, Dashboard/watchdog logs, six renderer scenes, source/practice-layout previews and the legacy retention screenshot. Source CAD/assets and screenshots are not redistributed in Git.

## Remaining physical limits

- This package is a source-scale practice assembly with a separate floor practice layout. Source staged/below-floor poses are available in visual-only previews. A verified official match setup remains future work.
- HIVE spring peak torque (1.5 Nm), damping (0.08), ball friction/restitution and CAD masses are provisional. Tipping thresholds and real intake ball retention have not been calibrated to hardware.
- Spherical balls omit shell holes/compliance. FLOWER annuli omit molded tabs; HIVE sheet proxies omit small fastener holes/fillets and rib detail. Collision clearance is validated for the tested trajectories, not every possible contact pose.
- Original full-detail graphics remain available at greater rendering/memory cost. Metric clustering bounds retained vertex movement, not all topology changes.
- Automatic scoring, officiating and AprilTag vision are outside this import implementation.

## Delivery

Code, plan, guide, probes and regression tests are ready for the shipping PR. Local REV DUO launchers include `select-field.command`, `drive-biobuzz-field.command`, `drive-biobuzz-game.command`, and `verify-biobuzz.command`; the supplied field is already prepared.
