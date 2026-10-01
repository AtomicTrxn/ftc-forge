# Field CAD import and modes

The field is selected independently of the robot hardware/model. With no `field` configuration, FTC Forge opens the generic 3.6576 m practice field with **no game pieces**.

## Desktop selection

From the repository root:

```sh
./gradlew :gui-runner:selectField --args='gui-runner/sample-teamcode FieldDriveOpMode'
```

Choose **Generic field** or **Imported CAD field**, then **Field only** or **Field + game pieces**. The import button accepts the supplied BIOBUZZ ZIP, its original URDF, or an already prepared `field.json`. ZIP/URDF preparation runs locally with Python 3.9+ and requires no ROS installation. Launching the selector requires desktop Java/AWT; the 3D renderer runs in a separate JVM on macOS.

**Field only** keeps the imported floor, perimeter, FLOWERS and moving HIVES, and removes loose scoring pieces. **Field + game pieces** includes 40 POLLEN, 8 red NECTAR and 8 blue NECTAR on the prepared practice layout. Generic game mode provides a six-ball practice set; a torus is available as a separate legacy preset.

Use **Preview robot + field** to inspect the configured physical scene without starting TeamCode. Drag to orbit, scroll to zoom. `R` resets pieces and HIVE positions without restarting the robot or OpMode. Releasing a held torus during reset restores its initial pose and clears retention state.

Choose **Review collisions** to overlay the actual native field, robot and piece collision shapes. C toggles the overlay; V toggles CAD visibility. Cyan shapes are dynamic and orange shapes are static. The overlay preserves physics and never substitutes visual meshes for collision geometry. See [collision review](COLLISION_REVIEW.md) for robot coverage reports and commands.

## Prepare through the command line

```sh
python3 tools/prepare_biobuzz.py '/absolute/path/am-5850 BIOBUZZ.zip' .local/fields/biobuzz/prepared
```

The destination must be new and outside the source package. The tool preserves the original files under `source/`, writes bounded visual meshes under `meshes/`, and produces `field.json` plus `AUDIT.json`. The profile recognizes this specific BIOBUZZ assembly. An unrelated CAD export needs another field preparation profile.

Configure a team project with:

```json
{
  "field": {
    "source": "imported",
    "package": "/absolute/prepared/field.json",
    "mode": "game-pieces",
    "piece_set": "biobuzz",
    "friction": 0.6,
    "restitution": 0.15
  },
  "robot_start_xyz_m": [-1.2, -1.2, 0.003],
  "robot_start_yaw_rad": 0
}
```

Merge these fields with the existing `sim.config`; keep its robot, source and hardware paths. Paths are relative to the team project or absolute. Robot starting coordinates use URDF X/Y/Z, with Z above the floor; yaw is radians about +Z. Choose the height appropriate to the robot's CAD frame (`0.003` m is the prepared REV DUO profile; the generic chassis uses `0.1` m). An imported field defaults to a clear corner at X/Y=(-1.2,-1.2), using the robot's configured start height. Starting footprints and field/piece penetrations are checked before TeamCode starts.

CLI overrides leave the project configuration unchanged:

```sh
./gradlew :gui-runner:runSimulatorApp --args='gui-runner/sample-teamcode FieldDriveOpMode --field /absolute/prepared/field.json --mode game-pieces'
./gradlew :gui-runner:runSimulatorApp --args='gui-runner/sample-teamcode FieldDriveOpMode --field /absolute/prepared/field.json --mode field-only'
./gradlew :gui-runner:runSimulatorApp --args='gui-runner/sample-teamcode FieldDriveOpMode --field generic --mode field-only'
```

Add `--preview` to skip starting TeamCode. An optional PNG path captures the final scene and closes. Explicitly selected invalid packages fail with an import error.

Legacy torus demo:

```sh
./gradlew :gui-runner:runSimulatorApp --args='gui-runner/sample-teamcode IntakeDemoOpMode --field generic --mode game-pieces --piece-set torus'
```

Prepared REV DUO projects explicitly select this torus practice preset. Switching to BIOBUZZ automatically chooses actual balls and makes the torus-specific retention controller inactive. Flexible paddle contacts remain physical; region reporting does not imply validated ball retention.

## Scale and preparation corrections

Both CAD models use **meters at scale 1**. URDF `(x,y,z)` maps to jME `(x,z,-y)`. The source tile top is Z=0. Camera zoom changes the view only. Field and robot are never independently resized to fit a scene.

The supplied export measures **3.590237 × 3.590236 m between inside wall faces**. Its tile footprint is approximately 3.58574 m. The generic field remains 3.6576 m. The source balls measure approximately 71 mm and 92 mm, while the prepared REV DUO wheels remain 90 mm in diameter and its track remains 380.997 mm. This establishes matching metric units; it does not certify the export against nominal competition dimensions. The [FIRST BIOBUZZ manual, sections 9.2 and 9.8](https://ftc-resources.firstinspires.org/ftc/archive/2027/game/cm-html/BIOBUZZ%20Competition%20Manual%20-%20TU02.htm) gives nominal field and scoring-element dimensions.

Preparation corrects assembly relationships and ball origins while preserving source poses/dimensions:

- Four CELL groups attached directly to the CAD root are assigned to the matching colored HIVE pivot. CAD mass, inertia and center of mass are assembled for each complete HIVE.
- Loose balls are detached after resolving their complete source transforms. Source staged/below-floor positions are retained for visual inspection; a separate, nonoverlapping practice layout is used for simulation.

Ball collision radii come from mesh dimensions, not distance from the STL origin. Practice ball visuals are centered on their sphere bodies; source-layout previews preserve the original offset mesh origins and assembly transforms.

Source/practice audit previews:

```sh
./gradlew :gui-runner:previewField --args='/absolute/prepared/field.json /absolute/robotProject --source-layout'
./gradlew :gui-runner:previewField --args='/absolute/prepared/field.json /absolute/robotProject'
```

These previews create no physics bodies and run no TeamCode. `--full-detail` shows original field meshes and hardware with larger graphics/memory costs. Standard renderer visuals use a 0.5 mm grid for dense meshes; retained vertices move at most approximately 0.433 mm. This is a vertex-displacement bound, not a topology guarantee. Robot collision meshes and inertia retain their original data. `field.full_detail=true` retains original robot visuals and additional field hardware.

## Physical approximations

- One floor collider follows the imported wall interior and tile top. Generic walls/floor are replaced in imported mode.
- Perimeter and extrusions use metric boxes. FLOWER rings use 32 convex annulus segments; molded tabs are omitted. Backstops preserve their L-shaped silhouette with a 3 mm raster (at most 4.25 mm planar coverage error).
- HIVE cell skins use thin convex sheet faces; small fastener holes and edge fillets are omitted. Rib detail remains visual. Passive hinges retain the source stops and use a finite two-position spring. Each HIVE's `spring_peak_torque_nm` (default 1.5) and `damping` (default 0.08) are editable in `field.json` and remain uncalibrated.
- Balls use solid sphere collision proxies and CAD-estimated masses (about 20.98 g POLLEN / 40.59 g NECTAR). Shell holes, deformation, real friction and impact response require calibration. They remain independent dynamic bodies; legacy proximity capture does not remove them from physics.
- This is a practice field/layout. Automatic scoring, officiating, AprilTag vision and a verified match starting arrangement are future work.

## Verification

```sh
./gradlew test
python3 -m unittest discover -s tools -p 'test_*.py'
./gradlew :gui-runner:verifyFieldPhysics --args='/absolute/prepared/field.json'
```

The native probe checks piece inventory, floor support, perimeter contact, finite dynamics, HIVE stops/motion, ball contacts with all CELL floors, FLOWER clearance and repeated resets. See [implementation evidence](../tasks/field-import/RESULTS.md).
