# Calibration export, import and CAD migration — results

Date: 2026-10-02

## Coverage added

- Two parameterized Java controller cases, one differential and one Mecanum: enter weight/wheel measurements through the guide's save path, save with a real native construction proof, export, import into an independent library, migrate a compatible renamed/repositioned CAD part, resolve changed geometry using retained settings, validate/review, then export/import again.
- At each stage, check complete saved runtime data, mass, radius/track/wheelbase, differential motor names/shaft signs, signed transmission ratio, measurement provenance and guide snapshots. Load the prepared battery/motor/drive calibration. Native bodies retain the measured total mass after migration.
- Assert migrated drafts cannot run or be approved using stale validation, unresolved choices block review, and every earlier saved revision remains unchanged.
- Six new Python preparation unit tests cover an actual URDF/STL ZIP bundle, automatic escaped-name migration, explicit changed-geometry retention and a second bundle round trip, both changed-transmission choices, manual mapping of an ambiguous renamed part, and reuse of a removed part's name.
- Verify prepared URDF transmissions and generated calibration JSON, not just profile fields. Synthetic fixtures and temporary independent libraries require no private CAD or team project.

## Bugs reproduced and fixed

1. **Renamed-part provenance:** automatic migration copied motor settings but left their provenance under the old entity name. The new regression initially failed with a missing new-name provenance key. Migration now remaps entity provenance using escaped path segments from a snapshot. Explicit retained-settings resolution also restores that part's provenance from the original migration snapshot, including ambiguous renames.
2. **New CAD binding attribution:** selecting new CAD actuators replaced the values but retained old nested user/measured labels. The regression initially failed because stale actuator provenance remained. Explicit CAD replacement now removes those nested labels and marks the actuator settings as source CAD. Keeping saved bindings leaves the saved mapping/ratio and labels intact.
3. **Reused part name:** renaming a retained part to a removed part's old name could overwrite its ratio label with the removed part's provenance. The new regression reproduced the incorrect label; retained-part provenance now takes precedence.

Regenerated collision geometry is labeled as generated and still requires review. Existing runtime measurements and response calibration are preserved; changing CAD does not claim the physical measurements are newly verified.

## Validation

- `python3 -m unittest discover -s tools -p 'test_*.py'`: **35 tests passed**, including **24 model preparation tests** and six new cases. Evidence: `.local/models/calibration-portability-python.log`.
- `./gradlew :gui-runner:test --tests simrunner.RobotMeasurementsTest`: **7 cases passed**, including both new drivetrain preservation flows.
- `./gradlew :gui-runner:test`: **72 tests passed** in the full affected GUI/controller/native suite; evidence in `.local/models/calibration-portability-java.log` and JUnit XML. The Gradle task now tracks production Python scripts as inputs, so changing migration code invalidates cached Java test results.
- `git diff --check`: passed.

No UI controls or physics solver behavior changed. Native validation in the Java flows is real; Python unit helpers supply a test validation receipt to exercise preparation review rules. This verifies settings preservation and review gates, not agreement with physical robot measurements.
