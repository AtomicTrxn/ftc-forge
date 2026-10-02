# Calibration preservation regression plan

Status: implemented and validated; see RESULTS.md for reproduced bugs and test evidence.

1. Exercise the production guide/controller flow: save measurements and motor bindings, export a reviewed profile, import it into a separate library, then reuse it with changed CAD.
2. Cover differential and Mecanum dimensions, manual measurement provenance, existing battery/motor/drive calibration, signed transmission ratios, unchanged saved revisions and renewed review requirements.
3. Add preparation unit tests for automatic renames, explicit retained-settings choices and changed CAD transmission choices, including escaped part names. Verify prepared transmissions and calibration assets after a second portable round trip.
4. Reproduce and correct any loss or incorrect attribution exposed by these tests. Run focused tests and the affected preparation/GUI suites, document results and ship.
