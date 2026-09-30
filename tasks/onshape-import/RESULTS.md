# Native Onshape import validation

On September 30, 2026, a user supplied a native Onshape URDF/STL package. It contains 614 links, 613 fixed joints, 750 visuals, 207 binary STL files, and seven geometry-free assembly frames without inertial data. There are no collision shapes or transmissions. CAD mass totals 2.697 kg and needs confirmation against the physical robot. The user confirmed the standard REV DUO starter drivetrain/intake.

## Fixes and evidence

- The parser now retains separate visual geometry and inline/named material colors. Geometry-free assembly frames may omit inertial data and contribute zero mass; visible links still require inertia, and physical subtrees still require positive aggregate inertia/mass.
- Package mesh lookup recognizes Onshape's sibling `urdf/` and `meshes/` folders. Shared STL references reuse mesh buffers. Binary loading uses primitive arrays instead of one boxed Float per coordinate, and triangle normals support lighting. Visual STL capacity is 500,000 triangles; collision decomposition keeps its 200,000 limit.
- `previewRobot` renders CAD independently of physics/TeamCode. The actual model loaded and rendered with approximately 0.420 × 0.462 × 0.237 m bounds. A PNG was saved and visually inspected. Source CAD and private import reports remain outside Git.
- Screenshot export initially hung in macOS `glfwPollEvents` after PNG encoding initialized AWT. Setting `java.awt.headless=true` for the preview task prevents the competing AWT windowing path; a repeated real export completed with exit code 0 in four seconds. Screenshot mode also runs when its window loses focus.
- Physics explicitly rejects a URDF with no collision shapes and directs the user to the preview command.
- `OnshapeImportTest` covers massless frames, nested package lookup, named colors, visual/collision separation, coordinate conversion/normals, shared mesh buffers, mass override, and missing-inertia rejection. The native physics test verifies that visual-only CAD fails with preview guidance.
- `./gradlew test --offline --no-daemon` passed all modules; `git diff --check` was clean.

## Remaining setup for this robot

The original export remains unchanged. Powered simulation needs actual wheel/intake joint definitions, collision proxies, FTC hardware bindings, and measured weight. The standard REV differential drivetrain also needs a simulator drive adapter because the current drive model assumes four Mecanum motors. The delivered preview demonstrates geometry compatibility; it does not demonstrate this robot driving or operating its intake in physics.
