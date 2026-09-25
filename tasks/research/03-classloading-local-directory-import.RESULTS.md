# R3 Results: Classloading Strategy & Local-Directory Import

**Status:** Complete
**Task spec:** [03-classloading-local-directory-import.md](03-classloading-local-directory-import.md)

## Summary

Per R2, this is a native JVM classloading problem, not a transpilation problem — the standard, well-established pattern is `javax.tools.JavaCompiler` + a fresh `URLClassLoader` per reload cycle, watched by `java.nio.file.WatchService`. The bigger finding in this task is on OpMode discovery and hardware naming: the real FTC SDK already solves "how does the app find my OpModes" with a decentralized `@TeleOp`/`@Autonomous` annotation scan, and it already solves "what do hardware names mean" with a real, exportable robot-configuration XML file. The simulator should reuse both rather than inventing its own conventions — this makes the "point at your folder" experience simpler than the plan originally assumed (no entry-point naming needed) and makes the HardwareMap naming convention not a new design at all, but a parser for something that already exists.

## Classloading Mechanism Chosen

**`javax.tools.JavaCompiler` (via `ToolProvider.getSystemJavaCompiler()`) + `URLClassLoader`, watched by `java.nio.file.WatchService`.** This is the standard, well-documented Java idiom for runtime compilation and hot-reload, not a novel design:

1. A `WatchService` watches the team's source directory (resolved per the `sim.config` below) for `.java` file changes.
2. On a change, `JavaCompiler` recompiles the affected source files to a scratch output directory.
3. A **new** `URLClassLoader` instance is created pointed at that output directory, and the old one is discarded.

Point 3 is deliberate, not a shortcut: the JVM does not allow redefining a class's structure in an already-loaded classloader without a HotSwap/instrumentation agent (e.g. JRebel-style tooling), which is real added complexity and fragility not justified for v1. Creating a fresh classloader per reload is simpler, well-understood, and has no meaningful downside here, since each OpMode run already starts from a clean `init()` — there's no long-lived static state worth preserving across a reload.

**Packaging implication carried forward to Phase 1:** `javax.tools.JavaCompiler` requires the `jdk.compiler` module, which means the simulator needs to ship on (or require the user to have) a **full JDK**, not a bare JRE — or bundle a custom `jlink` runtime image that includes `jdk.compiler`. This should be an explicit Phase 1 packaging decision, not an assumption.

## Local-Directory Import Design

**Simpler than the plan's original framing.** The real FTC Robot Controller app doesn't require teams to name an entry point at all — it finds runnable OpModes by scanning compiled classes for the `@TeleOp` and `@Autonomous` annotations, a decentralized discovery mechanism with no central registry. The simulator should copy this exactly, rather than inventing a naming scheme:

- `sim.config` at the team's project root (JSON, kept intentionally minimal):
  ```json
  {
    "sourceRoot": "TeamCode/src/main/java",
    "robotConfig": "robot_config.xml"
  }
  ```
  - `sourceRoot` — defaults to the standard FTC/Android Studio layout (`TeamCode/src/main/java`) if omitted; only needs to be set for non-standard project layouts.
  - `robotConfig` — path to the robot's exported hardware configuration file (see below). Optional at this stage if Phase 1 is still using hand-authored presets; required once Phase 5's importer is in play.
- On file-watch trigger: recompile everything under `sourceRoot`, reload via a fresh classloader, then **reflectively scan all loaded classes for `@TeleOp`/`@Autonomous`** to rebuild the list of runnable OpModes — mirroring the real app's own UX. No entry-class field needed in `sim.config` at all.

## Finalized HardwareMap Naming Convention

**Do not invent a new naming/config scheme — parse the real FTC robot-configuration XML format instead.**

- FTC's real Robot Controller app already stores hardware device names and port mappings in an XML file (root element `<Robot type="FirstInspires-FTC">...`), which teams routinely export/copy between robot controllers today.
- The device-name strings a team's OpMode already calls (e.g. `hardwareMap.get(DcMotor.class, "left_front_drive")`) originate from *this file*, not from anything the code itself defines. This means the simulator's mock `HardwareMap` should be seeded by literally parsing a team's real exported configuration XML, rather than asking teams to redefine their hardware names in a new JSON/URDF schema just for the simulator.
- **This is a stronger zero-refactor result than originally scoped**: not just the OpMode code, but the hardware configuration artifact itself can be the real one, unmodified.
- `HardwareMap.get(Class, String)` resolution: parse the XML into a `Map<String, DeviceEntry>` (name → device type + port), and back each mock device instance with a `DeviceEntry` at construction time. Lookup is then a direct, exact-string match against whatever the XML (or, in Phase 1's absence of a real file, the hand-authored preset) declares.

## Risks / Unknowns Remaining

- **The XML schema is not officially/fully documented.** Public sources confirm the root element and general shape but note "little documentation on the allowed tags" — implementing this parser will require reverse-engineering the format from real sample exports (several are easy to obtain from FTC teams/forums) rather than working from an official spec. This is a real, non-trivial implementation task, not a trivial format-parsing exercise — scope it accordingly in Phase 1/5 estimates.
- Because the XML format is somewhat undocumented, there's a fallback risk: if the parser can't handle some hardware type or config variant a real team exports, Phase 1 will need a clear error path (not a silent failure) telling the team exactly which config line failed to parse.
- Annotation scanning requires loading every compiled class to inspect its annotations, which is fine for typical FTC team codebases (rarely more than a few dozen OpModes) but should use `Class.isAnnotationPresent()` defensively — a class that fails to initialize (e.g. missing an unrelated dependency) shouldn't crash the whole discovery scan.

## Implication for R6

R6 (CAD & Robot Configuration Pipeline) evaluates URDF vs. a custom JSON schema for **importing custom robot geometry** (mass, joints, mechanisms) — that's a different, larger problem than hardware *naming*, and still stands. But R6 should treat this task's finding as a hard interoperability constraint: whatever format R6 lands on for geometry/mass, the **device name strings** it produces must still reconcile with the real XML config's naming — either by having R6's tooling directly emit/consume the same XML for the naming layer, or by keeping the two concerns (geometry vs. naming) cleanly separated so a team's real config XML and R6's richer geometry file can both be loaded together without name collisions.
