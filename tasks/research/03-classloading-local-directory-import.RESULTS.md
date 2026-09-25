# R3 Results: Classloading Strategy & Local-Directory Import

**Status:** Complete (amended after Opus review pass — see changelog)
**Task spec:** [03-classloading-local-directory-import.md](03-classloading-local-directory-import.md)

**Changelog:** added third-party dependency resolution (previously entirely missing — team code that imports Road Runner/Pedro/FTCDashboard would fail to compile under the original design); added an explicit simulated-time decision; added a runaway-OpMode/classloader-isolation design; upgraded `HardwareMap` to type-aware lookup; hardened annotation-based discovery against class-init failures; added a worked robot-configuration XML example and flagged the motor-ratio gap; noted `WatchService`'s recursive-watch and macOS-polling limitations; recorded the benchmark as explicitly deferred rather than silently skipped; flagged Kotlin TeamCode as out of scope for `javax.tools`-based compilation.

## Summary

Per R2, this is a native JVM classloading problem, not a transpilation problem — the standard, well-established pattern is `javax.tools.JavaCompiler` + a fresh `URLClassLoader` per reload cycle, watched by `java.nio.file.WatchService`. The bigger finding in this task is on OpMode discovery and hardware naming: the real FTC SDK already solves "how does the app find my OpModes" with a decentralized `@TeleOp`/`@Autonomous` annotation scan, and it already solves "what do hardware names mean" with a real, exportable robot-configuration XML file. The simulator should reuse both rather than inventing its own conventions — this makes the "point at your folder" experience simpler than the plan originally assumed (no entry-point naming needed) and makes the HardwareMap naming convention not a new design at all, but a parser for something that already exists.

## Classloading Mechanism Chosen

**`javax.tools.JavaCompiler` (via `ToolProvider.getSystemJavaCompiler()`) + `URLClassLoader`, watched by `java.nio.file.WatchService`.** This is the standard, well-documented Java idiom for runtime compilation and hot-reload, not a novel design:

1. A `WatchService` watches the team's source directory (resolved per the `sim.config` below) for `.java` file changes.
2. On a change, `JavaCompiler` recompiles the affected source files to a scratch output directory.
3. A **new** `URLClassLoader` instance is created pointed at that output directory, and the old one is discarded.

Point 3 is deliberate, not a shortcut: the JVM does not allow redefining a class's structure in an already-loaded classloader without a HotSwap/instrumentation agent (e.g. JRebel-style tooling), which is real added complexity and fragility not justified for v1. Creating a fresh classloader per reload is simpler, well-understood, and has no meaningful downside here, since each OpMode run already starts from a clean `init()` — there's no long-lived static state worth preserving across a reload.

**Packaging implication carried forward to Phase 1:** `javax.tools.JavaCompiler` requires the `jdk.compiler` module, which means the simulator needs to ship on (or require the user to have) a **full JDK**, not a bare JRE — or bundle a custom `jlink` runtime image that includes `jdk.compiler`. This should be an explicit Phase 1 packaging decision, not an assumption. (R2 has since pinned this to JDK 21 LTS.)

**Compile target must match the team's real Android target, not the simulator's own JDK.** Compile team sources with `--release` set to the team's actual Gradle `sourceCompatibility` (FTC's project templates have historically targeted Java 8) — otherwise the simulator will happily accept desktop-only APIs the Control Hub's actual Android runtime doesn't have, defeating the entire point of catching bugs before deploying to hardware. Confirm the exact `sourceCompatibility` value against the pinned v12.0 templates rather than assuming Java 8 is still current.

**Kotlin TeamCode is out of scope for v1.** `javax.tools.JavaCompiler` compiles Java only — it cannot compile Kotlin sources. This is a real gap only for the (less common) case of a team writing their own OpModes directly in Kotlin; it does not affect Road Runner itself, which teams consume as a precompiled Maven dependency (already-compiled bytecode on the classpath, not recompiled from source by this project's classloader).

## Third-Party Dependency Resolution

**This was missing entirely from the original design, and is a real Phase 1 blocker, not a nice-to-have.** Any team using Road Runner, Pedro Pathing, or FTCDashboard has source that imports those libraries' packages — compiling only the team's own `sourceRoot` with no additional classpath entries will fail with unresolved-symbol errors on the very first real project tested, defeating "zero-refactor" immediately.

Design addition:
- Parse the team's actual dependency declarations — the `implementation '<group>:<artifact>:<version>'` lines in `build.dependencies.gradle` / `TeamCode/build.gradle` — with a targeted regex or line-scan. This is deliberately **not** a Gradle evaluation (embedding a Gradle build engine just to read dependency coordinates is far more machinery than the problem needs).
- Resolve each declared coordinate against the relevant Maven repository (Maven Central, `maven.brott.dev` for Road Runner, Pedro's own repository) and add the resulting JAR (extracting `classes.jar` if the artifact is packaged as an **AAR** — likely, since these libraries are published for Android consumption, though this should be confirmed per-library rather than assumed uniformly) to the compile classpath.
- **Drop, rather than resolve, any `org.firstinspires.ftc:*` coordinate** (replaced entirely by this project's mock SDK) and any `com.acmerobotics.dashboard:*` coordinate (replaced by the FTCDashboard shim tracked in R1).
- Add an `extraClasspath` array to `sim.config` (below) as a manual escape hatch for anything the automatic resolution misses.

## Simulated Time (open decision, not previously addressed)

**The original design never decided how the simulator controls time, and this is a real architectural gap, not a minor detail.** R1 requires `ElapsedTime` to run on the simulator's own clock once pause/step/fast-forward exist. But team code and libraries also call the JDK's own wall-clock primitives directly — `Thread.sleep`, `System.currentTimeMillis()`, and (plausibly, not independently confirmed here) `System.nanoTime()` inside Road Runner's own trajectory-following loop. None of those calls can be intercepted by anything short of bytecode rewriting; they don't go through any interface this project controls.

This needs an explicit decision, not a silent assumption:
- **Option A (recommended default for v1): run in real time only.** `ElapsedTime` reads the simulator's own timer for consistency with what teams see in telemetry, but no pause/fast-forward/deterministic-replay capability is promised in v1. This is the minimal-scope answer and doesn't block anything in Phases 1–3.
- **Option B (larger scope, defer to a later phase if wanted): rewrite `System.nanoTime`/`currentTimeMillis`/`Thread.sleep` calls via ASM bytecode rewriting**, in both team and library classes, to route through the simulator's clock. This is what real pause/fast-forward/deterministic-replay (needed for R5's calibration determinism) would require — but it's a materially bigger undertaking than anything else in this task, and shouldn't be adopted implicitly just because the classloader happens to sit in the right place to do it.

**Recommend Option A for v1**, and treat Option B as an explicit, separately-scoped future task if deterministic replay becomes a hard requirement — R5's calibration pass (which replays real recorded logs) works fine under Option A as long as replay compares against the *recorded* `t_ms`/`loop_time_ms` per row rather than requiring the simulator's live clock to match exactly, which is already how R5 is designed.

## Runaway OpMode Handling (open gap, not previously addressed)

**A real risk with no answer in the original design:** team code with a bug — a `while(true)` loop missing `opModeIsActive()`, a deadlock — pins its thread forever. This matters more than it would in ordinary Java tooling because **`Thread.stop()` is not a fallback**: per R2's findings, it throws `UnsupportedOperationException` unconditionally on JDK 20+, and the fresh-classloader-per-reload design means a stuck thread also prevents the old classloader from ever being released (a real, compounding leak across repeated reload attempts during a debugging session).

Recommended v1 approach (lighter-weight than a full child-JVM, which is a larger architectural commitment worth considering only if this proves insufficient):
- Run each OpMode invocation on a dedicated, named daemon thread.
- A watchdog monitors for a stuck thread (no progress against an expected loop cadence) and, on detection, **marks that session dead and refuses further interaction with it** — it does not attempt to force-kill the thread (which is exactly what's no longer possible), it isolates it and requires a fresh classloader + fresh thread for the next run.
- Document this as a known v1 limitation: a sufficiently stuck OpMode will leak one thread until the JVM process itself restarts. A child-JVM-per-OpMode-session model would fully solve this (and additionally isolate `System.exit` calls and native crashes) at the cost of needing IPC between the child process and the renderer — worth a dedicated follow-up spike if the daemon-thread-plus-watchdog approach proves too leaky in practice.

## Classloader Isolation

**Not previously specified.** Recommended split:
- **Parent loader:** exposes only the mock SDK's own classes plus the JDK — simulator-internal classes (the physics engine, the renderer, this project's own Kotlin stdlib if any) must be invisible to team code, both to prevent accidental coupling and to avoid classpath collisions if a team's own dependencies happen to overlap with the simulator's internals.
- **Child loader (recreated each reload):** loads team code plus the resolved third-party dependency JARs together, so their static state (Pedro's config constants, an FTCDashboard `@Config` class's static fields) resets cleanly on every reload rather than persisting stale values from a previous run.
- Explicitly `close()` the outgoing `URLClassLoader` once its thread has ended (or been marked dead per the watchdog above), rather than just dropping the reference and relying on GC.

## Local-Directory Import Design

**Simpler than the plan's original framing.** The real FTC Robot Controller app doesn't require teams to name an entry point at all — it finds runnable OpModes by scanning compiled classes for the `@TeleOp` and `@Autonomous` annotations, a decentralized discovery mechanism with no central registry. The simulator should copy this exactly, rather than inventing a naming scheme:

- `sim.config` at the team's project root (JSON, kept intentionally minimal):
  ```json
  {
    "sourceRoot": "TeamCode/src/main/java",
    "robotConfig": "robot_config.xml",
    "extraClasspath": []
  }
  ```
  - `sourceRoot` — defaults to the standard FTC/Android Studio layout (`TeamCode/src/main/java`) if omitted; only needs to be set for non-standard project layouts.
  - `robotConfig` — path to the robot's exported hardware configuration file (see below). Optional at this stage if Phase 1 is still using hand-authored presets; required once Phase 5's importer is in play.
  - `extraClasspath` — manual escape hatch for any dependency JAR the automatic Gradle-coordinate resolution (above) doesn't catch. Empty by default; most teams shouldn't need it.
- On file-watch trigger: recompile everything under `sourceRoot`, reload via a fresh classloader, then **reflectively scan all loaded classes for `@TeleOp`/`@Autonomous`** to rebuild the list of runnable OpModes — mirroring the real app's own UX. No entry-class field needed in `sim.config` at all.
- **Discovery must be resilient to a single bad class, not just annotation-driven.** Checking `isAnnotationPresent()` isn't enough on its own — a class can fail during static initialization or linkage (e.g. it references something legitimately missing from the mock SDK) before annotation inspection even happens. Load candidates with `Class.forName(name, false, loader)` and catch `LinkageError` (and its subtypes) per-class, so one broken OpMode doesn't abort discovery for the whole project. Also: honor `@Disabled` (skip it), and default an OpMode's display name to its simple class name when the annotation doesn't specify one — both are real behaviors of the actual Robot Controller app's discovery mechanism this project is deliberately mirroring.
- **`WatchService` has two real platform limitations worth knowing before relying on it as-is:** it does not watch subdirectories recursively (each subdirectory must be registered individually, or the tree walked and re-registered on new-directory-created events), and historically the JDK's default macOS implementation has been polling-based rather than using native OS file-change notification, which can mean a multi-second detection delay depending on JDK vintage — not appropriate for a "save your file and see it reload" UX. If this turns out to matter in practice, a third-party directory-watching library with native macOS support is worth evaluating as a drop-in replacement, plus a short debounce window (IDEs often save via multiple rapid file events/atomic renames for one logical save).
- **Benchmark:** not measured in this research pass (spec instruction 2 asked for one). Recording this as an explicit deferral rather than a silent omission: target under 2 seconds for an incremental recompile-and-reload of a typical (~50-file) TeamCode project; measure this for real during Phase 1 rather than assuming it.

## Finalized HardwareMap Naming Convention

**Do not invent a new naming/config scheme — parse the real FTC robot-configuration XML format instead.**

- FTC's real Robot Controller app already stores hardware device names and port mappings in an XML file (root element `<Robot type="FirstInspires-FTC">...`), which teams routinely export/copy between robot controllers today.
- The device-name strings a team's OpMode already calls (e.g. `hardwareMap.get(DcMotor.class, "left_front_drive")`) originate from *this file*, not from anything the code itself defines. This means the simulator's mock `HardwareMap` should be seeded by literally parsing a team's real exported configuration XML, rather than asking teams to redefine their hardware names in a new JSON/URDF schema just for the simulator.
- **This is a stronger zero-refactor result than originally scoped**: not just the OpMode code, but the hardware configuration artifact itself can be the real one, unmodified.
- `HardwareMap.get(Class, String)` resolution needs to be **type-aware, not just name-keyed** — the original `Map<String, DeviceEntry>` sketch was too simple to match real usage:
  - `get(DcMotorEx.class, name)` and `get(DcMotor.class, name)` for the same device name must resolve to the **same underlying object** (team code routinely casts between the two).
  - Whether a name resolves to a `Servo` or a `CRServo` depends on which XML tag declared it — the lookup must check the requested class is assignable from the stored device's actual type, not just match the name string.
  - Each Control/Expansion Hub is itself registered as a `VoltageSensor` (per R1's stub table) — the naming/lookup design must account for this hub-as-sensor registration, not just per-port peripheral devices.
  - `getAll(Class)` and `DeviceMapping`-style iteration (e.g. `hardwareMap.voltageSensor.iterator().next()`, a real pattern from the Road Runner quickstart) need to work, not just single-name lookup.
  - On a miss, throw `IllegalArgumentException` matching the real SDK's message shape, and have `tryGet` return `null` rather than throwing — team `try`/`catch` code around hardware lookups depends on this distinction behaving the same way it does on real hardware.

### Worked Example: Real Robot-Configuration XML

To satisfy this task's own "worked example" requirement (previously missing), a representative fragment of what a team's real, exported configuration XML looks like (exact tag names below are reconstructed from general knowledge of the format, not independently re-verified against a real sample export in this pass — confirm against an actual exported file before the parser is implemented):

```xml
<Robot type="FirstInspires-FTC">
  <LynxUsbDevice name="Control Hub" serialNumber="(embedded)">
    <LynxModule name="Control Hub" port="173">
      <goBILDA5202SeriesMotor name="left_front_drive" port="0"/>
      <goBILDA5202SeriesMotor name="right_front_drive" port="1"/>
      <goBILDA5202SeriesMotor name="left_back_drive" port="2"/>
      <goBILDA5202SeriesMotor name="right_back_drive" port="3"/>
      <Servo name="claw" port="0"/>
      <RevIMU name="imu" port="0" bus="0"/>
    </LynxModule>
  </LynxUsbDevice>
</Robot>
```

**Important gap this surfaces:** the motor-type XML tag most likely does not distinguish between different gear ratios of the same physical motor family — i.e. it's plausible a `goBILDA5202SeriesMotor` tag alone can't tell a 13.7:1 from a 19.2:1 unit apart. If so, **R4's per-ratio motor constants (`τ_stall`, `ω_no_load`, encoder counts/rev) cannot be sourced from this XML alone** — the robot-configuration file establishes *what device exists and where*, but not *which specific gear ratio it is*. This needs a home: most likely a small supplementary field in Phase 1's preset format (and, later, R6's URDF `<ftc:motor sku="...">`-style extension) that pairs a device name with its specific SKU/ratio, layered on top of (not replacing) this XML. Flagging this explicitly rather than letting it surface as a surprise during Phase 1/5 implementation.

## Risks / Unknowns Remaining

- **The XML schema is not officially/fully documented.** Public sources confirm the root element and general shape but note "little documentation on the allowed tags" — implementing this parser will require reverse-engineering the format from real sample exports (several are easy to obtain from FTC teams/forums) rather than working from an official spec. This is a real, non-trivial implementation task, not a trivial format-parsing exercise — scope it accordingly in Phase 1/5 estimates. The worked example above is reconstructed from general knowledge of the format, not verified against a real export — treat its exact tag names as illustrative until checked against one.
- Because the XML format is somewhat undocumented, there's a fallback risk: if the parser can't handle some hardware type or config variant a real team exports, Phase 1 will need a clear error path (not a silent failure) telling the team exactly which config line failed to parse.
- **The real config XML normally lives only on the Control Hub** (typically retrieved via `adb pull`), not in a team's git repo by default — though teams can also commit a copy under `TeamCode/src/main/res/xml/` for it to travel with the source (not independently confirmed this still works unchanged in v12.0). Either way, `sim.config`'s `robotConfig` path assumes the team has gotten a copy of this file into their project folder; the onboarding flow needs to say this explicitly rather than assume it's already there.
- The motor-ratio gap noted in the worked example above (XML likely can't distinguish gear ratios) needs an owner — flagged for Phase 1/R6, not resolved in this task.

## Implication for R6

R6 (CAD & Robot Configuration Pipeline) evaluates URDF vs. a custom JSON schema for **importing custom robot geometry** (mass, joints, mechanisms) — that's a different, larger problem than hardware *naming*, and still stands. But R6 should treat this task's finding as a hard interoperability constraint: whatever format R6 lands on for geometry/mass, the **device name strings** it produces must still reconcile with the real XML config's naming — either by having R6's tooling directly emit/consume the same XML for the naming layer, or by keeping the two concerns (geometry vs. naming) cleanly separated so a team's real config XML and R6's richer geometry file can both be loaded together without name collisions.
