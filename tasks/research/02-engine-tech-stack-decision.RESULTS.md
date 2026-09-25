# R2 Results: Engine & Tech Stack Decision

**Status:** Complete (amended after Opus review pass — see changelog)
**Task spec:** [02-engine-tech-stack-decision.md](02-engine-tech-stack-decision.md)

**Changelog:** added the rendering/input stack decision the spec's own objective required but the original pass omitted (jMonkeyEngine + Minie); added Libbulletjme native-library packaging note, JDK version pin, and a mecanum-drivetrain modeling constraint for Phase 4/R6.

## Summary

No hard "must run in-browser with zero install" requirement has been stated anywhere in this project's history, so the plan's recommended default applies: **Desktop JVM (Option A)**. R1's audit found nothing that outright disqualifies Web/WASM, but nothing that makes it free either — and Option A remains the strictly lower-risk path to the "zero-refactor" promise, since it runs real bytecode instead of transpiling it. Separately, this task found that the plan's own suggested physics libraries for Option A — **JBullet** and **"Rapier-Java"** — are not actually viable as named: JBullet has been unmaintained since 2013, and "Rapier-Java" does not exist as a mature, off-the-shelf dependency (only small, unofficial JNI-wrapper side-projects). The concrete, actively-maintained replacement is **Libbulletjme**, a real JVM binding for Bullet Physics published on Maven Central. No 50Hz micro-prototype was built — the product question resolved the engine choice outright, making the prototype comparison moot per the task's own instructions.

## Decision

**Desktop JVM (Option A).** Web/WASM (Option B) is deferred to a v2 stretch goal.

## Reasoning

1. **The zero-install product question was never raised as a hard requirement** anywhere in the plan, prior research, or conversation history. Absent that signal, the recommended default applies directly: a one-time desktop download is acceptable for the target users (FTC teams — students and mentors already comfortable installing Android Studio, which is a far heavier install than a single JAR or desktop app).
2. **R1's findings reinforce, rather than override, the default:**
   - Road Runner is Kotlin — native JVM classloading handles Kotlin bytecode exactly like Java (it's already compiled bytecode by the time the classloader sees it); a transpiled Web target would need to additionally verify CheerpJ/TeaVM compatibility with the Kotlin stdlib, an unverified variable specific to Option B.
   - Pedro Pathing (once resolved) is pure Java in its core module — no advantage either way.
   - FTCDashboard's real risk (Android app-lifecycle coupling) is identical under both options, since neither runs inside a real Android robot-controller app. This means Option B gets zero benefit from "the browser is closer to Android" reasoning — a custom `FtcDashboard`-equivalent shim over the portable `DashboardCore` is required either way (tracked as its own follow-up, per R1).
3. **Nothing in R1 makes a compelling case *for* Option B** strong enough to override the default and pay for the transpilation prototype. Building the 50 Hz comparison would only be justified if the zero-install requirement were confirmed hard — it isn't, so skipping the prototype is the correct, cost-saving call per the task's own instructions ("build the prototype only if the default doesn't resolve it").

## Physics Engine Binding

**`com.github.stephengold:Libbulletjme`** (JVM bindings for Bullet Physics + V-HACD, published on Maven Central, actively maintained through 2025, native desktop support for Windows/Linux/macOS on x86_64 and ARM).

This corrects the original plan text, which named **JBullet** and **"Rapier-Java"** as the Option A physics choices:
- **JBullet** — confirmed unmaintained since its last release in November 2013. Not viable for new work.
- **"Rapier-Java"** — does not exist as a first-party, off-the-shelf dependency. Dimforge (Rapier's maintainer) does not publish an official Java binding; what exists are small, unofficial JNI/FFI wrapper projects built for other purposes (e.g. a Minecraft server physics mod), not a general-purpose, documented library a team could just add to Gradle and expect to maintain.
- Libbulletjme is the closest real equivalent: mature, versioned, Maven-Central-published, with prebuilt native binaries for exactly the desktop targets this project needs.

**Phase 4 should plan around Libbulletjme, not the plan's originally-named libraries.**

**Native library packaging (verify before Phase 1 packaging decisions):** the plain `com.github.stephengold:Libbulletjme` Maven artifact is understood to hold Java classes only — native binaries ship as separate per-platform artifacts (or GitHub release downloads) and are loaded explicitly (a `NativeLibraryLoader`-style call), not pulled in automatically by a single dependency line. Distribution needs to bundle native libraries for Windows x64, macOS (arm64 and x64), and Linux x64 explicitly, and pick the single- or double-precision build deliberately. This wasn't independently verified against Libbulletjme's actual release artifacts in this pass — confirm the exact artifact/loader mechanism before Phase 1 locks its packaging approach.

## Rendering & Input Stack

**Not decided in the original pass, despite this task's own objective stating the engine decision "gates … the rendering stack."** Correcting that gap:

**Recommended: jMonkeyEngine + Minie**, not a hand-rolled renderer over raw Libbulletjme.

- **Minie** is a physics-integration library for jMonkeyEngine, maintained by the same author as Libbulletjme (Stephen Gold) — Minie's own native code lives in the Libbulletjme repository itself, and Minie exists specifically to integrate Libbulletjme into a full 3D engine. Picking Minie means physics-to-render transform syncing, collision-shape visualization, and scene-graph management come for free from a library built by the same maintainer against the same physics binding, rather than being hand-synced by this project.
- jMonkeyEngine's LWJGL3 backend also provides gamepad/joystick input natively, which resolves the plan's separately-named "Jamepad" dependency question — a second input library is likely unnecessary.
- **Fallback options, not chosen but worth recording:** `dyn4j` (pure-Java 2D physics) could cover Phase 2/3's wall-collision needs well before Phase 4's full 3D engine is ready, since most FTC game-piece interaction during those phases is close to planar. `jolt-jni` (a JNI binding for Jolt Physics, by the same Libbulletjme author) is a newer alternative worth a glance if Minie/Libbulletjme hits a real limitation — maturity not independently assessed here.

## Prototype Results

Not built. The zero-install product question resolved the engine decision outright (see Reasoning), and the task's own instructions say to build the prototype "only on whichever option(s) remain viable after steps 1-2" when the default doesn't resolve it — here it did, so building a comparison prototype would have been unnecessary spend. If the desktop-download assumption is challenged later, this is the first thing to revisit, and the 50Hz loop-stability prototype should be built at that point, not before.

## Implications for R3

Classloading (R3) is now scoped as a **native JVM classloading problem**, not a transpilation/module-loading one: the Java Compiler API (`javax.tools.JavaCompiler`) plus `URLClassLoader` is the direct, idiomatic path, with no transpiler toolchain (CheerpJ/TeaVM) in the loop at all for v1. This significantly simplifies R3's scope — it doesn't need to design around two different loading mechanisms, only one.

## Target JDK

**Verified, confirmed real and worth pinning explicitly:** target **JDK 21 LTS** (or 25 LTS once broadly available), packaged as a `jpackage` bundle whose underlying `jlink` runtime image includes the `jdk.compiler` module (required by R3's runtime-compilation design).

One concrete, confirmed-real reason this matters beyond packaging: `Thread.stop()` was deprecated for removal since JDK 18 and, confirmed via OpenJDK's own release notes, **re-specified to throw `UnsupportedOperationException` unconditionally starting in JDK 20.** Any design for forcibly terminating a runaway OpMode thread (a real scenario — team code can infinite-loop) cannot rely on `Thread.stop()` on any JDK version this project would plausibly target. See R3's results for the corresponding watchdog/isolation design this forces.

## Mecanum Drivetrain Modeling Constraint (for Phase 4 / R6)

**A plain Bullet wheel shape (or `btRaycastVehicle`) cannot model a Mecanum drivetrain — it has no concept of the 45°-roller sideways-slip behavior that makes Mecanum wheels strafe.** This is a real physics-engine limitation, not a Libbulletjme-specific gap: standard rigid-body vehicle physics assumes wheels only roll forward/back with lateral friction, which is the opposite of what a Mecanum roller does.

**Resolution:** drivetrain propulsion forces should be computed by this project's own kinematics/motor model (Phases 2–3, per R4) and applied directly to the chassis rigid body as a force + torque each tick. Bullet/Libbulletjme's job is limited to collision response — chassis-vs-wall, chassis-vs-game-piece, game-piece-vs-field — not wheel-vs-ground traction. R6's URDF worked example should not expect wheel `continuous` joints to produce strafing motion on their own; see R6's results for the corresponding correction.

## Risks / Unknowns Remaining

- This decision assumes the desktop-download assumption holds. If the user or a future stakeholder states that browser-only access is in fact a hard requirement, this task should be re-run with the 50Hz prototype actually built, since that's the one piece of evidence this task didn't need to produce.
- Libbulletjme's Android ARM support (mentioned in its docs) is irrelevant to this decision now but is worth remembering if a future mobile-companion-app idea ever comes up — not in scope for this project as currently planned.
- No native support was checked for ARM Linux specifically beyond what the search turned up; if the dev/CI environment is an unusual architecture, verify Libbulletjme's prebuilt binaries cover it before committing further.
- Minie's release cadence relative to Libbulletjme's own releases was not independently verified — confirm they're not meaningfully out of sync before committing to Minie over hand-rolled Libbulletjme integration.
