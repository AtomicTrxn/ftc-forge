# Project Plan: Next-Gen FTC Robot Simulator

## 1. Project Vision & Goals

The objective of this project is to build an open-source, high-fidelity FIRST Tech Challenge (FTC) robot simulator that bridges the **Sim-to-Real Gap**. Unlike existing tools, this simulator will combine native FTC SDK execution, configurable hardware non-idealities (bus latency, motor back-EMF, wheel slip), and custom CAD/URDF importing to allow teams to accurately test autonomous routines, motion profiles, and control loops before deploying code to physical hardware.

### Primary Objectives
* **Zero-Refactor Code Execution:** Execute unmodified Java `OpMode` and `LinearOpMode` code directly against a mocked `com.qualcomm.robotcore` package structure.
* **Realistic Hardware Dynamics:** Model real-world physical behavior including I2C bus latency (7–10 ms), battery voltage sag under load, wheel slippage, and non-ideal Mecanum/Swerve kinematics.
* **Custom Mechanism & Robot Import:** Provide a streamlined pipeline (Onshape/URDF/JSON) to allow teams to import custom robot geometries, mass properties, and motor configurations.
* **Low-Friction Setup:** Eliminate complex Android Studio repo refactoring by supplying a lightweight, cross-platform runner (Desktop Java or WebAssembly/WebGL). Teams point the simulator directly at their existing project folder on disk — the runner locates, compiles, and loads `OpMode` source from that live directory, with no export, packaging, or build-config step required.

---

## 2. System Architecture Overview


```

+-------------------------------------------------------------------+
|                        User FTC Code Base                         |
|   (Unmodified Java OpModes: LinearOpMode, Road Runner, Pedro)     |
+-------------------------------------------------------------------+
|
v
+-------------------------------------------------------------------+
|                 Hardware Abstraction Layer (HAL)                  |
|  - Mock DcMotor / Servo / IMU / DistanceSensor                    |
|  - Simulated Hardware Queues (I2C Delay, Bus Jitter)              |
|  - Gamepad Input Bridge (Jamepad / Web Gamepad API)               |
+-------------------------------------------------------------------+
|
+---------------------+---------------------+
|                                           |
v                                           v
+-----------------------+                   +-----------------------+
|  Physics & Kinematics |                   |    Visual Renderer    |
| - Drivetrain Models   |                   | - Field & Robot Mesh  |
| - Battery Voltage Sag |                   | - Real-time Dashboard |
| - Rigid-Body Dynamics |                   | - Sensor Visualizers  |
+-----------------------+                   +-----------------------+

```

---

## 3. Pre-Implementation Phase: Research & Design Roadmap

Steps below are sequenced by **dependency**, not by topic, so the decisions that gate the most downstream work land first, and independent tracks run in parallel rather than in the original 1-2-3-4 topic order. Each step lists the open questions that need an explicit answer — with a recommended default — before it's safe to build on top of it.

**Sequencing at a glance:**
1. Step 1 (SDK Surface Inventory) → blocks Step 2
2. Step 2 (Engine Decision) → blocks Step 3
3. Step 3 (Classloading & Local-Directory Import) → blocks Step 6
4. Step 4 (Non-Ideality Math) → **parallel** with Steps 1–3, no dependencies
5. Step 5 (Telemetry Schema & Logging Wrapper) → **parallel**, starts immediately; only its calibration algorithm depends on Step 4
6. Step 6 (CAD & Robot Config Pipeline) → depends on Step 3; recommend descoping for MVP (see below)

### Step 1: SDK Surface Inventory *(no dependencies — do this first)*
* **Goal:** Catalog exactly what must be stubbed, before anything downstream can be sized or decided.
* **Action Items:**
  * Enumerate the full stub surface from `com.qualcomm.robotcore` (`HardwareMap`, `Gamepad`, `Telemetry`, `DcMotorEx`, `IMU`, etc.), pinned to one specific FTC SDK season/version.
  * Audit **Road Runner**, **Pedro Pathing**, and **FTCDashboard** for Android-only coupling (`Looper`-based threading, `NanoHTTPD`, native OpenCV bindings) that would block a transpiled/web target — this feeds directly into Step 2.
* **IP/Licensing Note:** All `com.qualcomm.robotcore` stub classes are authored clean-room against the publicly documented API surface only — no FTC SDK source is copied or derived — to avoid encumbering the project under REV/Qualcomm's SDK license.
* **Open questions:**
  * *Which SDK version is the pinned target?* → Recommend: current competition season's SDK, re-pinned each offseason.
  * *Is camera-based vision (`VisionPortal`/AprilTag) in scope for v1?* → Recommend **defer**. Native OpenCV/Android camera bindings are their own large mocking problem, distinct from motor/sensor stubbing; shipping motion/kinematics sim without vision still delivers real value and doesn't block Phase 1 on the hardest sub-problem.

### Step 2: Engine & Tech Stack Decision *(depends on Step 1)*
* **Goal:** Choose Desktop JVM vs. Web/WASM, using Step 1's findings — this single decision gates the classloading strategy, physics engine, and rendering stack.
* **Action Items:**
  * Feed Step 1's Android-coupling audit into the Option A/B comparison: any third-party library that breaks under CheerpJ/TeaVM is a hard strike against Option B for true zero-refactor.
  * **Option A (Desktop):** JavaFX / LibGDX + JBullet / Rapier-Java (native bytecode execution; easiest zero-refactor path).
  * **Option B (Web):** Three.js + Rapier.js + TeaVM / CheerpJ (zero installation; requires transpiling user code, which reintroduces the friction the project is trying to remove).
  * Build the 50 Hz motor-loop micro-prototype only on whichever option Step 1's audit doesn't already eliminate.
* **Open questions:**
  * *Is "runs in-browser with zero install" a hard product requirement, or is a one-time desktop download acceptable?* This is a product call, not a technical one, and may make the prototype comparison moot — answer it first.
  * → Recommend default: **Desktop JVM (Option A) as the committed v1 target**; treat Web/WASM as a v2 stretch goal once zero-refactor is proven on native bytecode.

### Step 3: Classloading Strategy & Local-Directory Import *(depends on Step 2)*
* **Goal:** Finalize how team code is loaded, now that the engine is decided.
* **Action Items:**
  * Benchmark reflection vs. dynamic classloading vs. byte-code rewriting for loading team `.java` files, scoped to the chosen engine.
  * Design the local-directory import mechanism: point the simulator at a team's existing FTC project root (their Android Studio repo checkout) and have it discover, compile, and hot-reload `OpMode` source directly, with no manual export/build step.
* **Open questions:**
  * *Does the watcher need to parse the team's existing Gradle project as-is, or can it require a small marker/config file at the repo root?* → Recommend: require a lightweight `sim.config` marker — much lower implementation effort than fully understanding arbitrary Gradle setups, and it's a one-time addition for the team.

### Step 4: Sim-to-Real Non-Ideality Modeling *(no dependencies — run in parallel with Steps 1–3)*
* **Goal:** Establish mathematical formulations for realistic hardware quirks. Pure math, independently unit-testable — doesn't need the engine decided.
* **Action Items:**
  * **Motor Model:** Implement speed-torque curves, back-EMF, and thermal throttling equations:
    $$\tau = \tau_{stall} \left(1 - \frac{\omega}{\omega_{no\_load}}\right) \cdot \frac{V_{actual}}{V_{nominal}}$$
  * **Battery Sag:** Define a real-time system power draw model:
    $$V_{actual} = V_{internal} - I_{total} \cdot R_{battery}$$
  * **Sensor Delay:** Design an asynchronous thread-safe message queue that introduces $N$ ms configurable latency to sensor reads.
* **Open questions:**
  * *Must all three non-idealities ship together for v1?* → Recommend: no — ship motor curve + battery sag first (they directly affect autonomous path accuracy); defer bus/sensor latency to a fast-follow, since it has smaller gameplay impact and the highest implementation cost (threaded queue).

### Step 5: Telemetry Schema & Logging Wrapper *(no dependencies — start immediately; the calibration algorithm depends on Step 4)*
* **Goal:** Make the "sim-to-real gap" claim measurable, not just asserted, by defining the real-robot data format early so collection can start before the simulator itself is calibration-ready.
* **Action Items:**
  * Define a standard telemetry log schema (timestamped CSV/JSON) capturing motor power commands, encoder ticks, IMU readings, battery voltage, and loop timing.
  * Ship a drop-in logging `OpMode`/telemetry wrapper that teams run once on their physical robot to produce this file with zero custom instrumentation.
  * Build a calibration pass (depends on Step 4's models existing): teams drop the resulting log into the simulator, which replays the same commands in sim, diffs simulated vs. recorded trajectories/voltages, and auto-tunes model parameters (motor curve coefficients, I2C latency, battery internal resistance) to minimize error.
  * This log format and calibration flow becomes the project's ongoing validation methodology, and a regression check for future physics-model changes.
* **Open questions:**
  * *How does the log file get off the Control Hub / Driver Station and onto the dev machine?* → Recommend: write to local storage (USB/internal), plain-file drag-and-drop. Avoid requiring network/cloud upload — competition venue wifi is unreliable and this needs to work in the pits.

### Step 6: CAD & Robot Configuration Pipeline *(depends on Step 3 — needs `HardwareMap` naming finalized)*
* **Goal:** Specify how teams will import custom robots — recommend descoping for MVP.
* **Action Items:**
  * Evaluate standard formats: **URDF** (Unified Robot Description Format) vs. simplified JSON schema.
  * Define joint relationships: Revolute (pivots, arms), Prismatic (linear slides), Continuous (wheels).
  * Map imported joint definitions to `HardwareMap` names (e.g., `"left_front_drive"`).
* **Open questions:**
  * *Does full CAD/URDF import need to exist before the simulator is useful to a team?* → Recommend **no**. Ship early phases with 2–3 hand-authored preset configs (e.g., a standard goBILDA Mecanum kit) so Phases 1–4 aren't blocked waiting on this. Treat full custom import as its own post-MVP phase, informed by feedback from teams already using the presets.

---

## 4. Implementation Phasing Strategy

| Phase | Milestone / Focus | Primary Deliverables | Estimated Duration |
| :--- | :--- | :--- | :--- |
| **Phase 1** | **Core Executor & HAL** | Mocked FTC SDK package, headless Java `OpMode` executor, basic console logging, 2–3 hand-authored preset robot configs (no CAD import yet). | 2 Weeks |
| **Phase 2** | **Kinematics & 2D Canvas** | 12x12 ft tile rendering, Mecanum wheel kinematic solver, USB Gamepad integration. | 2 Weeks |
| | **→ MVP Checkpoint** | **Ship here.** Phases 1–2 alone let a real team load their own `OpMode` and watch it drive on a 2D field — the riskiest assumption (does unmodified code actually run?) is now validated with real users before investing further. Gather feedback before committing to Phase 3+. | — |
| **Phase 3** | **Sensor Latency & Noise** | Threaded IMU/Encoder emulation, configurable I2C bus delay, battery sag engine, first pass of the telemetry-log calibration flow (Step 5). | 3 Weeks |
| **Phase 4** | **3D Engine & Physics** | Rigid-body collision (walls, field game pieces), 3D visualization, intake simulation. | 4 Weeks |
| **Phase 5** | **Custom Robot Importer** | JSON/URDF parser for custom motor locations, slide mechanics, and weight distributions — full replacement for Phase 1's presets. | 3 Weeks |

---

## 5. Next Immediate Action Items

1. **Set Up Proof-of-Concept Repository:**
   * Create the project structure (`core-sdk-mock`, `physics-engine`, `gui-runner`).
2. **Execute Step 1 (SDK Surface Inventory):**
   * Write a basic test runner that compiles a sample `LinearOpMode` at runtime and extracts motor power output calls.
   * Answer the two open questions before moving on: pin the SDK version, and confirm vision is deferred.
3. **Execute Step 2 (Engine Decision):**
   * Confirm whether zero-install web access is a hard requirement; absent that, default to Desktop JVM and skip the Web/WASM prototype.
4. **Draft Robot Configuration Specification (Preset, not CAD):**
   * Create a sample `robot_config.json` schema defining a 4-motor Mecanum chassis, goBILDA motor spec parameters, and IMU orientation — as a hand-authored preset, not the full Step 6 importer.
5. **Draft Telemetry Log Schema (Step 5):**
   * Define the CSV/JSON fields (motor power, encoder ticks, IMU, battery voltage, loop timing) so the logging wrapper can be written in parallel with Phase 1 engineering.