# Project Plan: Next-Gen FTC Robot Simulator

## 1. Project Vision & Goals

The objective of this project is to build an open-source, high-fidelity FIRST Tech Challenge (FTC) robot simulator that bridges the **Sim-to-Real Gap**. Unlike existing tools, this simulator will combine native FTC SDK execution, configurable hardware non-idealities (bus latency, motor back-EMF, wheel slip), and custom CAD/URDF importing to allow teams to accurately test autonomous routines, motion profiles, and control loops before deploying code to physical hardware.

### Primary Objectives
* **Zero-Refactor Code Execution:** Execute unmodified Java `OpMode` and `LinearOpMode` code directly against a mocked `com.qualcomm.robotcore` package structure.
* **Realistic Hardware Dynamics:** Model real-world physical behavior including I2C bus latency (7–10 ms), battery voltage sag under load, wheel slippage, and non-ideal Mecanum/Swerve kinematics.
* **Custom Mechanism & Robot Import:** Provide a streamlined pipeline (Onshape/URDF/JSON) to allow teams to import custom robot geometries, mass properties, and motor configurations.
* **Low-Friction Setup:** Eliminate complex Android Studio repo refactoring by supplying a lightweight, cross-platform runner (Desktop Java or WebAssembly/WebGL).

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

Before writing core engine code, the following research spikes and technical specs must be completed:

### Task 1: FTC SDK Mocking & Class Loading Strategy
* **Goal:** Determine the best mechanism to run actual team code without Android SDK dependencies.
* **Research Action Items:**
  * Benchmark reflection vs. dynamic classloading vs. byte-code rewriting for loading team `.java` files.
  * Define the full set of stubbed classes required from `com.qualcomm.robotcore` (e.g., `HardwareMap`, `Gamepad`, `Telemetry`, `DcMotorEx`, `IMU`).
  * Investigate integration with popular third-party FTC libraries (**Road Runner**, **Pedro Pathing**, **FTCDashboard**).

### Task 2: Engine & Tech Stack Evaluation
* **Goal:** Select the primary engine architecture (Desktop Java vs. WebAssembly/WebGL).
* **Research Action Items:**
  * **Option A (Desktop):** JavaFX / LibGDX + JBullet / Rapier-Java (Easiest Java code loading; native speed).
  * **Option B (Web):** Three.js + Rapier.js + TeaVM / CheerpJ (Zero installation; accessible in web browsers).
  * Build a micro-prototype comparing motor update loop stability at 50 Hz on both platforms.

### Task 3: Sim-to-Real Non-Ideality Modeling
* **Goal:** Establish mathematical formulations for realistic hardware quirks.
* **Research Action Items:**
  * **Motor Model:** Implement speed-torque curves, back-EMF, and thermal throttling equations:
    $$\tau = \tau_{stall} \left(1 - \frac{\omega}{\omega_{no\_load}}\right) \cdot \frac{V_{actual}}{V_{nominal}}$$
  * **Battery Sag:** Define a real-time system power draw model:
    $$V_{actual} = V_{internal} - I_{total} \cdot R_{battery}$$
  * **Sensor Delay:** Design an asynchronous thread-safe message queue that introduces $N$ ms configurable latency to sensor reads.

### Task 4: CAD & Robot Configuration Pipeline
* **Goal:** Specify how teams will import custom robots.
* **Research Action Items:**
  * Evaluate standard formats: **URDF** (Unified Robot Description Format) vs. simplified JSON schema.
  * Define joint relationships: Revolute (pivots, arms), Prismatic (linear slides), Continuous (wheels).
  * Map imported joint definitions to `HardwareMap` names (e.g., `"left_front_drive"`).

---

## 4. Implementation Phasing Strategy

| Phase | Milestone / Focus | Primary Deliverables | Estimated Duration |
| :--- | :--- | :--- | :--- |
| **Phase 1** | **Core Executor & HAL** | Mocked FTC SDK package, headless Java `OpMode` executor, basic console logging. | 2 Weeks |
| **Phase 2** | **Kinematics & 2D Canvas** | 12x12 ft tile rendering, Mecanum wheel kinematic solver, USB Gamepad integration. | 2 Weeks |
| **Phase 3** | **Sensor Latency & Noise** | Threaded IMU/Encoder emulation, configurable I2C bus delay, battery sag engine. | 3 Weeks |
| **Phase 4** | **3D Engine & Physics** | Rigid-body collision (walls, field game pieces), 3D visualization, intake simulation. | 4 Weeks |
| **Phase 5** | **Custom Robot Importer** | JSON/URDF parser for custom motor locations, slide mechanics, and weight distributions. | 3 Weeks |

---

## 5. Next Immediate Action Items

1. **Set Up Proof-of-Concept Repository:**
   * Create the project structure (`core-sdk-mock`, `physics-engine`, `gui-runner`).
2. **Execute Research Task 1 (SDK Mocking):**
   * Write a basic test runner that compiles a sample `LinearOpMode` at runtime and extracts motor power output calls.
3. **Draft Robot Configuration Specification:**
   * Create a sample `robot_config.json` schema defining a 4-motor Mecanum chassis, goBILDA motor spec parameters, and IMU orientation.