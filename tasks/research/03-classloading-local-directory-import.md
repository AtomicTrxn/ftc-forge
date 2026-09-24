# Task R3: Classloading Strategy & Local-Directory Import

**Type:** Research
**Depends on:** [`tasks/research/02-engine-tech-stack-decision.RESULTS.md`](02-engine-tech-stack-decision.RESULTS.md) — read this first; the engine choice determines whether this is a native JVM classloading problem or a transpilation/module-loading problem.
**Produces:** `tasks/research/03-classloading-local-directory-import.RESULTS.md`
**Plan reference:** [Next-Gen FTC Robot Simulator Plan.md](../../Next-Gen%20FTC%20Robot%20Simulator%20Plan.md), § 3 Step 3

## Context

This finalizes *how* a team's existing project gets into the simulator. The product goal (per the plan's "Low-Friction Setup" objective) is: a team points the simulator at their existing FTC project folder on disk — the simulator finds, compiles, and hot-reloads their `OpMode` source directly, with no export or packaging step. This task both benchmarks the loading mechanism and designs that folder-watching UX. Its output — specifically, the finalized `HardwareMap` stub naming/shape — is a direct input to Phase 5 (R6/custom robot importer), since imported robot configs must map onto whatever naming convention this task settles on.

## Objective

1. Pick a concrete classloading mechanism (reflection / `URLClassLoader` / dynamic compilation via the Java Compiler API / bytecode rewriting), scoped to the engine chosen in R2.
2. Design the local-directory watch-and-import mechanism.

## Instructions

1. **Read R2's results.** If Desktop JVM was chosen, this is a native classloading problem (javax.tools `JavaCompiler` + `URLClassLoader` is the likely path). If Web/WASM was chosen, this is a transpilation-triggered rebuild problem instead — different tooling entirely.
2. **Benchmark the options** for the chosen engine: compile-and-load latency, hot-reload feasibility (can a changed file be recompiled without restarting the whole simulator?), and how cleanly it isolates team code from the simulator's own classpath.
3. **Design the "point at your folder" mechanism.** Decide: does the simulator need to parse the team's existing Gradle project structure, or can it require a small marker/config file at the repo root (e.g. `sim.config` naming the source root and entry OpMode)? Recommended default: **require the marker file** — much less effort than handling arbitrary Gradle setups, and it's a one-time addition per team.
4. **Finalize the mock `HardwareMap` naming/shape** — the exact API surface and string-keyed lookup convention (e.g. `hardwareMap.get(DcMotor.class, "left_front_drive")`) that both this loader and, later, Phase 5's robot-config importer must agree on.

## Decisions required

| Question | Recommended default |
|---|---|
| Parse team's Gradle project as-is, or require a marker file? | Require a lightweight `sim.config` marker file at the repo root. |

## Output contract

Save to `tasks/research/03-classloading-local-directory-import.RESULTS.md`:

- `## Summary`
- `## Classloading Mechanism Chosen` — and why, relative to the engine from R2.
- `## Local-Directory Import Design` — the `sim.config` format (or alternative), file-watch approach, hot-reload behavior.
- `## Finalized HardwareMap Naming Convention` — exact shape/example, since Phase 5 depends on this being stable.
- `## Risks / Unknowns Remaining`

## Definition of done

- [ ] R2's results read and the classloading approach matches the chosen engine.
- [ ] A concrete `sim.config` (or equivalent) format specified with an example.
- [ ] HardwareMap naming convention finalized with a worked example.
- [ ] RESULTS.md saved at the path above.
