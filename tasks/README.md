# Agent Task Pipeline — Next-Gen FTC Robot Simulator

This directory breaks the [project plan](../Next-Gen%20FTC%20Robot%20Simulator%20Plan.md) into individual, self-contained task specs. Each file in `research/` or `implementation/` is written to be handed directly to an AI agent (or a person) as a starting prompt — it carries its own context, dependencies, and output contract, so it can be worked independently without needing this conversation's history.

## How this works

- Each task file states what it **depends on** (prior task RESULTS files to read first) and what it **produces** (a `*.RESULTS.md` file, and for implementation tasks, code).
- A dependency's RESULTS file may not exist yet if that task hasn't run. Task files still link to the *expected* path — check whether it exists before starting; if it doesn't, that dependency needs to run first.
- When a task completes, its RESULTS.md is saved alongside its spec file, in the same directory, so the pair travels together.
- Later tasks — especially implementation phases — read the RESULTS files of the research steps they depend on, rather than re-deriving decisions already made. This is the "dynamic link back" mechanism: the spec doesn't hardcode the research's conclusions, it points at the file that holds them, so if a research task is redone or updated, downstream tasks automatically pick up the new answer next time they run.
- **This only works if amendments actually remove what they supersede.** A cross-file review pass (e.g. R2 resolving something R1 had flagged as an open risk) must update or delete every statement in the other file that the new finding contradicts, not just add a note — a downstream reader has no way to know an "unresolved" risk was quietly fixed elsewhere unless the file that stated it is corrected too. When amending a RESULTS file for this reason, add a one-line **Changelog** entry at the top (what changed and why) so the amendment itself is visible, not just its effect.

## Execution order & dependency graph

**Research** (parallel tracks noted):

```
R1 SDK Surface Inventory ──▶ R2 Engine & Tech Stack Decision ──▶ R3 Classloading & Local-Dir Import ──┐
                                                                                                        │
R4 Non-Ideality Modeling (parallel, no deps) ─────────────────────────────────────────────────────────┼─▶ used by Phase 3
                                                                                                        │
R5 Telemetry Schema & Logging Wrapper (parallel; calibration piece needs R4) ──────────────────────────┼─▶ used by Phase 3
                                                                                                        │
R6 CAD & Robot Config Pipeline (needs R3) ─────────────────────────────────────────────────────────────┴─▶ used by Phase 5
```

**Implementation** (sequential; each depends on the previous phase's code plus the research listed):

| Phase | Spec file | Depends on (research) | Depends on (prior phase) |
|---|---|---|---|
| 1 | [implementation/phase-1-core-executor-hal.md](implementation/phase-1-core-executor-hal.md) | R1, R2, R3 | — |
| 2 | [implementation/phase-2-kinematics-2d-canvas.md](implementation/phase-2-kinematics-2d-canvas.md) | — | Phase 1 |
| 3 | [implementation/phase-3-sensor-latency-noise.md](implementation/phase-3-sensor-latency-noise.md) | R4, R5 | Phase 2 |
| 4 | [implementation/phase-4-3d-engine-physics.md](implementation/phase-4-3d-engine-physics.md) | R2 | Phase 2, Phase 3 |
| 5 | [implementation/phase-5-custom-robot-importer.md](implementation/phase-5-custom-robot-importer.md) | R6, R3 | Phase 1 |

**MVP checkpoint:** ship after Phase 2 and validate with a real team before starting Phase 3 (see Phase 2's spec).

## Research task index

| Task | Spec file |
|---|---|
| R1 | [research/01-sdk-surface-inventory.md](research/01-sdk-surface-inventory.md) |
| R2 | [research/02-engine-tech-stack-decision.md](research/02-engine-tech-stack-decision.md) |
| R3 | [research/03-classloading-local-directory-import.md](research/03-classloading-local-directory-import.md) |
| R4 | [research/04-non-ideality-modeling.md](research/04-non-ideality-modeling.md) |
| R5 | [research/05-telemetry-schema-logging-wrapper.md](research/05-telemetry-schema-logging-wrapper.md) |
| R6 | [research/06-cad-robot-config-pipeline.md](research/06-cad-robot-config-pipeline.md) |

## Conventions every task file follows

- **Depends on** — RESULTS.md files to read before starting, with reasons.
- **Produces** — the exact path of the RESULTS.md this task must write, plus (for implementation tasks) the code module(s) it should create/modify.
- **Context** — enough background to work with zero prior conversation memory.
- **Objective / Instructions** — concrete, ordered action items.
- **Decisions required** — open questions carried over from the plan, each with a recommended default. An agent may deviate from the default but must record its reasoning in the RESULTS file.
- **Output contract** — the required section headings for the RESULTS.md file.
- **Definition of done** — a checklist to confirm before considering the task complete.
