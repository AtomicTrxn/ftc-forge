# Task R2: Engine & Tech Stack Decision

**Type:** Research
**Depends on:** [`tasks/research/01-sdk-surface-inventory.RESULTS.md`](01-sdk-surface-inventory.RESULTS.md) — read this first; its third-party library audit directly informs the Option A/B comparison below.
**Produces:** `tasks/research/02-engine-tech-stack-decision.RESULTS.md`
**Plan reference:** [Next-Gen FTC Robot Simulator Plan.md](../../Next-Gen%20FTC%20Robot%20Simulator%20Plan.md), § 3 Step 2

## Context

This is the single highest-leverage decision in the project: it gates the classloading strategy (R3), the physics engine used in Phase 4, and the rendering stack. Get this wrong and multiple downstream tasks need redoing. If R1's audit found that Road Runner, Pedro Pathing, or FTCDashboard rely on Android-only APIs that a transpiler (CheerpJ/TeaVM) can't handle, that's a near-automatic disqualifier for the Web/WASM option under the project's "zero-refactor" promise — check R1's results before spending time on a symmetric bake-off.

## Objective

Choose between:
- **Option A (Desktop):** JavaFX / LibGDX + JBullet / Rapier-Java — runs real JVM bytecode natively; the most direct path to true zero-refactor.
- **Option B (Web):** Three.js + Rapier.js + TeaVM / CheerpJ — zero install, browser-accessible, but requires *transpiling* user Java code, which reintroduces exactly the kind of code-translation friction the project exists to eliminate.

## Instructions

1. **Read R1's results first.** If its "Recommendation for R2" section already flags a library as incompatible with transpilation, treat that as a strong prior against Option B — don't re-litigate it from scratch, just confirm it against the specific transpiler (CheerpJ vs TeaVM differ in what they support).
2. **Resolve the product question before prototyping:** is "zero install, runs in browser" a hard requirement, or is a one-time desktop download acceptable to the target users (FTC teams — students and mentors, generally comfortable installing a JAR or app)? This is not a technical question and may make the rest of this task moot. Recommended default: **desktop download is acceptable; Desktop JVM (Option A) is the v1 target.** If you have information suggesting otherwise (e.g. a strong signal that browser-only access is a hard constraint), override the default and say why.
3. If the default doesn't resolve it (or you're overriding it), build the 50 Hz motor-update-loop micro-prototype on whichever option(s) remain viable after steps 1-2, and compare stability/jitter.
4. Record which physics engine binding you're committing to (JBullet vs Rapier-Java for Option A; Rapier.js for Option B) — this is a direct input to Phase 4.

## Decisions required

| Question | Recommended default |
|---|---|
| Is zero-install browser access a hard requirement? | No — desktop download is acceptable. |
| Engine choice | Desktop JVM (Option A), given the above. Web/WASM becomes a v2 stretch goal. |

## Output contract

Save to `tasks/research/02-engine-tech-stack-decision.RESULTS.md`:

- `## Summary`
- `## Decision` — Option A or B, stated plainly, one line.
- `## Reasoning` — including how R1's findings factored in.
- `## Physics Engine Binding` — the specific library chosen (e.g. Rapier-Java) — Phase 4 will read this directly.
- `## Prototype Results` (if built) — loop stability/jitter data at 50 Hz.
- `## Implications for R3` — what this decision means for the classloading approach (native JVM classloading vs. transpiled module loading).
- `## Risks / Unknowns Remaining`

## Definition of done

- [ ] R1's results read and referenced.
- [ ] The zero-install product question explicitly answered (default used or overridden with reasoning).
- [ ] A single engine option chosen — no "TBD" or "either works."
- [ ] Physics engine binding named.
- [ ] RESULTS.md saved at the path above.
