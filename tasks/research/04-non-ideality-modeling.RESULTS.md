# R4 Results: Sim-to-Real Non-Ideality Modeling

**Status:** Complete (amended after Opus review pass — see changelog)
**Task spec:** [04-non-ideality-modeling.md](04-non-ideality-modeling.md)

**Changelog — the first item below is a correctness fix, not a refinement:** the plan's original motor torque equation was physically wrong (it implied no-load speed doesn't change with applied voltage, which contradicts basic DC motor behavior and makes braking/`ZeroPowerBehavior.BRAKE` unmodelable) — replaced with the correct linear torque-speed-voltage relation, and added the current equation the original pass left as "not re-derived here" despite Phase 3 needing it directly. Also replaced the wall-clock `DelayQueue` sensor-latency design (inconsistent with running on simulated time) with a deterministic ring-buffer approach, re-assessed `R_battery`'s default (the ≈0.011 Ω figure looks like it was conflated with FRC's lead-acid battery spec, not FTC's NiMH pack), and added a per-ratio encoder/motor-constant table since Phase 1/2 need encoder ticks-per-revolution and the original fixture (50.9:1) is an arm ratio, not a typical drivetrain one.

## Summary

Real datasheet constants exist and are usable directly for the motor and battery models — goBILDA publishes per-SKU stall torque/no-load speed/current figures that plug straight into corrected versions of the plan's equations, and REV's competition battery is a well-documented 10-cell NiMH pack. The one place real numbers are genuinely unavailable is thermal throttling (not published by any FTC hardware vendor) and precise battery internal resistance (only a generic, and on reflection probably wrong-chemistry, figure could be found) — both are flagged as calibration targets for R5 rather than hardcoded constants, which is exactly what R5's telemetry-calibration pass exists to refine. Sequencing default confirmed: motor + battery ship in Phase 3, sensor/bus latency deferred to fast-follow.

## Motor Model

**Corrected equation** (the plan's original equation is physically wrong — see below):

$$\tau = \tau_{stall} \left(\frac{V_{actual}}{V_{nominal}} - \frac{\omega}{\omega_{no\_load}}\right)$$

$$I = I_{stall} \left(\frac{V_{actual}}{V_{nominal}} - \frac{\omega}{\omega_{no\_load}}\right)$$

**Why the plan's original equation (`τ = τ_stall(1 − ω/ω_nl)·(V/V_nom)`) was wrong:** at any nonzero applied voltage, that equation reaches `τ = 0` only when `ω = ω_no_load` — i.e. it implies every motor coasts up to the *same* no-load speed regardless of applied voltage, which contradicts basic DC motor behavior (real no-load speed scales with applied voltage: half voltage means roughly half no-load speed, not full speed at half torque). A direct consequence: the original equation cannot model `ZeroPowerBehavior.BRAKE` at all, since at `V_actual = 0` it always returns `τ = 0` regardless of `ω`, when physically a shorted/braked motor produces a *reverse* (braking) torque proportional to its speed. The corrected equation above is the standard linear DC-motor torque-speed-voltage relation (derivable directly from `V = IR + k_e·ω`, `τ = k_t·I`): at `V = V_nom, ω = 0` it gives `τ_stall` (matches the datasheet point); at `V = V_nom, ω = ω_no_load` it gives `τ = 0` (matches); at `V = 0` (brake) it gives `τ = −τ_stall·ω/ω_no_load`, a physically correct speed-proportional braking torque. The two equations are indistinguishable at `V = V_nom` and at `V = 0.5·V_nom, ω = 0` — which is exactly why the original Validation Notes' unit tests didn't catch this; a new test that does is added below.

**Reference constants** (goBILDA 5203 series Yellow Jacket planetary gear motor — see the per-ratio table below; 50.9:1 was the original illustrative fixture, but it's an arm/slide ratio, not a typical drivetrain one):

| Constant | Value | Source |
|---|---|---|
| `V_nominal` | 12 V | goBILDA 5203 series datasheet |
| `ω_no_load` | 117 RPM (12.25 rad/s) | goBILDA 5203 series, 50.9:1 ratio |
| `τ_stall` | 68.4 kg·cm (6.71 N·m) | goBILDA 5203 series, 50.9:1 ratio |
| `I_no_load` | 0.25 A | goBILDA 5203 series datasheet |
| `I_stall` | 9.2 A | goBILDA 5203 series datasheet |

**Per-ratio table (drivetrains typically use 13.7:1 or 19.2:1, not 50.9:1):**

| Ratio | No-load speed | Stall torque | Encoder counts/rev at output |
|---|---|---|---|
| 19.2:1 | 312 RPM | 24.3 kg·cm | **537.7** |
| 13.7:1 | 435 RPM | ~18.7 kg·cm *(not independently confirmed — verify against goBILDA's product page before hardcoding)* | *(not independently confirmed)* |
| 50.9:1 | 117 RPM | 68.4 kg·cm | *(not independently confirmed)* |

The 19.2:1 row was independently fetched and confirmed directly from goBILDA's own product page during this review pass (312 RPM, 24.3 kg·cm, 537.7 PPR at the output shaft, all at 12 VDC) — treat it as verified. The other rows are placeholders following the same expected pattern and **must be confirmed against goBILDA's actual product pages** before Phase 1 hardcodes them; do not trust them as-is. Also note: goBILDA's own published ratios are not exact integers (e.g. "19.2:1" is actually closer to 19.203:1) — relevant for exact encoder-tick math, not just torque/speed.

**Important modeling note:** goBILDA sells this motor in ~10 different gear ratios (3.7:1 through 139:1), each with its own published no-load speed, stall torque, and encoder resolution. **The motor model's constants table must be keyed per-SKU/per-ratio, not a single hardcoded set** — Phase 1's preset configs (and Phase 5's importer) need to carry the specific ratio a team is using and look up (or let the team supply) that ratio's own constants, not assume every DcMotor on the robot behaves like one specific gearing. (See R3's results for where this ratio identifier needs to live, since the real robot-configuration XML likely can't encode it.)

**Function signature (language-agnostic):**
```
torque(motorSpec: {tauStall, iStall, omegaNoLoad, vNominal}, omega: double, vActual: double) -> double
current(motorSpec: {tauStall, iStall, omegaNoLoad, vNominal}, omega: double, vActual: double) -> double
```

**`DcMotorEx` run-mode behaviors this model must back (flagged as unaddressed in the original pass):** `RUN_USING_ENCODER` doesn't apply `vActual` directly — `setPower()` in this mode becomes a target-velocity command that REV's hub firmware tracks with its own onboard PIDF loop; the simulated equivalent is a velocity controller (target velocity = commanded power × max ticks/sec) running at a fixed rate on top of the torque model above, not a direct voltage passthrough. `RUN_TO_POSITION` layers a position controller on top of that. `getCurrent()` reads directly from the `current()` function above. Default PIDF coefficients for the simulated hub-side controller are not established here — a reasonable placeholder tuned during Phase 3, refined by R5's calibration pass.

**Thermal throttling:** no FTC-legal motor vendor (goBILDA, REV, AndyMark) publishes a thermal derating curve — this is not an omission in this research, it's genuinely undocumented in the ecosystem. Recommended v1 approach: model it as a simple duty-cycle heuristic (e.g. derate available torque by a configurable factor after N seconds of continuous stall-current draw above a threshold), explicitly labeled as an estimate, and treat its exact shape as a parameter R5's calibration pass tunes per-team rather than a fixed constant asserted here.

## Battery Sag Model

**Equation (unchanged from the plan):**

$$V_{actual} = V_{internal} - I_{total} \cdot R_{battery}$$

**Reference constants:** REV Robotics' "12V Slim Battery" (`REV-31-1302`), the standard competition battery — confirmed as a **10-cell NiMH pack, 3000 mAh**, XT30 connector, 20A inline replaceable fuse.

- `V_internal` ≈ 12–13.2 V depending on charge state (10 cells × ~1.2–1.32 V/cell nominal NiMH range).
- `R_battery`: **could not find a REV-specific published internal-resistance figure, and on reflection the ≈0.011 Ω figure cited in the original pass is probably the wrong battery chemistry entirely.** That figure is much more consistent with the "good battery" internal-resistance threshold commonly cited for FRC's 18 Ah lead-acid batteries — a physically different, much larger pack — than a 3000 mAh 10-cell NiMH pack. At 0.011 Ω, even a 20 A load only sags the pack by 0.22 V, which would make the sag model nearly invisible in Phase 3 and undermine the whole point of modeling it. **Revised default: `R_battery` ≈ 0.15 Ω** (a placeholder consistent with a small NiMH pack plus wiring/connector/fuse resistance, not independently bench-measured), with a **calibration search range of 0.05–0.40 Ω** for R5's fit — both numbers only as good as the first real telemetry log run through calibration, which is exactly the point of that pass. REV's own guidance (found during this research) is that teams should measure and track each individual battery's internal resistance over its lifetime, since it rises with wear — a strong argument this constant belongs in R5's per-team calibration flow rather than being hardcoded as one "true" number for all simulated batteries.

**Function signature — closed-form, multi-motor.** The original pass deferred the current-draw derivation entirely ("not re-derived here"); that's a real gap, since Phase 3 has nothing to implement without it. Treating each motor's commanded power `p_i` as a PWM duty cycle applied to the shared battery rail (`V_applied_i = p_i · V_b`), and approximating average battery-side current per motor as duty-scaled motor current (`I_batt_i ≈ p_i · I_i`), the system solves in closed form with no iteration:

```
A = Σ_i  p_i² · I_stall_i / V_nom_i
B = Σ_i  p_i · I_stall_i · (ω_i / ω_no_load_i)

V_b = (V_internal + R_battery · B) / (1 + R_battery · A)

batteryVoltage(vInternal, rBattery, motors: [{p_i, iStall_i, vNom_i, omega_i, omegaNoLoad_i}]) -> double
```

This is a **duty-averaged H-bridge approximation** — it does not model per-PWM-cycle current ripple, which is an acceptable simplification for a per-loop-tick simulation running far slower than actual PWM switching frequency. Derivation: substituting the motor current equation above into `I_batt_i = p_i · I_i` and summing across motors gives `I_total = V_b·A − B`; substituting into `V_b = V_internal − I_total·R_battery` and solving for `V_b` yields the closed form above.

## Sensor/Bus Latency Model

**Corrected design — the original `DelayQueue` approach is withdrawn, not just refined.** A `java.util.concurrent.DelayQueue` reads the JVM's wall clock (`System.nanoTime()` internally) to decide when an element becomes available, which directly conflicts with two other decisions already made: R3's requirement that the simulator's own clock (not the wall clock) governs time, and R5's deterministic-replay calibration pass, which needs identical inputs to always produce identical outputs — a wall-clock-timed queue makes results depend on real elapsed time between calls, which will vary between runs and machines. It also adds a real background thread for what should be simple per-tick state.

**Replacement: a per-sensor ring buffer of `(sim_time, value)` samples, read against the simulator's own clock, single-threaded and deterministic:**

```
RingBuffer<(simTimeMs: long, value: T)>  // sized to hold at least `latencyMs / tickIntervalMs` samples

onTick(simTimeMs, trueValue):
    buffer.push((simTimeMs, trueValue))

read(simTimeMs, latencyMs) -> T:
    return buffer.valueAtOrBefore(simTimeMs - latencyMs)
```

- Every simulation tick pushes the current true sensor value tagged with the simulator's own current time — no separate producer thread.
- A read returns whatever value was true at `now − latencyMs`, and (optionally) advances the OpMode's own clock view by a configurable `readCostMs` to model the blocking cost of an I2C round-trip on the loop thread, not just delivery delay.
- `latencyMs` and `readCostMs` are configurable per sensor type. The plan's cited 7–10 ms range for I2C bus latency is retained as the starting default, but the specific claim that this matches "a ~100 Hz Lynx I2C polling thread" should be treated as **unverified and possibly conflated** — Lynx I2C reads are understood to be synchronous command/response round-trips rather than a fixed-rate polling loop, and a fixed "~100 Hz polling thread" framing may be describing an older, Modern-Robotics-era hardware generation rather than current Lynx-based hubs. Keep the 7–10 ms default, but don't cite the polling-thread justification for it without independently confirming it against REV's actual firmware behavior.
- This design is also *cheaper* to implement than the original threaded queue, which weakens (rather than strengthens) the case for deferring it — Phase 3 should reassess whether sensor latency can ship alongside motor/battery after all, now that its implementation cost has dropped.

## Sequencing Decision

**Confirmed, no override:** motor model + battery sag ship in Phase 3; sensor/bus latency deferred to a fast-follow. This research reinforces the default rather than challenging it — the motor and battery constants above are grounded in real, obtainable datasheet numbers today, while the latency model's one debatable parameter (the exact per-sensor `N`) is safe to leave configurable and revisit later without blocking Phase 3.

## Validation Notes

Unit-test reference points, using the goBILDA 50.9:1 constants above as the concrete test fixture:
- At `vActual = vNominal` and `omega = 0`: `torque` should equal `τ_stall` (6.71 N·m) exactly.
- At `vActual = vNominal` and `omega = ω_no_load` (12.25 rad/s): `torque` should equal 0.
- **At `vActual = 0.5 · vNominal` and `omega = 0.5 · ω_no_load`: `torque` should equal 0.** This is the test that actually distinguishes the corrected equation from the plan's original (wrong) one — the "half voltage → half stall torque at zero speed" test alone passes under both formulas and does not catch the bug, so it's retained below only as a secondary check, not the primary one.
- At `vActual = 0.5 * vNominal`, `omega = 0`: `torque` should equal half the full-voltage stall torque (both equations agree here — this checks linearity in `V`, not the `ω`/`V` interaction).
- Battery model: at `totalCurrentDraw = 0` (equivalently, all `p_i = 0`), `batteryVoltage` should equal `vInternal` exactly (no sag with no load).

## Risks / Unknowns Remaining

- **Battery internal resistance is not a confirmed REV-specific spec, and the original default was likely the wrong chemistry entirely** (see above) — should be explicitly surfaced to R5 as the single highest-priority parameter for the calibration pass to tune per real telemetry, using the revised 0.05–0.40 Ω search range, not trusted as a fixed value.
- **Thermal throttling has no vendor-published curve to validate against** — the v1 duty-cycle heuristic is a reasonable placeholder, not a validated model; don't present its output as "realistic" without a calibration pass behind it.
- Motor constants are keyed to one specific goBILDA SKU/ratio (50.9:1) for illustration, and only the 19.2:1 row of the per-ratio table is independently confirmed — Phase 1/5 must not hardcode any of these numbers without checking them against goBILDA's actual product pages first.
- **Battery internal voltage is treated as constant** (12–13.2 V band) rather than a function of state of charge. A full ~2:30 match can draw down a meaningful share of a 3000 mAh pack's capacity, and `V_internal` should reasonably droop over that time — not modeled here; worth a follow-up if Phase 3's calibration shows systematic late-match error.
- The duty-averaged battery current approximation (H-bridge PWM modeled as a scaled DC equivalent rather than simulating actual switching) is a deliberate simplification appropriate for per-tick simulation, not a limitation expected to matter at this fidelity level — flagged for completeness, not as a concern requiring near-term follow-up.
