# R4 Results: Sim-to-Real Non-Ideality Modeling

**Status:** Complete
**Task spec:** [04-non-ideality-modeling.md](04-non-ideality-modeling.md)

## Summary

Real datasheet constants exist and are usable directly for the motor and battery models — goBILDA publishes per-SKU stall torque/no-load speed/current figures that plug straight into the plan's equations, and REV's competition battery is a well-documented 10-cell NiMH pack. The one place real numbers are genuinely unavailable is thermal throttling (not published by any FTC hardware vendor) and precise battery internal resistance (only a generic, not REV-specific, figure could be found) — both are flagged as calibration targets for R5 rather than hardcoded constants, which is exactly what R5's telemetry-calibration pass exists to refine. Sequencing default confirmed: motor + battery ship in Phase 3, sensor/bus latency deferred to fast-follow.

## Motor Model

**Equation (unchanged from the plan):**

$$\tau = \tau_{stall} \left(1 - \frac{\omega}{\omega_{no\_load}}\right) \cdot \frac{V_{actual}}{V_{nominal}}$$

**Reference constants** (goBILDA 5203 series Yellow Jacket planetary gear motor, 50.9:1 ratio variant — the most common FTC drivetrain/arm motor family, values are at the geared output shaft, already reflecting this specific gear ratio):

| Constant | Value | Source |
|---|---|---|
| `V_nominal` | 12 V | goBILDA 5203 series datasheet |
| `ω_no_load` | 117 RPM (12.25 rad/s) | goBILDA 5203 series, 50.9:1 ratio |
| `τ_stall` | 68.4 kg·cm (6.71 N·m) | goBILDA 5203 series, 50.9:1 ratio |
| `I_no_load` | 0.25 A | goBILDA 5203 series datasheet |
| `I_stall` | 9.2 A | goBILDA 5203 series datasheet |

**Important modeling note:** goBILDA sells this motor in ~10 different gear ratios (3.7:1 through 139:1), each with its own published no-load speed and stall torque (torque scales up, speed scales down, roughly proportionally with ratio, minus a small efficiency loss per gear stage — typically ~90% per planetary stage, compounding for multi-stage ratios). **The motor model's constants table must be keyed per-SKU/per-ratio, not a single hardcoded set** — Phase 1's preset configs (and Phase 5's importer) need to carry the specific ratio a team is using and look up (or let the team supply) that ratio's own constants, not assume every DcMotor on the robot behaves like one specific gearing.

**Function signature (language-agnostic):**
```
torque(motorSpec: {tauStall, omegaNoLoad, vNominal}, omega: double, vActual: double) -> double
```

**Thermal throttling:** no FTC-legal motor vendor (goBILDA, REV, AndyMark) publishes a thermal derating curve — this is not an omission in this research, it's genuinely undocumented in the ecosystem. Recommended v1 approach: model it as a simple duty-cycle heuristic (e.g. derate available torque by a configurable factor after N seconds of continuous stall-current draw above a threshold), explicitly labeled as an estimate, and treat its exact shape as a parameter R5's calibration pass tunes per-team rather than a fixed constant asserted here.

## Battery Sag Model

**Equation (unchanged from the plan):**

$$V_{actual} = V_{internal} - I_{total} \cdot R_{battery}$$

**Reference constants:** REV Robotics' "12V Slim Battery" (`REV-31-1302`), the standard competition battery — confirmed as a **10-cell NiMH pack, 3000 mAh**, XT30 connector, 20A inline replaceable fuse.

- `V_internal` ≈ 12–13.2 V depending on charge state (10 cells × ~1.2–1.32 V/cell nominal NiMH range).
- `R_battery`: **could not find a REV-specific published internal-resistance figure.** A commonly-cited generic figure of **≈0.011 Ω** turned up in FTC community sources but is not confirmed as REV's own spec for this exact battery — treat it as a starting default, not a verified constant. REV's own guidance (found during this research) is that teams should measure and track each individual battery's internal resistance over its lifetime, since it rises with wear — which is itself a strong argument that this constant belongs in R5's per-team calibration flow rather than being hardcoded as one "true" number for all simulated batteries.

**Function signature:**
```
batteryVoltage(vInternal: double, totalCurrentDraw: double, rBattery: double) -> double
```

where `totalCurrentDraw` sums each active motor's instantaneous current draw (derivable from the motor model's torque/speed state via the standard DC motor current relation, not re-derived here).

## Sensor/Bus Latency Model

**Design (spec-level, since R4's sequencing decision defers implementation):** an asynchronous, thread-safe delay queue. Java's own `java.util.concurrent.DelayQueue` is a direct, off-the-shelf fit for this — it natively blocks a consumer from retrieving an element until that element's configured delay has elapsed, which is exactly the "reads are available only after N ms" semantics an I2C latency model needs:

```
DelayQueue<DelayedReading<T>>  // DelayedReading wraps a sensor value + a target availability timestamp
```

- On a sensor read request, the producer enqueues the current true value wrapped with `now + N ms` as its delay.
- The consumer (OpMode-facing sensor stub) polls the queue; `DelayQueue` only yields entries once their delay has expired, giving correct-by-construction latency without hand-rolled timer bookkeeping.
- `N` should be configurable per sensor type — the plan's cited 7–10 ms range for I2C is consistent with the known ~100 Hz (≈10 ms) polling cadence of the Lynx (Control/Expansion Hub) I2C read/write thread, though this wasn't independently re-verified against REV's firmware source in this task.

## Sequencing Decision

**Confirmed, no override:** motor model + battery sag ship in Phase 3; sensor/bus latency deferred to a fast-follow. This research reinforces the default rather than challenging it — the motor and battery constants above are grounded in real, obtainable datasheet numbers today, while the latency model's one debatable parameter (the exact per-sensor `N`) is safe to leave configurable and revisit later without blocking Phase 3.

## Validation Notes

Unit-test reference points, using the goBILDA 50.9:1 constants above as the concrete test fixture:
- At `vActual = vNominal` and `omega = 0`: `torque` should equal `τ_stall` (6.71 N·m) exactly.
- At `vActual = vNominal` and `omega = ω_no_load` (12.25 rad/s): `torque` should equal 0.
- At `vActual = 0.5 * vNominal`: stall torque at `omega = 0` should be exactly half of the full-voltage stall torque (linear scaling term in the equation).
- Battery model: at `totalCurrentDraw = 0`, `batteryVoltage` should equal `vInternal` exactly (no sag with no load).

## Risks / Unknowns Remaining

- **Battery internal resistance is not a confirmed REV-specific spec** — flagged above, and should be explicitly surfaced to R5 as the single highest-priority parameter for the calibration pass to tune per real telemetry, rather than trusted as-is.
- **Thermal throttling has no vendor-published curve to validate against** — the v1 duty-cycle heuristic is a reasonable placeholder, not a validated model; don't present its output as "realistic" without a calibration pass behind it.
- Motor constants are keyed to one specific goBILDA SKU/ratio (50.9:1) for illustration — Phase 1/5 must not hardcode this single ratio's numbers as if they applied to every DcMotor a team might configure.
