# Battery sag and bus latency audit

Baseline: `8ee8d9d`. Proposal item 3.

## Audit result

| Area | Finding |
|---|---|
| Battery sag | Implemented per R4 and wired to motors and the hub voltage sensor. Closed form verified by new tests. Only changeable through a calibration profile, not `sim.config`. |
| Encoder latency | Implemented, but the ring buffer rounded latency up to the 20 ms tick: a configured 8 ms acted as 20 ms. Not configurable. |
| Velocity | Not delayed, although the hub returns it with the position. |
| IMU latency | Implemented and configurable (`imu_latency_ms`), renderer only. |
| Other sensors, Pinpoint/OTOS | No latency (constant or undelayed). |

## Changes

1. `InterpolatedSensorBuffer`: linear interpolation between ticks, so configured latency is exact.
2. `SimDcMotorEx`: position and velocity delayed through it; per-motor latency setting replaces the global static.
3. `sim.config`: `encoder_latency_ms` and `battery.internal_voltage_v` / `internal_resistance_ohm`, validated, applied before a
   calibration profile.
4. Tests: latency reproduction (failed before the fix), buffer unit tests, battery closed form, recovery, resistance ordering,
   calibration precedence, config validation.
5. `gui-runner/POWER_AND_BUS.md`: behavior and explicit limits.

## Not changed

Placeholder rotor inertia and zero default friction (sag is transient headless), state-of-charge droop, Pinpoint/OTOS latency,
current/voltage read latency, I2C read cost.
