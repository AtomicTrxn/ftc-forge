# Power-system, bus and sensor-error modeling

Baseline: `b06863e`. Implements the "not modeled" list from the battery/bus audit, every item behind configuration.

| Item | Parameters (sim.config) | Default |
|---|---|---|
| Battery state of charge | `battery.capacity_mah`, `initial_charge_fraction`, `empty_voltage_v`, `curve_shape` | off |
| Brownout | `battery.brownout_voltage_v`, `brownout_hold_s`, `brownout_recovery_voltage_v` | off |
| Hub/servo loads | `loads.hub_baseline_a`, `servo_hold_a`, `servo_active_a`, `servo_active_duration_ms` | 0 |
| Voltage/current read latency | `bus.voltage_latency_ms`, `current_latency_ms` | 0 |
| Bulk caching (OFF/AUTO/MANUAL) and read cost | `LynxModule` mode, `bus.read_cost_ms` | live reads, free |
| Pinpoint/OTOS error model | `odometry.*` | ideal |
| Motor placeholders | `motor_defaults`, `motor_overrides` | previous constants |

Also fixed: FLOAT zero-power motors were braked as if BRAKE.

Design: pure models in `physics-engine` (`BatteryPack`, `BatteryModel` extra load, `InterpolatedSensorBuffer`); wiring and parsing in
`simcore.ElectricalConfig`, applied before any calibration profile so measured values win. Per-hub bus state lives in the
hub object (`SimVoltageSensor`), so each hub caches independently.

Not changed: real hub brownout threshold/reboot behavior, battery temperature, regeneration; none are checked against a robot.
