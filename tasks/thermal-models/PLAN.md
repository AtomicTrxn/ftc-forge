# Thermal models and sensor bus cost

Baseline: `84e849f`. All behind `sim.config`; defaults leave previous behavior unchanged.

- **Winding thermal model** (`motor_defaults` / `motor_overrides`): temperature from I^2*R, ambient cooling, resistance rising with
  temperature (less torque/current), derating between two temperatures. Exact first-order step. Off unless `thermal_capacity_j_per_k > 0`.
- **Pack temperature** (`battery`): self-heating, ambient, cold-resistance coefficient, initial temperature.
- **`bus.sensor_read_cost_ms`** for distance and color sensor reads.

Not changed: gearbox efficiency/backlash (needs articulated-joint work), cell imbalance, capacity-vs-temperature, touch-sensor cost.
Thermal parameters are not fitted: the robot log has no temperature column. No vendor publishes the defaults.
