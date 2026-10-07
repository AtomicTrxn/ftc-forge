# Actuator models

Baseline: `c36a025` (unshipped thermal branch). All behind `sim.config`; defaults change nothing.

- `transmission_efficiency`: asymmetric driving/driven torque through the post-motor stage, for physics-joint motors.
- `backlash_motor_rad`: encoder hysteresis against the load (read-side only).
- `velocity_i_gain` / `velocity_d_gain`: integral (anti-windup) and derivative terms in the `RUN_USING_ENCODER` loop, normalized units.
- `battery.capacity_temp_coeff_per_k`: usable capacity falls when cold.
- `bus.sensor_read_cost_ms` now also applies to touch sensors.

Not changed: backlash torque dead zone, hub PIDF firmware units (a team's tuned hub values do not transfer), cell imbalance.
Verified in the native articulated-joint physics (`ActuatorPhysicsTest`) and unit tests; nothing is checked against hardware.
