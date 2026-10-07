# Sensor realism: IMU errors, bus cost, velocity estimation, battery resistance

Baseline: `e5fbd01`. Next batch of unmodeled items, all behind `sim.config` parameters (defaults ideal / previous behavior).

- `imu` block: scale error, drift, gyro bias/noise, angle noise, seed, read cost (reads block outside the sample lock).
- `odometry.read_cost_ms` and `imu.read_cost_ms`: I2C round-trip time for IMU, Pinpoint and OTOS reads.
- `bus.velocity_window_ms`: hub-style velocity from whole encoder counts over a window (quantized, smoothed).
- `battery.empty_resistance_ohm`: internal resistance rising with depletion.

Not changed: temperature, regeneration, distance/color/touch bus cost, real hub timing, fitting these parameters from a recording
(they are exposed so a future calibration pass can fit them; no pass exists yet).
