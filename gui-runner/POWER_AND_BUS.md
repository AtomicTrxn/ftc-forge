# Power system, bus and sensor-error models

All of these are plain parameters in `sim.config`, validated on load (unknown keys, wrong types and
out-of-range values are rejected), so each can be adjusted by hand now and fitted from a recording later.
**Every default is an estimate, not a measurement**, and the optional models are off until you configure them.
A [calibration profile](CALIBRATION.md) can fit many of these from a recording; its `electrical` block overlays this file
setting by setting, and its measured battery and friction values win.

```json
{
  "imu_latency_ms": 8,
  "encoder_latency_ms": 8,
  "battery": {
    "internal_voltage_v": 12.6, "internal_resistance_ohm": 0.15,
    "capacity_mah": 0, "initial_charge_fraction": 1.0, "empty_voltage_v": 10.5, "curve_shape": 1.0,
    "empty_resistance_ohm": -1,
    "brownout_voltage_v": 0, "brownout_hold_s": 2.0, "brownout_recovery_voltage_v": 0
  },
  "loads": { "hub_baseline_a": 0, "servo_hold_a": 0, "servo_active_a": 0, "servo_active_duration_ms": 300 },
  "bus": { "voltage_latency_ms": 0, "current_latency_ms": 0, "read_cost_ms": 0, "velocity_window_ms": 0 },
  "imu": {
    "yaw_scale_error": 0, "yaw_drift_rad_s": 0, "gyro_bias_rad_s": 0, "gyro_noise_rad_s": 0,
    "angle_noise_rad": 0, "seed": 0, "read_cost_ms": 0
  },
  "odometry": {
    "latency_ms": 0, "linear_scale_error": 0, "heading_scale_error": 0, "heading_drift_rad_s": 0,
    "position_noise_m": 0, "heading_noise_rad": 0, "seed": 0, "read_cost_ms": 0
  },
  "motor_defaults": {
    "rotor_inertia_kg_m2": 0.0015, "static_friction_nm": 0, "viscous_friction_nm_s_per_rad": 0,
    "thermal_threshold_fraction": 0.7, "thermal_time_constant_s": 20, "thermal_max_derate": 0.3,
    "velocity_p_gain": 1.0, "position_p_gain": 0.01
  },
  "motor_overrides": { "arm_motor": { "viscous_friction_nm_s_per_rad": 0.05 } }
}
```

## Battery

Every tick one shared voltage is solved for all motors with R4's closed-form duty-averaged model, plus constant non-motor
current: `V = Voc - R * (I_motors + I_loads)`. Motor torque, the hub `VoltageSensor` and `LynxModule` all see it.

- **State of charge** (`capacity_mah > 0`): battery current is integrated, and the open-circuit voltage falls as
  `empty + (full - empty) * charge^curve_shape`, where `full` is `internal_voltage_v` (or the calibrated value). `0` keeps
  the voltage constant. The curve shape is a placeholder, not a measured NiMH discharge curve.
- **Resistance rise** (`empty_resistance_ohm >= 0`, with charge tracking on): internal resistance rises linearly from
  `internal_resistance_ohm` at full charge to this value when empty, so a drained pack sags harder. `-1` keeps it constant.
- **Brownout** (`brownout_voltage_v > 0`): if the terminal voltage drops below it, the hub resets: motor drivers are cut (the
  motors float and draw no current) for at least `brownout_hold_s` and until the open-circuit voltage is back above
  `brownout_recovery_voltage_v` (default threshold + 0.5 V). OpModes keep running, which a real hub restart would not do.
  The real threshold and reboot time are not established here; tune them to your hub.
- **Loads**: `hub_baseline_a` is a constant draw; each commanded servo draws `servo_hold_a`, and `servo_active_a` for
  `servo_active_duration_ms` after its position changes. A continuous-rotation servo draws `servo_active_a` scaled by `|power|`.
  Currents are battery-side amps.

## Latency and bus cost

- `encoder_latency_ms` delays `getCurrentPosition()` and `getVelocity()` (interpolated between ticks, so values below the
  20 ms tick are honoured). `bus.voltage_latency_ms` delays the hub voltage; `bus.current_latency_ms` delays motor current.
- **Bulk caching** is modelled per hub (hubs come from the `<LynxModule>` each device sits under). `OFF`: every read is a bus
  transaction. `AUTO`: reads share one snapshot until the same value of the same motor is read again. `MANUAL`: reads share one
  snapshot until `clearBulkCache()`; forgetting to clear it returns stale values, as on a real hub.
- `bus.read_cost_ms` blocks the calling thread for that long per bus transaction, so uncached loops get slower and bulk
  caching pays off. `SimVoltageSensor.busTransactions()` counts them. Real hub round trips are a few milliseconds; the
  default `0` leaves reads free.
- `bus.velocity_window_ms` (0 = instantaneous): the hub velocity is differenced from whole encoder counts over this window, so
  it is quantized (one count per window) and smoothed, as on a real hub. `0` reports the exact shaft speed.
- **IMU errors** (`imu`): `yaw_scale_error` (fraction of rotation over/under-read), `yaw_drift_rad_s` (since the last
  `resetYaw()`), `gyro_bias_rad_s` and `gyro_noise_rad_s` (all three rate axes), `angle_noise_rad` (yaw/pitch/roll), a repeatable
  `seed`, and `read_cost_ms`, which blocks each orientation or rate read (the physics thread is never blocked). Latency stays
  `imu_latency_ms`. Drift and noise need the simulator clock, so they apply in the 3D and native-physics runs.
- Pinpoint/OTOS: `odometry.read_cost_ms` blocks each Pinpoint `update()` and OTOS read (one transaction per block read).
  `odometry.latency_ms` delays the reported pose (interpolated). `linear_scale_error` and
  `heading_scale_error` scale reported distance and heading change, `heading_drift_rad_s` adds drift since the last reset,
  and `position_noise_m` / `heading_noise_rad` add zero-mean Gaussian noise per pose update. `seed` makes noisy runs repeat;
  each device mixes in its own name. Latency and drift need the simulator clock, so they apply in the 3D and native-physics
  runs, not in the legacy motor-only headless run.
- Scene distance, color, touch and camera sensors have their own `latency_ms`/noise settings in the scene sensor config.

## Motor placeholders

`motor_defaults` and per-motor `motor_overrides` expose the values that were hard-coded placeholders: rotor inertia,
Coulomb/viscous friction, the thermal-derating heuristic and the `RUN_USING_ENCODER` / `RUN_TO_POSITION` P gains. Overrides
beat defaults field by field; a name that is not a configured motor is an error. A team's own `setPIDFCoefficients` call
still replaces the gains at run time. **Free-spinning motors draw almost no current once up to speed**, so sustained sag needs
friction or a physical load; without them the sag is only a spin-up transient.

## Also fixed here

A motor on `ZeroPowerBehavior.FLOAT` (or with its driver cut by a brownout) used to be braked by its back-EMF as if
`BRAKE`; it now coasts and draws no current.

## Not modeled

Temperature effects on the pack, cell imbalance, regenerative current flowing back into the battery, bus cost for
distance/color/touch sensors, and real hub brownout behavior. Nothing here is checked against a robot recording: calibrate
against your hardware before trusting the numbers.
