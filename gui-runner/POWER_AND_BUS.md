# Battery sag and bus latency

## Battery sag

Every simulator tick solves one shared battery voltage for all motors with R4's closed-form duty-averaged model
(`V = (Vint + R*B) / (1 + R*A)`), then each motor's torque uses that voltage. The hub (`LynxModule` / `VoltageSensor`)
reports the same value, so voltage-compensated feed-forward sees the sag.

Defaults are estimates, not measurements: 12.6 V open circuit, 0.15 ohm. Set your pack in `sim.config`:

```json
{
  "battery": { "internal_voltage_v": 12.9, "internal_resistance_ohm": 0.12 },
  "encoder_latency_ms": 8,
  "imu_latency_ms": 8
}
```

Ranges: voltage 6-18 V, resistance 0-2 ohm, latencies 0-200 ms. Unknown battery keys are rejected. A calibration profile
(`"calibration"`) is applied afterwards and wins over `battery`, because it comes from measurements.

## Encoder and velocity latency

`getCurrentPosition()` and `getVelocity()` return the value the shaft had `encoder_latency_ms` ago, interpolated between
simulator ticks. Before this change a latency shorter than the 20 ms tick was rounded up to a whole tick (a configured 8 ms
behaved as 20 ms; `EncoderLatencyTest` reproduces and guards it). Position and velocity share one latency because the real
hub returns them in the same bulk read. `encoder_latency_ms` applies to every motor.

## What is not modeled

- **Sustained sag needs load.** Free-spinning drive motors in headless runs draw current only while accelerating, because
  motor rotor inertia is a small placeholder and default friction is zero, so voltage recovers within tens of milliseconds.
  Sag under real robot load needs a calibration profile (friction) or the 3D renderer's physical load. Nothing here predicts a
  match-long voltage curve.
- Open-circuit voltage is constant: no state-of-charge droop, no temperature, no brownout cutoff. Servos and other loads
  draw no current.
- Hub voltage and motor current reads are immediate; only encoder position/velocity are delayed.
- Pinpoint and OTOS poses have no read latency or drift (the IMU does, via `imu_latency_ms`).
- Distance, color and touch sensors return constants, so their bus latency has nothing to delay.
- Latency is read-side delay only; the blocking time of an I2C round trip on the loop thread is not modeled.
- Values are checked by synthetic tests, not against a robot. Measure your pack and fit it with the
  [calibration guide](CALIBRATION.md) before trusting the numbers.
