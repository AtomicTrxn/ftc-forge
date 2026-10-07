# Parameter calibration pass

Baseline: `e104a90`. Fits the configurable parameters from a recorded log instead of tuning them by hand.

## Stages (all in `physics.calibration.ParameterCalibrator`, orchestrated by `simrunner.CalibrationFitter`)

| Stage | Method | Refuses when |
|---|---|---|
| Battery depletion + resistance rise | OLS of V on [1, I, Q, I*Q], Q = charge drawn | under 2% of the pack drawn, or current/charge not varied |
| Inertia + friction | coarse-to-fine grid, one-step-ahead (R5's method) | doubling/halving inertia barely changes the error |
| Velocity window | window that best reproduces reported velocity from reported counts | no window clearly beats the rest |
| IMU / odometry | lag search with pooled scale (+ drift) regression against a reference | reference does not move, or is collinear with time (drift dropped with a note) |
| Gyro bias/noise | stationary stretches over the whole latency window | fewer than 30 samples |

Results go into an `electrical` block of the calibration profile in `sim.config`'s schema, validated at load, overlaying sim.config
setting by setting (`SimConfig.applyElectrical(map, projectRoot)`); `imu_latency_ms` moved into `ElectricalConfig` so it can be overlaid.
Log schema gains optional IMU/odometry reading-versus-reference columns.

## Not fitted

Encoder/voltage/current latency, read cost, loads, brownout, thermal, seeds: a motor log has no reference for them.

## Validation

Closed-loop only: the simulator runs with known parameters, a log is recorded, the fit recovers them (`CalibrationPassTest`).
No real robot recording exists, so hardware agreement is unmeasured.
