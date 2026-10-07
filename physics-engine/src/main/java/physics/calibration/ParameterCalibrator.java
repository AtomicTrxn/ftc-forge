package physics.calibration;

import physics.MotorModel;
import physics.MotorSpec;

import java.util.ArrayList;
import java.util.List;

/**
 * Additional calibration stages for the power-system, bus and sensor-error parameters. Each stage
 * is a small, separately testable fit that refuses (throws {@link IllegalArgumentException}) when
 * the recording does not excite the parameter, rather than returning a number the data cannot
 * support. Stages follow R5's staging: closed-form where possible, one-step-ahead where a model is
 * integrated, and no stage fits more than the data can separate.
 *
 * What a recording can identify depends on what it contains: battery depletion needs a long loaded
 * run; inertia needs spin-up transients; sensor scale, drift, noise and latency need an
 * independent reference for the same quantity. Encoder, voltage and current latency have no
 * reference in a motor log and are not fitted.
 */
public final class ParameterCalibrator {
    private ParameterCalibrator() { }

    // ---- Battery depletion ------------------------------------------------------------------

    /**
     * Fit of V = a + b*I + c*Q + d*I*Q, where I is the duty-scaled rail current of the motors and Q the
     * charge drawn so far. {@code a} is the full-charge voltage seen at the motors, which also includes
     * any constant hub or servo load (their current is not in the log), so it can read lower than the
     * pack's true open-circuit voltage.
     */
    public record DepletionFit(double fullVoltageV, double baseResistanceOhm, double voltsPerAmpHour,
                               double resistanceRisePerAmpHour, double emptyVoltageV, double emptyResistanceOhm,
                               double chargeDrawnAh, double rmseV) { }

    public static DepletionFit fitBatteryDepletion(List<TelemetryLogRow> rows, double capacityAh) {
        if (!(capacityAh > 0)) throw new IllegalArgumentException("Pack capacity must be positive");
        int n = rows.size();
        if (n < 100) throw new IllegalArgumentException("Depletion fit needs at least 100 samples");
        double[] current = new double[n], charge = new double[n], voltage = new double[n];
        for (int i = 0; i < n; i++) {
            TelemetryLogRow row = rows.get(i);
            for (TelemetryLogRow.MotorSample s : row.motors.values()) current[i] += Math.abs(s.power) * Math.abs(s.currentA);
            voltage[i] = row.batteryVoltageV;
            if (i > 0) charge[i] = charge[i - 1]
                + 0.5 * (current[i] + current[i - 1]) * (row.tMs - rows.get(i - 1).tMs) / 3_600_000.0;
        }
        double drawn = charge[n - 1];
        if (drawn < 0.02 * capacityAh)
            throw new IllegalArgumentException(String.format("Depletion fit needs at least 2%% of the pack drawn (%.4f Ah of %.3f Ah); record a longer loaded run", drawn, capacityAh));
        double[][] features = new double[n][];
        for (int i = 0; i < n; i++) features[i] = new double[]{1, current[i], charge[i], current[i] * charge[i]};
        double[] beta = leastSquares(features, voltage, "Depletion fit: current and charge are not varied enough to separate resistance from depletion");
        double sse = 0;
        for (int i = 0; i < n; i++) {
            double predicted = 0;
            for (int k = 0; k < 4; k++) predicted += beta[k] * features[i][k];
            sse += (voltage[i] - predicted) * (voltage[i] - predicted);
        }
        double voltsPerAh = -beta[2], rBase = -beta[1], rRise = -beta[3];
        if (rBase < 0 || rBase > 2 || voltsPerAh < 0)
            throw new IllegalArgumentException("Depletion fit is outside physical bounds; check the voltage/current columns and that the run drained the pack");
        double emptyV = beta[0] - voltsPerAh * capacityAh;
        double emptyR = rBase + rRise * capacityAh;
        return new DepletionFit(beta[0], rBase, voltsPerAh, rRise, Math.max(0, emptyV), Math.max(rBase, Math.min(2, emptyR)),
            drawn, Math.sqrt(sse / n));
    }

    // ---- Motor inertia + friction -----------------------------------------------------------

    public record InertiaFrictionFit(double inertiaKgM2, double tauStaticNm, double viscousBNms, double rmseRadS) { }

    /**
     * One-step-ahead fit of rotor/load inertia together with Coulomb and viscous friction. Inertia
     * sets how fast the motor accelerates from a given torque, so it is only identifiable from spin-up
     * transients; the fit refuses when doubling or halving it barely changes the error.
     */
    public static InertiaFrictionFit fitMotorInertiaAndFriction(List<TelemetryLogRow> rows, String motorName, MotorSpec spec,
                                                                double minInertia, double maxInertia) {
        if (!(minInertia > 0) || !(maxInertia > minInertia)) throw new IllegalArgumentException("Inertia search range is invalid");
        if (rows.size() < 50) throw new IllegalArgumentException("Need at least 50 motor samples");
        double bestJ = 0, bestS = 0, bestV = 0, best = Double.MAX_VALUE;
        for (int i = 0; i < 30; i++) {
            double j = minInertia * Math.pow(maxInertia / minInertia, i / 29.0);
            for (double s = 0; s <= 0.4001; s += 0.04) for (double v = 0; v <= 0.0201; v += 0.002) {
                double e = oneStepError(rows, motorName, spec, j, s, v);
                if (e < best) { best = e; bestJ = j; bestS = s; bestV = v; }
            }
        }
        if (best == Double.MAX_VALUE) throw new IllegalArgumentException("No paired motor samples for " + motorName);
        double j0 = bestJ, s0 = bestS, v0 = bestV;
        for (int i = 0; i <= 20; i++) {
            double j = j0 / 1.25 * Math.pow(1.5625, i / 20.0);
            for (double s = Math.max(0, s0 - 0.04); s <= s0 + 0.0401; s += 0.005) {
                for (double v = Math.max(0, v0 - 0.002); v <= v0 + 0.00201; v += 0.0005) {
                    double e = oneStepError(rows, motorName, spec, j, s, v);
                    if (e < best) { best = e; bestJ = j; bestS = s; bestV = v; }
                }
            }
        }
        double up = oneStepError(rows, motorName, spec, bestJ * 2, bestS, bestV);
        double down = oneStepError(rows, motorName, spec, bestJ / 2, bestS, bestV);
        if (up < best * 1.1 || down < best * 1.1)
            throw new IllegalArgumentException("Inertia is not identifiable from this recording; include spin-up transients (steps or ramps in power)");
        int pairs = Math.max(1, rows.size() - 1);
        return new InertiaFrictionFit(bestJ, bestS, bestV, Math.sqrt(best / pairs));
    }

    private static double oneStepError(List<TelemetryLogRow> rows, String motor, MotorSpec spec, double inertia,
                                       double tauStatic, double viscous) {
        MotorModel model = new MotorModel();
        model.tauStaticNm = tauStatic;
        model.viscousBNms = viscous;
        double error = 0;
        for (int i = 0; i + 1 < rows.size(); i++) {
            TelemetryLogRow.MotorSample cur = rows.get(i).motors.get(motor), next = rows.get(i + 1).motors.get(motor);
            if (cur == null || next == null) continue;
            double dt = (rows.get(i + 1).tMs - rows.get(i).tMs) / 1000.0;
            if (dt <= 0) continue;
            double omega = cur.velTps / spec.encoderCountsPerRev * 2 * Math.PI;
            double tau = model.torque(spec, omega, cur.power * rows.get(i).batteryVoltageV);
            double predicted = omega + tau / inertia * dt;
            double actual = next.velTps / spec.encoderCountsPerRev * 2 * Math.PI;
            error += (predicted - actual) * (predicted - actual);
        }
        return error;
    }

    // ---- Velocity window --------------------------------------------------------------------

    public record VelocityWindowFit(double windowMs, double rmseTps) { }

    /**
     * Finds the window over which the hub's reported velocity matches a difference of its own reported
     * encoder counts. Refuses when no window fits clearly better than the others (constant speed).
     */
    public static VelocityWindowFit fitVelocityWindow(List<TelemetryLogRow> rows, String motorName) {
        int n = rows.size();
        double[] t = new double[n], ticks = new double[n], vel = new double[n];
        for (int i = 0; i < n; i++) {
            TelemetryLogRow.MotorSample s = rows.get(i).motors.get(motorName);
            if (s == null) throw new IllegalArgumentException("No samples for motor " + motorName);
            t[i] = rows.get(i).tMs;
            ticks[i] = s.ticks;
            vel[i] = s.velTps;
        }
        double bestW = 0, best = Double.MAX_VALUE, worst = 0;
        for (int w = 10; w <= 200; w += 10) {
            double sse = 0;
            int count = 0;
            for (int i = 0; i < n; i++) {
                if (t[i] - w < t[0]) continue;
                double older = interpolate(t, ticks, t[i] - w);
                double predicted = (ticks[i] - older) * 1000.0 / w;
                sse += (predicted - vel[i]) * (predicted - vel[i]);
                count++;
            }
            if (count < 30) continue;
            double rmse = Math.sqrt(sse / count);
            worst = Math.max(worst, rmse);
            if (rmse < best) { best = rmse; bestW = w; }
        }
        if (best == Double.MAX_VALUE) throw new IllegalArgumentException("Velocity-window fit needs a longer recording");
        if (best > 0.8 * worst) throw new IllegalArgumentException("Velocity window is not identifiable; the motor speed barely changes in this recording");
        return new VelocityWindowFit(bestW, best);
    }

    // ---- Sensor alignment: latency, scale, drift, noise ------------------------------------

    /** Result of aligning a measured series to its reference: meas = c + (1 + scaleError) * ref(t - lag) + drift * t. */
    public record Alignment(double lagMs, double scaleError, double driftPerS, double noiseStd, int samples) { }

    /**
     * Aligns one or more channels sharing a lag and a scale. Each channel has its own offset; a single
     * drift term (per second) is fitted only when {@code drift} is true. Refuses ill-conditioned problems.
     */
    public static Alignment align(double[] tMs, List<double[]> refs, List<double[]> meas, int maxLagMs, boolean drift) {
        int total = tMs.length, channels = refs.size();
        if (channels == 0 || meas.size() != channels) throw new IllegalArgumentException("Alignment needs matching channels");
        List<Integer> use = new ArrayList<>();
        for (int i = 0; i < total; i++) if (tMs[i] - maxLagMs >= tMs[0]) use.add(i);
        int n = use.size();
        if (n < 40) throw new IllegalArgumentException("Alignment needs at least 40 samples beyond the maximum lag");
        double bestSse = Double.MAX_VALUE, bestLag = 0, bestS = 0, bestD = 0;
        for (int lag = 0; lag <= maxLagMs; lag++) {
            double sxx = 0, sxy = 0, sxt = 0, stt = 0, sty = 0, syy = 0;
            for (int k = 0; k < channels; k++) {
                double[] x = new double[n], y = new double[n], tt = new double[n];
                double mx = 0, my = 0, mt = 0;
                for (int idx = 0; idx < n; idx++) {
                    int i = use.get(idx);
                    x[idx] = interpolate(tMs, refs.get(k), tMs[i] - lag);
                    y[idx] = meas.get(k)[i];
                    tt[idx] = (tMs[i] - tMs[0]) / 1000.0;
                    mx += x[idx]; my += y[idx]; mt += tt[idx];
                }
                mx /= n; my /= n; mt /= n;
                for (int idx = 0; idx < n; idx++) {
                    double dx = x[idx] - mx, dy = y[idx] - my, dt = tt[idx] - mt;
                    sxx += dx * dx; sxy += dx * dy; sxt += dx * dt; stt += dt * dt; sty += dt * dy; syy += dy * dy;
                }
            }
            double s, d = 0;
            if (drift) {
                double det = sxx * stt - sxt * sxt;
                if (det <= 1e-9 * sxx * stt || sxx <= 1e-12) continue;
                s = (sxy * stt - sty * sxt) / det;
                d = (sty * sxx - sxy * sxt) / det;
            } else {
                if (sxx <= 1e-12) continue;
                s = sxy / sxx;
            }
            double sse = syy - 2 * s * sxy - 2 * d * sty + s * s * sxx + 2 * s * d * sxt + d * d * stt;
            if (sse < bestSse) { bestSse = sse; bestLag = lag; bestS = s; bestD = d; }
        }
        if (bestSse == Double.MAX_VALUE)
            throw new IllegalArgumentException("Reference does not vary enough (or is collinear with time) to align this sensor");
        int params = channels + 1 + (drift ? 1 : 0);
        return new Alignment(bestLag, bestS - 1, bestD, Math.sqrt(Math.max(0, bestSse) / Math.max(1, n * channels - params)), n);
    }

    // ---- IMU --------------------------------------------------------------------------------

    public record ImuFit(Double latencyMs, Double yawScaleError, Double yawDriftRadS, Double angleNoiseRad,
                         Double gyroBiasRadS, Double gyroNoiseRadS, List<String> notes) { }

    /**
     * Yaw latency/scale/drift/noise from paired IMU and reference yaw, and gyro bias/noise from the
     * stationary stretches (reference rate near zero). Parts the data cannot support are left null with a note.
     */
    public static ImuFit fitImu(List<TelemetryLogRow> rows, int maxLagMs) {
        List<String> notes = new ArrayList<>();
        Double lag = null, scale = null, drift = null, noise = null, bias = null, gyroNoise = null;
        List<TelemetryLogRow> yawRows = rows.stream().filter(r -> r.sensors != null && r.sensors.imuYawRad() != null).toList();
        if (yawRows.size() >= 80) {
            double[] t = yawRows.stream().mapToDouble(r -> r.tMs).toArray();
            double[] ref = unwrap(yawRows.stream().mapToDouble(r -> r.sensors.refYawRad()).toArray());
            double[] imu = unwrap(yawRows.stream().mapToDouble(r -> r.sensors.imuYawRad()).toArray());
            if (range(ref) < 0.5) {
                notes.add("IMU yaw not fitted: the reference turns less than 0.5 rad");
            } else {
                Alignment a;
                boolean driftFitted = true;
                try {
                    a = align(t, List.of(ref), List.of(imu), maxLagMs, true);
                } catch (IllegalArgumentException collinear) {
                    a = align(t, List.of(ref), List.of(imu), maxLagMs, false);
                    driftFitted = false;
                    notes.add("IMU yaw drift not fitted: the reference yaw moves proportionally to time, so drift and scale are inseparable");
                }
                lag = a.lagMs();
                scale = a.scaleError();
                drift = driftFitted ? a.driftPerS() : null;
                noise = a.noiseStd();
            }
        } else if (!yawRows.isEmpty()) {
            notes.add("IMU yaw not fitted: only " + yawRows.size() + " paired samples (need 80)");
        }
        // Stationary means the reference rate stayed near zero for the whole latency window, so a delayed
        // reading cannot still be reporting earlier motion.
        List<TelemetryLogRow> still = new ArrayList<>();
        for (int i = 0; i < rows.size(); i++) {
            TelemetryLogRow row = rows.get(i);
            if (row.sensors == null || row.sensors.imuRateRadS() == null) continue;
            boolean quiet = true;
            for (int j = i; j >= 0 && rows.get(j).tMs >= row.tMs - maxLagMs && quiet; j--) {
                var other = rows.get(j).sensors;
                quiet = other != null && other.refRateRadS() != null && Math.abs(other.refRateRadS()) < 0.02;
            }
            if (quiet && rows.get(0).tMs <= row.tMs - maxLagMs) still.add(row);
        }
        if (still.size() >= 30) {
            double mean = still.stream().mapToDouble(r -> r.sensors.imuRateRadS() - r.sensors.refRateRadS()).average().orElse(0);
            double ss = still.stream().mapToDouble(r -> Math.pow(r.sensors.imuRateRadS() - r.sensors.refRateRadS() - mean, 2)).sum();
            bias = mean;
            gyroNoise = Math.sqrt(ss / (still.size() - 1));
        } else if (rows.stream().anyMatch(r -> r.sensors != null && r.sensors.imuRateRadS() != null)) {
            notes.add("Gyro bias/noise not fitted: fewer than 30 stationary samples");
        }
        return new ImuFit(lag, scale, drift, noise, bias, gyroNoise, notes);
    }

    // ---- Odometry ---------------------------------------------------------------------------

    public record OdometryFit(Double latencyMs, Double linearScaleError, Double headingScaleError, Double headingDriftRadS,
                              Double positionNoiseM, Double headingNoiseRad, List<String> notes) { }

    public static OdometryFit fitOdometry(List<TelemetryLogRow> rows, int maxLagMs) {
        List<String> notes = new ArrayList<>();
        List<TelemetryLogRow> usable = rows.stream().filter(r -> r.sensors != null && r.sensors.odoXM() != null).toList();
        if (usable.isEmpty()) return new OdometryFit(null, null, null, null, null, null, notes);
        if (usable.size() < 80) {
            notes.add("Odometry not fitted: only " + usable.size() + " paired samples (need 80)");
            return new OdometryFit(null, null, null, null, null, null, notes);
        }
        double[] t = usable.stream().mapToDouble(r -> r.tMs).toArray();
        double[] rx = usable.stream().mapToDouble(r -> r.sensors.refXM()).toArray(), ry = usable.stream().mapToDouble(r -> r.sensors.refYM()).toArray();
        double[] mx = usable.stream().mapToDouble(r -> r.sensors.odoXM()).toArray(), my = usable.stream().mapToDouble(r -> r.sensors.odoYM()).toArray();
        double[] rh = unwrap(usable.stream().mapToDouble(r -> r.sensors.refHeadingRad()).toArray());
        double[] mh = unwrap(usable.stream().mapToDouble(r -> r.sensors.odoHeadingRad()).toArray());
        Double lag = null, linear = null, headingScale = null, drift = null, posNoise = null, headNoise = null;
        Alignment position = null;
        if (Math.max(range(rx), range(ry)) >= 0.3) {
            position = align(t, List.of(rx, ry), List.of(mx, my), maxLagMs, false);
            lag = position.lagMs();
            linear = position.scaleError();
            posNoise = position.noiseStd();
        } else {
            notes.add("Odometry position not fitted: the reference moves less than 0.3 m");
        }
        if (range(rh) >= 0.5) {
            Alignment heading;
            try {
                heading = align(t, List.of(rh), List.of(mh), maxLagMs, true);
                drift = heading.driftPerS();
            } catch (IllegalArgumentException collinear) {
                heading = align(t, List.of(rh), List.of(mh), maxLagMs, false);
                notes.add("Odometry heading drift not fitted: reference heading moves proportionally to time");
            }
            headingScale = heading.scaleError();
            headNoise = heading.noiseStd();
            if (lag == null) lag = heading.lagMs();
        } else {
            notes.add("Odometry heading not fitted: the reference turns less than 0.5 rad");
        }
        return new OdometryFit(lag, linear, headingScale, drift, posNoise, headNoise, notes);
    }

    // ---- helpers ----------------------------------------------------------------------------

    static double[] unwrap(double[] angles) {
        double[] out = new double[angles.length];
        if (angles.length == 0) return out;
        out[0] = angles[0];
        for (int i = 1; i < angles.length; i++) {
            double step = angles[i] - angles[i - 1];
            step = Math.atan2(Math.sin(step), Math.cos(step));
            out[i] = out[i - 1] + step;
        }
        return out;
    }

    private static double range(double[] values) {
        double lo = Double.MAX_VALUE, hi = -Double.MAX_VALUE;
        for (double v : values) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
        return hi - lo;
    }

    /** Linear interpolation of y over ascending x; clamps outside the range. */
    static double interpolate(double[] x, double[] y, double at) {
        if (at <= x[0]) return y[0];
        int hi = java.util.Arrays.binarySearch(x, at);
        if (hi >= 0) return y[hi];
        hi = -hi - 1;
        if (hi >= x.length) return y[y.length - 1];
        double f = (at - x[hi - 1]) / (x[hi] - x[hi - 1]);
        return y[hi - 1] + (y[hi] - y[hi - 1]) * f;
    }

    /** Ordinary least squares via standardized normal equations; refuses singular systems. */
    static double[] leastSquares(double[][] features, double[] target, String singularMessage) {
        int n = features.length, k = features[0].length;
        double[] mean = new double[k], scale = new double[k];
        for (int j = 1; j < k; j++) {
            for (double[] row : features) mean[j] += row[j];
            mean[j] /= n;
            for (double[] row : features) scale[j] += (row[j] - mean[j]) * (row[j] - mean[j]);
            scale[j] = Math.sqrt(scale[j] / n);
            if (scale[j] < 1e-12) throw new IllegalArgumentException(singularMessage);
        }
        scale[0] = 1;
        double[][] a = new double[k][k + 1];
        for (int r = 0; r < n; r++) {
            double[] z = new double[k];
            z[0] = 1;
            for (int j = 1; j < k; j++) z[j] = (features[r][j] - mean[j]) / scale[j];
            for (int i = 0; i < k; i++) {
                for (int j = 0; j < k; j++) a[i][j] += z[i] * z[j];
                a[i][k] += z[i] * target[r];
            }
        }
        for (int c = 0; c < k; c++) {
            int pivot = c;
            for (int r = c + 1; r < k; r++) if (Math.abs(a[r][c]) > Math.abs(a[pivot][c])) pivot = r;
            if (Math.abs(a[pivot][c]) < 1e-9 * n) throw new IllegalArgumentException(singularMessage);
            double[] swap = a[c]; a[c] = a[pivot]; a[pivot] = swap;
            for (int r = 0; r < k; r++) {
                if (r == c) continue;
                double f = a[r][c] / a[c][c];
                for (int j = c; j <= k; j++) a[r][j] -= f * a[c][j];
            }
        }
        double[] z = new double[k];
        for (int i = 0; i < k; i++) z[i] = a[i][k] / a[i][i];
        double[] beta = new double[k];
        double intercept = z[0];
        for (int j = 1; j < k; j++) {
            beta[j] = z[j] / scale[j];
            intercept -= beta[j] * mean[j];
        }
        beta[0] = intercept;
        return beta;
    }
}
