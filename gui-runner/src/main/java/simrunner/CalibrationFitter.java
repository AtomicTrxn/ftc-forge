package simrunner;

import physics.MotorSpec;
import physics.calibration.ParameterCalibrator;
import physics.calibration.TelemetryLogRow;
import simcore.ElectricalConfig;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeSet;

/**
 * Runs the optional fit stages (battery depletion, inertia, velocity window, IMU and odometry errors)
 * and turns the results into an {@code electrical} block in sim.config's own schema, so a calibration
 * profile overlays exactly the settings a person would otherwise tune by hand. A stage whose data
 * cannot support it is skipped with a note; nothing is guessed. Fitted values outside the range
 * sim.config accepts are not applied.
 */
final class CalibrationFitter {
    final Map<String, Object> electrical = new LinkedHashMap<>();
    final List<String> notes = new ArrayList<>();
    final Map<String, Object> evidence = new LinkedHashMap<>();
    /** Motor friction fitted together with inertia, by motor name: {static, viscous}. */
    final Map<String, double[]> friction = new LinkedHashMap<>();

    private static final int DEFAULT_MAX_LAG_MS = 200;

    void fitDepletion(List<TelemetryLogRow> rows, double capacityMah) {
        try {
            var fit = ParameterCalibrator.fitBatteryDepletion(rows, capacityMah / 1000.0);
            put("battery", "capacity_mah", capacityMah, 0, 100000);
            put("battery", "internal_voltage_v", fit.fullVoltageV(), 6, 18);
            put("battery", "internal_resistance_ohm", fit.baseResistanceOhm(), 0, 2);
            put("battery", "empty_voltage_v", fit.emptyVoltageV(), 0, 18);
            put("battery", "empty_resistance_ohm", fit.emptyResistanceOhm(), 0, 2);
            evidence.put("depletion", Map.of("charge_drawn_ah", fit.chargeDrawnAh(), "rmse_v", fit.rmseV(),
                "volts_per_amp_hour", fit.voltsPerAmpHour()));
            notes.add("Battery depletion assumes a linear discharge curve (curve_shape 1) and that the pack capacity you supplied is usable; "
                + "the fitted full voltage includes any constant hub/servo load because that current is not in the log.");
        } catch (IllegalArgumentException e) {
            notes.add("Battery depletion not fitted: " + e.getMessage());
        }
    }

    /** Returns the fitted friction for the motor, or null when inertia was not identifiable. */
    double[] fitInertia(List<TelemetryLogRow> rows, String motor, MotorSpec spec) {
        try {
            var fit = ParameterCalibrator.fitMotorInertiaAndFriction(rows, motor, spec, 0.0003, 0.02);
            overrides().computeIfAbsent(motor, k -> new LinkedHashMap<>());
            @SuppressWarnings("unchecked") Map<String, Object> entry = (Map<String, Object>) overrides().get(motor);
            if (fit.inertiaKgM2() >= 1e-7 && fit.inertiaKgM2() <= 10) entry.put("rotor_inertia_kg_m2", fit.inertiaKgM2());
            evidence.put("inertia_" + motor, Map.of("inertia_kg_m2", fit.inertiaKgM2(), "rmse_rad_s", fit.rmseRadS()));
            double[] f = {fit.tauStaticNm(), fit.viscousBNms()};
            friction.put(motor, f);
            return f;
        } catch (IllegalArgumentException e) {
            notes.add("Inertia not fitted for " + motor + ": " + e.getMessage());
            return null;
        }
    }

    void fitVelocityWindow(List<TelemetryLogRow> rows, Iterable<String> motors) {
        TreeSet<Double> windows = new TreeSet<>();
        List<Double> all = new ArrayList<>();
        for (String motor : motors) {
            try {
                var fit = ParameterCalibrator.fitVelocityWindow(rows, motor);
                all.add(fit.windowMs());
                evidence.put("velocity_window_" + motor, Map.of("window_ms", fit.windowMs(), "rmse_tps", fit.rmseTps()));
            } catch (IllegalArgumentException e) {
                notes.add("Velocity window not fitted for " + motor + ": " + e.getMessage());
            }
        }
        if (all.isEmpty()) return;
        all.sort(Double::compare);
        put("bus", "velocity_window_ms", all.get(all.size() / 2), 0, 200);   // median across motors
    }

    void fitSensors(List<TelemetryLogRow> rows) {
        var imu = ParameterCalibrator.fitImu(rows, DEFAULT_MAX_LAG_MS);
        notes.addAll(imu.notes());
        if (imu.latencyMs() != null) {
            put(null, "imu_latency_ms", imu.latencyMs(), 0, 200);
            put("imu", "yaw_scale_error", imu.yawScaleError(), -0.5, 0.5);
            put("imu", "yaw_drift_rad_s", imu.yawDriftRadS(), -0.1, 0.1);
            put("imu", "angle_noise_rad", imu.angleNoiseRad(), 0, 0.1);
        }
        put("imu", "gyro_bias_rad_s", imu.gyroBiasRadS(), -0.5, 0.5);
        put("imu", "gyro_noise_rad_s", imu.gyroNoiseRadS(), 0, 0.5);

        var odo = ParameterCalibrator.fitOdometry(rows, DEFAULT_MAX_LAG_MS);
        notes.addAll(odo.notes());
        put("odometry", "latency_ms", odo.latencyMs(), 0, 200);
        put("odometry", "linear_scale_error", odo.linearScaleError(), -0.5, 0.5);
        put("odometry", "heading_scale_error", odo.headingScaleError(), -0.5, 0.5);
        put("odometry", "heading_drift_rad_s", odo.headingDriftRadS(), -0.1, 0.1);
        put("odometry", "position_noise_m", odo.positionNoiseM(), 0, 0.05);
        put("odometry", "heading_noise_rad", odo.headingNoiseRad(), 0, 0.1);
        if (imu.latencyMs() != null || odo.latencyMs() != null)
            notes.add("Fitted noise values are the residual spread after alignment, so they include any unmodeled error and quantization.");
    }

    @SuppressWarnings("unchecked")
    private Map<String, Object> overrides() {
        return (Map<String, Object>) electrical.computeIfAbsent("motor_overrides", k -> new LinkedHashMap<String, Object>());
    }

    /** Whole-millisecond settings are rounded; out-of-range values are dropped with a note. */
    @SuppressWarnings("unchecked")
    private void put(String block, String key, Double value, double low, double high) {
        if (value == null) return;
        double v = key.endsWith("_ms") && !key.equals("read_cost_ms") ? Math.rint(value) : value;
        if (!Double.isFinite(v) || v < low || v > high) {
            notes.add(String.format("%s%s fitted to %.6g, outside the accepted range %.3g..%.3g; not applied",
                block == null ? "" : block + ".", key, value, low, high));
            return;
        }
        Map<String, Object> target = electrical;
        if (block != null) target = (Map<String, Object>) electrical.computeIfAbsent(block, k -> new LinkedHashMap<String, Object>());
        target.put(key, key.endsWith("_ms") && !key.equals("read_cost_ms") && !key.equals("capacity_mah") ? (Object) (long) v : (Object) v);
    }

    /** Validates the assembled block exactly as the simulator will read it. */
    void validate() {
        ElectricalConfig.parse(electrical);
    }
}
