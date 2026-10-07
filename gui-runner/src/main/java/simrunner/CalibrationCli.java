package simrunner;

import physics.MotorSpec;
import physics.calibration.Calibrator;
import physics.calibration.TelemetryLogReader;
import physics.calibration.TelemetryLogRow;
import simcore.PresetRobotConfig;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/** Fits and writes an opt-in simulator profile from actual telemetry CSV files. */
public final class CalibrationCli {
    private CalibrationCli() { }

    public static void main(String[] args) throws Exception {
        // Optional trailing key=value settings select the extra fit stages (see CALIBRATION.md).
        java.util.Map<String, String> options = new java.util.LinkedHashMap<>();
        List<String> positional = new ArrayList<>();
        for (String arg : args) {
            int eq = arg.indexOf('=');
            if (eq > 0 && arg.substring(0, eq).matches("[a-z_]+")) options.put(arg.substring(0, eq), arg.substring(eq + 1));
            else positional.add(arg);
        }
        args = positional.toArray(String[]::new);
        for (String key : options.keySet())
            if (!java.util.Set.of("capacity_mah", "fit_inertia", "fit_velocity_window", "sensor_log").contains(key))
                throw new IllegalArgumentException("Unknown option " + key + "; expected capacity_mah, fit_inertia, fit_velocity_window or sensor_log");
        if (args.length != 7 && args.length != 8) {
            System.err.println("Usage: CalibrationCli <teamDir> <unloadedMotorLog.csv> <output.json> "
                + "<measuredMotorLoadInertiaKgM2> <wheelRadiusM> <trackWidthM> <wheelBaseM> [driveLog.csv] "
                + "[capacity_mah=N] [fit_inertia=true] [fit_velocity_window=true] [sensor_log=file.csv]");
            System.exit(2);
        }
        Path team = Path.of(args[0]);
        SimConfig config = SimConfig.load(team);
        PresetRobotConfig preset = PresetRobotConfig.load(team.resolve(config.presetMotors));
        List<TelemetryLogRow> motorRows = TelemetryLogReader.read(team.resolve(args[1]));
        double inertia = Double.parseDouble(args[3]);
        if (!Double.isFinite(inertia) || inertia <= 0) throw new IllegalArgumentException("Measured motor load inertia must be positive");
        CalibrationFitter extra = new CalibrationFitter();
        var battery = Calibrator.fitBattery(motorRows);
        double batteryError = 0;
        for (TelemetryLogRow row : motorRows) {
            double railCurrent = row.motors.values().stream()
                .mapToDouble(m -> Math.abs(m.power) * Math.abs(m.currentA)).sum();
            batteryError += Math.pow(row.batteryVoltageV - (battery.vInternal - battery.rBattery * railCurrent), 2);
        }
        double batteryRmse = Math.sqrt(batteryError / motorRows.size());
        if (batteryRmse > .5)
            throw new IllegalArgumentException("Battery fit RMSE is too high (" + batteryRmse + " V); check current/power data");
        StringBuilder json = new StringBuilder();
        json.append("{\n  \"battery\": {\"v_internal\": ").append(battery.vInternal)
            .append(", \"r_battery\": ").append(battery.rBattery).append("},\n  \"motors\": {");
        List<String> fitted = new ArrayList<>();
        for (var entry : preset.motors.entrySet()) {
            MotorSpec spec = entry.getValue();
            List<Double> velocities = motorRows.stream().map(row -> row.motors.get(entry.getKey()))
                .filter(java.util.Objects::nonNull).map(m -> m.velTps).toList();
            double maxPower = motorRows.stream().map(row -> row.motors.get(entry.getKey()))
                .filter(java.util.Objects::nonNull).mapToDouble(m -> Math.abs(m.power)).max().orElse(0);
            if (velocities.size() < 30 || maxPower < .2
                || velocities.stream().mapToDouble(Double::doubleValue).max().orElse(0)
                    - velocities.stream().mapToDouble(Double::doubleValue).min().orElse(0) < 50) continue;
            double[] jointFriction = "true".equals(options.get("fit_inertia")) ? extra.fitInertia(motorRows, entry.getKey(), spec) : null;
            var fit = Calibrator.fitMotorFriction(motorRows, entry.getKey(), spec, inertia);
            double rmse = Math.sqrt(fit.residualSumSquares / (velocities.size() - 1));
            if (rmse > spec.omegaNoLoadRadS * .5)
                throw new IllegalArgumentException("Motor fit poor for " + entry.getKey()
                    + " (" + rmse + " rad/s); verify unloaded test and inertia");
            if (!fitted.isEmpty()) json.append(',');
            double staticNm = jointFriction != null ? jointFriction[0] : fit.tauStaticNm;
            double viscous = jointFriction != null ? jointFriction[1] : fit.viscousBNms;
            json.append("\n    \"").append(escaped(entry.getKey())).append("\": {\"tau_static_nm\": ")
                .append(staticNm).append(", \"viscous_b_nm_s_per_rad\": ")
                .append(viscous).append('}');
            fitted.add(entry.getKey());
        }
        if (fitted.isEmpty()) throw new IllegalArgumentException("No motors have enough speed/power variation to fit");
        json.append("\n  }");
        int driveSamples = 0;
        if (args.length == 8) {
            List<TelemetryLogRow> driveRows = TelemetryLogReader.read(team.resolve(args[7]));
            var fit = Calibrator.fitDrive(driveRows, preset.motors, Double.parseDouble(args[4]),
                Double.parseDouble(args[5]), Double.parseDouble(args[6]));
            driveSamples = fit.samples();
            json.append(",\n  \"drive\": {\"response_time_s\": ").append(fit.responseTimeS())
                .append(", \"max_accel_mps2\": ").append(fit.maxAccelMps2())
                .append(", \"yaw_response_time_s\": ").append(fit.yawResponseTimeS())
                .append(", \"max_yaw_accel_radps2\": ").append(fit.maxYawAccelRadps2())
                .append('}');
            System.out.printf(Locale.ROOT, "[CALIBRATION] Drive RMSE: %.4f m/s, %.4f rad/s over %d pairs%n",
                fit.translationRmseMps(), fit.yawRmseRadps(), driveSamples);
        }
        if (options.containsKey("capacity_mah"))
            extra.fitDepletion(motorRows, Double.parseDouble(options.get("capacity_mah")));
        if ("true".equals(options.get("fit_velocity_window"))) extra.fitVelocityWindow(motorRows, fitted);
        if (options.containsKey("sensor_log")) extra.fitSensors(TelemetryLogReader.read(team.resolve(options.get("sensor_log"))));
        extra.validate();
        if (!extra.electrical.isEmpty()) json.append(",\n  \"electrical\": ").append(ProfileIO.json(extra.electrical));
        for (String note : extra.notes) System.out.println("[CALIBRATION] Note: " + note);
        json.append(",\n  \"fit_notes\": ").append(ProfileIO.json(extra.notes));
        if (!extra.evidence.isEmpty()) json.append(",\n  \"fit_evidence\": ").append(ProfileIO.json(extra.evidence));
        json.append(",\n  \"evidence\": {\"motor_rows\": ").append(motorRows.size())
            .append(", \"motor_load_inertia_kg_m2\": ").append(inertia)
            .append(", \"battery_rmse_v\": ").append(batteryRmse)
            .append(", \"drive_pairs\": ").append(driveSamples).append("}\n}\n");
        Path output = team.resolve(args[2]);
        Files.writeString(output, json.toString());
        CalibrationProfile.load(output); // validate the exact artifact that will be consumed
        System.out.println("[CALIBRATION] Wrote " + output + " using " + motorRows.size()
            + " recorded motor rows; fitted " + fitted);
    }

    private static String escaped(String text) {
        return text.replace("\\", "\\\\").replace("\"", "\\\"");
    }
}
