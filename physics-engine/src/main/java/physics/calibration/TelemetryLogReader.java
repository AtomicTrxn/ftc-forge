package physics.calibration;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/** Validated CSV reader for recorded FTC motor and optional chassis telemetry. */
public class TelemetryLogReader {
    private static final Pattern MOTOR_POWER_COL = Pattern.compile("^motor_(.+)_power$");

    public static List<TelemetryLogRow> read(Path path) throws IOException {
        List<String> lines = Files.readAllLines(path);
        if (lines.isEmpty()) throw new IllegalArgumentException("Telemetry CSV is empty");
        String[] header = lines.get(0).split(",", -1);
        Map<String, Integer> columns = new LinkedHashMap<>();
        for (int i = 0; i < header.length; i++) {
            if (columns.putIfAbsent(header[i].trim(), i) != null)
                throw new IllegalArgumentException("Duplicate telemetry column " + header[i]);
        }
        for (String required : List.of("t_ms", "loop_iter", "loop_time_ms", "battery_voltage_v"))
            require(columns, required);
        List<String> motorNames = new ArrayList<>();
        for (String col : columns.keySet()) {
            Matcher match = MOTOR_POWER_COL.matcher(col);
            if (match.matches()) {
                String name = match.group(1);
                for (String suffix : List.of("ticks", "vel_tps", "current_a")) require(columns, "motor_" + name + "_" + suffix);
                motorNames.add(name);
            }
        }
        if (motorNames.isEmpty()) throw new IllegalArgumentException("Telemetry CSV has no motor columns");
        boolean chassis = columns.containsKey("vx_mps") || columns.containsKey("vy_mps") || columns.containsKey("omega_rad_s");
        if (chassis) for (String name : List.of("vx_mps", "vy_mps", "omega_rad_s")) require(columns, name);
        boolean sensors = false;
        for (String name : TelemetryLogRow.SensorSample.COLUMNS) sensors |= columns.containsKey(name);
        if (sensors) for (String name : TelemetryLogRow.SensorSample.COLUMNS) require(columns, name);
        List<TelemetryLogRow> rows = new ArrayList<>();
        long previousTime = -1;
        for (int line = 1; line < lines.size(); line++) {
            if (lines.get(line).isBlank()) continue;
            String[] values = lines.get(line).split(",", -1);
            if (values.length != header.length)
                throw new IllegalArgumentException("Wrong column count on telemetry line " + (line + 1));
            TelemetryLogRow row = new TelemetryLogRow();
            try {
                row.tMs = Long.parseLong(value(values, columns, "t_ms"));
                row.loopIter = Integer.parseInt(value(values, columns, "loop_iter"));
                row.loopTimeMs = number(values, columns, "loop_time_ms");
                row.batteryVoltageV = number(values, columns, "battery_voltage_v");
                if (row.tMs <= previousTime || row.loopTimeMs < 0 || row.batteryVoltageV <= 0)
                    throw new IllegalArgumentException("Invalid time or battery voltage");
                previousTime = row.tMs;
                for (String name : motorNames) {
                    String prefix = "motor_" + name + "_";
                    double power = number(values, columns, prefix + "power");
                    long ticks = Long.parseLong(value(values, columns, prefix + "ticks"));
                    double velocity = number(values, columns, prefix + "vel_tps");
                    double current = number(values, columns, prefix + "current_a");
                    if (Math.abs(power) > 1 || current < 0) throw new IllegalArgumentException("Invalid motor power/current");
                    row.motors.put(name, new TelemetryLogRow.MotorSample(power, ticks, velocity, current));
                }
                if (chassis) {
                    String vx = value(values, columns, "vx_mps");
                    String vy = value(values, columns, "vy_mps");
                    String omega = value(values, columns, "omega_rad_s");
                    if (!vx.isBlank() || !vy.isBlank() || !omega.isBlank()) {
                        if (vx.isBlank() || vy.isBlank() || omega.isBlank())
                            throw new IllegalArgumentException("Chassis velocity columns must be all present or all blank");
                        row.vxMps = Double.parseDouble(vx);
                        row.vyMps = Double.parseDouble(vy);
                        row.omegaRadS = Double.parseDouble(omega);
                        if (!Double.isFinite(row.vxMps) || !Double.isFinite(row.vyMps) || !Double.isFinite(row.omegaRadS))
                            throw new IllegalArgumentException("Non-finite chassis velocity");
                    }
                }
                if (sensors) {
                    Double[] cells = new Double[10];
                    for (int i = 0; i < 10; i++) {
                        String raw = value(values, columns, TelemetryLogRow.SensorSample.COLUMNS.get(i));
                        if (raw.isBlank()) continue;
                        cells[i] = Double.parseDouble(raw);
                        if (!Double.isFinite(cells[i])) throw new IllegalArgumentException("Non-finite sensor value");
                    }
                    pairs(cells[0], cells[1], "imu yaw");
                    pairs(cells[2], cells[3], "imu rate");
                    if ((cells[4] == null) != (cells[7] == null) || (cells[5] == null) != (cells[8] == null)
                        || (cells[6] == null) != (cells[9] == null))
                        throw new IllegalArgumentException("Each odometry reading needs its reference on the same row");
                    boolean any = false;
                    for (Double cell : cells) any |= cell != null;
                    if (any) row.sensors = new TelemetryLogRow.SensorSample(cells[0], cells[1], cells[2], cells[3],
                        cells[4], cells[5], cells[6], cells[7], cells[8], cells[9]);
                }
            } catch (RuntimeException error) {
                throw new IllegalArgumentException("Invalid telemetry line " + (line + 1) + ": " + error.getMessage(), error);
            }
            rows.add(row);
        }
        if (rows.size() < 30) throw new IllegalArgumentException("Need at least 30 telemetry samples");
        return rows;
    }

    private static void pairs(Double reading, Double reference, String what) {
        if ((reading == null) != (reference == null))
            throw new IllegalArgumentException("Each " + what + " reading needs its reference on the same row");
    }

    private static void require(Map<String, Integer> columns, String name) {
        if (!columns.containsKey(name)) throw new IllegalArgumentException("Missing telemetry column " + name);
    }
    private static String value(String[] values, Map<String, Integer> columns, String name) {
        return values[columns.get(name)].trim();
    }
    private static double number(String[] values, Map<String, Integer> columns, String name) {
        double result = Double.parseDouble(value(values, columns, name));
        if (!Double.isFinite(result)) throw new IllegalArgumentException("Non-finite " + name);
        return result;
    }
}
