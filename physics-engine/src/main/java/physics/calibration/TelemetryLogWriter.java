package physics.calibration;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Locale;

/** Writes motor/battery and optional independently measured chassis telemetry CSV. */
public class TelemetryLogWriter implements AutoCloseable {
    private final BufferedWriter writer;
    private final List<String> motorNames;
    private int loopIter = 0;
    private final boolean chassisColumns;
    private final boolean sensorColumns;

    public TelemetryLogWriter(Path path, List<String> motorNames) throws IOException {
        this(path, motorNames, false);
    }

    public TelemetryLogWriter(Path path, List<String> motorNames, boolean chassisColumns) throws IOException {
        this(path, motorNames, chassisColumns, false);
    }

    /** {@code sensorColumns} adds the optional IMU/odometry reading-versus-reference columns. */
    public TelemetryLogWriter(Path path, List<String> motorNames, boolean chassisColumns, boolean sensorColumns) throws IOException {
        this.motorNames = motorNames;
        this.chassisColumns = chassisColumns;
        this.sensorColumns = sensorColumns;
        this.writer = Files.newBufferedWriter(path);
        StringBuilder header = new StringBuilder("t_ms,loop_iter,loop_time_ms,battery_voltage_v");
        for (String name : motorNames) {
            header.append(",motor_").append(name).append("_power")
                  .append(",motor_").append(name).append("_ticks")
                  .append(",motor_").append(name).append("_vel_tps")
                  .append(",motor_").append(name).append("_current_a");
        }
        if (chassisColumns) header.append(",vx_mps,vy_mps,omega_rad_s");
        if (sensorColumns) for (String column : TelemetryLogRow.SensorSample.COLUMNS) header.append(',').append(column);
        writer.write(header.toString());
        writer.newLine();
    }

    public void logTick(long tMs, double loopTimeMs, double batteryVoltageV, List<TelemetryLogRow.MotorSample> samples) throws IOException {
        logTick(tMs, loopTimeMs, batteryVoltageV, samples, null, null, null);
    }

    public void logTick(long tMs, double loopTimeMs, double batteryVoltageV, List<TelemetryLogRow.MotorSample> samples,
                        Double vxMps, Double vyMps, Double omegaRadS) throws IOException {
        logTick(tMs, loopTimeMs, batteryVoltageV, samples, vxMps, vyMps, omegaRadS, null);
    }

    public void logTick(long tMs, double loopTimeMs, double batteryVoltageV, List<TelemetryLogRow.MotorSample> samples,
                        Double vxMps, Double vyMps, Double omegaRadS, TelemetryLogRow.SensorSample sensors) throws IOException {
        StringBuilder row = new StringBuilder();
        row.append(tMs).append(',').append(loopIter++).append(',')
           .append(String.format(Locale.ROOT, "%.3f", loopTimeMs)).append(',')
           .append(String.format(Locale.ROOT, "%.4f", batteryVoltageV));
        for (int i = 0; i < motorNames.size(); i++) {
            TelemetryLogRow.MotorSample s = samples.get(i);
            row.append(',').append(String.format(Locale.ROOT, "%.4f", s.power))
               .append(',').append(s.ticks)
               .append(',').append(String.format(Locale.ROOT, "%.4f", s.velTps))
               .append(',').append(String.format(Locale.ROOT, "%.4f", s.currentA));
        }
        if (chassisColumns) {
            if ((vxMps == null) != (vyMps == null) || (vxMps == null) != (omegaRadS == null))
                throw new IllegalArgumentException("Chassis velocity values must be all present or all absent");
            row.append(',').append(vxMps == null ? "" : vxMps)
               .append(',').append(vyMps == null ? "" : vyMps)
               .append(',').append(omegaRadS == null ? "" : omegaRadS);
        }
        if (sensorColumns) {
            Double[] values = sensors == null ? new Double[10] : sensors.values();
            for (Double value : values) row.append(',').append(value == null ? "" : value.toString());
        } else if (sensors != null) {
            throw new IllegalArgumentException("Writer was created without sensor columns");
        }
        writer.write(row.toString());
        writer.newLine();
    }

    @Override public void close() throws IOException { writer.close(); }
}
