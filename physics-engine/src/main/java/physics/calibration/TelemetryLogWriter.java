package physics.calibration;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Locale;

/**
 * Writes R5's telemetry log CSV schema. Scope note: this implementation covers the
 * battery/motor columns Phase 3's calibration actually consumes (t_ms, loop_iter,
 * loop_time_ms, battery_voltage_v, per-motor power/ticks/vel_tps/current_a) -- R5's IMU
 * columns are not included, since IMU orientation isn't dynamically tracked anywhere in
 * this codebase yet (a pre-existing gap, not something this phase needed to fix; see
 * Phase 3's RESULTS.md).
 *
 * Per the Opus review's correction of R5's original design: this class only needs the
 * real, public HardwareMap API to run on an actual robot -- it has no dependency on this
 * simulator's internal simcore.* classes. In this repo it doubles as the tool used to
 * generate a synthetic ground-truth log for validating the calibration algorithm, since no
 * physical robot is available in this environment (see CalibratorTest).
 */
public class TelemetryLogWriter implements AutoCloseable {
    private final BufferedWriter writer;
    private final List<String> motorNames;
    private int loopIter = 0;

    public TelemetryLogWriter(Path path, List<String> motorNames) throws IOException {
        this.motorNames = motorNames;
        this.writer = Files.newBufferedWriter(path);
        StringBuilder header = new StringBuilder("t_ms,loop_iter,loop_time_ms,battery_voltage_v");
        for (String name : motorNames) {
            header.append(",motor_").append(name).append("_power")
                  .append(",motor_").append(name).append("_ticks")
                  .append(",motor_").append(name).append("_vel_tps")
                  .append(",motor_").append(name).append("_current_a");
        }
        writer.write(header.toString());
        writer.newLine();
    }

    public void logTick(long tMs, double loopTimeMs, double batteryVoltageV, List<TelemetryLogRow.MotorSample> samples) throws IOException {
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
        writer.write(row.toString());
        writer.newLine();
    }

    @Override public void close() throws IOException { writer.close(); }
}
