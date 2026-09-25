package physics.calibration;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/** Reads back TelemetryLogWriter's CSV format. Header is parsed dynamically -- not a fixed schema, per R5. */
public class TelemetryLogReader {

    private static final Pattern MOTOR_POWER_COL = Pattern.compile("^motor_(.+)_power$");

    public static List<TelemetryLogRow> read(Path path) throws IOException {
        List<String> lines = Files.readAllLines(path);
        String[] header = lines.get(0).split(",");

        List<String> motorNames = new ArrayList<>();
        for (String col : header) {
            Matcher m = MOTOR_POWER_COL.matcher(col);
            if (m.matches()) motorNames.add(m.group(1));
        }

        List<TelemetryLogRow> rows = new ArrayList<>();
        for (int i = 1; i < lines.size(); i++) {
            if (lines.get(i).isBlank()) continue;
            String[] cols = lines.get(i).split(",");
            TelemetryLogRow row = new TelemetryLogRow();
            row.tMs = Long.parseLong(cols[0]);
            row.loopIter = Integer.parseInt(cols[1]);
            row.loopTimeMs = Double.parseDouble(cols[2]);
            row.batteryVoltageV = Double.parseDouble(cols[3]);

            int idx = 4;
            for (String name : motorNames) {
                double power = Double.parseDouble(cols[idx++]);
                long ticks = Long.parseLong(cols[idx++]);
                double velTps = Double.parseDouble(cols[idx++]);
                double currentA = Double.parseDouble(cols[idx++]);
                row.motors.put(name, new TelemetryLogRow.MotorSample(power, ticks, velTps, currentA));
            }
            rows.add(row);
        }
        return rows;
    }
}
