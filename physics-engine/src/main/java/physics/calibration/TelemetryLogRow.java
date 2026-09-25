package physics.calibration;

import java.util.LinkedHashMap;
import java.util.Map;

/** One row of R5's telemetry log schema. Per-motor fields are keyed by device name (matching R3's HardwareMap naming). */
public class TelemetryLogRow {
    public long tMs;
    public int loopIter;
    public double loopTimeMs;
    public double batteryVoltageV;
    public final Map<String, MotorSample> motors = new LinkedHashMap<>();

    public static class MotorSample {
        public double power;
        public long ticks;
        public double velTps;   // ticks/sec, per R5's addition (needed to fit friction without differentiating noisy position data)
        public double currentA; // per R5: "measured current is what makes R_battery identifiable"

        public MotorSample(double power, long ticks, double velTps, double currentA) {
            this.power = power;
            this.ticks = ticks;
            this.velTps = velTps;
            this.currentA = currentA;
        }
    }
}
